/*
 * jasre_stress — standalone ASIC STRESS bench, no pool.
 *
 * GOAL. Drive the ASICs at FULL hashing load (all ALUs, full DAG) on SYNTHETIC
 * work, with no pool and no network, and measure the per-chip SILICON reject
 * rate (bad mix) UNDER LOAD. It is the only measurement that captures the
 * load-dependent defect: a bus scrub reads too gently and misses it (banks
 * pass the scan while mining rejects). Here we hash exactly like production,
 * we just count fast.
 *
 * PRINCIPLE. A LOW on-chip target makes the ALUs report thousands of nonces.
 * Every nonce is re-verified on the CPU (ethash_nonce_check with a MAX verify
 * target, so only the mix is judged, not the difficulty): a wrong mix = faulty
 * DAG read = silicon error. Over ~10 s that gives a stable per-chip rate.
 *
 * INIT. The real bring-up is reused: seed_update_operation() runs
 * prepare_state (enables the ALU and memory-controller masks; WITHOUT it the
 * cache build times out because the ALUs that build it are off), self_check,
 * dag_calc, pushes the first work and reopens the ALUs. Only the per-chip coin
 * contexts are pre-loaded beforehand with our synthetic epoch/header/target,
 * so the whole build and first work use them.
 *
 * FREQUENCY vs VOLTAGE.
 *   - Frequency canNOT change live: reprogramming the PLL needs a power-cycle +
 *     reset that WIPES the SeDRAM DAG. So it comes from the config (-c: "freq"
 *     or "asicN_freq") and stays FIXED for a whole run.
 *   - Voltage changes live: pure I2C (asic_pmbus_config), per chip, DAG intact.
 *     It is SWEPT inside the run (~10 s per point).
 * To test another frequency: edit the config and relaunch (fresh DAG build).
 *
 * USAGE
 *   jasre_stress -c <config.json> [--epoch N] [--volts mv,mv,...]
 *                   [--window S] [--diff D] [--tmax C]
 *
 *   --epoch  N   DAG epoch (default 828, an ETHW-size DAG of ~8 GB: fills the
 *                memory the most, hardest stress; must fit the healthy banks)
 *   --volts  L   list of voltages (mV) applied to all 3 chips at each point;
 *                without it, a single point at the config voltage
 *   --window S   COUNTING window per voltage point (default 12 s)
 *   --diff   D   on-chip difficulty = 2^D (default 24 -> ~1 accepted nonce per
 *                16 M hashes = ~37 nonces/s/chip at 600 MH/s, the ARM keeps up)
 *   --tmax   C   abort everything if a chip exceeds C degrees (default 95)
 *   --fan    N   fixed fan setting in percent (default: untouched)
 *   --power-off  cut the ASIC rails and exit (cools down without a relay)
 *   --reclaim-trial  production bring-up with JASRE_CACHE_RECLAIM=1 and
 *                JASRE_RECLAIM_TRIAL=N (a tail of N nodes written by the
 *                CPU), then a check of the tail; run it with --epoch 1
 *
 * RUN WITH THE MINER STOPPED (it drives the ASICs): jasminer.sh stop, then this
 * binary, then jasminer.sh start. Fans: the bench does not regulate them and
 * the setting does NOT fall back to 100 % (use --fan); the temperature is read
 * at each point and everything aborts above --tmax.
 */

#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include "cache_reclaim.h"
#include "jas_types.h"
#include "sl_transfer.h"
#include "dft.h"

/* Reused firmware functions (no exposing header for some of them). */
extern void            sl_wrapper_init(void);
extern void            seed_update_operation(void);       /* real bring-up + DAG build */
extern void            prepare_state(void);                /* masks ALU/mc + banks (before self_check) */
extern void            purge_results(int thread_id, uint8_t asic_mask);
extern ethash_light_t  ethash_light_new(uint64_t block_number);
extern void            ethash_get_seedhash(ethash_h256_t *out, uint64_t block_number);
extern void            jas_cache_save(uint64_t block, const ethash_h256_t *seed,
                                             ethash_light_t light);
extern void            jas_coin_from_block_seed(jas_coin_ctx *c, uint64_t block_number,
                                                  const ethash_h256_t *seed);
extern void            jas_config_set(const char *path);
extern void            jas_log(const char *fmt, ...);   /* -> /var/log/run.log (= /log.html) + stderr */
extern void          (*jas_veto_hook)(void);            /* hook seed_update_operation (before dag_calc) */

#define EPOCH_LEN        30000ull   /* blocks per ethash epoch */
#define DIFF_DEFAULT      24u        /* 2^24 : ~37 nonces/s/chip at 600 MH/s */
#define WINDOW_DEFAULT    12         /* s counting window per voltage point */
#define TMAX_DEFAULT      95         /* degrees: abort above this */

/* Registers/commands for the dedicated-reader-ALU mode (--verify-load).
 * Same as seed.c/dag_calc.c; redefined here because they are local #defines. */
#define REG_ALU_EN_LOC        0x0040   /* hash pipeline enable, per-ALU/mesh mask (2 words) */
#define REG_MC_EN_LOC         0x0c00   /* bank enable, 64 words/mesh */
#define REG_MC_REFRESH_EN_LOC 0x0c40   /* MC refresh engine, 64 words/mesh */
#define CMD_DIFFUSION_LOC     0xf8     /* broadcast to all meshes */
#define CMD_MESH_W_LOC(m)     ((uint8_t)((0x10u << (m)) | 0x8u))
#define MESHES_LOC            4
#define SETTLE_S         2          /* s to wait after a voltage change */
#define WARMUP_S         1          /* s of nonces discarded before counting */
#define ROLL_S           5          /* s: renew the header (avoids exhausting the
                                     * nonce range on a fixed header) */
#define RAIL1_DEF        1150       /* secondary rails: values from --pmbusfix */
#define RAIL2_DEF        1500
#define PMBUS_MODE       0x500      /* machine mode (adr_500 = 0x6a/0x60/0x64) */
#define SCRUB_SETTLE     1500       /* us before each scrub read: avoids the
                                     * stale burst read (the diag, ~1 ms gap
                                     * through jas_log, matched 100 %; 500 us
                                     * was not enough -> 1500) */

/* VERIFY target = MAX: ethash_nonce_check then judges only the mix (result < MAX
 * always true), not the difficulty. Symmetric -> immune to any byte reversal. */
static const uint8_t TARGET_MAX[32] = {
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
};

/* Referenced by wrapper.c (sl_wrapper_work); main.c is excluded from this
 * binary, so we define it here. Also serves as our local stop flag. */
volatile sig_atomic_t jas_stop_request;
static void on_signal(int s) { jas_stop_request = s; }
#define stop jas_stop_request

/* Hard watchdog. If the run ever wedges inside a firmware call stuck in a
 * syscall that the stop flag can't interrupt, fire after WATCHDOG_S and
 * hard-exit — the tool can then never hang the machine (the miner restart
 * re-inits the ASICs). Not a substitute for clean shutdown, a last resort. */
static void on_alarm(int s)
{
    (void)s;
    static const char m[] = "stress: WATCHDOG fired, hard exit\n";
    if (write(2, m, sizeof(m) - 1)) { /* best effort */ }
    _exit(4);
}

static long elapsed_ms(const struct timeval *t0)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec - t0->tv_sec) * 1000L + (t.tv_usec - t0->tv_usec) / 1000;
}

/* On-chip target = 2^(256-D): acceptance probability 2^-D (difficulty 2^D).
 * A single bit set, big-endian (b[0] = MSB). */
static void set_target(ethash_h256_t *t, unsigned diff_exp)
{
    memset(t->b, 0, 32);
    if (diff_exp == 0 || diff_exp >= 256) { memset(t->b, 0xff, 32); return; }
    unsigned bit = 256u - diff_exp;             /* bit to set, counted from LSB */
    unsigned idx = 31u - bit / 8u;              /* b[31] = LSB */
    t->b[idx] = (uint8_t)(1u << (bit % 8u));
}

/* A recognizable synthetic header, distinct on each call via the salt. */
static void set_header(ethash_h256_t *h, uint32_t sel)
{
    memset(h->b, 0, 32);
    h->b[0] = 0x5a; h->b[1] = 0x53; h->b[2] = 0x54; h->b[3] = 0x52;   /* "ZSTR" */
    memcpy(h->b + 28, &sel, sizeof(sel));
}

/* Apply a header to every live chip and push the work. */
static void broadcast_header(const ethash_h256_t *h)
{
    for (int b = 0; b < BOARD_NUM; b++) {
        if (!(BOARD_EN >> b & 1)) continue;
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(b) >> a & 1)
                jas_coin_of(b, a)->header = *h;
        header_update_operation(b, ASIC_EN(b));
    }
}

/* Drain at most `budget` nonces PER BOARD and count per chip (verify = mix only).
 * BOUNDED so the caller's wall-clock deadline is always re-checked, even when a
 * broken chip floods results or a hung bus read stalls. Returns how many were
 * drained. */
/* Per-ALU counting (alu_id 0..119 = mesh*30 + alu). Lets the report located the
 * reject: per chip, per mesh, and the worst individual ALUs -> tells whether the
 * reject is concentrated (vetoable) or spread (hard silicon). */
static unsigned drain_and_count(const ethash_h256_t *header,
                                   unsigned acc[3][3][120], unsigned rej[3][3][120],
                                   const struct timeval *t0, long deadline_ms)
{
    unsigned total = 0;
    for (int b = 0; b < BOARD_NUM && !stop; b++) {
        if (!(BOARD_EN >> b & 1)) continue;
        gooddata_detail d;
        /* Deadline checked BEFORE each read (a read can time out up to 4 s on a
         * marginal chip) -> the window overshoots by at most one read. */
        while (!stop && elapsed_ms(t0) < deadline_ms
               && get_asic_result_operation(b, &d)) {
            unsigned a = d.asic_id, alu = d.alu_id;
            if (a < 3 && alu < 120) {
                if (ethash_nonce_check(&d, header->b, TARGET_MAX) == 1) acc[b][a][alu]++;
                else                                                   rej[b][a][alu]++;
            }
            total++;
        }
    }
    return total;
}

/* Empty the buffers without counting (warm-up / after a voltage change). */
static void drain_silent(void)
{
    for (int b = 0; b < BOARD_NUM; b++) {
        if (!(BOARD_EN >> b & 1)) continue;
        gooddata_detail d;
        unsigned guard = 0;
        while (get_asic_result_operation(b, &d) && guard++ < 100000u) ;
        purge_results(b, ASIC_EN(b));
    }
}

/* Highest temperature read, -1 if unreadable. Uses jas_sensors_read (a
 * single bounded pass that fills jas_asic_temp) — NOT get_temp_func, which is
 * the sensor THREAD entry point and loops forever (for(;;)). Calling that was
 * the hang: temp_max_read never returned and the counting window never ran. */
static int temp_max_read(void)
{
    int tmax = -1;
    for (int b = 0; b < BOARD_NUM; b++) {
        if (!(BOARD_EN >> b & 1)) continue;
        jas_sensors_read((unsigned)b);
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if ((ASIC_EN(b) >> a & 1) && (int)jas_asic_temp[b][a] > tmax)
                tmax = (int)jas_asic_temp[b][a];
    }
    return tmax;
}

/* Apply a voltage (mV) to every live chip, per chip (I2C, live). */
static void set_voltage(unsigned mv)
{
    for (int b = 0; b < BOARD_NUM; b++) {
        if (!(BOARD_EN >> b & 1)) continue;
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(b) >> a & 1)
                asic_pmbus_config(PMBUS_MODE, b, a, (uint16_t)mv, RAIL1_DEF, RAIL2_DEF);
    }
}

/* DAG scrub PER BANK, UNDER LOAD, WITH A VOTE: find the banks that PASSED the
 * init test (DFT/CBSC) but do NOT HOLD under load. Each DAG entry is read
 * n_votes times through the bus (ALU0) while the other ALUs mine (real memory
 * pressure) and compared with the CPU value; only PERSISTENT errors (majority
 * of wrong votes) are kept, the vote removing the bus's transient noise
 * (~2 %). Result: error rate per bank, count of bad banks. A healthy chip must
 * come out at ~0. */
static unsigned g_bank_err[4096], g_bank_tot[4096];
static void do_scrub(int b, unsigned a, unsigned n_sample, unsigned n_votes)
{
    jas_coin_ctx *c = jas_coin_of(b, a);
    ethash_light_t cache = c ? jas_cache_find(c->block_number, &c->seed) : NULL;
    unsigned bank_num = jas_good_bank_num_cbsc[b * 3 + a] & 0xffffu;
    uint32_t max_addr, step, i;
    unsigned addr_failed = 0, addr_tot = 0, shown = 0, banks_failed = 0, bk, bins[24];

    if (!cache)    { jas_log("scrub b%d a%u: no CPU cache, skipped", b, a); return; }
    if (!bank_num) { jas_log("scrub b%d a%u: bank_num 0, skipped", b, a); return; }
    if (bank_num > 4096) bank_num = 4096;
    if (n_votes == 0) n_votes = 1;
    memset(bins, 0, sizeof(bins));
    memset(g_bank_err, 0, sizeof(g_bank_err));
    memset(g_bank_tot, 0, sizeof(g_bank_tot));
    max_addr = (uint32_t)(c->dag_size / 128u);   /* 128 B blocks = 2 nodes */
    step = max_addr / n_sample; if (!step) step = 1;

    {   /* LARGE JUMPS: consecutive addresses very far apart (like the diag
         * that matched 100 %), to avoid the row-buffer effect of the monotone
         * scan. stride ~ max_addr/3, odd -> covers the space with few
         * revisits. */
    uint32_t stride = (max_addr / 3u) | 1u;
    for (i = 0; i < n_sample && !stop; i++) {
        uint32_t A = (uint32_t)(((uint64_t)i * stride) % max_addr);
        node it1, it2;
        unsigned bad = 0, v, bank;
        (void)step;
        ethash_calculate_dag_item(&it1, 2u * A,      cache);
        ethash_calculate_dag_item(&it2, 2u * A + 1u, cache);
        for (v = 0; v < n_votes; v++) {
            uint32_t recv[32];
            memset(recv, 0, sizeof(recv));
            /* Interval BEFORE each read: without it, a burst read returns the
             * previous address's data (VALID not yet dropped) -> false
             * mismatches. The diag (jas_log between reads) matched 100 %. */
            usleep(SCRUB_SETTLE);
            if (single_manual_read((uint8_t)(1u << a), 0, A, (uint16_t *)recv, b) < 0)
                continue;
            if (memcmp(recv, it1.words, sizeof(it1.words)) != 0 ||
                memcmp(recv + 16, it2.words, sizeof(it2.words)) != 0)
                bad++;
        }
        bank = (A / 4u) % bank_num;
        g_bank_tot[bank] += n_votes;
        g_bank_err[bank] += bad;
        addr_tot++;
        if (bad > 0 && bad * 2u >= n_votes) {   /* persistent: majority of wrong votes */
            addr_failed++;
            bins[(i * 24u) / n_sample]++;
            if (shown < 10) {
                jas_log("scrub b%d a%u failed addr %u (bank~%u): %u/%u wrong votes",
                        b, a, A, bank, bad, n_votes);
                shown++;
            }
        }
    }
    }   /* end of the LARGE JUMPS block */
    for (bk = 0; bk < bank_num; bk++)
        if (g_bank_tot[bk] && g_bank_err[bk] * 2u >= g_bank_tot[bk]) banks_failed++;
    jas_log("scrub b%d a%u: %u/%u adr persistently failed (%.2f%%); %u banks failed / %u; votes=%u",
            b, a, addr_failed, addr_tot, addr_tot ? 100.0 * addr_failed / addr_tot : 0.0,
            banks_failed, bank_num, n_votes);
    {
        char line[220]; int off = 0; unsigned k;
        off += snprintf(line + off, (size_t)(sizeof(line) - off),
                        "scrub b%d a%u failed per region(24):", b, a);
        for (k = 0; k < 24; k++)
            off += snprintf(line + off, (size_t)(sizeof(line) - off), "%s%u",
                            k ? "," : " ", bins[k]);
        jas_log("%s", line);
    }
}

/* Targeted diagnosis: why does the scrub come out wrong on a healthy chip?
 * Reads addresses ALIGNED like dram_check_dag (index=2*address PROVEN there)
 * and ARBITRARY addresses, each 3 times, and prints node1/node2 ok/BAD + the
 * first words read vs expected + the stability. Settles: structural mapping
 * (aligned ok / arbitrary wrong) vs unstable read (varies) vs bug (aligned
 * wrong). */
static void do_scrub_diag(int b, unsigned a)
{
    jas_coin_ctx *c = jas_coin_of(b, a);
    ethash_light_t cache = c ? jas_cache_find(c->block_number, &c->seed) : NULL;
    unsigned bank_num = jas_good_bank_num_cbsc[b * 3 + a] & 0xffffu;
    uint32_t step_addr, n, tests[12];
    unsigned nt = 0, t, rep;
    uint64_t block;

    if (!cache || !bank_num) { jas_log("diag b%d a%u: no cache/bank", b, a); return; }
    step_addr = bank_num * 4u;
    block = c->dag_size / ((uint64_t)bank_num << 9);   /* dag_size/(bank_num*512) */
    n = (uint32_t)(block / 6u);

    tests[nt++] = 1u * n * step_addr;          /* aligned like dram_check (probe 1..4) */
    tests[nt++] = 2u * n * step_addr;
    tests[nt++] = 3u * n * step_addr;
    tests[nt++] = 4u * n * step_addr;
    tests[nt++] = 1u * n * step_addr + 64u;    /* aligned + small offset */
    tests[nt++] = 1u * n * step_addr + 4096u;
    tests[nt++] = 1u;                          /* arbitrary */
    tests[nt++] = 100u;
    tests[nt++] = 4096u;
    tests[nt++] = 1000000u;
    tests[nt++] = 10000000u;
    tests[nt++] = 30000000u;

    jas_log("diag b%d a%u: n=%u addr_step=%u (bank_num %u)", b, a, n, step_addr, bank_num);
    for (t = 0; t < nt; t++) {
        uint32_t A = tests[t];
        node it1, it2;
        ethash_calculate_dag_item(&it1, 2u * A,      cache);
        ethash_calculate_dag_item(&it2, 2u * A + 1u, cache);
        for (rep = 0; rep < 3; rep++) {
            uint32_t recv[32];
            int m1, m2;
            memset(recv, 0, sizeof(recv));
            if (single_manual_read((uint8_t)(1u << a), 0, A, (uint16_t *)recv, b) < 0) {
                jas_log("diag a%u A=%u rep%u: READ FAIL", a, A, rep);
                continue;
            }
            m1 = memcmp(recv, it1.words, sizeof(it1.words)) != 0;
            m2 = memcmp(recv + 16, it2.words, sizeof(it2.words)) != 0;
            jas_log("diag a%u A=%-9u rep%u: n1 %s n2 %s | read %08x %08x | exp %08x %08x | read2 %08x exp2 %08x",
                    a, A, rep, m1 ? "BAD" : "ok ", m2 ? "BAD" : "ok ",
                    recv[0], recv[1], it1.words[0], it1.words[1], recv[16], it2.words[0]);
        }
    }
}

/* VERIFY-DAG: is the DAG WRITE correct, AT REST?
 *
 * n_reads reads spread over RANDOM addresses of the SAFE ZONE, the first 4/6
 * of the DAG, where the index = 2*address mapping is confirmed; 4/6 is an
 * anti-overflow margin (the remap can move an access up). With the striping
 * bank = (A/4) % bank_num, the zone touches ALL banks: it is a per-bank map.
 * SCRUB_SETTLE before each read is mandatory: without it a burst read returns
 * the previous address's data (VALID not yet dropped) and fakes errors.
 *
 * At rest (ALUs stopped, refresh running), a PERSISTENT error is a STATIC
 * defect (write or hard cell). The reject while mining = this static part +
 * the load-induced part: verify ~0 while mining rejects means the write is
 * cleared and the LOAD is the cause.
 *
 * VERDICT: a healthy chip gives the instrument's noise FLOOR; the others are
 * read RELATIVE to it. The 95 % upper bound (3/n_addr if 0 bad) puts a figure
 * on the exoneration. */
static void do_verify_read(int b, unsigned a, unsigned reader_sel,
                                 unsigned n_reads, unsigned n_votes, const char *phase)
{
    jas_coin_ctx *c = jas_coin_of(b, a);
    ethash_light_t cache = c ? jas_cache_find(c->block_number, &c->seed) : NULL;
    unsigned bank_num = jas_good_bank_num_cbsc[b * 3 + a] & 0xffffu;
    uint32_t max_safe, i, n_addr;
    unsigned addr_failed = 0, addr_tot = 0, shown = 0, banks_failed = 0, bk, bins[24];
    const uint32_t HASH = 2654435761u;  /* 32-bit golden ratio: uniform spread */

    if (!cache)    { jas_log("verify b%d a%u: no CPU cache, skipped", b, a); return; }
    if (!bank_num) { jas_log("verify b%d a%u: bank_num 0, skipped", b, a); return; }
    if (bank_num > 4096) bank_num = 4096;
    if (n_votes == 0) n_votes = 1;
    n_addr = n_reads / n_votes; if (!n_addr) n_addr = 1;
    memset(bins, 0, sizeof(bins));
    memset(g_bank_err, 0, sizeof(g_bank_err));
    memset(g_bank_tot, 0, sizeof(g_bank_tot));

    /* SAFE ZONE = 4/6 of the DAG. address = byte/128 (unit 128 B = 2 nodes). */
    max_safe = (uint32_t)((c->dag_size / 128ull) * 4ull / 6ull);
    if (!max_safe) { jas_log("verify b%d a%u: DAG too small, skipped", b, a); return; }

    for (i = 0; i < n_addr && !stop; i++) {
        uint32_t A = (uint32_t)(((uint64_t)i * HASH) % max_safe);
        node it1, it2;
        unsigned bad = 0, v, bank;
        uint32_t seen[4] = { 0, 0, 0, 0 };   /* first words read (last vote) */

        ethash_calculate_dag_item(&it1, 2u * A,      cache);
        ethash_calculate_dag_item(&it2, 2u * A + 1u, cache);
        for (v = 0; v < n_votes; v++) {
            uint32_t recv[32];
            memset(recv, 0, sizeof(recv));
            usleep(SCRUB_SETTLE);
            if (single_manual_read((uint8_t)(1u << a), (uint16_t)reader_sel, A,
                                   (uint16_t *)recv, b) < 0)
                continue;
            if (memcmp(recv, it1.words, sizeof(it1.words)) != 0 ||
                memcmp(recv + 16, it2.words, sizeof(it2.words)) != 0)
                bad++;
            seen[0] = recv[0]; seen[1] = recv[1]; seen[2] = recv[16]; seen[3] = recv[17];
        }
        bank = (A / 4u) % bank_num;
        g_bank_tot[bank] += n_votes;
        g_bank_err[bank] += bad;
        addr_tot++;
        if (bad > 0 && bad * 2u >= n_votes) {   /* persistent: majority wrong */
            addr_failed++;
            bins[(uint64_t)i * 24u / n_addr]++;
            if (shown < 10) {
                jas_log("verify[%s] b%d a%u failed addr %u (bank~%u) %u/%u wrong |"
                        " read %08x %08x n2 %08x | exp %08x %08x n2 %08x",
                        phase, b, a, A, bank, bad, n_votes, seen[0], seen[1], seen[2],
                        it1.words[0], it1.words[1], it2.words[0]);
                shown++;
            }
        }
    }
    for (bk = 0; bk < bank_num; bk++)
        if (g_bank_tot[bk] && g_bank_err[bk] * 2u >= g_bank_tot[bk]) banks_failed++;
    {
        double rate  = addr_tot ? 100.0 * addr_failed / addr_tot : 0.0;
        double bound = addr_failed ? rate : (addr_tot ? 300.0 / addr_tot : 0.0);
        jas_log("verify[%s] b%d a%u: %u/%u adr persistently failed (%.4f%%); 95%%"
                " upper bound %.4f%%; %u banks failed / %u; votes=%u; safe zone 4/6",
                phase, b, a, addr_failed, addr_tot, rate, bound, banks_failed, bank_num, n_votes);
    }
    {
        char line[240]; int off = 0; unsigned k;
        off += snprintf(line + off, (size_t)(sizeof(line) - off),
                        "verify[%s] b%d a%u failed per region(24 over the 4/6):", phase, b, a);
        for (k = 0; k < 24; k++)
            off += snprintf(line + off, (size_t)(sizeof(line) - off), "%s%u",
                            k ? "," : " ", bins[k]);
        jas_log("%s", line);
    }
}

/* ====================================================================
 * (a) DEDICATED READER ALU: read the DAG through the MINING PATH without
 * the rot.
 *
 * Two measurement states, same read loop (do_verify_read):
 *  - IDLE : all ALUs stopped BUT MC refresh re-enabled (0xc40/0xc00):
 *           fixes the self-inflicted rot; baseline of the written CONTENT.
 *  - LOAD : refresh cut again + all ALUs back on (they mine = live
 *           refresh), then ONE SINGLE ALU is idled and read through: same
 *           path as mining, under the real contention/thermal conditions.
 *  Difference LOAD - IDLE = load-induced errors = the reject, located per
 *  bank. asic1 (healthy) = safeguard.
 * ==================================================================== */

/* IDLE: stop all the board's ALUs + RE-ENABLE refresh (0xc40) and bank
 * enable (0xc00); otherwise the DAG rots (close_refresh_and_reopen_alu cut
 * 0xc40 at the end of the bring-up). Masks = the scan's ones (jas_mc_*). */
static void state_idle_refresh_on(unsigned board)
{
    uint8_t asic_en = ASIC_EN(board);
    uint16_t zero2[2] = { 0, 0 };
    unsigned p, m;

    sl_multi_write(asic_en, CMD_DIFFUSION_LOC, REG_ALU_EN_LOC, zero2, 2, (int)board);
    for (p = 0; p < ASIC_NUM; p++) {
        if (!(asic_en >> p & 1)) continue;
        for (m = 0; m < MESHES_LOC; m++) {
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_MC_EN_LOC,
                           jas_mc_en[board][p][m], 0x40, (int)board);
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_MC_REFRESH_EN_LOC,
                           jas_mc_refresh_en[board][p][m], 0x40, (int)board);
        }
    }
    jas_log("stress: board %u IDLE refresh re-enabled (0xc40/0xc00)", board);
}

/* LOAD: cut the refresh again (like mining) then turn all the ALUs back on
 * -> they mine, their reads refresh the DAG (no rot). */
static void state_mining_on(unsigned board)
{
    uint8_t asic_en = ASIC_EN(board);
    uint16_t zeros[64];
    unsigned p, m;

    memset(zeros, 0, sizeof(zeros));
    for (p = 0; p < ASIC_NUM; p++) {
        if (!(asic_en >> p & 1)) continue;
        for (m = 0; m < MESHES_LOC; m++)
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_MC_REFRESH_EN_LOC,
                           zeros, 0x40, (int)board);
        for (m = 0; m < MESHES_LOC; m++)
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_ALU_EN_LOC,
                           jas_alu_en[board][p][m], 2, (int)board);
    }
    jas_log("stress: board %u LOAD mining back on (refresh off)", board);
}

/* Find a chip's first ENABLED ALU (for a reliable dedicated reader). -1 if none. */
static int first_good_alu(unsigned board, unsigned chip)
{
    unsigned m, bit;
    for (m = 0; m < MESHES_LOC; m++)
        for (bit = 0; bit < 30u; bit++)
            if (jas_alu_en[board][chip][m][bit >> 4] & (1u << (bit & 0xf)))
                return (int)(m * 30u + bit);
    return -1;
}

/* Idle ONE ALU (sel in 0..119) of a chip, leaving the others mining. */
static void idle_one_alu(unsigned board, unsigned chip, unsigned sel)
{
    unsigned mesh = sel / 30u, alu = sel % 30u;
    uint16_t mask[2];

    mask[0] = jas_alu_en[board][chip][mesh][0];
    mask[1] = jas_alu_en[board][chip][mesh][1];
    mask[alu >> 4] &= (uint16_t)~(1u << (alu & 0xf));
    sl_multi_write((uint8_t)(1u << chip), CMD_MESH_W_LOC(mesh), REG_ALU_EN_LOC,
                   mask, 2, (int)board);
}

/* Restore the idled ALU (puts the mesh's mask back to its mining value). */
static void restore_one_alu(unsigned board, unsigned chip, unsigned sel)
{
    unsigned mesh = sel / 30u;
    sl_multi_write((uint8_t)(1u << chip), CMD_MESH_W_LOC(mesh), REG_ALU_EN_LOC,
                   jas_alu_en[board][chip][mesh], 2, (int)board);
}

/* REFRESH ON WHILE MINING (--refresh-on): re-enable the explicit refresh
 * (0xc40) + bank enable (0xc00) WITHOUT touching the ALUs -> they keep
 * mining. Tests whether the reject comes from RETENTION through the access
 * queue (Sunlune patent CN114388025B: the chip refreshes itself through
 * fast access, and close_refresh_and_reopen_alu cuts 0xc40 at the end of the
 * bring-up). Cost = a little bandwidth; if the reject drops -> trivial fix
 * (keep the refresh). */
static void refresh_on_while_mining(unsigned board)
{
    uint8_t asic_en = ASIC_EN(board);
    unsigned p, m;
    for (p = 0; p < ASIC_NUM; p++) {
        if (!(asic_en >> p & 1)) continue;
        for (m = 0; m < MESHES_LOC; m++) {
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_MC_EN_LOC,
                           jas_mc_en[board][p][m], 0x40, (int)board);
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W_LOC(m), REG_MC_REFRESH_EN_LOC,
                           jas_mc_refresh_en[board][p][m], 0x40, (int)board);
        }
    }
    jas_log("stress: board %u REFRESH-ON while mining (0xc40/0xc00 re-enabled, ALUs mining)", board);
}

/* Override req_interval (--req-interval N): reg 0x2da = spacing between MC
 * requests in cycles. 0 = back to back (default). Slows the DAG fetch
 * without touching frequency or settle. To be tried WITH --refresh-on (the
 * retention safety net). */
static void set_req_interval(unsigned board, unsigned cycles)
{
    sl_single_write(ASIC_EN(board), CMD_DIFFUSION_LOC, 0x02da, (uint16_t)cycles, (int)board);
    jas_log("stress: board %u req_interval (0x2da) = %u cycles", board, cycles);
}

/* CBSC tool: how many banks are needed, and at what SEVERITY (minimum ALU
 * coverage) the target is reached by dropping the weakest.
 *   target = ceil((dag_size + 2*cache_size) / (0xff6 * 512))
 * (0xff6 rows max, valid_banks*512 bytes/row; 2x cache margin like
 * prepare_chip). A bank's "quality" = jas_map_bank_alu (number of ALUs
 * that reach it, 0..120); raising the threshold = keeping the best-covered
 * banks = removing the weakest. */
static void do_cbsc_target(int b, unsigned a)
{
    const jas_coin_ctx *c = jas_coin_of(b, a);
    unsigned base = a * 4080u, bank, hist[121];
    unsigned good_total = 0, cur, target, keep, chosen_S = 0, chosen_keep = 0;
    int S;
    uint64_t rowbytes = (uint64_t)0xff6u * 512u;   /* 2,092,032 B per bank (addressing) */

    memset(hist, 0, sizeof(hist));
    for (bank = 0; bank < 4080u; bank++) {
        if (jas_bank_good[b][base + bank]) {
            unsigned cov = jas_map_bank_alu[b][base + bank];
            if (cov > 120u) cov = 120u;
            hist[cov]++;
            good_total++;
        }
    }
    cur = jas_good_bank_num_cbsc[b * 3 + a];
    target = (unsigned)((c->dag_size + rowbytes - 1u) / rowbytes);  /* DAG floor (cache in the headroom) */

    jas_log("cbsc b%d a%u: DAG %llu MiB -> NEEDS %u banks; good_cbsc=%u map=%u; margin=%d",
            b, a, (unsigned long long)(c->dag_size >> 20), target, cur, good_total,
            (int)good_total - (int)target);

    /* curve: cov>=S -> banks kept; 1st S (high->low) where keep>=target */
    keep = 0;
    for (S = 120; S >= 0; S--) {
        keep += hist[S];
        if (keep >= target && chosen_S == 0 && S > 0) { chosen_S = (unsigned)S; chosen_keep = keep; }
        if (S % 10 == 0 || (chosen_S && (unsigned)S == chosen_S))
            jas_log("cbsc b%d a%u: cov>=%3d -> %u banks", b, a, S, keep);
    }
    jas_log("cbsc b%d a%u: THRESHOLD coverage>=%u keeps %u (target %u) -> removes %u weak banks",
            b, a, chosen_S, chosen_keep, target,
            good_total > chosen_keep ? good_total - chosen_keep : 0);
}

/* VETO: reduces chip g_veto_asic to its DAG target by removing the banks
 * with the lowest coverage (coverage ascending). Set as jas_veto_hook, it
 * is called by seed_update_operation between the CBSC scan and dag_calc
 * -> the DAG is built on the reduced set, and the reject is measured
 * afterwards. */
static int g_veto_asic = -1;   /* -1 = no veto; otherwise the chip (board 0) */
/*
 * --cov-floor S: remove ALL the banks reached by fewer than S ALUs, instead
 * of going down to the target: it tests the coverage hypothesis directly
 * (e.g. a binary histogram, 3761 banks at 120 ALUs and 163 at exactly 72:
 * --cov-floor 120 removes the 163 at once). Use a DAG small enough for the
 * capacity loss not to cause a cache/DAG collision, or both effects are
 * measured at once; the safeguard below refuses it.
 */
static unsigned g_cov_floor = 0;
static void veto_hook_fn(void)
{
    int b = 0;
    unsigned a, base, bank, valid = 0, dropped = 0, to_drop, S;
    const jas_coin_ctx *c;
    uint64_t rowbytes = (uint64_t)0xff6u * 512u;
    unsigned target;

    if (g_veto_asic < 0) return;
    a = (unsigned)g_veto_asic;
    base = a * 4080u;
    c = jas_coin_of(b, a);
    /*
     * THE TARGET COUNTS THE CACHE, NOT ONLY THE DAG: the ethash cache stays
     * resident at the top of the memory during the whole build (3894 banks at
     * epoch 828, not the 3834 the DAG alone gives); forgetting it lets a chip
     * write its DAG over its own cache with nothing reporting it.
     */
    target = (unsigned)((c->dag_size + c->cache_size + rowbytes - 1u) / rowbytes);

    for (bank = 0; bank < 4080u; bank++)
        if (jas_bank_partial[b][base + bank]) valid++;
    if (valid <= target) {
        jas_log("VETO a%u: %u valid banks <= target %u -> nothing to remove", a, valid, target);
        return;
    }

    if (g_cov_floor) {
        /* How many banks would fall under the coverage threshold? */
        unsigned under_threshold = 0;

        for (bank = 0; bank < 4080u; bank++) {
            unsigned idx = base + bank;
            if (jas_bank_partial[b][idx] && jas_map_bank_alu[b][idx] < g_cov_floor)
                under_threshold++;
        }
        if (valid - under_threshold < target) {
            /* Removing them all would go under the target: the cache/DAG
             * collision would be measured along with the coverage. Refused. */
            jas_log("VETO a%u: cov<%u = %u banks, but %u - %u = %u < target %u"
                    " -> REFUSED (use a smaller epoch)",
                    a, g_cov_floor, under_threshold, valid, under_threshold,
                    valid - under_threshold, target);
            return;
        }
        for (bank = 0; bank < 4080u; bank++) {
            unsigned idx = base + bank;
            if (jas_bank_partial[b][idx] && jas_map_bank_alu[b][idx] < g_cov_floor) {
                jas_bank_partial[b][idx] = 0;
                jas_bank_good[b][idx] = 0;
                dropped++;
            }
        }
        jas_good_bank_num_cbsc[b * 3 + a] = (uint16_t)(valid - dropped);
        jas_log("VETO a%u: coverage < %u -> %u banks removed, %u -> %u (target"
                " %u, margin %d)",
                a, g_cov_floor, dropped, valid, valid - dropped, target,
                (int)(valid - dropped) - (int)target);
        return;
    }

    to_drop = valid - target;
    for (S = 0; S <= 120u && dropped < to_drop; S++)
        for (bank = 0; bank < 4080u && dropped < to_drop; bank++) {
            unsigned idx = base + bank;
            if (jas_bank_partial[b][idx] && jas_map_bank_alu[b][idx] == S) {
                jas_bank_partial[b][idx] = 0;
                jas_bank_good[b][idx] = 0;
                dropped++;
            }
        }
    jas_good_bank_num_cbsc[b * 3 + a] = (uint16_t)(valid - dropped);
    jas_log("VETO a%u: %u -> %u banks (removed the %u with the weakest coverage; target %u)",
            a, valid, valid - dropped, dropped, target);
}

/*
 * --reclaim-probe: prove the write/read ROUND TRIP of a node, on which the
 * whole cache reclaim rests (the processor writes the DAG tail).
 * Non-destructive: writes go ONLY beyond the end of the DAG (the former
 * cache area, never read by mining). The ALUs must be stopped by the
 * caller.
 *
 *   phase 1  read only: read DAG node 0 back (built by the chip) and
 *            compare it with the processor's computation -> confirms the
 *            READ addressing.
 *   phase 2  pattern: write a recognizable pattern to a free node, read it
 *            back, compare -> confirms the write AND the round trip at a
 *            node address.
 *   phase 3  ethash: compute ethash(i), write it, read it back, compare ->
 *            the exact reclaim primitive (CPU computation + write + check).
 */
/* P1: read only, BEFORE any reset: the real DAG built by the chip is
 * still in place. Same path as --scrub / --verify-dag (ALUs stopped). */
static void do_reclaim_probe_read(int b, unsigned a)
{
    jas_coin_ctx *c = jas_coin_of(b, a);
    ethash_light_t cache = c ? jas_cache_find(c->block_number, &c->seed) : NULL;
    uint8_t en = (uint8_t)(1u << a);
    node exp, loaded;
    uint16_t block[0x40];

    if (!c || !cache) { jas_log("reclaim-probe b%d a%u: no CPU cache", b, a); return; }
    ethash_calculate_dag_item(&exp, 0, cache);
    if (single_manual_read(en, 0, 0, block, b) < 0)
        jas_log("reclaim-probe b%d a%u: P1 read of node0 SILENT", b, a);
    else {
        memcpy(loaded.bytes, block, 64);
        jas_log("reclaim-probe b%d a%u: P1 node0 %s | read %08x %08x | exp %08x %08x",
                b, a, memcmp(loaded.bytes, exp.bytes, 64) == 0 ? "OK" : "DIFFERENT",
                loaded.words[0], loaded.words[1], exp.words[0], exp.words[1]);
    }
}

/*
 * PROVEN PREAMBLE before any manual write, the same as crossbar_check (the
 * CBSC fill) and main.c's write modes: soft reset 0x830, reassignment, DRAM
 * timings (the reset erases them), then memory controllers AND refresh
 * turned back on (0xc00/0xc40).
 *
 * MANDATORY: writing after merely stopping the ALUs, with the refresh closed
 * by mining, sends the write to a sleeping controller; the AXI transaction is
 * never acknowledged and the ARM CORE stalls (no network, no SSH, mains power
 * cut required).
 */
static void preamble_write(int b)
{
    uint16_t full[0x40];
    unsigned a;

    sl_single_write(ASIC_EN(b), 0xf8, 0x830, 0, b);
    sl_delay(0, 2000000);
    sl_single_write(ASIC_EN(b), 0xf8, 0x830, 0xffff, b);
    for (a = 0; a < ASIC_NUM; a++)
        if (ASIC_EN(b) >> a & 1)
            config_remap_info(b, (uint8_t)a, 1);
    config_asic_fix_param(b);
    memset(full, 0xff, sizeof(full));
    sl_multi_write(ASIC_EN(b), 0xf8, 0xc00, full, 0x40, b);
    sl_multi_write(ASIC_EN(b), 0xf8, 0xc40, full, 0x40, b);
    jas_log("stress: board %d write preamble done (0x830, remap, timings, 0xc00/0xc40 ON)", b);
}

/* P2 + P3: writes beyond the DAG, AFTER the preamble. */
static void do_reclaim_probe_write(int b, unsigned a)
{
    jas_coin_ctx *c = jas_coin_of(b, a);
    ethash_light_t cache = c ? jas_cache_find(c->block_number, &c->seed) : NULL;
    uint32_t total = c ? (uint32_t)(c->dag_size / 64u) : 0;
    unsigned valid = jas_good_bank_num_cbsc[b * 3 + a] & 0xffffu;
    uint32_t usable = (uint32_t)(0xff6u * (valid ? valid : 1u) * 8u);
    uint8_t  en = (uint8_t)(1u << a);
    uint32_t A;
    node exp, word, loaded;
    uint16_t block[0x40];
    const uint16_t *half;
    unsigned i;

    if (!c || !cache) { jas_log("reclaim-probe b%d a%u: no CPU cache", b, a); return; }
    if (!valid)     { jas_log("reclaim-probe b%d a%u: bank_num 0", b, a); return; }

    jas_log("reclaim-probe b%d a%u: dag %u nodes, cache at %u, useful %u, margin %u nodes",
            b, a, total, valid ? (unsigned)(0xff6u - c->cache_size / ((uint64_t)valid << 9)) * valid * 8u : 0,
            usable, usable > total ? usable - total : 0);

    /* Trial node address, beyond the DAG, in the former cache (never mined). */
    A = total + 2048u;
    if (A + 2u >= usable) { jas_log("reclaim-probe b%d a%u: no safe margin, P2/P3 skipped", b, a); return; }

    /* --- phase 2: recognizable pattern --- */
    for (i = 0; i < 16; i++) word.words[i] = 0xBA5E0000u + (A ^ (i * 0x1000193u));
    single_manual_write(en, A, (uint16_t *)word.words, b);
    if (single_manual_read(en, 0, A >> 1, block, b) < 0)
        jas_log("reclaim-probe b%d a%u: P2 read-back SILENT at %u", b, a, A);
    else {
        half = block + ((A & 1u) ? 0x20u : 0u);
        memcpy(loaded.bytes, half, 64);
        jas_log("reclaim-probe b%d a%u: P2 pattern %s at node %u | read %08x %08x | wrote %08x %08x",
                b, a, memcmp(loaded.bytes, word.bytes, 64) == 0 ? "OK (round trip proven)" : "DIFFERENT",
                A, loaded.words[0], loaded.words[1], word.words[0], word.words[1]);
    }

    /* --- phase 3: the exact reclaim primitive (ethash + write + check) --- */
    ethash_calculate_dag_item(&exp, A, cache);
    single_manual_write(en, A, (uint16_t *)exp.words, b);
    if (single_manual_read(en, 0, A >> 1, block, b) < 0)
        jas_log("reclaim-probe b%d a%u: P3 read-back SILENT", b, a);
    else {
        half = block + ((A & 1u) ? 0x20u : 0u);
        memcpy(loaded.bytes, half, 64);
        jas_log("reclaim-probe b%d a%u: P3 ethash %s at node %u | read %08x %08x | exp %08x %08x",
                b, a, memcmp(loaded.bytes, exp.bytes, 64) == 0 ? "OK (reclaim validated)" : "DIFFERENT",
                A, loaded.words[0], loaded.words[1], exp.words[0], exp.words[1]);
    }
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    /* --- parameters --- */
    const char *config = NULL;
    uint64_t epoch = 828;
    unsigned diff = DIFF_DEFAULT;
    int window = WINDOW_DEFAULT, tmax = TMAX_DEFAULT;
    /*
     * --fan N: FIXED FAN SETTING, in percent. 0 = do not touch.
     *
     * The bench never calls set_PWM_according_to_temperature_x16, and the
     * setting does NOT fall back to 100 %: it stays where it was (e.g. 42 %
     * with chips at 66 degrees, where the firmware's curve demands 100 %). The
     * bench then runs hotter than production, and the temperature drops when
     * the voltage is lowered, which biases a voltage sweep. A FIXED setting
     * leaves a single variable in the experiment; ~63 % matches the machines
     * in production.
     */
    int fan_fixed = 0;
    unsigned volts[32]; int n_volts = 0;
    int probe_rails = 0, scrub = 0, scrub_diag = 0, cbsc_target = 0, verify_dag = 0, verify_load = 0;
    int reclaim_probe = 0;           /* --reclaim-probe: write/read round trip of a node */
    int reclaim_trial = 0;           /* --reclaim-trial: production path + forced tail (JASRE_RECLAIM_TRIAL) */
    int power_off_only = 0;          /* --power-off: cut the ASIC rails and exit (cooling) */
    int refresh_on = 0;              /* --refresh-on: keep 0xc40 on while mining */
    int scan_regs = 0;               /* --scan-regs: register discovery, read only */
    unsigned scan_lo = 0x0000, scan_hi = 0x0fff, scan_asic = 0, scan_us = 2000;
    int scan_cfg = 0;                /* --scan-cfg: configure before reading */
    unsigned scan_cmd = 0;           /* --scan-cmd: restrict to one command */
    long req_interval = -1;          /* --req-interval N: override reg 0x2da (-1 = do not touch) */
    unsigned scrub_n = 15000, scrub_votes = 3, verify_n = 77000;

    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "--probe-rails"))                 probe_rails = 1;
        else if (!strcmp(argv[k], "--cbsc-target"))            cbsc_target = 1;
        else if (!strcmp(argv[k], "--cov-floor") && k + 1 < argc) {
            g_cov_floor = (unsigned)strtoul(argv[++k], NULL, 10);
            if (g_veto_asic < 0) g_veto_asic = 2;   /* asic2 by default */
        }
        else if (!strcmp(argv[k], "--cbsc-veto")) {
            g_veto_asic = 2;   /* default: the marginal chip to test */
            if (k + 1 < argc && argv[k + 1][0] >= '0' && argv[k + 1][0] <= '9')
                g_veto_asic = atoi(argv[++k]);
        }
        else if (!strcmp(argv[k], "--verify-dag")) {
            verify_dag = 1;
            if (k + 1 < argc && argv[k + 1][0] >= '0' && argv[k + 1][0] <= '9')
                verify_n = (unsigned)strtoul(argv[++k], NULL, 10);
        }
        else if (!strcmp(argv[k], "--verify-load")) {
            verify_load = 1;   /* IDLE (refresh on) + LOAD (dedicated reader ALU) */
            if (k + 1 < argc && argv[k + 1][0] >= '0' && argv[k + 1][0] <= '9')
                verify_n = (unsigned)strtoul(argv[++k], NULL, 10);
        }
        else if (!strcmp(argv[k], "--refresh-on"))             refresh_on = 1;
        else if (!strcmp(argv[k], "--scan-regs")) {
            scan_regs = 1;
            if (k + 1 < argc && argv[k + 1][0] != '-') scan_lo = (unsigned)strtoul(argv[++k], NULL, 0);
            if (k + 1 < argc && argv[k + 1][0] != '-') scan_hi = (unsigned)strtoul(argv[++k], NULL, 0);
        }
        else if (!strcmp(argv[k], "--scan-asic") && k + 1 < argc) scan_asic = (unsigned)atoi(argv[++k]);
        else if (!strcmp(argv[k], "--scan-us") && k + 1 < argc)   scan_us = (unsigned)strtoul(argv[++k], NULL, 0);
        else if (!strcmp(argv[k], "--scan-cfg"))                  scan_cfg = 1;
        else if (!strcmp(argv[k], "--scan-cmd") && k + 1 < argc)  scan_cmd = (unsigned)strtoul(argv[++k], NULL, 0);
        else if (!strcmp(argv[k], "--req-interval") && k + 1 < argc) req_interval = strtol(argv[++k], NULL, 0);
        else if (!strcmp(argv[k], "--reclaim-probe"))          reclaim_probe = 1;
        else if (!strcmp(argv[k], "--reclaim-trial"))          reclaim_trial = 1;
        else if (!strcmp(argv[k], "--power-off"))              power_off_only = 1;
        else if (!strcmp(argv[k], "--scrub-diag"))             scrub_diag = 1;
        else if (!strcmp(argv[k], "--votes") && k + 1 < argc)  scrub_votes = (unsigned)strtoul(argv[++k], NULL, 10);
        else if (!strcmp(argv[k], "--scrub")) {
            scrub = 1;
            if (k + 1 < argc && argv[k + 1][0] >= '0' && argv[k + 1][0] <= '9')
                scrub_n = (unsigned)strtoul(argv[++k], NULL, 10);
        }
        else if (!strcmp(argv[k], "-c") && k + 1 < argc)       config = argv[++k];
        else if (!strcmp(argv[k], "--epoch") && k + 1 < argc)  epoch = strtoull(argv[++k], NULL, 0);
        else if (!strcmp(argv[k], "--diff") && k + 1 < argc)   diff = (unsigned)atoi(argv[++k]);
        else if (!strcmp(argv[k], "--window") && k + 1 < argc) window = atoi(argv[++k]);
        else if (!strcmp(argv[k], "--tmax") && k + 1 < argc)   tmax = atoi(argv[++k]);
        else if (!strcmp(argv[k], "--fan") && k + 1 < argc)    fan_fixed = atoi(argv[++k]);
        else if (!strcmp(argv[k], "--volts") && k + 1 < argc) {
            char *p = strtok(argv[++k], ",");
            while (p && n_volts < 32) { volts[n_volts++] = (unsigned)atoi(p); p = strtok(NULL, ","); }
        }
    }
    if (config) jas_config_set(config);

    /*
     * --probe-rails: READ-ONLY detection of the voltage regulators, by I2C
     * ACK (iic_single_byte_read == 0 => the chip answers => present). Presence
     * does NOT go through mcp2973_get_vol: its identity check requires
     * OPERATION=0x80, yet a regulator that is present but switched OFF returns
     * 0x00 => false "absent". The ACK depends only on physical presence.
     *
     * Goal: know whether the SECOND regulators (0x6b/0x62/0x66) exist, hence
     * whether the 0x450 topology (controllable memory rails) is physically
     * possible. The "a" ones (0x6a/0x60/0x64 = rail0) must answer: that is the
     * probe's own sanity check.
     */
    if (probe_rails) {
        static const struct { uint8_t addr; const char *role; } regs[] = {
            { 0x6a, "asic0 rail0  (a, always present)" },
            { 0x6b, "asic0 rail1/2 (b, 0x450 only)" },
            { 0x60, "asic1 rail0  (a, always present)" },
            { 0x62, "asic1 rail1/2 (b, 0x450 only)" },
            { 0x64, "asic2 rail0  (a, always present)" },
            { 0x66, "asic2 rail1/2 (b, 0x450 only)" },
        };
        unsigned i;

        printf("=== probe rails (read-only, I2C ACK) ===\n");
        if (jas_hal_init() != 0) { fprintf(stderr, "hal init failed\n"); return 1; }
        jas_bus_lock();
        jas_iic_map_init(0);
        for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
            uint8_t v = 0;
            int ack;
            /* Re-init BEFORE each read: a no-ACK (absent regulator) leaves
             * the AXI-IIC controller in error and makes all the following
             * reads fail (bug of the 1st version: only 0x6a answered). Start
             * clean each time. */
            iic_initialization_op(0);
            ack = iic_single_byte_read(regs[i].addr, 0x8b, &v, 0);
            jas_log("probe 0x%02x %-26s: %s",
                    regs[i].addr, regs[i].role,
                    ack == 0 ? "PRESENT (ACK)" : "ABSENT (no ACK)");
        }
        jas_bus_unlock();
        return 0;
    }

    struct sigaction sa; memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL); sigaction(SIGHUP, &sa, NULL);

    /* watchdog: generous cover for build (~11 min if a chip retries) + sweep */
    signal(SIGALRM, on_alarm);
    {
        unsigned pts = n_volts ? (unsigned)n_volts : 1u;
        unsigned wd  = (scrub || scrub_diag || cbsc_target || verify_dag || verify_load) ? 2700u   /* build + scrub/diag/cbsc/verify */
                             : 900u + pts * ((unsigned)(window > 0 ? window : 1) + 60u) + 120u;
        alarm(wd);
        printf("watchdog %u s\n", wd);
    }

    /* --- CBSC-TARGET: banks needed + severity curve (without dag_calc) --- */
    if (cbsc_target) {
        uint64_t block = epoch * EPOCH_LEN;
        ethash_h256_t seed;
        int bb; unsigned aa;

        printf("=== cbsc-target (epoch %llu) ===\n", (unsigned long long)epoch);
        sl_wrapper_init();
        if (!BOARD_EN) { fprintf(stderr, "no board detected\n"); return 1; }
        ethash_get_seedhash(&seed, block);
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                for (aa = 0; aa < ASIC_NUM; aa++)
                    if (ASIC_EN(bb) >> aa & 1)
                        jas_coin_from_block_seed(jas_coin_of(bb, aa), block, &seed);
        prepare_state();
        for (bb = 0; bb < BOARD_NUM && !stop; bb++)
            if (BOARD_EN >> bb & 1) {
                jas_log("cbsc: self-check board %d (scan CBSC)...", bb);
                self_check_func(bb);
            }
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                for (aa = 0; aa < ASIC_NUM; aa++)
                    if (ASIC_EN(bb) >> aa & 1)
                        do_cbsc_target(bb, aa);
        jas_log("cbsc-target done, powering off.");
        power_off_operation();
        return 0;
    }

    printf("=== jasre_stress ===\n");
    printf("epoch %llu, diff 2^%u, window %d s, tmax %d C\n",
           (unsigned long long)epoch, diff, window, tmax);

    /* --- 1. hardware + power-on (frequency = the config's) --- */
    sl_wrapper_init();
    if (power_off_only) {
        power_off_operation();
        jas_log("stress: ASIC rails cut (--power-off): the machine cools down, end.");
        return 0;
    }
    if (!BOARD_EN) { fprintf(stderr, "no board detected\n"); return 1; }

    /*
     * --scan-regs: DISCOVERY OF THE CHIP'S REGISTERS (read only).
     *
     * The idea comes from Bitmain's BM13xx chips, whose "Fast UART
     * Configuration" register (0x28: BT8D baud divider, BCLK_SEL clock
     * selector) takes the serial chain from 115 kbps to 3.1 Mbps: if Sunlune
     * did the same, the lever on the bus's slowness is a CHIP REGISTER.
     *
     * SAFETY. Never sl_single_read for a scan: its wait loop has NO TIME BOUND
     * and only exits on data ready or a reported link error; a register that
     * produces neither makes it spin forever, hammering the AXI bus, and the
     * machine becomes unreachable. The firmware's sl_single_read_soft is made
     * for probing: it honors the DELAY passed, does not test the link's error
     * bit, and does NOT reset the link when it fails (board_check uses it to
     * find which connectors carry a board).
     *
     * Three safeguards, all needed:
     *   1. sl_single_read_soft with a short delay (a real time bound);
     *   2. a log WRITTEN AND FLUSHED at each line, so that nothing is lost if
     *      the machine freezes anyway;
     *   3. a scan in SLICES, resumable, starting with the ranges known to be
     *      populated rather than 4096 blind addresses.
     *
     * Guessed AXI addresses are NOT touched: an unmapped AXI read never gets
     * an answer and freezes the ARM core. No write anywhere.
     */
    if (scan_regs) {
        static struct { uint8_t cmd; const char *name; } reads[] = {
            { 0x1c, "mesh0" }, { 0x2c, "mesh1" }, { 0xfd, "board" },
        };
        unsigned n_reads = sizeof(reads) / sizeof(reads[0]);

        /* --scan-cmd: probe ONLY ONE command. To query the BOARD/FPGA space
         * (0xfd) without needlessly involving the meshes: each silent address
         * leaves a pending transaction, so the fewer, the better. */
        if (scan_cmd) {
            reads[0].cmd = (uint8_t)scan_cmd;
            reads[0].name = (scan_cmd == 0xfd) ? "board" :
                              (scan_cmd == 0x1c) ? "mesh0" : "cmd";
            n_reads = 1;
        }
        unsigned l, a, found = 0, silent = 0;
        int b = 0;
        FILE *j;

        while (b < BOARD_NUM && !(BOARD_EN >> b & 1)) b++;
        if (b >= BOARD_NUM) { fprintf(stderr, "no board\n"); return 1; }

        /* Separate log, flushed at each line: survives a freeze. */
        j = fopen("/media/data/scan_regs.txt", "a");
        if (j)
            fprintf(j, "\n=== scan board %d asic %u 0x%04x..0x%04x ===\n",
                    b, scan_asic, scan_lo, scan_hi);

        /*
         * CONFIGURE BEFORE READING (--scan-cfg): right after the hardware reset
         * the registers hold an undetermined state (a scan returns noise).
         * config_asic_fix_param sets the 25 known registers in a few seconds,
         * without building a DAG; the scan can then be VALIDATED (do we find
         * the written values?) and what is around them read.
         */
        if (scan_cfg) {
            config_asic_fix_param(b);
            jas_log("stress: config_asic_fix_param applied before the scan");
        }
        jas_log("stress: REGISTER SCAN board %d asic %u, 0x%04x..0x%04x (GENTLE"
                " read, bounded to %u us%s)", b, scan_asic, scan_lo, scan_hi,
                scan_us, scan_cfg ? ", after config" : ", raw after reset");
        for (l = 0; l < n_reads && !stop; l++) {
            unsigned n = 0, no_reply = 0;

            for (a = scan_lo; a <= scan_hi && !stop; a++) {
                uint16_t v = 0;

                /* The PROBING primitive: bounded, no link reset. */
                if (sl_single_read_soft((uint8_t)(1u << scan_asic), reads[l].cmd,
                                        (uint16_t)a, &v, scan_us, b) < 0) {
                    no_reply++;
                    continue;                       /* no answer: move on */
                }
                if (v == 0x0000 || v == 0xffff)
                    continue;                       /* of no interest */
                n++;
                if (j) {
                    fprintf(j, "%s 0x%04x = 0x%04x\n", reads[l].name, a, v);
                    fflush(j);                      /* <- nothing must be lost */
                }
                if (n <= 48)
                    jas_log("  reg[%s] 0x%04x = 0x%04x", reads[l].name, a, v);
            }
            jas_log("stress: %s: %u non-trivial registers, %u without reply",
                    reads[l].name, n, no_reply);
            found += n; silent += no_reply;
        }
        jas_log("stress: scan done: %u registers, %u silent. Log in"
                " /media/data/scan_regs.txt. No write made.",
                found, silent);
        if (j) fclose(j);
        power_off_operation();
        return 0;
    }
    for (int b = 0; b < BOARD_NUM; b++)   /* sensors, per board, for temp reads */
        if (BOARD_EN >> b & 1) jas_sensors_configure((unsigned)b);

    /* --- 2. CPU cache for the epoch (needed by dag_calc's checks + our verify) --- */
    uint64_t block = epoch * EPOCH_LEN;
    ethash_h256_t seed; ethash_get_seedhash(&seed, block);
    jas_log("stress: building CPU cache for epoch %llu...", (unsigned long long)epoch);
    ethash_light_t light = ethash_light_new(block);
    if (!light) { fprintf(stderr, "CPU cache failed (memory?)\n"); return 1; }
    jas_cache_save(block, &seed, light);

    /* --- 3. per-chip coin context: our synthetic epoch/header/low target, so the
     *        real bring-up builds and first-works with them --- */
    ethash_h256_t target; set_target(&target, diff);
    ethash_h256_t hdr;   uint32_t sel = 0; set_header(&hdr, ++sel);
    for (int b = 0; b < BOARD_NUM; b++)
        if (BOARD_EN >> b & 1)
            for (unsigned a = 0; a < ASIC_NUM; a++)
                if (ASIC_EN(b) >> a & 1) {
                    jas_coin_ctx *c = jas_coin_of(b, a);
                    jas_coin_from_block_seed(c, block, &seed);
                    c->header = hdr;
                    c->target = target;
                }

    /* --- 4. REAL bring-up: prepare_state (ALU + memory-controller masks) +
     *        self_check + dag_calc + first work + ALU reopen --- */
    if (g_veto_asic >= 0) {
        jas_veto_hook = veto_hook_fn;   /* veto injected in seed_update_operation */
        jas_log("stress: VETO asic %d active (reduced to the DAG target before the build)", g_veto_asic);
    }
    jas_log("stress: full bring-up + DAG build (~5-7 min)...");
    seed_update_operation();
    if (stop) { power_off_operation(); return 0; }

    /* --- RECLAIM-ESSAI: the PRODUCTION path (dag_calc: prefix by the
     *     chip, tail written by the processor, remaps, checks) on a small
     *     epoch, with a FORCED tail of N nodes (JASRE_RECLAIM_TRIAL=N,
     *     JASRE_CACHE_RECLAIM=1). A cycle takes ~8 min instead of 30+ at
     *     epoch 840. Here: an independent, wider check of the tail. --- */
    if (reclaim_trial) {
        uint16_t zero[2] = { 0, 0 };
        const char *e = getenv("JASRE_RECLAIM_TRIAL");
        const char *ee = getenv("JASRE_RECLAIM_TRIAL_SAMPLES");
        uint32_t n = e ? (uint32_t)strtoul(e, NULL, 0) : 0;
        unsigned samples = ee ? (unsigned)strtoul(ee, NULL, 0) : 64u;
        int bb;

        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                sl_multi_write(ASIC_EN(bb), 0xf8, 0x40, zero, 2, bb);   /* ALU off: clean bus */
        sleep(1);
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++) {
                const jas_coin_ctx *c = jas_coin_of(bb, aa);
                uint32_t total = (uint32_t)(c->dag_size / 64u);
                uint32_t start = (n && n < total) ? ((total - n) & ~1u) : total;
                int r;

                if (!(ASIC_EN(bb) >> aa & 1)) continue;
                r = dag_check_repair(bb, aa, c, start, total, samples, 0, NULL);
                jas_log("reclaim-trial b%d a%u: tail [%u..%u) %u samples -> %s (%d)",
                        bb, aa, start, total, samples,
                        r == 0 ? "ALL GOOD" : r < 0 ? "NO CACHE" : "WRONG NODES", r);
            }
        }
        jas_log("stress: reclaim-trial done, powering off ASICs.");
        power_off_operation();
        return 0;
    }

    /* --- RECLAIM-PROBE: prove the write/read round trip (at rest) --- */
    if (reclaim_probe) {
        uint16_t zero[2] = { 0, 0 };
        int bb;
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                sl_multi_write(ASIC_EN(bb), 0xf8, 0x40, zero, 2, bb);   /* ALU off: clean bus */
        sleep(1);
        jas_log("stress: RECLAIM-PROBE (ALUs stopped, writes ONLY beyond the DAG)");
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++)
                if (ASIC_EN(bb) >> aa & 1) do_reclaim_probe_read(bb, aa);
        }
        /* PROVEN PREAMBLE before writing: without it the ARM core freezes. */
        for (bb = 0; bb < BOARD_NUM && !stop; bb++)
            if (BOARD_EN >> bb & 1) {
                preamble_write(bb);
                sl_multi_write(ASIC_EN(bb), 0xf8, 0x40, zero, 2, bb);   /* ALU still off */
            }
        sleep(1);
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++)
                if (ASIC_EN(bb) >> aa & 1) do_reclaim_probe_write(bb, aa);
        }
        jas_log("stress: reclaim-probe done, powering off ASICs.");
        power_off_operation();
        return 0;
    }

    /* --- VERIFY-DAG: is the DAG WRITE correct? (at rest, safe zone) --- */
    if (verify_dag || verify_load) {
        int bb;
        /* IDLE PASS: all ALUs stopped BUT MC refresh re-enabled (0xc40/0xc00)
         * -> cures the self-inflicted rot -> baseline of written CONTENT. */
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1) state_idle_refresh_on((unsigned)bb);
        sleep(1);
        jas_log("stress: VERIFY pass IDLE (refresh kept, %u reads/chip, %u votes, zone 4/6)",
                verify_n, scrub_votes);
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++) {
                int rd;
                if (!(ASIC_EN(bb) >> aa & 1)) continue;
                rd = first_good_alu((unsigned)bb, aa);
                if (rd < 0) { jas_log("verify[idle] b%d a%u: no good ALU, skipped", bb, aa); continue; }
                do_verify_read(bb, aa, (unsigned)rd, verify_n, scrub_votes, "idle");
            }
        }

        /* LOAD PASS (--verify-load): turn mining back on (live refresh through
         * the ALU reads), then idle ONE SINGLE ALU and read through it = same
         * path as mining, under real load. Load-idle difference = reject per
         * bank. */
        if (verify_load) {
            for (bb = 0; bb < BOARD_NUM; bb++)
                if (BOARD_EN >> bb & 1) state_mining_on((unsigned)bb);
            sleep(3);   /* let mining load/heat the chip */
            jas_log("stress: VERIFY pass LOAD (119 ALUs mining, 1 dedicated reader ALU)");
            for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
                if (!(BOARD_EN >> bb & 1)) continue;
                for (unsigned aa = 0; aa < ASIC_NUM; aa++) {
                    int rd;
                    if (!(ASIC_EN(bb) >> aa & 1)) continue;
                    rd = first_good_alu((unsigned)bb, aa);
                    if (rd < 0) { jas_log("verify[load] b%d a%u: no good ALU, skipped", bb, aa); continue; }
                    idle_one_alu((unsigned)bb, aa, (unsigned)rd);
                    usleep(200000);   /* let the reader ALU drain from mining */
                    do_verify_read(bb, aa, (unsigned)rd, verify_n, scrub_votes, "load");
                    restore_one_alu((unsigned)bb, aa, (unsigned)rd);
                }
            }
        }
        jas_log("stress: verify done, powering off ASICs.");
        power_off_operation();
        return 0;
    }

    /* --- SCRUB-DIAG: why the scrub is wrong (aligned vs arbitrary) --- */
    if (scrub_diag) {
        uint16_t zero[2] = { 0, 0 };
        int bb;
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                sl_multi_write(ASIC_EN(bb), 0xf8, 0x40, zero, 2, bb);
        sleep(1);
        jas_log("stress: SCRUB-DIAG (ALUs stopped)");
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++)
                if (ASIC_EN(bb) >> aa & 1) do_scrub_diag(bb, aa);
        }
        jas_log("stress: diag done, powering off ASICs.");
        power_off_operation();
        return 0;
    }

    /* --- SCRUB: locate the bad banks AT REST + VOTE --- */
    if (scrub) {
        uint16_t zero[2] = { 0, 0 };
        int bb;
        /* AT REST: stop the ALUs (REG_ALU_EN=0) -> no conflict with mining
         * (the scrub under load through ALU0 gave 100 % garbage; ALU0 cannot
         * read and mine at the same time). The VOTE removes the bus noise (~2 %
         * transient) -> only the banks really bad at rest remain. asic1
         * (healthy) must come out at ~0 = safeguard. */
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1)
                sl_multi_write(ASIC_EN(bb), 0xf8, 0x40, zero, 2, bb);
        sleep(1);
        jas_log("stress: SCRUB at idle + vote (%u samples/chip, %u votes)", scrub_n, scrub_votes);
        for (bb = 0; bb < BOARD_NUM && !stop; bb++) {
            if (!(BOARD_EN >> bb & 1)) continue;
            for (unsigned aa = 0; aa < ASIC_NUM; aa++)
                if (ASIC_EN(bb) >> aa & 1) do_scrub(bb, aa, scrub_n, scrub_votes);
        }
        jas_log("stress: scrub done, powering off ASICs.");
        power_off_operation();
        return 0;
    }

    /* --- (a bis) RETENTION / THROUGHPUT levers applied BEFORE the reject
     *     sweep. req_interval first (slows the fetch), refresh_on next (the
     *     retention safety net). The section 5 sweep then measures the reject
     *     with these settings. */
    if (req_interval >= 0) {
        int bb;
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1) set_req_interval((unsigned)bb, (unsigned)req_interval);
    }
    if (refresh_on) {
        int bb;
        for (bb = 0; bb < BOARD_NUM; bb++)
            if (BOARD_EN >> bb & 1) refresh_on_while_mining((unsigned)bb);
    }

    /* --- 5. voltage sweep (single point if --volts absent) --- */
    int n_points = n_volts ? n_volts : 1;

    jas_log("stress: %d voltage point(s) x %d s (epoch %llu, diff 2^%u, config freq)",
            n_points, window, (unsigned long long)epoch, diff);
    for (int pt = 0; pt < n_points && !stop; pt++) {
        unsigned mv = n_volts ? volts[pt] : 0;

        if (mv) { jas_log("stress: -- voltage %u mV --", mv); set_voltage(mv); }
        else      jas_log("stress: -- config voltage --");

        for (int s = 0; s < SETTLE_S && !stop; s++) sleep(1);

        /* fresh header + warm-up (nonces discarded) */
        set_header(&hdr, ++sel);
        broadcast_header(&hdr);
        drain_silent();
        for (int s = 0; s < WARMUP_S && !stop; s++) { sleep(1); drain_silent(); }

        jas_log("stress: reading temperature...");
        int t = temp_max_read();
        jas_log("stress: temperature max = %d C", t);
        if (t >= 0 && t >= tmax) {
            jas_log("stress: !! temperature %d C >= %d: ABORT", t, tmax);
            stop = 1; break;
        }

        /* counting window — FIXED header (no in-window roll: header_update on a
         * marginal chip can hang; the nonce range easily lasts the window at
         * 2^D). Time-bounded drain => always terminates on the deadline. */
        if (fan_fixed > 0) {
            fan_speed_set(fan_fixed);
            jas_log("stress: fans fixed at %d %%", fan_fixed);
        }
        jas_log("stress: counting window %d s...", window);
        static unsigned acc[3][3][120], rej[3][3][120];   /* static: ~8.6KB, off the stack */
        memset(acc, 0, sizeof(acc));
        memset(rej, 0, sizeof(rej));
        struct timeval t0; gettimeofday(&t0, NULL);
        long deadline = (long)window * 1000L, last_log = 0;
        while (elapsed_ms(&t0) < deadline && !stop) {
            unsigned got = drain_and_count(&hdr, acc, rej, &t0, deadline);
            if (got == 0) usleep(20000);   /* nothing pending: brief wait */
            long e = elapsed_ms(&t0);
            if (e - last_log >= 5000) {
                unsigned tot = 0;
                for (int b = 0; b < BOARD_NUM; b++)
                    for (unsigned a = 0; a < 3; a++)
                        for (unsigned u = 0; u < 120; u++) tot += acc[b][a][u] + rej[b][a][u];
                /* Refresh the setting: if an FPGA watchdog brings it back to a
                 * default value, impose it again. */
                if (fan_fixed > 0)
                    fan_speed_set(fan_fixed);
                jas_log("stress: window t=%ld s, %u nonces so far", e / 1000, tot);
                last_log = e;
            }
        }

        /* --- per-chip report: total, per mesh, worst ALU (locates the reject) --- */
        t = temp_max_read();
        jas_log("stress: === RESULTS %s%u mV (tmax %d C) ===",
                mv ? "" : "config ", mv, t);
        for (int b = 0; b < BOARD_NUM; b++) {
            if (!(BOARD_EN >> b & 1)) continue;
            for (unsigned a = 0; a < ASIC_NUM; a++) {
                unsigned ta = 0, tr = 0, tot;
                double rate;
                unsigned banks;
                if (!(ASIC_EN(b) >> a & 1)) continue;
                for (unsigned u = 0; u < 120; u++) { ta += acc[b][a][u]; tr += rej[b][a][u]; }
                tot = ta + tr;
                rate = tot ? 100.0 * tr / tot : 0.0;
                banks = jas_good_bank_num_cbsc[b * 3 + a];
                jas_log("stress: %u mV board %d asic %u: reject %6.2f%% (%u/%u nonces, %u/s), banks %u",
                        mv, b, a, rate, tr, tot,
                        tot / (unsigned)(window ? window : 1), banks);

                /* reject/mesh (concentrated -> amputable; uniform -> hard) */
                {
                    char line[160]; int off = 0; unsigned m;
                    off += snprintf(line + off, (size_t)(sizeof(line) - off), "  mesh:");
                    for (m = 0; m < 4; m++) {
                        unsigned ma = 0, mr = 0, mt, u;
                        for (u = m * 30; u < m * 30 + 30; u++) { ma += acc[b][a][u]; mr += rej[b][a][u]; }
                        mt = ma + mr;
                        off += snprintf(line + off, (size_t)(sizeof(line) - off),
                                        " m%u %.0f%%(%u/%u)", m, mt ? 100.0 * mr / mt : 0.0, mr, mt);
                    }
                    jas_log("stress:%s", line);
                }

                /* worst ALU (if reject is notable): few ALUs = vetoable */
                if (rate >= 5.0) {
                    unsigned wu = 0, wr = 0, u, wt;
                    for (u = 0; u < 120; u++)
                        if (rej[b][a][u] > wr) { wr = rej[b][a][u]; wu = u; }
                    wt = acc[b][a][wu] + rej[b][a][wu];
                    jas_log("stress:   worst ALU #%u (mesh %u): %u/%u reject", wu, wu / 30, wr, wt);
                }
            }
        }
    }

    jas_log("stress: done, powering off ASICs.");
    power_off_operation();
    return 0;
}
