/*
 * Crossbar scan (CBSC): producing the data.
 *
 * Memory is filled with a pattern whose every word is predictable, then
 * each ALU goes through the whole bank space and reports what it reads
 * back correctly: a matrix of 120 ALUs by 4080 banks per ASIC, which
 * crossbar_check_analysis then goes through.
 *
 * Each ALU goes through 32,640 addresses, i.e. eight passes over the 4,080
 * banks, one unit at a time inside a segment, then a large step from one
 * segment to the next. The pattern starts at 0x5a5a5a5a and advances with
 * the address: each word read back must be exactly what its address
 * predicts, so an error cannot cancel out.
 *
 * THE MEMORY CONTROLLERS ARE ON during the fill, and turned off only after
 * each chip's scan: the pattern is written the ordinary way, and only the
 * scan takes a direct path to read it back. Turning them off before the
 * fill makes it ineffective, and no ALU reports the end of its scan.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#define N_MESH        4u
#define ALUS_PER_MESH 30u
#define N_ALU         (N_MESH * ALUS_PER_MESH)
#define N_BANK        0xff0u
#define RESULT_WORDS  0xffu     /* 255 16-bit words cover the 4080 banks */

/* The 120x170 fallback is defined at the end of the file; it is called
 * from the normal scan, which comes before it. */
#define CB_TIMEOUT_THRESHOLD   0xbf3fu
int  cb_fallback_needed(unsigned expiries, int thread_id, unsigned asic);
int  cb_fallback_120x170(int thread_id, unsigned asic);
unsigned cbsc_ethw_floor(void);   /* defined in crossbar.c: ETHW viability floor */

#define SWEEP_LEN     0x7f80u   /* 32640 addresses, i.e. eight passes */
#define SEG_LEN       N_BANK    /* length of a segment */
#define BIG_STEP_LO   0x4041u   /* segment jump, 32 bits */
#define BIG_STEP_HI   0x007fu
#define FILL_PATTERN  0x5a5a5a5au

#define REG_SOFT_RESET  0x830
#define REG_MC_EN       0xc00
#define REG_MC_REFRESH  0xc40
#define REG_START_ADDR  0x502
#define REG_ADDR_STEP   0x540
#define REG_ADDR_SEG    0x6a5
#define REG_STEP_BIG    0x1000
#define REG_STEP_NUM    0x580
#define REG_START_DATA  0x103c
#define REG_DATA_STEP   0x103e
#define REG_ARM         0x5bc
#define REG_SCAN_EN     0x500
#define REG_SCAN_DONE   0x5be
#define REG_RESULT_SEL  0x5e0
#define REG_RESULT_BUF  0x16e2   /* the scan's result */
#define REG_TIMEOUT_BUF 0x1040   /* the timeouts */
#define CMD_BROADCAST   0xf8

/*
 * Fills the whole memory with the reference pattern.
 *
 * The address and the data advance together: by one unit inside a segment
 * of 4080, then by a large step when moving to the next segment. Each word
 * is thus determined by its position, which lets the hardware check by
 * itself what it reads back.
 */
/*
 * Length of the scan, adjustable for trials.
 *
 * 32640 in operation: eight passes over the 4080 banks, with a large step
 * between segments. Reducing it to 4080 removes the large step and only
 * tries one segment: it is the only way to know whether the divergence
 * comes from the segment boundary or from the scan itself.
 */
static unsigned sweep_len(void)
{
    const char *e = getenv("JASRE_CB_SWEEP");

    return (e && *e) ? (unsigned)strtoul(e, NULL, 0) : SWEEP_LEN;
}

/*
 * Fill variant, for the measurement campaign.
 *   0  control: two blocks per scan address, at addresses 2n and 2n+1
 *   1  a single block, at address 2n
 *   2  two blocks, at addresses n and n+1 (no doubling)
 *   3  two distinct blocks at 2n and 2n+1 (the 64-word block is no longer
 *      uniform: tests whether the scan really compares 64 different words)
 */
int jas_cb_variant;

/*
 * FAST FILL (the default): the same sequence of blocks, but without the
 * 1 ms pause after each validation (two thirds of the original fill's
 * 91 s); instead, each targeted chip must announce "ready" before the next
 * block is broadcast to it, which guards against a block lost by a chip
 * that was not ready (see below). Judged by the per-chip CBSC count.
 */
static int cbsc_fast(void)
{
    static int loaded = 0, r = 0;

    if (!loaded) {
        const char *e = getenv("JASRE_CBSC_FAST");

        /* THE DEFAULT (fill in about 10 s instead of 91, normal CBSC count).
         * JASRE_CBSC_FAST=0 or the file /tmp/jasre_cbsc_slow bring back
         * the original fill. */
        r = (e && *e) ? (*e != '0') : (access("/tmp/jasre_cbsc_slow", F_OK) != 0);
        loaded = 1;
        if (r)
            jas_log("crossbar: FAST fill (no 1 ms pause, ready poll before each block)");
    }
    return r;
}

static void cb_write(uint8_t asic_en, uint32_t address, uint16_t *data, int thread_id)
{
    if (cbsc_fast())
        single_manual_write_param(asic_en, address, data, thread_id, 0u, 1);
    else
        single_manual_write(asic_en, address, data, thread_id);
}

void crossbar_fill_pub(int thread_id)
{
    uint32_t sram[16];
    uint32_t addr = 0, data = FILL_PATTERN;
    uint32_t big = ((uint32_t)BIG_STEP_HI << 16) | BIG_STEP_LO;
    unsigned n;

    jas_log("board %d: fill, %u writes", thread_id, sweep_len() * 2);

    /* The fill alone is more than two million serial transactions. Marking
     * its progress is the only way to tell "slow" from "stuck": without it,
     * an endless wait in sl_single_write cannot be told from a loop that
     * advances. */
    unsigned sweep = sweep_len();
    const char *e = getenv("JASRE_CB_DATABIG");
    int data_big = !(e && *e == '0');
    const char *e2 = getenv("JASRE_CB_CONSTDATA");
    const char *e3 = getenv("JASRE_CB_ONEASIC");
    int one_chip = (e3 && *e3 == '1');

    if (one_chip)
        jas_trace("board %d: JASRE_CB_ONEASIC, fill asic by asic", thread_id);
    int const_data = (e2 && *e2 == '1');

    if (const_data)
        jas_log("board %d: JASRE_CB_CONSTDATA, constant pattern", thread_id);

    for (n = 0; n < sweep; n++) {
        if (n % 2000 == 0) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            jas_trace("  fill %u/%u  t=%ld.%03ld",
                    n, sweep, (long)tv.tv_sec, (long)tv.tv_usec / 1000);
            /* The home page's bar: the fill takes the 40-90 % range, like the
             * original's steps. */
            jas_web_dag_publish(40u + (n * 50u) / (sweep ? sweep : 1u));
        }
        for (unsigned k = 0; k < 16; k++)
            sram[k] = data;

        /*
         * ONE CHIP AT A TIME, OR ALL THREE BY BROADCAST? The fill writes to
         * asic_en = 7, all three chips at once, while the acknowledgement is
         * INDIVIDUAL (0x824 is polled chip by chip; patent CN114003526B
         * describes a target controller that caches, then signals ready). A
         * chip not ready at broadcast time loses its block while the other
         * two absorb it: a uniform loss, independent of data, address and
         * time. JASRE_CB_ONEASIC=1 writes chip by chip to test it.
         */
        if (one_chip) {
            unsigned a;

            for (a = 0; a < ASIC_NUM; a++) {
                if (!(ASIC_EN(thread_id) >> a & 1)) continue;
                single_manual_write((uint8_t)(1u << a), addr * 2,
                                    (uint16_t *)sram, thread_id);
                single_manual_write((uint8_t)(1u << a), addr * 2 + 1,
                                    (uint16_t *)sram, thread_id);
            }
        } else
        switch (jas_cb_variant) {
        case 1:
            single_manual_write(ASIC_EN(thread_id), addr * 2, (uint16_t *)sram, thread_id);
            break;
        case 2:
            single_manual_write(ASIC_EN(thread_id), addr,     (uint16_t *)sram, thread_id);
            single_manual_write(ASIC_EN(thread_id), addr + 1, (uint16_t *)sram, thread_id);
            break;
        case 3: {
            uint32_t second[16];
            for (unsigned q = 0; q < 16; q++) second[q] = data + 16 + q;
            for (unsigned q = 0; q < 16; q++) sram[q] = data + q;
            single_manual_write(ASIC_EN(thread_id), addr * 2,     (uint16_t *)sram, thread_id);
            single_manual_write(ASIC_EN(thread_id), addr * 2 + 1, (uint16_t *)second, thread_id);
            break;
        }
        default:
            cb_write(ASIC_EN(thread_id), addr * 2,     (uint16_t *)sram, thread_id);
            cb_write(ASIC_EN(thread_id), addr * 2 + 1, (uint16_t *)sram, thread_id);
        }

        if (const_data) {
            /* Constant pattern: the data no longer follows the address. If the
             * coverage jumps, the divergence is in the data SEQUENCE, not in
             * the address walk. */
            data = FILL_PATTERN;
        }
        if ((n + 1) % SEG_LEN == 0) {
            addr += big;
            /*
             * The hardware has only ONE step register for the data (0x103e),
             * with no equivalent of the large address step. Nothing says, then,
             * that its data counter also jumps at the segment boundary. The
             * original adds the large step to both; the trial adds it to the
             * address only.
             */
            if (data_big)
                data += big;
            else
                data += 1;
        } else {
            addr += 1;
            data += 1;
        }
    }
}

/* Programs the walk parameters, identical for the 120 ALUs. */
void crossbar_config_pub(int thread_id)
{
    uint16_t start_addr[N_MESH][ALUS_PER_MESH * 2];
    uint16_t addr_step[N_MESH][ALUS_PER_MESH * 2];
    uint16_t addr_seg[N_MESH][ALUS_PER_MESH * 2];
    uint16_t step_big[N_MESH][ALUS_PER_MESH * 2];
    uint16_t step_num[N_MESH][ALUS_PER_MESH * 2];
    uint16_t start_data[2] = { 0x5a5a, 0x5a5a };
    uint16_t data_step[2]  = { 1, 0 };
    const char *cd = getenv("JASRE_CB_CONSTDATA");

    if (cd && *cd == '1')
        data_step[0] = 0;               /* the scan no longer increments either */
    uint8_t en = ASIC_EN(thread_id);
    unsigned mesh, alu;

    /*
     * SINGLE-ADDRESS PROBE.
     *
     * Rather than guess the address -> result bit convention, measure it: a
     * scan with step_num=1 at start_addr=A must touch a single bit. By
     * varying A, the mapping can be read directly.
     */
    unsigned long sa = 0;
    { const char *e = getenv("JASRE_CB_STARTADDR");
      if (e && *e) { sa = strtoul(e, NULL, 0);
                     jas_log("board %d: start_addr forced to %lu (0x%lx)", thread_id, sa, sa); } }

    memset(start_addr, 0, sizeof(start_addr));
    memset(addr_step, 0, sizeof(addr_step));
    memset(addr_seg, 0, sizeof(addr_seg));
    memset(step_big, 0, sizeof(step_big));
    memset(step_num, 0, sizeof(step_num));

    /* Each parameter is a 32-bit word, hence two entries per ALU. */
    for (mesh = 0; mesh < N_MESH; mesh++) {
        for (alu = 0; alu < ALUS_PER_MESH; alu++) {
            unsigned lo = alu * 2, hi = lo + 1;

            start_addr[mesh][lo] = (uint16_t)(sa & 0xffff);
            start_addr[mesh][hi] = (uint16_t)(sa >> 16);
            addr_step[mesh][lo]  = 1;              addr_step[mesh][hi]  = 0;
            addr_seg[mesh][lo]   = SEG_LEN;        addr_seg[mesh][hi]   = 0;
            step_big[mesh][lo]   = BIG_STEP_LO;    step_big[mesh][hi]   = BIG_STEP_HI;
            step_num[mesh][lo]   = (uint16_t)sweep_len(); step_num[mesh][hi] = 0;
        }
    }

    sl_multi_write(en, CMD_BROADCAST, REG_START_ADDR, start_addr[0], 0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_ADDR_STEP,  addr_step[0],  0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_ADDR_SEG,   addr_seg[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_STEP_BIG,   step_big[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_STEP_NUM,   step_num[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_START_DATA, start_data,    2,    thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_DATA_STEP,  data_step,     2,    thread_id);
    sl_single_write(en, CMD_BROADCAST, REG_ARM, 0xffff, thread_id);
}

/* Turns the memory controllers of the whole board on or off. */
static void crossbar_mc_set(int thread_id, int on)
{
    uint16_t buf[64];

    memset(buf, on ? 0xff : 0x00, sizeof(buf));
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_MC_EN, buf, 0x40, thread_id);
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_MC_REFRESH, buf, 0x40, thread_id);
}

/* Same, for a single chip: the original turns them off chip by chip, after
 * its scan, and not all together at the end. */
static void crossbar_mc_set_asic(int thread_id, unsigned asic, int on)
{
    uint16_t buf[64];
    uint8_t en = (uint8_t)(1u << asic);

    memset(buf, on ? 0xff : 0x00, sizeof(buf));
    sl_multi_write(en, CMD_BROADCAST, REG_MC_REFRESH, buf, 0x40, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_MC_EN, buf, 0x40, thread_id);
}

/*
 * Starts the scan, waits for its end, and collects the results.
 *
 * Two buffers are read back per mesh: the successes and the timeouts.
 * Telling them apart matters: a bank not reached because the path timed
 * out is not in the same state as a bank whose read-back is wrong.
 */
/*
 * Starts a chip's scan and collects what each ALU could read.
 *
 * NO long wait: one second after the start order, the end flag is read
 * (one per mesh, with that mesh's read command), each of its thirty bits
 * designating an ALU. An ALU whose bit is missing is reported and left
 * aside; the others are collected. (The flag is not global: waiting for
 * all, or reading it by broadcast, never completes.)
 *
 * For each ALU kept, a single-bit selector designates which of the thirty
 * present its results, then two buffers of 255 words are read back:
 * 0x16e2 carries the RESULT, 0x1040 the TIMEOUTS. Swapping them would
 * analyze the wrong data with nothing reported.
 */
/*
 * Read back the bitmap of an ALREADY completed scan, without restarting
 * anything.
 *
 * Same per-ALU selection, same register, same length as the normal pass:
 * only the destination changes. Two reads of a buffer that has not moved
 * must return exactly the same thing.
 */
static void cb_reread(int thread_id, unsigned asic, uint16_t (*dst)[RESULT_WORDS])
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint16_t sel[N_MESH][2];
    unsigned mesh, alu;

    for (mesh = 0; mesh < N_MESH; mesh++) {
        uint8_t cmd_w = (uint8_t)((1u << mesh) << 4);
        uint8_t cmd_r = (uint8_t)(cmd_w | 0xc);

        for (alu = 0; alu < ALUS_PER_MESH; alu++) {
            unsigned idx = mesh * ALUS_PER_MESH + alu;

            memset(sel, 0, sizeof(sel));
            sel[mesh][alu >> 4] = (uint16_t)(1u << (alu & 0xf));
            sl_multi_write(asic_en, (uint8_t)(cmd_w | 8), REG_RESULT_SEL,
                           sel[mesh], 2, thread_id);
            sl_multi_read(asic_en, cmd_r, REG_RESULT_BUF, dst[idx],
                          RESULT_WORDS, 1000000, thread_id);
        }
    }
}

static unsigned long cb_bits(const uint16_t (*a)[RESULT_WORDS])
{
    unsigned long n = 0;
    unsigned k, w;

    for (k = 0; k < N_ALU; k++)
        for (w = 0; w < RESULT_WORDS; w++) {
            uint16_t v = a[k][w];
            while (v) { n += v & 1; v >>= 1; }
        }
    return n;
}

static unsigned long cb_gap(const uint16_t (*a)[RESULT_WORDS],
                              const uint16_t (*b)[RESULT_WORDS])
{
    unsigned long n = 0;
    unsigned k, w;

    for (k = 0; k < N_ALU; k++)
        for (w = 0; w < RESULT_WORDS; w++) {
            uint16_t v = (uint16_t)(a[k][w] ^ b[k][w]);
            while (v) { n += v & 1; v >>= 1; }
        }
    return n;
}

static uint16_t cb_buf_b[N_ALU][RESULT_WORDS];
static uint16_t cb_buf_c[N_ALU][RESULT_WORDS];
static uint16_t cb_buf_pre[N_ALU][RESULT_WORDS];
static unsigned long cb_pre_bits;

/*
 * Diagnostic: three stages, three questions.
 *
 *   A vs B: read-back of the same buffer, without restarting anything.
 *           different -> OUR read is unstable.
 *   A vs C: scan restarted, WITHOUT refilling the memory.
 *           different -> the scan draws lots at each pass;
 *           identical -> the result is fixed by the memory content, so
 *           the gap comes from the fill.
 */
static void cb_three_stages(int thread_id, unsigned asic)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint32_t scan_en;
    unsigned long a, b, c;

    memset(cb_buf_b, 0, sizeof(cb_buf_b));
    memset(cb_buf_c, 0, sizeof(cb_buf_c));

    cb_reread(thread_id, asic, cb_buf_b);

    scan_en = 0;
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);
    scan_en = 0xffffffffu;
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);
    usleep(1000000);
    cb_reread(thread_id, asic, cb_buf_c);

    a = cb_bits((const uint16_t (*)[RESULT_WORDS])jas_cb_sc_result[thread_id][asic]);
    b = cb_bits((const uint16_t (*)[RESULT_WORDS])cb_buf_b);
    c = cb_bits((const uint16_t (*)[RESULT_WORDS])cb_buf_c);

    jas_log("  A sweep+read:       %lu bits (%lu/ALU)", a, a / N_ALU);
    jas_log("  B read back only:   %lu bits (%lu/ALU), gap A-B = %lu",
            b, b / N_ALU,
            cb_gap((const uint16_t (*)[RESULT_WORDS])jas_cb_sc_result[thread_id][asic],
                     (const uint16_t (*)[RESULT_WORDS])cb_buf_b));
    jas_log("  C sweep restarted:  %lu bits (%lu/ALU), gap A-C = %lu",
            c, c / N_ALU,
            cb_gap((const uint16_t (*)[RESULT_WORDS])jas_cb_sc_result[thread_id][asic],
                     (const uint16_t (*)[RESULT_WORDS])cb_buf_c));
    jas_log("                      gap B-C = %lu",
            cb_gap((const uint16_t (*)[RESULT_WORDS])cb_buf_b,
                     (const uint16_t (*)[RESULT_WORDS])cb_buf_c));
    if (cb_pre_bits)
        jas_log("  PRE (no sweep):     %lu bits (%lu/ALU), gap PRE-A = %lu",
                cb_pre_bits, cb_pre_bits / N_ALU,
                cb_gap((const uint16_t (*)[RESULT_WORDS])cb_buf_pre,
                         (const uint16_t (*)[RESULT_WORDS])jas_cb_sc_result[thread_id][asic]));
}

int crossbar_run_pub(int thread_id, unsigned asic)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint32_t scan_en = 0xffffffffu;
    uint16_t sel[N_MESH][2];
    uint16_t timeout_buf[RESULT_WORDS];
    unsigned mesh, alu, missing = 0;
    unsigned long timeouts = 0;
    const char *e_delay = getenv("JASRE_CB_DELAY");
    unsigned delay_s = (e_delay && *e_delay) ? (unsigned)strtoul(e_delay, NULL, 0) : 0;

    /*
     * Deliberate wait between the fill and the scan.
     *
     * The fill lasts six and a half seconds; the first data written must
     * survive in DRAM until they are read back. If the failure rate comes
     * from retention, lengthening this delay must make it worse.
     */
    if (delay_s) {
        jas_log("board %d asic %u: JASRE_CB_DELAY, waiting %u s",
                thread_id, asic, delay_s);
        sleep(delay_s);
    }

    /*
     * Stage zero (diagnostic): read the result buffer BEFORE arming the
     * scan. If it already returns mostly ones with no scan run, the scan
     * contributes nothing and what is measured is the noise of a read on an
     * undriven bus.
     */
    if (getenv("JASRE_CB_STAGES")) {
        uint16_t d0[2] = { 0, 0 };

        memset(cb_buf_pre, 0, sizeof(cb_buf_pre));
        sl_multi_read(asic_en, 0x1c, REG_SCAN_DONE, d0, 2, 1000000, thread_id);
        jas_log("  0x5be BEFORE arming, mesh 0: %04x %04x", d0[0], d0[1]);
        cb_reread(thread_id, asic, cb_buf_pre);
        cb_pre_bits = cb_bits((const uint16_t (*)[RESULT_WORDS])cb_buf_pre);
        jas_log("  PRE read without sweep: %lu bits (%lu/ALU)",
                cb_pre_bits, cb_pre_bits / N_ALU);
    }

    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);
    usleep(1000000);

    memset(sel, 0, sizeof(sel));

    for (mesh = 0; mesh < N_MESH; mesh++) {
        uint8_t cmd_w = (uint8_t)((1u << mesh) << 4);
        uint8_t cmd_r = (uint8_t)(cmd_w | 0xc);
        uint16_t done[2] = { 0, 0 };
        uint32_t done_mask;

        sl_multi_read(asic_en, cmd_r, REG_SCAN_DONE, done, 2, 1000000, thread_id);
        done_mask = (uint32_t)done[0] | ((uint32_t)done[1] << 16);

        for (alu = 0; alu < ALUS_PER_MESH; alu++) {
            unsigned idx = mesh * ALUS_PER_MESH + alu;

            if (!(done_mask >> alu & 1)) {
                missing++;
                continue;
            }

            memset(sel, 0, sizeof(sel));
            sel[mesh][alu >> 4] = (uint16_t)(1u << (alu & 0xf));

            sl_multi_write(asic_en, (uint8_t)(cmd_w | 8), REG_RESULT_SEL,
                           sel[mesh], 2, thread_id);
            sl_multi_read(asic_en, cmd_r, REG_RESULT_BUF,
                          jas_cb_sc_result[thread_id][asic][idx],
                          RESULT_WORDS, 1000000, thread_id);
            sl_multi_read(asic_en, cmd_r, REG_TIMEOUT_BUF, timeout_buf,
                          RESULT_WORDS, 1000000, thread_id);

            /*
             * What do the buffers really look like?
             *
             * 0x1040 returns 489600 bits, i.e. all of them, on every ALU and
             * every chip, while the firmware counts only about fifteen. A
             * saturated register is not a measurement. So the first words of
             * BOTH buffers are looked at, plus two neighboring addresses, to
             * see which carry data and which are not driven.
             */
            if (getenv("JASRE_CB_DUMP") && mesh == 0 && alu == 0) {
                static const uint16_t cand[] = { 0x16e2, 0x1040, 0x17e2, 0x1140 };
                uint16_t probe[8];
                unsigned c;

                for (c = 0; c < 4; c++) {
                    memset(probe, 0, sizeof(probe));
                    sl_multi_read(asic_en, cmd_r, cand[c], probe, 8, 1000000, thread_id);
                    jas_log("  buffer 0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x",
                            cand[c], probe[0], probe[1], probe[2], probe[3],
                            probe[4], probe[5], probe[6], probe[7]);
                }
            }

            /*
             * The firmware counts these bits and calls them cb_timeout_cnt;
             * on a healthy machine it finds fewer than twenty per chip. A high
             * count says the scan's memory accesses do not complete, which then
             * reads as partial coverage, with nothing reporting the real cause.
             */
            for (unsigned w = 0; w < RESULT_WORDS; w++) {
                uint16_t t = timeout_buf[w];
                /*
                 * ~timeout MASK: a bank whose access TIMED OUT is NOT good,
                 * even if its result bit (0x16e2) latched at 1. On marginal
                 * memory these noisy bits latch differently at each boot while
                 * the timeout map (0x1040) is stable. The original does
                 * result & ~timeout (in crossbar_check), as does our 120x170
                 * fallback (crossbar_run_fallback). Without it the count is
                 * unstable from boot to boot, marginal banks are kept and the
                 * DAG flakes while mining; with it, ~3600 stable, like the
                 * original.
                 */
                jas_cb_sc_result[thread_id][asic][idx][w] &= (uint16_t)~t;
                while (t) { timeouts += t & 1; t >>= 1; }
            }
        }
    }

    {
        /* How many banks could each ALU reach, on average? A healthy scan
         * must come close to 4080; near zero, the pattern was not written or
         * the results are read from the wrong place. */
        unsigned long total = 0;
        unsigned k, w;

        for (k = 0; k < N_ALU; k++)
            for (w = 0; w < RESULT_WORDS; w++) {
                uint16_t v = jas_cb_sc_result[thread_id][asic][k][w];
                while (v) { total += v & 1; v >>= 1; }
            }
        jas_trace("board %d asic %u: %u alu never finishing, %lu banks reached on average, cb_timeout_cnt = %lu",
                thread_id, asic, missing, total / N_ALU, timeouts);

        /*
         * THE FALLBACK, if the scan timed out too much.
         *
         * Firmware threshold: 0xbf3f. JASRE_CB_THRESHOLD lowers it to TRY the
         * fallback on a healthy chip; otherwise it would only trigger on a
         * degraded chip, that is never when one wants to check it. The check
         * that counts: on a healthy chip, the fallback must return ROUGHLY THE
         * SAME result as the normal mode.
         */
        {
            const char *e_threshold = getenv("JASRE_CB_THRESHOLD");
            unsigned threshold = (e_threshold && *e_threshold)
                             ? (unsigned)strtoul(e_threshold, NULL, 0)
                             : CB_TIMEOUT_THRESHOLD;

            if (timeouts > threshold) {
                unsigned long before = total / N_ALU;

                if (before < cbsc_ethw_floor()) {
                    /*
                     * PER-CHIP RULE: below the ETHW floor, the chip is
                     * ETC-only and the fallback is NOT STARTED. The 120x170
                     * fallback (AND merge on a slow crossbar) WIPES such a
                     * chip to 0, while the masked normal scan (~timeout)
                     * already gives ~3600 solid banks, enough for ETC.
                     */
                    jas_log("board %d asic %u: bad asic, etc only (%lu banks <"
                            " %u) — fallback avoided",
                            thread_id, asic, before, cbsc_ethw_floor());
                } else {
                    cb_fallback_needed((unsigned)timeouts, thread_id, asic);
                    cb_fallback_120x170(thread_id, asic);

                    total = 0;
                    for (k = 0; k < N_ALU; k++)
                        for (w = 0; w < RESULT_WORDS; w++) {
                            uint16_t v = jas_cb_sc_result[thread_id][asic][k][w];
                            while (v) { total += v & 1; v >>= 1; }
                        }
                    jas_log("board %d asic %u: after fallback, %lu banks on"
                            " average (before %lu)", thread_id, asic,
                            total / N_ALU, before);
                }
            }
        }
    }

    if (getenv("JASRE_CB_STAGES"))
        cb_three_stages(thread_id, asic);

    scan_en = 0;
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);

    /* The controllers are turned off once the chip is collected. */
    crossbar_mc_set_asic(thread_id, asic, 0);
    return 0;
}

void crossbar_check(int thread_id)
{
    unsigned asic;

    if (ASIC_NUM == 0)
        return;

    /*
     * ZERO the result matrix BEFORE the scan, like the original (in
     * crossbar_check). Otherwise an ALU "not done" (end bit missing) keeps
     * its bits from the PREVIOUS scan in the same process (re-scan /
     * resume), which inflates the count non-deterministically. The BSS is
     * already zero on the first pass, but not on the following ones.
     */
    memset(jas_cb_sc_result[thread_id], 0, sizeof(jas_cb_sc_result[thread_id]));

    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_SOFT_RESET, 0, thread_id);
    sl_delay(0, 2000);
    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_SOFT_RESET, 0xffff, thread_id);

    crossbar_mc_set(thread_id, 1);      /* controllers ON: normal write path */
    crossbar_fill_pub(thread_id);
    crossbar_config_pub(thread_id);

    for (asic = 0; asic < ASIC_NUM; asic++) {
        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;
        crossbar_run_pub(thread_id, asic);
    }

    jas_log("board %d: analysis", thread_id);
    crossbar_check_analysis(thread_id);
    jas_log("board %d: crossbar done", thread_id);
}

/* ====================================================================== *
 * THE "120x170" FALLBACK: what the original does when the scan times out
 * too much.
 *
 * After merging the results, the original counts the timeout bits, and
 * beyond 0xbf3f (48,959):
 *
 *     printf("board[%d] asic[%d] too many cbsc timeout, try mode 120x170\n")
 *
 * Same registers, other values:
 *
 *     segment      0xaa      = 170 banks    (instead of the full segment)
 *     large step   0xfecf87                 (instead of 0x007f4041)
 *     count        0x2a8     = 680 steps    (instead of the whole scan)
 *
 * The fill follows the same cut: 680 writes, and every 170 steps the
 * address jumps by 0xfecf87 instead of advancing by 1.
 *
 * TWO DIFFERENCES OF SEMANTICS:
 *   - the result area is PRE-FILLED with 0xffff before the scan;
 *   - the merge is therefore an AND, not an OR: we start from "all good"
 *     and REMOVE what fails.
 *
 * Short windows give the memory controller time to keep up where a
 * continuous scan makes it time out: it is the firmware's recovery path
 * for a chip whose memory answers poorly.
 * ====================================================================== */

#define CB_FALLBACK_SEG       0xaau     /* 170 banks per window */
#define CB_FALLBACK_STEP_LO    0xcf87u   /* large step 0xfecf87, 32 bits */
#define CB_FALLBACK_STEP_HI    0x00feu
#define CB_FALLBACK_COUNT    0x2a8u    /* 680 steps */

/*
 * No counter here: the scan already counts these bits (see above,
 * "cb_timeout_cnt"). Writing a second one would mean two possible truths
 * for the same figure.
 */
int cb_fallback_needed(unsigned expiries, int thread_id, unsigned asic)
{
    if (expiries <= CB_TIMEOUT_THRESHOLD)
        return 0;
    jas_log("board[%d] asic[%d] too many cbsc timeout, try mode 120x170",
            thread_id, asic);
    return 1;
}

/*
 * Configure the scan in windows of 170 banks, for one chip.
 *
 * Same shape as the normal configuration (same registers, same order) with
 * the fallback's values. The original redoes it for the thirty ALUs of the
 * four meshes, which the double loop reproduces.
 */
void cb_config_fallback(int thread_id, unsigned asic, uint32_t base)
{
    uint16_t start_addr[N_MESH][ALUS_PER_MESH * 2];
    uint16_t addr_step[N_MESH][ALUS_PER_MESH * 2];
    uint16_t addr_seg[N_MESH][ALUS_PER_MESH * 2];
    uint16_t step_big[N_MESH][ALUS_PER_MESH * 2];
    uint16_t step_num[N_MESH][ALUS_PER_MESH * 2];
    uint16_t start_data[2] = { 0x5a5a, 0 };
    uint16_t data_step[2]  = { 1, 0 };
    uint8_t  en = (uint8_t)(1u << asic);
    unsigned mesh, alu;

    memset(start_addr, 0, sizeof(start_addr));
    memset(addr_step, 0, sizeof(addr_step));
    memset(addr_seg, 0, sizeof(addr_seg));
    memset(step_big, 0, sizeof(step_big));
    memset(step_num, 0, sizeof(step_num));

    for (mesh = 0; mesh < N_MESH; mesh++)
        for (alu = 0; alu < ALUS_PER_MESH; alu++) {
            unsigned lo = alu * 2, hi = lo + 1;

            start_addr[mesh][lo] = (uint16_t)(base & 0xffff);
            start_addr[mesh][hi] = (uint16_t)(base >> 16);
            addr_step[mesh][lo]  = 1;
            addr_seg[mesh][lo]   = CB_FALLBACK_SEG;
            step_big[mesh][lo]   = CB_FALLBACK_STEP_LO;
            step_big[mesh][hi]   = CB_FALLBACK_STEP_HI;
            step_num[mesh][lo]   = (uint16_t)CB_FALLBACK_COUNT;
        }

    sl_multi_write(en, CMD_BROADCAST, REG_START_ADDR, start_addr[0], 0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_ADDR_STEP,  addr_step[0],  0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_ADDR_SEG,   addr_seg[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_STEP_BIG,   step_big[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_STEP_NUM,   step_num[0],   0x3c, thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_START_DATA, start_data,    2,    thread_id);
    sl_multi_write(en, CMD_BROADCAST, REG_DATA_STEP,  data_step,     2,    thread_id);
    sl_single_write(en, CMD_BROADCAST, REG_ARM, 0xffff, thread_id);
}

/*
 * Scan ONE window and merge with AND.
 *
 * Same sequence as the normal mode (enable 0x500, leave a second, then
 * read back) but the result is combined by AND into the map already
 * pre-filled with 0xffff: a window can only invalidate banks.
 */
static void crossbar_run_fallback(int thread_id, unsigned asic, uint32_t base)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint32_t scan_en = 0xffffffffu;
    static uint16_t res[RESULT_WORDS];
    static uint16_t tmo[RESULT_WORDS];
    uint16_t done[2], sel[2];
    unsigned mesh, alu, w;

    /*
     * THREE POINTS OF THE ORIGINAL THAT ARE EASY TO GET WRONG:
     *
     * 1. THE MERGE COVERS THE 255 WORDS, not the window's slice:
     *    *puVar7 = *puVar7 & *puVar30 & ~uVar1; over the whole map. The
     *    "+ (uVar24 >> 4)" offset (base / 16) belongs to another branch
     *    (point 2); applied to the merge, it gives identical counts on
     *    different chips, an artifact.
     *
     * 2. AN ALU THAT HAS NOT FINISHED invalidates ONLY its window: the
     *    original reads 0x5be per mesh and, if the ALU's bit is zero,
     *    clears eleven words from base/16 (the 170 banks of the window,
     *    rounded to the word). That is where the offset lives.
     *
     * 3. THE TIMEOUTS COUNT: "& ~uVar1", uVar1 coming from 0x1040: a bank
     *    that timed out is removed even if 0x16e2 says it is good.
     */

    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);
    sleep(1);

    for (mesh = 0; mesh < N_MESH; mesh++) {
        uint8_t cmd_w = (uint8_t)((1u << mesh) << 4);
        uint8_t cmd_r = (uint8_t)(cmd_w | 0xc);
        uint32_t bits;

        done[0] = done[1] = 0;
        sl_multi_read(asic_en, cmd_r, REG_SCAN_DONE, done, 2, 1000000, thread_id);
        bits = (uint32_t)done[0] | ((uint32_t)done[1] << 16);

        for (alu = 0; alu < ALUS_PER_MESH; alu++) {
            unsigned idx = mesh * ALUS_PER_MESH + alu;

            if (((bits >> alu) & 1u) == 0) {
                /*
                 * "cbsc is not done" branch: eleven words, no more.
                 * The carry "(uVar24 & 0xe) != 0" is copied as is: it
                 * ignores bit 0, which is no ordinary rounding, and
                 * transcribing beats rationalizing here.
                 */
                unsigned w0 = base / 16u + ((base & 0xeu) != 0u ? 1u : 0u);
                unsigned k;

                for (k = 0; k < 11u && w0 + k < RESULT_WORDS; k++)
                    jas_cb_sc_result[thread_id][asic][idx][w0 + k] = 0;
                continue;
            }

            sel[0] = sel[1] = 0;
            sel[alu >> 4] = (uint16_t)(1u << (alu & 0xf));
            sl_multi_write(asic_en, (uint8_t)(cmd_w | 8), REG_RESULT_SEL,
                           sel, 2, thread_id);

            memset(res, 0, sizeof(res));
            memset(tmo, 0, sizeof(tmo));
            sl_multi_read(asic_en, cmd_r, REG_RESULT_BUF,  res,
                          RESULT_WORDS, 1000000, thread_id);
            sl_multi_read(asic_en, cmd_r, REG_TIMEOUT_BUF, tmo,
                          RESULT_WORDS, 1000000, thread_id);

            for (w = 0; w < RESULT_WORDS; w++)
                jas_cb_sc_result[thread_id][asic][idx][w] &=
                    (uint16_t)(res[w] & ~tmo[w]);
        }
    }

    /*
     * Turn the scan and the memory controllers off BEFORE the next
     * window: the original does it, this code did not. The next window
     * starts with its own reset and its own fill; leaving them on makes
     * the fill work against an active refresh.
     */
    scan_en = 0;
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SCAN_EN,
                   (uint16_t *)&scan_en, 2, thread_id);
    crossbar_mc_set_asic(thread_id, asic, 0);
}

/*
 * THE WHOLE FALLBACK: twenty-four windows of 170 banks (the original's
 * outer loop: uVar24 += 0xaa while uVar24 != 0xff0, i.e. up to 4080),
 * each with its fill, its configuration, its arming and a one-second wait.
 * About thirty seconds per chip: a RECOVERY path, not the common case.
 *
 * THE SEMANTICS ARE INVERTED: in normal mode we start from zero and
 * ACCUMULATE what succeeded (OR); here the area is pre-filled with 0xffff
 * and what fails is REMOVED (AND). Each window can only invalidate banks,
 * never validate them.
 */
int cb_fallback_120x170(int thread_id, unsigned asic)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint32_t base;
    unsigned window = 0;

    jas_log("board %d asic %u: 120x170 fallback, %u windows of %u banks",
            thread_id, asic, N_BANK / CB_FALLBACK_SEG, CB_FALLBACK_SEG);

    /* AND semantics: start from "all good" and remove. */
    memset(jas_cb_sc_result[thread_id][asic], 0xff,
           sizeof(jas_cb_sc_result[thread_id][asic]));

    for (base = 0; base != N_BANK; base += CB_FALLBACK_SEG, window++) {
        uint32_t sram[16];
        uint32_t addr = base, data = FILL_PATTERN + base;
        uint32_t large = ((uint32_t)CB_FALLBACK_STEP_HI << 16) | CB_FALLBACK_STEP_LO;
        unsigned n;

        /* Soft reset, then memory controllers on, as the original does before
         * each window. */
        sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0, thread_id);
        sl_delay(0, 2000);
        sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, thread_id);
        crossbar_mc_set_asic(thread_id, asic, 1);

        /* 680 writes: +1 inside the window, large-step jump every 170. */
        for (n = 0; n < CB_FALLBACK_COUNT; n++) {
            unsigned k;

            for (k = 0; k < 16; k++)
                sram[k] = data;
            single_manual_write(asic_en, addr * 2,     (uint16_t *)sram, thread_id);
            single_manual_write(asic_en, addr * 2 + 1, (uint16_t *)sram, thread_id);
            if ((n + 1) % CB_FALLBACK_SEG == 0) { addr += large; data += large; }
            else                             { addr += 1;     data += 1;     }
        }

        cb_config_fallback(thread_id, asic, base);
        crossbar_run_fallback(thread_id, asic, base);

        if (window % 6 == 0)
            jas_log("board %d asic %u: fallback, window %u/%u",
                    thread_id, asic, window, N_BANK / CB_FALLBACK_SEG);
    }
    return 0;
}
