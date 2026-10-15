/*
 * Top of the driver: initialization and work flow.
 *
 * sl_wrapper_init is the firmware's very first hardware code, before
 * find_borad_and_poweron. It reads the Zynq's logic version, which sets the
 * number of boards:
 *
 *     version >= 0x50  ->  BOARD_NUM_G = 3   "master chip use zynq7020-400"
 *     version <  0x50  ->  unchanged         "master chip use zynq7020-484"
 *
 * The window is base_zynq = 0x80007000, one page, mapped before any other.
 */

#include <signal.h>
#include "jas_hal.h"

extern volatile sig_atomic_t jas_stop_request;
#include "jas_state.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include "sl_transfer.h"
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#define BASE_ZYNQ   0x80007000ul
#define VERSION_3_BOARDS  0x50

void sl_wrapper_init(void)
{
    int fd;
    volatile uint32_t *zynq;
    uint32_t version;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        jas_log("zynq PL open(/dev/mem) failed!");
        exit(0);
    }

    zynq = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE_ZYNQ);
    if (zynq == MAP_FAILED) {
        jas_log("zynq PL mmap 0x%lx failed", BASE_ZYNQ);
        close(fd);
        exit(0);
    }
    jas_zynq_p = (void *)zynq;

    version = *zynq;
    if (version < VERSION_3_BOARDS) {
        jas_log("zynq PL version 0x%x, master chip use zynq7020-484", version);
    } else {
        BOARD_NUM = 3;
        jas_log("zynq PL version 0x%x, master chip use zynq7020-400", version);
    }

    /* The AXI windows of each board. jas_hal_init does all three at once
     * where the original calls sl_transfer_init(board) in a loop. */
    jas_hal_init();
    jas_log("axi init done");

    find_borad_and_poweron();
    jas_log("check boards and asics finish %04x", BOARD_EN);

    jas_log("dev_id is %llu", (unsigned long long)get_dev_id());
    jas_log("done");
}

/* ---------------------------------------------------------------------- */

/*
 * The original's pow_flow_operation (@ 0x2756c) switches the WHOLE machine
 * between three coins, each with its remembered seed, processor cache and
 * cache thread:
 *
 *     seed_ori_g      0x387ce0  ethw
 *     seed_ori_zil_g  0x387cd8  zil   cpu_cache_zil, sl_cpu_calc_cache_zil
 *     seed_ori_etc_g  0x387cd0  etc   cpu_cache_etc, sl_cpu_calc_cache
 *
 * triggered by a seed change detected on the first eight bytes only ("switch
 * minning mode to ethw / zil / etc" in its log). It is not transcribed: here
 * each chip follows the block and seed of ITS pool in sl_wrapper_work and
 * asks jas_cache_request for its own cache (one registry entry per
 * (block, seed) pair), which lets two coins be mined at once.
 */

/* ====================================================================== *
 * sl_wrapper_work @ 0x2e508: the mining loop.
 *
 * The backbone: the rest of the driver is called, directly or not, from
 * this loop. It runs in its own thread ("wrapperWork" in the original) and
 * only returns on an alarm.
 *
 * The original's sequence, in order:
 *
 *   1. the alarms (missing or excessive temperature, stopped fan, Ctrl-C):
 *      each cuts the ASICs' power through power_off_operation and ends
 *      mining. It is the only place in the firmware that turns running
 *      chips off.
 *   2. the pool's work; if there is none, wait three seconds ("no task
 *      from pool, for wait 3s").
 *   3. has the work changed? Header AND seed are compared (task_ori_header,
 *      task_ori_seed): a new seed means a new epoch (pow_flow_operation
 *      rebuilds the processor cache), a new header only needs
 *      header_update_operation.
 *   4. the harvest: for each board, get_asic_result_operation returns at
 *      most ONE nonce per pass; each nonce is judged again by
 *      ethash_nonce_check before being submitted.
 *   5. late results: a nonce arriving AFTER the header changed was computed
 *      on the old work; the firmware counts it apart ("get the result of
 *      asic, but headhash update, %d times"), not as a reject.
 *
 * OUR VERSION: header, target and epoch come from each chip's context
 * (jas_coin_of), not from fpga_data; an epoch change rebuilds that chip's
 * cache and DAG only; the harvest drains every slot and comes before the
 * work changes. The pool layer (core/pool.c) meets this loop at two points
 * only: jas_work_current and jas_submit.
 * ====================================================================== */

/* The current work, as the pool layer drops it off. */
jas_work jas_work_g;

/* The loop's counters, the ones the log publishes. */
static unsigned rounds;
static unsigned nonces_stale;
static unsigned submissions;
static unsigned rejects;
/*
 * AND THE SAME, PER CHIP: the local reject rate must be counted per chip
 * (as the original does: one line per chip), not with the machine's
 * globals labeled with whichever chip triggered the print.
 */
#define N_BOARDS_MAX 3
#define N_CHIPS_MAX  3
static unsigned submissions_chip[N_BOARDS_MAX][N_CHIPS_MAX];
static unsigned rejects_chip[N_BOARDS_MAX][N_CHIPS_MAX];
/* The local reject rate is printed only ONCE, after 1000 loop turns,
 * like the original's powon. See below. */
static int rate_local_display;
static unsigned nonces_seen;
static unsigned nonces_stale_proven;

/*
 * A chip's LOCAL reject rate, for the periodic display (surveillance.c):
 * nonces with a wrong mix, discarded by ethash_nonce_check BEFORE
 * submission. It is the real silicon health, distinct from the pool reject
 * (always ~0, since the local filter protects the pool); a chip with a dead
 * crossbar block shows it here. Cumulative since startup. Returns the %,
 * sets *rej / *total (nonces judged) if not null.
 */
double jas_reject_local_chip(unsigned board, unsigned asic,
                             unsigned *rej, unsigned *total)
{
    unsigned s, r, t;

    if (board >= N_BOARDS_MAX || asic >= N_CHIPS_MAX) {
        if (rej)   *rej = 0;
        if (total) *total = 0;
        return 0.0;
    }
    s = submissions_chip[board][asic];
    r = rejects_chip[board][asic];
    t = s + r;
    if (rej)   *rej = r;
    if (total) *total = t;
    return t ? 100.0 * (double)r / (double)t : 0.0;
}

/*
 * "THE CHIPS ARE READY": the flag that lets the thread start EARLY.
 *
 * As in the original (its mining thread starts at 21 s, its self-test ends
 * at 4 min 51), the thread starts BEFORE the bring-up, so the pool's work
 * is consumed continuously from the first second instead of piling up. It
 * must not touch the silicon until it is configured: main.c raises this
 * flag when seed_update_operation has finished; until then the loop only
 * keeps the pool up to date.
 */
static volatile int chips_ready;

void jas_chips_ready(void) { chips_ready = 1; }

/*
 * THE LAST HEADERS, from the most recent to the oldest.
 *
 * Used only to diagnose a reject: replaying the computation against them
 * tells whether the chip was right with a previous header (stale nonce) or
 * whether its memory diverges (corrupt DAG). Four are enough: the pool
 * pushes a job every six seconds, and a result twenty-four seconds late
 * would point to a different problem altogether.
 */
#define HEADERS_KEPT 4
static uint8_t headers_recent[HEADERS_KEPT][32];
static unsigned headers_n;

static void store_header(const uint8_t *h)
{
    unsigned k;

    for (k = HEADERS_KEPT - 1; k > 0; k--)
        memcpy(headers_recent[k], headers_recent[k - 1], 32);
    memcpy(headers_recent[0], h, 32);
    if (headers_n < HEADERS_KEPT)
        headers_n++;
}

/*
 * These counters ARE the mining instrumentation, and no other is needed:
 * on this bus, a measurement that takes the lock is not an observation, it
 * is a load (a periodic read of each ALU's register 0x43 starved the mining
 * thread until production stopped). These counters are kept by the loop
 * itself and cost nothing.
 */
unsigned jas_mining_rounds(void) { return rounds; }

void jas_mining_counters(unsigned *seen, unsigned *submitted, unsigned *rej,
                          unsigned *stale)
{
    if (seen)     *seen     = nonces_seen;
    if (submitted)    *submitted    = submissions;
    if (rej)     *rej     = rejects;
    if (stale) *stale = nonces_stale + nonces_stale_proven;
}

/* Pool layer contact points: weak fallbacks, overridden by core/pool.c. */
int  __attribute__((weak)) jas_work_current(jas_work *out) { (void)out; return 0; }
void __attribute__((weak)) jas_submit(const gooddata_detail *d) { (void)d; }

/*
 * The four alarms that cut mining: fields of dev_info_g, as read by the
 * original sl_wrapper_work:
 *
 *     +0x3458   missing temperature
 *     +0x345c   excessive temperature
 *     +0x3468   stopped fan
 *     +0x346c   Ctrl-C
 *
 * This loop only READS them. They are raised by core/surveillance.c, once
 * per second, with the vendor's delay counters (68 degrees over 2 rounds,
 * missing sensor or stopped fan over 4). jas_alarm_ctrl_c is never
 * raised: SIGINT and SIGTERM go through a clean path in main.c, which
 * turns the chips off and exits (during the bring-up, this loop exits
 * without touching them, see below).
 *
 * DO NOT INVENT THRESHOLDS HERE: the vendor's values come from its binary;
 * guessed ones would give a protection whose real behavior nobody knows.
 */


static int alarm_raised(void)
{
    if (!*jas_alarm_temp_missing && !*jas_alarm_temp_high
        && !*jas_alarm_fan && !*jas_alarm_ctrl_c)
        return 0;

    jas_log("temperature null alarm %d, temperature over alarm %d, "
            "ctrl_c_alarm %d, fan stop alarm %d, power off operation",
            *jas_alarm_temp_missing, *jas_alarm_temp_high,
            *jas_alarm_ctrl_c, *jas_alarm_fan);
    power_off_operation();
    return 1;
}

/* STANDBY: requested by the API (web.c), applied by the mining loop, the only
 * one that touches the bus while mining. See jas_standby_enter (seed.c). */
volatile int jas_standby_requested = 0;
volatile int jas_in_standby = 0;

void *sl_wrapper_work(void *arg)
{
    static uint8_t header_memo[32];
    struct timeval start;
    unsigned board;

    (void)arg;
    jas_name_thread("mining");
    jas_log("sl_miner thread_id : %d", 0);
    gettimeofday(&start, NULL);

    /* Turns of the mining loop: it is the original's counter (local_1160)
     * that triggers the reject measurement, not a nonce count (much
     * slower). ~1000 turns make a window of a few tens of seconds, like
     * the original powon's "calculate ... times". */
    unsigned mining_rounds = 0;

    for (;;) {
        jas_work t;

        pthread_testcancel();
        mining_rounds++;

        if (alarm_raised())
            break;

        if (jas_standby_requested != jas_in_standby) {
            if (jas_standby_requested)
                jas_standby_enter();
            else
                jas_standby_exit();
            jas_in_standby = jas_standby_requested;
        }
        if (jas_in_standby) {
            usleep(200000);             /* the pool keeps receiving its jobs */
            continue;
        }

        if (!jas_work_current(&t)) {
            jas_log("no task from pool, for wait 3s ");
            sleep(3);
            continue;
        }


        /*
         * Work changes are handled per chip, further down: a new seed (new
         * epoch) rebuilds that chip's processor cache and DAG; a new header
         * alone only needs reprogramming.
         */
        /*
         * Until the chips are configured, only keep the pool up to date: no
         * cache, no header pushed, no harvest.
         */
        if (!chips_ready) {
            /*
             * ANSWER THE SIGNAL DURING THE BRING-UP: the state loop that reads
             * the flag only starts after the whole bring-up, and a SIGTERM
             * ignored for minutes makes the miner look dead (and a second one
             * started next to it freezes the machine). This thread turns every
             * 20 ms from the first second. It exits WITHOUT turning the chips
             * off: the bring-up may be in the middle of the DAG fill, where
             * power_off_operation has already cost two machines. The next
             * startup picks the silicon up cleanly.
             */
            if (jas_stop_request) {
                jas_log("signal %d during startup — exiting without"
                        " touching the asics", (int)jas_stop_request);
                _exit(0);
            }
            usleep(20000);
            continue;
        }

        /*
         * THE HARVEST COMES BEFORE THE WORK CHANGE: results still in the slots
         * were computed with the header header_memo still holds; pushing the
         * new header first would purge them. Cash them first, change
         * afterwards: the purge then only finds what is really stale.
         */
        for (board = 0; board < BOARD_NUM; board++) {
            gooddata_detail detail;

            unsigned taken = 0;

            if (!(BOARD_EN >> board & 1))
                continue;

            /*
             * DRAIN COMPLETELY, not a single nonce per pass.
             *
             * get_asic_result_operation stops at the FIRST slot that
             * returns something; its own header notes that this starves
             * the following slots. By looping, a prolific chip no longer
             * deprives its neighbors. The bound of 32 keeps a register
             * stuck high from freezing the mining loop.
             */
            while (taken < 32 && get_asic_result_operation((int)board, &detail)) {
            taken++;
            nonces_seen++;
            if (jas_trace_active())
                jas_log("board %u asic %u fpga result nonce: %016llx",
                    board, detail.asic_id,
                    (unsigned long long)detail.good_data.nonce);

            /*
             * NO REJECT ON "the header changed": a result harvested here was
             * computed with header_memo, still the current header, and
             * ethash_nonce_check judges it against that header. Real staleness
             * (a result computed with a header ALREADY replaced) is detected by
             * ethash_nonce_stale, which replays the computation against the
             * last four headers.
             */
            /*
             * JUDGE WITH THE CHIP'S HEADER, not the machine's. In dual
             * mining two chips work on different headers and targets;
             * header_memo describes only one.
             */
            {
            const jas_coin_ctx *cc = jas_coin_of((int)board, detail.asic_id);
            const uint8_t *header = cc ? cc->header.b : header_memo;
            const uint8_t *target = cc ? cc->target.b : (const uint8_t *)jas_fpga_target;

            if (ethash_nonce_check(&detail, header, target)) {
                submissions++;
                if (board < N_BOARDS_MAX && detail.asic_id < N_CHIPS_MAX)
                    submissions_chip[board][detail.asic_id]++;
                jas_submit(&detail);
            } else {
                /*
                 * The mix does not match. BEFORE blaming the silicon,
                 * replay the computation against the previous headers: if
                 * one gives back exactly the chip's mix, the chip was right
                 * and it is our timing that lost the nonce.
                 */
                int rank = ethash_nonce_stale(&detail, headers_recent, headers_n);

                if (rank > 0) {
                    nonces_stale_proven++;
                    jas_log("board %u asic %u: STALE nonce, computed with"
                            " header -%d (%u in total) — the asic is healthy",
                            board, detail.asic_id, rank, nonces_stale_proven);
                } else {
                    rejects++;
                    if (board < N_BOARDS_MAX && detail.asic_id < N_CHIPS_MAX)
                        rejects_chip[board][detail.asic_id]++;
                }
            }

            }
            /*
             * THE LOCAL REJECT RATE, ONCE ONLY, like the original
             * (sl_wrapper_work @ 0x2e511), which prints the per-chip rate ONCE
             * when its loop counter reaches its threshold (2000 there), then
             * sets a flag (local_1160, run_stage=5, guarded by bVar3).
             *
             * Here: ONLY THE MEASUREMENT, no frequency/voltage adjustment.
             * After 1000 TURNS of the mining loop, per chip:
             *   local reject = rejects * 100 / (submissions + rejects)
             * where rejects are nonces with a wrong mix (silicon health),
             * NEITHER stale NOR pool refusals. The three lines are printed
             * ONCE.
             */
            if (!rate_local_display && mining_rounds >= 1000) {
                unsigned b, a;

                for (b = 0; b < N_BOARDS_MAX; b++) {
                    if (!(BOARD_EN >> b & 1))
                        continue;
                    for (a = 0; a < N_CHIPS_MAX; a++) {
                        unsigned t = submissions_chip[b][a] + rejects_chip[b][a];

                        if (!(ASIC_EN(b) >> a & 1))
                            continue;
                        jas_log("board[%u]asic[%u] wrapper_work local reject"
                                " rate:%.2f%%", b, a,
                                t ? 100.0 * rejects_chip[b][a] / (double)t : 0.0);
                    }
                }
                rate_local_display = 1;
            }
            }
        }


        /*
         * THE WORK, CHIP BY CHIP.
         *
         * Each chip depends on a pool (jas_pool_of_chip) and carries its own
         * context: block, seed, header, target. This is what lets chip 1 mine
         * ETHW while chips 0 and 2 mine ETC.
         *
         * The hardware already allowed it: header_update_operation writes the
         * DAG size, header and target with a per-chip mask.
         */
        for (board = 0; board < BOARD_NUM; board++) {
            unsigned chip;
            /*
             * A MASK, NOT A FLAG: only the chips that received a job get their
             * header rewritten and their results purged. With a per-board flag,
             * in dual-coin mode, the ETHW chip would suffer the ETC jobs.
             */
            uint8_t change = 0;

            if (!(BOARD_EN >> board & 1))
                continue;

            for (chip = 0; chip < ASIC_NUM; chip++) {
                jas_coin_ctx *c = jas_coin_of((int)board, chip);
                jas_work tp;

                if (!jas_work_of_chip(chip, &tp))
                    continue;

                /* Epoch changed: new context, new cache, DAG to rebuild. */
                if (c->block_number != tp.block_number ||
                    memcmp(c->seed.b, tp.seed.b, 32) != 0) {
                    /*
                     * LATE-EPOCH SAFEGUARD: a pool whose sync fell behind
                     * announces an old block again; going BACK an epoch would
                     * rebuild the cache, and a back-and-forth between two
                     * epochs would stack caches in RAM until an OOM kill. Keep
                     * the current context and ignore the stale work; the chip
                     * resumes as soon as a pool serves an epoch >= the current
                     * one. (block_number is aligned on the epoch:
                     * 24750000 = 825*30000.)
                     */
                    if (c->block_number != 0 &&
                        tp.block_number < c->block_number) {
                        static uint64_t already_reported;
                        if (tp.block_number != already_reported) {
                            jas_log("board %u asic %u: epoch %llu behind %llu"
                                    " (pool %d) — stale work ignored",
                                    board, chip,
                                    (unsigned long long)(tp.block_number / 30000),
                                    (unsigned long long)(c->block_number / 30000),
                                    jas_pool_of_chip(chip));
                            already_reported = tp.block_number;
                        }
                        continue;       /* no rebuild, no push: keep current */
                    }
                    jas_log("board %u asic %u: epoch %llu -> %llu (pool %d)",
                            board, chip,
                            (unsigned long long)(c->block_number / 30000),
                            (unsigned long long)(tp.block_number / 30000),
                            jas_pool_of_chip(chip));
                    jas_coin_from_block_seed(c, tp.block_number, &tp.seed);
                    jas_cache_request(tp.block_number, &tp.seed);
                    /*
                     * AND THE CHIP'S DAG: jas_cache_request only redoes the
                     * PROCESSOR cache, while the chip's memory keeps the old
                     * epoch's DAG (~100 % rejects). Ask for the rebuild; the
                     * dedicated thread of dag_calc.c does it, for THIS chip
                     * only and with ITS coin.
                     */
                    jas_dag_request_rebuild((int)board, (uint8_t)(1u << chip));
                    /*
                     * Epoch change: this chip's DAG size is about to change,
                     * and its DAG will be rebuilt. What work.c's memo believes
                     * it knows of its registers no longer holds.
                     */
                    jas_work_memo_forget((int)board, (uint8_t)(1u << chip));
                }

                if (memcmp(c->header.b, tp.header.b, 32) != 0) {
                    store_header(c->header.b);
                    c->header = tp.header;
                    c->target = tp.target;
                    snprintf(c->work_id, sizeof(c->work_id), "%s", tp.work_id);
                    change |= (uint8_t)(1u << chip);
                }
            }

            if (change) {
                header_update_operation((int)board, change);
                purge_results((int)board, change);
                /* header_memo follows chip 0, for the inherited traces. */
                {
                    const jas_coin_ctx *c0 = jas_coin_of((int)board, 0);
                    if (c0) {
                        memcpy(header_memo, c0->header.b, 32);
                        memcpy(jas_fpga_header, c0->header.b, 32);
                        memcpy(jas_fpga_target, c0->target.b, 32);
                    }
                }
                if (jas_trace_active())
                    jas_log("sl_wrapper_work, work pushed (board %u)", board);
            }
        }

        rounds++;
        usleep(1000);
    }

    {
        struct timeval end;
        gettimeofday(&end, NULL);
        jas_log("calculate %u times, taking time: %ds",
                rounds, (int)(end.tv_sec - start.tv_sec));
    }
    jas_log("sl_wrapper_work, exit !!!");
    pthread_exit(NULL);
    return NULL;
}
