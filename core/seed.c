/*
 * Preparing the state of the banks: the fragment of seed_update_operation
 * @ 0x25f00 that initializes it.
 *
 * It must run BEFORE self_check: config_remap_info writes a table entry for
 * EACH of the 4080 banks, and a flag at zero becomes a "hole" entry pointing
 * to 0xfff, outside the addressable space. With all flags at zero the table
 * maps nothing, for reading as for writing (patent CN114003526B: the write
 * path goes through the same translation), and the crossbar scan returns
 * only timeouts. The firmware marks ALL banks good (12240 entries for a
 * board) before any test has run: an initialization, not a verdict.
 *
 * The rest of seed_update_operation (seed, cache, DAG) belongs to the mining
 * path, not here.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include <stdio.h>

/*
 * The original wraps this in `if (dev_info_g.seed_switch < 2)`, then loops
 * over the enabled boards. It also initializes alu_en_g on the way — left
 * aside as long as we do not touch the mining path.
 *
 * The line "board %d init params" is the firmware's, word for word. It does
 * not appear in its flash log: it is a plain printf, without applog_driver —
 * like the probing lines of board_check.
 */
/* Startup attempts before giving up on a chip (3 is enough). */
#define STARTUP_RETRIES 3u

void seed_init_bank_params(void)
{
    unsigned board, k;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;

        printf("board %u init params\n", board);
        for (k = 0; k < ASIC_NUM * 0xff0u; k++) {
            jas_bank_good[board][k]    = 1;
            jas_bank_bad[board][k]     = 0;
            jas_bank_partial[board][k] = 1;
        }
    }
}

/* ======================================================================
 * seed_update_operation @ 0x25e9c: the orchestrator of the startup.
 *
 * It prepares the state of the banks (seed_init_bank_params, above), runs
 * the three self-tests in parallel, waits, calls dag_calc, hands out the
 * first work, reopens the ALUs and publishes the sorting summary.
 *
 * The initialization covers more than the banks: alu_en_g is reset to
 * 0xffff (all ALUs declared good), and so are the three memory masks
 * mc_en_g, mc_refresh_en_g and mc_rf_moniter_en_g.
 *
 * The registers of the initial work, in order (register 0x20 does carry
 * the HEADER: this function writes fpga_data + 0x40 there):
 *
 *     reg 0x08   DAG size           fpga_data + 0x10, 4 words
 *     reg 0x10   target             REVERSED bytes, 16 words
 *     reg 0x20   header             fpga_data + 0x40, 16 words
 *     reg 0x30   seed               fpga_data + 0x60, 16 words
 *     reg 0x0c   starting nonce     per ALU, one mesh at a time
 *
 * THE EXTRANONCE SHIFT: 48 bits if it fits in sixteen, 40 otherwise. What
 * remains below that shift is the space the hardware sweeps.
 * ====================================================================== */

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "sl_transfer.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include "dft.h"      /* self_check_func */
#include "jas_types.h"
#include <sys/time.h>

#define BANKS_PER_CHIP  0xff0u
#define NONCES_PER_CHIP   0x78u    /* 120 ALU */
#define NONCES_PER_MESH   0x1eu    /* 30 per mesh */
#define MESHES            4
#define ALUS_PER_CHIP     120

#define REG_DAG_SIZE      0x0008
#define REG_NONCE         0x000c
#define REG_TARGET        0x0010
#define REG_HEADER        0x0020
#define REG_SEED          0x0030
#define REG_ALU_EN        0x0040
#define REG_MC_REFRESH    0x0c40
#define REG_MC_STANDBY  0x0c00
#define REG_SOFT_RESET    0x0830
#define CMD_DIFFUSION     0xf8
#define CMD_MESH_W(m)     ((uint8_t)((0x10u << (m)) | 0x8u))
#define CMD_MESH_R(m)     ((uint8_t)((0x10u << (m)) | 0xcu))

/*
 * The three self-test threads, one per board. The original has three
 * distinct functions — self_check_thread_0/1/2 — that differ only by the
 * board number they pass to self_check_func.
 */
static pthread_t jas_self_check_tid[3];

static void *self_check_thread_0(void *nothing) { (void)nothing; self_check_func(0); return NULL; }
static void *self_check_thread_1(void *nothing) { (void)nothing; self_check_func(1); return NULL; }
static void *self_check_thread_2(void *nothing) { (void)nothing; self_check_func(2); return NULL; }

static void *(*const jas_self_check_fn[3])(void *) = {
    self_check_thread_0, self_check_thread_1, self_check_thread_2,
};

/*
 * Full preparation: banks, ALUs and memory masks. This is the block the
 * original wraps in `if (dev_info_g.seed_switch < 2)`.
 */
/* Experimentation hook, set by the stress bench; NULL in production.
 * Called in seed_update_operation between the CBSC scan and dag_calc. */
void (*jas_veto_hook)(void);

void prepare_state(void)   /* exported: the stress bench calls it before self_check (scrub/cbsc) */
{
    unsigned board, chip, quarter, k;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;

        printf("board %u init params\n", board);

        for (k = 0; k < ASIC_NUM * BANKS_PER_CHIP; k++) {
            jas_bank_good[board][k]    = 1;
            jas_bank_bad[board][k]     = 0;
            jas_bank_partial[board][k] = 1;
        }

        /*
         * THE DFT COUNTERS TOO: they would ADD UP from one startup attempt to
         * the next ("7864 good banks" on a chip that has 4080). No decision
         * depends on them (the DAG geometry goes through jas_bank_partial,
         * reset above), but the log must not show impossible figures.
         */
        for (chip = 0; chip < ASIC_NUM; chip++) {
            jas_good_bank_num_pre[board][chip]  = 0;
            jas_good_bank_num_post[board][chip] = 0;
            jas_dp_fail_num[board][chip]        = 0;
            jas_hb_fail_num[board][chip]        = 0;
            jas_good_alu_num_dft[board][chip]   = 0;
        }

        /* All ALUs declared good: 4 meshes x 2 sixteen-bit words. */
        for (chip = 0; chip < ASIC_NUM; chip++)
            for (quarter = 0; quarter < MESHES; quarter++) {
                jas_alu_en[board][chip][quarter][0] = 0xffff;
                jas_alu_en[board][chip][quarter][1] = 0xffff;
            }

        /* And the three memory masks, 4 quarters of 64 words per chip. */
        for (chip = 0; chip < ASIC_NUM; chip++)
            for (quarter = 0; quarter < MESHES; quarter++)
                for (k = 0; k < 64; k++) {
                    jas_mc_en[board][chip][quarter][k]         = 0xffff;
                    jas_mc_refresh_en[board][chip][quarter][k] = 0xffff;
                    jas_mc_monitor_en[board][chip][quarter][k] = 0xffff;
                }
    }
}

/* The three self-tests in parallel, then the wait. An absent board has no
 * thread: board_en decides. */
static void start_auto_tests(void)
{
    unsigned board;

    for (board = 0; board < BOARD_NUM && board < 3; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        if (pthread_create(&jas_self_check_tid[board], NULL,
                           jas_self_check_fn[board], NULL) != 0)
            jas_log("fail to pthread_create self_check_thread_%u", board);
    }
    for (board = 0; board < BOARD_NUM && board < 3; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        pthread_join(jas_self_check_tid[board], NULL);
    }
    jas_log("end self check");
}

/* How many ALUs survived, per chip: 30 bits per mesh, spread over two
 * sixteen-bit words. */
static unsigned count_alu(unsigned board, unsigned chip)
{
    unsigned quarter, bit, n = 0;

    for (quarter = 0; quarter < MESHES; quarter++)
        for (bit = 0; bit < 30; bit++) {
            uint16_t word = jas_alu_en[board][chip][quarter][bit >> 4];
            if (word >> (bit & 0xf) & 1)
                n++;
        }
    return n;
}

/*
 * Recompute board_en from asic_en: a board on which no chip answers any
 * more leaves the game. The original updates dev_info_g on the way — number
 * of active boards, mask, and number of chips per board.
 */
static void recompute_boards(void)
{
    uint16_t mask = BOARD_EN;
    unsigned board, n = 0;
    int change = 0;

    for (board = 0; board < BOARD_NUM; board++)
        if (ASIC_EN(board) == 0) {
            mask &= (uint16_t)~(1u << board);
            change = 1;
        }
    if (change)
        BOARD_EN = mask;

    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            n++;

    for (board = 0; board < BOARD_NUM && board < 3; board++) {
        unsigned chip, p = 0;

        jas_board_asic_num[board] = 0;
        if (BOARD_EN >> board & 1)
            for (chip = 0; chip < ASIC_NUM; chip++)
                if (ASIC_EN(board) >> chip & 1)
                    p++;
        jas_board_asic_num[board] = (uint8_t)p;
        jas_log("board %u dev_info_g.board_asic_num = %u", board, p);
    }
}

/*
 * The first work sent to a board: sizes, target, header, seed, then one
 * nonce range per ALU.
 *
 * The original takes these four values from fpga_data, BROADCAST to all the
 * chips — the last trace of the "single coin" assumption. Our version takes
 * them from each chip's context, as header_update_operation already does
 * (core/work.c). The nonce distribution was already per chip on its side:
 * nothing to change.
 */
static void send_initial_work(unsigned board, uint64_t extranonce, unsigned offset)
{
    /* The chips waiting for their DAG end (per-chip startup) must suffer
     * neither the cut, nor the programming, nor above all the soft reset. */
    uint8_t asic_en = (uint8_t)(ASIC_EN(board) & ~jas_dag_deferred((int)board)
                                & ~jas_dag_discarded((int)board));
    unsigned chip, mesh;
    uint16_t zero[2] = { 0, 0 };

    /* Cut the ALUs before reprogramming: a chip computing while its header
     * is being changed returns results on the old one. */
    sl_multi_write(asic_en, CMD_DIFFUSION, REG_ALU_EN, zero, 2, (int)board);

    for (chip = 0; chip < ASIC_NUM; chip++) {
        const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
        uint8_t mask = (uint8_t)(1u << chip);
        uint8_t target_wire[32];
        uint64_t nonce;
        int k;

        if (!(asic_en >> chip & 1))
            continue;

        sl_multi_write(mask, CMD_DIFFUSION, REG_DAG_SIZE,
                       (uint16_t *)&coin->dag_size, 4, (int)board);
        sl_multi_write(mask, CMD_DIFFUSION, REG_HEADER,
                       (uint16_t *)&coin->header, 16, (int)board);
        sl_multi_write(mask, CMD_DIFFUSION, REG_SEED,
                       (uint16_t *)&coin->seed, 16, (int)board);

        /* The target goes out with its bytes reversed — the hardware expects it so. */
        for (k = 0; k < 32; k++)
            target_wire[k] = coin->target.b[31 - k];
        sl_multi_write(mask, CMD_DIFFUSION, REG_TARGET,
                       (uint16_t *)target_wire, 16, (int)board);

        nonce = ((uint64_t)extranonce << offset)
              + (uint64_t)board * ASIC_NUM * NONCES_PER_CHIP
              + (uint64_t)chip * NONCES_PER_CHIP;

        for (mesh = 0; mesh < MESHES; mesh++) {
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_NONCE,
                           (uint16_t *)&nonce, 4, (int)board);
            nonce += NONCES_PER_MESH;
        }
    }

    printf("board %u send task done\n", board);

    /* Reset, then release: that is what validates the new job. */
    sl_single_write(asic_en, CMD_DIFFUSION, REG_SOFT_RESET, 0, (int)board);
    sl_delay(0, 2000);
    sl_single_write(asic_en, CMD_DIFFUSION, REG_SOFT_RESET, 0xffff, (int)board);
}

/*
 * Close the memory refresh, then reopen the ALUs.
 *
 * The order matters and it is not intuitive: the refresh is cut BEFORE the
 * ALUs restart. Once the DAG is written, the memory no longer needs to be
 * refreshed — it is read all the time by the mining itself — and the
 * refresh would steal bandwidth.
 *
 * The half-second wait per chip is the original's.
 */
static void close_refresh_and_reopen_alu(unsigned board)
{
    uint8_t asic_en = (uint8_t)(ASIC_EN(board) & ~jas_dag_deferred((int)board)
                                & ~jas_dag_discarded((int)board));
    uint16_t zeros[64];
    unsigned chip, mesh;

    memset(zeros, 0, sizeof(zeros));

    for (chip = 0; chip < ASIC_NUM; chip++) {
        uint8_t mask = (uint8_t)(1u << chip);

        if (!(asic_en >> chip & 1))
            continue;

        sl_multi_write(mask, CMD_DIFFUSION, REG_MC_REFRESH, zeros, 64, (int)board);
        jas_log("board[%u] asic[%u] close mc refresh done", board, chip);

        for (mesh = 0; mesh < MESHES; mesh++) {
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_ALU_EN,
                           jas_alu_en[board][chip][mesh], 2, (int)board);
            printf("alu_en_g[%u][%u][%u][0] = %x\n", board, chip, mesh,
                   jas_alu_en[board][chip][mesh][0]);
            printf("alu_en_g[%u][%u][%u][1] = %x\n", board, chip, mesh,
                   jas_alu_en[board][chip][mesh][1]);
        }
        usleep(500000);
    }
    jas_log("board %u alu enable done", board);
}

/*
 * The measured yield: the sum of the 120 per-ALU values get_eff filled in.
 * The original also stores it in dev_info_g for the web page.
 */
static void efficiency_summary(void)
{
    unsigned board, chip, alu;
    float total = 0.0f;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            float eff = 0.0f;

            for (alu = 0; alu < ALUS_PER_CHIP; alu++)
                eff += jas_eff_real_value[board][chip][alu];
            jas_log("board %u asic %u eff is : %f", board, chip, (double)eff);
            total += eff;
        }
    }
    jas_log("all_eff is : %f", (double)total);
}

/*
 * The sorting summary, published when seed_switch is exactly 1 — the
 * vendor's "screening" mode. Each line is an acceptance criterion, and the
 * whole is the best health portrait of a board the firmware can produce. It
 * is for us that it is worth reimplementing.
 */
static void screening_summary(void)
{
    unsigned board, chip;

    if (*jas_seed_switch != 1)
        return;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            unsigned k = board * 3 + chip;
            /* screening_status : device_screening_result + board*0x260
             *                    + chip*0xb0 + 0x8a */
            uint8_t state = jas_screening_status[board * 0x260 + chip * 0xb0 + 0x8a];
            int pll_ok   = (jas_pll_lock[board] >> chip & 1) != 0;
            int spi_ok   = (*jas_spi_status)[board] >> chip & 1;

            jas_log("board[%u] asic[%u] screening_status:%u", board, chip, state);
            jas_log("board[%u] asic[%u] pll:%s", board, chip, pll_ok ? "pass" : "fail");
            jas_log("board[%u] asic[%u] pll_clk_test:%.2fMHz", board, chip,
                    jas_pll_clk_test[board][chip] * 80000.0 / 1000000.0);
            jas_log("board[%u] asic[%u] spi:%s", board, chip, spi_ok ? "pass" : "fail");
            jas_log("board[%u] asic[%u] good_alu_num:%d", board, chip, jas_good_alu_num_dft[board][chip]);
            jas_log("board[%u] asic[%u] hb_fail_num:%d", board, chip, jas_hb_fail_num[board][chip]);
            jas_log("board[%u] asic[%u] good_bank_num_pre:%d", board, chip,
                    jas_good_bank_num_pre[board][chip]);
            jas_log("board[%u] asic[%u] good_bank_num_post:%d", board, chip,
                    jas_good_bank_num_post[board][chip]);
            jas_log("board[%u] asic[%u] good_bank_num_cbsc:%d", board, chip,
                    jas_good_bank_num_cbsc[k]);
            jas_log("board[%u] asic[%u] wtf:%lld", board, chip,
                    (long long)jas_asic_max_capacity[k]);
        }
    }
}

void seed_update_operation(void)
{
    uint64_t extranonce;
    unsigned offset, board, attempt;
    uint16_t zero[2] = { 0, 0 };

    /*
     * The extranonce comes from the pool; failing that, the machine's
     * identifier serves as seed — otherwise two miners on the same pool
     * would not sweep distinct ranges.
     */
    extranonce = *jas_extranonce;
    if (extranonce == 0)
        extranonce = *jas_dev_id;
    /* jas_dev_id is never written either: the seed comes from the serial
     * number, otherwise the whole fleet shares the same range. */
    if (extranonce == 0)
        extranonce = jas_identity_nonce();
    offset = (extranonce >> 16) ? 40 : 48;

    /*
     * RETRY THE STARTUP WHEN A DAG FAILS.
     *
     * The CBSC is a LOTTERY: its condemnations change from one startup to the
     * next on identical hardware (the same chip can lose 700 banks to a single
     * ALU, fall under the floor and fail its DAG, then lose none at the next
     * startup). dag_calc's own retries do NOT redo the scan and replay the
     * same draw, so the startup restarts from prepare_state + the self-test,
     * which resets everything (banks, ALUs, memory masks): each attempt starts
     * from a blank page. Without it, a chip stays dead until the next restart,
     * silently.
     *
     * THE POOL WATCHDOG restarts the miner when "works" stagnates (12 x 30 s
     * beyond 300 s of grace, ~11 min) while a startup already takes ~9: it
     * must stay in standby while dag_time < 100 (jasminer.sh), or a second
     * attempt would be cut in mid-flight.
     */
    for (attempt = 1; ; attempt++) {
        if (*jas_seed_switch < 2) {
            prepare_state();
            start_auto_tests();

            for (board = 0; board < BOARD_NUM; board++) {
                unsigned chip;

                if (!(BOARD_EN >> board & 1))
                    continue;
                for (chip = 0; chip < ASIC_NUM; chip++)
                    if (ASIC_EN(board) >> chip & 1)
                        jas_log("board %u asic%u alu good num %u",
                                board, chip, count_alu(board, chip));
            }
        }

        /*
         * THE CONDITION REALLY IS ">= 2": the decompiled code says
         * `if (seed_switch < 2) goto LAB_0002639e;`, LAB_0002639e being the
         * "start calc dag" that FOLLOWS this loop. So in the normal case
         * (seed_switch < 2) the ALUs are NOT cut: they are what builds the
         * cache, and cutting them makes every cache build time out.
         */
        if (*jas_seed_switch >= 2)
            for (board = 0; board < BOARD_NUM; board++)
                if (BOARD_EN >> board & 1)
                    sl_multi_write(ASIC_EN(board), CMD_DIFFUSION, REG_ALU_EN,
                                   zero, 2, (int)board);

        jas_dump_map();   /* read-only diagnostic, gated by JASRE_MAP_DUMP */

        /* Experimentation hook (stress bench): called AFTER the CBSC scan and
         * BEFORE dag_calc, the only window in which to modify the bank map
         * (jas_bank_partial/good) before the DAG is built. NULL in production. */
        if (jas_veto_hook)
            jas_veto_hook();

        jas_log("start calc dag");
        jas_dag_step = 3;
        dag_calc();
        jas_log("dag calc done");
        fflush(stdout);

        if (dag_calc_failures() == 0)
            break;
        if (*jas_seed_switch >= 2) {
            jas_log("startup: DAG failed, but the self-test is skipped"
                    " (seed_switch >= 2) — nothing to replay");
            break;
        }
        if (attempt >= STARTUP_RETRIES) {
            jas_log("startup: %u chip(s) still without a DAG after %u attempts"
                    " — carrying on without them",
                    dag_calc_failures(), attempt);
            break;
        }
        jas_log("startup: %u chip(s) without a DAG — replaying the full"
                " self-test (attempt %u/%u), the CBSC draw changes from one"
                " startup to the next",
                dag_calc_failures(), attempt + 1u, STARTUP_RETRIES);
        /* The bar starts again: this is how the pool watchdog recognizes a
         * machine that is still working. */
        jas_web_dag_publish(0);
    }

    /*
     * AND IT GOES BACK UP TO 100 ON THE WAY OUT, EVEN IF EVERYTHING FAILED.
     *
     * Otherwise a completely failed startup would leave the bar below 100
     * for the life of the process, the watchdog would stay suspended
     * forever, and a machine without any DAG would never be restarted —
     * exactly the failure this loop is meant to remove.
     */
    jas_web_dag_publish(100);

    /*
     * The thread that will rebuild the DAG in place at epoch changes. Started
     * HERE, once the startup is finished: before, it would have nothing to
     * do, and it must above all not run while dag_calc() is already at work.
     * See core/dag_calc.c, dag_rebuild_thread.
     */
    jas_dag_rebuild_start();

    recompute_boards();

    /*
     * REFRESH THE HEADER BEFORE STARTING THE CHIPS: each chip's context
     * still carries the header captured BEFORE the DAG build, minutes ago;
     * the pool keeps only a few works (~36 s) and would refuse the first
     * share found on it.
     *
     * jas_work_of_chip honours each chip's pool group, so dual mining
     * stays correct. The header is only taken if it is of the SAME EPOCH as
     * the DAG just built; otherwise nothing is touched and the mining loop
     * will see the change and rebuild cleanly.
     */
    for (board = 0; board < BOARD_NUM; board++) {
        unsigned chip;

        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            jas_coin_ctx *c = jas_coin_of((int)board, chip);
            jas_work tp;

            if (!(ASIC_EN(board) >> chip & 1) || !c)
                continue;
            if (!jas_work_of_chip(chip, &tp))
                continue;                       /* nothing fresh at hand */
            if (c->block_number / 30000ull != tp.block_number / 30000ull)
                continue;                       /* other epoch: do not touch */
            if (memcmp(c->header.b, tp.header.b, 32) == 0)
                continue;                       /* already up to date */
            c->header = tp.header;
            c->target = tp.target;
            snprintf(c->work_id, sizeof(c->work_id), "%s", tp.work_id);
            jas_log("board %u asic %u: header refreshed before start",
                    board, chip);
        }
    }

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        send_initial_work(board, extranonce, offset);
        close_refresh_and_reopen_alu(board);
        jas_fan_mining(1);          /* the ALUs are mining: normal cooling */
        fflush(stdout);
    }

    usleep(10000);

    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            get_eff((int)board, 1);

    efficiency_summary();

    /* Mining has started: the rebuild thread can finish the chips put on
     * hold by the per-chip startup. */
    jas_dag_start_done();
    screening_summary();
}


/* ====================================================================== *
 * STANDBY: stop hashing WITHOUT losing the DAG, to keep the machine powered
 * and resume mining in a few seconds instead of a full startup.
 *
 * The DAG holds at rest on one condition: restoring the memory refresh
 * (0xc40) and the bank enable (0xc00), which the startup CUTS before
 * opening the ALUs (while mining, the hashing reads refresh the memory by
 * themselves). ALUs off WITHOUT refresh, the DAG rots in 15 to 25 s.
 *
 * Resume: DAG check (ALUs off, refresh still active: the only state in
 * which the read-back is right), headers refreshed, then exactly the end
 * of the startup: work, closing the refresh, ALUs. A chip whose DAG did
 * not hold is handed to the rebuild thread (as for an epoch change).
 * Called by the mining loop (wrapper.c), the only one touching the bus.
 * ====================================================================== */
static void extranonce_current(uint64_t *x, unsigned *d)
{
    uint64_t e = *jas_extranonce;

    if (e == 0)
        e = *jas_dev_id;
    if (e == 0)
        e = jas_identity_nonce();
    *x = e;
    *d = (e >> 16) ? 40 : 48;
}

void jas_standby_enter(void)
{
    uint16_t zero[2] = { 0, 0 };
    unsigned board, chip, mesh;

    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            sl_multi_write(ASIC_EN(board), CMD_DIFFUSION, REG_ALU_EN, zero, 2, (int)board);
    usleep(100000);
    for (board = 0; board < BOARD_NUM; board++) {
        gooddata_detail d;
        unsigned guard = 0;

        if (!(BOARD_EN >> board & 1))
            continue;
        while (get_asic_result_operation((int)board, &d) && guard++ < 100000u)
            ;
        purge_results((int)board, ASIC_EN(board));
        for (chip = 0; chip < ASIC_NUM; chip++) {
            uint8_t mask = (uint8_t)(1u << chip);

            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            for (mesh = 0; mesh < MESHES; mesh++) {
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_STANDBY,
                               jas_mc_en[board][chip][mesh], 0x40, (int)board);
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_REFRESH,
                               jas_mc_refresh_en[board][chip][mesh], 0x40, (int)board);
            }
        }
    }
    jas_fan_mining(0);
    jas_log("STANDBY: ALUs off, memory refresh on — DAG kept");
}

void jas_standby_exit(void)
{
    struct timeval t0, t1;
    uint64_t extranonce;
    unsigned offset, board, chip;
    uint8_t failed[3] = { 0, 0, 0 };

    gettimeofday(&t0, NULL);
    /* 1. did the DAG hold? (ALUs off, refresh active) */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            if (dram_check_dag((int)board, (uint8_t)chip, 0) < 0) {
                failed[board] |= (uint8_t)(1u << chip);
                jas_log("RESUME: board %u asic %u — the DAG did NOT survive the standby",
                        board, chip);
            }
        }
    }
    /* 2. headers up to date (as at the end of the startup) */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            jas_coin_ctx *c = jas_coin_of((int)board, chip);
            jas_work tp;

            if (!(ASIC_EN(board) >> chip & 1) || !c || !jas_work_of_chip(chip, &tp))
                continue;
            if (c->block_number / 30000ull != tp.block_number / 30000ull)
                continue;                       /* other epoch: the loop will handle it */
            c->header = tp.header;
            c->target = tp.target;
            snprintf(c->work_id, sizeof(c->work_id), "%s", tp.work_id);
        }
    }
    /* 3. the end of the startup */
    extranonce_current(&extranonce, &offset);
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        send_initial_work(board, extranonce, offset);
        close_refresh_and_reopen_alu(board);
    }
    jas_fan_mining(1);
    usleep(10000);
    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            get_eff((int)board, 1);
    /* 4. chips whose DAG did not hold: rebuild */
    for (board = 0; board < BOARD_NUM; board++)
        if (failed[board])
            jas_dag_request_rebuild((int)board, failed[board]);
    gettimeofday(&t1, NULL);
    jas_log("mining RESUMED in %.1f s (DAG %s)",
            (double)(t1.tv_sec - t0.tv_sec) + (double)(t1.tv_usec - t0.tv_usec) / 1e6,
            (failed[0] | failed[1] | failed[2]) ? "partly lost, rebuild started" : "intact on all chips");
}
