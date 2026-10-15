/*
 * Measured throughput and status readout.
 *
 * get_eff is the only throughput measurement the firmware makes at the
 * source: it reads each ALU's counters, waits, reads them again, and divides.
 * Everything the web page shows derives from it. It is also the function that
 * CONDEMNS an ALU whose throughput is absurd — see below, a discreet but
 * lasting side effect.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define ALUS_PER_CHIP   120u
#define ALUS_PER_MESH   30u
#define MESHES          4u
#define WORDS_PER_MESH   0x3c    /* 60 16-bit words = 30 32-bit counters */
#define REG_EFF         0x43
#define REG_ALU_EN      0x40
#define REG_PLL_CFG     0x13
#define REG_PLL_CLK     0x23
#define CMD_BOARD_R     0xfd
#define CMD_MESH_R(m)   ((uint8_t)((0x10u << (m)) | 0xcu))
#define CMD_MESH_W(m)   ((uint8_t)((0x10u << (m)) | 0x8u))

/*
 * The two readouts, before and after the wait. 120 32-bit counters per
 * chip — one per ALU. The firmware calls them eff_record_ori_one_g
 * (0x3e1f0c) and eff_record_ori_two_g (0x3f7960).
 */
static uint32_t reading_before[3][3][ALUS_PER_CHIP];
static uint32_t reading_after[3][3][ALUS_PER_CHIP];
static struct timeval t_before[3], t_after[3];

/*
 * The two constants of the computation, as they are:
 *
 *     eff = (delta / 360) / seconds * 1024
 *
 * 360 is var_nonce_interval (0x387dd0), 1024 a literal; neither is
 * explained. The order of magnitude checks out, though: a healthy chip runs
 * around 5 MH/s per ALU, the anomaly threshold is at 8 MH/s, and the result
 * lands in hashes per second.
 */
#define INTERVAL_NONCE  360u
#define FACTOR           1024.0
#define OUTLIER_THRESHOLD    8.0e6f

/* Read a chip's 120 counters, one mesh at a time. */
static void read_counters(int board, unsigned chip, uint32_t *dest)
{
    unsigned mesh;

    for (mesh = 0; mesh < MESHES; mesh++)
        sl_multi_read((uint8_t)(1u << chip), CMD_MESH_R(mesh), REG_EFF,
                      (uint16_t *)(dest + mesh * ALUS_PER_MESH),
                      WORDS_PER_MESH, 1000000, board);
}

static void sweep(int board, uint32_t dest[3][ALUS_PER_CHIP])
{
    unsigned chip;

    memset(dest, 0, sizeof(uint32_t) * 3 * ALUS_PER_CHIP);
    for (chip = 0; chip < ASIC_NUM; chip++)
        if (ASIC_EN(board) >> chip & 1)
            read_counters(board, chip, dest[chip]);
}

/*
 * Condemn an ALU whose throughput is aberrant.
 *
 * THIS SIDE EFFECT IS LASTING, and worth knowing: get_eff does not only write
 * a statistic, it turns the ALU off in alu_en_g AND in the chip. An ALU cut
 * here only comes back at the next startup — seed_update_operation resets
 * alu_en_g to 0xffff. A readout taken during a disturbance can thus cost ALUs
 * for the whole session.
 *
 * The original's message, "alu_en_g[%d][%d][%d][%d] &= %x", says exactly
 * what happens: a logical AND on a sixteen-bit word of the mask.
 */
/* Consecutive aberrant readouts, per ALU. See their use in get_eff:
 * we condemn only after three in a row, so as not to turn off a healthy
 * ALU on a measurement artefact. */
static uint8_t outliers[3][3][ALUS_PER_CHIP];

static void condemn_alu(int board, unsigned chip, unsigned alu)
{
    unsigned mesh = alu / ALUS_PER_MESH;
    unsigned rank = alu % ALUS_PER_MESH;
    unsigned word  = rank >> 4;
    unsigned bit  = rank & 0xf;

    jas_alu_en[board][chip][mesh][word] &= (uint16_t)~(1u << bit);
    printf("alu_en_g[%d][%u][%u][%u] &= %x\n", board, chip, mesh, word,
           jas_alu_en[board][chip][mesh][word]);
    sl_multi_write((uint8_t)(1u << chip), CMD_MESH_W(mesh), REG_ALU_EN,
                   jas_alu_en[board][chip][mesh], 2, board);
}

void get_eff(int thread_id, int delay)
{
    unsigned chip, alu;
    double seconds;

    if (thread_id < 0 || thread_id > 2)
        return;

    gettimeofday(&t_before[thread_id], NULL);
    sweep(thread_id, reading_before[thread_id]);

    sleep(delay);

    gettimeofday(&t_after[thread_id], NULL);
    sweep(thread_id, reading_after[thread_id]);

    seconds = (double)(t_after[thread_id].tv_sec - t_before[thread_id].tv_sec)
             + (double)(t_after[thread_id].tv_usec - t_before[thread_id].tv_usec) / 1e6;
    if (seconds <= 0.0) {
        jas_log("get_eff: zero duration, measurement abandoned (board %d)", thread_id);
        return;
    }

    memset(jas_eff_real_value[thread_id], 0,
           sizeof(float) * 3 * ALUS_PER_CHIP);

    for (chip = 0; chip < ASIC_NUM; chip++) {
        if (!(ASIC_EN(thread_id) >> chip & 1))
            continue;
        for (alu = 0; alu < ALUS_PER_CHIP; alu++) {
            uint32_t delta = reading_after[thread_id][chip][alu]
                           - reading_before[thread_id][chip][alu];

            jas_eff_real_value[thread_id][chip][alu] =
                (float)(((double)(delta / INTERVAL_NONCE) / seconds) * FACTOR);
        }
    }

    printf("board %d real eff value:\n", thread_id);
    for (chip = 0; chip < ASIC_NUM; chip++) {
        if (!(ASIC_EN(thread_id) >> chip & 1))
            continue;
        for (alu = 0; alu < ALUS_PER_CHIP; alu++) {
            float *v = &jas_eff_real_value[thread_id][chip][alu];

            if (*v > OUTLIER_THRESHOLD) {
                /*
                 * THREE ABERRANT READOUTS IN A ROW, NOT A SINGLE ONE:
                 * condemn_alu turns an ALU off FOR GOOD (only prepare_state,
                 * at the next startup, resets the mask to 0xffff), and the
                 * measurement is a counter delta over one second: a soft reset
                 * of the chip, a counter wrapping around or an unlucky
                 * scheduling makes the delta huge with nothing wrong in the
                 * silicon. A silicon defect shows up again at every readout,
                 * an artefact does not: counting, as the temperature alarms of
                 * surveillance.c do, separates the two without losing the
                 * stock protection.
                 */
                printf("board %d asic %u alu[%u] real_eff = %f, error! (%u/3) \n",
                       thread_id, chip, alu, (double)*v,
                       (unsigned)outliers[thread_id][chip][alu] + 1u);
                *v = 0.0f;
                if (++outliers[thread_id][chip][alu] >= 3)
                    condemn_alu(thread_id, chip, alu);
            } else {
                /* A normal readout clears the counter: only an UNBROKEN
                 * series of aberrations condemns. */
                outliers[thread_id][chip][alu] = 0;
            }
            printf("board %d asic %u real_eff[%u] = %f \n",
                   thread_id, chip, alu, (double)*v);
        }
    }
    putchar('\n');
}

/*
 * get_current_status @ 0x1e574 — status readout of a running machine.
 *
 * Nothing is written to the hardware: two reads per board, the PLL
 * configuration and its clock measurement, then the voltages and the
 * temperature. It is the safest readout one can take on a machine in
 * production, and for us the most useful.
 *
 * The real frequency is derived from pll_clk_test with the same formula as
 * power_on_operation — 20000 x 8 / 2, i.e. 160000 / 2 — divided by a
 * million. A chip at 510 MHz returns 6375.
 *
 * Rails 1 and 2 are only shown in PMBus mode 0x450; ours are in 0x500, where
 * only rail 0 is set.
 */
void get_current_status(void)
{
    unsigned board, chip;

    jas_log("get_current_status:");

    for (board = 0; board < BOARD_NUM; board++) {
        uint16_t pll_cfg[3] = { 0, 0, 0 };
        uint16_t pll_clk[3] = { 0, 0, 0 };

        if (!(BOARD_EN >> board & 1))
            continue;

        sl_multi_read(0xff, CMD_BOARD_R, REG_PLL_CFG, pll_cfg, 3, 2000000, (int)board);
        sl_multi_read(0xff, CMD_BOARD_R, REG_PLL_CLK, pll_clk, 3, 4000000, (int)board);

        for (chip = 0; chip < ASIC_NUM; chip++) {
            double freq = (double)((int)(pll_clk[chip] * 160000u) >> 1) / 1000000.0;

            jas_log("board[%u] asic[%u] pll_cfg = %x", board, chip, pll_cfg[chip]);
            jas_log("board[%u] asic[%u] real_freq = %.2f MHz", board, chip, freq);
            jas_log("board[%u] asic[%u] asic_vol_rail0 = %dmv", board, chip,
                    jas_asic_vol_rail(board, chip, 0));
            if (jas_pmbus_mode == 0x450) {
                jas_log("board[%u] asic[%u] asic_vol_rail1 = %dmv", board, chip,
                        jas_asic_vol_rail(board, chip, 1));
                jas_log("board[%u] asic[%u] asic_vol_rail2 = %dmv", board, chip,
                        jas_asic_vol_rail(board, chip, 2));
            }
            jas_log("board[%u] asic[%u] temp = %d", board, chip,
                    jas_asic_temp[board][chip]);
        }
    }
}
