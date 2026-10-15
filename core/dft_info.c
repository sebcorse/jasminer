/*
 * Readouts of the built-in test block.
 *
 * These functions question a mesh's DFT state machine about the state of its
 * memory: which rows were repaired, whether the data path is intact, whether
 * the sense amplifiers answer. They feed the good or bad bank verdict, and
 * therefore, in the end, the capacity announced to the chip by
 * config_remap_info.
 *
 * All follow the same mechanics: set the reference voltages, push a command,
 * read three answer words back, and judge.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include "dft.h"

#define DFT_W(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0x8u))
#define DFT_R(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0xcu))
#define ASIC(i)      ((uint8_t)(1u << (i)))

#define REG_ROW_REPAIR   0xd43
#define REG_FULL_INDEX   0xd32
#define REG_FULL_STROBE  0xd31
#define REG_FULL_DATA    0xd33
#define REG_ROW_COUNT    0xd53
#define REG_COL_COUNT    0xd54
#define REG_TRIM_0       0xd60
#define REG_TRIM_1       0xd61
#define REG_TRIM_2       0xd62
#define REG_TRIM_3       0xd63
#define REG_TRIM_APPLY   0xd6d

#define FULL_ROWS        0x810   /* 2064 rows read one by one */
#define FULL_WORDS       0x0e    /* 14 words per row */

/* The sixteen row repair words of a mesh, in one go. */
int dft_get_row_repair_info(int thread_id, uint16_t i, uint16_t mesh,
                            uint16_t *ltc_row_data)
{
    sl_multi_read(ASIC(i), DFT_R(mesh), REG_ROW_REPAIR, ltc_row_data, 0x10,
                  1000000, thread_id);
    return 0;
}

/*
 * The full readout: 2064 rows, fourteen words each, nearly 29,000 words read
 * one at a time. It is the most expensive read of the whole startup — each
 * word is a round trip on the serial link.
 */
int dft_get_full_repair_info(int thread_id, uint16_t i, uint16_t mesh,
                             uint16_t *ltc_full_data)
{
    uint8_t a = ASIC(i), cw = DFT_W(mesh), cr = DFT_R(mesh);

    for (unsigned row = 0; row < FULL_ROWS; row++) {
        sl_single_write(a, cw, REG_FULL_INDEX, (uint16_t)row, thread_id);
        sl_single_write(a, cw, REG_FULL_STROBE, 1, thread_id);
        sl_multi_read(a, cr, REG_FULL_DATA, ltc_full_data, FULL_WORDS,
                      1000000, thread_id);
        sl_single_write(a, cw, REG_FULL_STROBE, 0, thread_id);
        ltc_full_data += FULL_WORDS;
    }
    return 0;
}

/*
 * How many rows and columns had to be repaired, and what to conclude. The
 * thresholds come from the original:
 *
 *    3  no repair                                  — memory intact
 *    2  up to 16 rows                              — repairable without doubt
 *    1  more, but fewer than 8193 columns          — repairable
 *    0  fewer than 20001 columns                   — borderline
 *   -1  more                                       — out of reach
 */
/*
 * DIAGNOSTIC: the last column value read, exposed without changing the
 * function's signature. Tells whether banks classified "out of reach" (-1)
 * are so because of a real avalanche of defects or because of a SATURATED
 * counter (failed measurement); the difference is worth ~200 banks per chip.
 * 0 when the column read did not happen (rows <= 16).
 */
static __thread uint32_t last_col_count;   /* thread-local: DFT runs in parallel */

uint32_t dft_last_col_count(void) { return last_col_count; }

int dft_get_result_way(int thread_id, uint16_t i, uint16_t mesh,
                       uint32_t *ltc_row_count)
{
    uint8_t a = ASIC(i), c = DFT_R(mesh);
    uint16_t rows = 0;
    uint32_t cols = 0;

    sl_single_read(a, c, REG_ROW_COUNT, &rows, 1000000, thread_id);

    last_col_count = 0;
    *ltc_row_count = rows;
    if (rows == 0)
        return 3;
    if (rows <= 0x10)
        return 2;

    sl_multi_read(a, c, REG_COL_COUNT, (uint16_t *)&cols, 2, 1000000, thread_id);
    last_col_count = cols;
    if (cols <= 0x2000)
        return 1;
    if (cols <= 0x4e20)
        return 0;
    return -1;
}

/*
 * Sense amplifiers ("half buffer").
 * Three answer words, all zero if the read is healthy.
 */
int dft_read_HB_info(int thread_id, uint16_t i, uint16_t mesh,
                     uint16_t *lr_trim, uint16_t *single_cmd)
{
    uint32_t hb[3] = { 0, 0, 0 };

    dft_apply_trim(thread_id, i, mesh, lr_trim);
    dft_write_single_cmd(thread_id, i, mesh, single_cmd);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&hb[0]);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&hb[1]);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&hb[2]);

    return (hb[0] == 0 && hb[1] == 0 && hb[2] == 0) ? 0 : -1;
}

/*
 * Data path.
 * Here the expected value is the opposite: all bits set, except the last word
 * limited to one byte. A zero somewhere means a broken internal link.
 */
int dft_read_DP_info(int thread_id, uint16_t i, uint16_t mesh,
                     uint16_t *lr_trim, uint16_t *single_cmd1,
                     uint16_t *single_cmd2)
{
    uint32_t dp[3] = { 0, 0, 0 };

    dft_apply_trim(thread_id, i, mesh, lr_trim);
    dft_write_single_cmd(thread_id, i, mesh, single_cmd1);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&dp[0]);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&dp[1]);
    dft_read_single_cmd(thread_id, i, mesh, (uint16_t *)&dp[2]);
    dft_write_single_cmd(thread_id, i, mesh, single_cmd2);

    if (dp[0] == 0xffffffffu && dp[1] == 0xffffffffu && dp[2] == 0xffu)
        return 0;

    jas_log("board %d asic %u mesh %u: datapath %08x %08x %08x",
            thread_id, i, mesh, dp[0], dp[1], dp[2]);
    return -1;
}
