/*
 * Primitives of the built-in test block (DFT).
 *
 * The DFT is a state machine embedded in each mesh, driven through a window of
 * registers around 0xd00. You give it an identity, push commands to it, read
 * its results back, and you can burn its conclusions into a one-time
 * programmable memory — the OTP, which keeps from one boot to the next the row
 * repairs decided at the factory.
 *
 * The command convention is the one found everywhere: the target mesh takes
 * the high bits, 8 to write and 0xc to read.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"

/* The target mesh, with 8 to write and 0xc to read. */
#define DFT_W(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0x8u))
#define DFT_R(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0xcu))
#define ASIC(i)      ((uint8_t)(1u << (i)))

#define REG_ID          0xd00
#define REG_ID_MASK     0xd01
#define REG_CMD_CLEAR   0xd07
#define REG_CMD_DATA    0xd08
#define REG_CMD_STROBE  0xd2b
#define REG_READ_STROBE 0xd2c
#define REG_READ_DATA   0xd54
#define REG_OTP_STROBE  0xd5a
#define REG_OTP_DATA    0xd5b
#define REG_PREPOST     0xd6f
#define REG_DFT_RESET   0x40

/*
 * Reset of the DFT block of the whole board.
 *
 * The two one-millisecond delays are the original's: the line must stay low
 * long enough for the state machine to restart cleanly. Parameters i and mesh
 * are unused — the reset covers the whole board — but the signature carries
 * them.
 */
void dft_reset(int thread_id, uint16_t i, uint16_t mesh)
{
    (void)i; (void)mesh;

    sl_single_write(0xff, 0xf9, REG_DFT_RESET, 0, thread_id);
    sl_delay(0, 1000000);
    sl_single_write(0xff, 0xf9, REG_DFT_RESET, 0xffff, thread_id);
    sl_delay(0, 1000000);
}

/* Identity a mesh's state machine must answer to. */
void dft_set_id(int thread_id, uint16_t i, uint16_t mesh,
                uint16_t id_set, uint16_t id_mask_set)
{
    sl_single_write(ASIC(i), DFT_W(mesh), REG_ID,      id_set,      thread_id);
    sl_single_write(ASIC(i), DFT_W(mesh), REG_ID_MASK, id_mask_set, thread_id);
}

/* Choice of the pre/post mode — which stage of the data path is tested. */
void dft_set_pre_post_mode(int thread_id, uint16_t i, uint16_t mesh, uint16_t mode)
{
    sl_single_write(ASIC(i), DFT_W(mesh), REG_PREPOST, mode, thread_id);
}

/*
 * Pushes a 32-bit command into the state machine.
 * The pulse on the validation register must rise then fall: it is the edge
 * that triggers, not the level.
 */
void dft_write_single_cmd(int thread_id, uint16_t i, uint16_t mesh,
                          uint16_t *single_cmd_din)
{
    uint8_t a = ASIC(i), c = DFT_W(mesh);

    sl_single_write(a, c, REG_CMD_CLEAR, 0, thread_id);
    sl_multi_write(a, c, REG_CMD_DATA, single_cmd_din, 2, thread_id);
    sl_single_write(a, c, REG_CMD_STROBE, 1, thread_id);
    sl_single_write(a, c, REG_CMD_STROBE, 0, thread_id);
}

/* Reads back the state machine's answer. */
void dft_read_single_cmd(int thread_id, uint16_t i, uint16_t mesh,
                         uint16_t *ltc_single_read_data)
{
    uint8_t a = ASIC(i);

    sl_single_write(a, DFT_W(mesh), REG_READ_STROBE, 1, thread_id);
    sl_delay(0, 1000);
    sl_multi_read(a, DFT_R(mesh), REG_READ_DATA, ltc_single_read_data, 2,
                  1000000, thread_id);
    sl_single_write(a, DFT_W(mesh), REG_READ_STROBE, 0, thread_id);
}

/*
 * Burns a repair record into the OTP.
 *
 * Irreversible by nature: one-time programmable memory cannot be rewritten.
 * The original has no safeguard here either; the caution lies with the
 * caller.
 */
void dft_write_otp(int thread_id, uint16_t i, uint16_t mesh,
                   uint16_t *ap_repair_info)
{
    uint8_t a = ASIC(i), c = DFT_W(mesh);

    sl_single_write(a, c, REG_OTP_STROBE, 0, thread_id);
    sl_multi_write(a, c, REG_OTP_DATA, ap_repair_info, 2, thread_id);
    sl_single_write(a, c, REG_OTP_STROBE, 1, thread_id);
}

/* Widens sixteen row half-words into as many 32-bit words. */
void dft_conv_otp_row_data(uint16_t *ltc_row_data, uint32_t *ltc_otp_row_data)
{
    for (unsigned k = 0; k < 16; k++)
        ltc_otp_row_data[k] = (uint32_t)ltc_row_data[k];
}

/* BIST pattern length, register 0xd02. */
void dft_set_bist_l(int thread_id, uint16_t i, uint16_t mesh, uint16_t bist_l)
{
    sl_single_write(ASIC(i), DFT_W(mesh), 0xd02, bist_l, thread_id);
}

/* Access mode of the one-time programmable memory — register 0xd59. */
void dft_set_otp_mode(int thread_id, uint16_t i, uint16_t mesh, uint16_t mode)
{
    sl_single_write(ASIC(i), DFT_W(mesh), 0xd59, mode, thread_id);
}
