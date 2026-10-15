/*
 * Identity burnt into the chips.
 *
 * Each ASIC carries a serial number burnt at the factory into a fuse memory,
 * readable but not rewritable. It is the only stable identity of a chip: it
 * survives reboots, reflashes and board swaps.
 *
 * None of this plays any part in mining, but it lets a chip be followed over
 * time ("chip 1 of board 0" always points at the same silicon).
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <string.h>
#include <unistd.h>

#define REG_GLOBAL_UNLOCK  0x5002
#define REG_SOFT_RESET     0x830
#define REG_EFUSE_STROBE   0x8fe
#define REG_EFUSE_VALID    0x8ff
#define REG_EFUSE_DATA     0x900
#define EFUSE_WORDS        0x100
#define EFUSE_RETRIES      7

#define CMD_MESH0        0x18
#define CMD_MESH0_READ   0x1c
#define CMD_BROADCAST    0xf8
#define CMD_BOARD        0xf9

/*
 * Reads the 256 words of a chip's fuse memory.
 *
 * The read needs a reset first, then up to seven seconds of waiting: the fuse
 * block has its own sequencer, slower than the serial link. If the valid flag
 * never rises, data stays as it was — hence the memset in the caller.
 */
void efuse_read(uint8_t asic_en, uint16_t *data, int thread_id)
{
    uint16_t valid = 0;
    int tries;

    sl_single_write(0xff, CMD_BOARD, REG_GLOBAL_UNLOCK, 0, thread_id);
    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0, thread_id);
    sl_delay(0, 2000);
    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, thread_id);

    sl_single_write(asic_en, CMD_MESH0, REG_EFUSE_STROBE, 1, thread_id);
    for (tries = EFUSE_RETRIES; tries > 0; tries--) {
        sl_single_read(asic_en, CMD_MESH0_READ, REG_EFUSE_VALID, &valid,
                       1000000, thread_id);
        if (valid) {
            sl_multi_read(asic_en, CMD_MESH0_READ, REG_EFUSE_DATA, data,
                          EFUSE_WORDS, 1000000, thread_id);
            break;
        }
        sleep(1);
    }
    sl_single_write(asic_en, CMD_MESH0, REG_EFUSE_STROBE, 0, thread_id);
}

/*
 * Reads the serial number of every active chip of every board.
 *
 * A leading null byte means a chip whose fuses were never burnt: it happens,
 * and it is not a defect.
 */
void sl_get_asic_id(void)
{
    int board;

    for (board = 0; board < BOARD_NUM; board++) {
        unsigned asic;

        if (!(BOARD_EN >> board & 1))
            continue;

        for (asic = 0; asic < ASIC_NUM; asic++) {
            uint16_t raw[EFUSE_WORDS];
            char *id = jas_asic_id(board, asic);

            if (!(ASIC_EN(board) >> asic & 1))
                continue;

            memset(raw, 0, sizeof(raw));
            efuse_read((uint8_t)(1u << asic), raw, board);

            if ((raw[0] & 0xff) == 0) {
                jas_log("board %d asic %u: blank fuses, no serial number",
                        board, asic);
                if (id) id[0] = '\0';
                continue;
            }

            if (id) {
                memcpy(id, raw, JAS_ASIC_ID_LEN - 1);
                id[JAS_ASIC_ID_LEN - 1] = '\0';
                jas_log("board %d asic %u: serial number %s", board, asic, id);
            }
        }
    }
}
