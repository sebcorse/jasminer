/*
 * Daughter board discovery — board_check @ 0x1d834.
 *
 * The main board has THREE connectors; each connector takes a daughter board
 * of 3 ASICs. The X16-Q has only one, hence `board_en_g = 0x1`. The firmware
 * is written for three and probes which ones answer — this function.
 *
 * The probe: write 0x5e into register 0x30 of the connector, read it back,
 * and compare. If the echo does not come back, wake the connector up and try
 * again with 0xe5. Up to five rounds per connector.
 *
 * The read uses sl_single_read_soft, and that is essential: the hard version
 * resets the link when it fails, which would reset it at every empty
 * connector. See sl_transfer.c.
 *
 * REFERENCE — the firmware logs this step, and our output must match it line
 * by line:
 *
 *     board 0 mmap base address ok.           (x3)
 *      board 0 reset fpga hard reset done     (x3, same second)
 *     board 0 is exist
 *     board_en_g is 0x1
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define REG_PROBE 0x30

/*
 * find_borad_and_poweron @ 0x1e8c4: the backbone of the hardware startup.
 *
 * The order is the firmware's, read in the binary and confirmed line by line
 * by its trace in flash:
 *
 *     board_check                        discovers the populated connectors
 *     sensor_board_en_g = board_en_g
 *     sensor_init                        IIC bus + fans
 *     "sensor init finish"
 *     power_on_operation, per board      PLL, voltages, clocks
 *     sensor_thread_create               the temperature thread
 */
void find_borad_and_poweron(void)
{
    static int already_done;
    unsigned board;

    if (already_done)
        return;
    already_done = 1;

    board_check();
    jas_sensor_board_en = BOARD_EN;
    sensor_init();
    puts("sensor init finish");

    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            power_on_operation((int)board);

    /* sensor_thread_create(): the temperature reading thread. Not reimplemented
     * yet — it is not on the path to mining, and the system fan guard already
     * takes care of cooling. */
}

void board_check(void)
{
    uint16_t pattern_read = 0;
    unsigned board, attempt;

    BOARD_EN = 0;
    jas_sensor_board_en = 0;

    /* All boards together: 4.2 s, and the three following lines carry the
     * same second in the firmware's log. */
    jas_link_hard_reset_all();

    for (board = 0; board < BOARD_NUM; board++)
        printf(" board %u reset fpga hard reset done\n", board);
    fflush(stdout);
    sleep(2);

    for (board = 0; board < BOARD_NUM; board++) {
        uint16_t bit = (uint16_t)(1u << board);
        /*
         * EMPTY SLOTS PROBED ONLY ONCE: the X16-Q has a single board, in slot
         * 0, and the original probed slots 1 and 2 five times each, with two
         * fast resets per round (4.5 s lost at every startup). Slot 0 keeps its
         * five attempts; the others get only one, without reset, once a board
         * has been found before them. JASRE_BOARDS_FULL_POLL=1 restores
         * the stock behaviour.
         */
        unsigned attempts = 5;

        if (board > 0 && BOARD_EN != 0 && !getenv("JASRE_BOARDS_FULL_POLL")
            && !jas_marker("jasre_boards_full_poll"))
            attempts = 1;

        for (attempt = 0; attempt < attempts; attempt++) {
            sl_single_write(0xff, 0xf9, REG_PROBE, 0x5e, (int)board);
            sl_single_read_soft(0xff, 0xfd, REG_PROBE, &pattern_read, 5000, (int)board);
            if (pattern_read == 0x5e) {
                BOARD_EN |= bit;
                printf("board %u read 0x5e time %u success\n", board, attempt);
                break;
            }
            BOARD_EN &= (uint16_t)~bit;
            printf("board %u read 0x5e time %u fail, value = 0x%x\n",
                   board, attempt, pattern_read);
            if (attempts > 1)
                jas_link_hard_reset_fast((int)board);

            sl_single_write(0xff, 0xf9, REG_PROBE, 0xe5, (int)board);
            sl_single_read_soft(0xff, 0xfd, REG_PROBE, &pattern_read, 5000, (int)board);
            if (pattern_read == 0xe5) {
                BOARD_EN |= bit;
                printf("board %u read 0xe5 time %u success\n", board, attempt);
                break;
            }
            BOARD_EN &= (uint16_t)~bit;
            printf("board %u read 0xe5 time %u fail, value = 0x%x\n",
                   board, attempt, pattern_read);
            if (attempts > 1)
                jas_link_hard_reset_fast((int)board);
        }
    }

    for (board = 0; board < BOARD_NUM; board++)
        if (BOARD_EN >> board & 1)
            printf("board %u is exist \n", board);

    printf("board_en_g is 0x%x\n", BOARD_EN);
    fflush(stdout);

    /*
     * DELIBERATE DEPARTURE from the original: if the firmware finds no board,
     * it enters `while (board_en_g == 0) sleep(1);`, an endless wait that only
     * another thread could end. Nothing could change the value here, so the
     * wait is bounded to thirty seconds, and said.
     */
    if (BOARD_EN == 0) {
        unsigned n;

        printf("*********NO BOARD FOUND, PLEASE CHECK*********\n");
        for (n = 0; n < 30 && BOARD_EN == 0; n++)
            sleep(1);
        if (BOARD_EN == 0)
            jas_log("board_check: no board after 30 s (the stock firmware would"
                    " wait forever)");
    }

    jas_sensor_board_en = BOARD_EN;
}
