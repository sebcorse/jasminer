/*
 * Standalone binary: the miner itself (the default mode), plus the
 * diagnostic and bench modes that grew around it, listed further down from
 * the riskless to the complete.
 */

#include <signal.h>
#include <pthread.h>
#include "jas_hal.h"
#include "dft.h"
#include <sys/time.h>
#include <unistd.h>
#include "jas_state.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include "sl_transfer.h"
#include <sys/resource.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern void self_check_func(int thread_id);
extern int  jas_cb_variant;
extern void crossbar_fill_pub(int thread_id);
extern void crossbar_config_pub(int thread_id);
extern int  crossbar_run_pub(int thread_id, unsigned asic);
extern unsigned jas_manual_timeouts;
extern unsigned long jas_mw_calls, jas_mw_polls, jas_mw_ready_immediate, jas_mw_us_wait;
extern unsigned long jas_mw_mismatches;
extern void crossbar_check(int thread_id);
extern int  jas_selfcheck_skip_crossbar;

/*
 * CLEAN SHUTDOWN. A `kill -9` runs no exit code: the ASICs stay powered and
 * hashing at full power, with nobody left to watch their temperature. So
 * SIGINT and SIGTERM must reach power_off_operation, the only place that
 * turns running chips off. The handler only raises a flag
 * (power_off_operation takes the bus lock and is not signal-safe); the
 * state loop sees it within a second.
 */
volatile sig_atomic_t jas_stop_request;

static void jas_on_signal(int sig)
{
    jas_stop_request = sig;
}

/*
 * The seed the thread must use, copied HERE from the pool's work: NOT
 * jas_fpga_seed, which the mining loop only fills later (a cache built on
 * that null seed fails every DAG check).
 */


int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);   /* otherwise the log arrives in 4 KB blocks */

    /*
     * "-c <file>", as /etc/init.d/jasminer.sh starts it:
     *
     *     start-stop-daemon -b -S -x /usr/bin/jasminer.elf -- -c /media/configs/jasminer.json
     *
     * The path must come from the command line: the script may point to
     * another configuration (a trial, a factory fallback).
     */
    {
        int k;

        for (k = 1; k + 1 < argc; k++)
            if (!strcmp(argv[k], "-c")) {
                jas_config_set(argv[k + 1]);
                break;
            }
    }

    {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = jas_on_signal;
        sigaction(SIGINT,  &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);
        sigaction(SIGHUP,  &sa, NULL);

        /*
         * SIGPIPE IGNORED, belt and braces: every send() carries MSG_NOSIGNAL
         * (pool.c, web.c), but a single plain write() on a closed socket would
         * otherwise kill the miner silently. Ignoring it costs nothing: the
         * calls return EPIPE, which the error paths already handle.
         */
        signal(SIGPIPE, SIG_IGN);
    }

    if (jas_hal_init() != 0) {
        fprintf(stderr, "jasre: hardware unreachable\n");
        return 1;
    }
    printf("jasre, mode %s\n\n", jas_hal_mode());

    for (int board = 0; board < 2; board++) {
        volatile uint32_t *base = jas_axi_base(board);
        if (!base) { printf("  board %d: not mapped\n", board); continue; }

        volatile uint32_t *r = (volatile uint32_t *)
                               ((volatile unsigned char *)base + 0x20000);
        uint32_t st = r[3];      /* +12: status register, pure read */

        printf("  board %d: window mapped, state = 0x%08x", board, st);
        if (st == 0xffffffffu)      printf("  (bus silent - board missing?)\n");
        else if (st & 0x8u)         printf("  (transfer in progress)\n");
        else                        printf("  (link idle)\n");

        /* Neighboring words, to judge whether the window is well placed. */
        printf("            +0 %08x  +4 %08x  +8 %08x  +16 %08x  +20 %08x\n",
               r[0], r[1], r[2], r[4], r[5]);
    }

    if (argc > 2 && strcmp(argv[1], "--read") == 0) {
        /*
         * Issues real transactions.
         *
         * The firmware serializes its own around dev_info_g.sl_dev_mutex, in
         * sl_wrapper_work, not in sl_single_write. In bridge mode we take that
         * lock; in standalone mode we cannot, and a read while the miner runs
         * collides. That is expected: standalone mode is made to replace the
         * firmware, not to coexist with it.
         */
        unsigned cmd = (unsigned)strtoul(argv[2], NULL, 0);
        unsigned reg = (argc > 3) ? (unsigned)strtoul(argv[3], NULL, 0) : 0x80;
        unsigned n   = (argc > 4) ? (unsigned)strtoul(argv[4], NULL, 0) : 1;
        unsigned sel = (argc > 5) ? (unsigned)strtoul(argv[5], NULL, 0) : 0x01;

        for (unsigned k = 0; k < n; k++) {
            uint16_t v = 0;
            int rc;

            jas_bus_lock();
            rc = sl_single_read((uint8_t)sel, (uint8_t)cmd, (uint16_t)reg, &v, 1000000, 0);
            jas_bus_unlock();

            printf("  sel 0x%02x cmd 0x%02x reg 0x%04x -> rc=%2d value=0x%04x\n", sel, cmd, reg, rc, v);
        }
    }
    if (argc > 1 && strcmp(argv[1], "--dft") == 0) {
        /* The full memory test. Long (several minutes per board), and it
         * writes a lot into the chips: the miner must be stopped. */
        int board = (argc > 2) ? atoi(argv[2]) : 0;

        printf("\n  DFT test on board %d\n", board);
        jas_bus_lock();
        /*
         * The preamble of self_check_func. Without it, the chips stay in the
         * state mining left them in, and the "pre" phase measures something
         * else (thousands of healthy banks before repair where the firmware
         * counts zero).
         */
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        printf("  -> %d\n", dft_short_pattern_test_110(board));
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--selfcheck") == 0) {
        /*
         * The whole verification chain, in the original's order. It is the
         * only context where the memory test's verdicts are comparable to the
         * firmware's: a hand-assembled preamble skips config_asic_fix_param,
         * and a DRAM test without the calibrated timings does not measure the
         * same thing.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;

        printf("\n  full check of board %d\n", board);
        jas_bus_lock();
        self_check_func(board);
        jas_bus_unlock();

        for (unsigned a = 0; a < ASIC_NUM; a++)
            printf("  asic %u: pre=%u post=%u  (dp_fail=%u hb_fail=%u)\n", a,
                   jas_good_bank_num_pre[board][a], jas_good_bank_num_post[board][a],
                   jas_dp_fail_num[board][a], jas_hb_fail_num[board][a]);

        /* The full table, DFT and crossbar included: it is the one that
         * compares with the one the firmware keeps in RAM. */
        {
            const char *out = (argc > 3) ? argv[3] : "/tmp/ours_full.bin";
            FILE *f = fopen(out, "wb");

            if (f) {
                fwrite(jas_bank_good[board], sizeof(uint16_t), 12240, f);
                fclose(f);
                printf("  full table written to %s\n", out);
            }
            for (unsigned a = 0; a < ASIC_NUM; a++) {
                unsigned n = 0;
                for (unsigned b = 0; b < 4080; b++)
                    if (jas_bank_good[board][a * 4080 + b]) n++;
                printf("  asic %u: %u good banks after crossbar\n", a, n);
            }

            /* The raw scan record, to compare word for word with the one the
             * firmware keeps at 0x4a6a38. A global gap does not say WHICH banks
             * are missing; this dump does. */
            f = fopen("/tmp/ours_cb.bin", "wb");
            if (f) {
                fwrite(jas_cb_sc_result[board], sizeof(uint16_t), 3 * 120 * 255, f);
                fclose(f);
                printf("  crossbar readout written to /tmp/ours_cb.bin\n");
            }
        }
    }
    if (argc > 1 && strcmp(argv[1], "--dft-only") == 0) {
        /*
         * The same sequence as --selfcheck, but without crossbar_check, then a
         * raw dump of the bank table. Used to compare index by index with the
         * one the firmware keeps in RAM: a global gap does not say where it
         * happens, the list of indices does.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        const char *out = (argc > 3) ? argv[3] : "/tmp/ours.bin";
        FILE *f;

        jas_selfcheck_skip_crossbar = 1;
        jas_bus_lock();
        self_check_func(board);
        jas_bus_unlock();

        for (unsigned a = 0; a < ASIC_NUM; a++)
            printf("  asic %u: pre=%u post=%u dp=%u hb=%u\n", a,
                   jas_good_bank_num_pre[board][a], jas_good_bank_num_post[board][a],
                   jas_dp_fail_num[board][a], jas_hb_fail_num[board][a]);

        f = fopen(out, "wb");
        if (f) {
            fwrite(jas_bank_good[board], sizeof(uint16_t), 12240, f);
            fclose(f);
            printf("  bank table written to %s\n", out);
        }
    }
    if (argc > 1 && strcmp(argv[1], "--crossbar") == 0) {
        int board = (argc > 2) ? atoi(argv[2]) : 0;

        printf("  crossbar only, board %d\n", board);
        jas_bus_lock();
        crossbar_check(board);
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--clr") == 0) {
        int board = (argc > 2) ? atoi(argv[2]) : 0;

        printf("  link recovery, board %d\n", board);
        if (argc > 3 && strcmp(argv[3], "hard") == 0) {
            printf("  hard recovery (2 CLR+RSTN cycles)\n");
            jas_link_hard_reset(1, board);
        } else
            jas_link_reset(board);
        printf("  state after recovery: 0x%08x\n",
               ((volatile uint32_t *)((volatile unsigned char *)jas_axi_base(board)
                                      + 0x20000))[3]);
    }
    if (argc > 1 && strcmp(argv[1], "--filltest") == 0) {
        /*
         * THE EXPERIMENT THAT SETTLES IT: are the memory controllers on or off
         * during the fill? The original turns them on before filling; here,
         * with them on, manual writes time out en masse, and off they all go
         * through. So both settings are measured on prepared silicon: how many
         * writes time out, and what the memory really contains afterwards (a
         * write that "goes through" proves nothing if nothing was written).
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned n_write = (argc > 3) ? (unsigned)strtoul(argv[3], NULL, 0) : 200;
        int mode;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);

        /*
         * config_remap_info builds the remap table from the banks declared
         * good. Without the memory test, they are all zero: the table is
         * empty, no ALU reaches anything, and every read-back returns a
         * constant value.
         *
         * dft_short_pattern_test_110 starts precisely by declaring everything
         * good. The same is done here, otherwise the experiment only measures
         * the absence of a table.
         */
        for (unsigned k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k]    = 1;
            jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k]     = 0;
        }

        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        printf("  preparation done\n\n");

        for (mode = 0; mode < 2; mode++) {
            uint16_t buf[64];
            uint32_t sram[16];
            struct timeval t0, t1;
            unsigned k, timeouts = 0, mismatch = 0, checked = 0;
            long ms;

            /* Reset, then the setting under trial. */
            sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0, board);
            sl_delay(0, 2000);
            sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);

            memset(buf, mode ? 0xff : 0x00, sizeof(buf));
            sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
            sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board);

            printf("  --- memory controllers %s ---\n", mode ? "ON" : "off");

            jas_manual_timeouts = 0;
            gettimeofday(&t0, NULL);
            for (k = 0; k < n_write; k++) {
                for (unsigned w = 0; w < 16; w++)
                    sram[w] = 0x5a5a5a5au + k;
                single_manual_write(ASIC_EN(board), k * 2,     (uint16_t *)sram, board);
                single_manual_write(ASIC_EN(board), k * 2 + 1, (uint16_t *)sram, board);
            }
            gettimeofday(&t1, NULL);
            timeouts = jas_manual_timeouts;
            ms = (t1.tv_sec - t0.tv_sec) * 1000L + (t1.tv_usec - t0.tv_usec) / 1000;

            printf("    %u writes in %ld ms, %u timeouts\n",
                   n_write * 2, ms, timeouts);

            /* And above all: was anything written? */
            for (k = 0; k < n_write && checked < 8; k += n_write / 8 + 1) {
                uint16_t back[64];
                uint32_t expect = 0x5a5a5a5au + k;

                memset(back, 0, sizeof(back));
                /* The write sets 32 words per call, the read returns 64: the
                 * read's address unit is thus worth TWO write units. Reading
                 * address A returns what was written at 2A. */
                if (single_manual_read(1, 0, k, back, board) < 0)
                    continue;
                checked++;
                uint32_t got; memcpy(&got, back, 4);
                if (got != expect) {
                    if (mismatch < 3)
                        printf("    read %u (write %u): got 0x%08x, expected 0x%08x\n",
                               k, k * 2, got, expect);
                    mismatch++;
                }
            }
            printf("    read back: %u addresses checked, %u mismatched\n\n",
                   checked, mismatch);
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--segtest") == 0) {
        /*
         * DOES THE FILL REACH THE EIGHT SEGMENTS? The scan walks eight segments
         * separated by a large step of 0x7f4041, and the write address is
         * twice that: this checks that the far segments are really written (a
         * one-third coverage would mean only three of them get their data).
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        const uint32_t big = 0x007f4041u;
        const uint32_t seg = 4080;
        uint32_t sram[16];
        unsigned k, w, seg_i, ok = 0;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);

        /* Controllers on: established by --filltest. */
        {
            uint16_t buf[64];
            memset(buf, 0xff, sizeof(buf));
            sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
            sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board);
        }
        printf("  writing the 8 segments (8 x 4080 blocks)\n");

        /* The real fill, address and data in lockstep. */
        {
            uint32_t addr = 0, data = 0x5a5a5a5au;
            unsigned n;

            for (n = 0; n < 32640u; n++) {
                for (w = 0; w < 16; w++) sram[w] = data;
                single_manual_write(ASIC_EN(board), addr * 2,     (uint16_t *)sram, board);
                single_manual_write(ASIC_EN(board), addr * 2 + 1, (uint16_t *)sram, board);
                if ((n + 1) % seg == 0) { addr += big; data += big; }
                else                    { addr += 1;   data += 1;   }
            }
        }

        printf("\n  read back: ONE address at the start of each segment\n");
        for (seg_i = 0; seg_i < 8; seg_i++) {
            uint32_t addr = seg_i * (seg - 1 + big);
            uint32_t expect = 0x5a5a5a5au + addr;
            uint16_t back[64];
            uint32_t got = 0;

            memset(back, 0, sizeof(back));
            if (single_manual_read(1, 0, addr, back, board) < 0) {
                printf("    segment %u (addr 0x%08x): READ WITH NO REPLY\n", seg_i, addr);
                continue;
            }
            memcpy(&got, back, 4);
            printf("    segment %u (addr 0x%08x): got 0x%08x expected 0x%08x  %s\n",
                   seg_i, addr, got, expect, got == expect ? "OK" : "MISMATCH");
            if (got == expect) ok++;
        }
        printf("\n  %u of 8 segments read back correctly\n", ok);
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--aliastest") == 0) {
        /*
         * DO THE SEGMENTS OVERLAP? If the eight segments fall back on the same
         * physical memory, each erases the previous one. Two writes and two
         * reads settle it: a pattern at address 0, another at the large
         * step's address, then address 0 read back; the second pattern there
         * means the addresses wrap around.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        const uint32_t big = 0x007f4041u;
        uint32_t sram[16];
        uint16_t back[64];
        uint32_t got = 0;
        unsigned w;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (unsigned k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

        /* pattern A at address 0 */
        for (w = 0; w < 16; w++) sram[w] = 0xAAAA1111u;
        single_manual_write(ASIC_EN(board), 0, (uint16_t *)sram, board);
        single_manual_write(ASIC_EN(board), 1, (uint16_t *)sram, board);

        memset(back, 0, sizeof(back));
        single_manual_read(1, 0, 0, back, board);
        memcpy(&got, back, 4);
        printf("  after A: address 0 = 0x%08x (expected 0xaaaa1111)  %s\n",
               got, got == 0xAAAA1111u ? "OK" : "MISMATCH");

        /* pattern B at the large step's address */
        for (w = 0; w < 16; w++) sram[w] = 0xBBBB2222u;
        single_manual_write(ASIC_EN(board), big * 2,     (uint16_t *)sram, board);
        single_manual_write(ASIC_EN(board), big * 2 + 1, (uint16_t *)sram, board);

        memset(back, 0, sizeof(back));
        single_manual_read(1, 0, big, back, board);
        memcpy(&got, back, 4);
        printf("  after B: address 0x%08x = 0x%08x (expected 0xbbbb2222)  %s\n",
               big, got, got == 0xBBBB2222u ? "OK" : "MISMATCH");

        /* and did address 0 survive? */
        memset(back, 0, sizeof(back));
        single_manual_read(1, 0, 0, back, board);
        memcpy(&got, back, 4);
        printf("  read back address 0 = 0x%08x  ->  %s\n", got,
               got == 0xAAAA1111u ? "INTACT, the segments do not overlap"
             : got == 0xBBBB2222u ? "OVERWRITTEN BY B: THE ADDRESSES WRAP AROUND"
             : "SOMETHING ELSE");
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--pmbusfix") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * Set the voltages back to the setpoint, and check it by reading back.
         *
         * Also serves as a repair: after a power_on_operation without PMBus,
         * the machine stays at ~901 mV instead of 860.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        int mv = (argc > 3) ? atoi(argv[3]) : 860;
        unsigned a;

        jas_bus_lock();
        jas_iic_map_init(board);
        iic_initialization_op(board);

        printf("\n  before:\n");
        for (a = 0; a < ASIC_NUM; a++) {
            uint16_t v = 0;
            if (jas_pmbus_read_vout(board, a, &v) == 0)
                printf("    asic %u: %u mV\n", a, v);
        }
        printf("\n  writing setpoint %d mV\n", mv);
        for (a = 0; a < ASIC_NUM; a++) {
            asic_pmbus_config(0x500, board, a, (uint16_t)mv, 1150, 1500);
        }
        printf("\n  after:\n");
        for (a = 0; a < ASIC_NUM; a++) {
            uint16_t v = 0;
            if (jas_pmbus_read_vout(board, a, &v) == 0)
                printf("    asic %u: %u mV   %s\n", a, v,
                       (v > (unsigned)mv - 15 && v < (unsigned)mv + 15) ? "OK" : "<<< off target");
        }
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--pmbus") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * READ THE VOLTAGES, SETTING NOTHING: READ_VOUT (0x8b) as the firmware
         * left it, to check whether power_on_operation restores the voltages
         * after its power cycle, and to confirm the encoding (raw 123 = 0x7B =
         * 860 mV) before writing the same value.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        const uint8_t *adr = jas_pmbus_addresses();
        unsigned a;

        jas_bus_lock();
        jas_iic_map_init(board);
        iic_initialization_op(board);

        printf("\n  voltages applied, read by READ_VOUT (0x8b)\n");
        for (a = 0; a < ASIC_NUM; a++) {
            uint16_t raw = 0;

            if (jas_pmbus_read_vout(board, a, &raw) < 0) {
                printf("    asic %u (0x%02x): no reply\n", a, adr[a]);
                continue;
            }
            /*
             * READ_VOUT returns the millivolts DIRECTLY (858, 860, 860 for a
             * setpoint of 860), unlike VOUT_COMMAND, where mcp2973_set_vol
             * writes (mV / 5 - 0x31): the two registers differ in encoding.
             */
            printf("    asic %u (0x%02x): %u mV   (setpoint 860)\n",
                   a, adr[a], raw);
        }
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--startup") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * THE COMPLETE SEQUENCE, IN THE FIRMWARE'S ORDER.
         *
         *   find_borad_and_poweron : board_check, sensor_init, power_on_operation
         *   self_check_thread_0    : self_check_func per board
         *
         * The self-test thus runs with the chips' clock (pll_lock 0x07,
         * 510 MHz) and voltage (860 mV) under our control.
         */
        unsigned board;

        jas_bus_lock();
        find_borad_and_poweron();

        /* seed_update_operation initializes the bank state before the
         * self-test. Without it, config_remap_info writes an empty table and
         * 100 % of the crossbar accesses time out. See core/seed.c. */
        seed_init_bank_params();

        printf("\nin self_check_thread_0\n");
        for (board = 0; board < BOARD_NUM; board++) {
            if (!(BOARD_EN >> board & 1))
                continue;
            self_check_func((int)board);
            /* The raw record, to compare with the firmware's: the total is not
             * enough, the SHAPE proves the memory is measured (banks dead for
             * all 120 ALUs, near-perfect agreement between ALUs). */
            {
                char name[64];
                FILE *f;
                unsigned a;

                for (a = 0; a < ASIC_NUM; a++) {
                    snprintf(name, sizeof(name), "/tmp/cb_c%u_a%u.bin", board, a);
                    f = fopen(name, "wb");
                    if (f) {
                        fwrite(jas_cb_sc_result[board][a], sizeof(uint16_t),
                               120 * 255, f);
                        fclose(f);
                    }
                }
            }
        }
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--pll") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * CLOCK CHECK, in three reads:
         *
         *   1. read 0x11 touching nothing             -> must be 0x07
         *   2. do the FPGA reset only, read again     -> must drop to 0x00
         *   3. call power_on_operation, read again    -> must come back to 0x07
         *
         * The FPGA reset leaves the ASICs without a clock, and only
         * power_on_operation gives it back (with no clock, the DRAM absorbs
         * nothing).
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint16_t v = 0, c[3] = { 0, 0, 0 };

        jas_bus_lock();

        sl_single_read(0xff, 0xfd, 0x11, &v, 1000000, board);
        sl_multi_read(0xff, 0xfd, 0x23, c, 3, 4000000, board);
        printf("\n  1. initial state:         pll_lock=0x%02x  clk=%u %u %u\n",
               v, c[0], c[1], c[2]);

        jas_link_hard_reset_all();
        /*
         * LET THE LINK COME BACK BEFORE USING IT: reading an AXI link just out
         * of reset blocks the ARM core (mains power cut needed). board_check
         * waits 2 s after this same reset before touching the bus.
         */
        sleep(2);
        v = 0; memset(c, 0, sizeof(c));
        sl_single_read(0xff, 0xfd, 0x11, &v, 1000000, board);
        sl_multi_read(0xff, 0xfd, 0x23, c, 3, 4000000, board);
        printf("  2. after the FPGA reset:  pll_lock=0x%02x  clk=%u %u %u\n",
               v, c[0], c[1], c[2]);

        power_on_operation(board);
        v = 0; memset(c, 0, sizeof(c));
        sl_single_read(0xff, 0xfd, 0x11, &v, 1000000, board);
        sl_multi_read(0xff, 0xfd, 0x23, c, 3, 4000000, board);
        printf("\n  3. after power_on:        pll_lock=0x%02x  clk=%u %u %u\n",
               v, c[0], c[1], c[2]);
        printf("     asic_en=0x%02x  pll_stable=%u  (expected: 0x07 and 7)\n",
               ASIC_EN(board), jas_pll_stable[board]);

        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--boardcheck") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * Step 2 of the recovery, in the firmware's order.
         *
         * Success criterion, given by its own trace in flash:
         *     board 0/1/2 mmap base address ok.
         *      board 0/1/2 reset fpga hard reset done
         *     board 0 is exist
         *     board_en_g is 0x1
         */
        jas_bus_lock();
        board_check();
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--poweron") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * The complete hardware startup, in the firmware's order: board_check,
         * sensor_init, then powering on per board.
         */
        jas_bus_lock();
        find_borad_and_poweron();
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--rdstab") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * WHICH REGISTERS REALLY READ BACK? In the 0x7xx / 0x8xx block, a read
         * may return a stream unrelated to the register asked for (probably
         * write-only registers), and polling 0x824, on which the whole fill
         * rests, would then measure nothing. Starting at 0x580 (known to read
         * back), each register is read eight times in a row, writing nothing:
         * a real register returns the same value eight times, a stream will
         * not.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint8_t asic_en = 1;
        static const uint16_t regs[] = { 0x580, 0x824, 0x800, 0x802, 0x822,
                                         0x705, 0x5be, 0x0d00 };
        unsigned i, k;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        {
            unsigned q, m = 0;
            for (q = 0; q < ASIC_NUM; q++)
                if ((ASIC_EN(board) >> q & 1) && jas_spi_status[board][q] != 1) m++;
            if (m) { printf("  STOP: %u asic(s) with no valid phase\n", m);
                     jas_bus_unlock(); return 1; }
        }
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);

        printf("\n  eight reads in a row, writing nothing in between:\n");
        for (i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
            uint16_t v[8];
            int stable = 1;

            for (k = 0; k < 8; k++) {
                v[k] = 0;
                sl_single_read(asic_en, 0x1c, regs[i], &v[k], 1000000, board);
                if (k && v[k] != v[0]) stable = 0;
            }
            printf("    0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x  %s\n",
                   regs[i], v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
                   stable ? "STABLE" : "<<< unstable");
        }
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--dramchk") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * DOES THE MANUAL PATH ANSWER AT ALL? Three reads at the SAME place,
         * with a different write between each: if the value changes, both
         * paths work and only a question of scale remains; if not, one of
         * the two does nothing.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned alu = (argc > 3) ? (unsigned)strtoul(argv[3], NULL, 0) : 0;
        uint8_t asic_en = 1;
        uint16_t sel[2] = { 0, 0 };
        uint8_t base_cmd = (uint8_t)(0x10u << (alu / 30u));
        uint8_t cmd_w = (uint8_t)(base_cmd | 8), cmd_r = (uint8_t)(base_cmd | 0xc);
        uint32_t sram[16];
        unsigned k;

        { unsigned r = alu % 30u; sel[r >> 4] = (uint16_t)(1u << (r & 0xf)); }

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        {
            unsigned q, m = 0;
            for (q = 0; q < ASIC_NUM; q++)
                if ((ASIC_EN(board) >> q & 1) && jas_spi_status[board][q] != 1) m++;
            if (m) { printf("  STOP: %u asic(s) with no valid phase\n", m);
                     jas_bus_unlock(); return 1; }
        }
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (k = 0; k < ASIC_NUM; k++)
            if (ASIC_EN(board) >> k & 1) config_remap_info(board, (uint8_t)k, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

        /*
         * The memory test loads the redundancy registers. Without them the DRAM
         * may absorb no write at all: the project notes already say that "the
         * scan gives nothing if the DFT has not run before". That is the
         * variable under trial here, and the only one.
         */
        if (getenv("JASRE_WITH_DFT")) {
            printf("  preliminary memory test\n");
            alu_check(board);
            alu_check_analysis(board);
            dft_short_pattern_test_110(board);
            { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
              sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
              sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }
        }

        sl_multi_write(asic_en, cmd_w, 0x700, sel, 2, board);

        {
            unsigned step;
            static const uint32_t patterns[3] = { 0, 0xaaaa0001u, 0xbbbb0002u };

            printf("\n  ALU %u: read of address 0, before and after write\n", alu);
            for (step = 0; step < 3; step++) {
                uint32_t manual_addr = 0;
                uint16_t low[32], high[32], valid;
                unsigned rounds;

                if (step > 0) {
                    uint16_t e0 = 0, e1 = 0, e2 = 0, ready = 0, mode = 0;

                    for (k = 0; k < 16; k++) sram[k] = patterns[step];
                    single_manual_write(asic_en, 0, (uint16_t *)sram, board);
                    single_manual_write(asic_en, 1, (uint16_t *)sram, board);
                    printf("    wrote 0x%08x at 0 and 1\n", patterns[step]);

                    /*
                     * DOES THE CHIP EVEN RECEIVE OUR WRITES?
                     *
                     * Before looking for where the data lands, check that it
                     * leaves. The registers of the write path are read back:
                     * if 0x800 returns the address set and 0x802 the first
                     * word, the chip took them. Otherwise, nothing that
                     * follows means anything.
                     */
                    sl_single_read(asic_en, 0x1c, 0x800, &e0, 1000000, board);
                    sl_single_read(asic_en, 0x1c, 0x801, &e1, 1000000, board);
                    sl_single_read(asic_en, 0x1c, 0x802, &e2, 1000000, board);
                    sl_single_read(asic_en, 0x1c, 0x822, &mode, 1000000, board);
                    sl_single_read(asic_en, 0x1c, 0x824, &ready, 1000000, board);
                    printf("    read back 0x800=%04x 0x801=%04x 0x802=%04x  (expected 0001 0000 %04x)\n",
                           e0, e1, e2, (uint16_t)(patterns[step] & 0xffff));
                    printf("    read back 0x822=%04x (mode) 0x824=%04x (ready)\n", mode, ready);
                }

                sl_multi_write(asic_en, cmd_w, 0x702, (uint16_t *)&manual_addr, 2, board);
                sl_single_write(asic_en, cmd_w, 0x704, 1, board);
                usleep(1000);
                valid = 0; rounds = 0;
                while (valid == 0 && rounds < 3000) {
                    sl_single_read(asic_en, cmd_r, 0x705, &valid, 1000000, board);
                    rounds++;
                }
                printf("    0x705 = %04x after %u polls\n", valid, rounds);
                sl_single_write(asic_en, cmd_w, 0x705, 1, board);
                {   /* does the flag drop after acknowledgement? */
                    uint16_t after = 0xffff;
                    sl_single_read(asic_en, cmd_r, 0x705, &after, 1000000, board);
                    printf("    0x705 = %04x after acknowledgment\n", after);
                }
                memset(low, 0, sizeof(low)); memset(high, 0, sizeof(high));
                sl_multi_read(asic_en, cmd_r, 0x706, low,  0x20, 1000000, board);
                sl_multi_read(asic_en, cmd_r, 0x726, high, 0x20, 1000000, board);
                printf("    read 0x706: %04x %04x %04x %04x %04x %04x %04x %04x\n",
                       low[0], low[1], low[2], low[3], low[4], low[5], low[6], low[7]);
                printf("    read 0x726: %04x %04x %04x %04x\n",
                       high[0], high[1], high[2], high[3]);
            }
        }
        sl_single_write(asic_en, 0xf8, 0x700, 0, board);
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--dramrd") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * WHERE DOES WHAT WE WRITE LAND? The firmware's dram_dump gives a
         * third path: a manual read through an ALU (0x700 selection, 0x702
         * address, 0x704 trigger, 0x705 acknowledgement, 0x706 and 0x726
         * data), 2 x 64 bytes per address, precisely the two blocks
         * single_manual_write sets at 2n and 2n+1, with the scan's address
         * unit. A recognizable pattern is written, then SEARCHED for: its
         * position gives the mapping without any hypothesis.
         */
        int board  = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned alu = (argc > 3) ? (unsigned)strtoul(argv[3], NULL, 0) : 0;
        unsigned n_scan = (argc > 4) ? (unsigned)strtoul(argv[4], NULL, 0) : 64;
        uint8_t asic_en = 1;                 /* chip 0 alone, not to mix things up */
        uint16_t sel[2] = { 0, 0 };
        uint8_t base_cmd, cmd_w, cmd_r;
        uint32_t sram[16];
        unsigned k, a;
        const uint32_t MARKER = 0xc0ffee00u;

        base_cmd = (uint8_t)(0x10u << (alu / 30u));
        cmd_w = (uint8_t)(base_cmd | 8);
        cmd_r = (uint8_t)(base_cmd | 0xc);
        {
            unsigned r = alu % 30u;
            sel[r >> 4] = (uint16_t)(1u << (r & 0xf));
        }

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        {
            unsigned q, silent = 0;
            for (q = 0; q < ASIC_NUM; q++)
                if ((ASIC_EN(board) >> q & 1) && jas_spi_status[board][q] != 1) silent++;
            if (silent) { printf("  STOP: %u asic(s) with no valid phase\n", silent);
                           jas_bus_unlock(); return 1; }
        }
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);

        /* identity table: the only configuration where the scan is stable */
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (k = 0; k < ASIC_NUM; k++)
            if (ASIC_EN(board) >> k & 1) config_remap_info(board, (uint8_t)k, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

        /* A distinct marker per write address, to recognize which one is
         * found: 0xc0ffee00 + block number. */
        printf("\n  writing markers at manual addresses 0..7\n");
        for (a = 0; a < 8; a++) {
            for (k = 0; k < 16; k++) sram[k] = MARKER + a;
            single_manual_write(asic_en, a, (uint16_t *)sram, board);
        }

        printf("  manual read by alu %u (mesh %u, rank %u), addresses 0..%u\n",
               alu, alu / 30u, alu % 30u, n_scan - 1);
        sl_multi_write(asic_en, cmd_w, 0x700, sel, 2, board);

        for (a = 0; a < n_scan; a++) {
            uint32_t manual_addr = a;
            uint16_t low[32], high[32], valid = 0;
            unsigned rounds = 0;

            sl_multi_write(asic_en, cmd_w, 0x702, (uint16_t *)&manual_addr, 2, board);
            sl_single_write(asic_en, cmd_w, 0x704, 1, board);
            usleep(1000);
            while (valid == 0 && rounds < 2000) {
                sl_single_read(asic_en, cmd_r, 0x705, &valid, 1000000, board);
                rounds++;
            }
            if (valid == 0) { printf("    adr %4u: NO VALID DATA\n", a); continue; }
            sl_single_write(asic_en, cmd_w, 0x705, 1, board);

            memset(low, 0, sizeof(low));  memset(high, 0, sizeof(high));
            sl_multi_read(asic_en, cmd_r, 0x706, low,  0x20, 1000000, board);
            sl_multi_read(asic_en, cmd_r, 0x726, high, 0x20, 1000000, board);

            {
                uint32_t m0 = (uint32_t)low[0]  | ((uint32_t)low[1]  << 16);
                uint32_t m1 = (uint32_t)high[0] | ((uint32_t)high[1] << 16);
                const char *note = "";

                if ((m0 & 0xffffff00u) == MARKER || (m1 & 0xffffff00u) == MARKER)
                    note = "   <<<<< MARKER";
                if (a < 16 || *note)
                    printf("    adr %4u: 0x706 %08x %04x %04x | 0x726 %08x%s\n",
                           a, m0, low[2], low[3], m1, note);
            }
        }
        sl_single_write(asic_en, 0xf8, 0x700, 0, board);
        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--chantest") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * MEASURE THE READ CHANNEL, WITH A GROUND TRUTH: a known pattern is
         * written to the trial window 0x580 (the one spi_check uses) and read
         * back; any wrong bit is a channel defect and nothing else. The
         * sampling phase 0x746 is then swept: if the error rate depends on
         * it, the calibration is the cause.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned asic = (argc > 3) ? (unsigned)atoi(argv[3]) : 0;
        uint8_t asic_en = (uint8_t)(1u << asic);
        const unsigned REPEATS = 20;
        static const uint16_t patterns[3] = { 0xa5a5, 0x5a5a, 0xffff };
        unsigned p, mesh, r, i, m;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        if (!getenv("JASRE_NO_SPI"))
            spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);

        printf("\n  asic %u — read error rate per phase 0x746\n", asic);
        printf("    phase   wrong bits / bits read   rate      perfect words\n");

        for (p = 0; p < 18; p++) {
            unsigned long wrong = 0, read = 0, perfect = 0, words = 0;
            uint16_t phase;

            /* p == 17: reprogram nothing, keep spi_check's choice. */
            if (p < 17) {
                phase = (p == 0) ? 0 : (uint16_t)(1u << (p - 1));
                for (mesh = 0; mesh < 4; mesh++)
                    sl_single_write(asic_en, (uint8_t)(((1u << mesh) << 4) | 8),
                                    0x746, phase, board);
            } else {
                phase = 0xdead;
            }

            for (mesh = 0; mesh < 4; mesh++) {
                uint8_t cmd_w = (uint8_t)(((1u << mesh) << 4) | 8);
                uint8_t cmd_r = (uint8_t)(((1u << mesh) << 4) | 0xc);

                for (m = 0; m < 3; m++) {
                    uint16_t w_data[60], r_data[60];

                    for (i = 0; i < 60; i++) w_data[i] = patterns[m];
                    sl_multi_write(asic_en, cmd_w, 0x580, w_data, 0x3c, board);

                    for (r = 0; r < REPEATS; r++) {
                        memset(r_data, 0, sizeof(r_data));
                        sl_multi_read(asic_en, cmd_r, 0x580, r_data, 0x3c,
                                      4000000, board);
                        for (i = 0; i < 60; i++) {
                            uint16_t d = (uint16_t)(r_data[i] ^ w_data[i]);
                            if (!d) perfect++;
                            words++;
                            while (d) { wrong += d & 1; d >>= 1; }
                            read += 16;
                        }
                    }
                }
            }

            if (p < 17)
                printf("    0x%04x  %7lu / %7lu   %6.3f %%   %lu/%lu\n",
                       phase, wrong, read, wrong * 100.0 / read, perfect, words);
            else
                printf("    spi_ck  %7lu / %7lu   %6.3f %%   %lu/%lu   <- spi_check's choice\n",
                       wrong, read, wrong * 100.0 / read, perfect, words);
        }

        jas_bus_unlock();
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--readtest") == 0) {
        /*
         * VALIDATE THE INSTRUMENT BEFORE USING IT: single_manual_read
         * returned the right value when hundreds of writes came between the
         * write and the read-back, and zero when the read-back followed at
         * once. TIME and TRAFFIC are varied separately to tell which one it
         * needs.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint32_t sram[16];
        uint16_t back[64];
        unsigned w, k;
        struct { const char *name; unsigned us; unsigned dummies; } cases[] = {
            { "immediate",            0,       0   },
            { "1 ms wait",            1000,    0   },
            { "10 ms wait",           10000,   0   },
            { "100 ms wait",          100000,  0   },
            { "1 s wait",             1000000, 0   },
            { "10 writes elsewhere",  0,       10  },
            { "100 writes elsewhere", 0,       100 },
            { "400 writes elsewhere", 0,       400 },
        };

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }
        printf("\n  write at address 0, read back per condition:\n");

        for (k = 0; k < sizeof(cases)/sizeof(cases[0]); k++) {
            uint32_t pattern = 0x11110000u + k;
            uint32_t got = 0;
            unsigned d;

            for (w = 0; w < 16; w++) sram[w] = pattern;
            single_manual_write(ASIC_EN(board), 0, (uint16_t *)sram, board);
            single_manual_write(ASIC_EN(board), 1, (uint16_t *)sram, board);

            if (cases[k].us) usleep(cases[k].us);
            for (d = 0; d < cases[k].dummies; d++) {
                uint32_t junk[16];
                for (w = 0; w < 16; w++) junk[w] = 0xdeadbeefu;
                single_manual_write(ASIC_EN(board), 1000 + d * 2,     (uint16_t *)junk, board);
                single_manual_write(ASIC_EN(board), 1000 + d * 2 + 1, (uint16_t *)junk, board);
            }

            memset(back, 0, sizeof(back));
            if (single_manual_read(1, 0, 0, back, board) < 0) {
                printf("    %-24s READ WITH NO REPLY\n", cases[k].name);
                continue;
            }
            memcpy(&got, back, 4);
            printf("    %-24s got 0x%08x expected 0x%08x  %s\n",
                   cases[k].name, got, pattern, got == pattern ? "OK" : "no");
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--wtest") == 0) {
        /*
         * DO THE WRITE AND THE READ WANT THE SAME STATE? The manual write
         * takes a direct path (0x800/0x802/0x822), the read goes through an
         * ALU (0x700-0x706). Four combinations of the memory controllers'
         * state, each with a DISTINCTIVE pattern (never 0x5a5a5a5a, which the
         * fills leave behind and which would pass old data for a write).
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint32_t sram[16];
        uint16_t back[64];
        unsigned w, k;
        int wmc, rmc;

        printf("\n  preparing board %d\n", board);
        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        printf("\n  write / read per controller state:\n");

        k = 0;
        for (wmc = 0; wmc < 2; wmc++) {
            for (rmc = 0; rmc < 2; rmc++) {
                uint32_t pattern = 0xC0DE0000u + (k++ << 8) + 0x5Au;
                uint32_t got = 0;
                uint16_t buf[64];

                /* state for the WRITE */
                memset(buf, wmc ? 0xff : 0x00, sizeof(buf));
                sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
                sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board);

                for (w = 0; w < 16; w++) sram[w] = pattern;
                single_manual_write(ASIC_EN(board), 0, (uint16_t *)sram, board);
                single_manual_write(ASIC_EN(board), 1, (uint16_t *)sram, board);

                /* state for the READ */
                memset(buf, rmc ? 0xff : 0x00, sizeof(buf));
                sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
                sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board);

                memset(back, 0, sizeof(back));
                if (single_manual_read(1, 0, 0, back, board) < 0) {
                    printf("    write mc=%s read mc=%s: READ WITH NO REPLY\n",
                           wmc ? "ON " : "off", rmc ? "ON " : "off");
                    continue;
                }
                memcpy(&got, back, 4);
                printf("    write mc=%s read mc=%s: got 0x%08x expected 0x%08x  %s\n",
                       wmc ? "ON " : "off", rmc ? "ON " : "off", got, pattern,
                       got == pattern ? "*** OK ***" : "no");
            }
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--rdctl") == 0) {
        /*
         * CHECK OF THE READ INSTRUMENT, without any write. If the read works,
         * different addresses return different values (the last fill left
         * 0x5a5a5a5a + address: 0, 1, 2, 100 -> ...5a, ...5b, ...5c, ...abe);
         * if it is inert, all return the same thing. Reads also go through
         * different ALUs: a value that does not depend on the ALU either
         * means the instrument measures nothing.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint16_t back[64];
        unsigned k;
        const uint32_t addrs[] = { 0, 1, 2, 100, 4079, 4080 };

        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

        printf("\n  A) same ALU (0), different addresses:\n");
        for (k = 0; k < sizeof(addrs)/sizeof(addrs[0]); k++) {
            uint32_t got = 0;
            memset(back, 0, sizeof(back));
            if (single_manual_read(1, 0, addrs[k], back, board) < 0) {
                printf("    address %5u: NO REPLY\n", addrs[k]); continue;
            }
            memcpy(&got, back, 4);
            printf("    address %5u: 0x%08x   (0x5a5a5a5a + adr = 0x%08x)\n",
                   addrs[k], got, 0x5a5a5a5au + addrs[k]);
        }

        printf("\n  B) same address (0), different ALUs:\n");
        for (k = 0; k < 4; k++) {
            uint16_t sel = (uint16_t)(k * 31);   /* mesh 0 alu0, mesh1 alu1, ... */
            uint32_t got = 0;
            memset(back, 0, sizeof(back));
            if (single_manual_read(1, sel, 0, back, board) < 0) {
                printf("    ALU %3u (mesh %u): NO REPLY\n", sel, sel / 30); continue;
            }
            memcpy(&got, back, 4);
            printf("    ALU %3u (mesh %u): 0x%08x\n", sel, sel / 30, got);
        }

        printf("\n  C) the first 8 words of address 0:\n     ");
        memset(back, 0, sizeof(back));
        if (single_manual_read(1, 0, 0, back, board) == 0)
            for (k = 0; k < 8; k++) printf("%04x ", back[k]);
        printf("\n");
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--maptest") == 0) {
        /*
         * WHERE DOES A WRITE GO? And, incidentally, does it write.
         *
         * ONE distinctive pattern is left at ONE single write address, then the
         * read addresses around the two candidates are swept.
         *
         * Expected IF the write works and read(A) = write(2A):
         *     the pattern shows up at read address 100.
         * Expected IF the write works and the units coincide:
         *     it shows up at read address 200.
         * Expected IF the write does not work:
         *     it shows up NOWHERE: that is the negative control.
         *
         * The pattern 0xC0FFEE.. looks like nothing the fills leave behind.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        const uint32_t W_ADDR = 200;
        const uint32_t pattern = 0xC0FFEE42u;
        uint32_t sram[16];
        uint16_t back[64];
        unsigned k, w, found = 0;

        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

        printf("\n  writing 0x%08x at write address %u (single block)\n",
               pattern, W_ADDR);
        for (w = 0; w < 16; w++) sram[w] = pattern;
        single_manual_write(ASIC_EN(board), W_ADDR, (uint16_t *)sram, board);

        printf("  sweeping read addresses 95..105 and 195..205:\n");
        for (k = 95; k <= 205; k++) {
            uint32_t got = 0;
            if (k == 106) k = 195;
            memset(back, 0, sizeof(back));
            if (single_manual_read(1, 0, k, back, board) < 0) continue;
            memcpy(&got, back, 4);
            if (got == pattern) {
                printf("    *** FOUND at read address %u ***\n", k);
                found++;
            }
        }
        if (!found)
            printf("    pattern not found: THE WRITE IS NOT TAKING\n");

        /* negative control: a far address must contain nothing */
        {
            uint32_t got = 0;
            memset(back, 0, sizeof(back));
            if (single_manual_read(1, 0, 3000, back, board) == 0) {
                memcpy(&got, back, 4);
                printf("    negative control, address 3000: 0x%08x  %s\n", got,
                       got == pattern ? "!! CONTAMINATED, test invalid" : "clean");
            }
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--wvariant") == 0) {
        /*
         * FOUR VARIANTS OF THE WRITE PROTOCOL.
         *
         * The decompiled code gives: address (0x800), write-enable
         * (0x822 <- 1), data (0x802), wait for ready (0x824). We transcribe
         * that faithfully and nothing is written, while the chip
         * acknowledges.
         *
         * The original's names (sram_waddr, sram_wen, sram_wdata,
         * sram_wen_ready) suggest an enable signal. An enable signal is
         * usually lowered again. So we try:
         *
         *   V0  the current transcription                    (control)
         *   V1  same, then 0x822 <- 0 after the data
         *   V2  address, data, THEN 0x822 <- 1
         *   V3  explicit rising edge before the data
         *
         * Each variant writes a distinctive pattern at a distinct address,
         * then the pattern is searched for in the two candidate mappings. No
         * variant found = none is the right one: that is a result, not a
         * measurement failure.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        uint16_t sram[32];
        uint16_t back[64];
        unsigned k, w, v;
        uint8_t en, cmd = 0x18;

        jas_bus_lock();
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);
        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        for (k = 0; k < ASIC_NUM * 4080u; k++) {
            jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
            jas_bank_bad[board][k] = 0;
        }
        for (unsigned a = 0; a < ASIC_NUM; a++)
            if (ASIC_EN(board) >> a & 1)
                config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
          sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }
        en = ASIC_EN(board);

        for (v = 0; v < 4; v++) {
            uint32_t waddr = 400 + v * 20;
            uint32_t pattern = 0xC0FFEE00u + v;
            uint32_t a32 = waddr;
            unsigned found = 0;

            for (w = 0; w < 16; w++) ((uint32_t *)sram)[w] = pattern;

            sl_multi_write(en, cmd, 0x800, (uint16_t *)&a32, 2, board);
            if (v == 3) { sl_single_write(en, cmd, 0x822, 0, board);
                          sl_single_write(en, cmd, 0x822, 1, board); }
            else if (v != 2) sl_single_write(en, cmd, 0x822, 1, board);

            sl_multi_write(en, cmd, 0x802, sram, 0x20, board);

            if (v == 1) sl_single_write(en, cmd, 0x822, 0, board);
            if (v == 2) sl_single_write(en, cmd, 0x822, 1, board);

            /* wait for ready, chip 0 */
            for (k = 0; k < 200; k++) {
                uint16_t rdy = 0;
                sl_single_read(1, 0x1c, 0x824, &rdy, 1000000, board);
                if (rdy) break;
                usleep(1000);
            }

            /* search for the pattern in the two mappings */
            for (k = waddr / 2; k <= waddr + 2; k++) {
                uint32_t got = 0;
                if (k == waddr / 2 + 3) k = waddr - 2;
                memset(back, 0, sizeof(back));
                if (single_manual_read(1, 0, k, back, board) < 0) continue;
                memcpy(&got, back, 4);
                if (got == pattern) {
                    printf("  V%u: *** FOUND *** write %u -> read %u\n", v, waddr, k);
                    found = 1;
                }
            }
            if (!found) printf("  V%u: pattern 0x%08x not found (ready after %u loops)\n",
                                v, pattern, k);
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--cbvar") == 0) {
        if (jas_require_session() < 0) return 1;
        /*
         * SINGLE-VARIABLE CAMPAIGN, MEASURED BY THE SCAN, the only instrument
         * known to measure the same thing as in the firmware (a coverage per
         * ALU). The preamble is done once and only the fill changes between
         * variants; the control runs first AND last, to check that the
         * measurement does not drift.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned k;
        int order[] = { 0, 0 };
        const char *name[] = { "control  2n and 2n+1, same block",
                              "a single block at 2n",
                              "n and n+1, no doubling",
                              "2n and 2n+1, different blocks" };

        jas_bus_lock();
        printf("\n  preamble\n");
        sl_single_write(0xff, 0xf9, 0x830, 0, board);
        sl_delay(0, 2000000);
        dft_reset(board, 0, 0);
        sl_delay(0, 3000000);
        sl_single_write(0xff, 0xf9, 0x830, ASIC_EN(board), board);
        sl_delay(0, 2000000);
        spi_check(board);

        /*
         * REFUSE TO MEASURE IF THE LINK IS NOT CALIBRATED: with all of
         * spi_check's phases rejected, the chip does not answer and the bus
         * floats, which reads as a perfect "4080 banks per ALU".
         */
        {
            unsigned a, silent = 0;

            for (a = 0; a < ASIC_NUM; a++)
                if ((ASIC_EN(board) >> a & 1) && jas_spi_status[board][a] != 1)
                    silent++;
            if (silent) {
                printf("\n  STOP: %u asic(s) with no valid phase: the link is\n"
                       "  not calibrated, no later measurement would mean"
                       " anything.\n", silent);
                return 1;
            }
        }

        sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
        /*
         * THE SEEDING OF THE BANKS IS THE VARIABLE UNDER TRIAL.
         * self_check_func calls config_remap_info BEFORE the memory test, on
         * banks still all at zero, so the remap table is EMPTY; declaring them
         * good first fills it, and the two give opposite regimes.
         * JASRE_SEED=0 reproduces the firmware's order.
         */
        {
            const char *sd = getenv("JASRE_SEED");
            int seed = !(sd && *sd == '0');

            printf("  bank seeding: %s\n", seed ? "YES (full table)"
                                                               : "NO (empty table, like the firmware)");
            if (seed)
                for (k = 0; k < ASIC_NUM * 4080u; k++) {
                    jas_bank_good[board][k] = 1; jas_bank_partial[board][k] = 1;
                    jas_bank_bad[board][k] = 0;
                }
        }
        /*
         * WHEN the table is written is a variable in its own right: an empty
         * table gives massive timeouts and a random result; a full one (all
         * good) a deterministic result but few banks per ALU. The firmware
         * calls config_remap_info with its REAL verdicts: setting them first
         * and writing the table AFTER the DFT reproduces that state.
         */
        if (!getenv("JASRE_REMAP_AFTER"))
            for (unsigned a = 0; a < ASIC_NUM; a++)
                if (ASIC_EN(board) >> a & 1)
                    config_remap_info(board, (uint8_t)a, 0);
        config_asic_fix_param(board);
        alu_check(board);
        alu_check_analysis(board);

        /*
         * The memory test MUST come before the scan: without it the control
         * returns zero. It is not a precaution, it is a dependency: the DFT
         * leaves the chips in the state the scan expects (repairs burnt, post
         * mode, bist_l).
         */
        /*
         * Each run burns repairs into OTP. If the scan holds without the DFT
         * under the identity table, we stop touching them at each trial, and
         * the trials go from 150 s to 25 s.
         */
        if (getenv("JASRE_SKIP_DFT")) {
            printf("  memory test SKIPPED (JASRE_SKIP_DFT)\n");
        } else {
            printf("  memory test (needed by the sweep)\n");
            dft_short_pattern_test_110(board);
        }
        if (getenv("JASRE_REMAP_AFTER")) {
            printf("  remap table written AFTER the dft, on the real verdicts\n");
            for (unsigned a = 0; a < ASIC_NUM; a++)
                if (ASIC_EN(board) >> a & 1)
                    config_remap_info(board, (uint8_t)a, 0);
        }
        printf("  preamble done; firmware reference on this board: 3872\n\n");

        for (k = 0; k < sizeof(order)/sizeof(order[0]); k++) {
            unsigned long total = 0;
            unsigned alu, w;

            jas_cb_variant = order[k];
            memset(jas_cb_sc_result[board][0], 0, 120 * 255 * sizeof(uint16_t));

            /* reset then controllers ON: established, invariant */
            sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0, board);
            sl_delay(0, 2000);
            sl_single_write(ASIC_EN(board), 0xf8, 0x830, 0xffff, board);
            { uint16_t buf[64]; memset(buf, 0xff, sizeof(buf));
              sl_multi_write(ASIC_EN(board), 0xf8, 0xc00, buf, 0x40, board);
              sl_multi_write(ASIC_EN(board), 0xf8, 0xc40, buf, 0x40, board); }

            {
                struct timeval a, b;
                long ms;

                jas_mw_calls = jas_mw_polls = 0;
                jas_mw_ready_immediate = jas_mw_us_wait = 0;
                jas_mw_mismatches = 0;
                gettimeofday(&a, NULL);
                /*
                 * With the identity table, the scan is finally DETERMINISTIC
                 * (gap of 1 in 489,600 between two passes) and returns 68
                 * banks. Not filling at all answers the next question: do
                 * these 68 depend on what we write? If the figure does not
                 * move, the scan does not read where we write.
                 */
                if (!getenv("JASRE_CB_NOFILL"))
                    crossbar_fill_pub(board);
                else
                    printf("  fill SKIPPED (JASRE_CB_NOFILL)\n");
                gettimeofday(&b, NULL);
                ms = (b.tv_sec - a.tv_sec) * 1000L + (b.tv_usec - a.tv_usec) / 1000;
                printf("  fill: %ld ms for %lu calls\n", ms, jas_mw_calls);
                printf("  0x824 probes: %lu total, %lu calls ready on the 1st\n",
                       jas_mw_polls, jas_mw_ready_immediate);
                printf("  cumulated wait: %lu us\n", jas_mw_us_wait);
                printf("  1st/2nd probe mismatches: %lu\n", jas_mw_mismatches);
                printf("  (firmware reference: 86 000 ms for 65280 calls)\n");
            }
            crossbar_config_pub(board);
            crossbar_run_pub(board, 0);

            for (alu = 0; alu < 120; alu++)
                for (w = 0; w < 255; w++) {
                    uint16_t x = jas_cb_sc_result[board][0][alu][w];
                    while (x) { total += x & 1; x >>= 1; }
                }
            printf("  V%d  %-32s -> %lu banks per ALU\n",
                   order[k], name[order[k]], total / 120);

            /*
             * Dump the raw record after each pass of the control.
             *
             * The coverage is stable (3464 twice in a row) but nothing says
             * these are the SAME banks. If so, a deterministic subset is
             * unreachable; if not, the failure is random per access. The two
             * call for opposite investigations, and this is the only
             * measurement that separates them.
             */
            {
                char fname[64];
                FILE *fd;

                snprintf(fname, sizeof(fname), "/tmp/cb_run%u.bin", k);
                fd = fopen(fname, "wb");
                if (fd) {
                    fwrite(jas_cb_sc_result[board][0], sizeof(uint16_t),
                           120 * 255, fd);
                    fclose(fd);
                    printf("      raw readout: %s\n", fname);
                }
            }
        }
        jas_bus_unlock();
    }
    if (argc > 1 && strcmp(argv[1], "--state") == 0) {
        /*
         * READ THE CHIPS' STATE, DIVERTING NOTHING: the registers the memory
         * test leaves behind are readable. They are read after the firmware's
         * self-test, then after ours, and compared. No write, no risk.
         */
        int board = (argc > 2) ? atoi(argv[2]) : 0;
        unsigned asic = (argc > 3) ? (unsigned)strtoul(argv[3], NULL, 0) : 0;
        unsigned mesh = (argc > 4) ? (unsigned)strtoul(argv[4], NULL, 0) : 0;
        static const uint16_t regs[] = {
            0x0d00, 0x0d01, 0x0d02, 0x0d03, 0x0d07, 0x0d20, 0x0d28,
            0x0d2a, 0x0d2b, 0x0d2c, 0x0d2d, 0x0d30, 0x0d31, 0x0d32,
            0x0d43, 0x0d53, 0x0d54, 0x0d59, 0x0d5a, 0x0d5b,
            0x0d60, 0x0d61, 0x0d62, 0x0d63, 0x0d6d, 0x0d6f,
            0x0500, 0x0502, 0x0540, 0x0580, 0x05bc, 0x05e0, 0x06a5,
            0x0800, 0x0822, 0x0824, 0x0830, 0x0c00, 0x0c40, 0x103c,
            0x103e, 0x1000, 0x0080, 0x0004, 0x0008,
        };
        uint8_t en = (uint8_t)(1u << asic);
        uint8_t cmd_r = (uint8_t)(((1u << mesh) << 4) | 0xcu);
        unsigned k;

        printf("\n  state of board %d asic %u mesh %u (read only)\n",
               board, asic, mesh);
        jas_bus_lock();
        for (k = 0; k < sizeof(regs)/sizeof(regs[0]); k++) {
            uint16_t v = 0;
            int rc = sl_single_read(en, cmd_r, regs[k], &v, 1000000, board);

            printf("  0x%04x = %s0x%04x\n", regs[k],
                   rc < 0 ? "(error) " : "", v);
        }
        jas_bus_unlock();
    }
    /* ==================================================================
     * TEST LADDER: from the riskless to the complete.
     *
     * The rule learnt the hard way: STATE THE EXPECTED EFFECT BEFORE
     * STARTING. A trial whose success is decided afterwards measures
     * nothing. Each mode below prints its expectation first.
     * ================================================================== */

    /*
     * --survey: STEP 1, the hardware startup and its readout. Not passive:
     * sl_wrapper_init calls find_borad_and_poweron, which configures the
     * PLLs and sets the voltages (as the firmware does at each startup).
     *
     * EXPECTED, line by line:
     *   zynq PL version 0x.. >= 0x50, "master chip use zynq7020-400"
     *   BOARD_NUM = 3, board_en = 0x0001  (a single daughter board)
     *   pll_cfg = 198 per chip            (0x198 = 408, for 510 MHz)
     *   real_freq = 510.00 MHz            (pll_clk_test = 6375)
     *   temp between 25 and 50
     *
     * It validates the Zynq mapping, jas_hal_init, sl_multi_read and
     * get_current_status.
     */
    if (argc > 1 && strcmp(argv[1], "--survey") == 0) {
        if (jas_require_session() < 0)
            return 1;
        printf("\n  STEP 1 - survey, read only\n");
        printf("  expected: zynq version >= 0x50, BOARD_NUM 3, board_en 0x0001,\n");
        printf("            pll_cfg 198, real_freq 510.00 MHz, temp 25-50\n\n");
        sl_wrapper_init();
        printf("\n  --- state ---\n");
        get_current_status();
        printf("\n  BOARD_NUM = %u, board_en = 0x%04x, asic_en[0] = 0x%02x\n",
               BOARD_NUM, BOARD_EN, ASIC_EN(0));
        return 0;
    }

    /*
     * --cache <block>: STEP 2, the processor's light cache, ALONE. It does
     * not touch the serial bus (it does map the AXI windows, as the
     * program's initialization always does), and runs on the workstation
     * as well as on the target: it isolates the speed and correctness of
     * our keccak.
     *
     * Reference, from the firmware's log:
     *     "cpu cache_zil calc done. block number is 24810000,
     *       size 125173696, time diff 151"
     * i.e. 1.21 s per megabyte.
     */
    if (argc > 1 && strcmp(argv[1], "--cache") == 0) {
        uint64_t block = (argc > 2) ? strtoull(argv[2], NULL, 0) : 30000ull;
        uint64_t size = ethash_get_cachesize(block);
        struct timeval d, f;
        ethash_light_t light;
        double elapsed_s;

        printf("\n  STEP 2 - processor cache alone, block %llu\n",
               (unsigned long long)block);
        printf("  size %llu B (%.1f MiB), %llu nodes\n",
               (unsigned long long)size, size / 1048576.0,
               (unsigned long long)(size / 64));
        printf("  firmware reference: 1.21 s/MiB\n\n");

        gettimeofday(&d, NULL);
        light = ethash_light_new(block);
        gettimeofday(&f, NULL);
        if (!light) {
            printf("  FAILED: allocation refused\n");
            return 1;
        }
        elapsed_s = (f.tv_sec - d.tv_sec) + (f.tv_usec - d.tv_usec) / 1e6;
        printf("  built in %.1f s  -  %.2f s/MiB\n",
               elapsed_s, elapsed_s / (size / 1048576.0));

        /*
         * Is the cache RIGHT? Speed is useless if the content is wrong. A DAG
         * item recomputed from this cache must be reproducible: it is computed
         * twice and compared, then two different indices are checked to give
         * different nodes; a null cache would pass the first check, not the
         * second.
         */
        {
            node a1, a2, b1;
            ethash_calculate_dag_item(&a1, 12345, light);
            ethash_calculate_dag_item(&a2, 12345, light);
            ethash_calculate_dag_item(&b1, 12346, light);
            printf("  reproducible:   %s\n",
                   memcmp(&a1, &a2, sizeof(a1)) == 0 ? "yes" : "NO");
            printf("  non-degenerate: %s\n",
                   memcmp(&a1, &b1, sizeof(a1)) != 0 ? "yes" : "NO");
            printf("  item 12345:     %08x %08x %08x %08x\n",
                   a1.words[0], a1.words[1], a1.words[2], a1.words[3]);
        }
        ethash_light_delete(light);
        return 0;
    }

    /*
     * --pattern: write a pattern where each byte is distinct, read it back,
     * and look at the gap: unlike "real" values (a size, an address), it
     * leaves no ambiguity about a byte shift.
     */
    if (argc > 1 && strcmp(argv[1], "--pattern") == 0) {
        static const uint8_t pattern[8] = { 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88 };
        uint16_t readback[8];
        unsigned k, n;

        if (jas_require_session() < 0)
            return 1;
        printf("\n  WRITING A PATTERN AND READING IT BACK\n");
        sl_wrapper_init();

        for (n = 1; n <= 4; n++) {
            memset(readback, 0, sizeof(readback));
            sl_multi_write(1, 0xf8, 0x04, (uint16_t *)pattern, n, 0);
            sl_multi_read(1, 0x1c, 0x04, readback, n, 1000000, 0);
            printf("  %u words: wrote ", n);
            for (k = 0; k < n * 2u; k++) printf("%02x ", pattern[k]);
            printf(" | read back ");
            for (k = 0; k < n * 2u; k++) printf("%02x ", ((uint8_t *)readback)[k]);
            printf("\n");
        }
        return 0;
    }

    /*
     * --read (alone): WHERE IS THE SHIFT? Registers written at 0xf8 read back
     * divided by 256 (leading byte undefined), while pll_lock, pll_clk_test
     * and logic_version, read through command 0xfd and not 0x1c, read back
     * right: the shift may come from the read command, not the register.
     * This trial reads the SAME thing in several ways; it writes only
     * register 0x04.
     */
    if (argc > 1 && strcmp(argv[1], "--read") == 0) {
        uint16_t v;

        if (jas_require_session() < 0)
            return 1;
        printf("\n  WHERE IS THE OFFSET?\n\n");
        sl_wrapper_init();

        printf("  --- known references, read as usual (cmd 0xfd)\n");
        v = 0; sl_single_read(0xff, 0xfd, 0x0011, &v, 1000000, 0);
        printf("      0x0011 pll_lock       = 0x%04x   (expected 0x0007)\n", v);
        v = 0; sl_single_read(0xff, 0xfd, 0x7000, &v, 1000000, 0);
        printf("      0x7000 logic_version  = 0x%04x   (expected 0x0500)\n", v);

        printf("\n  --- the SAME, read by a mesh (cmd 0x1c)\n");
        v = 0; sl_single_read(1, 0x1c, 0x0011, &v, 1000000, 0);
        printf("      0x0011 pll_lock       = 0x%04x\n", v);
        v = 0; sl_single_read(1, 0x1c, 0x7000, &v, 1000000, 0);
        printf("      0x7000 logic_version  = 0x%04x\n", v);

        printf("\n  --- writing a known word at 0x04, then reading back\n");
        sl_single_write(1, 0xf8, 0x04, 0x1234, 0);
        v = 0; sl_single_read(1, 0x1c, 0x04, &v, 1000000, 0);
        printf("      wrote 0x1234 (cmd 0xf8), read back by 0x1c = 0x%04x\n", v);
        v = 0; sl_single_read(1, 0xfd, 0x04, &v, 1000000, 0);
        printf("      wrote 0x1234 (cmd 0xf8), read back by 0xfd = 0x%04x\n", v);

        sl_single_write(1, 0x18, 0x04, 0x5678, 0);
        v = 0; sl_single_read(1, 0x1c, 0x04, &v, 1000000, 0);
        printf("      wrote 0x5678 (cmd 0x18), read back by 0x1c = 0x%04x\n", v);

        /*
         * OBSERVE RATHER THAN DEDUCE. A recognizable value is written, a read
         * per mesh is started, and the AXI window is dumped word by word. The
         * value is somewhere: better look where than assume a shift.
         */
        {
            volatile uint32_t *w = (volatile uint32_t *)
                ((volatile unsigned char *)jas_axi_base(0) + 0x20000u);
            unsigned k;

            sl_single_write(1, 0xf8, 0x04, 0xa5c3, 0);
            printf("\n  --- raw axi window after a per-mesh read\n");
            printf("      (value written: 0xa5c3, command 0x1c, asic 1)\n");
            sl_single_write(1, 0x1c, 0x04, 0, 0);
            for (k = 0; k < 12; k++)
                printf("      +%-3u  0x%08x\n", k * 4, w[k]);
        }
        return 0;
    }

    /*
     * --miner: THE MINER, end to end (also the default, see below).
     *
     * sl_wrapper_init, the self-test, dag_calc, then the threads: monitoring
     * and the mining loop; the pools have their own network threads, and the
     * monitoring refreshes the fans every second.
     *
     * EXPECTED: "calc dag OK!" on all three chips, then work from the pool
     * and nonces coming up; the fan setting follows the temperature.
     *
     * DO NOT INTERRUPT during the DAG construction.
     *
     *   JASRE_POOL=<host>:3333 /tmp/jasre
     */
    /*
     * MINING IS THE DEFAULT BEHAVIOR, as in the original firmware:
     * /etc/init.d/jasminer.sh starts the binary with "-c <config>" only, so
     * no mode on the command line must mean "mine", not "exit". "--miner"
     * is still accepted.
     */
    {
        uint64_t block;
        int board;
        unsigned chip;
        jas_work first;

        if (jas_require_session() < 0)
            return 1;

        /*
         * ASK FOR THE WORK BEFORE BUILDING ANYTHING, in the firmware's order:
         * the seed arrives, the epoch is deduced from it, and only then are
         * the cache and the DAG built (a DAG for the wrong epoch passes every
         * check and makes every share wrong).
         */
        /*
         * THE NETWORK THREAD FIRST. jas_work_current no longer queries the
         * pool: it is a pure consumer, which takes what the thread dropped off.
         * Starting it afterwards would have blocked the startup: with nobody
         * reading the socket, main would have waited for a work that would
         * never come.
         */
        {
            pthread_t pool_thread;
            int np;

            /* JASRE_MAP="1,0,1" -> chip 0 on pool 1, chip 1 on 0, chip 2 on
             * 1. Absent: everyone on pool 0. */
            {
                const char *m = getenv("JASRE_MAP");

                /* JASRE_MAP wins for trials; otherwise the configuration,
                 * where the web page writes "asic_pool_group". */
                if (m && *m) jas_pool_assign(m);
                else         read_public_assignment_config();
            }

            /*
             * ONE THREAD PER CONFIGURED SLOT, backups included, in the order
             * [g0 main, g0 backup, g1 main, g1 backup]. A backup's thread stays
             * in reserve, without a socket, until its group switches to it: the
             * switch then costs a connection and a login, never the restart of
             * the whole miner that the shell watchdog would do.
             *
             * A slot without an address gets no thread: it would fail in a loop
             * every five seconds and flood the log.
             */
            for (np = 0; np < jas_pool_count(); np++) {
                if (!jas_pool_configure(np))
                    continue;
                if (pthread_create(&pool_thread, NULL, jas_pool_thread,
                                   jas_pool_n(np)) != 0) {
                    printf("  cannot start network thread %d\n", np);
                    return 1;
                }
                pthread_detach(pool_thread);
            }

            /*
             * The miner's API, port 4028: the one the web page queries,
             * through lighttpd then jasminer_api. Detached: if the port is
             * already taken, the thread says so and withdraws, mining goes
             * on. A silent page is no reason to stop mining.
             */
            {
                pthread_t web_thread;

                if (pthread_create(&web_thread, NULL, jas_web_thread, NULL) != 0)
                    jas_log("api: cannot start web thread");
                else
                    pthread_detach(web_thread);
            }
        }

        printf("\n  MINER - asking the pool for work first\n");
        memset(&first, 0, sizeof(first));
        for (board = 0; board < 60; board++) {
            if (jas_work_current(&first))
                break;
            sleep(1);
        }
        if (!first.block_number) {
            printf("  no work from the pool, stopping\n");
            return 1;
        }
        block = first.block_number;
        printf("  epoch %llu (block %llu), work %s\n\n",
               (unsigned long long)(block / 30000), (unsigned long long)block,
               first.work_id);
        printf("  DO NOT INTERRUPT during the fill\n\n");

        /*
         * EACH CHIP'S CONTEXT BEFORE THE DAG CONSTRUCTION.
         *
         * seed_update_operation builds the DAG from jas_coin_of(board, chip),
         * so the contexts must ALREADY carry the right coin. If they were all
         * left on pool 0, the chips assigned to pool 1 would then receive ETC
         * work with an ETHW DAG, and nothing would rebuild it: all their nonces
         * would be wrong.
         *
         * So each pool is waited for until it has served a job before building.
         */
        for (chip = 0; chip < 3; chip++) {
            jas_work tp;
            int attempts;

            for (attempts = 0; attempts < 60; attempts++) {
                if (jas_work_of_chip(chip, &tp))
                    break;
                sleep(1);
            }
            if (attempts == 60) {
                printf("  asic %u: pool %d silent, stopping\n",
                       chip, jas_pool_of_chip(chip));
                return 1;
            }
            for (board = 0; board < 3; board++) {
                jas_coin_ctx *c = jas_coin_of(board, chip);
                jas_coin_from_block_seed(c, tp.block_number, &tp.seed);
                c->header = tp.header;
                c->target = tp.target;
            }
            printf("  asic %u -> pool %d, epoch %llu\n", chip,
                   jas_pool_of_chip(chip),
                   (unsigned long long)(tp.block_number / 30000));
            /* This coin's processor cache, if it does not exist yet. */
            jas_cache_request(tp.block_number, &tp.seed);
        }

        /*
         * THE PROCESSOR CACHE IS BUILT IN ITS OWN THREAD (jas_cache_request,
         * above), off the critical path: it only serves to JUDGE
         * (dram_check_dag, ethash_nonce_check), and dag_calc waits for it per
         * chip, just before dram_check_dag. Meanwhile the bring-up moves on.
         */
        /*
         * No global "processor cache" thread: jas_cache_request, called above
         * for EACH chip, already builds each coin's cache into the registry,
         * deduplicated on (block, seed). A global one would build a third
         * cache, compete for the single core, and free an entry the chips
         * are reading.
         */
        *jas_fpga_block_number = block;
        *jas_fpga_dag_size = jas_coin_of(0, 0)->dag_size;
        *jas_coin_type = 0;

        sl_wrapper_init();
        jas_sensors_configure(0);

        /*
         * THREADS FIRST, BRING-UP AFTERWARDS, in the firmware's order (its
         * mining thread starts at 21 s, its self-test ends at 4 min 51). The
         * network threads read the pools' sockets from the start; otherwise
         * everything pushed during the bring-up piles up and the first
         * jas_work_current drains it for minutes. The mining thread touches
         * nothing until jas_chips_ready is called; the monitoring thread
         * regulates the fans right away, like the original, whose
         * sensor_thread_create is in find_borad_and_poweron.
         */
        printf("\n  --- starting the threads\n");
        creat_pthread();

        seed_update_operation();
        jas_chips_ready();
        printf("\n  --- asics ready, mining can start\n");

        /*
         * The main thread displays the state. IT DOES NOT TOUCH THE BUS: a
         * probe taking the bus lock here (e.g. reading register 0x43 of each
         * ALU, with a one-second timeout each) would starve the mining thread
         * of the link. What is displayed comes from the counters
         * sl_wrapper_work already keeps: if `seen` moves, the chips produce
         * and report.
         */
        {
            for (;;) {
                unsigned seen = 0, submitted = 0, rej = 0, stale = 0;
                int s;

                /* In one-second slices, to see the stop coming. */
                for (s = 0; s < 10 && !jas_stop_request; s++)
                    sleep(1);
                if (jas_stop_request) {
                    printf("\n  signal %d received - powering the asics down before exit\n",
                           (int)jas_stop_request);
                    fflush(stdout);
                    power_off_operation();
                    printf("  asics powered down, clean exit\n");
                    return 0;
                }
                jas_mining_counters(&seen, &submitted, &rej, &stale);

                {
                    /*
                     * THE LOOP'S RATE, published because a slow loop pushes
                     * few of the jobs received, and the nonces found meanwhile
                     * carry a header the pool has already forgotten (refused
                     * as stale). It tells a harvest problem from a read that
                     * times out.
                     */
                    static unsigned rounds_prec;
                    unsigned t2 = jas_mining_rounds();

                    printf("  loop: %u loops in 10 s (%.1f ms/loop)\n",
                           t2 - rounds_prec,
                           (t2 > rounds_prec) ? 10000.0 / (t2 - rounds_prec) : 0.0);
                    rounds_prec = t2;
                }
                printf("  temp %d/%d  pwm %d  work %s  nonces %u  submitted %u"
                       "  rejected %u  stale %u\n",
                       jas_temp_min, jas_temp_max, jas_pwm_current(),
                       jas_work_g.work_id, seen, submitted, rej, stale);
                fflush(stdout);
            }
        }
    }

    /*
     * --fan [n]: regulation, running.
     *
     * EXPECTED: the setting must HOLD at the value the curve dictates,
     * instead of falling back to 100. At 33-42 degrees the curve gives 42 to
     * 51 %. If the setting stays stuck at 100, the refresh is not enough.
     */
    if (argc > 1 && strcmp(argv[1], "--fan") == 0) {
        unsigned n = (argc > 2) ? (unsigned)strtoul(argv[2], NULL, 0) : 20;
        volatile uint32_t *f0, *f1;
        unsigned k;

        if (jas_require_session() < 0)
            return 1;
        printf("\n  REGULATION - expected: the setpoint follows the curve,\n");
        printf("  and does not fall back to 100\n\n");
        sl_wrapper_init();
        jas_sensors_configure(0);
        sleep(1);
        f0 = jas_fan_p[0];
        f1 = jas_fan_p[1];

        printf("  time   temp0 temp1 temp2   pwm  register  rpm\n");
        for (k = 0; k < n; k++) {
            unsigned t;

            jas_sensors_read(0);
            set_PWM_according_to_temperature_x16();
            printf("  %4us   %5d %5d %5d   %3d   %3u/%-3u   %5u %5u\n",
                   k * 3, (int)jas_asic_temp[0][0], (int)jas_asic_temp[0][1],
                   (int)jas_asic_temp[0][2], jas_pwm_current(),
                   f0 ? f0[0] : 0, f1 ? f1[0] : 0,
                   f0 ? f0[2] * 30u : 0, f1 ? f1[2] * 30u : 0);
            fflush(stdout);
            /* Refresh at 1 Hz during the interval, as the thread would. */
            for (t = 0; t < 3; t++) { set_PWM_according_to_temperature_x16(); sleep(1); }
        }
        return 0;
    }

    /*
     * --pool: try the network layer, without touching the hardware.
     *
     * EXPECTED: a connection, then work whose header and seed are the ones
     * the pool announced, and a block number DEDUCED from the seed (30000
     * for epoch 1). A wrong block means ethash_get_blocknum is mistaken,
     * not the network.
     *
     *   JASRE_POOL=<host>:3333 /tmp/jasre --pool 5
     */
    if (argc > 1 && strcmp(argv[1], "--pool") == 0) {
        unsigned n = (argc > 2) ? (unsigned)strtoul(argv[2], NULL, 0) : 3;
        unsigned k, ok = 0;
        jas_work t;

        printf("\n  POOL - expected: block 30000 derived from the seed\n\n");
        for (k = 0; k < n; k++) {
            memset(&t, 0, sizeof(t));
            if (!jas_work_current(&t)) {
                printf("  %u: no work\n", k);
                sleep(1);
                continue;
            }
            ok++;
            printf("  %u: block %llu  header %02x%02x%02x%02x  seed"
                   " %02x%02x%02x%02x  target %02x%02x%02x%02x\n", k,
                   (unsigned long long)t.block_number,
                   t.header.b[0], t.header.b[1], t.header.b[2], t.header.b[3],
                   t.seed.b[0], t.seed.b[1], t.seed.b[2], t.seed.b[3],
                   t.target.b[0], t.target.b[1], t.target.b[2], t.target.b[3]);
        }
        /*
         * Do NOT compare with a hard-coded value: 30000 is the fake pool's
         * block, a real one serves another. The useful check is that the
         * block is an epoch boundary and that the work changes.
         */
        printf("\n  %u/%u jobs obtained, block %llu%s\n", ok, n,
               (unsigned long long)t.block_number,
               (t.block_number % 30000) ? "  <<< NOT AN EPOCH BOUNDARY" : "");

        /* A dummy submission: the pool answers true, which tries the send
         * path and the reading of the answer. */
        if (ok) {
            gooddata_detail d;
            memset(&d, 0, sizeof(d));
            d.good_data.nonce = 0x0123456789abcdefull;
            memcpy(jas_fpga_header, t.header.b, 32);
            /* A made-up nonce: a real pool MUST refuse it. What is tried here
             * is the send path and the matching of the answer, not the share's
             * validity. */
            printf("  test submission (made-up nonce, rejection expected)...\n");
            jas_submit(&d);
        }
        return ok ? 0 : 1;
    }

    /*
     * --fanwatch [n]: watch the fans while the FIRMWARE runs. It does not
     * touch the serial bus (only the two fan windows), so it coexists with
     * the miner (hence no jas_require_session). It shows whether the firmware
     * varies the setting as it heats up, and whether the tachometer follows.
     *
     *   columns 1 and 4: the setting (offset 0 of each window)
     *   columns 2,3 and 5,6: the tachometers, x30 for revolutions/minute
     */
    if (argc > 1 && strcmp(argv[1], "--fanwatch") == 0) {
        unsigned n = (argc > 2) ? (unsigned)strtoul(argv[2], NULL, 0) : 40;
        volatile uint32_t *f0, *f1;
        unsigned k;

        if (jas_fan_map_init() != 0) {
            printf("  fan windows unreachable\n");
            return 1;
        }
        f0 = jas_fan_p[0];
        f1 = jas_fan_p[1];
        /*
         * A setting can be given as 3rd argument. A miner's fans have
         * considerable inertia: measuring fifteen seconds after a change gives
         * the speed WHILE IT IS CHANGING, not the settled value. It is the
         * error that led to the wrong conclusion that the command did nothing.
         */
        /*
         * REFRESH, do not set once: written once, the setting goes back to
         * 100 within five seconds (probably a fan watchdog in the FPGA: no
         * refresh, full speed). So it is refreshed every second, as a
         * regulation thread would.
         */
        int setpoint = (argc > 3) ? atoi(argv[3]) : -1;

        if (setpoint >= 0)
            printf("\n  setpoint %d, refreshed every second\n", setpoint);
        printf("\n  setpoint0  rpm         setpoint1  rpm\n");
        for (k = 0; k < n; k++) {
            printf("  %8u   %5u %5u    %8u   %5u %5u\n",
                   f0 ? f0[0] : 0, f0 ? f0[2] * 30u : 0, f0 ? f0[3] * 30u : 0,
                   f1 ? f1[0] : 0, f1 ? f1[2] * 30u : 0, f1 ? f1[3] * 30u : 0);
            fflush(stdout);
            {
                unsigned t;
                for (t = 0; t < 5; t++) {
                    if (setpoint >= 0)
                        fan_speed_set(setpoint);
                    sleep(1);
                }
            }
        }
        return 0;
    }

    /*
     * --sensors: try temperatures, fans and voltages.
     *
     * EXPECTED:
     *
     *   temperature   between 20 and 60, and different from 32 on at least
     *                 one chip (32 is jas_asic_temp's initialization value:
     *                 three exact 32s mean the sensor is not being read).
     *   fans          non-zero, and the speed must FOLLOW the setting: it is
     *                 changed, measured again, set back.
     *   voltages      around 860 mV, what mcp2973_set_vol read back.
     */
    if (argc > 1 && strcmp(argv[1], "--sensors") == 0) {
        int t[9], fan[4], fan2[4];
        unsigned k;
        uint16_t vol[9];

        if (jas_require_session() < 0)
            return 1;
        printf("\n  SENSORS\n");
        printf("  expected: temp 20-60 and != 32, fans non-zero and\n");
        printf("            responsive to the setpoint, voltages ~860 mV\n\n");

        sl_wrapper_init();

        printf("  --- before any read (initial value)\n   ");
        for (k = 0; k < 3; k++)
            printf(" asic%u=%d", k, (int)jas_asic_temp[0][k]);
        printf("\n");

        jas_sensors_configure(0);
        sleep(1);
        jas_sensors_read(0);

        memset(t, 0, sizeof(t));
        get_temp_sensor_value(t);
        printf("\n  --- temperatures read\n   ");
        for (k = 0; k < 3; k++)
            printf(" asic%u=%d", k, t[k]);
        printf("\n");
        {
            int change = 0, plausible = 1;
            for (k = 0; k < 3; k++) {
                if (t[k] != 32) change = 1;
                if (t[k] < 20 || t[k] > 60) plausible = 0;
            }
            printf("      real reading (differs from 32): %s\n", change ? "YES" : "NO");
            printf("      plausible (20-60):              %s\n", plausible ? "yes" : "no");
        }

        /*
         * THE FAN TEST MUST BE ABLE TO FAIL: two settings clearly apart,
         * fifteen seconds for the fan's inertia, and a gap of at least 15 %
         * required (a few % is measurement noise).
         */
        printf("\n  --- fans\n");
        /*
         * DOES OUR WRITE REACH THE REGISTER? Before concluding that the
         * command does nothing, check that it arrives. Reading back what was
         * just written tells "the hardware ignores the setting" from "the
         * setting is never set", two unrelated causes.
         */
        {
            volatile uint32_t *f0 = jas_fan_p[0], *f1 = jas_fan_p[1];

            printf("      register before:   %08x / %08x\n",
                   f0 ? f0[0] : 0, f1 ? f1[0] : 0);
            fan_speed_set(20);
            printf("      after writing 20:  %08x / %08x\n",
                   f0 ? f0[0] : 0, f1 ? f1[0] : 0);
            fan_speed_set(100);
            printf("      after writing 100: %08x / %08x\n",
                   f0 ? f0[0] : 0, f1 ? f1[0] : 0);
        }
        fan_speed_set(20);
        sleep(15);
        fan_speed_read(fan);
        printf("      setpoint  20: %d %d %d %d rpm\n",
               fan[0], fan[1], fan[2], fan[3]);
        fan_speed_set(100);
        sleep(15);
        fan_speed_read(fan2);
        printf("      setpoint 100: %d %d %d %d rpm\n",
               fan2[0], fan2[1], fan2[2], fan2[3]);
        {
            int answer = 0, present = 0;

            for (k = 0; k < 4; k++) {
                int low = fan[k], high = fan2[k];

                if (low == 0 && high == 0)
                    continue;                    /* fan absent */
                present++;
                if (low > 0 && (high - low) * 100 / low >= 15)
                    answer++;
            }
            printf("      %d fan(s) present, %d respond to the setpoint\n",
                   present, answer);
            printf("      VERDICT: %s\n",
                   present && answer == present ? "the command works"
                                                     : "THE COMMAND HAS NO EFFECT");
        }
        printf("      back to 40\n");
        fan_speed_set(40);

        printf("\n  --- voltages (mode 0x%03x)\n", jas_pmbus_mode);
        memset(vol, 0, sizeof(vol));
        asic_pmbus_read_vol(jas_pmbus_mode, 0, vol);
        for (k = 0; k < 3; k++)
            printf("      asic%u rail0 = %u mV\n", k, vol[k * 3]);
        return 0;
    }

    /*
     * --cacheasic <block>: STEP 2b, the cache IN THE CHIP, alone: no DFT,
     * no crossbar, no DAG, all banks declared good (the simplest remap
     * table), one minute per trial. If the chip reports the end here, the
     * programming sequence is right and a step 3 failure comes from what
     * the self-test leaves behind; if not, the sequence itself is at fault.
     *
     * EXPECTED: "done=1" in less than five seconds, like the firmware.
     */
    if (argc > 1 && strcmp(argv[1], "--cacheasic") == 0) {
        uint64_t block = (argc > 2) ? strtoull(argv[2], NULL, 0) : 30000ull;
        int board = 0;
        unsigned chip, k, t;

        if (jas_require_session() < 0)
            return 1;

        printf("\n  STEP 2b - cache in the asic, alone. block %llu\n",
               (unsigned long long)block);
        printf("  all banks declared good, trivial table\n");
        printf("  expected: done=1 in less than 5 s\n\n");

        for (chip = 0; chip < 3; chip++)
            jas_coin_from_block(jas_coin_of(board, chip), block);

        sl_wrapper_init();

        /* Trivial table: 4080 good banks everywhere. */
        for (k = 0; k < 3u * 0xff0u; k++) {
            jas_bank_good[board][k]    = 1;
            jas_bank_bad[board][k]     = 0;
            jas_bank_partial[board][k] = 1;
        }
        for (chip = 0; chip < 3; chip++) {
            unsigned mesh, w;
            for (mesh = 0; mesh < 4; mesh++)
                for (w = 0; w < 64; w++) {
                    jas_mc_en[board][chip][mesh][w]         = 0xffff;
                    jas_mc_refresh_en[board][chip][mesh][w] = 0xffff;
                }
        }

        for (chip = 0; chip < 3; chip++) {
            uint8_t m = (uint8_t)(1u << chip);
            if (!(ASIC_EN(board) >> chip & 1)) continue;
            sl_single_write(m, 0xf8, 0x830, 0, board);
            sl_delay(0, 2000);
            sl_single_write(m, 0xf8, 0x830, 0xffff, board);
        }
        /* Give the chip its fixed parameters back, then RESTART IT. The reset
         * alone leaves it stopped. */
        config_asic_fix_param(board);
        sl_single_write(0xff, 0xf9, 0x0003, ASIC_EN(board), board);
        printf("  soft reset + restart done\n");

        for (chip = 0; chip < 3; chip++) {
            const jas_coin_ctx *c = jas_coin_of(board, chip);
            uint8_t m = (uint8_t)(1u << chip);
            uint64_t addr = dag_cache_init_addr(0xff0u, c->cache_size);

            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            sl_multi_write(m, 0xf8, 0x08, (uint16_t *)&c->dag_size, 4, board);
            sl_multi_write(m, 0xf8, 0x04, (uint16_t *)&c->cache_size, 4, board);
            sl_multi_write(m, 0xf8, 0x30, (uint16_t *)&c->seed, 16, board);
            config_remap_info(board, (uint8_t)chip, 1);
            /* The memory enable masks: without them the chip has its banks
             * off and never completes. See core/dag_calc.c. */
            {
                unsigned mesh;
                for (mesh = 0; mesh < 4; mesh++)
                    sl_multi_write(m, (uint8_t)((0x10u << mesh) | 8u), 0x0c00,
                                   jas_mc_en[board][chip][mesh], 0x40, board);
                for (mesh = 0; mesh < 4; mesh++)
                    sl_multi_write(m, (uint8_t)((0x10u << mesh) | 8u), 0x0c40,
                                   jas_mc_refresh_en[board][chip][mesh], 0x40, board);
            }
            sl_multi_write(m, 0x18, 0x304, (uint16_t *)&addr, 3, board);
            printf("  asic %u: cache %llu B, placement %llu\n", chip,
                   (unsigned long long)c->cache_size, (unsigned long long)addr);
        }

        /*
         * READ BACK BEFORE STARTING. "start=0" after writing 1 can mean two
         * opposite things: the register is a pulse that re-arms itself, or our
         * writes do not arrive. Reading back what MUST hold (the cache size
         * and the placement address) settles between the two without assuming
         * anything.
         */
        for (chip = 0; chip < 3; chip++) {
            uint8_t m = (uint8_t)(1u << chip);
            uint16_t readback4[4] = { 0, 0, 0, 0 };
            uint16_t readback304[3] = { 0, 0, 0 };
            const jas_coin_ctx *c = jas_coin_of(board, chip);

            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            /*
             * 0x1c and NOT 0xfc: a read at 0xfc addresses the FOUR meshes at
             * once, four responders on one bus collide, and the link locks up
             * ("state 0x0000000a"). A write can be broadcast, a read never.
             */
            sl_multi_read(m, 0x1c, 0x04, readback4, 4, 1000000, board);
            sl_multi_read(m, 0x1c, 0x304, readback304, 3, 1000000, board);
            printf("  asic %u read back: reg04 = %04x %04x %04x %04x  (expected %08llx)\n",
                   chip, readback4[0], readback4[1], readback4[2], readback4[3],
                   (unsigned long long)c->cache_size);
            printf("            reg304 = %04x %04x %04x\n",
                   readback304[0], readback304[1], readback304[2]);
        }

        for (chip = 0; chip < 3; chip++)
            if (ASIC_EN(board) >> chip & 1)
                sl_single_write((uint8_t)(1u << chip), 0x18, 0x300, 1, board);
        printf("  start issued\n\n");

        /* Look at the chip, not just the end flag. */
        for (t = 0; t < 20; t++) {
            printf("  t=%2u s:", t);
            for (chip = 0; chip < 3; chip++) {
                uint16_t done = 0, dag = 0, st = 0;
                uint8_t m = (uint8_t)(1u << chip);

                if (!(ASIC_EN(board) >> chip & 1))
                    continue;
                uint16_t done_b = 0;

                /* 0x301 through the mesh (what the firmware does) AND through
                 * the board. If the two differ, the flag exists but we do not
                 * read it in the right place. */
                sl_single_read(m, 0x1c, 0x301, &done, 1000000, board);
                sl_single_read(m, 0xfd, 0x301, &done_b, 1000000, board);
                sl_single_read(m, 0x1c, 0x303, &dag, 1000000, board);
                sl_single_read(m, 0x1c, 0x300, &st, 1000000, board);
                printf("   asic%u done(mesh)=%u done(board)=%u start=%u dag=%u",
                       chip, done, done_b, st, dag);
            }
            printf("\n");
            fflush(stdout);
            sl_delay(1, 0);
        }
        for (chip = 0; chip < 3; chip++)
            if (ASIC_EN(board) >> chip & 1)
                sl_single_write((uint8_t)(1u << chip), 0x18, 0x300, 0, board);
        return 0;
    }

    /*
     * --seedup <block>: STEP 3, the complete startup: prepare_state, the
     * three self-tests, dag_calc, sending the work. Several minutes. DO NOT
     * INTERRUPT: dag_calc fills the chips' memory, and cutting during the
     * fill has already cost two machines. Run under nohup, output to a file.
     *
     * EXPECTED:
     *   "board 0 init params"
     *   "board 0 self_check_func start" ... "self_check_func end"
     *   "board 0 asic N cache calc done", then "calc dag OK!"
     *   "board 0 asic N dag check" followed by OK on all three chips
     *
     * "calc dag OK!" is the only verdict that counts: it comes from
     * dram_check_dag, a byte-for-byte comparison between what the chip
     * wrote and what our ethash recomputes.
     */
    if (argc > 1 && strcmp(argv[1], "--seedup") == 0) {
        uint64_t block = (argc > 2) ? strtoull(argv[2], NULL, 0) : 21000000ull;
        int board;
        unsigned chip;

        if (jas_require_session() < 0)
            return 1;

        printf("\n  STEP 3 - full startup, block %llu\n",
               (unsigned long long)block);
        printf("  expected: self_check on 3 asics, 208/147/148 dead banks,\n");
        printf("            then \"calc dag OK!\" on all three\n");
        printf("  DO NOT INTERRUPT during the fill\n\n");

        for (board = 0; board < 3; board++)
            for (chip = 0; chip < 3; chip++)
                jas_coin_from_block(jas_coin_of(board, chip), block);
        printf("  cache %llu B, dag %llu B\n\n",
               (unsigned long long)jas_coin_of(0, 0)->cache_size,
               (unsigned long long)jas_coin_of(0, 0)->dag_size);

        /*
         * The processor cache, without which dag_calc would wait ten minutes
         * for nothing and dram_check_dag would have no judge. The firmware has
         * already built it at this stage, through pow_flow_operation; we must
         * do it explicitly since we have no pool.
         */
        printf("  building the processor cache (%llu B)...\n",
               (unsigned long long)jas_coin_of(0, 0)->cache_size);
        *jas_cache_zil = ethash_light_new(block);
        if (!*jas_cache_zil) {
            printf("  cpu cache failed - test abandoned\n");
            return 1;
        }
        *jas_fpga_block_number = block;
        *jas_fpga_dag_size = jas_coin_of(0, 0)->dag_size;
        *jas_coin_type = 0;
        printf("  cache ready.\n\n");

        sl_wrapper_init();
        seed_update_operation();
        return 0;
    }

    /*
     * --dagcheck <block>: STEP 4, the final arbiter. Run AFTER --seedup, on
     * a machine whose DAG is in place: it builds nothing, reads back ten
     * locations per chip and compares them with what our ethash recomputes
     * from the light cache. A single disagreement invalidates the whole
     * keccak / ethash / dag_calc / remap chain.
     *
     * EXPECTED: "board[0] asic[N] bank_num = ~3800" then nothing.
     *
     * It is the only trial that tests our ethash against the silicon;
     * tools/test_ethash.c tests it against a specification.
     */
    if (argc > 1 && strcmp(argv[1], "--dagcheck") == 0) {
        uint64_t block = (argc > 2) ? strtoull(argv[2], NULL, 0) : 21000000ull;
        int board = (argc > 3) ? atoi(argv[3]) : 0;
        unsigned chip;
        int good = 0, bad = 0;

        if (jas_require_session() < 0)
            return 1;

        printf("\n  STEP 4 - is the DAG of board %d correct? block %llu\n",
               board, (unsigned long long)block);
        printf("  expected: no \"WRONG DAG\". A single mismatch invalidates everything.\n\n");

        for (chip = 0; chip < 3; chip++)
            jas_coin_from_block(jas_coin_of(board, chip), block);

        /* The processor's light cache, without which there is no judge. */
        printf("  building the processor cache (%llu B)...\n",
               (unsigned long long)jas_coin_of(board, 0)->cache_size);
        *jas_cache_zil = ethash_light_new(block);
        if (!*jas_cache_zil) {
            printf("  cpu cache failed - test abandoned\n");
            return 1;
        }
        *jas_fpga_block_number = block;
        *jas_fpga_dag_size = jas_coin_of(board, 0)->dag_size;
        *jas_coin_type = 0;
        printf("  cache ready.\n\n");

        for (chip = 0; chip < 3; chip++) {
            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            if (dram_check_dag(board, (uint8_t)chip, 0) == 0) {
                printf("  asic %u: DAG CORRECT\n", chip);
                good++;
            } else {
                printf("  asic %u: WRONG DAG\n", chip);
                bad++;
            }
        }
        printf("\n  %d correct, %d wrong\n", good, bad);
        return bad ? 1 : 0;
    }

    return 0;
}
