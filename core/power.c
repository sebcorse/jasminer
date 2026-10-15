/*
 * Power-on: power_on_operation @ 0x1daf4.
 *
 * It configures the PLLs, sets the voltages and derives `asic_en` from the
 * real PLL lock. Without it, register 0x11 of a chip reads 0x0000 (no PLL
 * locked) instead of 0x07: the serial link stays readable (its clock is
 * independent), but everything depending on the internal clock is inert.
 *
 * REFERENCE, the firmware's log, values to reproduce:
 *
 *     Set freq 510, pll 198              (198 is HEXADECIMAL: 0x198 = 408)
 *     board[0] asic[0..2] freq=510
 *     reset pll count=0,pll_lock=0x07
 *     asic[0..2] pll_clk_test=6375
 *     pll_clk_test_g * 20000 * 8 / 2 / 1000000 is 510.00 MHz
 *     ###board 0 pll_stable_g is 7
 *     board 0 PLL IN ASIC 0..2 lock done
 *     board 0 asic N dev_info_g.asic_vol_rail0 = 860, ...rail1 = 1150, ...
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Registers of the board block, reached through cmd 0xf9 to write and 0xfd to read. */
#define REG_ASIC_RESET   0x0000   /* ASIC reset, value = mask */
#define REG_ASIC_POWER   0x0001   /* power cut/restore */
#define REG_ASIC_START   0x0003   /* startup, value = mask             */
#define REG_PLL_RESET    0x0010
#define REG_PLL_LOCK     0x0011
#define REG_PLL_CFG      0x0013
#define REG_PLL_CLK_TEST 0x0023
#define REG_SCK_DIV_WR   0x6000
#define REG_SCK_DIV_RD   0x6001
#define REG_LOGIC_VER    0x7000

#define CMD_BOARD_W      0xf9
#define CMD_BOARD_R      0xfd

/* Values read in the reference binary (.data segment). */
#define SCK_DIV_WR       4
#define SCK_DIV_RD       4

/*
 * The voltages, read from jasminer.json. After the ASICs' power cut the
 * regulators restart at ~901 mV; the firmware sets them again, and so do we.
 */
#define RAIL0            860
#define RAIL1            1150
#define RAIL2            1500

uint16_t jas_pll_lock[3];
uint16_t jas_pll_stable[3];
uint16_t jas_pll_clk_test[3][3];
uint16_t jas_pmbus_mode;

/*
 * The per-chip voltage setpoints, as the firmware keeps them in dev_info_g
 * (offsets 0x3c/0x3e/0x40, step of 6 per chip). We did not reimplement
 * dev_info_g; these three constants are what jasminer.json asks for and what
 * asic_pmbus_config really sets.
 */
struct config_hw { uint16_t freq; uint16_t freq_chip[3]; int mask; int rail[3];
                    int rail0_chip[3]; };
static const struct config_hw *config_hardware(void);

/* The voltage of THIS chip for rail 0: its own ("asicN_rail0") if it has one,
 * otherwise the global "asic_rail0". Rails 1 and 2 stay global. */
int jas_asic_vol_rail(unsigned board, unsigned chip, unsigned rail)
{
    const struct config_hw *c = config_hardware();

    (void)board;
    switch (rail) {
    case 0:  return (chip < 3 && c->rail0_chip[chip]) ? c->rail0_chip[chip] : c->rail[0];
    case 1:  return c->rail[1];
    default: return c->rail[2];
    }
}

/*
 * sl_get_pll_from_freq @ 0x1d0d0 — validated by computation against the log.
 *
 * Outside [270, 636) MHz, the original complains and returns 400.
 */
uint16_t sl_get_pll_from_freq(uint16_t freq)
{
    uint16_t pll;

    if ((unsigned)(uint16_t)(freq - 0x10e) >= 0x16e) {
        jas_log("frequency %u out of range, pll forced to 400", freq);
        return 400;
    }
    pll = (uint16_t)(((uint32_t)(((uint64_t)freq * 0xcccccccdull) >> 32)) & ~3u);
    printf("Set freq %d, pll %02x\n", freq, pll);
    return pll;
}

/*
 * The requested frequency, per chip.
 *
 * The firmware keeps it in dev_info_g.freq, filled from jasminer.json. The
 * 510 MHz of our machines is in fact a PATCH of the loader: the factory runs
 * at 520, and update_work_mode is rewritten in four places to go down to 510.
 */
/*
 * THE HARDWARE CONFIGURATION, read in a single pass. The original splits
 * it between update_freq (0x3e848, key "freq") and update_asic_rail_vol
 * (0x3f324, keys "asic_rail0" to "asic_rail2"), both called at POW_START.
 *
 * THE BOUNDS ARE OURS, the original has none: it trusts its own CGI, while
 * our file can be edited by hand, and a wrong core voltage cannot be
 * undone. Refusing and SAYING so beats silently setting an absurd value.
 */
static const int rail_default[3] = { RAIL0, RAIL1, RAIL2 };
static const int rail_min[3]    = {  700, 1000, 1300 };
static const int rail_max[3]    = { 1000, 1300, 1700 };

/*
 * THE WORK MODES (1 Efficiency, 2 Balance, 3 Performance, 4 Performance
 * Ultra): the clock and core voltage of the Miner page's presets. They apply
 * when the configuration gives no "freq" or no "asic_rail0", as in a file
 * written by the original firmware, which has only "work_pattern": the machine
 * then runs its mode as the page shows it. Index 0: no mode given (Balance).
 */
static const uint16_t mode_freq[5]  = { 510, 500, 510, 520, 540 };
static const int      mode_rail0[5] = { RAIL0, 800, 860, 880, 900 };

static const struct config_hw *config_hardware(void)
{
    static int loaded;
    static struct config_hw c = { 510, { 0, 0, 0 }, 0, { RAIL0, RAIL1, RAIL2 },
                                   { 0, 0, 0 } };
    FILE *f;
    char line[512];
    int freq_seen = 0, rail0_seen = 0;
    unsigned long mode = 0;

    if (loaded)
        return &c;
    loaded = 1;

    f = fopen(jas_config_file(), "r");
    if (!f)
        return &c;
    while (fgets(line, sizeof(line), f)) {
        char *p = strstr(line, "\"freq\"");
        unsigned k;

        if (p && (p = strchr(p, ':'))) {
            c.freq = (uint16_t)strtoul(p + 1, NULL, 10);
            freq_seen = 1;
        }
        p = strstr(line, "\"work_pattern\"");
        if (p && (p = strchr(p, ':'))) {
            mode = strtoul(p + 1, NULL, 10);
            if (mode > 4)
                mode = 0;
        }
        /*
         * ONE FREQUENCY PER CHIP, "asicN_freq", taking precedence over "freq",
         * the fallback.
         */
        for (k = 0; k < 3; k++) {
            char key[16];

            snprintf(key, sizeof(key), "\"asic%u_freq\"", k);
            p = strstr(line, key);
            if (p && (p = strchr(p, ':'))) {
                unsigned long v = strtoul(p + 1, NULL, 10);

                if (v >= 270 && v <= 635)
                    c.freq_chip[k] = (uint16_t)v;
                else if (v)
                    jas_log("configuration: asic%u_freq = %lu out of [270,635],"
                            " ignored", k, v);
            }
        }
        {
            /*
             * THE MASK OF ACTIVE CHIPS.
             *
             * Read here IN ADDITION to asic_enabled_mask.conf, which hal_devmem
             * already reads: the page writes both, and the JSON is the reference.
             */
            char *q = strstr(line, "\"asic_enabled_mask\"");

            if (q && (q = strchr(q, ':'))) {
                unsigned long v = strtoul(q + 1, NULL, 10);

                if (v >= 1 && v <= 7)
                    c.mask = (int)v;
            }
        }
        for (k = 0; k < 3; k++) {
            char key[16];

            snprintf(key, sizeof(key), "\"asic_rail%u\"", k);
            p = strstr(line, key);
            if (p && (p = strchr(p, ':'))) {
                long v = strtol(p + 1, NULL, 10);

                if (v < rail_min[k] || v > rail_max[k]) {
                    jas_log("configuration: asic_rail%u = %ld mV out of range"
                            " [%d, %d], ignored", k, v, rail_min[k], rail_max[k]);
                } else {
                    if (v != c.rail[k])
                        jas_log("configuration: asic_rail%u = %ld mV (default %d)",
                                k, v, rail_default[k]);
                    c.rail[k] = (int)v;
                    if (k == 0)
                        rail0_seen = 1;
                }
            }
        }
        /*
         * ONE VOLTAGE PER CHIP, "asicN_rail0", taking precedence over
         * "asic_rail0". Each chip has its own regulator (MCP2973 at 0x6a, 0x60,
         * 0x64): a chip mining without errors can be lowered while the weaker
         * ones are kept higher. Same bounds as asic_rail0, which stays the
         * fallback; the page writes the highest of the three into it, safe for
         * another firmware that would only know the global one.
         *
         * NOT asic_rail1 / asic_rail2: these keys keep their original meaning
         * (other rails, 1150 and 1500 mV in all our configurations). Reading
         * them as voltages of chips 1 and 2 would put 1150 mV on chip 1.
         */
        for (k = 0; k < 3; k++) {
            char key[24];

            snprintf(key, sizeof(key), "\"asic%u_rail0\"", k);
            p = strstr(line, key);
            if (p && (p = strchr(p, ':'))) {
                long v;

                p++;
                while (*p == ' ' || *p == '"')
                    p++;
                v = strtol(p, NULL, 10);
                if (v >= rail_min[0] && v <= rail_max[0]) {
                    if (v != c.rail0_chip[k])
                        jas_log("configuration: asic%u_rail0 = %ld mV"
                                " (chip-specific)", k, v);
                    c.rail0_chip[k] = (int)v;
                } else if (v) {
                    jas_log("configuration: asic%u_rail0 = %ld mV out of range"
                            " [%d, %d], ignored", k, v, rail_min[0], rail_max[0]);
                }
            }
        }
    }
    fclose(f);
    if (!freq_seen) {
        c.freq = mode_freq[mode];
        jas_log("configuration: no \"freq\", %u MHz of work mode %lu", c.freq,
                mode ? mode : 2);
    }
    if (!rail0_seen) {
        c.rail[0] = mode_rail0[mode];
        jas_log("configuration: no \"asic_rail0\", %d mV of work mode %lu",
                c.rail[0], mode ? mode : 2);
    }
    return &c;
}

static uint16_t requested_frequency(void)
{
    return config_hardware()->freq;
}

/* The frequency of THIS chip: its own if it has one, otherwise the global one. */
static uint16_t chip_frequency(unsigned asic)
{
    const struct config_hw *c = config_hardware();

    if (asic < 3 && c->freq_chip[asic])
        return c->freq_chip[asic];
    return c->freq;
}

void power_on_operation(int thread_id)
{
    uint16_t pll_cfg[3] = { 0, 0, 0 };
    uint16_t logic_ver = 0;
    unsigned asic, round;

    if (thread_id < 0 || thread_id > 2)
        return;

    ASIC_EN(thread_id) = 0;
    jas_pll_lock[thread_id] = 0;
    jas_pll_stable[thread_id] = 0;

    jas_link_hard_reset(1, thread_id);          /* 4 x 1 s, like the original */

    /*
     * The banner, word for word, except the Zynq version: that read stays
     * off by default (JASRE_ZYNQ_VER) until a run validates it.
     */
    printf("********************version********************\n");
    printf("BOOT.BIN_version is %.2f \n", 1.00);
    printf("board %d driver_version is %.2f \n", thread_id, 1.90);
    if (getenv("JASRE_ZYNQ_VER")) {
        jas_zynq_map_init();
        printf("board %d zynq logic version is 0x%02x \n", thread_id,
               jas_zynq_p ? *(volatile uint32_t *)jas_zynq_p : 0);
    }

    sl_single_read(0xff, CMD_BOARD_R, REG_LOGIC_VER, &logic_ver, 1000000, thread_id);
    jas_pmbus_mode = (logic_ver < 0x500) ? 0x450 : logic_ver;
    printf("board %d calc board logic_version is 0x%02x \n", thread_id, logic_ver);
    printf("APP_version is %.2f \n", 1.09);
    printf("*************************************************\n");

    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_WR, SCK_DIV_WR, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_RD, SCK_DIV_RD, thread_id);

    for (round = 0; round < 5; round++) {
        /* Cutting then restoring the ASICs' power. */
        sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0, thread_id);
        sleep(2);
        sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0xff, thread_id);
        usleep(20000);

        for (asic = 0; asic < ASIC_NUM; asic++) {
            uint16_t freq = chip_frequency(asic);

            pll_cfg[asic] = sl_get_pll_from_freq(freq);
            printf("board[%d] asic[%u] freq=%d\n", thread_id, asic, freq);
        }
        sl_multi_write(0xff, CMD_BOARD_W, REG_PLL_CFG, pll_cfg, 3, thread_id);

        /* PLL reset, then one second for them to lock. */
        sl_single_write(0xff, CMD_BOARD_W, REG_PLL_RESET, 0xffff, thread_id);
        usleep(20);
        sl_single_write(0xff, CMD_BOARD_W, REG_PLL_RESET, 0, thread_id);
        sleep(1);

        sl_single_read(0xff, CMD_BOARD_R, REG_PLL_LOCK,
                       &jas_pll_lock[thread_id], 2000000, thread_id);
        printf("reset pll count=%u,pll_lock=0x%.2x\n", round, jas_pll_lock[thread_id]);

        sl_multi_read(0xff, CMD_BOARD_R, REG_PLL_CLK_TEST,
                      jas_pll_clk_test[thread_id], 3, 4000000, thread_id);

        for (asic = 0; asic < ASIC_NUM; asic++) {
            uint16_t measurement = jas_pll_clk_test[thread_id][asic];
            unsigned freq_measured = (unsigned)((unsigned long)measurement * 80000UL
                                               / 1000000UL);
            uint16_t freq_expected = chip_frequency(asic);
            int locked = (jas_pll_lock[thread_id] >> asic) & 1;

            /*
             * PLL diagnosis, in run.log (jas_log, not printf to /dev/null):
             * the reference requires the lock AND the measured frequency ==
             * expected; the lock alone is checked here, and both are logged.
             */
            jas_log("board %d asic[%u] pll_clk_test=%d (%u MHz, expected %u)"
                    " lock=%d", thread_id, asic, measurement, freq_measured,
                    freq_expected, locked);
            if (locked) {
                jas_pll_stable[thread_id] |= (uint16_t)((1u << asic) & 0xff);
                if (freq_measured != freq_expected)
                    jas_log("board %d asic %u: PLL LOCKED AT THE WRONG FREQ (%u"
                            " != %u) — the reference would exclude it",
                            thread_id, asic, freq_measured, freq_expected);
            }
        }

        if (jas_pll_stable[thread_id] == 1 || jas_pll_stable[thread_id] == 7)
            break;
    }

    /*
     * THE CONFIGURED MASK AND THE PLL LOCK, BOTH: a chip must be WANTED
     * (the mask the page sets) and HEALTHY (its PLL locked). With the lock
     * alone, a chip disabled from the page would come back on as soon as
     * its PLL locks. A missing or zero mask means "all".
     */
    {
        int wanted = config_hardware()->mask;
        uint8_t stable = (uint8_t)jas_pll_stable[thread_id];

        ASIC_EN(thread_id) = wanted ? (uint8_t)(stable & wanted) : stable;
        if (wanted && (stable & ~wanted))
            jas_log("board %d: mask %#x requested, asics %#x dropped",
                    thread_id, wanted, stable & ~wanted);
    }
    jas_log("###board %d pll_stable_g is %d (ASIC_EN=%#x)", thread_id,
            jas_pll_stable[thread_id], ASIC_EN(thread_id));
    for (asic = 0; asic < ASIC_NUM; asic++)
        jas_log("board %d PLL IN ASIC %u %s", thread_id, asic,
                (ASIC_EN(thread_id) >> asic & 1) ? "lock done" : "is abnomal");

    if (ASIC_EN(thread_id) == 0)
        BOARD_EN &= (uint16_t)~(1u << thread_id);

    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_RESET, 0, thread_id);
    sl_delay(0, 20000000);                      /* 20 ms: sl_delay takes ns */
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_RESET, ASIC_EN(thread_id), thread_id);
    sl_delay(0, 20000000);

    /*
     * THE VOLTAGES, set through I2C by asic_pmbus_config (JASRE_NO_PMBUS
     * skips it, leaving the regulators at their default). It is the only part
     * that can damage a chip. mcp2973_set_vol writes VOUT_COMMAND (0x21) as
     * (mV / 5 - 0x31), e.g. 860 mV -> 123 = 0x7B, then reads READ_VOUT (0x8b)
     * back.
     *
     * The original's safety bounds, reproduced as they are:
     *     rail0 [800, 1201)   rail1 [1000, 1251)   rail2 [1300, 1601)
     * asic_pmbus_config writes NOTHING, not even rail0, if any of the three
     * is out of its range.
     */
    if (getenv("JASRE_NO_PMBUS")) {
        jas_log("board %d: voltages NOT re-applied (JASRE_NO_PMBUS) — the"
                " asics stay at the regulator's default value", thread_id);
    } else {
        /*
         * DELIBERATE DEPARTURE: open the IIC bus here too. In the firmware,
         * find_borad_and_poweron calls sensor_init BEFORE power_on_operation;
         * called alone (as the bench does), this function would find an
         * uninitialized controller, mcp2973_init would fail, and the voltages
         * would stay at ~901 mV with nothing flagging it. Both calls are
         * idempotent.
         */
        jas_iic_map_init(thread_id);
        iic_initialization_op(thread_id);

        for (asic = 0; asic < ASIC_NUM; asic++) {
            int mv0 = jas_asic_vol_rail((unsigned)thread_id, asic, 0);

            if (!(ASIC_EN(thread_id) >> asic & 1))
                continue;
            printf("board %d asic %u dev_info_g.asic_vol_rail0 = %u, "
                   "dev_info_g.asic_vol_rail1 = %u, dev_info_g.asic_vol_rail2 = %u, "
                   "pmbus_mode = %u\n",
                   thread_id, asic, mv0,
                   config_hardware()->rail[1], config_hardware()->rail[2],
                   jas_pmbus_mode);
            jas_log("board %d asic %u: voltage %d mV (%s)", thread_id, asic, mv0,
                    config_hardware()->rail0_chip[asic] ? "chip setting"
                                                          : "global setting");
            asic_pmbus_config(jas_pmbus_mode, thread_id, asic, (uint16_t)mv0,
                              config_hardware()->rail[1],
                              config_hardware()->rail[2]);
            sl_delay(0, 20000000);
        }
    }

    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_START, ASIC_EN(thread_id), thread_id);
    fflush(stdout);
}

/* ------------------------------------------------------------------------
 * The three other power functions. They share power_on_operation's
 * sequence but each stops at a different point:
 *
 *   power_off_operation      cuts, all boards, and stops there
 *   power_on_only            powers back on and shows the banner, WITHOUT
 *                            touching the PLLs or the voltages: no asic_en
 *                            comes out of it
 *   power_restart_operation  powers back on AND reconfigures the PLLs, but
 *                            does not read the lock back: asic_en keeps its
 *                            previous value
 *
 * After power_on_only, `asic_en` is ZERO: the function zeroes it on entry
 * and never recomputes it (only power_on_operation derives it from the PLL
 * lock). Chaining power_on_only then a test means testing with an empty
 * mask.
 * ------------------------------------------------------------------------ */

/*
 * Copy one line at a time to the flash log and to the output, as the
 * original does: printf, then snprintf into a buffer, then applog_driver. The
 * firmware devotes 4 KB of stack to it; our lines fit in 256 bytes.
 */
static void trace_log(const char *fmt, ...)
{
    char buffer[256];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);

    fputs(buffer, stdout);
    jas_applog(buffer);
}

/*
 * power_off_operation @ 0x1e514: cut the ASICs' power, on all boards (it
 * takes no board number). FPGA reset, then register 0x0001 to zero, which
 * opens the chips' power supply. The regulators stay where they are: a
 * power-on finds them at their default value (~901 mV), not at zero.
 */
void power_off_operation(void)
{
    unsigned board;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        jas_link_hard_reset(1, (int)board);
        sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0, (int)board);
    }
}

/*
 * OFF THE EXECUTION PATH (tools/atteignable.py): no live function calls
 * this one in the running firmware. Kept as a tool.
 */
/*
 * power_on_only @ 0x1e3ac: power back on and announce the versions.
 *
 * The beginning is that of power_on_operation, and it stops just before the
 * PLLs. The order of the writes differs, and it matters: here the ASIC reset
 * (0x0000) and the PLL reset (0x0010 to 0xffff) are set BEFORE the power cut,
 * so the chips come back held in reset, PLLs blocked; power_on_operation
 * cuts first and only puts the ASICs back in reset after the lock.
 *
 * The three versions are constants of the binary, read in the literal pool:
 * BOOT.BIN 1.00 (movt r3, #0x3ff0), driver 1.90 (@0x1e4d0), APP 1.09
 * (@0x1e4d8).
 */
void power_on_only(int thread_id)
{
    uint16_t logic_ver = 0;

    if (thread_id < 0 || thread_id > 2)
        return;

    /* Note what the function does NOT do: pll_stable_g is not reset to
     * zero, unlike power_on_operation. */
    ASIC_EN(thread_id) = 0;
    jas_pll_lock[thread_id] = 0;

    jas_link_hard_reset(1, thread_id);

    printf("********************version********************\n");
    printf("BOOT.BIN_version is %.2f \n", 1.00);
    printf("board %d driver_version is %.2f \n", thread_id, 1.90);

    /* Same caveat as in power_on_operation: this read was never executed by
     * us, it stays off until a pass has validated it. See the comment
     * there. */
    if (getenv("JASRE_ZYNQ_VER")) {
        jas_zynq_map_init();
        printf("board %d zynq logic version is 0x%02x \n", thread_id,
               jas_zynq_p ? *(volatile uint32_t *)jas_zynq_p : 0);
    }

    sl_single_read(0xff, CMD_BOARD_R, REG_LOGIC_VER, &logic_ver, 1000000, thread_id);
    printf("board %d calc board logic_version is 0x%02x \n", thread_id, logic_ver);
    printf("APP_version is %.2f \n", 1.09);
    printf("*************************************************\n");

    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_WR, SCK_DIV_WR, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_RD, SCK_DIV_RD, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_RESET, 0, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_PLL_RESET, 0xffff, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0, thread_id);
    sleep(2);
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0xff, thread_id);
    sl_delay(0, 20000000);                      /* 20 ms */
}

/*
 * OFF THE EXECUTION PATH (tools/atteignable.py): no live function calls
 * this one in the running firmware. Kept as a tool.
 */
/*
 * power_restart_operation @ 0x1e20c: restart a board without starting from
 * scratch.
 *
 * It is the only power-on path that logs to the flash log ("board %d
 * restart begin/done", "board[%d] asic[%d] freq=%d"), which makes it a
 * usable reference to date an incident.
 *
 * Unlike power_on_operation, the PLL reset is applied only once, without a
 * catch-up loop and WITHOUT reading register 0x0011 back: asic_en_g is not
 * updated, and a PLL that does not lock goes unnoticed until mining shows
 * it.
 *
 * DEPARTURE: the original reads the frequency in dev_info_g.freq, at offset
 * 0x3422 + board*6 + chip*2 (`ldrh.w r0, [r6, #2]!` with pre-increment, so
 * the base 0x3420 is not the first item); here requested_frequency() reads
 * jasminer.json, as in power_on_operation.
 */
void power_restart_operation(int thread_id)
{
    uint16_t pll_cfg[3] = { 0, 0, 0 };
    unsigned asic;

    if (thread_id < 0 || thread_id > 2)
        return;

    trace_log("board %d restart begin\n", thread_id);

    jas_link_hard_reset(1, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_WR, SCK_DIV_WR, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_SCK_DIV_RD, SCK_DIV_RD, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0, thread_id);
    sleep(2);
    sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_POWER, 0xff, thread_id);
    sl_delay(0, 20000000);                      /* 20 ms */

    for (asic = 0; asic < ASIC_NUM; asic++) {
        uint16_t freq = requested_frequency();

        pll_cfg[asic] = sl_get_pll_from_freq(freq);
        trace_log("board[%d] asic[%u] freq=%d\n", thread_id, asic, freq);
    }

    sl_multi_write(0xff, CMD_BOARD_W, REG_PLL_CFG, pll_cfg, 3, thread_id);
    sl_single_write(0xff, CMD_BOARD_W, REG_PLL_RESET, 0xffff, thread_id);
    usleep(50);                                 /* 0x32 µs — power_on waits
                                                 * only 20 µs at the same spot */
    sl_single_write(0xff, CMD_BOARD_W, REG_PLL_RESET, 0, thread_id);
    sleep(1);

    trace_log("board %d restart done\n", thread_id);
    fflush(stdout);
}
