/*
 * Sequence of the startup check.
 *
 * This is the backbone of everything before mining. Order matters: each step
 * assumes the previous one is in place.
 *
 *   hardware reset        puts the ASICs and the DFT block in a known state
 *   spi_check             sets the link phase, without which everything else
 *                         would be misread
 *   sl_get_asic_id        identifies the chips actually present
 *   config_remap_info     first remapping table, on the banks assumed good
 *   config_asic_fix_param fixed parameters of the chip
 *   alu_check             do the compute units answer
 *   dft_short_pattern_test_110  the memory test proper
 *   crossbar_check        and finally the reachability of the banks by each ALU
 *
 * WATCH THE UNIT: `sl_delay(sec, ns)` takes NANOSECONDS as second argument:
 * `sl_delay(0, 2000000)` is 2 ms, not 2 s (the firmware's log shows "self_check
 * _func start" and "reset asic hard reset done" in the same second).
 *
 * These waits are short because the PLLs locked long before, in
 * power_on_operation, which must run BEFORE this function: without it the
 * ASICs have no clock (pll_lock 0x00) and everything that follows measures
 * nothing.
 */

#include <stdio.h>
#include <stdlib.h>
#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include "dft.h"

/* Not reimplemented yet: provided by the firmware in bridge mode. */
extern void sl_get_asic_id(void);
extern int  dft_short_pattern_test_110(int thread_id);

#define REG_ASIC_RESET 0x00     /* via cmd 0xf9: hardware reset */
#define REG_SOFT_RESET 0x830

/*
 * Test aid, not a production setting.
 *
 * crossbar_check takes several tens of minutes here (see the filling of the
 * test memory) and has no influence on the DFT verdicts. Skipping it lets us
 * compare our bank arrays with the firmware's without tying up the machine.
 */
int jas_selfcheck_skip_crossbar = 0;

void self_check_func(int thread_id)
{
    unsigned asic;

    /* Same words as the firmware: its trace in flash is our reference. */
    printf("board %d self_check_func start\n", thread_id);

    /* Hardware reset: power off, reset the DFT block, power the chips declared
     * present back on. Each step needs its time. */
    sl_single_write(0xff, 0xf9, REG_ASIC_RESET, 0, thread_id);
    sl_delay(0, 2000000);
    dft_reset(thread_id, 0, 0);
    sl_delay(0, 3000000);
    sl_single_write(0xff, 0xf9, REG_ASIC_RESET, ASIC_EN(thread_id), thread_id);
    sl_delay(0, 2000000);
    printf("board %d reset asic hard reset done\n", thread_id);
    printf("board %d start check spi\n", thread_id);

    /* Phase calibration comes first: everything that follows relies on
     * reliable reads. */
    spi_check(thread_id);
    sl_single_write(ASIC_EN(thread_id), 0xf8, REG_SOFT_RESET, 0xffff, thread_id);

    sl_get_asic_id();

    /*
     * config_remap_info writes an entry for EACH of the 4080 banks,
     * unconditionally: flag at zero -> "hole" entry towards 0xfff, outside the
     * addressable space. That is why the state of the flags AT THIS MOMENT
     * decides everything — and why seed_init_bank_params must have run
     * before, as in the firmware. See core/seed.c.
     */
    for (asic = 0; asic < ASIC_NUM; asic++) {
        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;
        config_remap_info(thread_id, (uint8_t)asic, 0);
    }

    config_asic_fix_param(thread_id);
    printf("board %d fix asic param done\n", thread_id);

    printf("board %d start check alu\n", thread_id);
    alu_check(thread_id);
    printf("board %d alu check done\n", thread_id);
    alu_check_analysis(thread_id);
    printf("board %d alu check analysis done\n", thread_id);

    /*
     * Skipping the memory test is legitimate ONLY if the bank flags are
     * provided otherwise, which is the case under JASRE_BANKS_OK.
     */
    if (getenv("JASRE_SKIP_DFT"))
        jas_log("board %d: memory test skipped (flags supplied)", thread_id);
    else
    {
        printf("board %d start dft test\n", thread_id);
        dft_short_pattern_test_110(thread_id);
    }

    if (jas_selfcheck_skip_crossbar)
        jas_log("board %d: crossbar_check skipped (test mode)", thread_id);
    else
        crossbar_check(thread_id);

    printf("board %d self_check_func end\n", thread_id);
}
