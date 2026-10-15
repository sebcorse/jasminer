/*
 * The monitoring thread: temperatures, fans, alarms.
 *
 * Takes over the alarm policy of pthread_dev_info_update (@0x46418), the
 * only firmware function writing the four flags sl_wrapper_work checks
 * (exit_func only writes the Ctrl-C one). The original (2796 bytes) mostly
 * fills dev_info_g for the web API; here the web data go through web.c.
 * The thresholds were read in the disassembly:
 *
 *     missing temperature    (max,min) == (-64,-64), more than 4 rounds
 *     excessive temperature  max > 68 degrees,       more than 2 rounds
 *     stopped fan                                    more than 4 rounds
 *
 * THE COUNTING IS THE ESSENCE: no alarm fires on an isolated measurement, the
 * condition must hold over several rounds; a sensor hiccuping once does not
 * shut the machine down, a real overheating is confirmed in three seconds.
 * 68 degrees leaves eight above the 60 at which the fan curve is already at
 * 100 %: the vendor wants proof that cooling has failed before cutting.
 *
 * DEDUCED, not read: the CONDITION of the fan alarm (its counter and
 * threshold are read). It is "no tachometer is turning", the only reliable
 * reading: the setpoint says what we ask, not what turns.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define T_MISSING        (-64)
#define TEMP_THRESHOLD        68
#define ROUNDS_TEMP         2      /* fires on the 3rd consecutive round */
#define ROUNDS_MISSING      4      /* on the 5th */
#define ROUNDS_FAN      4      /* on the 5th */

static pthread_t monitor_thread;

/* The maximum and minimum over all boards, like dev_info_g +0x3560. */
int jas_temp_max = T_MISSING;
int jas_temp_min = T_MISSING;

static void read_all_boards(void)
{
    unsigned board;
    int tmax = T_MISSING, tmin = 200;
    int found = 0;

    for (board = 0; board < BOARD_NUM; board++) {
        int temp[3], a, b;
        char rank = 0;
        unsigned chip;

        if (!(BOARD_EN >> board & 1))
            continue;
        /*
         * jas_sensors_read does I2C over an AXI window of the PL: if the
         * mining thread pulled the PL reset low meanwhile, this read would
         * never get its acknowledgement and the ARM core would stall.
         */
        jas_sensors_read(board);
        for (chip = 0; chip < 3; chip++)
            temp[chip] = (int)jas_asic_temp[board][chip];
        if (!temperature_filter(temp, (int)board, &a, &b, &rank))
            continue;
        if (a > tmax) tmax = a;
        if (b < tmin) tmin = b;
        found = 1;
    }
    jas_temp_max = found ? tmax : T_MISSING;
    jas_temp_min = found ? tmin : T_MISSING;
}

static int fans_stopped(void)
{
    int v[4], k, spinning = 0;

    fan_speed_read(v);
    for (k = 0; k < 4; k++)
        if (v[k] > 0)
            spinning++;
    return spinning == 0;
}

/*
 * The thread. One round per second: it is also what refreshes the fan
 * setpoint, without which the FPGA watchdog would put it back to 100 %.
 */
static void *monitor(void *nothing)
{
    unsigned n_missing = 0, n_hot = 0, n_fans = 0;
    unsigned round_eff = 0;

    (void)nothing;
    jas_name_thread("monitor");
    jas_log("monitoring: started (cutoff at %d degrees)", TEMP_THRESHOLD);

    for (;;) {
        jas_hashrate_rt_sample();   /* real hashrate sample (accepted nonces) */
        read_all_boards();

        /* No temperature at all. */
        if (jas_temp_max == T_MISSING && jas_temp_min == T_MISSING) {
            if (n_missing++ > ROUNDS_MISSING && !*jas_alarm_temp_missing) {
                *jas_alarm_temp_missing = 1;
                jas_log("ALARM: no temperature at all for %u loops", n_missing);
            }
        } else {
            n_missing = 0;

            if (jas_temp_max > TEMP_THRESHOLD) {
                if (n_hot++ > ROUNDS_TEMP && !*jas_alarm_temp_high) {
                    *jas_alarm_temp_high = 1;
                    jas_log("ALARM: %d degrees, above %d for %u loops",
                            jas_temp_max, TEMP_THRESHOLD, n_hot);
                }
            } else {
                n_hot = 0;
            }
        }

        if (fans_stopped()) {
            if (n_fans++ > ROUNDS_FAN && !*jas_alarm_fan) {
                *jas_alarm_fan = 1;
                jas_log("ALARM: no fan turning for %u loops", n_fans);
            }
        } else {
            n_fans = 0;
        }

        /* THE RAIL VOLTAGES, every second, for the web page. */
        {
            uint16_t vol[9];

            memset(vol, 0, sizeof(vol));
            asic_pmbus_read_vol(jas_pmbus_mode, 0, vol);
            {
                unsigned q;

                for (q = 0; q < 3; q++)
                    jas_web_rails_publish(0, q, vol[q * 3], vol[q * 3 + 1],
                                          vol[q * 3 + 2]);
            }
        }

        /*
         * THE THROUGHPUT PER CHIP, every thirty seconds: in dual mining it
         * tells a weak chip from one starved by the harvest. get_eff reads
         * register 0x43 of each ALU (twelve reads, one-second guard delay
         * each) under the bus lock, so they are serialized with the mining
         * transfers; every thirty seconds because the measurement needs one
         * second between two readouts.
         */
        if (++round_eff >= 30) {
            unsigned c2, p2, a2;

            round_eff = 0;
            for (c2 = 0; c2 < BOARD_NUM; c2++) {
                if (!(BOARD_EN >> c2 & 1))
                    continue;
                get_eff((int)c2, 1);
                for (p2 = 0; p2 < 3; p2++) {
                    double sum = 0.0;
                    unsigned live = 0;

                    if (!(ASIC_EN((int)c2) >> p2 & 1))
                        continue;
                    for (a2 = 0; a2 < 120; a2++) {
                        float v = jas_eff_real_value[c2][p2][a2];
                        if (v > 0.0f) { sum += v; live++; }
                    }
                    /* The sum is in hashes per second; showing it in MH/s
                     * needs the division, otherwise we read 658 million
                     * MH/s. A mistake of my first draft. */
                    /*
                     * ONCE EVERY TEN MINUTES, not at every round: the
                     * throughput is served continuously by the API and drawn by
                     * the home page charts; the log does not have to repeat
                     * it. JASRE_TRACE restores the stock pace.
                     */
                    {
                        static time_t last_hashrate;
                        time_t now = time(NULL);

                        if (jas_trace_active() ||
                            !last_hashrate || now - last_hashrate >= 600) {
                            if (p2 == 2 || jas_trace_active())
                                last_hashrate = now;
                            /*
                             * LOG NOTHING BEFORE THE CHIP MINES (live > 0):
                             * "0 MH/s on 0 alu" lines would clutter the log
                             * during the init. The web publish (below) stays
                             * unconditional.
                             */
                            if (live > 0) {
                                jas_log("hashrate board %u asic %u: %.0f MH/s"
                                        " on %u alu (%.1f MH/s per alu)",
                                        c2, p2, sum / 1e6, live,
                                        sum / 1e6 / live);
                                {
                                    /* Right after: the chip's LOCAL reject
                                     * (nonces with a wrong mix discarded before
                                     * submission; silicon health per chip). */
                                    unsigned rj = 0, tt = 0;
                                    double rr = jas_reject_local_chip(c2, p2,
                                                                      &rj, &tt);
                                    jas_log("reject local board %u asic %u:"
                                            " %.2f%% (%u/%u nonces)",
                                            c2, p2, rr, rj, tt);
                                }
                            }
                        }
                    }
                    /* And drop it off for the API: it must above all not
                     * read the silicon itself. */
                    jas_web_hashrate_publish((int)c2, p2, sum / 1e6, live);
                }
            }

            /*
             * THE HISTORY, ONCE PER PASS AND OUTSIDE THE LOOP: hooked on a
             * particular chip, it would never run when that chip is disabled
             * (the loop skips inactive chips), leaving the charts empty.
             */
            {
                unsigned seen = 0, submitted = 0, rej = 0, per = 0;
                unsigned ta = 0, tr = 0, q;
                double rt_real = 0, avg_real = 0;

                jas_mining_counters(&seen, &submitted, &rej, &per);
                jas_hashrate_rt_get(&rt_real, &avg_real);   /* REAL hashrate */
                for (q = 0; q < 4; q++) {
                    unsigned a2 = 0, r2 = 0;

                    if (jas_pool_counters((int)q, &a2, &r2, NULL)) { ta += a2; tr += r2; }
                }
                jas_web_sample(rt_real, avg_real,
                                       (ta + tr) ? 100.0 * tr / (ta + tr) : 0.0);
            }
        }

        /*
         * The regulation, here and nowhere else: it MUST be called at each
         * round. This refresh, not the value itself, is what keeps the FPGA
         * from putting everything back to full speed.
         */
        set_PWM_according_to_temperature_x16();
        sleep(1);
    }
    return NULL;
}

/*
 * creat_pthread @ 0x448??: start the threads.
 *
 * The original starts EIGHT: sl_wrapper_work, pthread_send_ether,
 * pthread_hash_monitor, pthread_dev_info_update, pthread_web,
 * pthread_websocket, pthread_web_config_monitor, pthread_device_status_monitor.
 *
 * This one starts TWO: monitoring and mining. The network threads (one per
 * pool slot) and the API thread (port 4028) are started from main.c, the
 * DAG rebuild thread from dag_calc.c. The original's statistics, LED and
 * web-configuration threads are not reproduced.
 */
void creat_pthread(void)
{
    pthread_t mining_thread;

    if (pthread_create(&monitor_thread, NULL, monitor, NULL) != 0)
        jas_log("fail to pthread_create monitor");
    else
        pthread_detach(monitor_thread);

    /* The network thread is started by main.c, BEFORE waiting for the first
     * work — otherwise the startup would block. */

    if (pthread_create(&mining_thread, NULL, sl_wrapper_work, NULL) != 0)
        jas_log("fail to pthread_create sl_wrapper_work");
    else
        pthread_detach(mining_thread);
}
