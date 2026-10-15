/*
 * Fan regulation.
 *
 * The fan setpoint falls back to 100 % if nobody refreshes it (an FPGA
 * watchdog): without this regulation, the fans would run flat out all the
 * time. This firmware is the only regulator (the custom layer's fan_guard
 * daemon, fan.html and fan_control.json are gone).
 *
 * THE CURVE, constants read in the binary (35, 36, 65):
 *
 *     t < 21          ->  30 %
 *     21 <= t < 57    ->  (t - 20) * 35 / 36 + 30      from 30 to 65 %
 *     57 <= t < 61    ->  65 + (t - 56) * 8.75         from 74 to 100 %
 *     t > 60          ->  100 %
 *
 * Continuous, gentle up to 56 degrees, then very steep over the last four:
 * the vendor accepts noise rather than heat.
 *
 * The HYSTERESIS is two degrees: below that, the setpoint is not touched,
 * or a sensor oscillating by one degree would make the fans vary all the
 * time.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PWM_MIN        30
#define PWM_MAX        100
#define T_LOW          21     /* below: minimum PWM */
#define T_KNEE        57     /* start of the steep slope */
#define T_HIGH         60     /* above: maximum PWM */
#define SLOPE_NUM      35
#define SLOPE_DEN      36
#define PWM_KNEE      65
#define HYSTERESIS     2
/*
 * "COLD" STATE (trial, JASRE_FAN_COLD=1): outside mining, below
 * T_COLD_OFF, cut the fans (PWM 0) to let the chips warm up after the
 * self-test (~31 C); back to the curve above T_COLD_ON or as soon as the
 * ALUs mine (the regulators then carry 100 A and need air). Off by
 * default: at PWM 0 the fans still turn at ~2000 rpm (hardware floor).
 */
#define T_COLD_OFF    40
#define T_COLD_ON     45

/* Plausibility bounds of a temperature, read in temperature_filter. */
#define T_MIN_VALID   (-29)
#define T_MAX_VALID   89
#define T_MISSING      (-64)
#define OUTLIER_GAP 20     /* a maximum above average+20 is rejected */

/*
 * THE ALARM THRESHOLDS, read in pthread_dev_info_update (@0x46418), the
 * only function that writes the four dev_info_g flags sl_wrapper_work
 * checks (exit_func only writes the Ctrl-C one):
 *
 *     excessive temperature   cmp #68  then a counter at 2
 *     missing temperature     counter at 4
 *     stopped fan             counter at 4
 *
 * Each alarm is DELAYED: an isolated measurement does not cut the machine.
 * Used by core/surveillance.c.
 */
#define ALARM_TEMP_MAX      68
#define ALARM_TEMP_ROUNDS     2
#define ALARM_MISSING_ROUNDS  4
#define ALARM_FAN_ROUNDS  4

static int pwm_current = PWM_MIN;
static int temp_previous = -1000;
static int fan_cold = 0;          /* 1: fans off */
static int jas_mining_active = 0;      /* 1: the ALUs are mining (seed.c) */
static unsigned fan_cold_rounds = 0;

void jas_fan_mining(int active)
{
    jas_mining_active = active ? 1 : 0;
}

static int fan_cold_allowed(void)
{
    static int loaded = 0, allowed = 1;

    if (!loaded) {
        const char *e = getenv("JASRE_FAN_COLD");

        /* Off by default: at PWM 0 the fans stay at ~2000 rpm (hardware
         * floor) and the chip does not warm up. JASRE_FAN_COLD=1 to
         * try again. */
        allowed = (e != NULL && *e == '1');
        loaded = 1;
    }
    return allowed;
}

/*
 * fan_pwm_set @ 0x44390 — set the setpoint, with its bound.
 *
 * The original refuses anything outside [30, 100]: the miner NEVER goes
 * below 30 %, even when cold. It stores the value in dev_info_g then calls
 * the setter through a function pointer — the one sl_wrapper_init installed
 * there, i.e. fan_speed_set. We call it directly.
 */
int fan_pwm_set(int pwm)
{
    if (pwm < PWM_MIN || pwm > PWM_MAX) {
        jas_log("fan_pwm_set: setpoint %d out of [%d, %d], ignored",
                pwm, PWM_MIN, PWM_MAX);
        return 0;
    }
    pwm_current = pwm;
    fan_speed_set(pwm);
    return 1;
}

int jas_pwm_current(void) { return pwm_current; }

/* BubbleSort @ 0x45f32 — three items, might as well keep it as is. */
static void sort(int *t, int n)
{
    int i, j;

    for (i = 0; i < n - 1; i++)
        for (j = 0; j < n - 1 - i; j++)
            if (t[j] > t[j + 1]) {
                int x = t[j]; t[j] = t[j + 1]; t[j + 1] = x;
            }
}

/*
 * temperature_filter @ 0x45f98 — a ROBUST maximum.
 *
 * Three precautions, in this order:
 *
 *   1. values outside [-29, 89] are left out of the average — they are
 *      silent (-64) or aberrant sensors;
 *   2. if the maximum exceeds the average by more than twenty degrees, it is
 *      REJECTED and the next one is taken. A single runaway sensor thus
 *      cannot race the fans on its own;
 *   3. symmetrically, a minimum below -20 is discarded.
 *
 * Returns 0 if no measurement is usable — in which case max and min are -64,
 * and the caller must treat that as "no temperature", not as "very cold".
 */
int temperature_filter(const int *temp, int board_id, int *max_temp,
                       int *min_temp, char *max_temp_asic_num)
{
    int t[3], sum = 0, kept = 3, average = 0, k;
    int rank;

    if (!(BOARD_EN >> board_id & 1))
        return 0;

    t[0] = temp[0]; t[1] = temp[1]; t[2] = temp[2];
    if (t[0] == T_MISSING && t[1] == T_MISSING && t[2] == T_MISSING) {
        *max_temp = T_MISSING;
        *min_temp = T_MISSING;
        jas_log("all temperatures are invalid, max_temp = %d, min_temp = %d",
                *max_temp, *min_temp);
        return 0;
    }

    sort(t, 3);

    for (k = 0; k < 3; k++) {
        if (t[k] >= T_MIN_VALID && t[k] <= T_MAX_VALID) {
            sum += t[k];
        } else {
            kept--;
            jas_log("board[%d] sum_temp discard temp = %d, dicard num = %d",
                    board_id, t[k], 3 - kept);
        }
    }
    if (kept > 0)
        average = sum / kept;

    rank = 2;
    while (rank > 0 && average + OUTLIER_GAP < t[rank])
        rank--;
    *max_temp = t[rank];
    if (max_temp_asic_num)
        *max_temp_asic_num = (char)rank;

    k = 0;
    while (k < 2 && t[k] < -20)
        k++;
    *min_temp = t[k];
    return 1;
}

/*
 * set_PWM_according_to_temperature_x16 @ 0x445d8 — the regulation itself.
 *
 * To be called REGULARLY, at least once a second: this repeated call is what
 * keeps the FPGA watchdog from putting everything back to 100 %.
 */
void set_PWM_according_to_temperature_x16(void)
{
    int board, t_max_global = T_MISSING;
    int pwm;

    for (board = 0; board < (int)BOARD_NUM; board++) {
        int temp[3], tmax, tmin;
        char rank = 0;
        unsigned chip;

        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < 3; chip++)
            temp[chip] = (int)jas_asic_temp[board][chip];
        if (!temperature_filter(temp, board, &tmax, &tmin, &rank))
            continue;
        if (tmax > t_max_global)
            t_max_global = tmax;
    }

    /*
     * No usable temperature. Do NOT lower the fans: we do not know whether
     * it is hot, and silence is not information.
     */
    if (t_max_global == T_MISSING) {
        fan_pwm_set(PWM_MAX);
        return;
    }

    /* COLD STATE: outside mining and below T_COLD_OFF, fans cut. */
    if (fan_cold_allowed()) {
        if (jas_mining_active || t_max_global >= T_COLD_ON) {
            if (fan_cold) {
                fan_cold = 0;
                temp_previous = -1000;      /* force the curve to be recomputed */
                jas_log("fans: RESUMED (chips at %d C%s)", t_max_global,
                        jas_mining_active ? ", mining" : "");
            }
        } else if (t_max_global < T_COLD_OFF && !fan_cold) {
            fan_cold = 1;
            fan_cold_rounds = 0;
            jas_log("fans: STOPPED — chips at %d C, not mining (resume at %d C"
                    " or when mining)", t_max_global, T_COLD_ON);
        }
        if (fan_cold) {
            pwm_current = 0;
            fan_speed_set(0);
            temp_previous = t_max_global;
            if (++fan_cold_rounds % 60 == 1) {
                int v[4];

                fan_speed_read(v);
                jas_log("fans stopped: chips at %d C, fans %d/%d/%d/%d rpm",
                        t_max_global, v[0], v[1], v[2], v[3]);
            }
            return;
        }
    }

    /* Hysteresis: less than two degrees apart, leave everything alone. */
    if (temp_previous > -1000 &&
        t_max_global - temp_previous < HYSTERESIS &&
        temp_previous - t_max_global < HYSTERESIS) {
        fan_pwm_set(pwm_current);      /* refresh anyway! */
        return;
    }

    if (t_max_global < T_LOW) {
        pwm = PWM_MIN;
    } else if (t_max_global < T_KNEE) {
        pwm = (int)((float)(t_max_global - (T_LOW - 1)) * SLOPE_NUM / SLOPE_DEN + 30.0f);
    } else if (t_max_global <= T_HIGH) {
        pwm = (int)(PWM_KNEE + (float)(t_max_global - (T_KNEE - 1)) * SLOPE_NUM * 0.25f);
    } else {
        pwm = PWM_MAX;
    }
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    fan_pwm_set(pwm);
    temp_previous = t_max_global;
}
