/*
 * Real-time hashrate measured on the stream of ACCEPTED nonces, PER CHIP.
 *
 * The NOMINAL throughput of the silicon (get_eff, register 0x43) is a
 * theoretical capacity, constant by nature: a silent ASIC, a truncated DAG
 * or broken ALUs do not show in it. Like the original, rt and avg are
 * measured on the work REALLY validated. jas_accept_num[9] (accepts PER
 * CHIP, indexed board*3+asic, see nonce_check.c) is sampled periodically:
 *
 *     throughput (H/s) = Σ_chip ( Δaccepts_chip × difficulty_chip ) / Δt
 *
 * DIFFICULTY PER CHIP: each chip is judged with the target of ITS coin,
 * 2^256 / jas_coin_of(chip)->target. In dual mining the ETHW and ETC
 * targets differ widely, and a single global target (the last job's) would
 * make the total jump around.
 *
 * rt  = throughput over a short window (~600 s); avg = long window (~40 min).
 * MONOTONIC clock (never time(), which jumps at startup).
 */
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"

/* Declared (defined) in nonce_check.c, not in jas_state.h. */
extern uint32_t jas_accept_num[9];

#define NASIC     9u       /* 3 boards x 3 chips, indexed board*3 + asic */
#define NS        512u     /* samples; at ~5 s, the buffer covers ~42 min */
#define STEP_S     5.0      /* minimum sampling period */
#define WIN_RT    600.0    /* instant window */
#define WIN_AVG   2400.0   /* average window (~40 min) */

static struct { double t; unsigned long long acc[NASIC]; } buf[NS];
static unsigned filled, head;
static pthread_mutex_t mx = PTHREAD_MUTEX_INITIALIZER;

static double mono(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void snapshot_accepts(unsigned long long *out)
{
    unsigned k;
    for (k = 0; k < NASIC; k++)
        out[k] = jas_accept_num[k];
}

/*
 * Difficulty of a chip = 2^256 / target of ITS coin. The target comes from
 * the per-chip coin context (jas_coin_of, 32 bytes most significant first),
 * renewed at each job — hence right whether the chip is on ETHW or ETC.
 */
static double difficulty_chip(unsigned k)
{
    const jas_coin_ctx *c = jas_coin_of((int)(k / 3u), k % 3u);
    double v = 0;
    int i;

    if (!c)
        return 0;
    for (i = 0; i < 32; i++)
        v = v * 256.0 + (double)c->target.b[i];
    if (v <= 0)
        return 0;
    return ldexp(1.0, 256) / v;      /* 2^256 / target */
}

void jas_hashrate_rt_sample(void)
{
    double now = mono();
    unsigned long long acc[NASIC];

    snapshot_accepts(acc);

    pthread_mutex_lock(&mx);
    if (filled == 0 || now - buf[(head - 1u + NS) % NS].t >= STEP_S) {
        buf[head].t = now;
        memcpy(buf[head].acc, acc, sizeof(acc));
        head = (head + 1u) % NS;
        if (filled < NS)
            filled++;
    }
    pthread_mutex_unlock(&mx);
}

/*
 * Throughput in MH/s over the window [now-window, now]: sum, PER CHIP, of
 * Δaccepts × difficulty of its coin. Called under mx.
 */
static double hashrate_window(double now, const unsigned long long *acc_now,
                            double window)
{
    unsigned i, k;
    int best = -1;
    double target = now - window, best_gap = 1e18, dt, total = 0;

    if (filled < 2)
        return 0;

    for (i = 0; i < filled; i++) {
        unsigned idx = (head - 1u - i + 2u * NS) % NS;
        double gap = buf[idx].t - target;

        if (gap < 0)
            gap = -gap;
        if (gap < best_gap) {
            best_gap = gap;
            best = (int)idx;
        }
    }
    if (best < 0)
        return 0;

    dt = now - buf[best].t;
    if (dt < 1.0)
        return 0;

    for (k = 0; k < NASIC; k++) {
        double diff = difficulty_chip(k);
        unsigned long long dacc;

        if (diff <= 0)
            continue;
        dacc = acc_now[k] - buf[best].acc[k];   /* wrap-safe on unsigned */
        total += (double)dacc * diff / dt;
    }
    return total / 1e6;      /* H/s -> MH/s */
}

/*
 * Empty the rt ring buffer, at EACH change of active pool: hashrate_window
 * values the accepts with the CURRENT share difficulty, so after a switch
 * the accepts earned under the OLD difficulty, still in the window, would be
 * revalued (a spike). Only the rt history is reset (jas_accept_num and the
 * pool counters stay monotonic: they feed the API). hashrate_window returns 0
 * while filled < 2 (~5-10 s), then re-estimates over a window that widens to
 * 600 s: a small dip instead of a spike.
 */
void jas_hashrate_rt_reset(void)
{
    pthread_mutex_lock(&mx);
    filled = 0;
    head   = 0;
    pthread_mutex_unlock(&mx);
}

void jas_hashrate_rt_get(double *rt, double *avg)
{
    static int calibrated;
    double now;
    unsigned long long acc[NASIC];

    jas_hashrate_rt_sample();        /* locks/unlocks internally */

    pthread_mutex_lock(&mx);
    now = mono();
    snapshot_accepts(acc);
    *rt  = hashrate_window(now, acc, WIN_RT);
    *avg = hashrate_window(now, acc, WIN_AVG);
    pthread_mutex_unlock(&mx);

    if (!calibrated && *rt > 0) {
        calibrated = 1;
        jas_log("hashrate rt (per chip): calibration rt=%.2f avg=%.2f MH/s",
                *rt, *avg);
    }
}
