/*
 * Fixed parameters of the memory controller.
 *
 * Twenty-five timing settings, pushed as they are into the chips at each
 * initialization: delays between activating a row and its word select,
 * between the sense amplifier and the column, refresh periods. They are the
 * electrical characteristics of the memory, and nothing reads correctly if
 * they are wrong.
 *
 * They come from a block of contiguous globals. Rather than rebuild a
 * structure — where an alignment offset would silently break everything —
 * the HAL exposes them through a table pairing each register with its value.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdlib.h>   /* getenv, strtoul */

void config_asic_fix_param(int thread_id)
{
    const jas_fix_param *p;
    unsigned n = 0;
    const char *e = getenv("JASRE_MC_FREQ");
    unsigned freq = (e && *e) ? (unsigned)strtoul(e, NULL, 10) : 0;  /* 0 = no rescaling */
    const char *eri = getenv("JASRE_MC_REQ_INTERVAL");
    long req_iv = (eri && *eri) ? strtol(eri, NULL, 0) : -1;  /* -1 = leave alone (0x2da stock) */

    for (p = jas_fix_params(); p->reg; p++, n++) {
        /*
         * REFRESH TIMINGS RESCALED for the actual frequency.
         *
         * The table is calibrated for 500 MHz (cycles = t_ns*500). At another
         * frequency, the REAL interval between refreshes = cycles/freq drifts:
         * below 500 MHz we refresh LESS often in real time -> marginal retention
         * -> corrupted DAG. The THREE big refresh counters are rescaled by
         * freq/500, stored as 32-bit pairs {high16, low16}:
         *   ref_start_time 0xce0, ref_period 0xcec, ref_pre_period 0x1c00.
         * Access latencies are ceil(cycles*0.9), unchanged (already over-timed
         * at lower frequency), so they are left alone. Active only if
         * JASRE_MC_FREQ is set; otherwise stock behaviour (500).
         */
        if (freq && freq != 500 && p->count >= 2 && p->count <= 16 &&
            (p->reg == 0xce0 || p->reg == 0xcec || p->reg == 0x1c00)) {
            const uint16_t *src = (const uint16_t *)p->value;
            uint16_t tmp[16];
            unsigned k;

            for (k = 0; k + 1 < p->count; k += 2) {
                uint32_t v = ((uint32_t)src[k] << 16) | src[k + 1];
                uint32_t s = (uint32_t)(((uint64_t)v * freq) / 500u);
                tmp[k]     = (uint16_t)(s >> 16);
                tmp[k + 1] = (uint16_t)(s & 0xffffu);
            }
            sl_multi_write(ASIC_EN(thread_id), p->cmd, p->reg, tmp, p->count, thread_id);
            continue;
        }

        /*
         * OVERRIDE of req_interval (0x2da) AT INIT: spacing between MC
         * requests in cycles (0 = back to back). The MC latches this register
         * at init, so setting it later has no effect. Active only if
         * JASRE_MC_REQ_INTERVAL is set.
         */
        if (req_iv >= 0 && p->reg == 0x2da) {
            sl_single_write(ASIC_EN(thread_id), p->cmd, p->reg, (uint16_t)req_iv, thread_id);
            jas_log("board %d: req_interval (0x2da) forced to %u at init", thread_id, (unsigned)req_iv);
            continue;
        }

        if (p->count > 1)
            sl_multi_write(ASIC_EN(thread_id), p->cmd, p->reg,
                           (uint16_t *)p->value, p->count, thread_id);
        else
            sl_single_write(ASIC_EN(thread_id), p->cmd, p->reg,
                            *(const uint16_t *)p->value, thread_id);
    }
    if (freq && freq != 500)
        jas_log("board %d: refresh timings recomputed for %u MHz", thread_id, freq);
    jas_log("board %d: %u timing parameters set", thread_id, n);
}
