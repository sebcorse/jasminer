/*
 * Checking a nonce reported by a chip: ethash_nonce_check @ 0x2dfd2.
 *
 * This is the referee of mining: a chip announces it found a nonce, and this
 * function redoes the computation on the processor to decide whether the share
 * deserves to be sent to the pool. Two conditions, and both matter:
 *
 *   1. the mix digest returned by the chip must be the one the processor
 *      finds again. If it differs, the chip computed wrongly: a hardware
 *      error, the "hw" of the statistics.
 *
 *   2. the result must be below the target. Otherwise the chip computed
 *      correctly but used the wrong threshold: "Find a target fail Nonce!".
 *
 * The first case blames the memory or the ALUs, the second the
 * configuration: the log keeps them apart.
 *
 * THE CACHE CHOICE is exactly that of dram_check_dag, and the decompiled code
 * expands it in the same misleading way (three branches, two of which lead
 * to the same cache):
 *
 *     coin_type != 0 AND block_number > 17800000  ->  ETC cache
 *     otherwise                                   ->  ZIL cache
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"
#include "jas_types.h"
#include "jas_ethash.h"
#include <stdio.h>
#include <string.h>

#define BLOCK_SWITCH_ETC   17800000ull
#define PERIOD_STATS      200      /* one statistics line every 200 */

/*
 * The counters, as the firmware stores them — four contiguous arrays from
 * 0x38c210, which explains the odd indices of the decompiled code
 * (piVar13[idx + 0xb] for errors, piVar13[idx + 2] for shares).
 */
uint32_t jas_cn;                    /* cn         0x38c210 */
uint32_t jas_cn_tmp;                /* cn_tmp     0x38c214 */
uint32_t jas_accept_num[9];         /* accept_num 0x38c218 */
uint32_t jas_hw_err[9];             /* 0x38c23c            */

/*
 * Comparison of two 256-bit big-endian integers — the format in which a pool
 * expresses its target. The most significant byte comes first, so the first
 * difference found decides.
 */
int isValueLessThenTarget_256bits(const uint8_t *value, const uint8_t *target)
{
    int k;

    for (k = 0; k < 32; k++) {
        if (value[k] != target[k])
            return value[k] < target[k];
    }
    return 0;                       /* equality: not STRICTLY lower */
}

static void trace_bytes(const char *title, const uint8_t *b, int n)
{
    int k;

    printf("%s", title);
    for (k = 0; k < n; k++)
        printf("%02x", b[k]);
    printf("\n");
}

/*
 * The hardware error rate per chip, published in the log once every two
 * hundred. The original computes it in floating point — hw = errors x 100
 * / (errors + shares) — and prints it with two decimals.
 */
static void publish_statistics(void)
{
    unsigned board, chip;

    if (jas_cn % PERIOD_STATS != 0 || jas_cn == jas_cn_tmp)
        return;

    for (board = 0; board < 3; board++) {
        for (chip = 0; chip < 3; chip++) {
            unsigned k = board * 3 + chip;
            uint32_t total = jas_hw_err[k] + jas_accept_num[k];
            double hw = total ? (double)jas_hw_err[k] * 100.0 / (double)total : 0.0;
            char line[256];

            snprintf(line, sizeof(line),
                     "board[%u]asic[%u]hw=%.2f%%, cn=%u\n", board, chip, hw, jas_cn);
            jas_applog(line);
        }
    }
    jas_cn_tmp = jas_cn;
}

int ethash_nonce_check(const gooddata_detail *detail,
                       const uint8_t *headhash, const uint8_t *target)
{
    ethash_light_t cache;
    ethash_return_value_t compute;
    ethash_h256_t header;
    unsigned k;

    /*
     * THE CACHE OF THIS VERY CHIP, not the machine's: a single choice on
     * *jas_coin_type would judge an ETC chip's nonces with the ETHW cache
     * and reject them all. The chip's context carries its block and seed,
     * exactly the registry's key; two chips on the same epoch share the
     * same cache.
     */
    {
        jas_coin_ctx *ctx = jas_coin_of((int)detail->board_id, detail->asic_id);

        cache = ctx ? jas_cache_find(ctx->block_number, &ctx->seed) : NULL;
    }

    if (!cache) {
        /* The original has three messages — "zil / etc / ethw cache is not
         * ready" — depending on the branch taken. They point to the same
         * defect: we have a nonce to check and no cache to do it. */
        jas_log("cache missing, cannot check nonce (coin_type=%u, block=%llu)",
                *jas_coin_type, (unsigned long long)*jas_fpga_block_number);
        return 0;
    }

    memcpy(header.b, headhash, 32);
    memset(&compute, 0, sizeof(compute));
    ethash_light_compute(&compute, cache, &header, detail->good_data.nonce);

    publish_statistics();

    k = (unsigned)detail->board_id * 3u + detail->asic_id;
    if (k >= 9)
        return 0;

    if (memcmp(detail->good_data.mixhash.b, compute.mix_hash.b, 32) != 0) {
        /* The chip did not find the same mix: computation error. */
        trace_bytes("asic result_mix:", detail->good_data.mixhash.b, 32);
        trace_bytes("cpu result_mix:", compute.mix_hash.b, 32);
        trace_bytes("ethash_nonce_check, fpga_headhash = ",
                     (const uint8_t *)jas_fpga_header, 32);
        trace_bytes("ethash_nonce_check, headhash = ", headhash, 32);
        trace_bytes("ethash_nonce_check, target = ",
                     (const uint8_t *)jas_fpga_target, 32);
        jas_hw_err[k]++;
        jas_cn++;
        return 0;
    }

    if (isValueLessThenTarget_256bits(compute.result.b, target)) {
        jas_accept_num[k]++;
        return 1;
    }

    /* Right mix, result above the target: the chip computed correctly but
     * should not have reported this nonce. */
    jas_log("board%u asic%u alu%u Find a target fail Nonce!",
            detail->board_id, detail->asic_id, detail->alu_id);
    jas_hw_err[k]++;
    jas_cn++;
    return 0;
}

/*
 * WHY IS THIS MIX WRONG? Two causes, and they have nothing in common.
 *
 *   - STALE NONCE: the chip computed perfectly, but with the PREVIOUS
 *     header. The result was sitting in its slot when we pushed the new one.
 *     The silicon is healthy, our timing is what loses.
 *   - CORRUPTED DAG: the chip computed with the right header and found
 *     another mix. Its memory differs from what the processor recomputes.
 *
 * The result returned by the chip contains only the nonce and the mix, never
 * the header used — nothing tells them apart directly. But we can REDO the
 * computation against the recent headers: if one of them gives back exactly
 * the chip's mix, the question is settled, and the right way.
 *
 * Cost: a few ethash_light_compute calls, and ONLY on a reject. None on the
 * normal path.
 *
 * Returns the rank of the matching header (1 = the previous one, 2 = the one
 * before...) or 0 if none: in that case it really is the chip's memory.
 */
int ethash_nonce_stale(const gooddata_detail *detail,
                        const uint8_t (*headers)[32], unsigned how_many)
{
    ethash_light_t cache;
    unsigned k;

    /*
     * THE CACHE OF THIS VERY CHIP, not the machine's: a single choice on
     * *jas_coin_type would judge an ETC chip's nonces with the ETHW cache
     * and reject them all. The chip's context carries its block and seed,
     * exactly the registry's key; two chips on the same epoch share the
     * same cache.
     */
    {
        jas_coin_ctx *ctx = jas_coin_of((int)detail->board_id, detail->asic_id);

        cache = ctx ? jas_cache_find(ctx->block_number, &ctx->seed) : NULL;
    }
    if (!cache)
        return 0;

    for (k = 0; k < how_many; k++) {
        ethash_return_value_t compute;
        ethash_h256_t header;

        memcpy(header.b, headers[k], 32);
        memset(&compute, 0, sizeof(compute));
        ethash_light_compute(&compute, cache, &header, detail->good_data.nonce);
        if (memcmp(detail->good_data.mixhash.b, compute.mix_hash.b, 32) == 0)
            return (int)(k + 1);
    }
    return 0;
}
