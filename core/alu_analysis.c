/*
 * Analysis of the compute units' self-test.
 *
 * alu_check reads six sixteen-bit words per mesh. Each ALU takes THREE
 * consecutive bits, and is declared good only if all three are set — a
 * unanimous vote, not a majority. Thirty ALUs per mesh: sixteen described by
 * the first three words, fourteen by the next three.
 *
 * The verdict feeds two things: a flat array, one flag per ALU, and the
 * enable mask that header_update_operation writes again at each job.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"

#define N_MESH         4u
#define ALUS_PER_MESH  30u
#define N_ALU          (N_MESH * ALUS_PER_MESH)
#define BITS_PER_ALU   3u
#define ALU_GOOD       0x7u    /* the three bits */

/* Extracts an ALU's three bits from a group of three sixteen-bit words. */
static unsigned alu_bits(const uint16_t *words, unsigned index)
{
    uint32_t lo = (uint32_t)words[0] | ((uint32_t)words[1] << 16);
    uint32_t hi = words[2];
    unsigned shift = index * BITS_PER_ALU;

    if (shift < 32)
        return (unsigned)((lo >> shift) | (hi << (32 - shift))) & ALU_GOOD;
    return (unsigned)(hi >> (shift - 32)) & ALU_GOOD;
}

void alu_check_analysis(int thread_id)
{
    unsigned asic, mesh, k;

    for (k = 0; k < ASIC_NUM * N_ALU; k++)
        jas_alu_flag[thread_id][k] = 0;

    for (asic = 0; asic < ASIC_NUM; asic++) {
        unsigned good = 0;

        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;

        for (mesh = 0; mesh < N_MESH; mesh++) {
            const uint16_t *res = &jas_alu_result[thread_id][asic][mesh * 6];
            uint16_t en_lo = 0, en_hi = 0;

            /* Sixteen ALUs described by words 0 to 2. */
            for (k = 0; k < 16; k++) {
                int ok = (alu_bits(&res[0], k) == ALU_GOOD);
                jas_alu_flag[thread_id][asic * N_ALU + mesh * ALUS_PER_MESH + k] =
                    (uint16_t)(ok ? 1 : 0);
                if (ok) { en_lo |= (uint16_t)(1u << k); good++; }
            }
            /* Fourteen more by words 3 to 5, thirty in all. */
            for (k = 0; k < 14; k++) {
                int ok = (alu_bits(&res[3], k) == ALU_GOOD);
                jas_alu_flag[thread_id][asic * N_ALU + mesh * ALUS_PER_MESH + 16 + k] =
                    (uint16_t)(ok ? 1 : 0);
                if (ok) { en_hi |= (uint16_t)(1u << k); good++; }
            }

            jas_alu_en[thread_id][asic][mesh][0] = en_lo;
            jas_alu_en[thread_id][asic][mesh][1] = en_hi;

            /*
             * THE SIX RAW WORDS, logged as they are (one line per mesh, once at
             * startup): a contiguous hole at the same place on all four meshes
             * blames the read, a scattered pattern blames the chip.
             */
            jas_log("board %d asic %u mesh %u: alu_check raw %04x %04x %04x"
                    " %04x %04x %04x -> %04x/%04x",
                    thread_id, asic, mesh,
                    res[0], res[1], res[2], res[3], res[4], res[5],
                    en_lo, en_hi);
        }

        jas_good_alu_num[thread_id][asic] = (uint16_t)good;
        jas_log("board %d asic %u: %u alu of %u", thread_id, asic, good, N_ALU);
    }
}
