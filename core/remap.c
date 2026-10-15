/*
 * Memory bank remapping table.
 *
 * Each ASIC exposes 4080 logical bank slots, part of which are defective —
 * the memory test typically declares 3870 to 3930 of them usable. The
 * remapping table packs the good ones into a contiguous space and sends the
 * bad ones out of range, so that the hashing engine addresses a continuous
 * DAG without knowing the memory has holes.
 *
 * The number of really usable banks is then announced to the chip (registers
 * 0xb03 and 0xb07). It is this value, not the DAG size, that bounds what the
 * chip can address.
 *
 * Reads and writes have two separate tables — registers 0xb00 to 0xb02 for
 * writing, 0xb04 to 0xb06 for reading — which allows, with remap_flag = 2,
 * writing into partially good banks while reading back only from perfectly
 * good ones.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"

#define BANKS_PER_ASIC  0xff0u   /* 4080 logical slots */
#define BANK_OUT        0xfffu   /* "out of range" index */

#define REG_W_PHYS      0xb00
#define REG_W_LOGIC     0xb01
#define REG_W_COMMIT    0xb02
#define REG_W_COUNT     0xb03
#define REG_R_PHYS      0xb04
#define REG_R_LOGIC     0xb05
#define REG_R_COMMIT    0xb06
#define REG_R_COUNT     0xb07

#define CMD_BROADCAST   0xf8
#define CMD_MESH0       0x18

/* remap_flag: 0 = perfect banks only, 1 = partial ones, 2 = write into
 * partial ones but read back from perfect ones only. */
int config_remap_info(int thread_id, uint8_t asic, uint8_t remap_flag)
{
    const uint16_t *w_src, *r_src;
    uint8_t asic_en = (uint8_t)(1u << asic);
    uint16_t logic, w_count = 0, r_count = 0;

    switch (remap_flag) {
    case 1:
        w_src = r_src = jas_bank_partial[thread_id];
        break;
    case 2:
        w_src = jas_bank_partial[thread_id];
        r_src = jas_bank_good[thread_id];
        break;
    case 0:
    default:
        w_src = r_src = jas_bank_good[thread_id];
        break;
    }
    w_src += (unsigned)asic * BANKS_PER_ASIC;
    r_src += (unsigned)asic * BANKS_PER_ASIC;

    for (logic = 0; logic < BANKS_PER_ASIC; logic++) {
        if (w_src[logic] == 0) {
            /* Unusable bank: sent out of the addressable space.
             * The offset (w_count + 0xfef) - logic keeps what comes before
             * contiguous, so the hole does not shift what follows. */
            sl_single_write(asic_en, CMD_BROADCAST, REG_W_PHYS,
                            (uint16_t)((w_count + 0xfefu) - logic), thread_id);
            sl_single_write(asic_en, CMD_BROADCAST, REG_W_LOGIC, BANK_OUT, thread_id);
        } else {
            sl_single_write(asic_en, CMD_BROADCAST, REG_W_PHYS, w_count, thread_id);
            sl_single_write(asic_en, CMD_BROADCAST, REG_W_LOGIC, logic, thread_id);
            w_count++;
        }
        sl_single_write(asic_en, CMD_BROADCAST, REG_W_COMMIT, 1, thread_id);

        if (r_src[logic] == 0) {
            sl_single_write(asic_en, CMD_MESH0, REG_R_PHYS,
                            (uint16_t)((r_count + 0xfefu) - logic), thread_id);
            sl_single_write(asic_en, CMD_MESH0, REG_R_LOGIC, BANK_OUT, thread_id);
        } else {
            sl_single_write(asic_en, CMD_MESH0, REG_R_PHYS, r_count, thread_id);
            sl_single_write(asic_en, CMD_MESH0, REG_R_LOGIC, logic, thread_id);
            r_count++;
        }
        sl_single_write(asic_en, CMD_MESH0, REG_R_COMMIT, 1, thread_id);
    }

    /* What the chip will believe it owns. */
    sl_single_write(asic_en, CMD_BROADCAST, REG_W_COUNT, w_count, thread_id);
    sl_single_write(asic_en, CMD_MESH0,     REG_R_COUNT, r_count, thread_id);

    jas_trace("board %d asic %u: %u banks for write, %u for read",
            thread_id, asic, w_count, r_count);
    return 0;
}

/* Starts again from the memory test's reference state. */
void modify_remap_data(int thread_id, uint16_t asic)
{
    unsigned base = (unsigned)asic * BANKS_PER_ASIC, k;

    for (k = 0; k < BANKS_PER_ASIC; k++)
        jas_bank_partial[thread_id][base + k] =
            jas_bank_partial_ref[thread_id][base + k];
}

/*
 * dag_remap_data: AMPUTATE, not kill. The original turns off a WINDOW and
 * copies the rest back from the reference arrays:
 *
 *     mesh_bank = 1   window of 0x3fc = 1020 banks, i.e. a whole mesh,
 *                     from (mesh_flag - 1) * 1020   -> "close mesh_N"
 *     mesh_bank = 2   window of 0xff  =  255 banks from
 *                     (mesh_flag - 1) * 1020 + (bank_flag - 1) * 255
 *                                                         -> "close bank255_N"
 *
 * Outside the window, the flags are TAKEN AGAIN from the reference copies
 * (_d for good ones, _c for partial ones): each call starts again from the
 * original state and amputates only the requested window, so two successive
 * amputations replace each other instead of adding up. (Marking all the
 * chip's banks bad instead would lose the chip at the first retry.)
 */
void dag_remap_data(int thread_id, uint16_t asic, uint8_t mesh_flag,
                    uint8_t bank_flag, uint8_t mesh_bank)
{
    unsigned base = (unsigned)asic * BANKS_PER_ASIC;
    unsigned start, length, j;

    if (!mesh_flag || !bank_flag)
        return;

    if (mesh_bank == 1) {
        start    = (mesh_flag - 1u) * 1020u;
        length = 1020u;
    } else if (mesh_bank == 2) {
        start    = (mesh_flag - 1u) * 1020u + (bank_flag - 1u) * 255u;
        length = 255u;
    } else {
        return;
    }

    for (j = 0; j < BANKS_PER_ASIC; j++) {
        unsigned k = base + j;

        if (j < start || j >= start + length) {
            uint16_t good = jas_bank_good_ref[thread_id][k];

            jas_bank_good[thread_id][k]    = good;
            jas_bank_partial[thread_id][k] = jas_bank_partial_ref[thread_id][k];
            jas_bank_bad[thread_id][k]     = (uint16_t)(good == 0);
        } else {
            jas_bank_good[thread_id][k]    = 0;
            jas_bank_partial[thread_id][k] = 0;
            jas_bank_bad[thread_id][k]     = 1;
        }
    }
}
