/*
 * Handing the work out to the ASICs.
 *
 * The nonce space is already split per chip in the original, and that is good
 * news: nothing to invent on that side.
 *
 *   nonce_base = (extranonce << shift)
 *              + board * number_of_chips * 120
 *              + chip * 120
 *   mesh 0 -> nonce_base      mesh 1 -> +30
 *   mesh 2 -> +60             mesh 3 -> +90
 *
 * One hundred and twenty nonces per chip, thirty per mesh: exactly the thirty
 * ALUs per mesh that the result collection confirms on its side. The shift is
 * 48 bits if the extranonce fits in sixteen bits, 40 otherwise — which leaves
 * the low half of the nonce to the hardware search.
 *
 * The header and the target, on the other hand, are BROADCAST from globals.
 * It is the last place where the "single coin" assumption survives after
 * dag_calc, and our version takes them from each chip's context.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "jas_coin.h"
#include "sl_transfer.h"
#include <string.h>

#define NONCES_PER_ASIC  0x78u   /* 120 */
#define NONCES_PER_MESH  0x1eu   /* 30  */
#define MESHES           4

#define REG_DAG_SIZE     0x08
#define REG_NONCE        0x0c
#define REG_TARGET       0x10
#define REG_HEADER       0x20
#define REG_ALU_EN       0x40
#define REG_SOFT_RESET   0x830
#define CMD_BROADCAST    0xf8

/* 0x18, 0x28, 0x48, 0x88: the target mesh, with 8 for a write. */
#define CMD_MESH_W(m)    ((uint8_t)((0x10u << (m)) | 0x8u))

/*
 * The target is sent with its bytes reversed relative to its representation
 * in memory — the hardware expects it in reverse order.
 */
static void target_to_wire(uint8_t out[32], const ethash_h256_t *target)
{
    for (int i = 0; i < 32; i++)
        out[i] = target->b[31 - i];
}

/*
 * WHAT EACH CHIP ALREADY HAS IN ITS REGISTERS: REG_DAG_SIZE only changes
 * at an EPOCH change and REG_TARGET when the pool's DIFFICULTY changes,
 * both rare; rewriting them at every job was 20 of the 52 writes per chip,
 * under the bus lock, every six seconds. Above all, the bus traffic then
 * reflects exactly what CHANGES, which keeps a trace readable.
 *
 * The memo is indexed [board][chip] and invalidated by jas_work_memo_forget,
 * called after any chip reset (otherwise we would believe the chip holds a
 * value it has lost).
 */
static struct {
    uint64_t dag_size;
    uint8_t  target[32];
    int      valid;
} memo_chip[3][3];

void jas_work_memo_forget(int thread_id, uint8_t asic_mask)
{
    unsigned a;

    if (thread_id < 0 || thread_id > 2)
        return;
    for (a = 0; a < 3; a++)
        if (asic_mask >> a & 1)
            memo_chip[thread_id][a].valid = 0;
}

void header_update_operation(int thread_id, uint8_t asic_mask)
{
    uint8_t asic_en = (uint8_t)(ASIC_EN(thread_id) & asic_mask);
    uint8_t target_wire[32];
    uint64_t extranonce = *jas_extranonce;
    unsigned shift, asic, mesh;

    if (!asic_en)
        return;

    if (extranonce == 0)
        extranonce = *jas_dev_id;
    /* Neither one is ever written: without this third source, the four
     * machines sweep the SAME range. See jas_identity_nonce. */
    if (extranonce == 0)
        extranonce = jas_identity_nonce();
    shift = (extranonce >> 16) ? 40 : 48;

    if (*jas_alu_close_flag) {
        /* TWO WORDS: a mesh's mask takes 0x40 AND 0x41 (16 + 14 ALUs);
         * with a single word, fourteen ALUs per mesh stay on. The original
         * writes both. */
        uint16_t zero2[2] = { 0, 0 };

        sl_multi_write(asic_en, CMD_BROADCAST, REG_ALU_EN, zero2, 2, thread_id);
    }

    for (asic = 0; asic < ASIC_NUM; asic++) {
        const jas_coin_ctx *coin = jas_coin_of(thread_id, asic);
        uint8_t mask = (uint8_t)(1u << asic);
        uint64_t nonce;

        if (!(asic_en >> asic & 1))
            continue;

        /*
         * DAG size, header and target: per chip here, broadcast in the
         * original. This is what lets a chip work on another coin — provided
         * its DAG was built accordingly, which dag_calc takes care of with the
         * same context.
         *
         * Only the HEADER goes out every time: it is the only one of the three
         * that changes from one job to the next.
         */
        target_to_wire(target_wire, &coin->target);

        if (!memo_chip[thread_id][asic].valid ||
            memo_chip[thread_id][asic].dag_size != coin->dag_size) {
            sl_multi_write(mask, CMD_BROADCAST, REG_DAG_SIZE,
                           (uint16_t *)&coin->dag_size, 4, thread_id);
            memo_chip[thread_id][asic].dag_size = coin->dag_size;
        }

        sl_multi_write(mask, CMD_BROADCAST, REG_HEADER,
                       (uint16_t *)&coin->header, 16, thread_id);

        if (!memo_chip[thread_id][asic].valid ||
            memcmp(memo_chip[thread_id][asic].target, target_wire, 32) != 0) {
            sl_multi_write(mask, CMD_BROADCAST, REG_TARGET,
                           (uint16_t *)target_wire, 16, thread_id);
            memcpy(memo_chip[thread_id][asic].target, target_wire, 32);
        }
        memo_chip[thread_id][asic].valid = 1;

        /* Nonce split: already per chip and per mesh in the original. */
        nonce = ((uint64_t)extranonce << shift)
              + (uint64_t)thread_id * ASIC_NUM * NONCES_PER_ASIC
              + (uint64_t)asic * NONCES_PER_ASIC;

        for (mesh = 0; mesh < MESHES; mesh++) {
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_NONCE,
                           (uint16_t *)&nonce, 4, thread_id);
            nonce += NONCES_PER_MESH;
        }
    }

    /*
     * The ALUs were switched off during reprogramming: we reset the chip and
     * then restore the enable mask, mesh by mesh.
     *
     * asic_en is already restricted to the mask: a chip outside the mask
     * suffers neither the switching off of its ALUs nor the soft reset. That
     * is the point of the whole change — in dual-coin mode, an ETC job must
     * not do anything to the chip mining ETHW.
     */
    if (*jas_alu_close_flag) {
        sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0, thread_id);
        sl_delay(0, 2000);
        sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, thread_id);

        for (asic = 0; asic < ASIC_NUM; asic++) {
            uint8_t mask = (uint8_t)(1u << asic);
            if (!(asic_en >> asic & 1))
                continue;
            for (mesh = 0; mesh < MESHES; mesh++)
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_ALU_EN,
                               jas_alu_en[thread_id][asic][mesh], 2, thread_id);
        }
        /*
         * WE DO NOT KNOW whether REG_SOFT_RESET clears the job registers.
         * The original writes the header BEFORE this reset and does not come
         * back to it, which suggests it does not — but "suggests" is not
         * "checked", and the memo is precisely what would make us SKIP a
         * write. Faced with an uncertainty that costs one write on one side
         * and a lost job on the other, we forget.
         */
        jas_work_memo_forget(thread_id, asic_en);
        *jas_alu_close_flag = 0;
    }
}
