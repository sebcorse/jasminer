/*
 * Collecting the mining results.
 *
 * Each ASIC carries four meshes of thirty ALUs, one hundred and twenty units
 * per chip — this layout explains why the nonce ranges handed out at each job
 * progress in steps of thirty.
 *
 * Collection goes in two stages. A global register tells which slots have
 * produced something, four bits per chip, one per mesh. Then, for a given
 * slot, a second register tells which of the mesh's thirty ALUs found it, and
 * the result is read at an address computed from that index.
 *
 * Important point for multi-coin mining: the loop stops at the first result
 * reported. A chip producing non-stop thus occupies the leading slots and
 * starves the following ones — this is how an ASIC with an incomplete DAG can
 * drop the output of a perfectly healthy neighbour to zero. Returning zero
 * after handling a result ourselves lets the loop go on and removes that
 * starvation.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "sl_transfer.h"
#include <string.h>

#define ALUS_PER_MESH   30
#define MESHES_PER_ASIC 4
#define RESULT_WORDS    0x14    /* 20 words = nonce (8 B) + mixhash (32 B) */

#define REG_SLOT_MASK   0x80    /* which slots hold a result */
#define REG_ALU_MASK    0x7f    /* which ALU of the mesh, and acknowledge */
#define REG_RESULT_BASE 0x81    /* + alu * 20                             */
#define READ_TIMEOUT    4000000

/* sl_transfer.c:4939 */
int get_asic_result_operation_calc(int thread_id, int asic, int mesh,
                                   gooddata_detail *detail)
{
    uint16_t result_tmp[RESULT_WORDS];
    uint32_t alu_mask = 0, ack;
    uint8_t mesh_bits, cmd_read, asic_en;
    int alu;

    if (!(ASIC_EN(thread_id) >> asic & 1))
        return 0;

    mesh_bits = (uint8_t)(0x10u << mesh);
    cmd_read  = (uint8_t)(mesh_bits | 0xcu);
    asic_en   = (uint8_t)(1u << asic);

    memset(result_tmp, 0, sizeof(result_tmp));
    sl_multi_read(asic_en, cmd_read, REG_ALU_MASK,
                  (uint16_t *)&alu_mask, 2, READ_TIMEOUT, thread_id);

    /* The first ALU of the mesh announcing a result, if there is one. */
    for (alu = 0; alu < ALUS_PER_MESH; alu++)
        if (alu_mask >> alu & 1)
            break;
    if (alu == ALUS_PER_MESH)
        return 0;

    sl_multi_read(asic_en, cmd_read,
                  (uint16_t)(REG_RESULT_BASE + alu * RESULT_WORDS),
                  result_tmp, RESULT_WORDS, READ_TIMEOUT, thread_id);

    /* Acknowledge: this ALU's bit is written back to free the slot. */
    ack = 1u << alu;
    sl_multi_write(asic_en, (uint8_t)(mesh_bits | 0x8u), REG_ALU_MASK,
                   (uint16_t *)&ack, 2, thread_id);

    memcpy(&detail->good_data, result_tmp, sizeof(gooddata));
    detail->board_id = (uint8_t)thread_id;
    detail->asic_id  = (uint8_t)asic;
    detail->alu_id   = (uint8_t)(alu + mesh * ALUS_PER_MESH);
    return 1;
}

/* sl_transfer.c:4972 */
int get_asic_result_operation(int thread_id, gooddata_detail *detail)
{
    static unsigned start;      /* where to start the next scan from */
    uint16_t slots = 0;
    unsigned n, slot, k;

    sl_single_read(0xff, 0xfd, REG_SLOT_MASK, &slots, READ_TIMEOUT, thread_id);
    if (slots == 0)
        return 0;

    /*
     * ROTATING START: the original always scans from slot zero and stops at
     * the first one that returns something, so a prolific chip occupies the
     * leading slots and starves the others (in dual mining, a chip's nonces
     * could wait minutes and arrive stale). Starting each time from the next
     * slot gives each chip the front in turn, at no cost.
     */
    n = (unsigned)ASIC_NUM * MESHES_PER_ASIC;
    for (k = 0; k < n; k++) {
        slot = (start + k) % n;
        if (!(slots >> slot & 1))
            continue;
        if (get_asic_result_operation_calc(thread_id,
                                           (int)(slot >> 2),
                                           (int)(slot & 3), detail)) {
            start = (slot + 1) % n;
            return 1;
        }
    }
    return 0;
}

/*
 * FLUSH THE SLOTS AFTER A HEADER CHANGE: a result in an ALU slot was
 * computed with the header in the registers at that time; once a new one is
 * pushed, everything pending belongs to the old work, and ethash_nonce_check
 * would count it as a HARDWARE FAULT although the chip computed perfectly
 * (e.g. the burst of results computed on the leftover header right after
 * the DAG build).
 *
 * No need to READ them: writing into REG_ALU_MASK the mask just read from it
 * acknowledges every slot at once (twelve reads and twelve writes per board,
 * once per job). A nonce found just before the switch is lost, but the mix
 * check, judging against the new header, would have rejected it anyway.
 *
 * ONLY THE CHIPS THAT CHANGE WORK: in dual-coin mode the two pools push
 * independently, and a chip whose header did not move keeps its pending
 * results.
 */
void purge_results(int thread_id, uint8_t asic_mask)
{
    int asic, mesh, empty = 0;

    for (asic = 0; asic < (int)ASIC_NUM; asic++) {
        uint8_t asic_en;

        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;
        if (!(asic_mask >> asic & 1))
            continue;
        asic_en = (uint8_t)(1u << asic);

        for (mesh = 0; mesh < MESHES_PER_ASIC; mesh++) {
            uint8_t mesh_bits = (uint8_t)(0x10u << mesh);
            uint32_t mask = 0;

            sl_multi_read(asic_en, (uint8_t)(mesh_bits | 0xcu), REG_ALU_MASK,
                          (uint16_t *)&mask, 2, READ_TIMEOUT, thread_id);
            if (!mask)
                continue;
            sl_multi_write(asic_en, (uint8_t)(mesh_bits | 0x8u), REG_ALU_MASK,
                           (uint16_t *)&mask, 2, thread_id);
            empty++;
        }
    }
    if (empty)
        jas_log("board %d: %d meshes purged of stale results",
                thread_id, empty);
}
