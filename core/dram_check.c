/*
 * Checking the cache present in a chip's memory.
 *
 * The chip computes its cache itself from the seed it was given. To make sure
 * it is correct, the firmware has it read back its LAST node and compares it
 * with the one it computed on its side:
 *
 *   address_of_last = cache_size / 128 + base
 *
 * OUR VERSION compares with the cache of THIS CHIP'S COIN (a chip mining
 * another coin than the firmware's would otherwise always fail). When the
 * processor does not have it (a cache weighs 68 to 120 MiB and the machine
 * has about 240 MiB free), the check is explicitly SKIPPED rather than
 * falsely validated: a check that cannot be done must say so.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include <stdio.h>
#include "sl_transfer.h"
#include <string.h>
#include <unistd.h>

#define ALUS_PER_MESH   30
#define REG_ALU_SEL     0x700
#define REG_NODE_ADDR   0x702
#define REG_READ_START  0x704
#define REG_READ_VALID  0x705
#define REG_READ_DATA   0x706
#define NODE_WORDS      0x20    /* a node = 32 words = 64 bytes */

/*
 * manual_sel designates one ALU among all those of the chip: the mesh is the
 * quotient by thirty, the ALU within the mesh the remainder. The target ALU
 * is then expressed as one bit in two sixteen-bit words.
 */
static void select_alu(uint16_t manual_sel, uint16_t sel_words[2], uint8_t *cmd_w,
                       uint8_t *cmd_r)
{
    unsigned alu  = manual_sel % ALUS_PER_MESH;
    unsigned mesh = manual_sel / ALUS_PER_MESH;

    sel_words[0] = sel_words[1] = 0;
    sel_words[alu >> 4] = (uint16_t)(1u << (alu & 0xf));

    *cmd_w = (uint8_t)((0x10u << mesh) | 0x8u);
    *cmd_r = (uint8_t)((0x10u << mesh) | 0xcu);
}

/*
 * Reads the last node of the cache as the chip built it.
 * Returns 0 if the read succeeded, -1 otherwise.
 */
int dram_read_last_cache_node(uint8_t asic_en, uint16_t manual_sel,
                              uint32_t base_addr, uint64_t cache_size,
                              int thread_id, uint16_t node[NODE_WORDS])
{
    uint16_t sel[2], valid = 0;
    uint8_t cmd_w, cmd_r;
    uint32_t last_node = (uint32_t)(cache_size >> 7) + base_addr;
    unsigned tries;

    select_alu(manual_sel, sel, &cmd_w, &cmd_r);

    sl_multi_write(asic_en, cmd_w, REG_ALU_SEL, sel, 2, thread_id);
    sl_multi_write(asic_en, cmd_w, REG_NODE_ADDR, (uint16_t *)&last_node, 2, thread_id);
    sl_single_write(asic_en, cmd_w, REG_READ_START, 1, thread_id);
    usleep(1000);

    memset(node, 0, NODE_WORDS * sizeof(uint16_t));
    for (tries = 0; tries < 1000; tries++) {
        sl_single_read(asic_en, cmd_r, REG_READ_VALID, &valid, 1000000, thread_id);
        if (valid) {
            sl_multi_read(asic_en, cmd_r, REG_READ_DATA, node, NODE_WORDS,
                          1000000, thread_id);
            sl_single_write(asic_en, cmd_w, REG_READ_VALID, 1, thread_id);
            return 0;
        }
        usleep(1000);
    }
    return -1;
}

/*
 * Full check.
 *
 * cpu_cache is the cache computed on the processor side FOR THIS CHIP'S COIN,
 * or NULL if we do not have it. In the latter case the function returns 1:
 * neither compliant nor faulty — not checked. Up to the caller to decide,
 * knowing what it does not know.
 */
int dram_check_cache(uint8_t asic_en, uint16_t manual_sel, uint32_t base_addr,
                     int thread_id, const jas_coin_ctx *coin,
                     const uint16_t *cpu_cache)
{
    uint16_t node[NODE_WORDS];
    size_t last_word;

    if (dram_read_last_cache_node(asic_en, manual_sel, base_addr,
                                  coin->cache_size, thread_id, node) != 0) {
        jas_log("board %d: cache unreadable (alu %u)", thread_id, manual_sel);
        return -1;
    }

    if (!cpu_cache) {
        jas_log("board %d alu %u: cache NOT CHECKED (no reference for this coin)",
                thread_id, manual_sel);
        return 1;
    }

    /* The node read is the last one: compare with the last 32 words. */
    last_word = (size_t)(coin->cache_size / sizeof(uint16_t));
    for (unsigned k = 0; k < NODE_WORDS; k++) {
        if (node[NODE_WORDS - 1 - k] != cpu_cache[last_word - 1 - k]) {
            jas_log("board %d alu %u: cache mismatch at word %u",
                    thread_id, manual_sel, k);
            return -1;
        }
    }
    return 0;
}

/*
 * dram_check_dag @ 0x23e5c: is the DAG written into the chip the right one?
 *
 * Called by dag_calc right after the build. It does not read everything back:
 * ten probes are enough to tell a correct DAG from a corrupted one, because an
 * addressing or bank error shows on any item.
 *
 * The principle is the only honest check possible: recompute the item on the
 * processor, from the light cache, and compare it byte for byte with what the
 * ALU returns. No status register is consulted: the data itself is judged.
 *
 * Two DAG nodes per memory address: a block is 128 bytes, a node 64. The item
 * of index 2n takes the first half, the one of index 2n+1 the second. Hence
 * the two-step comparison on recv[0..15] then recv[16..31].
 *
 * bank_num comes from the crossbar scan, minus the banks of the condemned mesh
 * when mesh_bad is non-zero (mesh_bad is 1 to 4, the index is mesh_bad - 1).
 *
 * THE CACHE CHOICE, as the binary writes it:
 *
 *     if coin_type != 0 AND block_number > 17800000  ->  ETC cache
 *     otherwise                                      ->  ZIL cache
 *
 * The decompiled code spreads this condition over three branches and three
 * sets of counters, but the counters advance in step and two branches end at
 * the same cache: there are only the two cases above (checked by following
 * the registers).
 */

#define BLOCK_SWITCH_ETC   17800000ull
#define POLLS           5      /* addresses tested per pass */
#define PASSES             2

int dram_check_dag(int thread_id, uint8_t asic, uint8_t mesh_bad)
{
    unsigned rank = (unsigned)thread_id * 3u + asic;
    unsigned bank_num;
    unsigned step_addr, step_bytes, half;
    unsigned pass, probe;
    unsigned offset = 0;

    bank_num = jas_good_bank_num_cbsc[rank];
    if (mesh_bad)
        bank_num -= jas_good_bank_num_cbsc_mesh[rank * 4u + (mesh_bad - 1u)];
    bank_num &= 0xffffu;

    printf("board[%d] asic[%d] bank_num = %d\n", thread_id, asic, bank_num);
    if (bank_num == 0)
        return -1;

    /*
     * THE BLIND SPOT OF THIS CHECK, filled here: what follows compares the
     * CONTENT of the DAG, node by node, against the cache, but a hash ALSO
     * depends on the DAG's TOTAL SIZE, from which each access index is
     * derived. A right DAG indexed with a wrong size (e.g. a cache built
     * without its block_number) passes the content check while every nonce
     * is rejected. So: the cache must belong to the SAME EPOCH as the chip,
     * and its size must be the one expected for that epoch.
     */
    {
        const jas_coin_ctx *cc = jas_coin_of(thread_id, asic);
        ethash_light_t c = cc ? jas_cache_find(cc->block_number, &cc->seed)
                              : NULL;

        if (!cc || !c) {
            jas_log("board[%d] asic[%d]: no cache for epoch %llu,"
                    " cannot check dag", thread_id, asic,
                    cc ? (unsigned long long)(cc->block_number / 30000) : 0ull);
            return -1;
        }
        if (c->block_number / 30000 != cc->block_number / 30000) {
            jas_log("board[%d] asic[%d]: CACHE FROM ANOTHER EPOCH — asic %llu,"
                    " cache %llu. Indexing would be wrong.",
                    thread_id, asic,
                    (unsigned long long)(cc->block_number / 30000),
                    (unsigned long long)(c->block_number / 30000));
            return -1;
        }
        if (c->cache_size != ethash_get_cachesize(cc->block_number)) {
            jas_log("board[%d] asic[%d]: UNEXPECTED CACHE SIZE — %llu B instead"
                    " of %llu for epoch %llu",
                    thread_id, asic,
                    (unsigned long long)c->cache_size,
                    (unsigned long long)ethash_get_cachesize(cc->block_number),
                    (unsigned long long)(cc->block_number / 30000));
            return -1;
        }
        if (cc->dag_size != ethash_get_datasize(cc->block_number)) {
            jas_log("board[%d] asic[%d]: INCONSISTENT DAG SIZE — context"
                    " %llu B, epoch %llu expects %llu",
                    thread_id, asic,
                    (unsigned long long)cc->dag_size,
                    (unsigned long long)(cc->block_number / 30000),
                    (unsigned long long)ethash_get_datasize(cc->block_number));
            return -1;
        }
    }

    step_addr   = bank_num * 4u;      /* from one probe to the next */
    step_bytes = bank_num << 9;      /* what one row of banks covers */
    half       = bank_num >> 1;      /* from one pass to the next */

    for (pass = 0; pass < PASSES; pass++) {
        unsigned counter = 0;

        for (probe = 0; probe < POLLS; probe++) {
            ethash_light_t cache;
            node expected1, expected2;
            uint32_t received[32];
            uint64_t block, cc_size;
            unsigned n, index, address;

            memset(received, 0, sizeof(received));

            /*
             * THE CACHE OF THIS VERY CHIP, not a global choice on
             * *jas_coin_type, which would judge an ETC chip's DAG against
             * the ETHW cache ("calc dag error" on correct DAGs).
             */
            {
                const jas_coin_ctx *cc = jas_coin_of(thread_id, asic);

                cache = cc ? jas_cache_find(cc->block_number, &cc->seed)
                           : NULL;
                cc_size = cc ? cc->dag_size : *jas_fpga_dag_size;
            }

            /* WHERE THE PROBES LAND, and why "6". As in the original
             * (dram_check_dag, @0x23e5c), probe s reads at address
             * s*n*step_addr, i.e. in BYTES (address unit = 128 B = 2 nodes):
             *     s * n * step_addr * 128
             *       = s * (block/6) * (bank_num*4) * 128
             *   with block = dag_size/(bank_num*512).
             * bank_num CANCELS OUT entirely:
             *       = s * dag_size / 6.
             * The DAG is thus cut into SIX equal slices, and the five probes
             * (s = 0..4) land on 0, 1/6, 2/6, 3/6, 4/6, never beyond two
             * thirds. The last third is the MARGIN: index = address*2 (the
             * neighbouring node +1) and the bank remap can move an access
             * upwards; stopping at 4/6 guarantees that no probe, nor its even
             * node, nor its remapped image, runs past the end of the DAG. The
             * 2nd pass adds offset = bank_num/2 (~250 KB): a SECOND sample
             * next to each point, not new coverage.
             *
             * THE SIZE OF THIS VERY CHIP, not *jas_fpga_dag_size: with a chip
             * on another coin, probes spread over the other coin's DAG size
             * fall outside its memory and declare a correct DAG wrong. */
            block = cc_size / step_bytes;
            n = (unsigned)(block / 6u);

            index  = (n * counter + offset) * 2u;
            address = probe * n * step_addr + offset;

            ethash_calculate_dag_item(&expected1, index,      cache);
            ethash_calculate_dag_item(&expected2, index + 1u, cache);

            if (single_manual_read((uint8_t)(1u << asic), (uint16_t)probe,
                                   address, (uint16_t *)received, thread_id) < 0)
                return -1;

            /* Diagnostic: dump the first node read vs expected, for every chip,
             * to compare a chip that passes with one that fails. */
            if (pass == 0 && probe == 0)
                jas_log("board[%d] asic[%u] DIAG dag[0] read %08x %08x %08x"
                        " %08x | exp %08x %08x %08x %08x | cache_block"
                        " %llu size %llu",
                        thread_id, asic,
                        received[0], received[1], received[2], received[3],
                        expected1.words[0], expected1.words[1],
                        expected1.words[2], expected1.words[3],
                        (unsigned long long)(cache ? cache->block_number : 0ull),
                        (unsigned long long)cc_size);

            if (memcmp(received, expected1.words, sizeof(expected1.words)) != 0 ||
                memcmp(received + 16, expected2.words, sizeof(expected2.words)) != 0) {
                jas_log("board %d asic %u: wrong dag at address %u (item %u),"
                        " pass %u probe %u | read %08x %08x %08x %08x | exp"
                        " %08x %08x %08x %08x",
                        thread_id, asic, address, index, pass, probe,
                        received[0], received[1], received[2], received[3],
                        expected1.words[0], expected1.words[1],
                        expected1.words[2], expected1.words[3]);
                return -1;
            }

            usleep(100000);
            counter += step_addr;
        }
        offset += half;
    }
    return 0;
}
