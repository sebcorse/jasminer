/*
 * Ethash: light cache, DAG items, digest.
 *
 * This is libethash (ethereum/ethash project, GPL v3 license), which the stock
 * firmware embeds as is: the vendor changed nothing in it. Each function was
 * checked against its decompiled form, the constants taken from the binary.
 *
 * The protocol constants:
 *     NODE_WORDS        16      a node is 64 bytes
 *     MIX_WORDS         32      the mix is 128
 *     MIX_NODES          2      i.e. two nodes
 *     ETHASH_ACCESSES   64      DAG accesses per digest
 *     DATASET_PARENTS  256      parents per DAG item
 *     CACHE_ROUNDS       3      cache mixing rounds
 *     EPOCH_LENGTH   30000      blocks per epoch
 *     FNV_PRIME 0x01000193
 *
 * Endianness: libethash calls fix_endian32/64 everywhere. On ARM as on x86
 * these are identities, and the firmware compiles the null variant (no byte
 * swap in the decompiled code), so they are not written here; the
 * dependency on little-endian is real, though.
 */

#include "jas_ethash.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>



#define NODE_WORDS        16
#define MIX_WORDS         32
#define MIX_NODES          2
#define ETHASH_ACCESSES   64
#define DATASET_PARENTS  256
#define CACHE_ROUNDS       3
#define EPOCH_LENGTH   30000u
#define FNV_PRIME 0x01000193u

static uint32_t fnv_hash(uint32_t x, uint32_t y)
{
    return x * FNV_PRIME ^ y;
}

/*
 * The seed of an epoch: the digest of the previous seed, starting from zero.
 *
 * REPLACES A STUB that returned thirty-two zeros. While it was in place, the
 * cache of epoch N was that of epoch 0 — a DAG consistent with itself, which
 * dram_check_dag would therefore have approved, but of which not one share
 * would have been accepted by the pool.
 */
void ethash_get_seedhash(ethash_h256_t *out, uint64_t block_number)
{
    uint64_t epochs = block_number / EPOCH_LENGTH;
    uint64_t k;

    memset(out, 0, sizeof(*out));
    for (k = 0; k < epochs; k++)
        sha3_256(out->b, 32, out->b, 32);
}

/*
 * ethash_get_blocknum @ 0x34e7a — find the epoch from the seed.
 *
 * This is NOT libethash: it is a vendor addition. The pool announces a seed,
 * the firmware must derive the block number from it to size the cache and the
 * DAG. There is no inverse, so it tries every epoch.
 *
 * Two surprising things that really are in the binary:
 *
 *   - the comparison covers only the FIRST EIGHT bytes of the seed, not the
 *     thirty-two. Over 2048 epochs a collision is unlikely, but the shortcut
 *     is there.
 *
 *   - when nothing matches, the function returns 61,410,000 — the last epoch
 *     tried — and not an error. The caller cannot tell "epoch 2047" from
 *     "unknown seed".
 *
 * The bound 0x3a98000 = 61,440,000 is exactly 2048 epochs.
 */
uint64_t ethash_get_blocknum(const ethash_h256_t *seed)
{
    /*
     * THE SEED DERIVES FROM THE PREVIOUS ONE: do not redo it from zero.
     * ethash_get_seedhash(block) replays k hashes from the null seed for
     * epoch k, so calling it at every turn is quadratic (k(k+1)/2: 350,000
     * hashes, about eleven seconds here, for epoch 836), and read_work
     * calls this FOR EVERY WORK RECEIVED. The seed of epoch k+1 is the
     * digest of that of epoch k: one application per turn is enough (836
     * hashes). Memoization does the rest: works of the same epoch follow
     * each other.
     */
    static ethash_h256_t last_seed;
    static uint64_t      last_block;
    static int           memo_valid;

    ethash_h256_t current;
    uint64_t block = 0;

    if (memo_valid && memcmp(last_seed.b, seed->b, 8) == 0)
        return last_block;

    memset(&current, 0, sizeof(current));       /* seed of epoch 0 */
    for (;;) {
        if (memcmp(current.b, seed->b, 8) == 0)
            break;
        if (block + EPOCH_LENGTH == 61440000ull)
            break;
        sha3_256(current.b, 32, current.b, 32); /* epoch k -> epoch k+1 */
        block += EPOCH_LENGTH;
    }

    last_seed = *seed;
    last_block    = block;
    memo_valid     = 1;
    return block;
}

/*
 * ethash_compute_cache_nodes @ 0x34aa4 — build the light cache.
 *
 * A chain of digests, then three RandMemoHash mixing rounds. The detail not
 * to miss is the index of the second operand: (n - 1 + i) % n, and not i - 1,
 * which makes the first item wrap around to the last.
 */
int ethash_compute_cache_nodes(node *nodes, uint64_t cache_size,
                               const ethash_h256_t *seed)
{
    uint32_t n = (uint32_t)(cache_size / sizeof(node));
    uint32_t round, i, w;

    if (cache_size % sizeof(node) != 0)
        return 0;

    sha3_512(nodes[0].bytes, 64, seed->b, 32);
    for (i = 1; i < n; i++)
        sha3_512(nodes[i].bytes, 64, nodes[i - 1].bytes, 64);

    for (round = 0; round < CACHE_ROUNDS; round++) {
        for (i = 0; i < n; i++) {
            uint32_t idx = nodes[i].words[0] % n;
            node data = nodes[(n - 1 + i) % n];

            for (w = 0; w < NODE_WORDS; w++)
                data.words[w] ^= nodes[idx].words[w];
            sha3_512(nodes[i].bytes, 64, data.bytes, 64);
        }
    }
    return 1;
}

/*
 * ethash_calculate_dag_item @ 0x34b88 — recompute a DAG item.
 *
 * This is the function that makes the light cache useful: any DAG item can be
 * rebuilt in 256 mixes from the cache, without the DAG. dram_check_dag uses
 * it to judge what the chip wrote.
 */
void ethash_calculate_dag_item(node *ret, uint32_t node_index, ethash_light_t light)
{
    uint32_t parents = (uint32_t)(light->cache_size / sizeof(node));
    const node *cache = (const node *)light->cache;
    uint32_t i, w;

    *ret = cache[node_index % parents];
    ret->words[0] ^= node_index;
    sha3_512(ret->bytes, 64, ret->bytes, 64);

    for (i = 0; i < DATASET_PARENTS; i++) {
        uint32_t index = fnv_hash(node_index ^ i, ret->words[i % NODE_WORDS]) % parents;
        const node *parent = &cache[index];

        for (w = 0; w < NODE_WORDS; w++)
            ret->words[w] = fnv_hash(ret->words[w], parent->words[w]);
    }
    sha3_512(ret->bytes, 64, ret->bytes, 64);
}

/*
 * ethash_hash @ 0x34c40 — the digest proper.
 *
 * full_nodes non-null: the DAG is in memory, we pick from it. Null: each item
 * is recomputed from the cache — this is the "light" mode, the only one the
 * firmware uses on the processor side, the whole DAG living in the ASICs.
 *
 * s_mix holds three nodes: the first carries the hashed header and nonce,
 * the next two the 128-byte mix.
 */
int ethash_hash(ethash_return_value_t *ret, const node *full_nodes,
                ethash_light_t light, uint64_t full_size,
                const ethash_h256_t *header_hash, uint64_t nonce)
{
    node s_mix[MIX_NODES + 1];
    node *mix = s_mix + 1;
    uint32_t pages = (uint32_t)(full_size / (sizeof(uint32_t) * MIX_WORDS));
    uint32_t i, n, w;

    if (full_size % MIX_WORDS != 0)
        return 0;

    memcpy(s_mix[0].bytes, header_hash->b, 32);
    memcpy(s_mix[0].bytes + 32, &nonce, 8);
    sha3_512(s_mix[0].bytes, 64, s_mix[0].bytes, 40);

    for (w = 0; w < MIX_WORDS; w++)
        mix->words[w] = s_mix[0].words[w % NODE_WORDS];

    for (i = 0; i < ETHASH_ACCESSES; i++) {
        uint32_t index = fnv_hash(s_mix[0].words[0] ^ i,
                                   mix->words[i % MIX_WORDS]) % pages;

        for (n = 0; n < MIX_NODES; n++) {
            node tmp;
            const node *dag;

            if (full_nodes) {
                dag = &full_nodes[MIX_NODES * index + n];
            } else {
                ethash_calculate_dag_item(&tmp, index * MIX_NODES + n, light);
                dag = &tmp;
            }
            for (w = 0; w < NODE_WORDS; w++)
                mix[n].words[w] = fnv_hash(mix[n].words[w], dag->words[w]);
        }
    }

    /* Compress the 32 words of the mix into 8, four at a time. */
    for (w = 0; w < MIX_WORDS; w += 4) {
        uint32_t r = mix->words[w];

        r = fnv_hash(r, mix->words[w + 1]);
        r = fnv_hash(r, mix->words[w + 2]);
        r = fnv_hash(r, mix->words[w + 3]);
        mix->words[w / 4] = r;
    }

    memcpy(ret->mix_hash.b, mix->words, 32);
    sha3_256(ret->result.b, 32, s_mix[0].bytes, 64 + 32);
    return 1;
}

/*
 * ethash_light_new_internal @ 0x34ed2 and ethash_light_new @ 0x34f1c.
 *
 * DELIBERATE DEPARTURE, a single one, and it fixes a libethash defect: the
 * original writes `ret->block_number = block_number` without checking that
 * ethash_light_new_internal returned something other than NULL. On a machine
 * with a 128 MiB RAM disk where the cache already weighs sixteen megabytes,
 * the allocation can really fail, and the dereference would be fatal. We keep
 * the guard.
 */
ethash_light_t ethash_light_new_internal(uint64_t cache_size, const ethash_h256_t *seed)
{
    ethash_light_t light = calloc(sizeof(*light), 1);

    if (!light)
        return NULL;
    light->cache = malloc((size_t)cache_size);
    if (!light->cache) {
        free(light);
        return NULL;
    }
    if (!ethash_compute_cache_nodes(light->cache, cache_size, seed)) {
        free(light->cache);
        free(light);
        return NULL;
    }
    light->cache_size = cache_size;
    return light;
}

ethash_light_t ethash_light_new(uint64_t block_number)
{
    ethash_h256_t seed;
    ethash_light_t light;

    ethash_get_seedhash(&seed, block_number);
    light = ethash_light_new_internal(ethash_get_cachesize(block_number), &seed);
    if (light)
        light->block_number = block_number;
    return light;
}

void ethash_light_delete(ethash_light_t light)
{
    if (!light)
        return;
    if (light->cache)
        free(light->cache);
    free(light);
}

/*
 * ethash_light_compute_internal @ 0x34f5a and ethash_light_compute @ 0x34fae.
 * Two wrappers: the second derives the DAG size from the block number.
 */
void ethash_light_compute_internal(ethash_return_value_t *ret, ethash_light_t light,
                                   uint64_t full_size,
                                   const ethash_h256_t *header_hash, uint64_t nonce)
{
    ret->success = 1;
    if (!ethash_hash(ret, NULL, light, full_size, header_hash, nonce))
        ret->success = 0;
}

void ethash_light_compute(ethash_return_value_t *ret, ethash_light_t light,
                          const ethash_h256_t *header_hash, uint64_t nonce)
{
    ethash_light_compute_internal(ret, light,
                                  ethash_get_datasize(light->block_number),
                                  header_hash, nonce);
}
