/*
 * Ethash checked against an independent implementation.
 *
 * The expected values come from tools/ref_ethash.py, written from the
 * Ethash specification on top of Cryptodome's Keccak: no line of our C
 * takes part in producing them.
 *
 * Deliberately tiny sizes (1 KB of cache, 8 KB of DAG) so the check runs in
 * a fraction of a second; they weaken nothing: every modulo, every mix and
 * the three mixing rounds are exercised.
 *
 *   gcc -O2 -Iinclude -o /tmp/te tools/test_ethash.c core/ethash.c core/keccak.c \
 *       core/ethash_sizes.c && /tmp/te
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jas_ethash.h"
#include "vecteurs_ethash.h"

static int good, total;

static void checked(const char *what, const void *obtained, const void *expected, size_t n)
{
    const uint8_t *a = obtained, *b = expected;
    size_t i;

    total++;
    if (memcmp(a, b, n) == 0) {
        printf("  ok    %s\n", what);
        good++;
        return;
    }
    printf("  WRONG %s\n        got      ", what);
    for (i = 0; i < n; i++) printf("%02x", a[i]);
    printf("\n        expected ");
    for (i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

int main(void)
{
    ethash_h256_t seed, seedhash;
    ethash_light light;
    node *cache;
    node item;
    ethash_return_value_t r;
    unsigned n = VEC_CACHE_SIZE / 64;

    memcpy(seed.b, VEC_SEED, 32);

    cache = malloc(VEC_CACHE_SIZE);
    if (!ethash_compute_cache_nodes(cache, VEC_CACHE_SIZE, &seed)) {
        printf("  ethash_compute_cache_nodes refused the size\n");
        return 1;
    }
    checked("cache, first node", cache[0].bytes, VEC_CACHE0, 64);
    checked("cache, last node", cache[n - 1].bytes, VEC_CACHEN, 64);

    light.cache = cache;
    light.cache_size = VEC_CACHE_SIZE;
    light.block_number = 0;

    ethash_calculate_dag_item(&item, 0, &light);
    checked("DAG item 0", item.bytes, VEC_DAG0, 64);
    ethash_calculate_dag_item(&item, 1, &light);
    checked("DAG item 1", item.bytes, VEC_DAG1, 64);
    ethash_calculate_dag_item(&item, 17, &light);
    checked("DAG item 17", item.bytes, VEC_DAG17, 64);
    /* Index far beyond the cache's node count: exercises the modulo. */
    ethash_calculate_dag_item(&item, 4095, &light);
    checked("DAG item 4095", item.bytes, VEC_DAG4095, 64);

    {
        ethash_h256_t header;
        memcpy(header.b, VEC_HEADER, 32);
        memset(&r, 0, sizeof(r));
        if (!ethash_hash(&r, NULL, &light, VEC_DAG_SIZE, &header, VEC_NONCE)) {
            printf("  ethash_hash refused the size\n");
            return 1;
        }
        checked("ethash_hash, mix", r.mix_hash.b, VEC_MIXHASH, 32);
        checked("ethash_hash, result", r.result.b, VEC_RESULT, 32);
    }

    ethash_get_seedhash(&seedhash, 0);
    checked("seed, epoch 0", seedhash.b, VEC_SEED_0, 32);
    ethash_get_seedhash(&seedhash, 30000);
    checked("seed, epoch 1", seedhash.b, VEC_SEED_30000, 32);
    ethash_get_seedhash(&seedhash, 60000);
    checked("seed, epoch 2", seedhash.b, VEC_SEED_60000, 32);
    ethash_get_seedhash(&seedhash, 3000000);
    checked("seed, epoch 100", seedhash.b, VEC_SEED_3000000, 32);

    /* ethash_get_blocknum must find the epoch back from its seed. */
    total++;
    ethash_get_seedhash(&seedhash, 3000000);
    if (ethash_get_blocknum(&seedhash) == 3000000) {
        printf("  ok    ethash_get_blocknum retrouve 3000000\n");
        good++;
    } else {
        printf("  WRONG ethash_get_blocknum returns %llu instead of 3000000\n",
               (unsigned long long)ethash_get_blocknum(&seedhash));
    }

    free(cache);
    printf("\n  %d/%d\n", good, total);
    return good == total ? 0 : 1;
}
