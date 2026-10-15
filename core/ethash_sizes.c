/*
 * Cache and DAG sizes: reference implementation.
 *
 * Ethash keeps the largest size below a bound whose number of pages is
 * prime. These sizes are what the chips' registers are programmed with (the
 * chips compute the cache themselves from the seed). Checked against the
 * machine: ETC epoch 418 gives 0x110ffea80 in register 0x08 and 0x443fec0 in
 * register 0x04, exactly what the firmware programs.
 */
#include "jas_ethash.h"

#define EPOCH_LENGTH        30000ull
#define DATASET_INIT        (1ull << 30)
#define DATASET_GROWTH      (1ull << 23)
#define CACHE_INIT          (1ull << 24)
#define CACHE_GROWTH        (1ull << 17)
#define MIX_BYTES           128ull
#define HASH_BYTES          64ull

static int is_prime(uint64_t n)
{
    if (n < 2)      return 0;
    if (n % 2 == 0) return n == 2;
    for (uint64_t d = 3; d * d <= n; d += 2)
        if (n % d == 0) return 0;
    return 1;
}

uint64_t ethash_get_datasize(uint64_t block_number)
{
    uint64_t sz = DATASET_INIT + DATASET_GROWTH * (block_number / EPOCH_LENGTH)
                - MIX_BYTES;
    while (!is_prime(sz / MIX_BYTES))
        sz -= 2 * MIX_BYTES;
    return sz;
}

uint64_t ethash_get_cachesize(uint64_t block_number)
{
    uint64_t sz = CACHE_INIT + CACHE_GROWTH * (block_number / EPOCH_LENGTH)
                - HASH_BYTES;
    while (!is_prime(sz / HASH_BYTES))
        sz -= 2 * HASH_BYTES;
    return sz;
}
