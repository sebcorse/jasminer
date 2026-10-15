/*
 * ethash_get_headerhash: a stub, on purpose: the header comes from the work
 * supplied by the pool, not from a computation. (ethash_get_seedhash and
 * ethash_calculate_dag_item live in core/ethash.c.)
 */
#include "jas_ethash.h"
#include <string.h>

void ethash_get_headerhash(ethash_h256_t *out, uint64_t block_number)
{
    (void)block_number;
    memset(out, 0, sizeof(*out));
}
