/*
 * Coin contexts, one per ASIC.
 *
 * By default all chips share the same coin and the behaviour is the stock
 * firmware's. Assigning a different coin to a chip is enough to make it build
 * its own cache and its own DAG.
 */
#include "jas_hal.h"
#include "jas_coin.h"
#include <stdio.h>
#include <string.h>
#include "jas_ethash.h"

#define MAX_BOARDS 3
#define MAX_ASICS  3

static jas_coin_ctx coins[MAX_BOARDS][MAX_ASICS];

/*
 * THE SEED COMES FROM THE POOL, THE SIZE COMES FROM THE EPOCH: two different
 * things, and mixing them up breaks etchash.
 *
 * In ethash the two coincide: the seed of epoch N is keccak^N(0), and
 * recomputing it from the block number gives exactly the one the pool sent.
 * In etchash (ETC, ECIP-1099) they diverge: the pool serves the seed of index
 * 2N while the DAG must have the size of epoch N. Recomputing the seed from
 * the block number would give keccak^N(0): a DAG consistent with itself,
 * accepted by our own check, and rejected by the pool.
 */
void jas_coin_from_block_seed(jas_coin_ctx *c, uint64_t block_number,
                                const ethash_h256_t *seed)
{
    c->block_number = block_number;
    c->cache_size   = ethash_get_cachesize(block_number);
    c->dag_size     = ethash_get_datasize(block_number);
    c->seed         = *seed;
}

/* Legacy variant: the seed is derived from the block. Only valid for ethash. */
void jas_coin_from_block(jas_coin_ctx *c, uint64_t block_number)
{
    ethash_h256_t seed;

    ethash_get_seedhash(&seed, block_number);
    jas_coin_from_block_seed(c, block_number, &seed);
}

jas_coin_ctx *jas_coin_of(int board, unsigned asic)
{
    if (board < 0 || board >= MAX_BOARDS || asic >= MAX_ASICS)
        return &coins[0][0];
    return &coins[board][asic];
}

/*
 * get_dev_id @ 0x27d8c: the identifier the original uses as fallback
 * extranonce, the LAST TWO bytes of eth0's MAC address:
 *
 *     res = (mac[4] << 8) | mac[5]
 *
 * seed_update_operation uses it when the pool provides no extranonce; hence
 * the 48-bit shift in that case (sixteen bits of extranonce, forty-eight of
 * search space). Sixteen bits only: two miners whose MACs end the same would
 * sweep the same nonce space.
 *
 * The stock firmware goes through popen("cat ... | cut -d: -f 5,6"); the file
 * is read directly here.
 */
uint64_t get_dev_id(void)
{
    FILE *f;
    char line[64];
    unsigned o[6];
    uint64_t res = 0;

    memset(line, 0, sizeof(line));
    f = fopen("/sys/class/net/eth0/address", "r");
    if (!f) {
        jas_log("get_dev_id: MAC address unreadable, null identifier");
        return 0;
    }
    if (fgets(line, sizeof(line), f) &&
        sscanf(line, "%x:%x:%x:%x:%x:%x",
               &o[0], &o[1], &o[2], &o[3], &o[4], &o[5]) == 6)
        res = ((uint64_t)(o[4] & 0xff) << 8) | (o[5] & 0xff);
    fclose(f);

    printf("mac[0] = %2llx, mac[1] = %2llx\n",
           (unsigned long long)(o[4] & 0xff), (unsigned long long)(o[5] & 0xff));
    printf("buf = %s, res = %4llx\n", line, (unsigned long long)res);
    return res;
}

/*
 * The coin type the API announces. The stock API returns "coin_type: 0" on
 * ETHW; without the vendor's table, 0 for ethash and 1 for etchash, enough
 * for the page to stay consistent (to be checked against update_work_mode).
 */
int jas_coin_type_current(void)
{
    const jas_coin_ctx *c = jas_coin_of(0, 0);

    return (c && c->block_number >= 11700000ULL) ? 1 : 0;
}
