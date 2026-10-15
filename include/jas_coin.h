/*
 * Coin context, PER CHIP.
 *
 * This is the deliberate divergence from the stock firmware, and the reason
 * for this whole project. Where it keeps a single set of parameters for the
 * whole board — fpga_data.cache_size, fpga_data.seed, fpga_data.dag_size —
 * each ASIC carries its own here.
 *
 * Everything follows from that: the cache size commands its placement in
 * memory, the seed commands its content, the DAG size commands the time given
 * to its construction. Tying these three values to a chip is exactly what was
 * missing — interposing on the register writes could not do it, since it let
 * the firmware compute the placement and the budget for its own coin.
 */
#ifndef JAS_COIN_H
#define JAS_COIN_H

#include "jas_types.h"
struct gooddata_detail;

typedef struct jas_coin_ctx {
    uint64_t      block_number;   /* from which both sizes derive */
    uint64_t      cache_size;
    uint64_t      dag_size;
    ethash_h256_t seed;
    ethash_h256_t header;         /* renewed at each job */
    ethash_h256_t target;
    /*
     * The identifier of the work this chip REALLY carries. Without it, a
     * share cannot be dated: not every work reaches the chips before the
     * next one arrives, so a chip may be computing on a work two
     * announcements old.
     */
    char          work_id[64];
} jas_coin_ctx;

/* Fills in the sizes and the seed from the block number. */
void jas_coin_from_block(jas_coin_ctx *c, uint64_t block_number);
void jas_coin_from_block_seed(jas_coin_ctx *c, uint64_t block_number,
                                const ethash_h256_t *seed);

/*
 * The work the pool layer drops off for the mining loop. It is the only
 * point of contact between the driver — written — and the network — to be
 * written.
 */
typedef struct jas_work {
    uint64_t      block_number;
    ethash_h256_t seed;
    ethash_h256_t header;
    ethash_h256_t target;
    char          work_id[64];
} jas_work;

extern jas_work jas_work_g;
int  jas_work_current(jas_work *out);          /* returns 0 if none */
void jas_submit(const struct gooddata_detail *d);

/* The context in force for a chip. */
jas_coin_ctx *jas_coin_of(int board, unsigned asic);

#endif

void *jas_pool_thread(void *nothing);

int  jas_work_of_chip(unsigned asic, jas_work *out);
int  jas_work_of_group(int group, jas_work *out);
int  jas_pool_of_chip(unsigned asic);
void jas_pool_assign(const char *board);
int  jas_group_etchash(void);                    /* the ETC group, -1 if none */
void jas_pool_switch_chip(unsigned asic, int group);

void *jas_pool_n(int idx);   /* opaque: a jas_pool, defined in pool.c */
int   jas_pool_count(void);      /* 4: two groups of two */
const char *jas_pool_host(int idx);   /* host of the slot, NULL if unknown */
int   jas_pool_configure(int idx);
int   jas_chip_group(unsigned asic);   /* the group feeding this chip */
int   jas_group_active(int group);     /* which rank of the group is serving */
void  read_public_assignment_config(void);

/*
 * A pool's counters, for the web page.
 *
 * The original keeps pool_accepts and pool_rejects per pool
 * (parse_submit_result, 0x3c555) and serves them through CGI. Returns 0 if the
 * index does not designate a configured pool; writes only the non-null
 * pointers.
 */
int jas_pool_counters(int idx, unsigned *accepts, unsigned *rejects,
                       unsigned *works);
int jas_pool_describe(int idx, char *url, size_t nurl,
                     char *user, size_t nuser, int *live);

/* The miner's API, port 4028 — see core/web.c. */
void *jas_web_thread(void *arg);
void  jas_web_hashrate_publish(int board, unsigned chip, double mh, unsigned alus);
void  jas_web_dag_publish(unsigned percent);
void  jas_web_rails_publish(int board, unsigned chip,
                            unsigned r0, unsigned r1, unsigned r2);
void  jas_web_sample(double rt, double avg, double refusal);
double jas_hashrate_total_mh(void);
int   jas_coin_type_current(void);

/* The serial numbers, decrypted from /media/configs — see core/serial.c. */
const char *jas_sn_machine(void);
const char *jas_sn_ctrlboard(void);
const char *jas_sn_pw(void);
const char *jas_sn_hashboard(unsigned k);

/* Seed separating the nonce ranges, derived from the serial number.
 * Returns 0 if the machine has no serial. See core/serial.c. */
uint64_t jas_identity_nonce(void);

