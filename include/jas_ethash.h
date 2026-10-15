/* Ethash interface. In bridge mode these functions are the firmware's,
 * called by address; in standalone mode, ours. */
#ifndef JAS_ETHASH_H
#define JAS_ETHASH_H
#include "jas_types.h"
#include <stddef.h>

/*
 * A node: 64 bytes, seen as bytes, 32-bit or 64-bit words as needed.
 * It really is a UNION and not a structure — the ethash code alternates the
 * three views over the same storage at every step.
 */
typedef union node {
    uint8_t  bytes[64];
    uint32_t words[16];
    uint64_t double_words[8];
} node;

/* The light cache, from which any DAG item can be recomputed without holding
 * the whole DAG in memory. Layout taken from the firmware's DWARF. */
typedef struct ethash_light {
    void    *cache;
    uint64_t cache_size;
    uint64_t block_number;
} ethash_light;
typedef struct ethash_light *ethash_light_t;

typedef struct ethash_return_value {
    ethash_h256_t result;
    ethash_h256_t mix_hash;
    int           success;
} ethash_return_value_t;

/* The two caches the firmware keeps open, one per coin. Declared as
 * pointers: in bridge mode they are ITS objects, at its fixed addresses. */
extern ethash_light_t *jas_cache_zil;   /* cpu_cache_zil  0x38c19c */
extern ethash_light_t *jas_cache_etc;   /* cpu_cache_etc  0x38c198 */
extern uint8_t        *jas_coin_type;   /* coin_type      0x387cc0 */

uint64_t ethash_get_cachesize(uint64_t block_number);
uint64_t ethash_get_datasize(uint64_t block_number);
void     ethash_get_headerhash(ethash_h256_t *out, uint64_t block_number);
void     ethash_get_seedhash(ethash_h256_t *out, uint64_t block_number);

uint64_t ethash_get_blocknum(const ethash_h256_t *seed);

int  ethash_compute_cache_nodes(node *nodes, uint64_t cache_size,
                                const ethash_h256_t *seed);
void ethash_calculate_dag_item(node *ret, uint32_t node_index, ethash_light_t light);
int  ethash_hash(ethash_return_value_t *ret, const node *full_nodes,
                 ethash_light_t light, uint64_t full_size,
                 const ethash_h256_t *header_hash, uint64_t nonce);

ethash_light_t ethash_light_new_internal(uint64_t cache_size, const ethash_h256_t *seed);
ethash_light_t ethash_light_new(uint64_t block_number);
void           ethash_light_delete(ethash_light_t light);
void ethash_light_compute_internal(ethash_return_value_t *ret, ethash_light_t light,
                                   uint64_t full_size,
                                   const ethash_h256_t *header_hash, uint64_t nonce);
void ethash_light_compute(ethash_return_value_t *ret, ethash_light_t light,
                          const ethash_h256_t *header_hash, uint64_t nonce);

/* The two threads that build the processor cache (core/cpu_cache.c). */
uint64_t get_dev_id(void);

int ethash_nonce_check(const gooddata_detail *detail,
                       const uint8_t *headhash, const uint8_t *target);
int isValueLessThenTarget_256bits(const uint8_t *value, const uint8_t *target);

/* Original Keccak (padding 0x01), not SHA-3. See core/keccak.c. */
int  sha3_256(uint8_t *out, size_t outlen, const uint8_t *in, size_t inlen);
int  sha3_512(uint8_t *out, size_t outlen, const uint8_t *in, size_t inlen);

#endif

int ethash_nonce_stale(const gooddata_detail *detail,
                        const uint8_t (*headers)[32], unsigned how_many);

ethash_light_t jas_cache_find(uint64_t block, const ethash_h256_t *seed);
void jas_cache_save(uint64_t block, const ethash_h256_t *seed,
                           ethash_light_t light);

void jas_cache_request(uint64_t block, const ethash_h256_t *seed);
