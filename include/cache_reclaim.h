/*
 * Cache reclaim: the PROCESSOR writes the end of the DAG into the chip.
 *
 * From epoch 840 on, the ETHW DAG plus the cache the chip builds to compute it
 * exceed the chip's memory: the DAG climbs into the cache rows and overwrites
 * the cache as it builds there (17 to 40 % rejects).
 *
 * The cache is useful ONLY during the DAG build. Once the chip has stopped at
 * the edge of the cache, the processor, which holds the same cache
 * (jas_cache_find), computes the missing DAG nodes and writes them into
 * the rows the cache occupied. Mining then reads only the DAG.
 *
 * On by default (jas_cache_reclaim_active()); JASRE_CACHE_RECLAIM=0 restores
 * the stock behaviour (truncated DAG). Built on single_manual_write and
 * ethash_calculate_dag_item.
 */
#ifndef JAS_CACHE_RECLAIM_H
#define JAS_CACHE_RECLAIM_H

#include <stddef.h>     /* jas_coin.h uses size_t without including it */
#include <stdint.h>
#include "jas_coin.h"

/* Summary of a reclaim, for the log and the status page. */
struct reclaim_summary {
    uint32_t nodes;        /* nodes written by the processor */
    uint32_t front_node;    /* first node written (= start of the cache) */
    uint32_t total_node;    /* nodes of the full DAG (dag_size/64) */
    uint64_t us_compute;     /* CPU computation time, microseconds */
    uint64_t us_write;   /* time spent writing into the chip */
};

/* The switch. Defined in core/cache_reclaim.c (JASRE_CACHE_RECLAIM). */
int jas_cache_reclaim_active(void);

/*
 * First DAG node the processor must write: the node where the cache starts.
 * `valid` = valid banks of the chip, `cache_start_row` = starting row of
 * the cache (both already computed by prepare_chip).
 * Returns 0 if the DAG fits without overflowing (nothing to reclaim).
 */
uint32_t reclaim_front_node(unsigned valid, unsigned cache_start_row,
                            const jas_coin_ctx *coin);

/*
 * Computes and writes the nodes [front_node .. dag_size/64) into chip `asic`
 * of board `board`, with the processor's ethash cache. Returns 0 if everything
 * is written, -1 if the processor cache is missing. `summary` may be NULL.
 * Writes NOTHING and returns 0 if front_node lies beyond the end of the DAG.
 */
int reclaim_write_tail(int board, unsigned asic, const jas_coin_ctx *coin,
                        uint32_t front_node, struct reclaim_summary *summary);

/*
 * Same thing for a GROUP of chips of a board mining the same coin (same
 * block, same seed — hence the same DAG): each node is computed ONCE and
 * written at once to all the chips of the mask for which front[p] <= i.
 * `front[3]` = first node to write per chip; `mask` = chips concerned
 * (ready, healthy cache). Returns 0 if everything is written, -1 if the
 * processor cache is missing. Divides both the write AND the compute time by
 * the number of chips.
 */
int reclaim_write_tail_group(int board, const jas_coin_ctx *coin,
                               const uint32_t front[3], uint8_t mask,
                               struct reclaim_summary *summary);

/*
 * Checks `samples` nodes spread over the written zone, reading the chip
 * back and comparing with the processor's recomputation. Returns 0 if they all
 * match, the number of wrong nodes otherwise, -1 if the processor cache is
 * missing.
 */
unsigned reclaim_samples_check(void);
/* Last storable node of a chip whose DAG exceeds its memory (0 = no bound):
 * writing and checking the end stop there. */
void reclaim_set_end(int board, unsigned asic, uint32_t end);
int reclaim_check_tail(int board, unsigned asic, const jas_coin_ctx *coin,
                          uint32_t front_node, unsigned samples);

/* ------------------------------------------------------------------------
 * Function 3 — DAG check and repair.
 * Reads `samples` nodes spread over [start, end) back, compares them with
 * the processor's computation; if `repair`, rewrites each wrong node then
 * reads it back. Returns the number of wrong nodes (0 = all good), -1 if the
 * processor cache is missing. `repaired` (may be NULL) gets the number of
 * successful rewrites.
 * ---------------------------------------------------------------------- */
int dag_check_repair(int board, unsigned asic, const jas_coin_ctx *coin,
                         uint32_t start, uint32_t end, unsigned samples,
                         int repair, unsigned *repaired);

/* ------------------------------------------------------------------------
 * Function 2 — diagnosis of the cache INSIDE the chip.
 * Reads back `samples` nodes of the cache built by the chip and compares
 * them with the processor's ethash cache (the same one, by construction). For
 * each wrong node, reads it again through an ALU of ANOTHER mesh: if both ALUs
 * read the same wrong value, it is the DATA in memory (bank); if the second
 * reads it right, it is the first one's read PATH. Logs a verdict.
 * Returns the number of wrong nodes, -1 if the processor cache is missing.
 * `last_block_wrong` (may be NULL) gets the block address of the last wrong
 * node whose data is at fault — a candidate for bank_remove_hot.
 * ---------------------------------------------------------------------- */
int cache_diagnose(int board, unsigned asic, const jas_coin_ctx *coin,
                        unsigned samples, uint32_t *last_block_wrong);

/* ------------------------------------------------------------------------
 * Function 1 — hot removal of a bank, then cache + DAG rebuilt WITHOUT
 * replaying the full self-test (DFT + CBSC, ~7 min).
 * `block_wrong` = block address (128 B) of a node whose data is wrong.
 * The bank is derived through the same mapping as the bench's scrub
 * (approximate); it is marked bad, the remapping table is rebuilt, then the
 * chip's cache and DAG are rebuilt (dag_calc_asic).
 * Returns the number of the bank removed (0..4079), -1 on failure, -2 if it
 * already was.
 * ---------------------------------------------------------------------- */
int bank_remove_hot(int board, unsigned asic, const jas_coin_ctx *coin,
                           uint32_t block_wrong);

/* ------------------------------------------------------------------------
 * CACHE DIAGNOSIS ON FAILURE: reads only (on by default,
 * JASRE_CACHE_DIAG=0 turns it off).
 * ---------------------------------------------------------------------- */
int jas_cache_diag_active(void);

/*
 * Map of the defect: reads, through ALU 1 then through an ALU of another
 * mesh, the 4 blocks of node 0's bank row, the block of the neighbouring bank,
 * the block of the next row in the same bank, and blocks spread up to the
 * last one; compares with the processor cache and logs each probe (OK / WRONG
 * data / WRONG path / silent). Returns the number of wrong probes EXCLUDING
 * block 0 (0 = only node 0 is wrong, or nothing is), -1 if unusable.
 */
int cache_map(int board, unsigned asic, const jas_coin_ctx *coin);

/* The same, with the verdict details (for dag_calc's decision tree). */
struct map_verdict {
    unsigned read;            /* probes read */
    unsigned wrong;         /* wrong probes, block 0 included */
    unsigned wrong_outside0;   /* wrong probes outside block 0 */
    unsigned data;          /* wrong, read the same by both ALUs (memory) */
    unsigned path;          /* wrong through ALU 1 only (read path) */
    unsigned others;          /* 2nd ALU silent or inconsistent reads */
    uint32_t block_data;     /* last absolute block with a data defect */
};
int cache_map_v(int board, unsigned asic, const jas_coin_ctx *coin,
                          struct map_verdict *v);

/*
 * Processor round trip to node 0's block: writes nodes 0 and 1 of the
 * processor cache at their place in the chip, reads back through ALU 1.
 * Returns 1 if the read-back is identical (the chip's engine had lost the
 * write: the block is now REPAIRED), 0 if different (faulty memory at that
 * spot), -1 if unreadable or without a processor cache.
 */
int cache_cpu_write_attempt(int board, unsigned asic, const jas_coin_ctx *coin);

/* The first node of block `block` (absolute address) as the chip reads it
 * through ALU 1; returns 0 if the read succeeded. */
int cache_read_block(int board, unsigned asic, const jas_coin_ctx *coin,
                    uint32_t block, uint16_t node[0x20]);

/*
 * CACHE WRITTEN BY THE PROCESSOR (on by default, JASRE_CACHE_CPU=0 turns
 * it off): the fallback when the chip's engine still returns a wrong cache
 * after its rebuilds (right registers, healthy memory, a cache entirely
 * wrong and deterministic). Writes the n nodes of the processor cache at
 * their place in the chip (node j -> node address 2*base + j), then reads
 * back `samples` spread blocks and compares them. Returns 0 if all is
 * good, the number of wrong blocks otherwise, -1 if impossible (no processor
 * cache). ~6 min at epoch 840 (0.17 ms/node).
 */
int jas_cache_cpu_active(void);
int cache_write_by_cpu(int board, unsigned asic, const jas_coin_ctx *coin,
                         unsigned samples);

#endif /* JAS_CACHE_RECLAIM_H */
