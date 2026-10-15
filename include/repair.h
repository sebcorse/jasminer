#ifndef REPAIR_H
#define REPAIR_H
#include <stdint.h>

/*
 * Allocation of a bank's memory redundancies (spare rows and columns):
 * core/repair_calc.c.
 *
 * It replaces the calculate function of librepairarm.so, a Sunlune library
 * the miner no longer depends on. Same contract, output identical word for
 * word (comparison bench, 0 difference), with one deliberate
 * exception: in row-only mode the original re-emitted the column repairs of
 * the previous bank (uninitialized tables).
 *
 * Original call sites (dft_short_pattern_test_110):
 *   calculate(ltc_otp_row_data, ltc_row_count, gd_repair_info + b, 0x34, id, 1, 0)
 *   calculate(ltc_otp_col_data, 0x3870,        gd_repair_info + b, 0x34, id, 0, 0)
 *
 *   in         full mode: 2064 rows × 7 words (A 0-1, B 2-3, spares 4,
 *              C 5-6); row-only: list of the faulty rows, 0x800 = faulty
 *              spare row (number in the low 4 bits)
 *   n_in       number of entries (ignored in full mode)
 *   out        52 words, zeroed by the caller; only the entries taken
 *              (bit 31) are written
 *   out_stride ignored (the original ignored it too)
 *   dft_id     identity of the bank, | 0x1000 for unit 1
 *   row_only   1: rows only (MRR verdict "way 2"); 0: full
 *   reserved   0 (incremental mode of the original, never used)
 *
 * Reentrant, ~85 KB of stack.
 */
void jas_repair_compute(const uint32_t *in, uint32_t n_in, int *out,
                             int out_stride, uint16_t dft_id, int row_only,
                             int reserved);

#ifdef JAS_REPAIR_ORIGINAL
/*
 * Built with REPAIR=original (Makefile): the full survey hands the
 * computation to librepairarm.so, the vendor's library. By default — and in
 * the production images since 20261011-3 — our code does it. Our code stays
 * compiled in both cases, for evaluation and capture.
 */
void calculate(const uint32_t *in, uint32_t n_in, int *out, int out_stride,
               uint16_t dft_id, int row_only, int reserved);
#endif

/*
 * The same computation, with a chosen mode.
 *   FAITHFUL  the original (except its uninitialized tables);
 *   CORRECTED the best of three allocations according to the simulator — the
 *             original, the patched original, the redesigned allocation —
 *             the original wins ties: never worse than it according to the
 *             simulator's model;
 *   PATCHED, REDESIGNED  only one of the two, for trials.
 * In row-only mode, every mode except FAITHFUL masks the address and removes
 * duplicates. Modes other than FAITHFUL are not validated on silicon.
 */
#define JAS_REPAIR_FAITHFUL    0
#define JAS_REPAIR_CORRECTED  1
#define JAS_REPAIR_PATCHED  2
#define JAS_REPAIR_REDESIGNED  3
void jas_repair_compute_mode(const uint32_t *in, uint32_t n_in, int *out,
                                  uint16_t dft_id, int row_only, int mode);

/* What the simulator finds after applying 52 words to a map. */
struct jas_repair_eval {
    int rows;        /* logical rows with at least one faulty cell */
    int cells;       /* wrong logical cells */
    int conflicts;       /* column, row or spare taken twice */
    int malformed;    /* valid word undecodable or of another bank */
};
void jas_repair_evaluate(const uint32_t *in, const int *out, uint16_t dft_id,
                            struct jas_repair_eval *e);

struct jas_repair_summary {
    int choice;                          /* 0 original, 1 patched, 2 redesigned */
    struct jas_repair_eval faithful;  /* the original, per the simulator    */
    struct jas_repair_eval kept;  /* the selected allocation            */
};
/* Returns the choice; summary may be NULL. Nothing is evaluated in row-only mode. */
int jas_repair_compute_summary(const uint32_t *in, uint32_t n_in, int *out,
                                  uint16_t dft_id, int row_only, int mode,
                                  struct jas_repair_summary *summary);

#endif
