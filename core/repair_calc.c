/*
 * repair_calc.c — allocation of a bank's memory redundancies.
 *
 * Replaces librepairarm.so, the Sunlune library our miner used to depend on.
 * A FUNCTIONAL rewrite: same input, same output, word for word, checked
 * against the original with a comparison bench.
 *
 * WHAT THE COMPUTATION DOES. For a 2 MB bank, the DFT reads the map of the
 * faulty cells. Each bank has spare parts: 16 spare rows, and per row section
 * 3 + 3 + 2 spare columns. One must decide which rows and which columns to
 * replace. The result — 52 words — is burnt by otp_program into the
 * redundancy registers.
 *
 * THE GEOMETRY. 2048 rows in six sections (344, 336, 344, 344, 336, 344),
 * plus 16 spare rows numbered 2048 to 2063 in the map. Spares 0 to 7 are
 * physically in section 1, spares 8 to 15 in section 4. Each row carries
 * three regions of 64 columns:
 *   A  (input words 0-1)       3 spare columns per section
 *   B  (words 2-3)             3 spare columns per section
 *   C  (words 5-6)             2 spare columns per section
 * Word 4 carries the bits of the spare columns: A in 0-2, B in 3-5, C in 6-7.
 * Spares 0 and 1 of B are COUPLED with those of C: replacing column X of B by
 * spare 0 or 1 also replaces column X of C. Spare 2 of B is alone. C thus has
 * no output of its own: its repairs appear in B's words.
 *
 * THE ALGORITHM (classic greedy BIRA):
 *   1. count the defects per column and per section, the section's spare rows
 *      included, and per row, three regions added up;
 *   2. sort the spares by their own defects, cleanest first (columns of A and
 *      rows only: B and C stay in order);
 *   3. MANDATORY repairs: a column with more than 16 defects in its section
 *      can only be saved by a column (there are only 16 spare rows); a row
 *      with more than 3 defects can only be saved by a row;
 *   4. GREEDY completion: while spares remain, replace the most loaded column
 *      then row, if the spare is cleaner than it;
 *   5. after each replacement, erase the covered defects from the map and
 *      keep the counters up to date.
 *
 * FAITHFULNESS. This file reproduces the original down to its clumsiness
 * (< or <= thresholds depending on the function, bookkeeping of the B/C
 * coupling, a spare read at the wrong rank in one case): as long as a change
 * has not been validated on silicon, a different output is a different repair
 * configuration. Each deliberate departure is marked "ORIGINAL:", and the
 * corrected mode's fixes "FIXED:". ONE SINGLE DIFFERENCE is assumed: in
 * row-only mode the original reads its column tables without having
 * initialized them (leftovers of the previous call on the stack); here they
 * are empty, so no ghost column repair is emitted.
 *
 * Reentrant: no global state. About 85 KB of stack per call, like the
 * original (the DFT threads have 1 MB).
 */
#include <stdint.h>
#include <string.h>

#include "repair.h"

/*
 * Coverage counters, for the bench only (-DJAS_REPAIR_COVERAGE):
 * they prove the comparison with the original went through every branch,
 * including the rare cases of the coupling bookkeeping.
 */
#ifdef JAS_REPAIR_COVERAGE
unsigned long jas_repair_coverage[32];
#define COVERED(n) (jas_repair_coverage[n]++)
#else
#define COVERED(n) ((void)0)
#endif

#define ROWS         2048            /* normal rows */
#define SPARE_R       16              /* spare rows */
#define ROWS_TOT     (ROWS + SPARE_R)
#define SECTIONS        6
#define COLUMNS        64              /* normal columns per region */
#define WORDS_IN     7               /* 32-bit words per row read */
#define WORDS_OUT     52
#define COLUMN_THRESHOLD   SPARE_R       /* beyond that, only a column saves */
#define ROW_THRESHOLD    3               /* beyond that, only a row saves */

enum { REG_A, REG_B, REG_C, REGIONS };

static const int spare_per_section[REGIONS] = { 3, 3, 2 };
static const int section_start[SECTIONS]  = { 0, 344, 680, 1024, 1368, 1704 };
static const int section_size[SECTIONS] = { 344, 336, 344, 344, 336, 344 };

/* A column region: its defect map and its counters. */
struct region {
    /*
     * For each row (normal then spare, in PHYSICAL order): words 0 and 1 =
     * columns 0-63, word 2 = spare columns (bit = physical number of the
     * spare).
     */
    uint32_t board[ROWS_TOT][3];
    /*
     * Defects per section: [s][0..63] normal columns, [s][64 + r] spare of
     * RANK r. For A the spares are sorted, rank r then designates the physical
     * spare rank_spare[s][r]; for B and C, rank = number.
     */
    int count[SECTIONS][COLUMNS + 3];
    int rank_spare[SECTIONS][3];
    int free[SECTIONS];               /* spares not yet consumed */
    int nb_spare;                     /* 3, 3 or 2 */
};

/* A consumed spare: taken (1) to replace "target", or discarded (0). */
struct usage {
    int taken;
    int target;                          /* column 0-63, or row 0-2047 */
    int number;                         /* physical spare */
};

struct compute {
    int corrected;                        /* JAS_REPAIR_CORRECTED           */
    struct region reg[REGIONS];
    /* Defects per row; [2048 + r] = spare row of RANK r. */
    int count_row[ROWS_TOT];
    int rank_row[SPARE_R];         /* rank -> spare row number */
    int free_rows;
    struct usage col_a[SECTIONS][3];   /* rank -> usage                      */
    struct usage col_b[SECTIONS][3];   /* B, and C by coupling (ranks 0, 1) */
    struct usage row[SPARE_R];    /* rank -> usage                      */
};

/* Section of a normal row. */
static int section_of(int row)
{
    int s;

    for (s = SECTIONS - 1; s > 0; s--)
        if (row >= section_start[s])
            break;
    return s;
}

/* Is spare row no. p physically in section s? */
static int spare_in_section(int p, int s)
{
    return (s == 1 && p >= 0 && p < 8) || (s == 4 && p >= 8 && p < 16);
}

static int bits_set(uint32_t v)
{
    return __builtin_popcount(v);
}

/* ------------------------------------------------------------------------
 * Preparation
 * ------------------------------------------------------------------------ */

/* Splits the map that was read into three regions. */
static void split(struct compute *k, const uint32_t *in)
{
    int r;

    for (r = 0; r < ROWS_TOT; r++) {
        const uint32_t *m = in + (size_t)r * WORDS_IN;

        k->reg[REG_A].board[r][0] = m[0];
        k->reg[REG_A].board[r][1] = m[1];
        k->reg[REG_A].board[r][2] = m[4] & 7u;
        k->reg[REG_B].board[r][0] = m[2];
        k->reg[REG_B].board[r][1] = m[3];
        k->reg[REG_B].board[r][2] = (m[4] >> 3) & 7u;
        k->reg[REG_C].board[r][0] = m[5];
        k->reg[REG_C].board[r][1] = m[6];
        k->reg[REG_C].board[r][2] = (m[4] >> 6) & 3u;
    }
}

static void initialize(struct compute *k)
{
    int g, s, r;

    memset(k->col_a, 0, sizeof(k->col_a));
    memset(k->col_b, 0, sizeof(k->col_b));
    memset(k->row, 0, sizeof(k->row));
    memset(k->count_row, 0, sizeof(k->count_row));
    for (g = 0; g < REGIONS; g++) {
        struct region *R = &k->reg[g];

        R->nb_spare = spare_per_section[g];
        memset(R->count, 0, sizeof(R->count));
        for (s = 0; s < SECTIONS; s++) {
            R->free[s] = R->nb_spare;
            for (r = 0; r < 3; r++)
                R->rank_spare[s][r] = r;
        }
    }
    for (r = 0; r < SPARE_R; r++)
        k->rank_row[r] = r;
    k->free_rows = SPARE_R;
}

/*
 * Defects per column and per section. The spare rows count in the section
 * that hosts them: a replaced column is replaced on them too.
 * At this stage no spare is sorted: index 64 + p is spare p.
 */
static void count_columns(struct region *R)
{
    int s, w, b, r, p;

    for (s = 0; s < SECTIONS; s++) {
        int end = section_start[s] + section_size[s];

        for (w = 0; w < 3; w++) {
            int nb = w < 2 ? 32 : R->nb_spare;

            for (b = 0; b < nb; b++) {
                int n = 0;

                for (r = section_start[s]; r < end; r++)
                    n += (int)(R->board[r][w] >> b & 1u);
                for (p = 0; p < SPARE_R; p++)
                    if (spare_in_section(p, s))
                        n += (int)(R->board[ROWS + p][w] >> b & 1u);
                R->count[s][32 * w + b] += n;
            }
        }
    }
}

/*
 * Defects per row, three regions added up. ORIGINAL: the bits of the spare
 * columns count, even for a spare that will never be used.
 */
static void count_rows(struct compute *k)
{
    int r, g;

    for (r = 0; r < ROWS_TOT; r++)
        for (g = 0; g < REGIONS; g++)
            k->count_row[r] += bits_set(k->reg[g].board[r][0]) +
                                   bits_set(k->reg[g].board[r][1]) +
                                   bits_set(k->reg[g].board[r][2]);
}

/*
 * Sorts the still free spares by increasing defects; they occupy the end of
 * the list (the consumed ones are in front and no longer move). STABLE sort:
 * on ties, the current order is kept — this is what the original's bubble
 * sort does, and the final order depends on it.
 */
static void rank(int *count, int *rank, int free)
{
    int i, j;

    for (i = 1; i < free; i++) {
        int c = count[i], g = rank[i];

        for (j = i; j > 0 && count[j - 1] > c; j--) {
            count[j] = count[j - 1];
            rank[j] = rank[j - 1];
        }
        count[j] = c;
        rank[j] = g;
    }
}

static void rank_spare_a(struct region *R)
{
    int s, l;

    for (s = 0; s < SECTIONS; s++) {
        l = R->free[s];
        if (l > 0)
            rank(&R->count[s][COLUMNS + R->nb_spare - l],
                    &R->rank_spare[s][R->nb_spare - l], l);
    }
}

static void rank_spare_rows(struct compute *k)
{
    int l = k->free_rows;

    if (l > 0)
        rank(&k->count_row[ROWS_TOT - l],
                &k->rank_row[SPARE_R - l], l);
}

/* ------------------------------------------------------------------------
 * Effects of a replacement on the map and the counters
 * ------------------------------------------------------------------------ */

/*
 * Column c (0-63) of region R is replaced in section s: its defects vanish
 * from the map, including on the section's spare rows, and the row counters
 * drop accordingly. The column's own counter is zeroed by the caller.
 */
static void column_replaced(struct compute *k, struct region *R, int s, int c)
{
    int w = c >> 5, r, q;
    uint32_t bit = 1u << (c & 31);

    for (r = section_start[s]; r < section_start[s] + section_size[s]; r++) {
        if (R->board[r][w] & bit)
            k->count_row[r]--;
        R->board[r][w] &= ~bit;
    }
    /* Spare rows: map by number, counter by rank. */
    for (q = 0; q < SPARE_R; q++) {
        int p = k->rank_row[q];

        if (!spare_in_section(p, s))
            continue;
        if (R->board[ROWS + p][w] & bit)
            k->count_row[ROWS + q]--;
        R->board[ROWS + p][w] &= ~bit;
    }
}

/*
 * Normal row r is replaced: its defects vanish from the region's map, and
 * the column counters of its section drop accordingly (spares included, by
 * rank).
 */
static void row_erased(struct region *R, int r)
{
    int s = section_of(r), w, b, q;

    for (w = 0; w < 2; w++)
        for (b = 0; b < 32; b++)
            if (R->board[r][w] >> b & 1u)
                R->count[s][32 * w + b]--;
    for (q = 0; q < R->nb_spare; q++)
        if (R->board[r][2] >> R->rank_spare[s][q] & 1u)
            R->count[s][COLUMNS + q]--;
    R->board[r][0] = R->board[r][1] = R->board[r][2] = 0;
}

static void row_replaced(struct compute *k, int r)
{
    row_erased(&k->reg[REG_A], r);
    row_erased(&k->reg[REG_B], r);
    row_erased(&k->reg[REG_C], r);
}

static void record(struct usage *e, int taken, int target, int number)
{
    if (e->taken == 1)
        COVERED(27);    /* ORIGINAL: a recorded repair is overwritten (defect 2) */
    e->taken = taken;
    e->target = target;
    e->number = number;
}

/* ------------------------------------------------------------------------
 * Mandatory repairs
 * ------------------------------------------------------------------------ */

/*
 * Right half (B and C coupled). In each section, first the columns of C with
 * more than 16 defects — they take a coupled B/C pair — then those of B.
 *
 * ORIGINAL: the spare is accepted only if it has FEWER than 16 defects (<
 * here, <= for A). When the second loop discards a B spare that is too
 * dirty, it does not count off the coupled C spare: the two counters drift
 * apart and the ranks used afterwards no longer match (defect 2).
 */
static void mandatory_right(struct compute *k)
{
    struct region *B = &k->reg[REG_B], *C = &k->reg[REG_C];
    int s, c, l;

    for (s = 0; s < SECTIONS; s++) {
        for (c = 0; c < COLUMNS; c++) {
            if (C->count[s][c] > COLUMN_THRESHOLD) {
                for (l = C->free[s]; l > 0; l--) {
                    int q = C->nb_spare - l;      /* rank of the C spare */

                    /* FIXED: same threshold as for A (<= 16) */
                    if (C->count[s][COLUMNS + q] < COLUMN_THRESHOLD + (k->corrected ? 1 : 0)) {
                        COVERED(0);
                        C->count[s][c] = 0;
                        C->free[s]--;
                        column_replaced(k, C, s, c);
                        record(&k->col_b[s][q], 1, c, C->rank_spare[s][q]);
                        B->count[s][c] = 0;
                        B->free[s]--;
                        column_replaced(k, B, s, c);
                        break;
                    }
                    COVERED(1);
                    C->free[s]--;
                    record(&k->col_b[s][q], 0, 0, 0);
                    B->free[s]--;
                }
            }
            if (C->free[s] == 0)
                break;
        }
        for (c = 0; c < COLUMNS; c++) {
            if (B->count[s][c] > COLUMN_THRESHOLD) {
                for (l = B->free[s]; l > 0; l--) {
                    int q = B->nb_spare - l;

                    if (B->count[s][COLUMNS + q] < COLUMN_THRESHOLD + (k->corrected ? 1 : 0)) {
                        COVERED(2);
                        record(&k->col_b[s][q], 1, c, B->rank_spare[s][q]);
                        B->count[s][c] = 0;
                        B->free[s]--;
                        column_replaced(k, B, s, c);
                        if (C->free[s] > 0) {
                            COVERED(3);
                            C->count[s][c] = 0;
                            C->free[s]--;
                            column_replaced(k, C, s, c);
                        }
                        break;
                    }
                    COVERED(4);     /* defect 2: C is not counted off */
                    record(&k->col_b[s][q], 0, 0, 0);
                    B->free[s]--;
                    /*
                     * FIXED: a B spare of rank 0 or 1 forms a pair with the
                     * C spare of the same rank; discarding one consumes the
                     * other, otherwise the ranks of B and C drift apart and a
                     * pair already recorded ends up overwritten.
                     */
                    if (k->corrected && q < 2 && C->free[s] > 0)
                        C->free[s]--;
                }
            }
            if (B->free[s] == 0)
                break;
        }
    }
}

/* Left half (A): same rule, sorted spares, threshold <= 16. */
static void mandatory_left(struct compute *k)
{
    struct region *A = &k->reg[REG_A];
    int s, c, l;

    for (s = 0; s < SECTIONS; s++) {
        for (c = 0; c < COLUMNS; c++) {
            if (A->count[s][c] > COLUMN_THRESHOLD) {
                for (l = A->free[s]; l > 0; l--) {
                    int q = A->nb_spare - l;

                    if (A->count[s][COLUMNS + q] <= COLUMN_THRESHOLD) {
                        COVERED(5);
                        record(&k->col_a[s][q], 1, c, A->rank_spare[s][q]);
                        A->count[s][c] = 0;
                        A->free[s]--;
                        column_replaced(k, A, s, c);
                        break;
                    }
                    COVERED(6);
                    record(&k->col_a[s][q], 0, 0, 0);
                    A->free[s]--;
                }
            }
            if (A->free[s] == 0)
                break;
        }
    }
}

/* Rows with more than 3 defects: only a spare row saves them. */
static void mandatory_rows(struct compute *k)
{
    int r, l;

    for (r = 0; r < ROWS; r++) {
        if (k->count_row[r] > ROW_THRESHOLD) {
            for (l = k->free_rows; l > 0; l--) {
                int q = SPARE_R - l;

                if (k->count_row[ROWS + q] <= ROW_THRESHOLD) {
                    COVERED(7);
                    record(&k->row[q], 1, r, k->rank_row[q]);
                    k->count_row[r] = 0;
                    k->free_rows--;
                    row_replaced(k, r);
                    break;
                }
                COVERED(8);
                record(&k->row[q], 0, 0, 0);
                k->free_rows--;
            }
        }
        if (k->free_rows < 1)
            return;
    }
}

/* ------------------------------------------------------------------------
 * Greedy completion
 * ------------------------------------------------------------------------ */

/* Most loaded normal column of the section (the first one on ties). */
static int worst_column(const struct region *R, int s, int *col)
{
    int c, m = R->count[s][0];

    *col = 0;
    for (c = 0; c < COLUMNS; c++)
        if (R->count[s][c] > m) {
            m = R->count[s][c];
            *col = c;
        }
    return m;
}

/*
 * Right half. While coupled pairs remain, take the most loaded column of B or
 * C (B on ties) and replace it in both regions. Then the remaining B spares,
 * alone.
 *
 * ORIGINAL: when the most loaded one is in B, the spare's cleanliness is read
 * at rank 3 - l of B while rank 2 - l is the one used (defect 3). And
 * the cleanliness of the coupled spare is never checked.
 */
static void greedy_right(struct compute *k)
{
    struct region *B = &k->reg[REG_B], *C = &k->reg[REG_C];
    int s, l;

    for (s = 0; s < SECTIONS; s++) {
        if (B->free[s] != C->free[s] + 1)
            COVERED(9);             /* B and C ranks shifted (defect 2) */
        if (C->free[s] >= 1) {
            for (l = C->free[s]; l > 0; l--) {
                int q = C->nb_spare - l;      /* rank of the pair */
                int col, colc, m, mc, in_c = 0, own;

                m = worst_column(B, s, &col);
                /* C wins only strictly */
                mc = m;
                for (colc = 0; colc < COLUMNS; colc++)
                    if (C->count[s][colc] > mc) {
                        mc = C->count[s][colc];
                        col = colc;
                        in_c = 1;
                    }
                if (in_c) {
                    m = mc;
                    own = C->count[s][COLUMNS + q];
                } else {
                    own = B->count[s][COLUMNS + B->nb_spare - l];
                    if (m >= 1 && (m <= own) !=
                                  (m <= B->count[s][COLUMNS + q]))
                        COVERED(10);    /* defect 3 changes the decision */
                    if (k->corrected)     /* FIXED: the rank used */
                        own = B->count[s][COLUMNS + q];
                }
                if (m < 1) {
                    COVERED(11);
                    continue;           /* nothing left: let it go */
                }
                if (m <= own) {
                    COVERED(in_c ? 12 : 13);
                    record(&k->col_b[s][q], 0, 0, 0);    /* spare discarded */
                    C->free[s]--;
                    B->free[s]--;
                    continue;
                }
                COVERED(in_c ? 14 : 15);
                C->count[s][col] = 0;
                C->free[s]--;
                column_replaced(k, C, s, col);
                record(&k->col_b[s][q], 1, col,
                      in_c ? C->rank_spare[s][q] : B->rank_spare[s][q]);
                B->count[s][col] = 0;
                B->free[s]--;
                column_replaced(k, B, s, col);
            }
        } else if (B->free[s] == 0 && C->free[s] == 0) {
            continue;
        }
        if (B->free[s] > 0) {
            for (l = B->free[s]; l > 0; l--) {
                int q = B->nb_spare - l, col, m;

                m = worst_column(B, s, &col);
                if (m < 1)
                    continue;
                if (m <= B->count[s][COLUMNS + q]) {
                    COVERED(16);
                    record(&k->col_b[s][q], 0, 0, 0);
                    B->free[s]--;
                    continue;
                }
                COVERED(q < 2 ? 17 : 18);   /* 17: coupled rank used alone */
                record(&k->col_b[s][q], 1, col, B->rank_spare[s][q]);
                B->count[s][col] = 0;
                column_replaced(k, B, s, col);
                B->free[s]--;
            }
        }
    }
}

/* Left half: the most loaded, as long as the spare is cleaner. */
static void greedy_left(struct compute *k)
{
    struct region *A = &k->reg[REG_A];
    int s, l;

    for (s = 0; s < SECTIONS; s++) {
        for (l = A->free[s]; l > 0; l--) {
            int q = A->nb_spare - l, col, m;

            m = worst_column(A, s, &col);
            if (m < 1 || m <= A->count[s][COLUMNS + q]) {
                COVERED(m < 1 ? 19 : 20);
                break;
            }
            COVERED(21);
            record(&k->col_a[s][q], 1, col, A->rank_spare[s][q]);
            A->count[s][col] = 0;
            column_replaced(k, A, s, col);
            A->free[s]--;
        }
    }
}

/* Rows: the most loaded, as long as the spare is cleaner. */
static void greedy_rows(struct compute *k)
{
    int l = k->free_rows;

    while (l >= 1) {
        int q = SPARE_R - l, r, worst = 0, m = k->count_row[0];

        for (r = 0; r < ROWS; r++)
            if (k->count_row[r] > m) {
                m = k->count_row[r];
                worst = r;
            }
        if (m < 1 || m <= k->count_row[ROWS + q]) {
            COVERED(m < 1 ? 22 : 23);
            return;
        }
        COVERED(24);
        record(&k->row[q], 1, worst, k->rank_row[q]);
        k->count_row[worst] = 0;
        k->free_rows--;
        row_replaced(k, worst);
        l--;
    }
}

/* ------------------------------------------------------------------------
 * Row-only mode (MRR verdict "way 2", at most 16 faulty rows)
 * ------------------------------------------------------------------------ */

/*
 * The input is the list of faulty rows. An entry marked 0x800 designates a
 * faulty SPARE row (number in the low 4 bits): it is discarded. Each other
 * row gets the first free spare, in order 0 to 15.
 *
 * ORIGINAL: the row is copied as is, without a mask; a duplicate 0x800 entry
 * is counted off twice (possible premature stop).
 */
static void rows_only(struct compute *k, const uint32_t *in, uint32_t n)
{
    int taken[SPARE_R] = { 0 };
    int rest = SPARE_R, q;
    uint32_t i;

    /*
     * The "way 2" verdict guarantees at most 16 rows and the caller reads no
     * more; the original read beyond without a check.
     */
    if (n > SPARE_R)
        n = SPARE_R;
    for (i = 0; i < n; i++)
        if (in[i] & 0x800u) {
            q = (int)(in[i] & 0xfu);
            if (k->corrected && taken[q])
                continue;       /* FIXED: a duplicate counts only once */
            taken[q] = 1;
            record(&k->row[q], 0, 0, 0);
            rest--;
        }
    for (i = 0; i < n; i++) {
        if (!(in[i] & 0x800u))
            for (q = 0; q < SPARE_R; q++)
                if (!taken[q]) {
                    /* FIXED: 11 bits, or the row spills into the identity */
                    int target = k->corrected ? (int)(in[i] & 0x7ffu) : (int)in[i];

                    record(&k->row[q], 1, target, k->rank_row[q]);
                    taken[q] = 1;
                    rest--;
                    break;
                }
        if (rest < 1)
            return;
    }
}

/* ------------------------------------------------------------------------
 * Output: 52 words
 * ------------------------------------------------------------------------ */

/*
 *   0-17   spare columns of A, section × 3 + rank
 *  18-35   spare columns of B (and coupled C), section × 3 + rank
 *  36-51   spare rows, by rank
 * bit 31 valid entry; 30 the spare itself is defect-free; 29 no defect left
 * in the bank (same value everywhere); 28 row; 15-27 identity; 14 region A;
 * 13 and 3-0 number of the column spare (section × 3 + number) or 3-0 spare
 * row number; 4-12 replaced column, 4-14 replaced row. Only the entries
 * taken are written.
 */
static void write(const struct compute *k, int *out, uint16_t id)
{
    const struct region *A = &k->reg[REG_A], *B = &k->reg[REG_B];
    uint32_t base = 0x80000000u | (uint32_t)id << 15;
    int s, q, r;

    for (r = 0; r < ROWS_TOT; r++)
        if (k->count_row[r] > 0)
            break;
    if (r == ROWS_TOT) {
        COVERED(25);
        base |= 1u << 29;
    } else {
        COVERED(26);
    }

    for (s = 0; s < SECTIONS; s++)
        for (q = 0; q < 3; q++) {
            const struct usage *e = &k->col_a[s][q];
            uint32_t n;

            if (e->taken != 1)
                continue;
            n = (uint32_t)(A->rank_spare[s][q] + 3 * s);
            out[3 * s + q] = (int)(base | 0x4000u |
                (A->count[s][COLUMNS + q] == 0 ? 1u << 30 : 0) |
                (n >> 4) << 13 | (uint32_t)e->target << 4 | (n & 0xfu));
        }
    for (s = 0; s < SECTIONS; s++)
        for (q = 0; q < 3; q++) {
            const struct usage *e = &k->col_b[s][q];
            uint32_t n;

            if (e->taken != 1)
                continue;
            n = (uint32_t)(B->rank_spare[s][q] + 3 * s);
            out[18 + 3 * s + q] = (int)(base |
                (B->count[s][COLUMNS + q] == 0 ? 1u << 30 : 0) |
                (n >> 4) << 13 | (uint32_t)e->target << 4 | (n & 0xfu));
        }
    for (q = 0; q < SPARE_R; q++) {
        const struct usage *e = &k->row[q];

        if (e->taken != 1)
            continue;
        out[36 + q] = (int)(base | 0x10000000u |
            (k->count_row[ROWS + q] == 0 ? 1u << 30 : 0) |
            (uint32_t)e->target << 4 | (uint32_t)k->rank_row[q]);
    }
}

/* The original's computation (patched if "corrige"). */
static void compute_faithful(const uint32_t *in, uint32_t n_in, int *out,
                          uint16_t dft_id, int row_only, int corrected)
{
    struct compute k;

    initialize(&k);
    k.corrected = corrected;
    if (row_only == 1) {
        rows_only(&k, in, n_in);
    } else {
        split(&k, in);
        count_columns(&k.reg[REG_A]);
        count_columns(&k.reg[REG_B]);
        count_columns(&k.reg[REG_C]);
        count_rows(&k);
        rank_spare_a(&k.reg[REG_A]);
        mandatory_right(&k);
        mandatory_left(&k);
        rank_spare_rows(&k);
        mandatory_rows(&k);
        rank_spare_a(&k.reg[REG_A]);
        greedy_right(&k);
        greedy_left(&k);
        rank_spare_rows(&k);
        greedy_rows(&k);
    }
    write(&k, out, dft_id);
}

/* ========================================================================
 * EVALUATION: apply 52 words to the map that was read and count what remains
 * wrong in the logical array (2048 rows × 192 columns).
 *
 * Model, the one the original algorithm assumes in its counts: a replaced row
 * is read in its spare row, with the column replacements of the section that
 * hosts that spare (1 for spares 0-7, 4 for 8-15); a replaced column is read
 * in its section's spare column; spares 0 and 1 of B also replace the column
 * of the same number in C. The judge is independent of the allocations it
 * judges.
 * ======================================================================== */

static const int word0_region[REGIONS] = { 0, 2, 5 };
static const int dec_spare[REGIONS] = { 0, 3, 6 };

static unsigned bit_normal(const uint32_t *in, int pr, int g, int c)
{
    return in[(size_t)pr * WORDS_IN + word0_region[g] + (c >> 5)] >> (c & 31) & 1u;
}

static unsigned bit_spare(const uint32_t *in, int pr, int g, int k)
{
    return in[(size_t)pr * WORDS_IN + 4] >> (dec_spare[g] + k) & 1u;
}

/* Section hosting a physical row, normal or spare. */
static int section_physical(int pr)
{
    if (pr < ROWS)
        return section_of(pr);
    return (pr - ROWS) < 8 ? 1 : 4;
}

void jas_repair_evaluate(const uint32_t *in, const int *out, uint16_t id,
                            struct jas_repair_eval *e)
{
    struct { signed char g, k; unsigned char c; } remap[SECTIONS][16];
    int nremap[SECTIONS];
    signed char rowmap[ROWS];
    char col_taken[REGIONS][SECTIONS][COLUMNS];
    char sp_col[REGIONS][SECTIONS][3], sp_row[SPARE_R];
    int k, r, s, i;

    memset(e, 0, sizeof(*e));
    memset(nremap, 0, sizeof(nremap));
    memset(rowmap, -1, sizeof(rowmap));
    memset(col_taken, 0, sizeof(col_taken));
    memset(sp_col, 0, sizeof(sp_col));
    memset(sp_row, 0, sizeof(sp_row));

    for (k = 0; k < 36; k++) {
        uint32_t x = (uint32_t)out[k];
        int g = k < 18 ? REG_A : REG_B, n, c, sp, gg;

        if (!(x >> 31))
            continue;
        n = (int)(((x >> 13) & 1u) << 4 | (x & 0xfu));
        c = (int)((x >> 4) & 0x1ffu);
        if ((x >> 28 & 1u) || (int)(x >> 14 & 1u) != (g == REG_A) || n > 17 ||
            c > 63 || ((x >> 15) & 0x1fffu) != id) {
            e->malformed++;
            continue;
        }
        s = n / 3;
        sp = n % 3;
        for (gg = g; ; gg = REG_C) {
            if (col_taken[gg][s][c] || sp_col[gg][s][sp])
                e->conflicts++;
            col_taken[gg][s][c] = 1;
            sp_col[gg][s][sp] = 1;
            if (nremap[s] < 16) {
                remap[s][nremap[s]].g = (signed char)gg;
                remap[s][nremap[s]].k = (signed char)sp;
                remap[s][nremap[s]].c = (unsigned char)c;
                nremap[s]++;
            }
            if (gg == REG_C || g != REG_B || sp >= 2)
                break;          /* B 0 and 1 drag C along */
        }
    }
    for (k = 36; k < WORDS_OUT; k++) {
        uint32_t x = (uint32_t)out[k];
        int p;

        if (!(x >> 31))
            continue;
        if (!(x >> 28 & 1u) || ((x >> 15) & 0x1fffu) != id) {
            e->malformed++;
            continue;
        }
        p = (int)(x & 0xfu);
        r = (int)((x >> 4) & 0x7ffu);
        if (rowmap[r] >= 0 || sp_row[p])
            e->conflicts++;
        rowmap[r] = (signed char)p;
        sp_row[p] = 1;
    }
    for (r = 0; r < ROWS; r++) {
        int pr = rowmap[r] >= 0 ? ROWS + rowmap[r] : r;
        const uint32_t *m = in + (size_t)pr * WORDS_IN;
        uint32_t w[6];
        int n;

        s = section_physical(pr);
        w[0] = m[0]; w[1] = m[1]; w[2] = m[2]; w[3] = m[3]; w[4] = m[5]; w[5] = m[6];
        for (i = 0; i < nremap[s]; i++) {
            int g = remap[s][i].g, c = remap[s][i].c;
            uint32_t v = m[4] >> (dec_spare[g] + remap[s][i].k) & 1u;
            int j = g * 2 + (c >> 5);

            w[j] = (w[j] & ~(1u << (c & 31))) | v << (c & 31);
        }
        n = bits_set(w[0]) + bits_set(w[1]) + bits_set(w[2]) +
            bits_set(w[3]) + bits_set(w[4]) + bits_set(w[5]);
        e->cells += n;
        if (n)
            e->rows++;
    }
}

/* ========================================================================
 * REDESIGNED ALLOCATION (corrected mode)
 *
 * Same resources, same output format, but an exact model of what will be
 * read: each replacement updates the defects of each row and each column AS
 * THEY WILL BE READ. Which fixes in one go:
 *   D2/D3/D4  a B/C pair is a single object (linked ranks), judged on its two
 *             spares and on what it removes on both sides;
 *   D5        a single criterion everywhere: the net balance (defects removed
 *             minus defects brought in), and a spare is never "discarded" for
 *             nothing;
 *   D6        a spare's defects only count if it is used: an unused spare
 *             column loads no row, an unused spare row no column; bit 29 tells
 *             the truth.
 * Sequence: mandatory repairs (a column too loaded for the free spare rows, a
 * row too loaded for the free spare columns of its section), until stable;
 * then, while a replacement removes more defects than it brings in, the best
 * one — column on an equal balance.
 * ======================================================================== */

struct redesigned {
    const uint32_t *in;
    signed char rowmap[ROWS];
    signed char sr[SPARE_R];                  /* spare row taken */
    short sr_target[SPARE_R];
    signed char colmap[REGIONS][SECTIONS][COLUMNS];
    signed char sc[REGIONS][SECTIONS][3];       /* spare column taken */
    short sc_target[REGIONS][SECTIONS][3];
    int colcnt[REGIONS][SECTIONS][COLUMNS];    /* defects read, live rows */
    int spcnt[REGIONS][SECTIONS][3];            /* spare's defects, live rows */
    int rowcnt[ROWS];                        /* defects read, logical row */
    int sprowcnt[SPARE_R];                    /* defects this spare would read */
};

static unsigned fault_read(const struct redesigned *R, int pr, int g, int c)
{
    int k = R->colmap[g][section_physical(pr)][c];

    return k >= 0 ? bit_spare(R->in, pr, g, k) : bit_normal(R->in, pr, g, c);
}

/* Defects read of a physical row in a region (normal columns). */
static int faults_region(const struct redesigned *R, int pr, int g)
{
    int c, n = 0;

    for (c = 0; c < COLUMNS; c++)
        n += (int)fault_read(R, pr, g, c);
    return n;
}

static int rows_free(const struct redesigned *R)
{
    int p, n = 0;

    for (p = 0; p < SPARE_R; p++)
        n += !R->sr[p];
    return n;
}

/* Free column spares of the region in the section (B: pairs + single). */
static int columns_free(const struct redesigned *R, int g, int s)
{
    int k, n = 0;

    for (k = 0; k < spare_per_section[g]; k++)
        n += !R->sc[g][s][k];
    return n;
}

/* The cleanest free spare row (as it would be read). */
static int best_free_row(const struct redesigned *R)
{
    int p, m = -1;

    for (p = 0; p < SPARE_R; p++)
        if (!R->sr[p] && (m < 0 || R->sprowcnt[p] < R->sprowcnt[m]))
            m = p;
    return m;
}

static void redesigned_init(struct redesigned *R, const uint32_t *in)
{
    int pr, g, c, k;

    memset(R, 0, sizeof(*R));
    R->in = in;
    memset(R->rowmap, -1, sizeof(R->rowmap));
    memset(R->colmap, -1, sizeof(R->colmap));
    for (pr = 0; pr < ROWS_TOT; pr++) {
        int s = section_physical(pr), n = 0;

        for (g = 0; g < REGIONS; g++)
            for (c = 0; c < COLUMNS; c++)
                if (bit_normal(in, pr, g, c)) {
                    n++;
                    if (pr < ROWS)
                        R->colcnt[g][s][c]++;
                }
        if (pr < ROWS) {
            R->rowcnt[pr] = n;
            for (g = 0; g < REGIONS; g++)
                for (k = 0; k < spare_per_section[g]; k++)
                    R->spcnt[g][s][k] += (int)bit_spare(in, pr, g, k);
        } else {
            R->sprowcnt[pr - ROWS] = n;
        }
    }
}

/* Column c of region g, section s, is now read in spare k. */
static void redesigned_column(struct redesigned *R, int g, int s, int c, int k)
{
    int pr, p;

    for (pr = section_start[s]; pr < section_start[s] + section_size[s]; pr++) {
        int d = (int)bit_spare(R->in, pr, g, k) - (int)bit_normal(R->in, pr, g, c);

        if (d && R->rowmap[pr] < 0)
            R->rowcnt[pr] += d;
    }
    for (p = 0; p < SPARE_R; p++) {
        int d;

        if (!spare_in_section(p, s))
            continue;
        d = (int)bit_spare(R->in, ROWS + p, g, k) -
            (int)bit_normal(R->in, ROWS + p, g, c);
        if (!d)
            continue;
        R->sprowcnt[p] += d;
        if (R->sr[p])
            R->rowcnt[R->sr_target[p]] += d;
    }
    R->colmap[g][s][c] = (signed char)k;
    R->sc[g][s][k] = 1;
    R->sc_target[g][s][k] = (short)c;
    R->colcnt[g][s][c] = R->spcnt[g][s][k];
}

/* B/C pair of rank k (0 or 1) on column c, or B alone (k = 2). */
static void redesigned_right(struct redesigned *R, int s, int c, int k)
{
    redesigned_column(R, REG_B, s, c, k);
    if (k < 2)
        redesigned_column(R, REG_C, s, c, k);
}

/* Logical row r is now read in spare row p. */
static void redesigned_row(struct redesigned *R, int r, int p)
{
    int s = section_of(r), sp = p < 8 ? 1 : 4, g, c, k, pr = ROWS + p;

    for (g = 0; g < REGIONS; g++) {
        for (c = 0; c < COLUMNS; c++) {
            if (fault_read(R, r, g, c))
                R->colcnt[g][s][c]--;
            if (fault_read(R, pr, g, c))
                R->colcnt[g][sp][c]++;
        }
        for (k = 0; k < spare_per_section[g]; k++) {
            R->spcnt[g][s][k] -= (int)bit_spare(R->in, r, g, k);
            R->spcnt[g][sp][k] += (int)bit_spare(R->in, pr, g, k);
        }
    }
    R->rowmap[r] = (signed char)p;
    R->sr[p] = 1;
    R->sr_target[p] = (short)r;
    R->rowcnt[r] = R->sprowcnt[p];
}

/* Net balance of a column replacement; -1,000,000 if impossible. */
#define IMPOSSIBLE (-1000000)

static int summary_left(const struct redesigned *R, int s, int c, int k)
{
    if (R->colmap[REG_A][s][c] >= 0 || R->sc[REG_A][s][k])
        return IMPOSSIBLE;
    return R->colcnt[REG_A][s][c] - R->spcnt[REG_A][s][k];
}

static int summary_right(const struct redesigned *R, int s, int c, int k)
{
    int b;

    if (R->colmap[REG_B][s][c] >= 0 || R->sc[REG_B][s][k])
        return IMPOSSIBLE;
    b = R->colcnt[REG_B][s][c] - R->spcnt[REG_B][s][k];
    if (k < 2) {
        if (R->colmap[REG_C][s][c] >= 0 || R->sc[REG_C][s][k])
            return IMPOSSIBLE;
        b += R->colcnt[REG_C][s][c] - R->spcnt[REG_C][s][k];
    }
    return b;
}

/*
 * Mandatory repairs, until stable. Returns 1 if something changed. A column
 * whose read defects exceed the free spare rows can only be saved by a
 * column: we take the best spare, if it removes more than it brings in. A row
 * that has, in a region, more faulty columns than free spares in its section
 * can only be saved by a row.
 */
static int redesigned_mandatory(struct redesigned *R)
{
    int change = 0, s, c, k, r;

    for (s = 0; s < SECTIONS; s++)
        for (c = 0; c < COLUMNS; c++) {
            int lib = rows_free(R), best = IMPOSSIBLE, bk = -1;

            if (R->colmap[REG_A][s][c] < 0 && R->colcnt[REG_A][s][c] > lib) {
                for (k = 0; k < 3; k++) {
                    int b = summary_left(R, s, c, k);
                    if (b > best) { best = b; bk = k; }
                }
                if (bk >= 0 && best > 0) {
                    redesigned_column(R, REG_A, s, c, bk);
                    change = 1;
                }
            }
            best = IMPOSSIBLE;
            bk = -1;
            if (R->colmap[REG_B][s][c] < 0 &&
                (R->colcnt[REG_B][s][c] > lib || R->colcnt[REG_C][s][c] > lib)) {
                for (k = 0; k < 3; k++) {
                    int b;
                    if (k == 2 && R->colcnt[REG_C][s][c] > lib)
                        continue;       /* C needs a pair */
                    b = summary_right(R, s, c, k);
                    if (b > best) { best = b; bk = k; }
                }
                if (bk >= 0 && best > 0) {
                    redesigned_right(R, s, c, bk);
                    change = 1;
                }
            }
        }
    for (r = 0; r < ROWS; r++) {
        int s2, g, too = 0, p;

        if (R->rowmap[r] >= 0 || R->rowcnt[r] == 0)
            continue;
        s2 = section_of(r);
        for (g = 0; g < REGIONS && !too; g++)
            if (faults_region(R, r, g) > columns_free(R, g, s2))
                too = 1;
        if (!too)
            continue;
        p = best_free_row(R);
        if (p >= 0 && R->sprowcnt[p] < R->rowcnt[r]) {
            redesigned_row(R, r, p);
            change = 1;
        }
    }
    return change;
}

static void redesigned_greedy(struct redesigned *R)
{
    for (;;) {
        int best = 0, type = -1, bs = 0, bc = 0, bk = 0, br = 0, s, c, k, r, p;

        for (s = 0; s < SECTIONS; s++)
            for (c = 0; c < COLUMNS; c++)
                for (k = 0; k < 3; k++) {
                    int b = summary_left(R, s, c, k);
                    if (b > best) { best = b; type = 0; bs = s; bc = c; bk = k; }
                    b = summary_right(R, s, c, k);
                    if (b > best) { best = b; type = 1; bs = s; bc = c; bk = k; }
                }
        p = best_free_row(R);
        if (p >= 0)
            for (r = 0; r < ROWS; r++)
                if (R->rowmap[r] < 0 && R->rowcnt[r] - R->sprowcnt[p] > best) {
                    best = R->rowcnt[r] - R->sprowcnt[p];
                    type = 2;
                    br = r;
                }
        if (type < 0)
            return;
        if (type == 0)
            redesigned_column(R, REG_A, bs, bc, bk);
        else if (type == 1)
            redesigned_right(R, bs, bc, bk);
        else
            redesigned_row(R, br, p);
    }
}

static void compute_redesigned(const uint32_t *in, int *out, uint16_t id)
{
    struct redesigned *R, local;
    uint32_t base;
    int s, k, p, r, residue = 0;

    R = &local;
    redesigned_init(R, in);
    while (redesigned_mandatory(R))
        ;
    redesigned_greedy(R);

    for (r = 0; r < ROWS; r++)
        residue += R->rowcnt[r];
    base = 0x80000000u | (uint32_t)id << 15 | (residue == 0 ? 1u << 29 : 0);
    for (s = 0; s < SECTIONS; s++)
        for (k = 0; k < 3; k++) {
            uint32_t n = (uint32_t)(3 * s + k);

            if (R->sc[REG_A][s][k])
                out[3 * s + k] = (int)(base | 0x4000u |
                    (R->spcnt[REG_A][s][k] == 0 ? 1u << 30 : 0) |
                    (n >> 4) << 13 | (uint32_t)R->sc_target[REG_A][s][k] << 4 | (n & 0xfu));
            if (R->sc[REG_B][s][k])
                out[18 + 3 * s + k] = (int)(base |
                    (R->spcnt[REG_B][s][k] == 0 ? 1u << 30 : 0) |
                    (n >> 4) << 13 | (uint32_t)R->sc_target[REG_B][s][k] << 4 | (n & 0xfu));
        }
    for (p = 0; p < SPARE_R; p++)
        if (R->sr[p])
            out[36 + p] = (int)(base | 0x10000000u |
                (R->sprowcnt[p] == 0 ? 1u << 30 : 0) |
                (uint32_t)R->sr_target[p] << 4 | (uint32_t)p);
}

/* ========================================================================
 * Public entry points
 * ======================================================================== */

/* is a better than b? (valid output first, then rows, then cells) */
static int best(const struct jas_repair_eval *a,
                    const struct jas_repair_eval *b)
{
    int va = a->conflicts == 0 && a->malformed == 0;
    int vb = b->conflicts == 0 && b->malformed == 0;

    if (va != vb)
        return va;
    if (a->rows != b->rows)
        return a->rows < b->rows;
    return a->cells < b->cells;
}

int jas_repair_compute_summary(const uint32_t *in, uint32_t n_in, int *out,
                                  uint16_t dft_id, int row_only, int mode,
                                  struct jas_repair_summary *summary)
{
    int cand[3][WORDS_OUT];
    struct jas_repair_eval ev[3];
    int i, choice = 0;

    if (summary)
        memset(summary, 0, sizeof(*summary));
    if (row_only == 1 || mode == JAS_REPAIR_FAITHFUL ||
        mode == JAS_REPAIR_PATCHED) {
        int t[WORDS_OUT];

        memset(t, 0, sizeof(t));
        compute_faithful(in, n_in, t, dft_id, row_only,
                      mode != JAS_REPAIR_FAITHFUL);
        memcpy(out, t, sizeof(t));
        if (summary) {
            summary->choice = mode == JAS_REPAIR_FAITHFUL ? 0 : 1;
            if (row_only != 1) {
                jas_repair_evaluate(in, out, dft_id, &summary->kept);
                summary->faithful = summary->kept;
            }
        }
        return summary ? summary->choice : 0;
    }
    if (mode == JAS_REPAIR_REDESIGNED) {
        int t[WORDS_OUT];

        memset(t, 0, sizeof(t));
        compute_redesigned(in, t, dft_id);
        memcpy(out, t, sizeof(t));
        if (summary) {
            summary->choice = 2;
            jas_repair_evaluate(in, out, dft_id, &summary->kept);
        }
        return 2;
    }
    /* CORRECTED: best of the three per the simulator, the original on ties. */
    memset(cand, 0, sizeof(cand));
    compute_faithful(in, n_in, cand[0], dft_id, 0, 0);
    compute_faithful(in, n_in, cand[1], dft_id, 0, 1);
    compute_redesigned(in, cand[2], dft_id);
    for (i = 0; i < 3; i++)
        jas_repair_evaluate(in, cand[i], dft_id, &ev[i]);
    for (i = 1; i < 3; i++)
        if (best(&ev[i], &ev[choice]))
            choice = i;
    memcpy(out, cand[choice], sizeof(cand[choice]));
    if (summary) {
        summary->choice = choice;
        summary->faithful = ev[0];
        summary->kept = ev[choice];
    }
    return choice;
}

void jas_repair_compute_mode(const uint32_t *in, uint32_t n_in, int *out,
                                  uint16_t dft_id, int row_only, int mode)
{
    jas_repair_compute_summary(in, n_in, out, dft_id, row_only, mode, NULL);
}

void jas_repair_compute(const uint32_t *in, uint32_t n_in, int *out,
                             int out_stride, uint16_t dft_id, int row_only,
                             int reserved)
{
    (void)out_stride;   /* ORIGINAL: ignored, the output is always 52 words */
    (void)reserved;     /* the original's incremental mode, never used */
    jas_repair_compute_mode(in, n_in, out, dft_id, row_only,
                                 JAS_REPAIR_FAITHFUL);
}
