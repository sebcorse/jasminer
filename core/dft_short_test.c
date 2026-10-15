/*
 * The startup memory test.
 *
 * The short memory test, called by self_check_func. It decides, bank by
 * bank, what will be usable: its verdicts feed bank_good/partial/bad, which
 * config_remap_info then turns into the capacity announced to the chip. An
 * error here does not show at startup: it shows hours later, as rejected
 * work.
 *
 * "short" as opposed to the long test: the firmware also carries "Item4"
 * patterns (4 × 19,668 bytes per unit) that this function does not use. It
 * sticks to three families:
 *
 *     initial   3 command blocks         conditioning
 *     Item3     138 blocks               the test pattern proper
 *     MRR       register read            reading the result
 *
 * Organization, from the widest to the finest:
 *
 *     for each chip, for each of its 4 meshes:
 *         data path test             (510 banks)
 *         hybrid bond test           (510 banks × 2 units)
 *         unit 0: pre → OTP → post_even → post_odd
 *         unit 1: same
 *         carry the 2 × 510 verdicts over to the mesh's 1020 banks
 *
 * A "unit" is half a chip; its 510 banks are scattered in the global space
 * through dft_id_2_remap_id_even / _odd. A mesh thus carries 1020 banks, a
 * chip 4080.
 *
 * TWO POINTS DESERVE ATTENTION.
 *
 * The multi → s2m fallback: each phase first tries dft_cmd_multi_write; if
 * it fails, it switches to dft_cmd_s2m_write and *stays there* for the rest
 * of the test (sel_single_or_mult): a lasting change of regime, decided by
 * the first failure met.
 *
 * The cache in flash: the full repair survey is very long
 * (dft_get_full_repair_info reads 2064 rows of 14 words, one word per
 * serial round trip), so its result is written to /media/configs/<mode>/
 * and read back as is at the following startups. A stale or corrupt repair
 * file means misplaced redundancies, with no test reporting it.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include "dft.h"
#include "dft_tables.h"
#include "repair.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>

#define N_MESH        4u
#define N_BANK_UNIT   0x1feu    /* 510 banks per unit */
#define N_BANK_MESH   1020u     /* both units */
#define N_BANK_ASIC   0xff0u    /* 4080                                       */
#define N_BANK_BOARD  0x2fd0u   /* 12240                                      */
#define REPAIR_WORDS  0x34u     /* 52 repair integers per bank */
#define FULL_WORDS    14448u    /* 2064 rows × 7 */
#define N_INITIAL     3u        /* blocks of the "initial" phase */
#define N_ITEM3       0x8au     /* 138 blocks of the "Item3" phase */
#define CMD_STRIDE32  11u       /* a command block = 11 32-bit words */

#define ASIC(i)       ((uint8_t)(1u << (i)))
#define DFT_W(mesh)   ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0x8u))
#define CMD_BROADCAST 0xf8

#define REG_MC_TRIM      0x1c0c
#define REG_TRIM_FIRST   0xd61
#define REG_TRIM_LAST    0xd6d
#define REG_TRIM_0       0xd60
#define REG_TRIM_APPLY   0xd6d
#define REG_FULL_STROBE  0xd31
#define REG_MRR_STROBE   0xd2d
#define REG_SLOT_STROBE  0xd2b
#define REG_BIST_STROBE  0xd2a

#define CONFIG_DIR       "/media/configs/"

/*
 * Where the repair files live.
 *
 * /media/configs/ in service. But these files drive the allocation of the
 * memory redundancies: overwriting them with the result of code still
 * being validated would durably damage a machine's configuration, without
 * any test reporting it. JASRE_CONFIG_DIR therefore sends them elsewhere
 * during trials.
 */
/*
 * MARKER IN RAM: as no environment variable reaches the miner started by
 * jasminer.sh, the file /tmp/jasre_config_dir can carry the trial
 * directory (first line). /tmp is a RAM disk: a reboot erases the marker
 * and brings back the factory files. A new directory, without a mode
 * subdirectory, triggers the full survey (dft_work_mode_init).
 */
static char config_dir_trial[128];
static pthread_once_t config_dir_once = PTHREAD_ONCE_INIT;

static void config_dir_read(void)
{
    const char *e = getenv("JASRE_CONFIG_DIR");
    size_t n;

    if (e && *e) {
        snprintf(config_dir_trial, sizeof(config_dir_trial), "%s", e);
    } else {
        FILE *f = fopen("/tmp/jasre_config_dir", "r");

        if (f) {
            if (!fgets(config_dir_trial, sizeof(config_dir_trial), f))
                config_dir_trial[0] = 0;
            config_dir_trial[strcspn(config_dir_trial, "\r\n")] = 0;
            fclose(f);
        }
    }
    n = strlen(config_dir_trial);
    if (n && config_dir_trial[n - 1] != '/' && n + 1 < sizeof(config_dir_trial)) {
        config_dir_trial[n] = '/';
        config_dir_trial[n + 1] = 0;
    }
    if (config_dir_trial[0])
        jas_log("DFT: repairs in the TRIAL directory %s (factory files of "
                CONFIG_DIR " untouched)", config_dir_trial);
}

static const char *config_dir(void)
{
    pthread_once(&config_dir_once, config_dir_read);
    return config_dir_trial[0] ? config_dir_trial : CONFIG_DIR;
}

/*
 * Redundancy allocation mode (core/repair_calc.c): JASRE_REPAIR,
 * otherwise the jasre_repair marker, read in /tmp (trial) then in
 * /media/configs (lasting, without reflashing): "corrected" (the default,
 * best of three per the simulator), "faithful" (the original, without its
 * uninitialized tables), "patched" or "redesigned" (trials). Only acts at
 * a full survey.
 */
static int repair_mode_val;
static pthread_once_t repair_once = PTHREAD_ONCE_INIT;

static void repair_mode_read(void)
{
    static const char *names[4] = { "faithful", "corrected", "patched", "redesigned" };
    static const char *markers[2] = { "/tmp/jasre_repair",
                                      "/media/configs/jasre_repair" };
    char v[32] = "";
    const char *e = getenv("JASRE_REPAIR");
    int i;

    if (e && *e) {
        snprintf(v, sizeof(v), "%s", e);
    } else {
        for (i = 0; i < 2 && !v[0]; i++) {
            FILE *f = fopen(markers[i], "r");

            if (f) {
                if (!fgets(v, sizeof(v), f))
                    v[0] = 0;
                v[strcspn(v, "\r\n")] = 0;
                fclose(f);
            }
        }
    }
    repair_mode_val = JAS_REPAIR_CORRECTED;
    for (i = 0; i < 4; i++)
        if (!strcmp(v, names[i]))
            repair_mode_val = i;
#ifdef JAS_REPAIR_ORIGINAL
    jas_log("DFT: redundancy allocation by librepairarm (production image;"
            " marker \"%s\" ignored)", names[repair_mode_val]);
#else
    jas_log("DFT: redundancy allocation \"%s\"", names[repair_mode_val]);
#endif
}

static int repair_mode(void)
{
    pthread_once(&repair_once, repair_mode_read);
    return repair_mode_val;
}

/*
 * CAPTURE OF THE REAL MAPS (JASRE_CAPTURE=1 or marker /tmp/jasre_capture).
 * During a full survey: each computed bank (input, outputs of the original
 * and of the chosen mode, simulated balances), then each post-burn verdict
 * of the banks repaired or not perfect. One file per mesh, in
 * <directory>/captures/, written by the mesh's thread alone. Format: 32-bit
 * words, each record starts with CAPTURE_MAGIC and its type.
 */
#define CAPTURE_MAGIC  0x50414352u      /* "RCAP" */
static int capture_val = -1;
static pthread_once_t capture_once = PTHREAD_ONCE_INIT;

static void capture_read(void)
{
    const char *e = getenv("JASRE_CAPTURE");

    capture_val = (e && *e == '1') || access("/tmp/jasre_capture", F_OK) == 0;
    if (capture_val)
        jas_log("DFT: capturing maps and verdicts in %scaptures/",
                config_dir());
}

static FILE *capture_open(int board, unsigned asic, unsigned mesh)
{
    char path[192];

    pthread_once(&capture_once, capture_read);
    if (!capture_val)
        return NULL;
    snprintf(path, sizeof(path), "%scaptures", config_dir());
    mkdir(path, 0755);                /* already exists: no effect */
    snprintf(path, sizeof(path), "%scaptures/cap_b%d_a%u_m%u.bin",
             config_dir(), board, asic, mesh);
    return fopen(path, "ab");
}

static void capture_words(FILE *f, const void *p, size_t n)
{
    if (f && n)
        fwrite(p, sizeof(uint32_t), n, f);
}

/*
 * The state of a test in progress.
 *
 * The original keeps all of it in stack variables, nearly 550 KB, which
 * holds because it runs on the main stack. We allocate: the test now runs
 * in threads (one per chip or per mesh), whose stacks are not that large.
 */
struct dft_run {
    int      board;
    unsigned asic;
    unsigned mesh;

    /* The write regime, once a failure has switched it. */
    int use_s2m;
    /* Parallel DFT (one thread per chip or per mesh): the switch is shared
     * between the threads, as when a single dft_run chained the chips. NULL
     * when sequential. */
    volatile int *s2m_shared;

    /* Verdicts per unit, before carrying them over to the global space. */
    uint16_t good[2][N_BANK_UNIT];
    uint16_t partial[2][N_BANK_UNIT];
    uint16_t bad[2][N_BANK_UNIT];

    /* Repairs decided, one line per bank (repair_calc.c or librepairarm). */
    int      (*repair)[REPAIR_WORDS];   /* [510][52]  */
    uint32_t *full;                     /* [14448]    */
    uint32_t *col;                      /* [14448]    */
    uint32_t  row[16];
    uint16_t  row_raw[16];

    /* Bank map of a previous full test (the "level A" trial): NULL in normal
     * operation. See map_load(). */
    uint16_t *map_buf;                /* good, partial, bad : 3 x 12240   */
    const uint16_t *map_good;
    const uint16_t *map_partial;
};

static int otp_allowed(void);
static pthread_mutex_t otp_lock[3][3] = {
    { PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER },
    { PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER },
    { PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER },
};

/* Description of a phase: its patterns, its delays, its name for the log.
 *
 * `trim` is the PER-BLOCK TRIM, and it only exists for Item4 (the margin
 * test left by Sunlune): four 16-bit words per block, pushed into
 * 0xd60-0xd63 just before the block. NULL for all the other phases, which
 * then use the nominal trim dft_normal_lr_trim: original behavior,
 * unchanged. */
struct dft_phase {
    const uint32_t *cmd;
    const uint32_t *timeout;
    const uint16_t *trim;       /* 4 words per block, or NULL = nominal trim */
    unsigned        count;
    const char     *name;
};

/* ------------------------------------------------------------------ */
/* Sending a phase of commands */
/* ------------------------------------------------------------------ */

/*
 * Pushes the blocks of a phase, with the lasting fallback to s2m.
 *
 * The original tries multi_write and, at the first failure, redoes the
 * same block in s2m then remembers the change of regime. The switch holds
 * for the whole rest of the test, following phases included.
 */
static int phase_send(struct dft_run *r, const struct dft_phase *ph)
{
    unsigned k;
    int failed = 0;

    for (k = 0; k < ph->count; k++) {
        const uint16_t *blk = (const uint16_t *)&ph->cmd[k * CMD_STRIDE32];
        uint32_t to = ph->timeout[k];
        /* Per-block trim for Item4, nominal trim everywhere else. */
        const uint16_t *tr = ph->trim ? &ph->trim[k * 4u] : dft_normal_lr_trim;
        int rc;

        if (r->s2m_shared && *r->s2m_shared)
            r->use_s2m = 1;             /* another chip has switched */
        if (r->use_s2m)
            rc = dft_cmd_s2m_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                   (uint16_t *)tr,
                                   (uint16_t *)blk, to);
        else {
            rc = dft_cmd_multi_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                     (uint16_t *)tr,
                                     (uint16_t *)blk, to);
            if (rc < 0) {
                /* The fallback, and it is final. */
                rc = dft_cmd_s2m_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                       (uint16_t *)tr,
                                       (uint16_t *)blk, to);
                r->use_s2m = 1;
                if (r->s2m_shared)
                    *r->s2m_shared = 1;
                jas_trace("board %d asic %u mesh %u: switch to s2m write",
                        r->board, r->asic, r->mesh);
            }
        }

        if (rc < 0) {
            jas_trace("board %d asic %u mesh %u: %s, block %u failed",
                    r->board, r->asic, r->mesh, ph->name, k);
            failed = 1;
        }
    }
    return failed ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* The bank-by-bank survey */
/* ------------------------------------------------------------------ */

/*
 * Goes through the 510 banks of a unit and decides their fate.
 *
 * For each bank still deemed good, it is selected by its DFT identity, a
 * read of the mode registers is started, then dft_get_result_way classifies
 * the result:
 *
 *    3 or <0   nothing to repair, or out of reach
 *    2         only rows are faulty             → repair by rows
 *    <2        columns too                      → full survey, heavier
 *
 * A bank whose count of faulty rows is zero is counted good
 * (good_bank_num_pre). The others have their remapped identity printed,
 * which gives the "pre bad bank" list of the startup log.
 */
static void mrr_scan(struct dft_run *r, unsigned unit, const uint16_t *mrr_cmd)
{
    unsigned b;
    int mode = repair_mode();
    FILE *cap = capture_open(r->board, r->asic, r->mesh);
    unsigned n_full = 0, n_rows = 0, n_different = 0;
    unsigned rf_faithful = 0, rf_kept = 0;

    jas_trace("board %d asic %u mesh %u unit %u: MRR read back",
            r->board, r->asic, r->mesh, unit);

    for (b = 0; b < N_BANK_UNIT; b++) {
        uint16_t id = (uint16_t)dft_id[b];
        uint16_t tag;                   /* the identity as calculate sees it */
        uint32_t row_count = 0;
        int way;

        /*
         * The reset BEFORE the skip: the repair table serves both units of the
         * mesh, and skipping a discarded bank without clearing it would write
         * the repairs computed for the same bank of the other unit into this
         * unit's file (the factory files show the original does exactly that),
         * to be burnt again at each startup.
         */
        memset(r->repair[b], 0, REPAIR_WORDS * sizeof(int));
        if (r->bad[unit][b])
            continue;

        memset(r->row, 0, sizeof(r->row));
        memset(r->row_raw, 0, sizeof(r->row_raw));
        memset(r->full, 0, FULL_WORDS * sizeof(uint32_t));
        memset(r->col, 0, FULL_WORDS * sizeof(uint32_t));

        /*
         * Bit 0x1000 marks unit 1, but ONLY for the repair allocation
         * (librepairarm or repair_calc.c), which it tells which half of the
         * chip the bank comes from. The hardware selection always uses the
         * bare identity: tagging it here would designate another bank than
         * the one we think we are testing.
         */
        tag = unit ? (uint16_t)(id | 0x1000u) : id;

        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, id, 0);

        if (dft_MRR_read(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                         (uint16_t *)dft_mrr_lr_trim, (uint16_t *)mrr_cmd) < 0) {
            jas_trace("board %d asic %u mesh %u unit %u: MRR not answering",
                    r->board, r->asic, r->mesh, unit);
            continue;
        }

        way = dft_get_result_way(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                 &row_count);

        if (row_count == 0)
            __sync_fetch_and_add(&jas_good_bank_num_pre[r->board][r->asic], 1);  /* 4 meshes in parallel */

        /*
         * ORDER OF THE MRR STROBE, as in the original: 0xd2d is released AFTER
         * reading the list of rows (way 2) and BEFORE the full survey (way
         * 0/1). Released earlier, the list of rows comes back at zero and each
         * "rows only" bank gets the repairs of row 0 instead of its own.
         */
        if (way == 3 || way < 0) {
            sl_single_write(ASIC(r->asic), DFT_W(r->mesh), REG_MRR_STROBE, 0, r->board);
            continue;                       /* nothing repairable here */
        }
        if (way != 2)
            sl_single_write(ASIC(r->asic), DFT_W(r->mesh), REG_MRR_STROBE, 0, r->board);

        {
            struct jas_repair_summary summary;
            const uint32_t *input;
            uint32_t n_input;
            int rows_only = (way == 2);

            if (rows_only) {
                /* Rows only: a short survey is enough, read BEFORE releasing
                 * the strobe, like the original. */
                dft_get_row_repair_info(r->board, (uint16_t)r->asic,
                                        (uint16_t)r->mesh, r->row_raw);
                sl_single_write(ASIC(r->asic), DFT_W(r->mesh), REG_MRR_STROBE, 0,
                                r->board);
                dft_conv_otp_row_data(r->row_raw, r->row);
                input = r->row;
                n_input = row_count > 16 ? 16 : row_count;
                n_rows++;
            } else {
                /* Columns too: the full survey, 2064 rows of 14 words. */
                dft_get_full_repair_info(r->board, (uint16_t)r->asic,
                                         (uint16_t)r->mesh, (uint16_t *)r->full);
                memcpy(r->col, r->full, FULL_WORDS * sizeof(uint32_t));
                input = r->col;
                n_input = FULL_WORDS;
                n_full++;
            }
#ifdef JAS_REPAIR_ORIGINAL
            /* REPAIR=original build: librepairarm computes (see repair.h).
             * Our simulator only judges its output, for summary and capture. */
            (void)mode;
            calculate(input, rows_only ? row_count : FULL_WORDS, r->repair[b],
                      REPAIR_WORDS, tag, rows_only, 0);
            memset(&summary, 0, sizeof(summary));
            if (!rows_only) {
                jas_repair_evaluate(input, r->repair[b], tag, &summary.faithful);
                summary.kept = summary.faithful;
            }
#else
            jas_repair_compute_summary(input, rows_only ? row_count : FULL_WORDS,
                                          r->repair[b], tag, rows_only, mode, &summary);
#endif
            if (!rows_only) {
                if (summary.choice != 0)
                    n_different++;
                rf_faithful += (unsigned)summary.faithful.rows;
                rf_kept += (unsigned)summary.kept.rows;
            }
            if (cap) {
                int faithful[REPAIR_WORDS];
                uint32_t header[20];

                memset(faithful, 0, sizeof(faithful));
#ifdef JAS_REPAIR_ORIGINAL
                if (1)
#else
                if (mode == JAS_REPAIR_FAITHFUL)
#endif
                    memcpy(faithful, r->repair[b], sizeof(faithful));
                else
                    jas_repair_compute_mode(input, rows_only ? row_count
                                                 : FULL_WORDS, faithful, tag,
                                                 rows_only, JAS_REPAIR_FAITHFUL);
                header[0] = CAPTURE_MAGIC;  header[1] = 1;  header[2] = unit;  header[3] = b;
                header[4] = tag;            header[5] = (uint32_t)way;
                header[6] = row_count;      header[7] = dft_last_col_count();
                header[8] = n_input;       header[9] = (uint32_t)summary.choice;
                header[10] = (uint32_t)summary.faithful.rows;  header[11] = (uint32_t)summary.faithful.cells;
                header[12] = (uint32_t)summary.kept.rows;  header[13] = (uint32_t)summary.kept.cells;
                header[14] = (uint32_t)mode;
                header[15] = header[16] = header[17] = header[18] = header[19] = 0;
                capture_words(cap, header, 20);
                capture_words(cap, input, n_input);
                capture_words(cap, faithful, REPAIR_WORDS);
                capture_words(cap, r->repair[b], REPAIR_WORDS);
            }
        }
    }
    if (cap)
        fclose(cap);
    jas_log("board %d asic %u mesh %u unit %u: computed survey — %u banks in"
            " full, %u by rows only; mode %d: %u different from the original,"
            " simulated wrong rows %u -> %u",
            r->board, r->asic, r->mesh, unit, n_full, n_rows, mode,
            n_different, rf_faithful, rf_kept);
}


/*
 * The two verification passes, after burning.
 *
 * The first (even banks) ESTABLISHES the verdict: a bank is declared good
 * only if dft_get_result_way reaches the acceptance threshold (3 in the
 * original, "nothing left to repair"; 2 by default here, see mrr_threshold).
 * The second (odd) CONFIRMS it: it can only condemn, never rehabilitate. A
 * bank must therefore pass both.
 *
 * Note that "partial" follows "good" exactly: the original only tells them
 * apart elsewhere; here they go together.
 */
/*
 * ACCEPTANCE THRESHOLD OF THE MRR VERDICT (default 2).
 *
 * The original only accepts way == 3 ("nothing left to repair"). The step
 * below, way == 2, covers banks that keep faulty rows AFTER repair (<= 16,
 * per dft_get_result_way's thresholds). The DFT removes far more banks than
 * the CBSC (about 200 per chip against 1 to 10), so the number of banks,
 * which governs how deep the DAG fills and hence the reject, is decided
 * HERE.
 *
 * Accepting way == 2 is HARMLESS, unlike the CBSC quorum: these banks stay
 * reachable by all 120 ALUs, they only have imperfect rows, so there is no
 * unacknowledged transaction and no possible freeze. The gain depends on
 * the defect profile: on a chip with a bimodal distribution (perfect or
 * unrecoverable, almost nothing in between) it is nil.
 *
 * Set from the settings page: "memory_bank_accept" in the configuration,
 * "tolerant" (way >= 2, the default) or "original" (way == 3). The former
 * key "memory_mrr_accept" is NOT read: older pages wrote "strict" in every
 * saved file, which would silently turn every machine to the original
 * threshold. JASRE_MRR_ACCEPT=<1..3> overrides it for trials; 1 would even
 * accept banks with faulty columns (untested, not advised).
 */
static int mrr_threshold_val;
static pthread_once_t mrr_threshold_once = PTHREAD_ONCE_INIT;

static void mrr_threshold_read(void)
{
    const char *e = getenv("JASRE_MRR_ACCEPT");
    const char *source = "default";

    mrr_threshold_val = 2;
    if (e && *e) {
        char *end;
        long v = strtol(e, &end, 10);

        if (end != e && v >= 1 && v <= 3) {
            mrr_threshold_val = (int)v;
            source = "JASRE_MRR_ACCEPT";
        } else {
            jas_log("DFT: JASRE_MRR_ACCEPT=\"%s\" ignored (1, 2 or 3)", e);
        }
    }
    if (strcmp(source, "default") == 0) {
        FILE *f = fopen(jas_config_file(), "r");
        char line[512];

        if (f) {
            while (fgets(line, sizeof(line), f)) {
                char *p = strstr(line, "\"memory_bank_accept\"");

                if (!p || !(p = strchr(p, ':')))
                    continue;
                if (strstr(p, "\"original\"")) {
                    mrr_threshold_val = 3;
                    source = "settings page";
                } else if (strstr(p, "\"tolerant\"")) {
                    mrr_threshold_val = 2;
                    source = "settings page";
                }
            }
            fclose(f);
        }
    }
    jas_log("DFT: MRR acceptance threshold = way >= %d%s (%s)", mrr_threshold_val,
            mrr_threshold_val == 3 ? " (original)" : "", source);
}

static unsigned mrr_threshold(void)
{
    pthread_once(&mrr_threshold_once, mrr_threshold_read);
    return (unsigned)mrr_threshold_val;
}

static void mrr_verdict(struct dft_run *r, unsigned unit,
                        const uint16_t *mrr_cmd, int establish)
{
    unsigned b;
    FILE *cap = capture_open(r->board, r->asic, r->mesh);
    unsigned threshold = mrr_threshold();
    unsigned tally[5] = { 0, 0, 0, 0, 0 };   /* the "way" distribution */
    /* Statistics on the "out of reach" banks (way -1): a real avalanche of
     * defects, or a saturated counter? Aggregated rather than 200 lines. */
    unsigned hp_n = 0, hp_saturated = 0;
    uint32_t hp_rmin = 0xffffffffu, hp_rmax = 0, hp_cmin = 0xffffffffu, hp_cmax = 0;
    uint32_t hp_ex_r[3] = { 0, 0, 0 }, hp_ex_c[3] = { 0, 0, 0 };

    for (b = 0; b < N_BANK_UNIT; b++) {
        uint32_t row_count = 0;
        int way;

        if (r->bad[unit][b])
            continue;

        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                   (uint16_t)dft_id[b], 0);
        dft_MRR_read(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                     (uint16_t *)dft_mrr_lr_trim, (uint16_t *)mrr_cmd);
        way = dft_get_result_way(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                 &row_count);
        sl_single_write(ASIC(r->asic), DFT_W(r->mesh), REG_MRR_STROBE, 0, r->board);

        tally[(way >= 0 && way <= 4) ? way : 4]++;

        if (cap) {
            unsigned k, nv = 0;

            for (k = 0; k < REPAIR_WORDS; k++)
                nv += r->repair[b][k] < 0;
            if (nv || way != 3) {
                uint32_t header[10];

                header[0] = CAPTURE_MAGIC; header[1] = 2; header[2] = unit; header[3] = b;
                header[4] = (uint32_t)establish; header[5] = (uint32_t)way;
                header[6] = row_count; header[7] = way < 3 ? dft_last_col_count() : 0;
                header[8] = nv; header[9] = 0;
                capture_words(cap, header, 10);
            }
        }

        if (way < 0) {
            uint32_t cols = dft_last_col_count();

            if (hp_n < 3) { hp_ex_r[hp_n] = row_count; hp_ex_c[hp_n] = cols; }
            if (row_count < hp_rmin) hp_rmin = row_count;
            if (row_count > hp_rmax) hp_rmax = row_count;
            if (cols < hp_cmin) hp_cmin = cols;
            if (cols > hp_cmax) hp_cmax = cols;
            /* saturation = all bits at 1 on either counter */
            if (row_count == 0xffffu || cols == 0xffffffffu || cols == 0xffffu)
                hp_saturated++;
            hp_n++;
        }

        if (establish) {
            if (way >= (int)threshold) {
                r->good[unit][b] = 1;
                r->partial[unit][b] = 1;
                r->bad[unit][b] = 0;
                continue;
            }
        } else if (way >= (int)threshold && r->good[unit][b] == 1) {
            continue;                   /* confirmed, left alone */
        }

        r->good[unit][b] = 0;
        r->partial[unit][b] = 0;
        r->bad[unit][b] = 1;
    }

    /* What tells us HOW MANY would be recovered by lowering the threshold. */
    jas_log("board %d asic %u mesh %u unit %u %s: way 0=%u 1=%u 2=%u 3=%u"
            " other=%u (threshold %u)", r->board, r->asic, r->mesh, unit,
            establish ? "establishing" : "confirming",
            tally[0], tally[1], tally[2], tally[3], tally[4], threshold);

    /* And what the "out of reach" ones are REALLY worth: a single line. */
    if (hp_n)
        jas_log("board %d asic %u mesh %u unit %u: %u out of range, rows"
                " %u..%u, cols %u..%u, saturated %u; e.g. %u/%u %u/%u %u/%u",
                r->board, r->asic, r->mesh, unit, hp_n,
                hp_rmin, hp_rmax, hp_cmin, hp_cmax, hp_saturated,
                hp_ex_r[0], hp_ex_c[0], hp_ex_r[1], hp_ex_c[1],
                hp_ex_r[2], hp_ex_c[2]);
    if (cap)
        fclose(cap);
}

/* ------------------------------------------------------------------ */
/* The bank map: reuse the verdicts of a full test */
/* ------------------------------------------------------------------ */

/*
 * "LEVEL A" TRIAL: skip the MRR reads.
 *
 * The test's 24,480 MRR reads (a fixed 10 ms each) take ~4 min of the DFT's
 * ~5, and only READ a verdict the pattern has already locked in. Here
 * EVERYTHING else is kept identical (DP, HB, repairs, post mode,
 * conditioning, Item3 patterns) and the reads are replaced by the verdicts
 * of a previous full test.
 *
 * Two safeguards in the loop itself:
 *   - the banks condemned by the DP/HB tests, which always run, stay
 *     condemned, whatever the map says;
 *   - the LAST bank the full loop would have read is read for real, and
 *     the measurement decides for it: the register state at the end of
 *     the pass (trims, MRR configuration, identity) is that of a full
 *     test, and each pass gives a control of the gap.
 *
 * Control, without touching jasminer.sh: the file /tmp/jasre_map
 * contains "save" (full test, then save) or "use". Absent or
 * other content: original behavior, nothing is read or written. The map
 * lives in /tmp (RAM disk): a power cut erases it.
 */
#define MAP_MODE     "/tmp/jasre_map"
#define MAP_FILE  "/tmp/jasre_map_b%d.bin"
#define MAP_MAGIC    "JBMAP001"

enum { MAP_NONE = 0, MAP_SAVE, MAP_USE };

struct map_header {
    char     magic[8];
    uint32_t header_size;
    uint32_t board;
    uint32_t asic_en;
    uint32_t threshold;             /* mrr_threshold() of the full test */
    uint32_t work_mode;         /* repair directory used */
    uint32_t n;                 /* N_BANK_BOARD                         */
    uint32_t date;              /* time() of the recording */
    uint32_t sum;             /* FNV-1a of the three tables */
};

static int map_mode(void)
{
    char m[32] = "";
    FILE *f = fopen(MAP_MODE, "r");

    if (!f)
        return MAP_NONE;
    if (!fgets(m, sizeof(m), f))
        m[0] = 0;
    fclose(f);
    if (strncmp(m, "save", 4) == 0)
        return MAP_SAVE;
    if (strncmp(m, "use", 3) == 0)
        return MAP_USE;
    return MAP_NONE;
}

static uint32_t map_sum(const uint16_t *a, const uint16_t *b, const uint16_t *c)
{
    const uint16_t *t[3] = { a, b, c };
    uint32_t h = 2166136261u;
    unsigned k, i;

    for (k = 0; k < 3; k++)
        for (i = 0; i < N_BANK_BOARD; i++) {
            h = (h ^ (t[k][i] & 0xffu)) * 16777619u;
            h = (h ^ (t[k][i] >> 8)) * 16777619u;
        }
    return h;
}

/* After a full test: the global tables carry the DFT's verdicts, not yet
 * the CBSC's (crossbar_check comes afterwards). */
static void map_save(int board, int work_mode)
{
    struct map_header e;
    char path[64], tmp[72];
    FILE *f;
    int ok;

    memset(&e, 0, sizeof(e));
    memcpy(e.magic, MAP_MAGIC, 8);
    e.header_size = sizeof(e);
    e.board     = (uint32_t)board;
    e.asic_en   = ASIC_EN(board);
    e.threshold     = mrr_threshold();
    e.work_mode = (uint32_t)work_mode;
    e.n         = N_BANK_BOARD;
    e.date      = (uint32_t)time(NULL);
    e.sum     = map_sum(jas_bank_good[board], jas_bank_partial[board],
                              jas_bank_bad[board]);

    snprintf(path, sizeof(path), MAP_FILE, board);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    f = fopen(tmp, "wb");
    if (!f) {
        jas_log("board %d: bank map NOT saved (%s)", board, tmp);
        return;
    }
    ok = fwrite(&e, sizeof(e), 1, f) == 1
      && fwrite(jas_bank_good[board], sizeof(uint16_t), N_BANK_BOARD, f) == N_BANK_BOARD
      && fwrite(jas_bank_partial[board], sizeof(uint16_t), N_BANK_BOARD, f) == N_BANK_BOARD
      && fwrite(jas_bank_bad[board], sizeof(uint16_t), N_BANK_BOARD, f) == N_BANK_BOARD;
    ok = (fclose(f) == 0) && ok;
    if (!ok || rename(tmp, path) != 0) {
        unlink(tmp);
        jas_log("board %d: bank map NOT saved (write)", board);
        return;
    }
    jas_log("board %d: bank map SAVED (%s, checksum %08x)",
            board, path, e.sum);
}

/* Returns 0 if the map is loaded into r, -1 otherwise (the full test is
 * then run, with no other consequence). */
static int map_load(struct dft_run *r, int work_mode)
{
    struct map_header e;
    char path[64];
    FILE *f;
    uint16_t *buf;
    const char *refusal = NULL;

    snprintf(path, sizeof(path), MAP_FILE, r->board);
    f = fopen(path, "rb");
    if (!f) {
        jas_log("board %d: bank map missing (%s) — full test",
                r->board, path);
        return -1;
    }
    buf = malloc(3u * N_BANK_BOARD * sizeof(uint16_t));
    if (!buf)
        refusal = "memory";
    else if (fread(&e, sizeof(e), 1, f) != 1
          || fread(buf, sizeof(uint16_t), 3u * N_BANK_BOARD, f) != 3u * N_BANK_BOARD)
        refusal = "truncated file";
    else if (memcmp(e.magic, MAP_MAGIC, 8) != 0 || e.header_size != sizeof(e)
          || e.n != N_BANK_BOARD)
        refusal = "format";
    else if (e.board != (uint32_t)r->board || e.asic_en != (uint32_t)ASIC_EN(r->board))
        refusal = "different board or active chips";
    else if (e.threshold != mrr_threshold())
        refusal = "different MRR threshold";
    else if (e.work_mode != (uint32_t)work_mode)
        refusal = "different work mode";
    else if (map_sum(buf, buf + N_BANK_BOARD, buf + 2u * N_BANK_BOARD) != e.sum)
        refusal = "checksum";
    fclose(f);

    if (refusal) {
        jas_log("board %d: bank map REFUSED (%s) — full test",
                r->board, refusal);
        free(buf);
        return -1;
    }
    r->map_buf     = buf;
    r->map_good    = buf;
    r->map_partial = buf + N_BANK_BOARD;
    jas_log("board %d: bank map USED (saved %ld s ago, checksum %08x) — MRR"
            " verdicts not read again, except one witness bank per pass",
            r->board, (long)time(NULL) - (long)e.date, e.sum);
    return 0;
}

/*
 * Replaces mrr_verdict when a map is loaded. Same effects on r->good,
 * r->partial and r->bad as a full test whose verdicts would be the map's;
 * same last hardware read as the full loop.
 */
static void mrr_verdict_map(struct dft_run *r, unsigned unit,
                              const uint16_t *mrr_cmd, int establish)
{
    const uint16_t *remap = unit ? dft_id_2_remap_id_odd : dft_id_2_remap_id_even;
    unsigned base = r->asic * N_BANK_ASIC + (r->mesh * N_BANK_ASIC) / 4;
    unsigned threshold = mrr_threshold();
    unsigned b, good = 0, discarded = 0;
    int last = -1, way = -9, map_says_good = 0;
    uint32_t row_count = 0;

    for (b = 0; b < N_BANK_UNIT; b++) {
        unsigned i = remap[b] + base;

        if (r->bad[unit][b])
            continue;                   /* DP/HB, or discarded at first pass */
        last = (int)b;
        if (!establish) {
            good++;
            continue;                   /* confirmation changes nothing here */
        }
        if (r->map_good[i]) {
            r->good[unit][b]    = 1;
            r->partial[unit][b] = r->map_partial[i];
            r->bad[unit][b]     = 0;
            good++;
        } else {
            r->good[unit][b]    = 0;
            r->partial[unit][b] = 0;
            r->bad[unit][b]     = 1;
            discarded++;
        }
    }

    /* The control bank: read for real, in the exact order of mrr_verdict,
     * and the measurement decides for it. */
    if (last >= 0) {
        b = (unsigned)last;
        map_says_good = r->map_good[remap[b] + base] != 0;
        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                   (uint16_t)dft_id[b], 0);
        dft_MRR_read(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                     (uint16_t *)dft_mrr_lr_trim, (uint16_t *)mrr_cmd);
        way = dft_get_result_way(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                                 &row_count);
        sl_single_write(ASIC(r->asic), DFT_W(r->mesh), REG_MRR_STROBE, 0, r->board);

        if (way < (int)threshold && r->good[unit][b]) {
            r->good[unit][b] = 0;
            r->partial[unit][b] = 0;
            r->bad[unit][b] = 1;
            good--;
            discarded++;
        } else if (establish && way >= (int)threshold && !r->good[unit][b]) {
            /* the full first pass would have declared it good */
            r->good[unit][b] = 1;
            r->partial[unit][b] = 1;
            r->bad[unit][b] = 0;
            good++;
            discarded--;
        }
    }

    jas_log("board %d asic %u mesh %u unit %u %s (map): %u good, %u set aside;"
            " witness bank %d way %d, map %s%s",
            r->board, r->asic, r->mesh, unit, establish ? "establishing" : "confirming",
            good, discarded, last, way, map_says_good ? "good" : "bad",
            (last >= 0 && (way >= (int)threshold) != map_says_good)
                ? " — MISMATCH" : "");
}

/* ------------------------------------------------------------------ */
/* The cache in flash */
/* ------------------------------------------------------------------ */

/*
 * One file per unit, not per mesh: the final index is mesh*2 for unit 0
 * and mesh*2+1 for unit 1. Hence eight files per chip, twenty-four per
 * board: what /media/configs/2/ actually holds on a machine in service.
 */
static void repair_path(char *out, size_t n, int work_mode,
                        int board, unsigned asic, unsigned mesh, unsigned unit)
{
    snprintf(out, n, "%s%d/repair_info_board%d_asic%d_%u",
             config_dir(), work_mode, board, (int)asic, mesh * 2 + unit);
}

/*
 * Reads the repairs back from flash.
 *
 * The original checks neither the length nor the consistency of what it
 * reads back: a truncated file simply leaves the end of the table at zero.
 * We count what we read and say so, without changing the behavior;
 * refusing a short file would be a divergence, and it must be decided,
 * not suffered.
 */
static int repair_info_load(struct dft_run *r, const char *path)
{
    FILE *f = fopen(path, "r");
    unsigned b, k, got = 0;

    if (!f)
        return -1;

    for (b = 0; b < N_BANK_UNIT; b++)
        for (k = 0; k < REPAIR_WORDS; k++)
            if (fscanf(f, "%d", &r->repair[b][k]) == 1)
                got++;
    fclose(f);

    if (got != N_BANK_UNIT * REPAIR_WORDS)
        jas_log("%s: %u values out of %u expected", path, got,
                N_BANK_UNIT * REPAIR_WORDS);
    return 0;
}

static void repair_info_save(const struct dft_run *r, const char *path)
{
    FILE *f = fopen(path, "w");
    unsigned b, k;

    if (!f) {
        jas_log("%s: cannot write", path);
        return;
    }
    for (b = 0; b < N_BANK_UNIT; b++)
        for (k = 0; k < REPAIR_WORDS; k++)
            fprintf(f, "%d\n", r->repair[b][k]);
    fclose(f);
}

/*
 * Burns the repairs into the one-time programmable memory.
 *
 * Only negative entries are written: the sign bit marks a slot actually to
 * be programmed. Irreversible if it really is one-time memory; see the
 * doubts noted at the OTP guard below.
 */
static void otp_program(struct dft_run *r)
{
    unsigned b, k;

    if (!otp_allowed()) {
        unsigned n = 0;
        for (b = 0; b < N_BANK_UNIT; b++)
            for (k = 0; k < REPAIR_WORDS; k++)
                if (r->repair[b][k] < 0)
                    n++;
        jas_trace("board %d asic %u mesh %u: %u entries would have been burned",
                r->board, r->asic, r->mesh, n);
        return;
    }

    jas_trace("board %d asic %u mesh %u: otp program",
            r->board, r->asic, r->mesh);

    dft_set_bist_l(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0);
    for (b = 0; b < N_BANK_UNIT; b++) {
        dft_set_otp_mode(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 1);
        for (k = 0; k < REPAIR_WORDS; k++)
            if (r->repair[b][k] < 0)
                dft_write_otp(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                              (uint16_t *)&r->repair[b][k]);
        dft_set_otp_mode(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0);
    }
    dft_set_bist_l(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 1);
}


/* ------------------------------------------------------------------ */
/* Work mode and burn safeguard */
/* ------------------------------------------------------------------ */

/*
 * OTP BURN.
 *
 * otp_program applies the computed repairs, as the firmware does, in both
 * paths: cache read back or fresh survey. With the cache present, the
 * values come from the same files as the firmware's, so nothing new is
 * written; with the cache absent, our own survey produces them, and an
 * error in it would go into the chip.
 *
 * Three elements suggest that the operation is not fuse burning but the
 * loading of redundancy registers: the firmware redoes it at each startup,
 * reads it from a cache in flash, and there are three mutually exclusive
 * sets of repairs (/media/configs/1, /2, /3) that could not be burnt
 * together. It is not a proof.
 *
 * JASRE_INHIBIT_OTP=1 makes the function silent: it only announces what
 * it would have written (for developing the survey path itself).
 */
static int otp_allowed(void)
{
    static int cached = -1;
    const char *e;

    if (cached >= 0)
        return cached;
    e = getenv("JASRE_INHIBIT_OTP");
    cached = (e && *e == '1') ? 0 : 1;
    if (!cached)
        jas_trace("otp program inhibited by JASRE_INHIBIT_OTP");
    return cached;
}

/*
 * Determines the work mode and decides whether the test must be redone
 * from scratch.
 *
 * Three pattern sets coexist, numbered 1 to 3, each with its repair
 * directory under /media/configs/. The file work_pattern.conf designates
 * the one in force; without it the firmware picks 2.
 *
 * The pre-test flag is raised in two cases: a type 3 chip, and a mode
 * directory that did not exist yet. In both cases the cache is ignored and
 * the full survey redone.
 */
static int dft_work_mode_init(void)
{
    char path[256];
    int work_mode = 2;
    FILE *f;

    if (jas_asic_type() == 3) {
        *jas_dft_pretest_flag = 1;
        return work_mode;
    }

    snprintf(path, sizeof(path), "%swork_pattern.conf", config_dir());
    f = (access(path, F_OK) == 0) ? fopen(path, "r") : NULL;
    if (!f) {
        jas_log("%s: missing, work mode 2 by default", path);
    } else {
        if (fscanf(f, "%d", &work_mode) != 1)
            work_mode = 2;
        fclose(f);
        jas_log("work mode %d", work_mode);
    }

    snprintf(path, sizeof(path), "%s%d", config_dir(), work_mode);
    if (access(path, F_OK) != 0) {
        char cmd[300];

        jas_trace("%s: missing, creating and full survey", path);
        snprintf(cmd, sizeof(cmd), "mkdir %s", path);
        if (system(cmd) != 0)
            jas_log("%s: cannot create", path);
        *jas_dft_pretest_flag = 1;
    }
    return work_mode;
}

/* ------------------------------------------------------------------ */
/* The two preliminary tests */
/* ------------------------------------------------------------------ */

/* Marks a bank condemned in both units at once. */
static void condemn_both(struct dft_run *r, unsigned b)
{
    unsigned u;

    for (u = 0; u < 2; u++) {
        r->good[u][b] = 0;
        r->partial[u][b] = 0;
        r->bad[u][b] = 1;
    }
}

/*
 * Data path: the internal link between the compute node and the bank.
 *
 * A failure here condemns the bank in BOTH units: the path is shared.
 */
static void datapath_test(struct dft_run *r, uint32_t timeout)
{
    unsigned b;

    dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0, 0xfff);
    dft_cmd_multi_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                        (uint16_t *)dft_normal_lr_trim,
                        (uint16_t *)dft_DataPathtest_cmd, timeout);

    for (b = 0; b < N_BANK_UNIT; b++) {
        uint16_t id = (uint16_t)dft_id[b];

        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, id, 0);
        if (dft_read_DP_info(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                             (uint16_t *)dft_normal_lr_trim,
                             (uint16_t *)dft_read_DP_info_cmd,
                             (uint16_t *)dft_end_DP_info_cmd) < 0) {
            jas_trace("board %d asic %u: datapath %u failed",
                    r->board, r->asic, (unsigned)id + r->mesh * N_BANK_ASIC / 8);
            condemn_both(r, b);
            jas_dp_fail_num[r->board][r->asic]++;
        }
    }
}

/*
 * Hybrid bond: the bonding between the two stacked halves of the chip.
 *
 * Tested separately for each unit, since each half has its own bond.
 */
static void hb_test(struct dft_run *r, unsigned unit, uint32_t timeout)
{
    const uint32_t *test_cmd = unit ? dft_Unit1_Hbtest_cmd : dft_Unit0_Hbtest_cmd;
    const uint16_t *info_cmd = unit ? dft_Unit1_read_HB_info_cmd
                                    : dft_Unit0_read_HB_info_cmd;
    unsigned b;

    dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0, 0xfff);
    if (dft_cmd_multi_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                            (uint16_t *)dft_normal_lr_trim,
                            (uint16_t *)test_cmd, timeout) < 0) {
        dft_cmd_s2m_write(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                          (uint16_t *)dft_normal_lr_trim,
                          (uint16_t *)test_cmd, timeout);
        r->use_s2m = 1;
        if (r->s2m_shared)
            *r->s2m_shared = 1;
        jas_trace("board %d asic %u mesh %u: switch to s2m write (hybrid link)",
                r->board, r->asic, r->mesh);
    }

    for (b = 0; b < N_BANK_UNIT; b++) {
        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                   (uint16_t)dft_id[b], 0);
        if (dft_read_HB_info(r->board, (uint16_t)r->asic, (uint16_t)r->mesh,
                             (uint16_t *)dft_normal_lr_trim,
                             (uint16_t *)info_cmd) < 0) {
            jas_trace("board %d asic %u: unit %u bank %u, hybrid link failed",
                    r->board, r->asic, unit, b);
            r->good[unit][b] = 0;
            r->partial[unit][b] = 0;
            /*
             * NO `r->bad[unit][b] = 1` here, as in the miners in service: the
             * stock runtime's loader neutralized exactly this write, at the two
             * places of the original function (0x2b722 and 0x2b83c, a `strh.w`
             * replaced by `nop`):
             *
             *     bank_bad_record_flag_dft_unit0[i] = 1;   removed
             *     bank_bad_record_flag_dft_unit1[i] = 1;   removed
             *
             * A hybrid bond failure thus removes the bank from "good" and
             * "partial" without CONDEMNING it; the MRR probe decides
             * (memory_hb_policy=mrr_probe, memory_hb_repair_filter=off). The
             * counter is kept.
             */
            jas_hb_fail_num[r->board][r->asic]++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* The complete sequence of a unit */
/* ------------------------------------------------------------------ */

/*
 * A unit goes three times through the same initial → Item3 → MRR
 * triptych: once before the OTP burn ("pre"), twice after, on the even
 * then odd banks. The second pass only makes sense after the burn: it is
 * the one that checks that the redundancies took.
 */
static int unit_sequence(struct dft_run *r, unsigned unit, int work_mode)
{
    char path[192];
    /* MANDATORY ZEROING: the fields are filled one by one below, and `trim`
     * (added for Item4) never is on these phases. Without this
     * initialization it would hold anything and phase_send would dereference
     * dirty stack, a defect the compiler does not report. */
    struct dft_phase ph = { 0 };

    const uint32_t *initial_pre  = unit ? dft_Unit1_initial_pre_cmd
                                        : dft_Unit0_initial_pre_cmd;
    const uint32_t *initial_post = unit ? dft_Unit1_initial_post_cmd
                                        : dft_Unit0_initial_post_cmd;
    const uint32_t *item3_pre    = unit ? dft_Unit1_Item3_pre_cmd
                                        : dft_Unit0_Item3_pre_cmd;
    const uint32_t *item3_even   = unit ? dft_Unit1_Item3_post_even_cmd
                                        : dft_Unit0_Item3_post_even_cmd;
    const uint32_t *item3_odd    = unit ? dft_Unit1_Item3_post_odd_cmd
                                        : dft_Unit0_Item3_post_odd_cmd;
    const uint16_t *mrr_pre      = unit ? dft_Unit1_MRR_read_pre_cmd
                                        : dft_Unit0_MRR_read_pre_cmd;
    const uint16_t *mrr_even     = unit ? dft_Unit1_MRR_read_post_even_cmd
                                        : dft_Unit0_MRR_read_post_even_cmd;
    const uint16_t *mrr_odd      = unit ? dft_Unit1_MRR_read_post_odd_cmd
                                        : dft_Unit0_MRR_read_post_odd_cmd;

    /*
     * ---- before burning: ONLY if the cache is absent ----
     *
     * The whole survey (conditioning, Item3 pattern, MRR read bank by bank)
     * lives inside this branch. On a machine in service the file exists and
     * the firmware skips it all: hence its good_bank_num_pre at zero and a
     * self-test of a few tens of seconds per chip. Running it anyway would
     * measure a memory already repaired by the OTP of previous startups, and
     * the "pre" count would mean nothing.
     */
    repair_path(path, sizeof(path), work_mode, r->board, r->asic, r->mesh, unit);

    /* The map only holds for repairs READ BACK: a fresh survey wants fresh
     * verdicts. */
    int map_here = 0;

    if (access(path, F_OK) == 0 && !*jas_dft_pretest_flag) {
        jas_trace("%s: repairs reloaded from flash", path);
        repair_info_load(r, path);
        map_here = r->map_good != NULL;
    } else {
        jas_trace("%s: missing, full survey from the asic", path);

        dft_set_pre_post_mode(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0);
        dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0, 0xfff);

        ph.cmd = initial_pre; ph.timeout = dft_initial_pre_timeout;
        ph.count = N_INITIAL; ph.name = "initial_pre";
        if (phase_send(r, &ph) < 0) {
            /* A failed conditioning condemns the whole mesh: nothing could be
             * concluded from what would follow. */
            close_mesh_bank((uint16_t)r->mesh, (uint16_t)r->asic, r->board);
            return -1;
        }

        ph.cmd = item3_pre; ph.timeout = dft_Item3_pre_timeout;
        ph.count = N_ITEM3; ph.name = "Item3_pre";
        if (phase_send(r, &ph) < 0)
            return -1;

        mrr_scan(r, unit, mrr_pre);
        repair_info_save(r, path);
    }

    /* ---- burn, then verification ---- */
    /* Meshes in parallel: loading the repairs stays one chip at a time
     * (short; caution on a shared internal resource). */
    pthread_mutex_lock(&otp_lock[r->board % 3][r->asic % 3]);
    otp_program(r);
    pthread_mutex_unlock(&otp_lock[r->board % 3][r->asic % 3]);


    dft_set_pre_post_mode(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 1);
    dft_set_id(r->board, (uint16_t)r->asic, (uint16_t)r->mesh, 0, 0xfff);

    ph.cmd = initial_post; ph.timeout = dft_initial_post_timeout;
    ph.count = N_INITIAL; ph.name = "initial_post";
    if (phase_send(r, &ph) < 0)
        return -1;

    ph.cmd = item3_even; ph.timeout = dft_Item3_post_timeout;
    ph.count = N_ITEM3; ph.name = "Item3_post_even";
    if (phase_send(r, &ph) < 0)
        return -1;
    if (map_here)
        mrr_verdict_map(r, unit, mrr_even, 1);
    else
        mrr_verdict(r, unit, mrr_even, 1);   /* establishes */

    ph.cmd = initial_post; ph.timeout = dft_initial_post_timeout;
    ph.count = N_INITIAL; ph.name = "initial_post";
    if (phase_send(r, &ph) < 0)
        return -1;

    ph.cmd = item3_odd; ph.timeout = dft_Item3_post_timeout;
    ph.count = N_ITEM3; ph.name = "Item3_post_odd";
    if (phase_send(r, &ph) < 0)
        return -1;
    if (map_here)
        mrr_verdict_map(r, unit, mrr_odd, 0);
    else
        mrr_verdict(r, unit, mrr_odd, 0);    /* confirms */

    return 0;
}

/* ------------------------------------------------------------------ */
/* Carrying the verdicts over to the global space */
/* ------------------------------------------------------------------ */

/*
 * Scatters the mesh's 2 × 510 verdicts over the chip's 4080 banks.
 *
 * The two units interleave: dft_id_2_remap_id_even places the banks of unit
 * 0, _odd those of unit 1. The mesh's offset is mesh*1020, written here as
 * (mesh * 4080) / 4 to stay as close as possible to the original.
 */
static void scatter_verdicts(const struct dft_run *r)
{
    unsigned base = r->asic * N_BANK_ASIC + (r->mesh * N_BANK_ASIC) / 4;
    unsigned b;

    for (b = 0; b < N_BANK_UNIT; b++) {
        unsigned i0 = dft_id_2_remap_id_even[b] + base;
        unsigned i1 = dft_id_2_remap_id_odd[b]  + base;

        jas_bank_good[r->board][i0]    = r->good[0][b];
        jas_bank_partial[r->board][i0] = r->partial[0][b];
        jas_bank_bad[r->board][i0]     = r->bad[0][b];

        jas_bank_good[r->board][i1]    = r->good[1][b];
        jas_bank_partial[r->board][i1] = r->partial[1][b];
        jas_bank_bad[r->board][i1]     = r->bad[1][b];
    }
}

/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* A dft_run: allocation and release */
/* ------------------------------------------------------------------ */

static struct dft_run *dft_run_new(int board)
{
    struct dft_run *r = calloc(1, sizeof(*r));

    if (!r)
        return NULL;
    r->repair = calloc(N_BANK_UNIT, sizeof(*r->repair));
    r->full   = calloc(FULL_WORDS, sizeof(uint32_t));
    r->col    = calloc(FULL_WORDS, sizeof(uint32_t));
    if (!r->repair || !r->full || !r->col) {
        free(r->repair); free(r->full); free(r->col); free(r);
        return NULL;
    }
    r->board = board;
    return r;
}

static void dft_run_free(struct dft_run *r)
{
    if (!r)
        return;
    free(r->map_buf);
    free(r->repair); free(r->full); free(r->col); free(r);
}

/*
 * THE TEST OF A MESH: its two units, its verdicts. Returns 1 if it
 * completed. Body shared by the three modes (sequential, one thread per
 * chip, one thread per mesh).
 */
static int dft_mesh(struct dft_run *r, unsigned asic, unsigned mesh, int work_mode,
                    uint32_t hb_timeout)
{
    int thread_id = r->board;
    unsigned u;
    int ok = 1;

    r->asic = asic;
    r->mesh = mesh;

    /* Each mesh starts again from a blank slate. */
    memset(r->good, 0, sizeof(r->good));
    memset(r->partial, 0, sizeof(r->partial));
    memset(r->bad, 0, sizeof(r->bad));

    sl_single_write(ASIC(asic), DFT_W(mesh), REG_FULL_STROBE, 0, thread_id);
    sl_single_write(ASIC(asic), DFT_W(mesh), REG_MRR_STROBE, 0, thread_id);
    sl_single_write(ASIC(asic), DFT_W(mesh), REG_SLOT_STROBE, 0, thread_id);
    sl_single_write(ASIC(asic), DFT_W(mesh), REG_BIST_STROBE, 0, thread_id);
    dft_set_bist_l(thread_id, (uint16_t)asic, (uint16_t)mesh, 1);

    datapath_test(r, hb_timeout);
    hb_test(r, 0, hb_timeout);
    hb_test(r, 1, hb_timeout);

    /*
     * A failure along the way skips the carry-over: the original jumps there
     * with a goto that steps over the scattering of the verdicts. The mesh's
     * banks then keep what close_mesh_bank or the initialization left them;
     * writing nothing is a choice here, not an oversight.
     */
    for (u = 0; u < 2 && ok; u++)
        if (unit_sequence(r, u, work_mode) < 0)
            ok = 0;

    if (ok) {
        dft_set_bist_l(thread_id, (uint16_t)asic, (uint16_t)mesh, 0);
        scatter_verdicts(r);
    }
    return ok;      /* 0: this mesh keeps initialization verdicts */
}

/* A chip's balance, once its four meshes are done. */
static void dft_summary_chip(int thread_id, unsigned asic, int all_ok)
{
    unsigned b;

    for (b = 0; b < N_BANK_ASIC; b++)
        if (jas_bank_good[thread_id][asic * N_BANK_ASIC + b])
            jas_good_bank_num_post[thread_id][asic]++;

    jas_log("board %d asic %u: %u good banks before, %u after",
            thread_id, asic,
            jas_good_bank_num_pre[thread_id][asic],
            jas_good_bank_num_post[thread_id][asic]);
}

/*
 * THE TEST OF A CHIP: its four meshes one after the other, then its
 * balance. Returns 1 if all the meshes completed.
 *
 * The same body serves the sequential mode (a single dft_run chains the
 * three chips, like the original) and the "one thread per chip" mode. Each
 * chip sees EXACTLY the same sequence of commands; only the interleaving
 * changes.
 */
static int dft_chip(struct dft_run *r, unsigned asic, int work_mode, uint32_t hb_timeout)
{
    unsigned mesh;
    int all_ok = 1;

    for (mesh = 0; mesh < N_MESH; mesh++)
        if (!dft_mesh(r, asic, mesh, work_mode, hb_timeout))
            all_ok = 0;
    dft_summary_chip(r->board, asic, all_ok);
    return all_ok;
}

/*
 * PARALLEL DFT: ONE THREAD PER CHIP.
 *
 * Possible because each serial bus access takes and releases a global
 * recursive lock (jas_bus_lock), and the 10 ms wait of each MRR read is a
 * nanosleep taken OUTSIDE the lock: while one chip waits for its verdict,
 * the others use the bus. No "all chips" write in a chip's body (dft_reset,
 * global, is done once before, by self_check). All the state is per chip
 * (dft_run, slots of jas_bank_* and of the counters); the only global of
 * the path (last column count, diagnostic) is thread-local; the s2m switch
 * is shared.
 *
 * Each chip sees the same commands, waits and reads as in sequence (unlike
 * level A, nothing is removed); the "way" counts per (chip, mesh, unit) and
 * the DFT totals must match the sequential mode. The factory firmware (one
 * thread per BOARD, a single board on the X16-Q) never tests two chips at
 * once.
 *
 * Now the first fallback of the per-mesh mode below:
 * JASRE_DFT_PARALLEL=chip or the file /tmp/jasre_dft_chip select it.
 * Back to the original sequential mode: JASRE_DFT_PARALLEL=0, or the
 * file /tmp/jasre_dft_sequential (no need to touch jasminer.sh).
 */
#define DFT_THREAD_STACK           (1024u * 1024u)   /* jas_repair_compute() takes ~85 KB of stack */

/*
 * ONE THREAD PER MESH: THE DEFAULT (about 30 s per board instead of ~100 s
 * with one thread per chip and ~300 s in sequence). The 12 DFT engines
 * (3 chips x 4 meshes) work together; each command targets a single
 * (chip, mesh). OTP loading stays one chip at a time (otp_lock), a chip's
 * "before" counter is incremented atomically, and a chip's balance is made
 * after its four meshes. The factory firmware never tests two meshes of the
 * same chip at once.
 */
#define DFT_CHIP_FILE "/tmp/jasre_dft_chip"
#define DFT_SEQUENTIAL_FILE "/tmp/jasre_dft_sequential"

static int dft_per_mesh_request(void)
{
    const char *e = getenv("JASRE_DFT_PARALLEL");

    /* THE DEFAULT (balances identical to the sequential mode).
     * JASRE_DFT_PARALLEL=chip (one thread per chip) or =0 (sequential),
     * or the files /tmp/jasre_dft_chip and /tmp/jasre_dft_sequential, to
     * go back. */
    if (e && *e)
        return strcmp(e, "mesh") == 0;
    return access(DFT_CHIP_FILE, F_OK) != 0 && access(DFT_SEQUENTIAL_FILE, F_OK) != 0;
}

struct dft_mesh_thread {
    struct dft_run *r;
    unsigned        asic, mesh;
    int             work_mode;
    uint32_t        hb_timeout;
    int             ok;
};

static void *dft_mesh_thread(void *arg)
{
    struct dft_mesh_thread *f = arg;

    f->ok = dft_mesh(f->r, f->asic, f->mesh, f->work_mode, f->hb_timeout);
    return NULL;
}

static int dft_parallel_request(void)
{
    const char *e = getenv("JASRE_DFT_PARALLEL");

    /* Used when the per-mesh mode is not (balances identical to the
     * sequential mode). JASRE_DFT_PARALLEL=0 or the file
     * /tmp/jasre_dft_sequential bring back the original sequential mode. */
    if (e && *e)
        return *e != '0';
    return access(DFT_SEQUENTIAL_FILE, F_OK) != 0;
}

struct dft_thread {
    struct dft_run *r;
    unsigned        asic;
    int             work_mode;
    uint32_t        hb_timeout;
    int             ok;
};

static void *dft_chip_thread(void *arg)
{
    struct dft_thread *f = arg;

    f->ok = dft_chip(f->r, f->asic, f->work_mode, f->hb_timeout);
    return NULL;
}

int dft_short_pattern_test_110(int thread_id)
{
    struct dft_run *r;
    unsigned asic, reg, k;
    uint32_t hb_timeout;
    int map_mode_now, all_ok = 1;
    int work_mode;
    struct timespec t0, t1;

    jas_log("board %d: short dft test", thread_id);
    clock_gettime(CLOCK_MONOTONIC, &t0);

    r = dft_run_new(thread_id);
    if (!r) { jas_log("dft test: not enough memory"); return -1; }
    {
        const char *e = getenv("JASRE_MC_TRIM");

        if (e && *e) {
            unsigned long v = strtoul(e, NULL, 0);

            jas_mc_trim[0] = (uint16_t)(v >> 16);
            jas_mc_trim[1] = (uint16_t)(v & 0xffffu);
            jas_log("board %d: mc_trim (0x1c0c) forced to %lu", thread_id, v);
        }
    }
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_MC_TRIM,
                   jas_mc_trim, 2, thread_id);
    jas_log("board %d: mc_trim (0x1c0c) = %u", thread_id,
            ((unsigned)jas_mc_trim[0] << 16) | jas_mc_trim[1]);
    for (reg = REG_TRIM_FIRST; reg < REG_TRIM_LAST; reg++)
        sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, (uint16_t)reg, 0, thread_id);
    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_TRIM_0, 0x300, thread_id);
    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_TRIM_APPLY, 1, thread_id);

    /* Everything is good until proven otherwise, "partial" included, which
     * the test will only remove. */
    for (k = 0; k < ASIC_NUM * N_BANK_ASIC; k++) {
        jas_bank_good[thread_id][k]    = 1;
        jas_bank_partial[thread_id][k] = 1;
        jas_bank_bad[thread_id][k]     = 0;
    }

    work_mode  = dft_work_mode_init();
    hb_timeout = dft_Hbtest_timeout[0];

    map_mode_now = map_mode();
    if (map_mode_now == MAP_USE)
        map_load(r, work_mode);    /* failure: full test, nothing else */

    if (dft_per_mesh_request() && map_mode_now == MAP_NONE) {
        struct dft_mesh_thread fm[ASIC_NUM][N_MESH];
        pthread_t tm[ASIC_NUM][N_MESH];
        int started[ASIC_NUM][N_MESH], ok_chip[ASIC_NUM];
        volatile int s2m = 0;
        pthread_attr_t at;
        unsigned n = 0, mesh;

        (void)mrr_threshold();
        (void)otp_allowed();
        pthread_attr_init(&at);
        pthread_attr_setstacksize(&at, DFT_THREAD_STACK);
        memset(started, 0, sizeof(started));
        memset(fm, 0, sizeof(fm));

        for (asic = 0; asic < ASIC_NUM; asic++) {
            ok_chip[asic] = 1;
            if (!(ASIC_EN(thread_id) >> asic & 1))
                continue;
            for (mesh = 0; mesh < N_MESH; mesh++) {
                struct dft_mesh_thread *f = &fm[asic][mesh];

                f->r = (asic == 0 && mesh == 0) ? r : dft_run_new(thread_id);
                f->asic = asic;
                f->mesh = mesh;
                f->work_mode = work_mode;
                f->hb_timeout = hb_timeout;
                if (!f->r)
                    continue;
                f->r->s2m_shared = &s2m;
                if (pthread_create(&tm[asic][mesh], &at, dft_mesh_thread, f) == 0) {
                    started[asic][mesh] = 1;
                    n++;
                }
            }
        }
        pthread_attr_destroy(&at);
        jas_log("board %d: DFT IN PARALLEL PER MESH, %u threads", thread_id, n);

        for (asic = 0; asic < ASIC_NUM; asic++)
            for (mesh = 0; mesh < N_MESH; mesh++)
                if (started[asic][mesh]) {
                    pthread_join(tm[asic][mesh], NULL);
                    if (!fm[asic][mesh].ok)
                        ok_chip[asic] = 0;
                }
        /* What did not get a thread runs now, in sequence. */
        for (asic = 0; asic < ASIC_NUM; asic++) {
            if (!(ASIC_EN(thread_id) >> asic & 1))
                continue;
            for (mesh = 0; mesh < N_MESH; mesh++) {
                struct dft_mesh_thread *f = &fm[asic][mesh];

                if (started[asic][mesh])
                    continue;
                if (!f->r) {
                    f->r = r;
                    r->s2m_shared = &s2m;
                }
                if (!dft_mesh(f->r, asic, mesh, work_mode, hb_timeout))
                    ok_chip[asic] = 0;
            }
            dft_summary_chip(thread_id, asic, ok_chip[asic]);
            if (!ok_chip[asic])
                all_ok = 0;
        }
        for (asic = 0; asic < ASIC_NUM; asic++)
            for (mesh = 0; mesh < N_MESH; mesh++)
                if (fm[asic][mesh].r && fm[asic][mesh].r != r)
                    dft_run_free(fm[asic][mesh].r);
        r->s2m_shared = NULL;
    } else if (dft_parallel_request() && map_mode_now == MAP_NONE) {
        struct dft_thread threads[ASIC_NUM];
        struct dft_run *rp[ASIC_NUM];
        pthread_t tid[ASIC_NUM];
        int started[ASIC_NUM];
        volatile int s2m = 0;
        pthread_attr_t at;
        unsigned n = 0;

        /* The two environment-read caches, initialized BEFORE the threads
         * (otherwise three concurrent first reads). */
        (void)mrr_threshold();
        (void)otp_allowed();

        pthread_attr_init(&at);
        pthread_attr_setstacksize(&at, DFT_THREAD_STACK);
        memset(started, 0, sizeof(started));
        memset(rp, 0, sizeof(rp));

        for (asic = 0; asic < ASIC_NUM; asic++) {
            if (!(ASIC_EN(thread_id) >> asic & 1))
                continue;
            rp[asic] = (asic == 0) ? r : dft_run_new(thread_id);
            threads[asic].r = rp[asic];
            threads[asic].asic = asic;
            threads[asic].work_mode = work_mode;
            threads[asic].hb_timeout = hb_timeout;
            threads[asic].ok = 0;
            if (!rp[asic]) {
                jas_log("board %d asic %u: parallel DFT, not enough memory —"
                        " this chip will go alone after the others", thread_id, asic);
                continue;
            }
            rp[asic]->s2m_shared = &s2m;
            if (pthread_create(&tid[asic], &at, dft_chip_thread, &threads[asic]) == 0) {
                started[asic] = 1;
                n++;
            } else
                jas_log("board %d asic %u: DFT thread impossible — this chip"
                        " will go alone after the others", thread_id, asic);
        }
        pthread_attr_destroy(&at);
        jas_log("board %d: DFT IN PARALLEL, %u chip(s) in their own thread", thread_id, n);

        for (asic = 0; asic < ASIC_NUM; asic++)
            if (started[asic]) {
                pthread_join(tid[asic], NULL);
                if (!threads[asic].ok)
                    all_ok = 0;
            }
        /* The chips that did not get a thread run now, in sequence. */
        for (asic = 0; asic < ASIC_NUM; asic++) {
            if (!(ASIC_EN(thread_id) >> asic & 1) || started[asic])
                continue;
            if (!rp[asic]) {
                rp[asic] = r;           /* the starting dft_run is free */
                r->s2m_shared = &s2m;
            }
            if (!dft_chip(rp[asic], asic, work_mode, hb_timeout))
                all_ok = 0;
        }
        for (asic = 1; asic < ASIC_NUM; asic++)
            if (rp[asic] && rp[asic] != r)
                dft_run_free(rp[asic]);
        r->s2m_shared = NULL;
    } else {
        if (dft_parallel_request())
            jas_log("board %d: parallel DFT requested but map mode active —"
                    " sequential", thread_id);
        for (asic = 0; asic < ASIC_NUM; asic++) {
            if (!(ASIC_EN(thread_id) >> asic & 1))
                continue;
            if (!dft_chip(r, asic, work_mode, hb_timeout))
                all_ok = 0;
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    jas_log("board %d: DFT done in %ld s", thread_id, (long)(t1.tv_sec - t0.tv_sec));

    if (map_mode_now == MAP_SAVE) {
        if (all_ok)
            map_save(thread_id, work_mode);
        else
            jas_log("board %d: bank map NOT saved (a mesh failed, its verdicts"
                    " are not measured)", thread_id);
    }

    dft_run_free(r);
    return 0;
}

/*
 * Condemns all 1020 banks of a mesh at once.
 *
 * Called when the conditioning of a mesh fails: nothing can then be
 * concluded about its banks, and the original chooses to discard them all
 * rather than leave them in an undetermined state. It is cautious, but
 * costly: a quarter of a chip's memory goes with it.
 *
 * The order of the parameters is the original's, mesh before asic.
 */
void close_mesh_bank(uint16_t mesh, uint16_t asic, int thread_id)
{
    unsigned base = thread_id * N_BANK_BOARD + mesh * N_BANK_MESH
                  + asic * N_BANK_ASIC;
    unsigned k;

    for (k = 0; k < N_BANK_MESH; k++) {
        jas_bank_good[thread_id][base + k]    = 0;
        jas_bank_partial[thread_id][base + k] = 0;
        jas_bank_bad[thread_id][base + k]     = 1;
    }
}
