/*
 * Building the cache and the DAG in the ASICs' memory.
 *
 * The original assumes a single coin for the whole board, and this
 * assumption is in its data structure (fpga_data.cache_size, .seed,
 * .dag_size: one of each). The cache placement in a chip's memory is
 * computed per chip (cache_init_addr_g) but from the GLOBAL cache size:
 *
 *   cache_start_row = 0xff6 - cache_size / (valid_banks << 9)
 *   cache_init_addr = (cache_start_row << 9) * valid_banks
 *
 * so one chip cannot build another coin's cache. Here the coin context is
 * PER CHIP (jas_coin_ctx): sizes, seed, placement and time budget all
 * follow from it. The firmware already handles a reduced DAG size per chip
 * (dag_len_tmp), just never a distinct seed or cache.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "jas_ethash.h"
#include "jas_coin.h"
#include "sl_transfer.h"
#include "jas_ethash.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define BANKS_PER_ASIC   0xff0u
#define BANK_BYTES       0x200000u   /* 2 MiB per bank */

#define REG_CACHE_SIZE   0x04
#define REG_DAG_SIZE     0x08
#define REG_SEED         0x30
#define REG_CACHE_START  0x300       /* starts the cache computation */
#define REG_CACHE_DONE   0x301
#define REG_CACHE_ADDR   0x304       /* where to place the cache */
#define REG_DAG_START    0x302
#define REG_DAG_DONE     0x303
#define REG_SOFT_RESET   0x830
#define REG_ALU_EN       0x40    /* ALU enable mask, per mesh */
#define CMD_BROADCAST    0xf8
#define CMD_MESH0        0x18
#define CMD_MESH0_READ   0x1c

/*
 * How many seconds to grant a build: derived from the size of what is
 * built, as in the original, but from the CHIP's size, not the global one.
 */
static unsigned cache_timeout_s(uint64_t cache_size)
{
    return (unsigned)(cache_size >> 23) + 5;
}

static unsigned dag_timeout_s(uint64_t dag_size)
{
    return ((unsigned)(dag_size >> 30) + 1) * 10;
}

/*
 * Placement of the cache in a chip's memory: at the top of the address
 * space, the DAG growing below. The position depends on the number of
 * usable banks AND on the cache size, which must be that of THIS CHIP'S
 * coin, or the computed cache and the reserved place do not match.
 */
uint64_t dag_cache_init_addr(unsigned valid_banks, uint64_t cache_size)
{
    uint32_t start_row;

    if (valid_banks == 0)
        return 0xdacu;                       /* the original's fallback value */

    start_row = 0xff6u - (uint32_t)(cache_size / ((uint64_t)valid_banks << 9));
    return (uint64_t)(start_row << 9) * valid_banks;
}

/* Really addressable memory capacity, in bytes. */
/*
 * A chip's capacity (asic_max_capacity_g, "wtf" in the log): MEASURED, not
 * deduced. The firmware publishes 0x3FC000000 (4080 x 4 MiB) for every
 * chip, whatever its number of valid banks, while our BANK_BYTES is 2 MiB;
 * the factor of two is unexplained, so the measured value is used as is.
 * Computing (4080 - bad) x 2 MiB instead would cap every DAG at half its
 * size, since dag_calc caps the DAG by the capacity.
 */
#define ASIC_CAPACITY_MEASURED  0x3FC000000ull

uint64_t dag_asic_capacity(unsigned bad_banks)
{
    (void)bad_banks;
    return ASIC_CAPACITY_MEASURED;
}

/* Counts the non-zero entries of a bank state table, for one chip. */
static unsigned count_banks(const uint16_t *flags, unsigned asic)
{
    unsigned base = asic * BANKS_PER_ASIC, k, n = 0;

    for (k = 0; k < BANKS_PER_ASIC; k++)
        if (flags[base + k])
            n++;
    return n;
}

/*
 * Programs and starts the cache build for one chip, then waits.
 * Returns 0 if the cache is ready, -1 on timeout.
 */
int dag_build_cache(int board, unsigned asic, const jas_coin_ctx *coin)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    unsigned valid = count_banks(jas_bank_partial[board], asic);
    uint64_t init_addr = dag_cache_init_addr(valid, coin->cache_size);
    unsigned budget = cache_timeout_s(coin->cache_size);
    uint16_t done = 0;
    unsigned waited;

    sl_multi_write(asic_en, CMD_BROADCAST, REG_CACHE_SIZE,
                   (uint16_t *)&coin->cache_size, 4, board);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SEED,
                   (uint16_t *)&coin->seed, 16, board);

    config_remap_info(board, (uint8_t)asic, 1);

    sl_multi_write(asic_en, CMD_MESH0, REG_CACHE_ADDR,
                   (uint16_t *)&init_addr, 3, board);

    sl_single_write(asic_en, CMD_MESH0, REG_CACHE_START, 1, board);

    for (waited = 0; waited <= budget; waited++) {
        sl_single_read(asic_en, CMD_MESH0_READ, REG_CACHE_DONE, &done,
                       1000000, board);
        if (done) {
            sl_single_write(asic_en, CMD_MESH0, REG_CACHE_START, 0, board);
            jas_log("board %d asic %u: cache ready (%u banks, %u s budget)",
                    board, asic, valid, budget);
            return 0;
        }
        sl_delay(1, 0);
    }

    sl_single_write(asic_en, CMD_MESH0, REG_CACHE_START, 0, board);
    jas_log("board %d asic %u: cache timeout after %u s", board, asic, budget);
    return -1;
}

/*
 * Same for the DAG. The size is that of the chip's coin, and so is the
 * time budget (where the original imposed its single coin's).
 */
int dag_build_dataset(int board, unsigned asic, const jas_coin_ctx *coin)
{
    uint8_t asic_en = (uint8_t)(1u << asic);
    unsigned budget = dag_timeout_s(coin->dag_size);
    uint16_t done = 0;
    unsigned waited;

    sl_multi_write(asic_en, CMD_BROADCAST, REG_DAG_SIZE,
                   (uint16_t *)&coin->dag_size, 4, board);

    sl_single_write(asic_en, CMD_MESH0, REG_DAG_START, 1, board);

    for (waited = 0; waited <= budget; waited++) {
        sl_single_read(asic_en, CMD_MESH0_READ, REG_DAG_DONE, &done,
                       1000000, board);
        if (done) {
            sl_single_write(asic_en, CMD_MESH0, REG_DAG_START, 0, board);
            jas_log("board %d asic %u: dag ready (%llu MiB, %u s budget)",
                    board, asic,
                    (unsigned long long)(coin->dag_size >> 20), budget);
            return 0;
        }
        sl_delay(1, 0);
    }

    sl_single_write(asic_en, CMD_MESH0, REG_DAG_START, 0, board);
    jas_log("board %d asic %u: dag timeout after %u s", board, asic, budget);
    return -1;
}

/*
 * Software reset of a chip before a rebuild; the original brackets the
 * pulse with two microseconds.
 */
void dag_soft_reset(int board, unsigned asic)
{
    uint8_t asic_en = (uint8_t)(1u << asic);

    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0, board);
    sl_delay(0, 2000);
    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, board);
}

/*
 * Complete sequence for one chip (soft reset, cache, DAG) with its own coin
 * context; no retry here. In the original, retries (up to four) reuse their
 * counter as the index of the mesh to close: the "close mesh_N" messages
 * give the attempt's rank, not a mesh diagnosed as faulty.
 */
int dag_calc_asic(int board, unsigned asic, const jas_coin_ctx *coin)
{
    dag_soft_reset(board, asic);

    if (dag_build_cache(board, asic, coin) != 0)
        return -1;
    if (dag_build_dataset(board, asic, coin) != 0)
        return -1;
    return 0;
}

/*
 * dag_progress_bar @ 0x472cc: the percentage shown during the build.
 *
 * It feeds the /index.html bar, and where it freezes tells at which step a
 * build stopped. Four steps, each mapped to a range:
 *
 *     step 0 :  0 -> 2 %       step 2 : 40 -> 90 %
 *     step 1 :  2 -> 40 %      step 3 : 90 -> 100 %
 *
 * Within a step, progress = counter/slice, clamped at both ends; when a
 * bound is reached the COUNTER is rewritten to match the percentage, or the
 * next step would jump backwards. The end flag (jasminer + 0x8a) forces
 * 100 %. (Ghidra renders ARM's signed compare as
 * `bVar9 || (iVar1 < 0) != bVar8`: it is simply `<=`.)
 */

uint8_t  jas_dag_percent;      /* jasminer + 0x09  */
uint16_t jas_dag_counter;      /* jasminer + 0x7a  */
uint16_t jas_dag_done;          /* jasminer + 0x8a  */
uint8_t  jas_dag_step;         /* dev_info_g + 0x3420 */

/* Set the percentage AND realign the counter: they go together. */
static void set_progress(uint8_t percent, uint8_t slice)
{
    jas_dag_percent = percent;
    jas_dag_counter = (uint16_t)(slice * (unsigned)percent);
}

void dag_progress_bar(uint8_t slice)
{
    unsigned p;

    jas_dag_counter++;

    if (jas_dag_done) {
        jas_dag_percent = 100;
        return;
    }
    if (slice == 0) {         /* the original divides without checking */
        jas_dag_percent = 100;
        return;
    }

    p = jas_dag_counter / slice;

    switch (jas_dag_step) {
    case 0:
        if (p < 2)
            jas_dag_percent = (uint8_t)p;
        else
            set_progress(2, slice);
        break;
    case 1:
        if (p < 2)
            set_progress(2, slice);
        else if (p <= 40)
            jas_dag_percent = (uint8_t)p;
        else
            set_progress(40, slice);
        break;
    case 2:
        if (p < 40)
            set_progress(40, slice);
        else if (p <= 90)
            jas_dag_percent = (uint8_t)p;
        else
            set_progress(90, slice);
        break;
    case 3:
        if (p < 90)
            set_progress(90, slice);
        else if (p < 100 && jas_dag_counter > slice * 90u)
            jas_dag_percent = (uint8_t)p;
        else
            set_progress(100, slice);
        break;
    default:
        jas_dag_percent = 100;
        break;
    }
}

/* ======================================================================
 * dag_calc @ 0x24199: the complete build, all boards.
 *
 * 1. IT WORKS BY PHASES, NOT BY CHIP. All chips receive their
 *    configuration, then all start, then we wait: the chips build their DAG
 *    in parallel.
 *
 * 2. IT CUTS THE DAG DOWN TO THE CHIP'S SIZE:
 *
 *        dag_len = min(chip_capacity - 2 x cache_size, requested_dag_len)
 *
 *    (the "change dag_len to %llu" of its log). A chip short of banks does
 *    not fail the startup: it carries a shorter DAG. The per-chip size
 *    mechanism is also what makes dual-coin mining possible.
 *
 * 3. IT AMPUTATES RATHER THAN GIVING UP: when a chip's DAG is wrong, it
 *    closes a mesh and retries, then bank 255. dag_done_flag carries the
 *    NUMBER of the mesh to close, mesh_bank_flag the amputation stage
 *    ("close mesh_N", "close bank255_1", "close bank255_N"). Here
 *    amputation is off by default (see the "fast" mode below).
 *
 * OUR VERSION: each chip carries its coin context (jas_coin_of) instead of
 * fpga_data; sizes, seed and time budget follow from it.
 * ====================================================================== */

#define REG_CACHE_ADDR3   0x304   /* cache placement, 3 words, mesh 0 */
#define REG_ASIC_START    0x0003  /* restart, value = chip mask */
#define CMD_BOARD_W       0xf9
#define REG_MC_EN         0x0c00  /* bank enable, 64 words per mesh */
#define REG_MC_REFRESH_EN 0x0c40  /* refresh, same */
#define CMD_MESH_W(m)     ((uint8_t)((0x10u << (m)) | 0x8u))
#define MESHES            4
#define AMPUT_NO      0
#define AMPUT_MESH        1
#define AMPUT_BANK255   2
#define RETRIES_MAX      4
/* Chip cache builds before the processor fallback: 3 in the original, 6
 * here, since a real rebuild costs 5 s against ~230 s for the fallback, and
 * a chip sometimes only succeeds at the 2nd try. */
#define REBUILD_CACHE_MAX 6

/*
 * The amputation state, per chip; it survives the whole function, which
 * carries the repair from one retry to the next.
 *
 *   done[c][p]  0 = the DAG is good; 1 to 4 = the rank of the mesh to close
 *   amput_stage[c][p] 0 = intact, 1 = one mesh closed, 2 = and bank 255
 */
static uint8_t amput_done[3][3];

/*
 * CACHE RECLAIM (see include/cache_reclaim.h). First node the processor
 * will write for each chip, computed by prepare_chip; equals the total
 * (= nothing to do) when the DAG fits or the reclaim is off.
 */
#include "cache_reclaim.h"
static uint32_t reclaim_front[3][3];
/* 1 when the chip's cache stayed corrupt after the rebuilds: its prefix
 * will be wrong, so writing a tail is useless (9 to 11 min lost per try).
 * The DAG check decides afterwards, as before. */
static uint8_t  reclaim_cache_failed[3][3];
/* 1 when the chip has finished building its prefix (pass A of one_pass). */
static uint8_t  reclaim_ready[3][3];
/* Chips whose DAG overflows memory past the reject threshold (prepare_chip). */
static uint8_t  dag_discarded[3];

/*
 * HOW MANY CHIPS DID NOT GET THEIR DAG. amput_done starts at 1 for each
 * active chip and is reset to 0 only by a successful build: what remains
 * at 1 after dag_calc is a failure (what dag_calc's final loop prints as
 * "calc dag fail"). Exposed so that seed_update_operation can decide to
 * replay the startup. To be called ONLY AFTER dag_calc.
 */
unsigned dag_calc_failures(void)
{
    unsigned board, chip, n = 0;

    for (board = 0; board < BOARD_NUM; board++)
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                && amput_done[board][chip])
                n++;
    return n;
}
static uint8_t amput_stage[3][3];
static uint8_t amput_mesh[3][3];

/* How many banks this chip can really address. */
static unsigned banks_valid(int board, unsigned chip)
{
    return count_banks(jas_bank_partial[board], chip);
}

/*
 * CAPACITY ALERT.
 *
 * The ethash cache sits at the TOP of the chip's memory and stays resident
 * during the WHOLE DAG build, which grows from row 0. When both no longer
 * fit, the DAG overwrites the cache it is built from: the DAG's high rows
 * come out wrong, and mining reads them. The real floor is thus not
 * ⌈dag / row⌉ but:
 *
 *     floor = ⌈ (dag_size + cache_size) / (0xff6 × 512) ⌉
 *
 * (3894 banks at epoch 828, against 3834 when the cache is forgotten.)
 *
 * WARN, DO NOT ACT: truncating the DAG like the factory firmware would be
 * WORSE (cutting ~2 % of the DAG gives ~77 % rejects), and our deliberately
 * optimistic dag_asic_capacity is what lets us mine better than the
 * original between 3894 and 3944 banks. Here the condition is only made
 * VISIBLE.
 *
 * Amplification: ethash makes 64 DAG accesses per nonce and one wrong item
 * is enough, hence reject = 1 − (1−p)^64 (six squarings, no libm).
 */
static double reject_expected(unsigned lost, unsigned total)
{
    double q = 1.0 - (double)lost / (double)total;
    int k;

    for (k = 0; k < 6; k++)          /* q^(2^6) = q^64 */
        q *= q;
    return 1.0 - q;
}

static void alert_capacity(int board, unsigned chip, const jas_coin_ctx *coin,
                            unsigned valid, unsigned cache_row)
{
    const uint64_t row_chip = (uint64_t)0xff6u * 512u;   /* 2 092 032 o */
    uint64_t row_bytes, need;
    unsigned dag_rows, floor;

    if (!valid || !coin->dag_size)
        return;

    row_bytes = (uint64_t)valid * 512u;               /* a whole row */
    dag_rows   = (unsigned)((coin->dag_size + row_bytes - 1) / row_bytes);
    need        = coin->dag_size + coin->cache_size;
    floor      = (unsigned)((need + row_chip - 1) / row_chip);

    if (dag_rows > cache_row) {
        unsigned overwritten = dag_rows - cache_row;

        /* With the reclaim, overflowing is no longer a failure: the chip
         * stops at the cache's edge and the processor writes the tail. */
        if (jas_cache_reclaim_active()) {
            jas_log("board %d asic%u: the DAG overflows the cache by %u rows"
                    " (%u banks, floor %u): tail written by the processor"
                    " (reclaim)%s",
                    board, chip, overwritten, valid, floor,
                    dag_rows > 0xff6u ? " — and it exceeds the memory, see"
                    " below" : ", no reject expected");
            return;
        }
        jas_log("board %d asic%u: CAPACITY TOO LOW — the DAG reaches row %u and"
                " the cache starts at %u: %u rows OVERWRITTEN. %u banks for a"
                " floor of %u (%u missing). Expected reject %.1f%%.",
                board, chip, dag_rows, cache_row, overwritten,
                valid, floor, floor - valid,
                100.0 * reject_expected(overwritten, dag_rows));
        return;
    }

    /*
     * No collision yet: say how soon. The DAG grows by 2^23 bytes per epoch
     * and the cache by 2^17, i.e. 8,519,680 B; a bank holds 2,092,032. One
     * bank of margin ≈ a quarter of an epoch ≈ 1.1 days.
     */
    {
        unsigned margin = valid > floor ? valid - floor : 0;
        double epochs = (double)margin * (double)row_chip / 8519680.0;

        if (margin < 60u)
            jas_log("board %d asic%u: LOW capacity margin — %u banks above the"
                    " floor %u, i.e. ~%.0f epochs (~%.0f days) before the DAG"
                    " reaches the cache.",
                    board, chip, margin, floor, epochs, epochs * 4.51);
        else
            jas_trace("board %d asic%u: capacity OK — %u banks, floor %u,"
                      " margin %u (~%.0f epochs)",
                      board, chip, valid, floor, margin, epochs);
    }
}

/*
 * SWITCH TO ETC: a chip whose expected ETHW reject reaches the threshold
 * (etc_switch_threshold, 70 % by default) moves to ETC, whose DAG (etchash) is
 * about half the size and still fits whole.
 *
 * The chip is assigned to the group whose pool is ETC, a job of that group
 * is awaited, and the chip's context is set on that job BEFORE the caller
 * programs cache, seed and DAG size: it reads them from that context.
 *
 * ORDER: the group first, the context after. A group without a chip has no
 * connection (pool.c, group_no_chip): the assignment is what brings the
 * ETC job. And if the mining loop runs (epoch rebuild), it sees an ETC job
 * older than the chip's ETHW context and ignores it (late-epoch guard)
 * instead of asking for a rebuild; the reverse order would put the chip
 * back on ETHW.
 *
 * For THIS startup only: the configuration file is not touched, and the
 * rule judges again at the next startup. Off switch: JASRE_ETC_SWITCH=0
 * or the jasre_no_etc_switch marker. Returns 1 if the chip switched.
 */
static int switch_etc_active(void)
{
    const char *e = getenv("JASRE_ETC_SWITCH");

    if (e && *e)
        return *e != '0';
    /* Marker jasre_no_etc_switch: /tmp (trial) or /media/configs (lasting). */
    return !jas_marker("jasre_no_etc_switch");
}

/*
 * THE SWITCH THRESHOLD, in % of expected reject: "etc_switch_threshold" in
 * the configuration (settings page), 70 by default, the point where ETHW
 * stops paying more than ETC. JASRE_ETHW_REJECT_MAX overrides it for
 * trials. Bounded to [0, 100]: 0 switches as soon as the DAG does not fit,
 * 100 only when no hash can be right any more. A value that does not read
 * as a number keeps the default: a misread 0 would move every chip whose
 * DAG overflows by a single node.
 */
static int read_percentage(const char *text, double *v)
{
    char *end;
    double x;

    while (*text == ' ' || *text == '\t' || *text == '"')
        text++;
    x = strtod(text, &end);
    if (end == text || x != x)
        return 0;
    *v = x < 0.0 ? 0.0 : x > 100.0 ? 100.0 : x;
    return 1;
}

static double etc_threshold = 70.0;
static pthread_once_t etc_threshold_once = PTHREAD_ONCE_INIT;

/* Read once: the chips may be prepared by several threads at once, and a
 * second thread must not take the default while the first one reads. */
static void etc_threshold_read(void)
{
    double threshold = 70.0;
    const char *source = "default";
    const char *e;

    e = getenv("JASRE_ETHW_REJECT_MAX");
    if (e && *e) {
        if (read_percentage(e, &threshold))
            source = "JASRE_ETHW_REJECT_MAX";
        else
            jas_log("ETC switch: JASRE_ETHW_REJECT_MAX=\"%s\" is not a number,"
                    " ignored", e);
    }
    if (strcmp(source, "default") == 0) {
        FILE *f = fopen(jas_config_file(), "r");
        char line[512];

        if (f) {
            while (fgets(line, sizeof(line), f)) {
                char *p = strstr(line, "\"etc_switch_threshold\"");

                if (p && (p = strchr(p, ':'))) {
                    if (read_percentage(p + 1, &threshold))
                        source = "settings page";
                    else
                        jas_log("ETC switch: etc_switch_threshold is not a"
                                " number, ignored");
                }
            }
            fclose(f);
        }
    }
    jas_log("ETC switch: threshold %.0f %% of expected reject (%s)", threshold, source);
    etc_threshold = threshold;
}

static double etc_switch_threshold(void)
{
    pthread_once(&etc_threshold_once, etc_threshold_read);
    return etc_threshold;
}

static int switch_to_etc(int board, unsigned chip)
{
    int g, before = jas_pool_of_chip(chip);
    jas_work tp;
    unsigned c, s;

    if (!switch_etc_active())
        return 0;
    g = jas_group_etchash();
    if (g < 0) {
        jas_log("board %d asic%u: no ETC pool configured, the chip cannot"
                " switch to ETC", board, chip);
        return 0;
    }
    if (g == before)
        return 0;                       /* already on ETC: nothing smaller */

    jas_pool_switch_chip(chip, g);
    for (s = 0; s < 60; s++) {
        if (jas_work_of_group(g, &tp))
            break;
        sleep(1);
    }
    if (s == 60) {
        jas_pool_switch_chip(chip, before);
        jas_log("board %d asic%u: the ETC pool (group %d) served no work in"
                " 60 s, the chip stays on its group", board, chip, g + 1);
        return 0;
    }
    for (c = 0; c < 3; c++) {
        jas_coin_ctx *cc = jas_coin_of((int)c, chip);

        jas_coin_from_block_seed(cc, tp.block_number, &tp.seed);
        cc->header = tp.header;
        cc->target = tp.target;
        snprintf(cc->work_id, sizeof(cc->work_id), "%s", tp.work_id);
    }
    /* The ETC processor cache, to judge the DAG and the nonces. */
    jas_cache_request(tp.block_number, &tp.seed);
    /* The DAG size changes: what work.c believes it knows no longer holds. */
    jas_work_memo_forget(board, (uint8_t)(1u << chip));
    return 1;
}

/*
 * Prepare a chip: count its banks, deduce where to place the cache and
 * what DAG size it can carry.
 */
static void prepare_chip(int board, unsigned chip, const jas_coin_ctx *coin,
                          uint64_t *dag_len)
{
    unsigned valid = banks_valid(board, chip);
    uint64_t capacity = dag_asic_capacity(0);   /* measured constant */
    unsigned cache_row;

    jas_asic_max_capacity[board * 3 + chip] = (int64_t)capacity;

    /*
     * TRIAL: JASRE_ETC_SWITCH_TRIAL=mask (or the file
     * /tmp/jasre_etc_switch_trial) moves these chips to ETC without any
     * reject condition, to try the switch on a machine whose ETHW DAG fits.
     * Never read from /media/configs: no lasting forced switch.
     */
    {
        const char *e = getenv("JASRE_ETC_SWITCH_TRIAL");
        unsigned long m = e ? strtoul(e, NULL, 0) : 0;

        if (!e) {
            FILE *f = fopen("/tmp/jasre_etc_switch_trial", "r");
            char v[16];

            if (f) {
                if (fgets(v, sizeof(v), f))
                    m = strtoul(v, NULL, 0);
                fclose(f);
            }
        }
        if ((m >> chip & 1) && switch_to_etc(board, chip)) {
            jas_log("board %d asic%u: switch to ETC FORCED (trial)", board, chip);
            coin = jas_coin_of(board, chip);
        }
    }

    {
        /* 0xff6u is promoted to uint64_t by the division: under ~60 banks the
         * subtraction wraps and the row number becomes huge, which would show a
         * comfortable margin on a chip that has nothing left. Bounded. */
        uint64_t rc = valid ? coin->cache_size / ((uint64_t)valid << 9) : 0;

        cache_row = (valid && rc < 0xff6u) ? (unsigned)(0xff6u - rc) : 0;
    }
    jas_trace("board %d asic%u cache_remap_good_valid_tmp = %u", board, chip, valid);
    jas_trace("board %d asic%u cache_start_row = %u", board, chip, cache_row);
    jas_trace("board %d asic%u wtf[%d][%u] = %lld", board, chip, board, chip,
            (long long)capacity);

    alert_capacity(board, chip, coin, valid, cache_row);

    /*
     * The reduction as the original firmware writes it: capacity minus TWICE
     * the cache (cache_size * -2), which keeps a cache_size of margin between
     * the DAG and the cache.
     */
    reclaim_front[board][chip] = (uint32_t)(coin->dag_size / 64u);   /* nothing to do */
    if (jas_cache_reclaim_active()) {
        /*
         * RECLAIM: instead of TRUNCATING the DAG (mining would read pages that
         * do not exist -> rejects), the chip builds only the PREFIX, up to the
         * cache's edge; the processor then writes the tail into the cache's
         * rows once the chip is done (one_pass).
         */
        uint32_t total = (uint32_t)(coin->dag_size / 64u);
        uint32_t front = reclaim_front_node(valid, cache_row, coin);

        /*
         * TRIAL MODE (bench): JASRE_RECLAIM_TRIAL=N forces a tail of N
         * nodes written by the processor even when the DAG fits, to exercise
         * the whole path (prefix, grouped write, remaps, checks) on a small
         * epoch in under a minute. No effect if the DAG already overflows.
         */
        if (front >= total) {
            const char *e = getenv("JASRE_RECLAIM_TRIAL");
            uint32_t n = e ? (uint32_t)strtoul(e, NULL, 0) : 0;

            if (n && n < total) {
                front = (total - n) & ~1u;
                jas_log("board %d asic%u: reclaim TRIAL — forced tail of %u"
                        " nodes (JASRE_RECLAIM_TRIAL)", board, chip, total - front);
            }
        }

        reclaim_front[board][chip] = front;

        /*
         * DAG LARGER THAN THE MEMORY: tolerated as long as the expected
         * reject stays under the switch threshold (etc_switch_threshold, 70 %
         * by default). Beyond 0xff6 rows x valid banks x 8 nodes there is no
         * memory left: those nodes are missing and the hashes that read them
         * are wrong. Expected reject: 1 - (1 - missing/total)^64.
         */
        {
            uint32_t node_capacity = valid ? (uint32_t)((uint64_t)0xff6u * valid * 8u) : 0;

            reclaim_set_end(board, chip, 0);
            dag_discarded[board] &= (uint8_t)~(1u << chip);
            if (valid && node_capacity < total) {
                double threshold = etc_switch_threshold();
                double reject = 100.0 * reject_expected(total - node_capacity, total);

                if (reject < threshold) {
                    reclaim_set_end(board, chip, node_capacity);
                    jas_log("board %d asic%u: INCOMPLETE DAG TOLERATED — %u"
                            " nodes (%u MB, %.2f %%) beyond the memory (%u"
                            " banks); expected reject %.0f %% < %.0f %%: mining"
                            " anyway", board, chip,
                            total - node_capacity,
                            (unsigned)(((uint64_t)(total - node_capacity) * 64u) >> 20),
                            100.0 * (double)(total - node_capacity) / (double)total, valid,
                            reject, threshold);
                } else if (switch_to_etc(board, chip)) {
                    const jas_coin_ctx *etc = jas_coin_of(board, chip);

                    jas_log("board %d asic%u: DAG TOO LARGE — %u nodes beyond"
                            " the memory (%u banks), expected reject %.0f %% >="
                            " %.0f %%: the chip SWITCHES TO ETC (group %d,"
                            " epoch %llu, DAG %llu MB)",
                            board, chip, total - node_capacity, valid, reject, threshold,
                            jas_pool_of_chip(chip) + 1,
                            (unsigned long long)(etc->block_number / 30000u),
                            (unsigned long long)(etc->dag_size >> 20));
                    /* Redo everything for the ETC DAG: cache, front, capacity. */
                    prepare_chip(board, chip, etc, dag_len);
                    return;
                } else {
                    dag_discarded[board] |= (uint8_t)(1u << chip);
                    jas_log("board %d asic%u: DAG TOO LARGE — %u nodes beyond"
                            " the memory (%u banks), expected reject %.0f %% >="
                            " %.0f %%: chip SET ASIDE from this coin",
                            board, chip, total - node_capacity, valid, reject, threshold);
                }
            }
        }

        if (front < total) {
            *dag_len = (uint64_t)front * 64u;
            jas_log("board %d asic%u: reclaim ACTIVE — the chip builds %u/%u"
                    " nodes, the processor will write the remaining %u MB", board, chip, front,
                    total, (unsigned)(((uint64_t)(total - front) * 64u) >> 20));
        } else {
            *dag_len = coin->dag_size;          /* DAG fits: no reclaim */
        }
    } else if (capacity > 2 * coin->cache_size
        && capacity - 2 * coin->cache_size < coin->dag_size) {
        *dag_len = capacity - 2 * coin->cache_size;
        jas_log("change dag_len to %llu ", (unsigned long long)*dag_len);
    } else {
        *dag_len = coin->dag_size;
    }
}

/*
 * Wait for THIS chip's processor cache (its own coin's): without it,
 * ethash_nonce_check would refuse every share ("cache is not ready").
 */
static void wait_cache_processor_chip(int board, unsigned chip)
{
    const jas_coin_ctx *cc = jas_coin_of(board, chip);
    unsigned rounds = 0;

    if (!cc)
        return;
    while (!jas_cache_find(cc->block_number, &cc->seed)) {
        if (rounds % 10 == 0)
            jas_log("waiting for cpu cache of epoch %llu (board %d asic %u)...",
                    (unsigned long long)(cc->block_number / 30000), board, chip);
        if (++rounds > 600) {
            jas_log("cpu cache missing after 600 s (epoch %llu)",
                    (unsigned long long)(cc->block_number / 30000));
            return;
        }
        sleep(1);
    }
}


/*
 * One build pass over all the chips kept.
 * Returns the number of chips whose DAG is wrong at the end.
 */
/*
 * Chip temperatures (one TMP401 per chip, the chip's diode) in the log.
 * `read` = 1: I2C read now (bench or trial; the I2C lock is global, the
 * call waits for the sensor thread); 0: the sensor thread's last value.
 */
static void log_temperatures(int board, const char *step, int read)
{
    if (read)
        jas_sensors_read((unsigned)board);
    jas_log("board %d: chip temperatures %d / %d / %d C — %s", board,
            (int)jas_asic_temp[board][0], (int)jas_asic_temp[board][1],
            (int)jas_asic_temp[board][2], step);
}

/*
 * PREHEATING (trial, JASRE_PREHEAT=seconds; default 0 = none).
 *
 * Tests whether a chip's cache is wrong only when COLD (always the first
 * attempt after power-up): the 120 ALUs hash a dummy job for N seconds
 * (zero target, no nonce can come out), temperatures are logged every
 * 10 s, then the ALUs are cut and the results purged. The build's normal
 * preamble then puts everything back in place. Same job sequence as
 * send_initial_work (core/seed.c).
 */
#define PREHEAT_REG_NONCE   0x000c
#define PREHEAT_REG_TARGET  0x0010
#define PREHEAT_REG_HEADER  0x0020

static unsigned preheat_seconds(void)
{
    static int loaded = 0;
    static unsigned s = 0;

    if (!loaded) {
        const char *e = getenv("JASRE_PREHEAT");

        s = e ? (unsigned)strtoul(e, NULL, 0) : 0;
        if (s > 600)
            s = 600;
        loaded = 1;
    }
    return s;
}

static void preheat(int board, unsigned seconds)
{
    uint8_t asic_en = 0;
    uint16_t zero2[2] = { 0, 0 };
    uint16_t target[16], header[16];
    unsigned chip, mesh, t, guard = 0;
    gooddata_detail d;

    for (chip = 0; chip < ASIC_NUM; chip++)
        if ((ASIC_EN(board) >> chip & 1) && amput_done[board][chip])
            asic_en |= (uint8_t)(1u << chip);
    if (!asic_en || !seconds)
        return;

    memset(target, 0, sizeof(target));        /* target 0: unreachable */
    memset(header, 0x5a, sizeof(header));   /* any header                   */

    log_temperatures(board, "before preheating", 1);
    jas_log("board %d: PREHEATING %u s, chips 0x%x — hashing on dummy work",
            board, seconds, asic_en);

    /* The same preamble as the build. */
    for (chip = 0; chip < ASIC_NUM; chip++) {
        if (!(asic_en >> chip & 1))
            continue;
        sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST, REG_SOFT_RESET, 0, board);
        sl_delay(0, 2000);
        sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST, REG_SOFT_RESET, 0xffff, board);
    }
    config_asic_fix_param(board);
    sl_single_write(0xff, CMD_BOARD_W, 3 /* REG_ASIC_EN */, (uint16_t)ASIC_EN(board), board);
    for (chip = 0; chip < ASIC_NUM; chip++) {
        uint8_t mask = (uint8_t)(1u << chip);

        if (!(asic_en >> chip & 1))
            continue;
        for (mesh = 0; mesh < MESHES; mesh++)
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_EN,
                           jas_mc_en[board][chip][mesh], 0x40, board);
        for (mesh = 0; mesh < MESHES; mesh++)
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_REFRESH_EN,
                           jas_mc_refresh_en[board][chip][mesh], 0x40, board);
    }

    /* A dummy job per chip, then the reset that validates it, then the ALUs. */
    for (chip = 0; chip < ASIC_NUM; chip++) {
        const jas_coin_ctx *coin = jas_coin_of(board, chip);
        uint8_t mask = (uint8_t)(1u << chip);
        uint64_t nonce = (uint64_t)chip * 0x78u;

        if (!(asic_en >> chip & 1) || !coin)
            continue;
        config_remap_info(board, (uint8_t)chip, 1);
        sl_multi_write(mask, CMD_BROADCAST, REG_DAG_SIZE,
                       (uint16_t *)&coin->dag_size, 4, board);
        sl_multi_write(mask, CMD_BROADCAST, REG_CACHE_SIZE,
                       (uint16_t *)&coin->cache_size, 4, board);
        sl_multi_write(mask, CMD_BROADCAST, REG_SEED,
                       (uint16_t *)&coin->seed, 16, board);
        sl_multi_write(mask, CMD_BROADCAST, PREHEAT_REG_HEADER, header, 16, board);
        sl_multi_write(mask, CMD_BROADCAST, PREHEAT_REG_TARGET, target, 16, board);
        for (mesh = 0; mesh < MESHES; mesh++) {
            sl_multi_write(mask, CMD_MESH_W(mesh), PREHEAT_REG_NONCE,
                           (uint16_t *)&nonce, 4, board);
            nonce += 0x1eu;
        }
    }
    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0, board);
    sl_delay(0, 2000);
    sl_single_write(asic_en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, board);
    for (chip = 0; chip < ASIC_NUM; chip++) {
        uint8_t mask = (uint8_t)(1u << chip);

        if (!(asic_en >> chip & 1))
            continue;
        for (mesh = 0; mesh < MESHES; mesh++)
            sl_multi_write(mask, CMD_MESH_W(mesh), REG_ALU_EN,
                           jas_alu_en[board][chip][mesh], 2, board);
    }

    for (t = 1; t <= seconds; t++) {
        sleep(1);
        if (t % 10 == 0 || t == seconds)
            log_temperatures(board, "preheating in progress", 1);
    }

    sl_multi_write(asic_en, CMD_BROADCAST, REG_ALU_EN, zero2, 2, board);
    usleep(100000);
    while (get_asic_result_operation(board, &d) && guard++ < 100000u)
        ;
    purge_results(board, asic_en);
    log_temperatures(board, "after preheating (ALUs off)", 1);
}

/* Verdict of the chip's cache map (diagnostic): number of wrong probes
 * outside node 0's block; 1 if unknown. */
static unsigned map_others[3][3];
static struct map_verdict map_v[3][3];

/*
 * DIAGNOSTIC: read back the cache PARAMETERS in the chip (seed 16 words,
 * cache size 4, DAG size 4, cache address 3) and compare them with what
 * was just written: a word lost from the broadcast write (unacknowledged)
 * would make the engine compute a valid cache of another seed or size.
 */
static void diag_reread_parameters(int board, unsigned chip, const jas_coin_ctx *coin,
                                   uint64_t dag_len, uint64_t init_addr,
                                   const char *step)
{
    uint8_t m = (uint8_t)(1u << chip);
    uint16_t sd[16], cs[4], ds[4], ca[3];
    const uint16_t *ws = (const uint16_t *)&coin->seed;
    const uint16_t *wc = (const uint16_t *)&coin->cache_size;
    const uint16_t *wd = (const uint16_t *)&dag_len;
    const uint16_t *wa = (const uint16_t *)&init_addr;
    unsigned k, os = 0, oc = 0, od = 0, oa = 0;

    memset(sd, 0, sizeof(sd)); memset(cs, 0, sizeof(cs));
    memset(ds, 0, sizeof(ds)); memset(ca, 0, sizeof(ca));
    sl_multi_read(m, CMD_MESH0_READ, REG_SEED, sd, 16, 1000000, board);
    sl_multi_read(m, CMD_MESH0_READ, REG_CACHE_SIZE, cs, 4, 1000000, board);
    sl_multi_read(m, CMD_MESH0_READ, REG_DAG_SIZE, ds, 4, 1000000, board);
    sl_multi_read(m, CMD_MESH0_READ, REG_CACHE_ADDR3, ca, 3, 1000000, board);
    for (k = 0; k < 16; k++) if (sd[k] == ws[k]) os++;
    for (k = 0; k < 4; k++)  if (cs[k] == wc[k]) oc++;
    for (k = 0; k < 4; k++)  if (ds[k] == wd[k]) od++;
    for (k = 0; k < 3; k++)  if (ca[k] == wa[k]) oa++;
    jas_log("board %d asic %u: registers read back (%s) — seed %u/16, cache"
            " size %u/4, DAG size %u/4, cache address %u/3 | seed read %04x"
            " %04x %04x %04x .. %04x (sent %04x %04x %04x %04x .. %04x) | cache"
            " read %04x %04x %04x %04x (sent %04x %04x %04x %04x) | address"
            " read %04x %04x %04x (sent %04x %04x %04x)",
            board, chip, step, os, oc, od, oa,
            sd[0], sd[1], sd[2], sd[3], sd[15], ws[0], ws[1], ws[2], ws[3], ws[15],
            cs[0], cs[1], cs[2], cs[3], wc[0], wc[1], wc[2], wc[3],
            ca[0], ca[1], ca[2], wa[0], wa[1], wa[2]);
    if (os < 16) {
        for (k = 0; k < 16; k++)
            if (sd[k] != ws[k])
                jas_log("board %d asic %u:   seed word %u: read %04x, sent %04x",
                        board, chip, k, sd[k], ws[k]);
    }
}

/*
 * REALLY REBUILD THE CACHE OF THE FAULTY CHIPS, as the original does.
 *
 * After a failed cache check, the original redoes, for the faulty chips
 * ONLY, the whole preparation (soft reset 0x830, config_asic_fix_param,
 * ASIC_EN, mc_en / mc_refresh, then DAG size, cache size, seed, remap,
 * cache address) and only then 0x300 = 1, a one-second wait, and the wait
 * for 0x301. Rewriting 0x300 = 1 alone does not restart the engine.
 *
 * The other chips, whose cache is good, are not touched (soft reset masked
 * to the chip); config_asic_fix_param sets the same timings for the whole
 * board, as in the original.
 */
static void cache_resume_chips(uint8_t failed[3][3])
{
    unsigned board, chip, mesh;

    for (board = 0; board < BOARD_NUM; board++) {
        uint8_t m = 0;

        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((ASIC_EN(board) >> chip & 1) && amput_done[board][chip] && failed[board][chip])
                m |= (uint8_t)(1u << chip);
        if (!m)
            continue;

        for (chip = 0; chip < ASIC_NUM; chip++) {
            if (!(m >> chip & 1))
                continue;
            sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST, REG_SOFT_RESET, 0, (int)board);
            sl_delay(0, 2000);
            sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST, REG_SOFT_RESET, 0xffff, (int)board);
        }
        config_asic_fix_param((int)board);
        sl_single_write(0xff, CMD_BOARD_W, 3 /* REG_ASIC_EN */,
                        (uint16_t)ASIC_EN(board), (int)board);
        for (chip = 0; chip < ASIC_NUM; chip++) {
            uint8_t mask = (uint8_t)(1u << chip);

            if (!(m >> chip & 1))
                continue;
            for (mesh = 0; mesh < MESHES; mesh++)
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_EN,
                               jas_mc_en[board][chip][mesh], 0x40, (int)board);
            for (mesh = 0; mesh < MESHES; mesh++)
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_REFRESH_EN,
                               jas_mc_refresh_en[board][chip][mesh], 0x40, (int)board);
        }
        for (chip = 0; chip < ASIC_NUM; chip++) {
            const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
            uint8_t mask = (uint8_t)(1u << chip);
            uint64_t dag_len, init_addr;
            unsigned valid;

            if (!(m >> chip & 1))
                continue;
            prepare_chip((int)board, chip, coin, &dag_len);
            valid = banks_valid((int)board, chip);
            init_addr = dag_cache_init_addr(valid, coin->cache_size);
            sl_multi_write(mask, CMD_BROADCAST, REG_DAG_SIZE,
                           (uint16_t *)&dag_len, 4, (int)board);
            sl_multi_write(mask, CMD_BROADCAST, REG_CACHE_SIZE,
                           (uint16_t *)&coin->cache_size, 4, (int)board);
            sl_multi_write(mask, CMD_BROADCAST, REG_SEED,
                           (uint16_t *)&coin->seed, 16, (int)board);
            config_remap_info((int)board, (uint8_t)chip, 1);
            sl_multi_write(mask, CMD_MESH0, REG_CACHE_ADDR3,
                           (uint16_t *)&init_addr, 3, (int)board);
        }
        jas_log("board %u: cache preparation REDONE for chips 0x%x (soft reset"
                " + parameters + programming, as the original)", board, m);
    }
}

/*
 * PER-CHIP STARTUP: mine ETC without waiting for the ETHW chips.
 *
 * On a board with two coins, the ETC chip builds its whole DAG in tens of
 * seconds but used to wait for the processor to write the ETHW chips' DAG
 * tail (80 to 120 s, more at each epoch).
 *
 * So at STARTUP only, if a board has both chips ready without a tail to
 * write and chips that overflow, the latter are PUT ON HOLD: their built
 * DAG (prefix) stays under the write remap, they are neither checked nor
 * started, and the mining start spares them (seed.c: no soft reset, no
 * ALU). The rebuild thread then writes their tail, checks them and has
 * them join mining through the epoch-change sequence (join_mining).
 * A single-coin board is unchanged. JASRE_STARTUP_PER_CHIP=0 or the
 * marker /tmp/jasre_startup_grouped restore the previous behavior.
 */
static uint8_t      dag_deferred[3];             /* chips waiting for their tail */
static int          dag_during_startup;       /* one_pass called by dag_calc() */
static volatile int dag_start_done;            /* seed.c has started mining */
static struct timeval dag_deferred_t0;

static int startup_per_chip(void)
{
    const char *e = getenv("JASRE_STARTUP_PER_CHIP");

    if (e && *e)
        return *e != '0';
    /* Marker jasre_startup_grouped: /tmp (trial) or /media/configs (lasting). */
    return !jas_marker("jasre_startup_grouped");
}

uint8_t jas_dag_deferred(int board)
{
    return (board >= 0 && board < 3) ? dag_deferred[board] : 0;
}

uint8_t jas_dag_discarded(int board)
{
    return (board >= 0 && board < 3) ? dag_discarded[board] : 0;
}

void jas_dag_start_done(void)
{
    dag_start_done = 1;
}

static unsigned one_pass(void)
{
    unsigned board, chip, failures = 0;

    /*
     * BISECTION (diagnostic, JASRE_BISECT_MESH=0..3): take mesh m of asic2
     * out of service (partial/good = 0, mc_en = 0) to locate a marginal
     * memory cell that corrupts its cache. The rest of one_pass recounts the
     * valid banks, redoes remap + mc_en, and builds the cache WITHOUT that
     * mesh.
     *   - asic2's cache GOOD without mesh m -> the bad cell is in that mesh;
     *   - no exclusion cures it             -> not a localized cell.
     */
    {
        const char *bm  = getenv("JASRE_BISECT_MESH");
        const char *blo = getenv("JASRE_BISECT_LO");
        const char *bhi = getenv("JASRE_BISECT_HI");
        unsigned per = BANKS_PER_ASIC / MESHES;                  /* 1020 */
        int lo = -1, hi = -1;

        if (blo && bhi) {                    /* asic2's bank range [lo,hi) */
            lo = atoi(blo);
            hi = atoi(bhi);
        } else if (bm) {                     /* a whole mesh */
            int m = atoi(bm);
            if (m >= 0 && m < MESHES) {
                lo = m * (int)per;
                hi = lo + (int)per;
            }
        }
        if (lo >= 0 && hi > lo && hi <= (int)BANKS_PER_ASIC) {
            unsigned b, cnt = 0;

            for (b = (unsigned)lo; b < (unsigned)hi; b++) {
                unsigned idx  = 2u * BANKS_PER_ASIC + b;
                unsigned mesh = b / per, within = b % per;

                if (jas_bank_partial[0][idx])
                    cnt++;
                jas_bank_partial[0][idx] = 0;
                jas_bank_good[0][idx]    = 0;
                jas_mc_en[0][2][mesh][within >> 4] &=
                    (uint16_t)~(1u << (within & 0xf));
            }
            jas_log("BISECT: asic2 banks [%d,%d) EXCLUDED (%u out of service),"
                    " cache built without them", lo, hi, cnt);
        }
    }

    if (preheat_seconds())
        for (board = 0; board < BOARD_NUM; board++)
            if (BOARD_EN >> board & 1)
                preheat((int)board, preheat_seconds());

    /*
     * SOFT RESET OF THE MEMORY CONTROLLERS BEFORE THE BUILD, as the original
     * does for EACH chip right before mc_en/cache (0x830 to 0, wait, then
     * 0xffff; then register 3 = asic_en, cmd 0xf9). Without it the
     * controllers keep the state the DFT/crossbar left, and on a marginal
     * chip the cache build writes garbage (deterministic within a boot,
     * variable between boots).
     */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                continue;
            sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST,
                            REG_SOFT_RESET, 0, (int)board);
            sl_delay(0, 2000);
            sl_single_write((uint8_t)(1u << chip), CMD_BROADCAST,
                            REG_SOFT_RESET, 0xffff, (int)board);
        }
        jas_log("board %u soft reset done", board);

        /*
         * RE-APPLY THE SeDRAM TIMINGS AFTER THE SOFT RESET: the 0x830 reset
         * clears the 25 DRAM timing registers (refresh: ref_ras / ref_period /
         * ref_pre_period, see asic_param.c), and the original sets them again
         * right here, before the build. Without them the cache is built with
         * the default refresh, and a marginal chip loses charge during the
         * long ETHW build -> wrong cells -> a garbage cache.
         */
        config_asic_fix_param((int)board);

        sl_single_write(0xff, CMD_BOARD_W, 3 /* REG_ASIC_EN */,
                        (uint16_t)ASIC_EN(board), (int)board);

        /*
         * mc_en / mc_refresh RIGHT AFTER the soft reset, BEFORE the remap, in
         * the original's order: the soft reset zeroes the controller, so the
         * enable masks must be sent again, mc_en first, remap next (loop 2).
         */
        for (chip = 0; chip < ASIC_NUM; chip++) {
            uint8_t mask = (uint8_t)(1u << chip);
            unsigned mesh;

            if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                continue;
            for (mesh = 0; mesh < MESHES; mesh++)
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_EN,
                               jas_mc_en[board][chip][mesh], 0x40, (int)board);
            for (mesh = 0; mesh < MESHES; mesh++)
                sl_multi_write(mask, CMD_MESH_W(mesh), REG_MC_REFRESH_EN,
                               jas_mc_refresh_en[board][chip][mesh], 0x40, (int)board);
        }
    }

    /* --- Program, then start: two sweeps, not one. --- */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
            uint8_t mask = (uint8_t)(1u << chip);
            uint64_t dag_len, init_addr;
            unsigned valid, mesh;

            if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                continue;

            prepare_chip((int)board, chip, coin, &dag_len);
            valid = banks_valid((int)board, chip);
            init_addr = dag_cache_init_addr(valid, coin->cache_size);

            sl_multi_write(mask, CMD_BROADCAST, REG_DAG_SIZE,
                           (uint16_t *)&dag_len, 4, (int)board);
            sl_multi_write(mask, CMD_BROADCAST, REG_CACHE_SIZE,
                           (uint16_t *)&coin->cache_size, 4, (int)board);
            sl_multi_write(mask, CMD_BROADCAST, REG_SEED,
                           (uint16_t *)&coin->seed, 16, (int)board);

            /* Cache with remap arg 1 (partial banks), as in the original;
             * arg 0 changed nothing (good == partial for the placement). */
            config_remap_info((int)board, (uint8_t)chip, 1);

            /*
             * mc_en / mc_refresh (0xc00 / 0xc40) are sent in the SOFT RESET
             * loop above (before the remap), in the original's order. The
             * masks: prepare_state -> 0xffff, cbsc_commit turns the condemned
             * banks off.
             */
            (void)mesh;

            sl_multi_write(mask, CMD_MESH0, REG_CACHE_ADDR3,
                           (uint16_t *)&init_addr, 3, (int)board);
            jas_trace("borad %u asic %u cache_remap send done", board, chip);
            if (jas_cache_diag_active())
                diag_reread_parameters((int)board, chip, coin, dag_len, init_addr,
                                       "after writing");
        }
    }

    /* ==================================================================
     * PHASE 1b (diagnostic, JASRE_MEMTEST): read NODE 0 BEFORE the
     * build, through the cache read path (dram_read_last_cache_node), to
     * compare with the value read AFTER:
     *   - before == after -> the build does NOT write where we READ:
     *                        build-write vs cache-read addressing mismatch;
     *   - before != after -> the build does write there; garbage =
     *                        compute/build.
     * ================================================================== */
    if (getenv("JASRE_MEMTEST")) {
        for (board = 0; board < BOARD_NUM; board++) {
            if (!(BOARD_EN >> board & 1))
                continue;
            for (chip = 0; chip < ASIC_NUM; chip++) {
                const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
                uint8_t asic_en = (uint8_t)(1u << chip);
                unsigned valid = banks_valid((int)board, chip);
                uint32_t base = (uint32_t)(dag_cache_init_addr(valid,
                                            coin->cache_size) >> 7);
                uint32_t base0 = base - (uint32_t)(coin->cache_size >> 7);
                ethash_light_t lc;
                uint16_t node[0x20];
                const uint16_t *cc;
                int r;

                if (!(ASIC_EN(board) >> chip & 1))
                    continue;

                lc = jas_cache_find(coin->block_number, &coin->seed);
                cc = lc ? (const uint16_t *)lc->cache : NULL;
                memset(node, 0, sizeof(node));
                r = dram_read_last_cache_node(asic_en, 1, base0,
                        coin->cache_size, (int)board, node);
                jas_log("MEMTEST-BEFORE asic %u node0 (valid=%u): %s read %04x"
                        " %04x %04x %04x | cpu %04x %04x", chip, valid,
                        r == 0 ? "ok" : "illisible", node[0], node[1], node[2],
                        node[3], cc ? cc[0] : 0, cc ? cc[1] : 0);

                /*
                 * SEED READ-BACK: the seed (REG_SEED, 16 words) is broadcast
                 * to all chips. If one chip reads it back wrong while another
                 * reads it right, the seed write is corrupted (wrong seed,
                 * garbage cache). If both read it wrong, REG_SEED is
                 * write-only and the test is inconclusive.
                 */
                {
                    uint16_t sd[16];
                    const uint16_t *ws = (const uint16_t *)&coin->seed;
                    unsigned k, smatch = 0;

                    memset(sd, 0, sizeof(sd));
                    sl_multi_read(asic_en, CMD_MESH0_READ, REG_SEED, sd, 16,
                                  1000000, (int)board);
                    for (k = 0; k < 16; k++)
                        if (sd[k] == ws[k])
                            smatch++;
                    jas_log("MEMTEST-SEED asic %u: read-back match %u/16 (sent"
                            " %04x %04x %04x | read %04x %04x %04x)",
                            chip, smatch, ws[0], ws[1], ws[2],
                            sd[0], sd[1], sd[2]);
                }
            }
        }
    }

    /* ==================================================================
     * BUILD, CHECK, REBUILD THE CACHE.
     *
     * The cache is built by the chip's ALUs: NON-DETERMINISTIC. A marginal
     * ALU or bank sometimes corrupts it, and a DAG built on a wrong cache
     * gives "wrong dag at address 0". The original CHECKS the cache
     * (dram_check_cache) and REBUILDS it while it is corrupt
     * (rebuild_cache_flag / retry_time, ~3 tries); so do we, up to
     * REBUILD_CACHE_MAX.
     * ================================================================== */
    {
        uint8_t cache_failed[3][3];
        unsigned rebuild;
        struct timeval t_cache_started;

        for (board = 0; board < BOARD_NUM; board++)
            for (chip = 0; chip < ASIC_NUM; chip++) {
                cache_failed[board][chip] = 1;
                reclaim_cache_failed[board][chip] = 0;
            }

        /* DIAGNOSTIC (on by default, JASRE_CACHE_DIAG=0 turns it off):
         * what node 0's block holds BEFORE the build (a pattern left by the
         * self-test?), and the temperatures at that moment. */
        if (jas_cache_diag_active())
            for (board = 0; board < BOARD_NUM; board++) {
                if (!(BOARD_EN >> board & 1))
                    continue;
                log_temperatures((int)board, "before the cache build", 1);
                for (chip = 0; chip < ASIC_NUM; chip++) {
                    const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
                    uint16_t n0[0x20];
                    const uint32_t *l = (const uint32_t *)n0;
                    uint32_t base;

                    map_others[board][chip] = 1;
                    if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip] || !coin)
                        continue;
                    base = (uint32_t)(dag_cache_init_addr(
                        banks_valid((int)board, chip), coin->cache_size) >> 7);
                    if (cache_read_block((int)board, chip, coin, base, n0) == 0)
                        jas_log("board %u asic %u: block %u BEFORE the cache"
                                " build: %08x %08x %08x %08x", board, chip, base,
                                l[0], l[1], l[2], l[3]);
                    else
                        jas_log("board %u asic %u: block %u BEFORE the cache"
                                " build: SILENT read", board, chip, base);
                }
            }

        for (rebuild = 0; rebuild < REBUILD_CACHE_MAX; rebuild++) {
            unsigned rest = 0;

            /* A REAL rebuild: redo the preparation of the faulty chips before
             * starting again (see cache_resume_chips). */
            if (rebuild > 0)
                cache_resume_chips(cache_failed);

            /* (Re)start the cache of the chips still faulty. */
            for (board = 0; board < BOARD_NUM; board++)
                for (chip = 0; chip < ASIC_NUM; chip++)
                    if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                        && amput_done[board][chip] && cache_failed[board][chip])
                        sl_single_write((uint8_t)(1u << chip), CMD_MESH0,
                                        REG_CACHE_START, 1, (int)board);
            gettimeofday(&t_cache_started, NULL);

            /*
             * ONE SECOND BEFORE LOOKING AT "DONE", as the original does
             * (sl_delay(1,0) right after 0x300 = 1). Without it, the 0x301
             * flag of the previous build is still up when read: the loop
             * exits in milliseconds and reads the SAME memory again.
             */
            sl_delay(1, 0);

            /* Wait, then CHECK. */
            for (board = 0; board < BOARD_NUM; board++) {
                if (!(BOARD_EN >> board & 1))
                    continue;
                for (chip = 0; chip < ASIC_NUM; chip++) {
                    const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
                    uint8_t mask = (uint8_t)(1u << chip);
                    unsigned budget = cache_timeout_s(coin->cache_size), t;
                    uint16_t ready = 0;
                    ethash_light_t lc;
                    uint32_t base;
                    int rcc;
                    uint16_t seen0 = 0, seen1 = 0, seen2 = 0, seen3 = 0;

                    if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip]
                        || !cache_failed[board][chip])
                        continue;

                    for (t = 0; t <= budget; t++) {
                        sl_single_read(mask, CMD_MESH0_READ, REG_CACHE_DONE,
                                       &ready, 1000000, (int)board);
                        if (ready)
                            break;
                        sl_delay(1, 0);
                    }
                    sl_single_write(mask, CMD_MESH0, REG_CACHE_START, 0, (int)board);
                    if (!ready) {
                        jas_log("board %u asic %u cache calc timeout ", board, chip);
                        rest++;
                        continue;
                    }
                    {
                        struct timeval t_done;
                        long ms;

                        gettimeofday(&t_done, NULL);
                        ms = (t_done.tv_sec - t_cache_started.tv_sec) * 1000L
                           + (t_done.tv_usec - t_cache_started.tv_usec) / 1000L;
                        jas_log("board %u asic %u cache calc done (try %u, %ld ms)",
                                board, chip, rebuild, ms);
                        /* A 127 MB cache is not built in under 2 s: below that,
                         * the restart did not happen. */
                        if (rebuild > 0 && ms < 2000)
                            jas_log("board %u asic %u: cache restart had NO"
                                    " EFFECT (%ld ms) — the \"done\" flag did"
                                    " not drop", board, chip, ms);
                    }

                    /* Is the cache IN THE CHIP right? NODE 0 is checked (the
                     * one DAG item 0 depends on), not the LAST one:
                     * dram_check_cache (last node) gave FALSE POSITIVES.
                     * Node 0 is reliable: the 6 ALUs read it identically.
                     * base in NODE units = init>>7; node 0 is at
                     * base0 = base - cache/128. */
                    lc = jas_cache_find(coin->block_number, &coin->seed);
                    base = (uint32_t)(dag_cache_init_addr(
                        banks_valid((int)board, chip), coin->cache_size) >> 7);
                    rcc = 1;                     /* not checkable by default */
                    if (lc) {
                        uint16_t n0[0x20];
                        uint32_t base0 = base - (uint32_t)(coin->cache_size >> 7);
                        const uint16_t *cc = (const uint16_t *)lc->cache;
                        unsigned k;

                        if (dram_read_last_cache_node(mask, 1, base0,
                                coin->cache_size, (int)board, n0) != 0)
                            rcc = -1;           /* illisible */
                        else {
                            seen0 = n0[0]; seen1 = n0[1]; seen2 = n0[2]; seen3 = n0[3];
                            for (rcc = 0, k = 0; k < 0x20; k++)
                                if (n0[k] != cc[k]) { rcc = -1; break; }
                        }
                    }
                    if (rcc == -1) {
                        /*
                         * Dump node 0 at EACH rebuild: identical between tries
                         * = deterministic (compute or stuck cell); different =
                         * a weak memory cell spread by the cache algorithm's
                         * mixing.
                         */
                        jas_log("board %u asic %u cache calc error, rebuild"
                                " (try %u) node0 read %04x %04x %04x %04x"
                                " (valid=%d)", board, chip, rebuild,
                                seen0, seen1, seen2, seen3,
                                banks_valid((int)board, chip));
                        rest++;                    /* still cache_failed: rebuild */
                        log_temperatures((int)board, "at the cache verdict (wrong)", 0);
                        if (jas_cache_diag_active()) {
                            int c = cache_map_v((int)board, chip, coin,
                                                          &map_v[board][chip]);
                            uint64_t dl = 0, ia;

                            map_others[board][chip] = (c < 0) ? 1u : (unsigned)c;
                            ia = dag_cache_init_addr(banks_valid((int)board, chip),
                                                     coin->cache_size);
                            prepare_chip((int)board, chip, coin, &dl);
                            diag_reread_parameters((int)board, chip, coin, dl, ia,
                                                   "at the wrong verdict");
                        }
                    } else {
                        cache_failed[board][chip] = 0;  /* cache good (or not checkable) */
                        jas_log("board %u asic %u: cache %s (try %u), chips at %d/%d/%d C",
                                board, chip, rcc == 0 ? "GOOD" : "not verifiable", rebuild,
                                (int)jas_asic_temp[board][0], (int)jas_asic_temp[board][1],
                                (int)jas_asic_temp[board][2]);
                    }
                }
            }

            if (rest == 0)
                break;
        }

        /* Cache still wrong after the retries: the chip fails (no DAG is built
         * on a rotten cache). */
        for (board = 0; board < BOARD_NUM; board++)
            for (chip = 0; chip < ASIC_NUM; chip++)
                if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                    && amput_done[board][chip] && cache_failed[board][chip]) {
                    const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
                    int repaired = 0;

                    jas_log("board %u asic %u cache still corrupt after %d tries",
                            board, chip, REBUILD_CACHE_MAX);
                    /*
                     * DECISION TREE (the probe map and the processor round
                     * trip replace the lottery of replayed self-tests):
                     *   all wrong + identical round trip -> the processor
                     *       writes the whole cache;
                     *   only block 0 wrong + identical round trip -> repaired;
                     *   read path only (a 2nd ALU reads right) -> the cache
                     *       is probably good: go on, the DAG will decide;
                     *   localized defect (data) or different round trip ->
                     *       the memory is at fault: bank removal recommended
                     *       (bank_remove_hot, not yet tried), and the
                     *       original behavior (chip condemned, self-test).
                     */
                    if (jas_cache_diag_active() && coin) {
                        const struct map_verdict *v = &map_v[board][chip];
                        int rt = cache_cpu_write_attempt((int)board, chip, coin);
                        int all_wrong = (v->read >= 2 && v->wrong_outside0 + 1 >= v->read);
                        int located  = (v->read >= 2 && v->data > 0 && !all_wrong);
                        int path    = (v->read >= 1 && v->wrong > 0 && v->path == v->wrong);

                        log_temperatures((int)board, "after the processor round trip", 1);
                        if (rt == 1 && v->wrong_outside0 == 0 && !path) {
                            jas_log("board %u asic %u: cache REPAIRED by the"
                                    " processor (only node 0's block was wrong)"
                                    " — going on",
                                    board, chip);
                            repaired = 1;
                        } else if (path) {
                            jas_log("board %u asic %u: only ALU 1's read PATH"
                                    " is at fault (the other mesh reads right):"
                                    " the cache is probably good — going on,"
                                    " the DAG check will decide", board, chip);
                            repaired = 1;
                        } else if (rt == 1 && all_wrong && jas_cache_cpu_active()) {
                            int f = cache_write_by_cpu((int)board, chip, coin, 64);

                            log_temperatures((int)board,
                                                 "after the cache written by the processor", 1);
                            if (f == 0) {
                                jas_log("board %u asic %u: going on with the"
                                        " cache written by the processor", board, chip);
                                repaired = 1;
                            }
                        } else if (located || rt == 0) {
                            unsigned vb = banks_valid((int)board, chip);
                            uint32_t block = (rt == 0)
                                ? (uint32_t)(dag_cache_init_addr(vb, coin->cache_size) >> 7)
                                : v->block_data;

                            jas_log("board %u asic %u: MEMORY at fault (%s) —"
                                    " removing bank ~%u (block %u) recommended,"
                                    " not tried yet: original behavior", board, chip,
                                    rt == 0 ? "processor round trip differs"
                                            : "localized defect", vb ? (block / 4u) % vb : 0u,
                                    block);
                        } else {
                            jas_log("board %u asic %u: undecided verdict (read"
                                    " %u, wrong %u, data %u, path %u, round"
                                    " trip %d): original behavior", board, chip, v->read, v->wrong,
                                    v->data, v->path, rt);
                        }
                    }
                    if (repaired) {
                        cache_failed[board][chip] = 0;
                    } else {
                        reclaim_cache_failed[board][chip] = 1;
                        failures++;
                    }
                }
    }

    /*
     * BISECTION: ONLY THE CACHE matters. With JASRE_BISECT_MESH set,
     * several nodes of asic2's cache are compared with the CPU cache, the
     * number of good ones is logged, and the DAG is SKIPPED (it does not fit
     * with a mesh removed, and would pollute). Cache 100 % good without mesh
     * m => the bad cell is in that mesh.
     */
    if (getenv("JASRE_BISECT_MESH")) {
        const jas_coin_ctx *coin = jas_coin_of(0, 2);
        uint8_t mask = (uint8_t)(1u << 2);
        ethash_light_t lc = coin ? jas_cache_find(coin->block_number,
                                                     &coin->seed) : NULL;
        const uint16_t *cc = lc ? (const uint16_t *)lc->cache : NULL;

        if (cc) {
            unsigned valid = banks_valid(0, 2);
            uint32_t base = (uint32_t)(dag_cache_init_addr(valid,
                                        coin->cache_size) >> 7);
            uint32_t base0 = base - (uint32_t)(coin->cache_size >> 7);
            static const uint32_t idx[16] = {
                0, 1, 2, 3, 8, 16, 32, 64, 128, 256, 512, 1024,
                2048, 4096, 8192, 16384 };
            unsigned i, good = 0, read = 0;

            for (i = 0; i < 16; i++) {
                uint16_t n[0x20];

                if (dram_read_last_cache_node(mask, 1, base0 + idx[i],
                        coin->cache_size, 0, n) == 0) {
                    read++;
                    if (n[0] == cc[idx[i] * 0x20] && n[1] == cc[idx[i] * 0x20 + 1])
                        good++;
                }
            }
            jas_log("BISECT RESULT: asic2 cache = %u/16 good nodes (%u read) %s",
                    good, read,
                    good == 16 ? "=> CLEAN CACHE, the cell is in the excluded mesh"
                               : "=> still corrupt (cell elsewhere, or several)");
        }
        return failures;                  /* no DAG during bisection */
    }

    /*
     * NO GLOBAL BARRIER HERE: the processor cache does not serve to BUILD
     * the DAG (the silicon builds it), only to CHECK it. The per-chip wait,
     * right before dram_check_dag below, sits exactly where the cache is
     * read. (A global wait on jas_cache_zil hung forever in dual mining,
     * where the caches live in the registry.)
     */

    for (board = 0; board < BOARD_NUM; board++)
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                && amput_done[board][chip]) {
                /*
                 * DAG remap (arg 2: write into the partial banks, read from the
                 * good ones) right before REG_DAG_START, as in the original;
                 * otherwise the DAG is built with the CACHE's table (arg 1).
                 */
                config_remap_info((int)board, (uint8_t)chip, 2);
                jas_log("board %u asic %u dag calc start", board, chip);
                sl_single_write((uint8_t)(1u << chip), CMD_MESH0,
                                REG_DAG_START, 1, (int)board);
            }

    /* ---- Pass A: wait for the end of the PREFIX (or of the whole DAG) on
     *      each chip, and remember which are ready. ---- */
    memset(reclaim_ready, 0, sizeof(reclaim_ready));
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
            uint8_t mask = (uint8_t)(1u << chip);
            unsigned budget = dag_timeout_s(coin->dag_size), t;
            uint16_t ready = 0;

            if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                continue;

            for (t = 0; t <= budget; t++) {
                sl_single_read(mask, CMD_MESH0_READ, REG_DAG_DONE, &ready,
                               1000000, (int)board);
                if (ready)
                    break;
                sl_delay(1, 0);
            }
            sl_single_write(mask, CMD_MESH0, REG_DAG_START, 0, (int)board);
            reclaim_ready[board][chip] = ready ? 1 : 0;
        }
    }

    /* ---- Pass B: grouped RECLAIM. The chips have just built the PREFIX
     *      (up to the cache's edge); the processor writes the DAG's tail into
     *      the cache's rows, UNDER THE SAME REMAP as the build (arg 2, set
     *      before REG_DAG_START); the switch to the read remap (arg 0) comes
     *      in pass C, as for a DAG built by the chip. Node i being the same
     *      for all the chips of a coin, it is computed ONCE and written at
     *      once to all those that are ready, with a healthy cache, and
     *      overflowing. A chip with a corrupt cache is skipped: its prefix
     *      is wrong, pass C's check will say so. ---- */
    if (jas_cache_reclaim_active()) {
        for (board = 0; board < BOARD_NUM; board++) {
            uint8_t done = 0;

            if (!(BOARD_EN >> board & 1))
                continue;

            dag_deferred[board] = 0;
            if (dag_during_startup && startup_per_chip()) {
                uint8_t no_tail = 0, with_tail = 0;

                for (chip = 0; chip < ASIC_NUM; chip++) {
                    const jas_coin_ctx *cp = jas_coin_of((int)board, chip);

                    if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip]
                        || !reclaim_ready[board][chip] || reclaim_cache_failed[board][chip]
                        || (dag_discarded[board] >> chip & 1))
                        continue;
                    if (reclaim_front[board][chip] >= (uint32_t)(cp->dag_size / 64u))
                        no_tail |= (uint8_t)(1u << chip);
                    else
                        with_tail |= (uint8_t)(1u << chip);
                }
                if (no_tail && with_tail) {
                    dag_deferred[board] = with_tail;
                    gettimeofday(&dag_deferred_t0, NULL);
                    jas_log("board %u: PER-CHIP STARTUP — chips 0x%x checked"
                            " and started right away; the DAG tail of chips"
                            " 0x%x will be written after mining starts", board, no_tail,
                            with_tail);
                    done |= with_tail;
                }
            }

            for (chip = 0; chip < ASIC_NUM; chip++) {
                const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
                uint32_t total = (uint32_t)(coin->dag_size / 64u);
                uint32_t fronts[3];
                uint8_t group = 0;
                unsigned q;

                if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip]
                    || (done >> chip & 1) || !reclaim_ready[board][chip]
                    || (dag_discarded[board] >> chip & 1))
                    continue;
                if (reclaim_cache_failed[board][chip]) {
                    if (reclaim_front[board][chip] < total)
                        jas_log("board %u asic %u: reclaim SKIPPED (cache"
                                " corrupt, prefix condemned) — the DAG check"
                                " will decide",
                                board, chip);
                    done |= (uint8_t)(1u << chip);
                    continue;
                }
                for (q = 0; q < ASIC_NUM; q++) {
                    const jas_coin_ctx *cq = jas_coin_of((int)board, q);

                    fronts[q] = reclaim_front[board][q];
                    if ((ASIC_EN(board) >> q & 1) && amput_done[board][q]
                        && !(done >> q & 1) && reclaim_ready[board][q]
                        && !reclaim_cache_failed[board][q] && fronts[q] < total
                        && !(dag_discarded[board] >> q & 1)
                        && cq->block_number == coin->block_number
                        && memcmp(&cq->seed, &coin->seed, sizeof(coin->seed)) == 0)
                        group |= (uint8_t)(1u << q);
                }
                done |= group | (uint8_t)(1u << chip);
                if (group == 0)
                    continue;               /* the DAG fits: nothing to write */

                wait_cache_processor_chip((int)board, chip);
                if (reclaim_write_tail_group((int)board, coin, fronts, group,
                                               NULL) != 0) {
                    for (q = 0; q < ASIC_NUM; q++)
                        if (group >> q & 1) {
                            jas_log("board %u asic %u: reclaim FAILED,"
                                    " incomplete DAG", board, q);
                            reclaim_cache_failed[board][q] = 1;
                        }
                }
            }
        }
    }

    /* ---- Pass C: read remap, DAG check, tail check ---- */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            const jas_coin_ctx *coin = jas_coin_of((int)board, chip);
            uint16_t ready;

            if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                continue;
            ready = reclaim_ready[board][chip];

            /* DAG too large for its memory beyond the reject threshold: no
             * check and no retry (each try cost 6 min and held the other
             * chips); seed.c does not turn it on. */
            if (dag_discarded[board] >> chip & 1) {
                jas_log("board %u asic %u: set aside (DAG too large), no mining"
                        " on this coin", board, chip);
                amput_done[board][chip] = 0;
                continue;
            }

            /* Waiting for its tail: no read remap, no check here;
             * finish_deferred() takes care of it after mining starts. It does
             * not count as a failure (or the retry loop would amputate it). */
            if (dag_deferred[board] >> chip & 1) {
                amput_done[board][chip] = 0;
                continue;
            }

            /*
             * READ remap (arg 0: good banks only) before the check and mining,
             * as the original does after the DAG. The DAG was written into the
             * partial banks (arg 2) but is read only from the good ones.
             */
            config_remap_info((int)board, (uint8_t)chip, 0);

            if (!ready) {
                jas_log("board %u asic %u calc dag timeout ", board, chip);
                failures++;
                continue;
            }

            /*
             * THE ONLY JUDGE: recompute on the processor and compare. A "done"
             * register only says "the hardware has finished", not "what it
             * wrote is right". See core/dram_check.c.
             */
            wait_cache_processor_chip((int)board, chip);
            jas_log("board %u asic %u dag check", board, chip);
            if (dram_check_dag((int)board, (uint8_t)chip, 0) < 0) {
                jas_log("board %u asic %u calc dag error ", board, chip);
                failures++;
                continue;
            }
            jas_log("board %u asic %u calc dag OK! ", board, chip);
            /* RECLAIM: the check above probes the prefix (up to 4/6 of the
             * DAG); the tail written by the processor has its own check. */
            if (jas_cache_reclaim_active() && !reclaim_cache_failed[board][chip]
                && reclaim_front[board][chip] < (uint32_t)(coin->dag_size / 64u)) {
                int r = reclaim_check_tail((int)board, chip, coin,
                                              reclaim_front[board][chip],
                                              reclaim_samples_check());
                if (r != 0)
                    jas_log("board %u asic %u: reclaim tail %s (%d) — mining"
                            " anyway, watch the reject", board, chip,
                            r < 0 ? "NOT VERIFIABLE" : "WITH WRONG NODES", r);
            }
            jas_web_dag_publish(100);
            amput_done[board][chip] = 0;
        }
    }
    return failures;
}

/*
 * "FAST" MODE: the vendor firmware that DOES mine asic2 in ETHW runs
 * original_jasminer_settings=0, hence apply_fast_ethw_memory_path, which
 * NOPs the amputation. Our amputation (close a mesh, then bank 255, on a
 * wrong DAG) lowers `valid`, which moves the cache down into the
 * marginal high rows (cache_start_row = 0xff6 - cache/(valid<<9)) and
 * corrupted asic2. So by default NOTHING is closed (fast);
 * JASRE_AMPUT=stock restores the old behavior.
 */
static int amput_closed(void)
{
    static int f = -1;

    if (f < 0) {
        const char *e = getenv("JASRE_AMPUT");
        f = (e && !strcmp(e, "stock"));
    }
    return f;
}

/*
 * The amputation: close a mesh, then bank 255, on the chips still wrong.
 * Returns 1 if something was closed, in which case a pass must be redone.
 */
static int amputate(void)
{
    unsigned board, chip;
    int change = 0;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            uint8_t cache_row = amput_done[board][chip];

            if (!(ASIC_EN(board) >> chip & 1) || cache_row == 0 || cache_row > MESHES)
                continue;

            jas_trace("board %u asic %u dag_done_flag = %u", board, chip, cache_row);

            if (amput_stage[board][chip] == AMPUT_NO) {
                amput_stage[board][chip] = AMPUT_MESH;
                amput_mesh[board][chip] = cache_row;
                if (amput_closed())
                    dag_remap_data((int)board, (uint16_t)chip, cache_row, 1, AMPUT_MESH);
                jas_trace("##########board %u asic %u close mesh_%u", board, chip, cache_row);
                change = 1;
            } else if (amput_stage[board][chip] == AMPUT_MESH) {
                amput_stage[board][chip] = AMPUT_BANK255;
                if (amput_closed())
                    dag_remap_data((int)board, (uint16_t)chip,
                                   amput_mesh[board][chip], 1, AMPUT_BANK255);
                jas_trace("##########board %u asic %u close bank255_1", board, chip);
                change = 1;
            } else {
                if (amput_closed())
                    dag_remap_data((int)board, (uint16_t)chip,
                                   amput_mesh[board][chip], cache_row, AMPUT_BANK255);
                jas_trace("##########board %u asic %u close bank255_%u", board, chip, cache_row);
                change = 1;
            }
        }
    }
    return change;
}

/*
 * DAG REBUILD, at startup or WHILE MINING.
 *
 * Two callers, two needs:
 *
 *   dag_calc()          startup. All active chips, preceded by the
 *                       preamble reset + config_asic_fix_param + ASIC_START.
 *   dag_calc_mask()   epoch change. ONLY the chips of the mask, and
 *                       WITHOUT the preamble.
 *
 * WHY WITHOUT THE PREAMBLE: config_asic_fix_param and the REG_ASIC_START
 * write act on the whole BOARD; applying them while a chip mines ANOTHER
 * coin would disturb it for nothing. The original does the same: in
 * seed_update_operation's seed_switch >= 2 branch (the epoch change,
 * @0x25e9c) it skips the self-test AND the preamble, cuts the ALUs, calls
 * dag_calc, then sets the registers again. one_pass does its own per-chip
 * soft reset anyway.
 */
static void dag_calc_internal(const uint8_t mask[3], int preamble)
{
    struct timeval start, end;
    unsigned board, chip, resume;

    gettimeofday(&start, NULL);
    dag_during_startup = preamble;
    if (preamble)
        memset(dag_deferred, 0, sizeof(dag_deferred));
    memset(amput_done, 0, sizeof(amput_done));
    memset(amput_stage, 0, sizeof(amput_stage));
    memset(amput_mesh, 0, sizeof(amput_mesh));

    /* The candidate chips: those of the mask, among the active chips. */
    for (board = 0; board < BOARD_NUM; board++)
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                && (mask[board] >> chip & 1))
                amput_done[board][chip] = 1;

    /*
     * REFERENCE SNAPSHOT FOR THE AMPUTATION.
     *
     * dag_remap_data (remap.c) restores the banks OUTSIDE its window from
     * jas_bank_good_ref / jas_bank_partial_ref, "the original state", which
     * the original initializes at reset. Without it, the first amputation
     * restored everything outside the window to zero and the bank map
     * collapsed. The POST-SCAN state (DFT + CBSC) is frozen here, right before
     * the amputation loop.
     */
    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            unsigned k;
            if (!(ASIC_EN(board) >> chip & 1))
                continue;
            for (k = chip * 4080u; k < (chip + 1u) * 4080u; k++) {
                jas_bank_good_ref[board][k]    = jas_bank_good[board][k];
                jas_bank_partial_ref[board][k] = jas_bank_partial[board][k];
            }
        }
    }

    /*
     * RESET, THEN RESTART, per CHIP:
     *
     *   - the reset is done PER CHIP, not per board;
     *   - config_asic_fix_param gives the chip back its fixed parameters,
     *     which the reset has just erased;
     *   - writing 0x0003 to asic_en RESTARTS it.
     *
     * Without the last one, the chip stays stopped: it accepts the cache
     * start (its register 0x300 does go to 0x0100) and computes nothing.
     */
    if (preamble) {
        for (board = 0; board < BOARD_NUM; board++) {
            if (!(BOARD_EN >> board & 1))
                continue;
            for (chip = 0; chip < ASIC_NUM; chip++) {
                uint8_t m = (uint8_t)(1u << chip);

                if (!(ASIC_EN(board) >> chip & 1) || !amput_done[board][chip])
                    continue;
                sl_single_write(m, CMD_BROADCAST, REG_SOFT_RESET, 0, (int)board);
                sl_delay(0, 2000);
                sl_single_write(m, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, (int)board);
            }
            jas_log("board %u soft reset done", board);
            config_asic_fix_param((int)board);
            sl_single_write(0xff, CMD_BOARD_W, REG_ASIC_START, ASIC_EN(board), (int)board);
        }
    }

    for (resume = 0; resume < RETRIES_MAX; resume++) {
        jas_dag_step = (uint8_t)(resume < 3 ? resume : 3);
        if (one_pass() == 0)
            break;
        jas_log("retry_time = %u", resume);
        if (!amputate())
            break;
    }

    for (board = 0; board < BOARD_NUM; board++)
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                && amput_done[board][chip])
                jas_log("board %u asic %u calc dag fail ", board, chip);

    gettimeofday(&end, NULL);
    jas_dag_done = 1;
    jas_log("dag calc time : %ld", (long)(end.tv_sec - start.tv_sec));
}

/* ---------------------------------------------------------------------------
 * IN-PLACE REBUILD AT AN EPOCH CHANGE.
 *
 * At an epoch change, wrapper.c updates the context, the PROCESSOR cache
 * and the header, but the chips still hold the OLD epoch's DAG: ~100 %
 * rejects until the DAG is rebuilt. Without this, the only cure was a full
 * restart (8 to 10 minutes) every ~4.6 days.
 *
 * WHAT THE ORIGINAL FIRMWARE DOES, reproduced here: its
 * seed_update_operation (@0x25e9c) has TWO paths, by
 * dev_info_g.seed_switch. Below 2 it is the full startup (self-tests,
 * DFT, CBSC, dag_calc). From 2 on it SKIPS all that: it cuts the ALUs,
 * calls dag_calc, sets dag_size / header / seed / nonce / target again,
 * does the soft reset 0x830 and turns the ALUs back on mesh by mesh. No
 * restart.
 *
 * OURS IS PER CHIP, AND PER COIN. The original broadcasts to the whole
 * board because it knows a single coin; here an ETHW epoch change must do
 * NOTHING to the chip mining ETC. The mask carries exactly the chips whose
 * epoch moved, and each is built with its own coin context.
 *
 * WHY A DEDICATED THREAD: the fill lasts several minutes; doing it in
 * sl_wrapper_work would freeze the work distribution to ALL the chips.
 * ------------------------------------------------------------------------ */
/*
 * Put chips back to mining after their DAG build: the epoch-change
 * sequence (header, soft reset, refresh closed, ALUs mesh by mesh, purge;
 * see the rebuild thread's comment below). Every write is masked per chip.
 */
static void join_mining(unsigned c, uint8_t m)
{
    uint8_t  en = (uint8_t)(ASIC_EN((int)c) & m);
    uint16_t zeros[64];
    unsigned p, mesh;

    if (!en)
        return;
    *jas_alu_close_flag = 0;
    header_update_operation((int)c, m);

    sl_single_write(en, CMD_BROADCAST, REG_SOFT_RESET, 0, (int)c);
    sl_delay(0, 2000);
    sl_single_write(en, CMD_BROADCAST, REG_SOFT_RESET, 0xffff, (int)c);
    /* Whether 0x830 clears the job registers is unknown: between one extra
     * write and a lost job, forget the memo (same reason as in work.c). */
    jas_work_memo_forget((int)c, m);

    memset(zeros, 0, sizeof(zeros));
    for (p = 0; p < ASIC_NUM; p++) {
        if (!(en >> p & 1))
            continue;

        sl_multi_write((uint8_t)(1u << p), CMD_BROADCAST,
                       REG_MC_REFRESH_EN, zeros, 0x40, (int)c);
        /* Same text as the original and the startup: this line proves the
         * write in the log. */
        jas_log("board[%u] asic[%u] close mc refresh done", c, p);

        for (mesh = 0; mesh < MESHES; mesh++)
            sl_multi_write((uint8_t)(1u << p), CMD_MESH_W(mesh),
                           REG_ALU_EN, jas_alu_en[c][p][mesh], 2, (int)c);

        /* The original's half second between two chips: the hundred and
         * twenty ALUs of one chip restart together, not the board's 360. */
        usleep(500000);
    }

    /* The slots still hold results of the ALUs left on and from before the
     * switch: computed on the OLD DAG, they would count as hardware faults.
     * Purge them, as sl_wrapper_work does after each header push. */
    purge_results((int)c, m);
}

/*
 * The rest of the per-chip startup: write the DAG tail of the chips put on
 * hold (one grouped write per coin, like pass B), check them like pass C,
 * then put them back to mining. Called by the rebuild thread, once the
 * other chips are mining.
 */
void jas_dag_request_rebuild(int board, uint8_t mask);

static void finish_deferred(void)
{
    unsigned c, p, q;

    for (c = 0; c < BOARD_NUM; c++) {
        uint8_t m = dag_deferred[c], done = 0, good = 0, bad = 0;
        struct timeval t1;
        long ms;

        if (!m || !(BOARD_EN >> c & 1))
            continue;

        for (p = 0; p < ASIC_NUM; p++) {
            const jas_coin_ctx *coin = jas_coin_of((int)c, p);
            uint32_t total, fronts[3];
            uint8_t group = 0;

            if (!(m >> p & 1) || (done >> p & 1) || !coin)
                continue;
            total = (uint32_t)(coin->dag_size / 64u);
            for (q = 0; q < ASIC_NUM; q++) {
                const jas_coin_ctx *cq = jas_coin_of((int)c, q);

                fronts[q] = reclaim_front[c][q];
                if ((m >> q & 1) && !(done >> q & 1) && cq
                    && cq->block_number == coin->block_number
                    && memcmp(&cq->seed, &coin->seed, sizeof(coin->seed)) == 0
                    && fronts[q] < total)
                    group |= (uint8_t)(1u << q);
            }
            done |= group | (uint8_t)(1u << p);
            if (!group)
                continue;
            wait_cache_processor_chip((int)c, p);
            if (reclaim_write_tail_group((int)c, coin, fronts, group, NULL) != 0) {
                jas_log("board %u: per-chip startup — DAG tail FAILED for 0x%x", c, group);
                bad |= group;
            }
        }

        for (p = 0; p < ASIC_NUM; p++) {
            const jas_coin_ctx *coin = jas_coin_of((int)c, p);

            if (!(m >> p & 1) || (bad >> p & 1) || !coin)
                continue;
            config_remap_info((int)c, (uint8_t)p, 0);
            wait_cache_processor_chip((int)c, p);
            jas_log("board %u asic %u dag check", c, p);
            if (dram_check_dag((int)c, (uint8_t)p, 0) < 0) {
                jas_log("board %u asic %u calc dag error ", c, p);
                bad |= (uint8_t)(1u << p);
                continue;
            }
            jas_log("board %u asic %u calc dag OK! ", c, p);
            if (reclaim_front[c][p] < (uint32_t)(coin->dag_size / 64u)) {
                int r = reclaim_check_tail((int)c, p, coin, reclaim_front[c][p],
                                              reclaim_samples_check());
                if (r != 0)
                    jas_log("board %u asic %u: reclaim tail %s (%d) — mining"
                            " anyway, watch the reject", c, p,
                            r < 0 ? "NOT VERIFIABLE" : "WITH WRONG NODES", r);
            }
            good |= (uint8_t)(1u << p);
        }

        join_mining(c, good);
        dag_deferred[c] = 0;
        gettimeofday(&t1, NULL);
        ms = (t1.tv_sec - dag_deferred_t0.tv_sec) * 1000L
           + (t1.tv_usec - dag_deferred_t0.tv_usec) / 1000L;
        jas_log("board %u: per-chip startup — chips 0x%x mining %ld ms after"
                " being put on hold%s", c, good, ms,
                bad ? "; WRONG DAG elsewhere, rebuild requested" : "");
        if (bad)
            jas_dag_request_rebuild((int)c, bad);
    }
    /* Writing the tail took the bar through 90-99 %: set it back to 100, or
     * the pool watchdog (jasminer.sh) would stay suspended. */
    jas_web_dag_publish(100);
}

static pthread_mutex_t dag_rebuild_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t         dag_rebuild_request[3];

void jas_dag_request_rebuild(int board, uint8_t mask)
{
    if (board < 0 || board >= 3 || mask == 0)
        return;
    pthread_mutex_lock(&dag_rebuild_mutex);
    dag_rebuild_request[board] |= mask;
    pthread_mutex_unlock(&dag_rebuild_mutex);
}

static void *dag_rebuild_thread(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t m[3];
        unsigned c, failures, total = 0;
        int to_do = 0;

        usleep(500000);

        /* First the rest of the per-chip startup, once mining has started. */
        if (dag_start_done && (dag_deferred[0] | dag_deferred[1] | dag_deferred[2]))
            finish_deferred();

        pthread_mutex_lock(&dag_rebuild_mutex);
        for (c = 0; c < 3; c++) {
            m[c] = dag_rebuild_request[c];
            dag_rebuild_request[c] = 0;
            if (m[c])
                to_do = 1;
        }
        pthread_mutex_unlock(&dag_rebuild_mutex);
        if (!to_do)
            continue;

        for (c = 0; c < BOARD_NUM; c++) {
            unsigned p;
            if (!m[c])
                continue;
            for (p = 0; p < ASIC_NUM; p++)
                if (m[c] >> p & 1)
                    total++;
            jas_log("dag epoch: board %u chips 0x%x — IN-PLACE DAG rebuild."
                    " These chips stop during the fill; the others keep mining.", c, m[c]);
            /*
             * TWO WORDS, NOT ONE. A mesh's enable mask spans 0x40 AND 0x41:
             * sixteen ALUs in the first word, fourteen in the second, thirty
             * per mesh. Writing only one address leaves 14 ALUs per mesh (56
             * per chip) mining during the rebuild, against the fill and while
             * one_pass resets the memory controller under them. The original
             * writes both words.
             */
            {
                uint16_t zero2[2] = { 0, 0 };

                sl_multi_write(m[c], CMD_BROADCAST, REG_ALU_EN, zero2, 2, (int)c);
            }
        }

        /*
         * The pool watchdog goes to standby until the DAG is at 100 % (see
         * etc/init.d/jasminer.sh): otherwise it would see "works" stall during
         * the fill and restart the machine in the middle.
         */
        jas_web_dag_publish(0);
        failures = dag_calc_mask(m);

        for (c = 0; c < BOARD_NUM; c++) {
            uint8_t en;

            if (!m[c])
                continue;
            en = (uint8_t)(ASIC_EN((int)c) & m[c]);
            if (!en)
                continue;

            /*
             * WHY THE TAIL OF THE SEQUENCE IS REDONE HERE, and not left to
             * header_update_operation.
             *
             * one_pass() re-arms the memory refresh (0xc40 <-
             * jas_mc_refresh_en) because the fill lasts minutes during which
             * nobody reads the memory. Once the DAG is built it is the
             * REVERSE: mining reads the memory constantly and refreshes it by
             * itself, and the refresh engine only steals bandwidth from ALUs
             * that already lack it (the memory path is SATURATED: 84 ALUs run
             * 5.6 % faster than 120). Left on, it cost 31 % of the hashrate
             * with an UNCHANGED reject rate. The startup closes it in
             * close_refresh_and_reopen_alu (seed.c); the original
             * closes it in BOTH cases, its tail being shared.
             *
             * THE ORDER IS NOT NEGOTIABLE:
             *   1. soft reset 0x830 FIRST: it puts the memory controller back
             *      to its defaults, so anything written before would be erased;
             *   2. THEN the zeros in 0xc40, chip by chip;
             *   3. ONLY THEN the ALUs back on, mesh by mesh.
             * Turning the ALUs on before closing would restart the chip in the
             * degraded state, and write to the memory controller while a
             * hundred and twenty ALUs hammer it.
             *
             * So alu_close_flag stays at ZERO: header_update_operation only
             * does the DAG size, header, target and nonce ranges, and we do
             * the rest (which also removes the race between this thread and
             * sl_wrapper_work over that global flag).
             *
             * ALL THE WRITES BELOW ARE MASKED PER CHIP: the chip mining
             * another coin is never touched.
             */
            join_mining(c, m[c]);
        }
        jas_web_dag_publish(100);

        if (failures)
            jas_log("dag epoch: FAILED — %u chip(s) out of %u without a DAG"
                    " after the rebuild. The epoch watchdog will take over with"
                    " a restart.", failures, total);
        else
            jas_log("dag epoch: rebuild done, %u chip(s) resume mining without"
                    " a restart.", total);
    }
    return NULL;
}

void jas_dag_rebuild_start(void)
{
    static int started;
    pthread_t t;

    if (started)
        return;
    started = 1;
    if (pthread_create(&t, NULL, dag_rebuild_thread, NULL) != 0)
        jas_log("dag epoch: cannot start the rebuild thread — falling back on"
                " the watchdog restart");
}

/* Startup: all active chips, with the preamble. */
void dag_calc(void)
{
    uint8_t all[3];
    unsigned c;

    jas_fan_mining(0);              /* no more mining: cold state allowed */
    for (c = 0; c < 3; c++)
        all[c] = 0xff;
    dag_calc_internal(all, 1);
}

/*
 * Epoch change: only the chips of the mask, without the preamble.
 * Returns the number of chips of the mask that did NOT complete (0 = all
 * fine).
 */
unsigned dag_calc_mask(const uint8_t mask[3])
{
    unsigned board, chip, remaining = 0;

    dag_calc_internal(mask, 0);
    for (board = 0; board < BOARD_NUM; board++)
        for (chip = 0; chip < ASIC_NUM; chip++)
            if ((BOARD_EN >> board & 1) && (ASIC_EN(board) >> chip & 1)
                && (mask[board] >> chip & 1) && amput_done[board][chip])
                remaining++;
    return remaining;
}
