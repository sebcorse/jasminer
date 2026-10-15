/*
 * Analysis of the crossbar scan (CBSC).
 *
 * This decides which memory banks are usable, hence the capacity announced
 * to the chip. Each ALU tries to reach each of the 4080 banks; the raw
 * result is a 120 x 4080 bit matrix per ASIC, which this file goes
 * through. The path from an ALU to a bank crosses several stages:
 *
 *   ALU -> compute node -> CB 6x6 -> CB 5x5 -> 1x34 -> bank
 *
 * and the analysis looks for where a break lies by the scale at which the
 * failures cluster: a quarter of 1020 banks, a group of 170, a group of
 * 34, or a single bank.
 *
 * THE SINGLE-BANK RULE: once link breaks are ruled out, any bank an ALU
 * does not reach is declared bad, for everyone. The verdicts vary from one
 * startup to the next because the scan itself is unreliable, but the rule
 * must stay (see QUORUM below). Around it: the gate, the ETHW floor, the
 * group rules the reference binary never runs, the coverage threshold.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"
#include "sl_transfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_ALU        120u
#define N_BANK       0xff0u    /* 4080 */
#define N_MESH       4u
#define ALUS_PER_MESH 30u

#define GRP_1020     1020u     /* a quarter of the bank space */
#define GRP_170      170u
#define GRP_34       34u
#define N_1020       (N_BANK / GRP_1020)   /* 4   */
#define N_170        (N_BANK / GRP_170)    /* 24  */
#define N_34         (N_BANK / GRP_34)     /* 120 */

/* The original's thresholds. */
#define ALU_FLOOR    0x820u    /* floor of an ALU's reachability threshold */
#define ALU_MIN_MEAN 0x821u
#define Q_MIN        0x91u     /* 145 accesses expected in a quarter */
#define Q_BANK_MIN   0x2fd0u   /* 12240 accesses summed over the quarter */
#define G170_MIN     0x11u     /* 17                                         */
#define G34_MIN      10u

/* How many epochs ahead the capacity target looks (see
 * cbsc_coverage_threshold): 8 epochs ~= 36 days of DAG growth. */
#define MARGIN_EPOCHS 8ull

/* Analysis matrices. Nearly three megabytes for the largest: the original
 * puts them on the stack, we allocate them; a thread's stack has no business
 * carrying that. */
struct cbsc_work {
    uint16_t (*reaches)[N_BANK];       /* [alu][bank]: 1 if reached */
    uint16_t (*per_1020)[N_1020];
    uint16_t (*per_170)[N_170];
    uint16_t (*per_34)[N_34];
    uint16_t bank_alu_count[N_BANK];   /* how many ALUs reach the bank */
    uint16_t alu_bank_count[N_ALU];    /* how many banks the ALU reaches */
};

static int cbsc_alloc(struct cbsc_work *w)
{
    w->reaches  = calloc(N_ALU, sizeof(*w->reaches));
    w->per_1020 = calloc(N_ALU, sizeof(*w->per_1020));
    w->per_170  = calloc(N_ALU, sizeof(*w->per_170));
    w->per_34   = calloc(N_ALU, sizeof(*w->per_34));
    memset(w->bank_alu_count, 0, sizeof(w->bank_alu_count));
    memset(w->alu_bank_count, 0, sizeof(w->alu_bank_count));
    return (w->reaches && w->per_1020 && w->per_170 && w->per_34) ? 0 : -1;
}

static void cbsc_free(struct cbsc_work *w)
{
    free(w->reaches); free(w->per_1020); free(w->per_170); free(w->per_34);
}

/* Turns an ALU off in the enable mask. */
static void alu_disable(int thread_id, unsigned asic, unsigned alu)
{
    unsigned mesh = alu / ALUS_PER_MESH;
    unsigned pos  = alu % ALUS_PER_MESH;

    jas_alu_en[thread_id][asic][mesh][pos >> 4] &= (uint16_t)~(1u << (pos & 0xf));
}

/* Marks a range of banks as bad. */
static void banks_mark_bad(int thread_id, unsigned asic, unsigned first, unsigned count)
{
    unsigned base = asic * N_BANK + first, k;

    for (k = 0; k < count; k++)
        jas_bank_bad[thread_id][base + k] = 1;
}

/* Goes through the raw scan of one ASIC. */
static void cbsc_tally(int thread_id, unsigned asic, struct cbsc_work *w)
{
    unsigned alu, bank;

    for (alu = 0; alu < N_ALU; alu++) {
        for (bank = 0; bank < N_BANK; bank++) {
            const uint16_t *row = jas_cb_sc_result[thread_id][asic][alu];
            if (!(row[bank >> 4] >> (bank & 0xf) & 1))
                continue;
            w->alu_bank_count[alu]++;
            w->bank_alu_count[bank]++;
            w->per_1020[alu][bank / GRP_1020]++;
            w->per_170[alu][bank / GRP_170]++;
            w->per_34[alu][bank / GRP_34]++;
            w->reaches[alu][bank] = 1;
        }
    }
}

/*
 * Returns the threshold below which an ALU is judged faulty: four fifths of
 * the average, with a floor. The floor protects badly damaged chips, where
 * the average itself would already be low.
 */
static unsigned cbsc_alu_threshold(const struct cbsc_work *w)
{
    unsigned long sum = 0;
    unsigned mean, alu;

    for (alu = 0; alu < N_ALU; alu++)
        sum += w->alu_bank_count[alu];
    mean = (unsigned)(sum / N_ALU);

    if (mean < ALU_MIN_MEAN)
        return ALU_FLOOR;
    return (mean * 8) / 10;
}

/*
 * OUR OWN GATE (not the vendor's; once a patch on the stock runtime, now
 * read here by cbsc_alu_gate). The threshold above (four fifths of the
 * average, floor 2080) is the stock one and alone decides the DISABLING of
 * ALUs. The gate PREVENTS an ALU under memory_cbsc_alu_veto_min from
 * MARKING BANKS BAD; the ALU stays active and keeps mining. Reason: the
 * three condemnation rules (window of 170, window of 34, single bank) each
 * decide from ONE ALU's coverage, so an ALU that fails its scan would take
 * healthy banks down with it. Mesh closure stays stock.
 *
 *     memory_cbsc_alu_veto      "stock" or "gate"
 *     memory_cbsc_alu_veto_min  3840, capped at 4080
 */
static unsigned cbsc_alu_gate(void)
{
    static int loaded;
    static unsigned threshold;
    FILE *f;
    char line[512];

    if (loaded)
        return threshold;
    loaded = 1;
    threshold = 0;                                  /* stock: no gate */

    /*
     * OVERRIDE FROM THE ENVIRONMENT, to try the gate without touching the
     * machine's jasminer.json in flash:
     *
     *     JASRE_ALU_GATE=0     gate lifted, stock threshold only (4/5 of avg)
     *     JASRE_ALU_GATE=<n>   gate at n banks
     *
     * Whether ALUs kept this way compute RIGHT is judged by the chip's
     * acceptance rate, not its hashrate.
     */
    {
        const char *e = getenv("JASRE_ALU_GATE");

        if (e) {
            threshold = (unsigned)strtoul(e, NULL, 10);
            if (threshold > N_BANK)
                threshold = N_BANK;
            jas_log("CBSC veto per ALU: JASRE_ALU_GATE override (threshold %u)", threshold);
            return threshold;
        }
    }

    f = fopen(jas_config_file(), "r");
    if (!f)
        return threshold;
    {
        int gate = 0;
        unsigned minimum = 3840;

        while (fgets(line, sizeof(line), f)) {
            if (strstr(line, "\"memory_cbsc_alu_veto\"") && strstr(line, "gate"))
                gate = 1;
            {
                char *p = strstr(line, "\"memory_cbsc_alu_veto_min\"");
                if (p) {
                    p = strchr(p, ':');
                    if (p) minimum = (unsigned)strtoul(p + 1, NULL, 10);
                }
            }
        }
        if (minimum > N_BANK) minimum = N_BANK;
        threshold = gate ? minimum : 0;
    }
    fclose(f);
    jas_log("CBSC veto per ALU: %s (threshold %u)",
            threshold ? "gate" : "stock", threshold);
    return threshold;
}

/*
 * ETHW VIABILITY FLOOR, per chip. A chip that does not reach this number of
 * banks in the normal scan cannot build the ETHW DAG anyway; for it,
 * keeping marginal banks (gate) or trying the fallback is useless for ETHW
 * and BREAKS ETC (corrupt cache, or a fallback that wipes it to 0). So both
 * are cancelled for these chips, back to the original firmware's ~3600
 * solid banks: clean ETC, no ETHW.
 *
 * Value = memory_cbsc_alu_veto_min (default 3840), consistent with the
 * gate's threshold; JASRE_ETHW_FLOOR overrides it for trials.
 */
unsigned cbsc_ethw_floor(void)
{
    static int loaded;
    static unsigned floor = 3840;
    FILE *f;
    char line[512];
    const char *e;

    if (loaded)
        return floor;
    loaded = 1;

    e = getenv("JASRE_ETHW_FLOOR");
    if (e) {
        floor = (unsigned)strtoul(e, NULL, 0);
        if (floor > N_BANK) floor = N_BANK;
        return floor;
    }
    f = fopen(jas_config_file(), "r");
    if (!f)
        return floor;
    while (fgets(line, sizeof(line), f)) {
        char *p = strstr(line, "\"memory_cbsc_alu_veto_min\"");
        if (p && (p = strchr(p, ':')))
            floor = (unsigned)strtoul(p + 1, NULL, 10);
    }
    fclose(f);
    if (floor > N_BANK) floor = N_BANK;
    return floor;
}

/*
 * Should the group stages (170 / 34) condemn? By DEFAULT NO: the reference
 * binary makes them unreachable (detailed proof in cbsc_judge_asic).
 * JASRE_CBSC_GROUPS=1 restores the old behavior, to allow the A/B.
 */
static int jas_cbsc_groups(void)
{
    static int loaded, val;

    if (!loaded) {
        const char *e = getenv("JASRE_CBSC_GROUPS");

        val = (e && *e && *e != '0');
        loaded = 1;
    }
    return val;
}

/*
 * CONDEMNATION QUORUM, AND WHY IT MUST STAY AT 1.
 *
 * How many ALUs must miss a bank for it to be condemned. The original
 * condemns from the FIRST one, which looks harsh, but a quorum of 2 froze
 * a machine (mains power cut needed). The DAG is striped over ALL the valid
 * banks and each ALU reads it pseudo-randomly: a bank kept while one ALU
 * cannot reach it WILL be accessed by that ALU, the transaction will never
 * be acknowledged, and the ARM core freezes.
 *
 * So the varying verdicts between startups do not mean over-condemnation:
 * the SCAN is unreliable (an ALU that fails its scan wrongly declares a
 * bank unreachable). The right lever is to vote over SEVERAL SCANS OF THE
 * SAME ALU, never between different ALUs.
 *
 * The setting stays readable, but it is CAPPED AT 1: any other value is
 * refused and reported.
 */
static unsigned jas_cbsc_votes(void)
{
    static int loaded;
    static unsigned val;

    if (!loaded) {
        const char *e = getenv("JASRE_CBSC_VOTES");
        unsigned request = (e && *e) ? (unsigned)strtoul(e, NULL, 10) : 1u;

        if (request != 1u)
            jas_log("CBSC: quorum %u REFUSED (it freezes the chip) — staying"
                    " at 1", request);
        val = 1u;
        loaded = 1;
    }
    return val;
}

static void cbsc_judge_asic(int thread_id, unsigned asic, struct cbsc_work *w)
{
    unsigned threshold = cbsc_alu_threshold(w);
    unsigned gate = cbsc_alu_gate();
    unsigned floor = cbsc_ethw_floor();
    unsigned long sum = 0;
    unsigned average;
    int gate_applies;
    unsigned alu, q, k, bank;

    /*
     * THE GATE DOES NOT APPLY HERE: it is a veto EXERCISED BY a weakly
     * covering ALU (it may no longer condemn banks), not a veto ON the ALU.
     * The disabling threshold stays the stock one, four fifths of the
     * average. In the patch's own words: "gate = an ALU whose bank-access
     * count is below memory_cbsc_alu_veto_min may no longer mark banks bad by
     * any of the three rules. Thresholds, ALU disabling and mesh closure stay
     * stock." Applying it to the disabling threshold turns off dozens of
     * healthy ALUs.
     */

    /*
     * PER-CHIP RULE: average of banks reached in the scan. Below the ETHW
     * floor, the chip is ETC-only and the gate is CANCELLED (full condemnation
     * -> ~3600 solid banks, clean cache). Above it, the gate protects from
     * stochastic misses and allows ETHW.
     */
    /* Marks avoided by following the reference (see below). */
    unsigned avoided_170 = 0, avoided_34 = 0;
    /* How many ALUs miss each bank: the vote, see below. */
    uint16_t *misses = calloc(N_BANK, sizeof(*misses));

    if (!misses) {
        jas_log("board %d asic %u: CBSC, not enough memory for the vote",
                thread_id, asic);
        return;
    }

    for (alu = 0; alu < N_ALU; alu++)
        sum += w->alu_bank_count[alu];
    average = (unsigned)(sum / N_ALU);
    gate_applies = (gate > 0) && (average >= floor);
    if (average < floor)
        jas_log("board %d asic %u: bad asic, etc only (%u banks < %u)",
                thread_id, asic, average, floor);

    for (alu = 0; alu < N_ALU; alu++) {
        unsigned mesh = alu / ALUS_PER_MESH;
        int link_bad = 0;
        /* The gate applies per chip: only if the chip is viable for ETHW
         * (average >= floor). Otherwise full condemnation, as originally. */
        int can_condemn = gate_applies ? (w->alu_bank_count[alu] >= gate) : 1;

        /* The ALU itself does not reach enough banks. ALU 0 is spared: the
         * original treats it separately, probably because it serves as the
         * scan's reference. */
        if (w->alu_bank_count[alu] < threshold && alu != 0) {
            alu_disable(thread_id, asic, alu);
            jas_trace("CBSC failing alu: asic %u mesh %u alu %u (%u banks < %u)",
                    asic, mesh, alu, w->alu_bank_count[alu], threshold);
            continue;
        }

        /* Break at the scale of a quarter: either the quarter's banks are
         * unreachable for everyone, or this ALU's link to this mesh is cut.
         * The sum of the accesses over the quarter decides. */
        for (q = 0; q < N_1020; q++) {
            unsigned long total = 0;

            if (w->per_1020[alu][q] >= Q_MIN)
                continue;

            for (k = 0; k < GRP_1020; k++)
                total += w->bank_alu_count[q * GRP_1020 + k];

            if (total < Q_BANK_MIN) {
                banks_mark_bad(thread_id, asic, q * GRP_1020, GRP_1020);
                jas_trace("CBSC dead quarter: asic %u mesh %u alu %u, 1020 banks",
                        asic, q, alu);
            } else {
                alu_disable(thread_id, asic, alu);
                jas_trace("CBSC broken link: asic %u alu %u to mesh %u (%s)",
                        asic, alu, q,
                        (mesh == q) ? "calcnode to CB 6x6" : "rtnode to mesh");
                link_bad = 1;
            }
        }

        if (alu == 0)
            link_bad = 0;
        if (link_bad)
            continue;

        /*
         * THE GROUP RULES (170 and 34) DO NOT RUN IN THE REFERENCE FIRMWARE
         * (the patched stock binary). Its disassembly shows:
         *
         *   0x2022e  b.w 0x29650   (code cave A: gate 3840)
         *   0x29664  b.w 0x20232   -> 0x20232  b.w 0x29680  (cave B)
         *   0x29688  b.w 0x2048a   -> jumps STRAIGHT to the per-bank scan
         *
         * The stretch 0x20236-0x20244 is skipped, yet it holds the ONLY
         * entry of the "170" block (`0x20244 b.n 0x203d0`), and the only
         * entry of the "34" block (`0x20402 b.n 0x2043a`) lies inside the
         * "170" block: both group stages are UNREACHABLE. Running them
         * condemns banks in batches the reference never condemns, and
         * fewer good banks mean a deeper DAG in each bank, reaching the
         * marginal high rows: rejects.
         *
         * JASRE_CBSC_GROUPS=1 restores them (control A/B). By default we
         * follow the reference, BUT count what the rules WOULD have condemned.
         */
        for (k = 0; can_condemn && k < N_170; k++)
            if (w->per_170[alu][k] < G170_MIN) {
                if (jas_cbsc_groups()) {
                    banks_mark_bad(thread_id, asic, k * GRP_170, GRP_170);
                    jas_trace("CBSC CB 6x6: asic %u alu %u group %u, 170 banks",
                            asic, alu, k);
                } else {
                    avoided_170 += GRP_170;
                }
            }

        /* CB 5x5 to 1x34 stage: groups of 34, but only if the stage above is
         * healthy; otherwise the same fault would be counted twice. */
        for (k = 0; can_condemn && k < N_34; k++)
            if (w->per_34[alu][k] < G34_MIN && w->per_170[alu][k / 5] > G170_MIN - 1) {
                if (jas_cbsc_groups()) {
                    banks_mark_bad(thread_id, asic, k * GRP_34, GRP_34);
                    jas_trace("CBSC CB 5x5: asic %u alu %u group %u, 34 banks",
                            asic, alu, k);
                } else {
                    avoided_34 += GRP_34;
                }
            }

        /*
         * And finally, bank by bank. This is the harshest rule: any bank THIS
         * ALU does not reach is condemned for all the others. See the file
         * header: it is the one that condemns healthy banks when a single ALU
         * fails its scan.
         */
        for (bank = 0; can_condemn && bank < N_BANK; bank++)
            if (!w->reaches[alu][bank] &&
                !jas_bank_bad[thread_id][asic * N_BANK + bank])
                misses[bank]++;        /* we VOTE, we do not condemn yet */
    }

    /*
     * THE VOTE: count how many ALUs miss each bank, and condemn only beyond
     * the quorum. The quorum is capped at 1 (see jas_cbsc_votes), i.e. the
     * original behavior; the histogram printed below still shows, in ONE
     * run, what each quorum would change.
     */
    {
        unsigned quorum = jas_cbsc_votes();
        unsigned h1 = 0, h2 = 0, h35 = 0, h610 = 0, hplus = 0, h_all = 0;
        unsigned condemned = 0, spared = 0;

        for (bank = 0; bank < N_BANK; bank++) {
            unsigned m = misses[bank];

            if (!m)
                continue;
            if      (m == 1)        h1++;
            else if (m == 2)        h2++;
            else if (m <= 5)        h35++;
            else if (m <= 10)       h610++;
            else if (m >= N_ALU)    h_all++;
            else                    hplus++;

            if (m >= quorum) {
                jas_bank_bad[thread_id][asic * N_BANK + bank] = 1;
                condemned++;
            } else {
                spared++;
            }
        }

        if (condemned || spared)
            jas_log("board %d asic %u CBSC vote (quorum %u): %u condemned, %u"
                    " spared; missed by 1 ALU=%u 2=%u 3-5=%u 6-10=%u 11+=%u"
                    " all=%u",
                    thread_id, asic, quorum, condemned, spared,
                    h1, h2, h35, h610, hplus, h_all);
    }
    free(misses);

    /* Puts a figure on the gain: group marks avoided (duplicates included: the
     * same bank can be targeted by several ALUs; this is an UPPER bound). */
    if (!jas_cbsc_groups() && (avoided_170 || avoided_34))
        jas_log("board %d asic %u: group rules aligned on the reference — %u"
                " 170-marks + %u 34-marks avoided (upper bound)",
                thread_id, asic, avoided_170, avoided_34);
}

/* Carries the verdicts over to the bank states and the memory masks. */
static void cbsc_commit(int thread_id, unsigned asic)
{
    unsigned bank;

    for (bank = 0; bank < N_BANK; bank++) {
        unsigned idx = asic * N_BANK + bank;
        unsigned q, word;
        uint16_t clear;

        if (!jas_bank_bad[thread_id][idx] || !jas_bank_good[thread_id][idx])
            continue;

        jas_bank_good[thread_id][idx] = 0;
        jas_bank_partial[thread_id][idx] = 0;
        jas_map_cbsc_killed[thread_id][idx] = 1;   /* diag: DFT-good, killed here */

        q    = bank / GRP_1020;
        word = (bank % GRP_1020) >> 4;
        clear = (uint16_t)~(1u << (bank % GRP_1020 & 0xf));

        jas_mc_en[thread_id][asic][q][word]         &= clear;
        jas_mc_refresh_en[thread_id][asic][q][word] &= clear;
        jas_mc_monitor_en[thread_id][asic][q][word] &= clear;
    }
}

/*
 * Record how many banks survive, in total and per mesh: dram_check_dag
 * derives its sampling step from good_bank_num_cbsc (zero would make it
 * refuse to judge the DAG), and the per-mesh table answers "how many would
 * remain if this mesh were closed?", which it asks when dag_calc has
 * amputated.
 */
static void cbsc_count(int thread_id, unsigned asic)
{
    unsigned bank, total = 0;
    unsigned per_quarter[4] = { 0, 0, 0, 0 };
    unsigned rank = (unsigned)thread_id * 3u + asic;

    for (bank = 0; bank < N_BANK; bank++)
        if (jas_bank_partial[thread_id][asic * N_BANK + bank]) {
            total++;
            per_quarter[bank / GRP_1020]++;
        }

    jas_good_bank_num_cbsc[rank] = (uint16_t)total;
    for (bank = 0; bank < 4; bank++)
        jas_good_bank_num_cbsc_mesh[rank * 4u + bank] = (uint16_t)per_quarter[bank];

    jas_log("board[%d] asic[%u] good_bank_num_cbsc:%u  (mesh %u/%u/%u/%u)",
            thread_id, asic, total,
            per_quarter[0], per_quarter[1], per_quarter[2], per_quarter[3]);
}

/*
 * POLICY "keep the well-covered banks" (JASRE_COV_KEEP, 0 = disabled,
 * the default).
 *
 * The condemnation rules each decide from ONE ALU: an ALU that misses a
 * group condemns it for everyone, even when most ALUs reach it (e.g. a
 * 170-bank group reached by 72 ALUs out of 120), and the loss can take the
 * chip under the ETHW capacity threshold. Here the banks the CBSC condemned
 * but that ENOUGH ALUs reach (bank_alu_count >= keep_min) are
 * REHABILITATED: bad is set back to 0, and cbsc_commit only turns a bank
 * off if bad AND good. Banks with dead memory (DFT verdict: good already
 * 0) are not resurrected.
 */
static void cbsc_keep_covered(int thread_id, unsigned asic, const struct cbsc_work *w,
                              unsigned keep_min)
{
    unsigned bank, rehab = 0;

    if (keep_min == 0)
        return;

    for (bank = 0; bank < N_BANK; bank++) {
        unsigned idx = asic * N_BANK + bank;
        if (jas_bank_bad[thread_id][idx] &&
            jas_bank_good[thread_id][idx] &&
            w->bank_alu_count[bank] >= keep_min) {
            jas_bank_bad[thread_id][idx] = 0;
            rehab++;
        }
    }
    if (rehab)
        jas_log("CBSC keep-covered: asic %u, %u banks restored (>= %u ALU)",
                asic, rehab, keep_min);
}

/*
 * COIN-DRIVEN COVERAGE THRESHOLD: JASRE_COV_FLOOR, INACTIVE BY DEFAULT.
 *
 * A bank "good" at the DFT is not necessarily reached by all 120 ALUs; the
 * ALUs that miss it read garbage there. On a chip whose histogram is binary
 * (3761 banks at 120 ALUs, 163 at exactly 72), dropping those 163 banks
 * took the reject from ~6 % to ~2 %, and an ALU at 100 % rejects back to
 * normal.
 *
 * It cannot always be active: removing banks shrinks each row, so the DAG
 * needs more rows. In ETHW only ~30 banks of margin remain and the DAG
 * would no longer fit; in ETC the floor drops to about 2100 and they can
 * go. Hence one rule, per chip and per coin:
 *
 *     target    = ⌈(dag_size + cache_size) / (0xff6 × 512)⌉
 *     threshold = the highest S such that kept(coverage ≥ S) ≥ target
 *
 * (the cache counts in the target: it stays resident during the build).
 *
 * JASRE_COV_FLOOR absent or 0 = inactive; "auto" = threshold computed as
 * above; a number = forced threshold, refused if it goes under the target.
 * Only banks already candidates are touched; a bank dead at the DFT stays
 * so.
 */
static int cov_floor_mode(unsigned *force)
{
    static int loaded, mode;
    static unsigned value;

    if (!loaded) {
        const char *e = getenv("JASRE_COV_FLOOR");

        loaded = 1;
        if (e && *e) {
            if (!strcmp(e, "auto")) {
                mode = 1;
            } else {
                unsigned v = (unsigned)strtoul(e, NULL, 10);

                if (v == 0) {
                    mode = 0;
                } else if (v < GRP_34 || v > N_ALU) {
                    jas_log("JASRE_COV_FLOOR=%u outside [%u..%u], ignored",
                            v, GRP_34, N_ALU);
                    mode = 0;
                } else {
                    mode = 2;
                    value = v;
                }
            }
        }
    }
    if (force)
        *force = value;
    return mode;
}

/*
 * COVERAGE RECOMPUTED OVER THE LIVE ALUS ONLY: w->bank_alu_count, set by
 * cbsc_tally before any verdict, counts all 120 ALUs, including those
 * cbsc_judge_asic has just turned off. An ALU turned off never reads the
 * DAG; counting it would make healthy banks look poorly covered (hundreds
 * of them on a chip with dozens of ALUs off). So the count is redone from
 * w->reaches, filtered by jas_alu_en.
 */
static void live_coverage(int thread_id, unsigned asic,
                               const struct cbsc_work *w, uint16_t *cov)
{
    unsigned alu, bank;

    memset(cov, 0, N_BANK * sizeof(*cov));
    for (alu = 0; alu < N_ALU; alu++) {
        unsigned mesh = alu / ALUS_PER_MESH, pos = alu % ALUS_PER_MESH;

        if (!(jas_alu_en[thread_id][asic][mesh][pos >> 4] >> (pos & 0xf) & 1))
            continue;                     /* off: it will read nothing */
        for (bank = 0; bank < N_BANK; bank++)
            if (w->reaches[alu][bank])
                cov[bank]++;
    }
}

static void cbsc_coverage_threshold(int thread_id, unsigned asic,
                                  const struct cbsc_work *w)
{
    uint16_t *cov;
    const uint64_t row = (uint64_t)0xff6u * 512u;   /* 2,092,032 B per bank */
    const jas_coin_ctx *coin;
    unsigned hist[N_ALU + 1];
    unsigned bank, candidates = 0, target, kept, threshold = 0, removed = 0, force = 0;
    unsigned c;
    int mode = cov_floor_mode(&force);
    int S;

    if (!mode)
        return;                            /* default: nothing, not even a malloc */

    coin = jas_coin_of(thread_id, asic);
    if (!coin || !coin->dag_size) {
        jas_log("COV_FLOOR asic %u: no coin context, ignored", asic);
        return;
    }

    cov = calloc(N_BANK, sizeof(*cov));
    if (!cov) {
        jas_log("COV_FLOOR asic %u: not enough memory, ignored", asic);
        return;
    }
    live_coverage(thread_id, asic, w, cov);
    /*
     * THE TARGET LOOKS AHEAD OF THE DAG'S GROWTH: the threshold is computed
     * at startup, but the DAG grows by 2^23 bytes per epoch (the cache by
     * 2^17), about one bank every ~1.1 days; a machine running for weeks
     * would end up under the real floor silently. So the target is
     * MARGIN_EPOCHS epochs ahead. Side effect, intended: in ETHW, where the
     * margin is already zero, the target is unreachable and nothing is
     * removed.
     */
    target = (unsigned)((coin->dag_size + coin->cache_size
                        + MARGIN_EPOCHS * (8388608ull + 131072ull)
                        + row - 1) / row);

    /* Histogram of the coverages, over the banks still candidates only. */
    memset(hist, 0, sizeof(hist));
    for (bank = 0; bank < N_BANK; bank++) {
        unsigned idx = asic * N_BANK + bank;

        if (!jas_bank_good[thread_id][idx] || jas_bank_bad[thread_id][idx])
            continue;
        c = cov[bank];
        if (c > N_ALU)
            c = N_ALU;
        hist[c]++;
        candidates++;
    }

    if (candidates <= target) {
        jas_log("COV_FLOOR asic %u: %u candidate banks <= target %u — no"
                " margin, nothing removed", asic, candidates, target);
        free(cov);
        return;
    }

    /* The highest threshold that still leaves enough banks for the coin. */
    kept = 0;
    for (S = (int)N_ALU; S >= 0; S--) {
        kept += hist[S];
        if (kept >= target) {
            threshold = (unsigned)S;
            break;
        }
    }

    if (mode == 2) {                       /* forced threshold: check it holds */
        unsigned rest = 0;

        for (S = (int)force; S <= (int)N_ALU; S++)
            rest += hist[S];
        if (rest < target) {
            jas_log("COV_FLOOR asic %u: forced threshold %u would leave only %u"
                    " banks for a target of %u — REFUSED (the computed"
                    " threshold would be %u)", asic, force, rest, target, threshold);
            free(cov);
            return;
        }
        threshold = force;
    }

    if (threshold == 0) {
        jas_trace("COV_FLOOR asic %u: threshold 0, nothing to remove", asic);
        free(cov);
        return;
    }

    for (bank = 0; bank < N_BANK; bank++) {
        unsigned idx = asic * N_BANK + bank;

        if (!jas_bank_good[thread_id][idx] || jas_bank_bad[thread_id][idx])
            continue;
        if (cov[bank] < threshold) {
            jas_bank_bad[thread_id][idx] = 1;   /* cbsc_commit turns it off */
            removed++;
        }
    }

    jas_log("COV_FLOOR asic %u: coin with %llu MiB of DAG -> target %u banks;"
            " coverage threshold >= %u (%s) -> %u removed, %u kept (margin %d)",
            asic, (unsigned long long)(coin->dag_size >> 20), target, threshold,
            mode == 1 ? "auto" : "forced", removed, candidates - removed,
            (int)(candidates - removed) - (int)target);
    free(cov);
}

void crossbar_check_analysis(int thread_id)
{
    struct cbsc_work w;
    unsigned asic;
    unsigned keep_min = 0, cov_mask = 0;
    const char *cov_env  = getenv("JASRE_COV_KEEP");
    const char *cov_asic = getenv("JASRE_COV_ASIC");

    if (cov_env)
        keep_min = (unsigned)strtoul(cov_env, NULL, 10);
    if (cov_asic)
        cov_mask = (unsigned)strtoul(cov_asic, NULL, 0);

    /* Bounds: below a CBSC group size (34) the condemnation would be wiped
     * out; beyond N_ALU, a meaningless value. Out of bounds => disabled. */
    if (keep_min != 0 && (keep_min < 34u || keep_min > N_ALU)) {
        jas_log("JASRE_COV_KEEP=%u outside [34..%u], ignored", keep_min, N_ALU);
        keep_min = 0;
    }

    if (ASIC_NUM == 0)
        return;
    if (cbsc_alloc(&w) != 0) {
        jas_log("CBSC: not enough memory for analysis");
        cbsc_free(&w);
        return;
    }

    for (asic = 0; asic < ASIC_NUM; asic++) {
        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;

        memset(w.reaches, 0, N_ALU * sizeof(*w.reaches));
        memset(w.per_1020, 0, N_ALU * sizeof(*w.per_1020));
        memset(w.per_170, 0, N_ALU * sizeof(*w.per_170));
        memset(w.per_34, 0, N_ALU * sizeof(*w.per_34));
        memset(w.bank_alu_count, 0, sizeof(w.bank_alu_count));
        memset(w.alu_bank_count, 0, sizeof(w.alu_bank_count));

        cbsc_tally(thread_id, asic, &w);

        /* carto diag: freeze the ALU coverage per bank (destroyed at free) */
        {
            unsigned bk;
            for (bk = 0; bk < N_BANK; bk++)
                jas_map_bank_alu[thread_id][asic * N_BANK + bk] =
                    w.bank_alu_count[bk];
        }

        cbsc_judge_asic(thread_id, asic, &w);
        cbsc_keep_covered(thread_id, asic, &w,
                          (cov_mask >> asic & 1) ? keep_min : 0);
        /* After the rehabilitation, before the commit: it is the last moment
         * when the bank map can be changed and bank_alu_count still exists (w
         * is freed right after). */
        cbsc_coverage_threshold(thread_id, asic, &w);
        cbsc_commit(thread_id, asic);
        cbsc_count(thread_id, asic);
    }

    cbsc_free(&w);
}
