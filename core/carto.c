/*
 * Memory map per bank / mesh: READ-ONLY diagnostic.
 *
 * Dumps, for each active ASIC, the state the scan established for each bank:
 * good / partial / bad, the hardware masks mc_en / mc_refresh_en, and the
 * difference between REALLY DEAD banks (DFT verdict) and banks ONLY CONDEMNED
 * BY THE CBSC (DFT-good but judged bad for poor ALU coverage, hence possibly
 * recoverable).
 *
 * jas_map_cbsc_killed is filled by cbsc_commit (crossbar.c) when it turns
 * off a DFT-good bank. NO hardware write. Gated by JASRE_MAP_DUMP.
 *
 * Output: /tmp/bank_map.txt (summary per mesh) and /tmp/bank_map.csv (one line per
 * bank: board,asic,mesh,bank,good,partial,bad,cbsc_killed,mc_en,mc_refresh).
 */
#include <stdio.h>
#include <stdlib.h>
#include "jas_hal.h"
#include "jas_state.h"

#define MAP_BANKS_PER_ASIC 4080u
#define MAP_BANKS_PER_MESH 1020u
#define MAP_MESHES         4u

/* Filled by cbsc_commit: 1 = DFT-good bank condemned by the CBSC. */
uint16_t jas_map_cbsc_killed[3][12240];

/* Filled by crossbar_check_analysis (after cbsc_tally): number of ALUs
 * reaching each bank. High coverage on a killed bank = the CBSC
 * over-condemned it (group rule), low coverage = real defect. */
uint16_t jas_map_bank_alu[3][12240];

void jas_dump_map(void)
{
    FILE *f, *csv;
    unsigned board, asic, mesh, b;

    if (!getenv("JASRE_MAP_DUMP"))
        return;

    f   = fopen("/tmp/bank_map.txt", "w");
    csv = fopen("/tmp/bank_map.csv", "w");
    if (!f || !csv) {
        jas_log("bank map: cannot open /tmp/bank_map.{txt,csv}");
        if (f)   fclose(f);
        if (csv) fclose(csv);
        return;
    }

    fprintf(csv, "board,asic,mesh,bank,good,partial,bad,cbsc_killed,bank_alu,mc_en,mc_refresh\n");

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(BOARD_EN >> board & 1))
            continue;

        for (asic = 0; asic < ASIC_NUM; asic++) {
            unsigned rank = board * 3u + asic;
            unsigned base = asic * MAP_BANKS_PER_ASIC;

            if (!(ASIC_EN(board) >> asic & 1))
                continue;

            fprintf(f, "=== board %u asic %u: cbsc_total=%u  mesh(%u/%u/%u/%u) ===\n",
                    board, asic, jas_good_bank_num_cbsc[rank],
                    jas_good_bank_num_cbsc_mesh[rank * 4 + 0],
                    jas_good_bank_num_cbsc_mesh[rank * 4 + 1],
                    jas_good_bank_num_cbsc_mesh[rank * 4 + 2],
                    jas_good_bank_num_cbsc_mesh[rank * 4 + 3]);

            for (mesh = 0; mesh < MAP_MESHES; mesh++) {
                unsigned g = 0, p = 0, bad = 0, en = 0, ref = 0;
                unsigned killed = 0, dead = 0;   /* v2: recoverable vs really dead */
                /* v3: ALU coverage (bank_alu_count), killed by CBSC vs surviving */
                unsigned kcov_min = 65535, kcov_max = 0; unsigned long kcov_sum = 0;
                unsigned long gcov_sum = 0;
                char list[6144];
                unsigned li = 0;

                list[0] = 0;

                for (b = 0; b < MAP_BANKS_PER_MESH; b++) {
                    unsigned gi   = base + mesh * MAP_BANKS_PER_MESH + b;
                    unsigned word = b >> 4, bit = b & 15u;
                    int good = jas_bank_good[board][gi]    != 0;
                    int part = jas_bank_partial[board][gi] != 0;
                    int bd   = jas_bank_bad[board][gi]     != 0;
                    int kill = jas_map_cbsc_killed[board][gi] != 0;
                    unsigned cov = jas_map_bank_alu[board][gi];
                    int mce  = (jas_mc_en[board][asic][mesh][word]         >> bit) & 1;
                    int mcr  = (jas_mc_refresh_en[board][asic][mesh][word] >> bit) & 1;

                    if (good) { g++; gcov_sum += cov; }
                    if (part) p++;
                    if (bd)   bad++;
                    if (mce)  en++;
                    if (mcr)  ref++;
                    if (!good) {
                        if (kill) {
                            killed++;   /* CBSC killed a DFT-good bank */
                            kcov_sum += cov;
                            if (cov < kcov_min) kcov_min = cov;
                            if (cov > kcov_max) kcov_max = cov;
                        } else {
                            dead++;     /* never good: the DFT condemned it */
                        }
                    }

                    fprintf(csv, "%u,%u,%u,%u,%d,%d,%d,%d,%u,%d,%d\n",
                            board, asic, mesh, b, good, part, bd, kill, cov, mce, mcr);

                    if (!good && li < sizeof(list) - 20) {
                        /* 'k'=recoverable by CBSC ; (cov)=nb of ALUs reaching the bank */
                        int n = snprintf(list + li, sizeof(list) - li,
                                         "%u%s(%u) ", b, kill ? "k" : "", cov);
                        if (n > 0)
                            li += (unsigned)n;
                    }
                }

                fprintf(f, "  mesh %u: good=%u bad=%u (cbsc_killed=%u dft_dead=%u) | "
                        "partial=%u mc_en=%u mc_refresh=%u\n",
                        mesh, g, bad, killed, dead, p, en, ref);
                fprintf(f, "    ALU coverage: good_avg=%lu | killed cov min/avg/max=%u/%lu/%u\n",
                        g ? gcov_sum / g : 0,
                        killed ? kcov_min : 0,
                        killed ? kcov_sum / killed : 0,
                        kcov_max);
                if (li)
                    fprintf(f, "    non-good NN[k](cov): %s\n", list);
            }

            fprintf(f, "\n");
        }
    }

    fclose(f);
    fclose(csv);
    jas_log("bank map: written to /tmp/bank_map.txt and /tmp/bank_map.csv");
}
