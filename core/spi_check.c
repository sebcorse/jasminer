/*
 * Calibration of the SPI link to the ASICs.
 *
 * The serial link has a sampling phase setting — the moment the FPGA reads
 * the data sent back by the chip. If it is badly placed, reads are wrong
 * intermittently, which would give random mining results rather than a clear
 * failure. This function determines it experimentally, mesh by mesh.
 *
 * For each of a chip's four meshes, seventeen phases are tried. A phase is
 * kept if sixteen consecutive round trips of two complementary patterns —
 * 0xa5a5 then 0x5a5a, sixty words each time — come back intact. The phase
 * finally programmed is the average of the indices kept, which puts the
 * sampling point at the centre of the valid window rather than at its edge.
 *
 * A chip of which a single mesh finds no valid phase is declared failed and
 * removed from the enable mask; a board all of whose chips fail is removed in
 * turn.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "sl_transfer.h"
#include <string.h>

#define SPI_PHASES      17      /* 0, then 1<<0 .. 1<<15 */
#define SPI_REPEATS     16      /* round trips per phase */
#define SPI_WORDS       0x3c    /* 60 words per pattern */
#define SPI_PATTERN_A   0xa5a5
#define SPI_PATTERN_B   0x5a5a
#define REG_PATTERN     0x580   /* test window */
#define REG_PHASE       0x746   /* phase setting */
#define REG_FPGA_PHASE  0x6002  /* FPGA-side phase */
#define SPI_TIMEOUT     4000000

/* cmd encodes the target mesh: 0x10<<mesh, then 8 to write and 0xc to read. */
#define CMD_WRITE(mesh) ((uint8_t)((0x10u << (mesh)) | 0x8u))
#define CMD_READ(mesh)  ((uint8_t)((0x10u << (mesh)) | 0xcu))

/* One round trip: write the pattern, read it back, compare. */
static int spi_roundtrip(uint8_t asic_mask, uint8_t cmd_w, uint8_t cmd_r,
                         uint16_t pattern, int thread_id)
{
    uint16_t w[128], r[128];
    unsigned n;

    memset(w, 0, sizeof(w));
    memset(r, 0, sizeof(r));
    for (n = 0; n < SPI_WORDS; n++)
        w[n] = pattern;

    sl_multi_write(asic_mask, cmd_w, REG_PATTERN, w, SPI_WORDS, thread_id);
    sl_multi_read(asic_mask, cmd_r, REG_PATTERN, r, SPI_WORDS, SPI_TIMEOUT, thread_id);

    return memcmp(w, r, SPI_WORDS * sizeof(uint16_t)) == 0;
}

/* A phase holds if both patterns come back intact, sixteen times in a row. */
static int spi_phase_holds(uint8_t asic_mask, uint8_t cmd_w, uint8_t cmd_r,
                           int thread_id)
{
    for (int rep = 0; rep < SPI_REPEATS; rep++) {
        if (!spi_roundtrip(asic_mask, cmd_w, cmd_r, SPI_PATTERN_A, thread_id))
            return 0;
        if (!spi_roundtrip(asic_mask, cmd_w, cmd_r, SPI_PATTERN_B, thread_id))
            return 0;
    }
    return 1;
}

void spi_check(int thread_id)
{
    uint16_t fpga_phase = 0;
    unsigned asic, mesh;

    if (ASIC_NUM == 0)
        return;

    for (asic = 0; asic < ASIC_NUM; asic++)
        jas_spi_status[thread_id][asic] = 0;

    for (asic = 0; asic < ASIC_NUM; asic++) {
        uint8_t mesh_ok[4] = { 0, 0, 0, 0 };
        uint8_t asic_mask = (uint8_t)(1u << asic);

        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;

        for (mesh = 0; mesh < 4; mesh++) {
            uint8_t cmd_w = CMD_WRITE(mesh), cmd_r = CMD_READ(mesh);
            unsigned sum = 0, kept = 0, k;

            /* Reset of the FPGA-side phase before each scan. */
            sl_single_write(0xff, 0xf9, REG_FPGA_PHASE, 0, thread_id);
            sl_single_read(0xff, 0xfd, REG_FPGA_PHASE, &fpga_phase,
                           SPI_TIMEOUT, thread_id);

            for (k = 0; k < SPI_PHASES; k++) {
                /* k = 0 means "no shift"; beyond that, a single bit. */
                uint16_t phase = (k == 0) ? 0 : (uint16_t)(1u << (k - 1));

                jas_spi_phase[thread_id][asic][mesh] = phase;
                sl_single_write(ASIC_EN(thread_id), cmd_w, REG_PHASE,
                                phase, thread_id);

                if (spi_phase_holds(asic_mask, cmd_w, cmd_r, thread_id)) {
                    sum += k;
                    kept++;
                } else {
                    jas_trace("board %d asic %u mesh %u: phase %u rejected",
                            thread_id, asic, mesh, k);
                }
            }

            /*
             * The centre of the valid window, expressed again as a single bit.
             * With no phase kept, we program zero and the mesh is marked as
             * failed.
             */
            uint16_t chosen = 0;
            if (kept) {
                unsigned mid = sum / kept;
                chosen = mid ? (uint16_t)(1u << (mid - 1)) : 0;
                mesh_ok[mesh] = 1;
            }
            jas_spi_phase[thread_id][asic][mesh] = chosen;
            sl_single_write(ASIC_EN(thread_id), cmd_w, REG_PHASE,
                            chosen, thread_id);
            jas_trace("board %d asic %u mesh %u: %u valid phases, kept 0x%04x",
                    thread_id, asic, mesh, kept, chosen);
        }

        /* All four meshes must hold: a single failure condemns the chip. */
        if (mesh_ok[0] && mesh_ok[1] && mesh_ok[2] && mesh_ok[3]) {
            jas_spi_status[thread_id][asic] = 1;
        } else {
            jas_spi_status[thread_id][asic] = 0;
            jas_log("board %d asic %u: spi failed", thread_id, asic);
        }
    }

    /* A chip that failed the test leaves the enable mask. */
    for (asic = 0; asic < ASIC_NUM; asic++) {
        if (!jas_spi_status[thread_id][asic] ||
            !(ASIC_EN(thread_id) >> asic & 1))
            ASIC_EN(thread_id) &= (uint8_t)~(1u << asic);
    }

    /* And a board on which no chip answers any more leaves in turn. */
    if (ASIC_EN(thread_id) == 0)
        *jas_board_en &= (uint16_t)~(1u << thread_id);

    for (asic = 0; asic < ASIC_NUM; asic++)
        if (ASIC_EN(thread_id) >> asic & 1)
            jas_log("board %d asic %u: SPI %s", thread_id, asic,
                    jas_spi_status[thread_id][asic] ? "ok" : "failed");
}
