/*
 * Direct memory accesses, outside the mining path.
 *
 * Writes a block of thirty-two words at a given address, without going
 * through the memory controllers. This is how the crossbar scan fills the
 * memory with its reference pattern: the normal path is precisely what it is
 * trying to test, it cannot use it to prepare the ground.
 *
 * The write is asynchronous: the chip raises a flag when it is done, and the
 * original waits two hundred milliseconds before giving up.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define REG_ADDR        0x800
#define REG_MODE        0x822
#define REG_DATA        0x802
#define REG_COMMIT      0x823
#define REG_READY       0x824
#define CMD_MESH0       0x18
#define CMD_MESH0_READ  0x1c
#define WRITE_WORDS     0x20
#define TIMEOUT_US      2000000    /* 2 s (was 200000 = 200 ms: memory wait cut
                                    * short; the original waits ~2 s). */

/* How many manual writes/reads timed out — reset to zero by the
 * caller before a measurement. */
unsigned jas_manual_timeouts;

/*
 * MEASURING THE WAIT: per call, how many polls of 0x824, and how many
 * times the very first one already returns "ready". A flag still armed
 * from the previous write would make the wait never kick in, and the fill
 * go faster than the memory absorbs it.
 */
unsigned long jas_mw_calls;
unsigned long jas_mw_polls;
unsigned long jas_mw_ready_immediate;
unsigned long jas_mw_us_wait;
unsigned long jas_mw_mismatches;   /* 1st and 2nd poll disagree */

/* Microseconds elapsed since a given moment. */
static long elapsed_us(const struct timeval *since)
{
    struct timeval now;

    gettimeofday(&now, NULL);
    return (now.tv_sec - since->tv_sec) * 1000000L
         + (now.tv_usec - since->tv_usec);
}

void single_manual_write(uint8_t asic_en, uint32_t w_addr, uint16_t *w_data,
                         int thread_id)
{
    single_manual_write_param(asic_en, w_addr, w_data, thread_id, 1000u, 0);
}

/*
 * PARAMETERIZED VARIANT, for the reclaim: same sequence, two settings:
 *   delay_us      : the pause after validation (1000 = the original; it is
 *                   what costs two thirds of a node: 1.4 ms measured for
 *                   ~0.4 ms of serial transactions);
 *   probe_before  : 1 = before loading the address, wait for each target
 *                   chip to announce "ready" (0x824): the real time of the
 *                   previous block, not a flat rate.
 * single_manual_write (CBSC fill, write modes of main.c) keeps (1000, 0):
 * stock behaviour, validated by the bank counts.
 */
void single_manual_write_param(uint8_t asic_en, uint32_t w_addr, uint16_t *w_data,
                               int thread_id, unsigned delay_us, int probe_before)
{
    uint16_t sram[WRITE_WORDS];
    unsigned asic;

    if (probe_before) {
        for (asic = 0; asic < ASIC_NUM; asic++) {
            struct timeval start;
            uint16_t ready = 0;

            if (!(asic_en >> asic & 1))
                continue;
            gettimeofday(&start, NULL);
            do {
                sl_single_read((uint8_t)(1u << asic), CMD_MESH0_READ, REG_READY,
                               &ready, 2000000, thread_id);
                if (ready)
                    break;
            } while (elapsed_us(&start) <= TIMEOUT_US);
            if (!ready) {
                jas_manual_timeouts++;
                if (jas_manual_timeouts <= 3)
                    jas_log("board %d asic %u: memory write engine still busy"
                            " before address %u", thread_id, asic, w_addr);
            }
        }
    }

    sl_multi_write(asic_en, CMD_MESH0, REG_ADDR, (uint16_t *)&w_addr, 2, thread_id);
    sl_single_write(asic_en, CMD_MESH0, REG_MODE, 1, thread_id);

    memcpy(sram, w_data, sizeof(sram));
    sl_multi_write(asic_en, CMD_MESH0, REG_DATA, sram, WRITE_WORDS, thread_id);

    /* Each target chip must confirm. */
    for (asic = 0; asic < ASIC_NUM; asic++) {
        struct timeval start;
        uint16_t ready = 0;

        if (!(asic_en >> asic & 1))
            continue;

        gettimeofday(&start, NULL);
        {
            unsigned rounds = 0;

            /*
             * A SINGLE POLL: the first and second polls almost never differ
             * (0.04 % of calls), so the flag is not stale, and a second read
             * even lowers the coverage slightly.
             */
            do {
                /*
                 * 2000000 and not 1000000: the loader doubles this constant
                 * in ten functions (see JasMiner-X16-Q-runtime/PATCHS.txt).
                 * The effect is NIL — sl_single_read ignores its timeout
                 * parameter and loops on the status bits — but the reference is
                 * the patched binary, so the figure might as well match.
                 */
                sl_single_read((uint8_t)(1u << asic), CMD_MESH0_READ, REG_READY,
                               &ready, 2000000, thread_id);
                rounds++;
                if (ready)
                    break;
            } while (elapsed_us(&start) <= TIMEOUT_US);

            jas_mw_calls++;
            jas_mw_polls += rounds;
            if (rounds == 1 && ready)
                jas_mw_ready_immediate++;
            jas_mw_us_wait += (unsigned long)elapsed_us(&start);
        }


        if (!ready) {
            jas_manual_timeouts++;
            if (jas_manual_timeouts <= 3)
                jas_log("board %d asic %u: memory write at address %u not answering",
                        thread_id, asic, w_addr);
        }
    }

    /*
     * VALIDATING THE BLOCK, AND THE DELAY THAT FOLLOWS. Without writing
     * 0x823, the address, the mode and the data are loaded but nothing is
     * committed to memory: the fill writes nothing, and the scan compares
     * the expected pattern with leftover memory content. The millisecond
     * that follows is most of the original's fill time (65280 blocks x
     * 1 ms = 65 s). These two calls are the very last lines of the
     * decompiled function, after a dense timeout-handling block: easy to
     * miss.
     */
    sl_single_write(asic_en, CMD_MESH0, REG_COMMIT, 1, thread_id);
    if (delay_us)
        usleep(delay_us);
}

/*
 * Direct read-back of a memory block, by a designated ALU.
 *
 * manual_sel designates the reading ALU in the space of 120: the mesh is
 * manual_sel / 30, the position in the mesh manual_sel % 30. That position
 * becomes a single-bit selector, spread over two sixteen-bit words.
 *
 * Sixty-four words are returned, against thirty-two on writing: the read
 * brings back the whole block.
 */

#define REG_R_SEL     0x700
#define REG_R_ADDR    0x702
#define REG_R_START   0x704
#define REG_R_VALID   0x705
#define REG_R_DATA    0x706
#define REG_R_DATA2   0x726   /* second half of the block, see dram_dump */
#define READ_WORDS    0x40
#define ALUS_PER_MESH 30u

int single_manual_read(uint8_t asic_en, uint16_t manual_sel, uint32_t manual_addr,
                       uint16_t *manual_dat, int thread_id)
{
    unsigned mesh = manual_sel / ALUS_PER_MESH;
    unsigned alu  = manual_sel % ALUS_PER_MESH;
    uint8_t cmd_w = (uint8_t)(((1u << mesh) << 4) | 0x8u);
    uint8_t cmd_r = (uint8_t)(((1u << mesh) << 4) | 0xcu);
    uint16_t sel[2] = { 0, 0 };
    uint32_t addr = manual_addr;
    struct timeval start;

    sel[alu >> 4] = (uint16_t)(1u << (alu & 0xf));

    sl_multi_write(asic_en, cmd_w, REG_R_SEL,  sel, 2, thread_id);
    sl_multi_write(asic_en, cmd_w, REG_R_ADDR, (uint16_t *)&addr, 2, thread_id);
    sl_single_write(asic_en, cmd_w, REG_R_START, 1, thread_id);

    gettimeofday(&start, NULL);
    do {
        uint16_t valid = 0;

        sl_single_read(asic_en, cmd_r, REG_R_VALID, &valid, 1000000, thread_id);
        if (valid) {
            sl_multi_read(asic_en, cmd_r, REG_R_DATA, manual_dat, READ_WORDS,
                          1000000, thread_id);
            sl_single_write(asic_en, cmd_w, REG_R_VALID, 1, thread_id);
            return 0;
        }
    } while (elapsed_us(&start) <= TIMEOUT_US);

    jas_log("board %d: memory read at address %u not answering",
            thread_id, manual_addr);
    return -1;
}

/*
 * OFF THE EXECUTION PATH (tools/atteignable.py): no live function calls
 * this one in the running firmware. Kept as a tool.
 */
/*
 * dram_dump @ 0x21f2c — dump an ALU's memory into a text file.
 *
 * No caller in the running firmware: it is a debugging tool the vendor left
 * in the binary. It is reimplemented because it serves us — it is the most
 * direct way to look at what an ALU really sees at an address, without going
 * through the DAG or the crossbar.
 *
 * Two differences with single_manual_read, and they make sense:
 *
 *   - the read is done in TWO halves, 0x706 then 0x726, thirty-two words
 *     each. A memory block is 128 bytes, i.e. two 64-byte DAG nodes; each of
 *     the two registers exposes its own. single_manual_read takes the
 *     sixty-four words at once from 0x706.
 *
 *   - the words are written from last to first. It is not an assignment:
 *     the memory returns the words little-endian, and the reversal gives the
 *     usual hexadecimal writing of the node, most significant on the left. It
 *     is in that order that it can be compared with ethash's output.
 *
 * The file gets one line per half, i.e. two lines per address.
 */
void dram_dump(uint8_t asic_en, uint16_t manual_sel, uint32_t base_addr,
               uint32_t jump_size, const char *file_name, int thread_id)
{
    unsigned mesh = manual_sel / ALUS_PER_MESH;
    unsigned alu  = manual_sel % ALUS_PER_MESH;
    uint8_t cmd_w = (uint8_t)(((1u << mesh) << 4) | 0x8u);
    uint8_t cmd_r = (uint8_t)(((1u << mesh) << 4) | 0xcu);
    uint16_t sel[2] = { 0, 0 };
    uint16_t block[32];
    uint32_t addr;
    FILE *f;
    int k;

    sel[alu >> 4] = (uint16_t)(1u << (alu & 0xf));
    sl_multi_write(asic_en, cmd_w, REG_R_SEL, sel, 2, thread_id);

    f = fopen(file_name, "w");
    if (!f) {
        jas_log("dram_dump: cannot open %s", file_name);
        sl_single_write(asic_en, 0xf8, REG_R_SEL, 0, thread_id);
        return;
    }

    for (addr = base_addr; addr != base_addr + jump_size; addr++) {
        uint32_t a = addr;
        uint16_t valid;

        sl_multi_write(asic_en, cmd_w, REG_R_ADDR, (uint16_t *)&a, 2, thread_id);
        sl_single_write(asic_en, cmd_w, REG_R_START, 1, thread_id);
        usleep(1000);

        /* The original loops here WITHOUT a time bound — a silent ALU blocks
         * it for good. We keep the loop but with the same limit as
         * single_manual_read, and we say so in the file. */
        {
            struct timeval start;

            gettimeofday(&start, NULL);
            do {
                valid = 0;
                sl_single_read(asic_en, cmd_r, REG_R_VALID, &valid, 1000000, thread_id);
            } while (!valid && elapsed_us(&start) <= TIMEOUT_US);

            if (!valid) {
                fprintf(f, "* address %u: no reply\n", addr);
                continue;
            }
        }
        sl_single_write(asic_en, cmd_w, REG_R_VALID, 1, thread_id);

        memset(block, 0, sizeof(block));
        sl_multi_read(asic_en, cmd_r, REG_R_DATA, block, 32, 1000000, thread_id);
        for (k = 31; k >= 0; k--)
            fprintf(f, "%04x", block[k]);
        fputc('\n', f);

        memset(block, 0, sizeof(block));
        sl_multi_read(asic_en, cmd_r, REG_R_DATA2, block, 32, 1000000, thread_id);
        for (k = 31; k >= 0; k--)
            fprintf(f, "%04x", block[k]);
        fputc('\n', f);
    }

    fflush(f);
    fclose(f);
    sl_single_write(asic_en, 0xf8, REG_R_SEL, 0, thread_id);
}
