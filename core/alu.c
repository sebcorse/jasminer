/*
 * Initialization and self-test of the compute units.
 *
 * Before putting the ALUs to work on real mining, the firmware gives them a
 * problem whose answer it knows and checks that they find it. The problem is
 * built on **epoch zero**: cache, DAG, header and seed are those of the very
 * first block, which makes the test vector independent of the mined coin and
 * reproducible.
 *
 * This is what makes these registers treacherous: 0x04, 0x08, 0x10, 0x20 and
 * 0x30 carry a test vector here, and the real values during mining.
 * Overwriting them without telling the two uses apart makes the self-test fail
 * and condemns the chip — "good alu number < 10" — while its memory is intact.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_types.h"
#include "sl_transfer.h"
#include "jas_ethash.h"
#include <string.h>
#include <unistd.h>

/* Provided by the firmware until the Ethash library is reimplemented. */





#define REG_BLOCK      0x00
#define REG_CACHE_SIZE 0x04
#define REG_DAG_SIZE   0x08
#define REG_NONCE      0x0c
#define REG_TARGET     0x10
#define REG_HEADER     0x20
#define REG_SEED       0x30
#define REG_ALU_SCAN   0x400
#define REG_ALU_RESULT 0x402
#define REG_TRIGGER    0x830
#define CMD_BROADCAST  0xf8

/* ALU test vector, built on epoch zero. */
void init_alu_reg(int thread_id)
{
    uint8_t asic_en = ASIC_EN(thread_id);
    uint64_t block_number = 0;
    uint64_t nonce = 0, cache_size, dag_size;
    ethash_h256_t header, seed;
    uint8_t target[32];

    sl_multi_write(asic_en, CMD_BROADCAST, REG_BLOCK,
                   (uint16_t *)&block_number, 4, thread_id);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_NONCE,
                   (uint16_t *)&nonce, 4, thread_id);

    cache_size = ethash_get_cachesize(block_number);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_CACHE_SIZE,
                   (uint16_t *)&cache_size, 4, thread_id);

    dag_size = ethash_get_datasize(block_number);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_DAG_SIZE,
                   (uint16_t *)&dag_size, 4, thread_id);

    ethash_get_headerhash(&header, block_number);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_HEADER,
                   (uint16_t *)&header, 16, thread_id);

    ethash_get_seedhash(&seed, block_number);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_SEED,
                   (uint16_t *)&seed, 16, thread_id);

    /*
     * Deliberately wide target: 0x00000000ffff…ff. The first three zero bytes
     * are enough to filter out noise, everything else passes. During mining
     * the firmware programs an equally open target anyway and filters on the
     * processor afterwards.
     */
    memset(target, 0, sizeof(target));
    memset(target + 3, 0xff, sizeof(target) - 3);
    sl_multi_write(asic_en, CMD_BROADCAST, REG_TARGET,
                   (uint16_t *)target, 16, thread_id);

    jas_log("board %d: alu vector set, cache %llu B, dag %llu B",
            thread_id, (unsigned long long)cache_size,
            (unsigned long long)dag_size);
}

/*
 * Starts the self-test and reads the results.
 *
 * The scan is triggered by a pulse on register 0x400: all enabled, all off,
 * all enabled again. The results are then read mesh by mesh, six words each,
 * through commands 0x1c, 0x2c, 0x4c and 0x8c — the same convention as the SPI
 * calibration, 0x10<<mesh combined with 0xc.
 */
void alu_check(int thread_id)
{
    uint16_t scan[2];
    unsigned asic, mesh;

    /* Start from a blank readout for each chip. */
    for (asic = 0; asic < ASIC_NUM; asic++)
        memset(jas_alu_result[thread_id][asic], 0,
               sizeof(jas_alu_result[thread_id][asic]));

    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_TRIGGER, 0, thread_id);
    sl_delay(0, 2000);
    sl_single_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_TRIGGER, 0xffff, thread_id);

    init_alu_reg(thread_id);

    /* Enable pulse: the falling then rising edge starts each ALU's internal
     * scan. */
    scan[0] = scan[1] = 0xffff;
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_ALU_SCAN, scan, 2, thread_id);
    usleep(5000);
    scan[0] = scan[1] = 0;
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_ALU_SCAN, scan, 2, thread_id);
    usleep(5000);
    scan[0] = scan[1] = 0xffff;
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_ALU_SCAN, scan, 2, thread_id);
    usleep(5000);

    for (asic = 0; asic < ASIC_NUM; asic++) {
        uint8_t asic_mask = (uint8_t)(1u << asic);

        if (!(ASIC_EN(thread_id) >> asic & 1))
            continue;

        for (mesh = 0; mesh < 4; mesh++)
            sl_multi_read(asic_mask, (uint8_t)((0x10u << mesh) | 0xcu),
                          REG_ALU_RESULT,
                          jas_alu_result[thread_id][asic] + mesh * 6, 6,
                          1000000, thread_id);
    }

    scan[0] = scan[1] = 0;
    sl_multi_write(ASIC_EN(thread_id), CMD_BROADCAST, REG_ALU_SCAN, scan, 2, thread_id);
}
