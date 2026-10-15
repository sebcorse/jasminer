/*
 * Sensor initialization: sensor_init @ 0x1bad8.
 *
 * Step 3 of the startup, between board_check and power-on. It opens the IIC
 * bus of each populated connector and maps the fans. The IIC bus carries the
 * temperature sensors BUT ALSO the PMBus, hence the supply voltages
 * power_on_operation programs right after: without it, nothing can be set.
 *
 * REFERENCE, the firmware's log:
 *
 *     board 0 open /dev/mem ok.
 *     board 0 mmap IIC base address ok.
 *     open /dev/mem ok.
 *     mmap fan base address ok.
 *     sensor init finish
 */

#include "jas_hal.h"
#include "jas_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/*
 * Temperatures per board and per chip, initialized to 32.
 *
 * The firmware keeps this array in a global whose address is not resolved yet
 * (referenced PIC-style through the GOT). Here it lives with us: the initial
 * value of 32 is what matters, and it shows in the fan guard's log —
 * "curve temp=32" before the first real reading.
 */
uint32_t jas_asic_temp[3][3];

void sensor_init(void)
{
    unsigned board, chip;

    /*
     * IIC WRITES ARE ON: /proc/<pid>/maps of the stock firmware confirms
     * 0x41600000 as the IIC window it maps (an AXI write to a slave that does
     * not answer would block the ARM core for good, hence the check). They
     * are needed to set and read the voltages.
     */
    int write_iic = 1;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(jas_sensor_board_en >> board & 1))
            continue;
        jas_iic_map_init((int)board);
        if (write_iic)
            iic_initialization_op((int)board);
        else
            jas_log("board %u: iic init inhibited (addresses not confirmed by"
                    " /proc/<pid>/maps)", board);
    }

    jas_fan_map_init();

    /* The firmware initializes a sensor mutex here; the global bus lock,
     * taken by the IIC wrappers of iic.c, plays that role. */

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(jas_sensor_board_en >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++)
            jas_asic_temp[board][chip] = 32;
    }
}

/* ======================================================================
 * TEMPERATURES AND FANS
 *
 * sl_wrapper_work cuts the ASICs' power on four alarms (missing
 * temperature, excessive temperature, stopped fan, Ctrl-C), which
 * core/surveillance.c raises from these readings.
 *
 * THE SENSOR. One TMP401 per chip, at address 0x4c, behind a bus
 * multiplexer at 0x70. The three sensors share a pair of wires, hence a
 * switch before each read.
 *
 * THE CONVERSION, and it is surprising: temperature = raw byte − 0x40. The
 * TMP401 is in "extended range" mode, offset by sixty-four degrees. A
 * failed read leaves cov_temp at zero, hence −64, which is also the value
 * get_temp_sensor_value forces on any aberrant measurement: −64 means "no
 * measurement", never "very cold".
 * ====================================================================== */

#include <pthread.h>
#include <unistd.h>

#define TMP401_ADR      0x4c
#define TMP401_TEMP     0x00    /* temperature, whole byte */
#define TMP401_STATE     0x02    /* bit 7: conversion in progress */
#define TMP401_CONFIG   0x09
#define TMP401_CADENCE  0x0a
#define MUX_ADR         0x70
#define OFFSET        0x40    /* extended range: raw - 64 */
#define TEMP_MISSING    (-64)
#define TEMP_MAX_PLAUSIBLE  90  /* beyond that, the measurement is rejected */

/*
 * The multiplexer channel of each sensor. Non-trivial order, read in the
 * binary: sensor 0 -> channel 2, sensor 1 -> channel 0, sensor 2 -> channel
 * 1. It is the board's wiring, not a convention.
 */
static const uint8_t channel_mux[3] = { 0x02, 0x00, 0x01 };

static pthread_mutex_t temp_lock = PTHREAD_MUTEX_INITIALIZER;
uint16_t jas_tmp_sensor_en = 0x7;      /* tmp_sensor_en_g */

/* Three attempts, like the original — the IIC bus sometimes misses once. */
#define THREE_TIMES(call, what)                                            \
    do {                                                                   \
        int _n = 3, _r = -1;                                               \
        while (_n-- > 0 && (_r = (call)) != 0)                            \
            ;                                                              \
        if (_r != 0)                                                       \
            jas_log("board %u sensor %u: %s", board, j, what);           \
    } while (0)

void fan_speed_set(int rate)
{
    if (jas_fan_p[0])
        *(volatile uint32_t *)jas_fan_p[0] = (uint32_t)rate;
    if (jas_fan_p[1])
        *(volatile uint32_t *)jas_fan_p[1] = (uint32_t)rate;
}

/*
 * Four fans, two per window. The hardware counter is multiplied by thirty:
 * it counts turns over two seconds, and the original turns that into
 * revolutions per minute.
 */
void fan_speed_read(int fan_speed[4])
{
    volatile uint32_t *f0 = jas_fan_p[0], *f1 = jas_fan_p[1];

    fan_speed[0] = f0 ? (int)(f0[2] * 30u) : 0;
    fan_speed[1] = f0 ? (int)(f0[3] * 30u) : 0;
    fan_speed[2] = f1 ? (int)(f1[2] * 30u) : 0;
    fan_speed[3] = f1 ? (int)(f1[3] * 30u) : 0;
}

/*
 * Copy the temperatures out for the caller, rejecting the implausible.
 * Above 90 degrees the original does not believe the measurement and
 * replaces it with -64 — the same value as "no sensor". Cautious, but it
 * means a chip really at 95 degrees is reported as silent, not as burning.
 * Not to be forgotten when writing the thermal alarm.
 */
void get_temp_sensor_value(int *temp_value)
{
    unsigned board, chip;

    for (board = 0; board < BOARD_NUM; board++) {
        if (!(jas_sensor_board_en >> board & 1))
            continue;
        for (chip = 0; chip < ASIC_NUM; chip++) {
            unsigned k = board * ASIC_NUM + chip;

            if (!(jas_tmp_sensor_en >> chip & 1))
                continue;
            if (jas_asic_temp[board][chip] > TEMP_MAX_PLAUSIBLE)
                jas_asic_temp[board][chip] = (uint32_t)TEMP_MISSING;
            temp_value[k] = (int)jas_asic_temp[board][chip];
        }
    }
}

/*
 * A board's reading thread. It configures its sensors, then loops forever:
 * switch the multiplexer, wait for the end of conversion, read.
 *
 * The conversion wait is bounded to 102 attempts of 30 ms — three seconds.
 * Beyond that, the original logs "TMP401_R_SR reg error" and keeps the last
 * value.
 */
/*
 * One reading round, taken out of get_temp_func to be callable on its own.
 *
 * The original only loops over these two steps; separating them changes
 * nothing in the reimplementation and lets us test them without starting a
 * thread that never returns.
 */
void jas_sensors_configure(unsigned board)
{
    unsigned j;

    for (j = 0; j < ASIC_NUM; j++) {
        if (!(jas_tmp_sensor_en >> j & 1))
            continue;
        THREE_TIMES(iic_mux_select(MUX_ADR, &channel_mux[j], (int)board),
                   "multiplexer unreachable");
        THREE_TIMES(iic_single_byte_write(TMP401_ADR, TMP401_CONFIG, 0x84, (int)board),
                   "configuration refused");
        THREE_TIMES(iic_single_byte_write(TMP401_ADR, TMP401_CADENCE, 0x05, (int)board),
                   "set conv rate fail");
        jas_log("temp senor board %u, tmp %u config done", board, j);
    }
}

void jas_sensors_read(unsigned board)
{
    unsigned j;

    {
        for (j = 0; j < ASIC_NUM; j++) {
            uint8_t state = 0, raw = 0;
            unsigned rounds;

            if (!(jas_tmp_sensor_en >> j & 1))
                continue;

            if (iic_mux_select(MUX_ADR, &channel_mux[j], (int)board) != 0 &&
                iic_mux_select(MUX_ADR, &channel_mux[j], (int)board) != 0 &&
                iic_mux_select(MUX_ADR, &channel_mux[j], (int)board) != 0) {
                pthread_mutex_lock(&temp_lock);
                jas_asic_temp[board][j] = (uint32_t)TEMP_MISSING;
                pthread_mutex_unlock(&temp_lock);
                continue;
            }

            for (rounds = 0; rounds < 0x66; rounds++) {
                state = 0;
                iic_single_byte_read(TMP401_ADR, TMP401_STATE, &state, (int)board);
                if (!(state & 0x80))
                    break;
                usleep(10000);
                usleep(20000);
            }
            if (rounds >= 0x65) {
                jas_log("get_temp_sensor_value SR = 0x%x, TMP401_R_SR reg error,"
                        " iic init", state);
                continue;
            }

            raw = 0;
            if (iic_single_byte_read(TMP401_ADR, TMP401_TEMP, &raw, (int)board) != 0 &&
                iic_single_byte_read(TMP401_ADR, TMP401_TEMP, &raw, (int)board) != 0 &&
                iic_single_byte_read(TMP401_ADR, TMP401_TEMP, &raw, (int)board) != 0)
                raw = 0;

            pthread_mutex_lock(&temp_lock);
            jas_asic_temp[board][j] = (uint32_t)((int)raw - OFFSET);
            pthread_mutex_unlock(&temp_lock);
        }
    }
}

/* The original function: configure once, then read forever. */
void get_temp_func(uint16_t thread_id)
{
    jas_sensors_configure(thread_id);
    sleep(1);
    for (;;) {
        jas_sensors_read(thread_id);
        sleep(1);
    }
}

static pthread_t temp_thread[3];
static void *get_temp_thread_0(void *r) { (void)r; jas_log("get_temp_thread %d", 0); get_temp_func(0); return NULL; }
static void *get_temp_thread_1(void *r) { (void)r; jas_log("get_temp_thread %d", 1); get_temp_func(1); return NULL; }
static void *get_temp_thread_2(void *r) { (void)r; jas_log("get_temp_thread %d", 2); get_temp_func(2); return NULL; }
static void *(*const thread_fn[3])(void *) = {
    get_temp_thread_0, get_temp_thread_1, get_temp_thread_2,
};

/* One thread per populated board, decided by sensor_board_en_g. */
void sensor_thread_create(void)
{
    unsigned board;

    for (board = 0; board < 3; board++) {
        if (!(jas_sensor_board_en >> board & 1))
            continue;
        if (pthread_create(&temp_thread[board], NULL, thread_fn[board], NULL) != 0)
            jas_log("fail to pthread_create get_temp_thread_%u", board);
    }
}
