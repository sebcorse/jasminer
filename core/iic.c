/*
 * IIC bus: the Xilinx AXI IIC controller, and the PMBus living on it.
 *
 * This bus carries the temperature sensors AND the voltage regulators, which
 * power_on_operation sets right after the ASICs' power is cut and restored
 * (otherwise they stay at the regulator's default, ~901 mV).
 * mcp2973_set_vol writes VOUT_COMMAND (0x21) then reads READ_VOUT (0x8b)
 * back; the two encodings differ:
 *
 *     raw = mV / 5 - 0x31         860 mV -> 123 = 0x7B
 *     mV  = (raw + 0x31) * 5
 */

#include "jas_hal.h"
#include "jas_state.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Registers of the AXI IIC controller, in bytes from the window base. */
#define IIC_SOFTR   0x40
#define IIC_CR      0x100
#define IIC_SR      0x104
#define IIC_TX      0x108
#define IIC_RX      0x10c
#define IIC_RX_PIRQ 0x120

#define SR_RX_EMPTY  0x40
#define SR_READY     0xc0      /* mask 0xc4, expected value 0xc0 */
#define SR_MASK   0xc4

static volatile uint32_t *iic_of(int thread_id)
{
    if (thread_id < 0 || thread_id > 2 || !jas_iic_p[thread_id])
        return 0;
    return (volatile uint32_t *)jas_iic_p[thread_id];
}

/* ====================================================================== *
 * THE POLLING STEP: 200 us, NOT 10 ms.
 *
 * These functions run UNDER THE BUS LOCK (see the wrappers at the bottom of
 * the file), and the lock is GLOBAL: while a temperature read waits for its
 * sensor, the mining loop can neither harvest a nonce nor push a header. An
 * I2C transaction at 100 kHz lasts about 400 us, so 10 ms steps sleep
 * twenty-five times too long, lock in hand.
 *
 * THE STEP CHANGES, NOT THE BUDGET: the number of rounds is multiplied
 * accordingly, the total deadline stays ~1 s, and a really silent chip
 * fails at the same moment; only the NORMAL case stops being expensive.
 *
 * Splitting the lock per board would be the next step, and a much riskier
 * one: the hard reset functions hold the whole PL in reset and rely on this
 * lock to exclude EVERYONE (hal/hal_devmem.c, jas_link_hard_reset*).
 * ====================================================================== */
#define IIC_STEP_US      200u        /* polling granularity */
#define IIC_ROUNDS       5050u       /* x 200 us = ~1.01 s, like 0x65 x 10 ms */

/*
 * Wait for a condition on the status register to become true.
 * Returns 1 if it did, 0 if the deadline passed.
 *
 * `ready`: we wait for (SR & SR_MASK) == SR_READY.
 * otherwise: we wait for SR_RX_EMPTY to drop.
 */
static int iic_wait(volatile uint32_t *iic, int ready, uint32_t *sr_final)
{
    unsigned rounds;

    for (rounds = 0; rounds < IIC_ROUNDS; rounds++) {
        uint32_t sr = iic[IIC_SR / 4];

        if (ready ? ((sr & SR_MASK) == SR_READY) : !(sr & SR_RX_EMPTY)) {
            if (sr_final)
                *sr_final = sr;
            return 1;
        }
        usleep(IIC_STEP_US);
    }
    if (sr_final)
        *sr_final = iic[IIC_SR / 4];
    return 0;
}

/*
 * iic_initialization_op @ 0x1b0e4.
 *
 * Long suspected of having frozen a machine; a trace in flash showed
 * the hang was elsewhere, and /proc/<pid>/maps confirms that address
 * 0x41600000 is indeed the one the firmware maps.
 */
void iic_initialization_op_nolock(int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);

    if (!iic)
        return;
    iic[IIC_SOFTR / 4] = 10;
    usleep(20000);
    iic[IIC_CR / 4] = 0;
    usleep(20000);
    iic[IIC_RX_PIRQ / 4] = 9;
    iic[IIC_CR / 4] = 9;
}

/*
 * iic_multi_byte_read @ 0x1b530.
 *
 * The bytes received are stored IN REVERSE: the first one read goes into
 * data[len-1]. This explains the CONCAT11(buff[0], buff[1]) of
 * mcp2973_set_vol — buff[0] carries the most significant byte.
 */
int iic_multi_byte_read(uint8_t device_id, uint8_t address,
                        uint8_t *data, int len, int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);
    uint8_t *p;
    uint32_t sr;

    if (!iic || len <= 0)
        return -1;

    if (!iic_wait(iic, 1, &sr)) {
        jas_log("iic: bus never ready (SR=0x%02x)", sr);
        iic_initialization_op(thread_id);
        return -1;
    }

    usleep(1000);
    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x100;   /* start, write */
    iic[IIC_TX / 4] = address;                              /* the register */
    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x101;   /* restart, read   */
    iic[IIC_TX / 4] = (uint32_t)len | 0x200;                /* stop + length */

    p = data + len;
    do {
        if (iic[IIC_SR / 4] & SR_RX_EMPTY) {
            if (!iic_wait(iic, 0, &sr)) {
                jas_log("iic: nothing comes (SR=0x%02x)", sr);
                iic_initialization_op(thread_id);
                return -1;
            }
        }
        p--;
        *p = (uint8_t)iic[IIC_RX / 4];
    } while (p != data);

    return 0;
}

/*
 * iic_single_byte_write @ 0x1b12c and iic_single_byte_read @ 0x1b1f8.
 */
int iic_single_byte_write_nolock(uint8_t device_id, uint8_t address,
                          uint8_t data, int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);

    if (!iic)
        return -1;
    if ((iic[IIC_SR / 4] & SR_MASK) != SR_READY)
        return -1;

    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x100;
    iic[IIC_TX / 4] = address;
    iic[IIC_TX / 4] = (uint32_t)data | 0x200;
    /*
     * DO NOT POLL RIGHT AWAY. Here we wait for the controller to be READY
     * again, and it STILL is the moment we finish writing into its queue:
     * the transaction has not started yet. An immediate poll would read
     * "ready" and return success BEFORE the byte has left (on the voltage
     * path, moving on while the write is still in flight). On the READ
     * path, where we wait for SR_RX_EMPTY to drop, the condition is false at
     * the start, so polling at once is safe.
     *
     * So the controller gets 200 us to take the bus (it does within a few
     * AXI cycles), then we poll: ~500 us instead of a blind 100 ms sleep,
     * with the ordering guarantee kept.
     */
    usleep(IIC_STEP_US);
    if (!iic_wait(iic, 1, NULL))
        return -1;
    return 0;
}

int iic_single_byte_read_nolock(uint8_t device_id, uint8_t address,
                         uint8_t *data, int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);

    if (!iic)
        return -1;
    if ((iic[IIC_SR / 4] & SR_MASK) != SR_READY)
        return -1;

    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x100;
    iic[IIC_TX / 4] = address;
    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x101;
    iic[IIC_TX / 4] = 0x201;

    /* This is the path the temperature thread takes, once a second and per
     * board: it held the lock 1 ms at best, 1.01 s at worst. Now ~400 us at
     * best, same deadline at worst. */
    if (!iic_wait(iic, 0, NULL))
        return -1;
    *data = (uint8_t)iic[IIC_RX / 4];
    return 0;
}

/*
 * iic_multi_byte_write @ 0x1b34c.
 *
 * The bytes go out IN REVERSE: data[len-1] first, data[0] last with the stop
 * bit. For VOUT_COMMAND, this puts the least significant byte on the wire
 * first — the usual PMBus order.
 */
int iic_multi_byte_write(uint8_t device_id, uint8_t address,
                         const uint8_t *data, int len, int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);
    int rounds, k;

    if (!iic || len <= 0)
        return -1;
    if ((iic[IIC_SR / 4] & SR_MASK) != SR_READY)
        return -1;

    usleep(1000);
    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x100;
    iic[IIC_TX / 4] = address;
    for (k = len - 1; k >= 1; k--)
        iic[IIC_TX / 4] = data[k];
    iic[IIC_TX / 4] = (uint32_t)data[0] | 0x200;
    usleep(100000);

    rounds = 0x65;
    while ((iic[IIC_SR / 4] & SR_MASK) != SR_READY) {
        usleep(10000);
        if (--rounds == 0)
            return -1;
    }
    return 0;
}

/*
 * iic_mux_select @ 0x1b45c — choose the bus multiplexer channel.
 *
 * The three temperature sensors share a single pair of wires; a PCA9548 at
 * address 0x70 switches to one or the other. The channel number is NOT sent
 * as is: it is 1 << data, a sixteen-bit mask pushed in two bytes. Hence the
 * wiring observed — sensor 0 on channel 2, sensor 1 on channel 0, sensor 2 on
 * channel 1.
 *
 * The sequence is that of a raw IIC write, without going through
 * iic_single_byte_write: start plus address (0x100), then the two bytes of
 * the mask, the last one carrying the stop bit (0x200).
 *
 * Fifteen milliseconds after the switch, otherwise the sensor still answers
 * for the previous channel.
 */
int iic_mux_select_nolock(uint8_t device_id, const uint8_t *data, int thread_id)
{
    volatile uint32_t *iic = iic_of(thread_id);
    uint32_t sr;

    if (!iic)
        return -1;

    if (!iic_wait(iic, 1, &sr)) {
        jas_log("SR = 0x%x, select SR reg error, iic init", sr);
        iic_initialization_op(thread_id);
        return -1;
    }

    usleep(1000);
    iic[IIC_TX / 4] = ((uint32_t)device_id << 1) | 0x100u;
    iic[IIC_TX / 4] = (uint32_t)((1u << *data) & 0xffffu) >> 8;
    iic[IIC_TX / 4] = ((1u << *data) & 0xffu) | 0x200u;
    /*
     * THE FIFTEEN MILLISECONDS STAY. It is not a poll that could be shortened:
     * it is the time the MULTIPLEXER takes to switch channel. The header
     * comment says it — without them, the sensor still answers for the
     * previous channel, and we read the neighbour's temperature. No status
     * register signals this switch; so we can only wait for it.
     */
    usleep(15000);
    return 0;
}

/*
 * mcp2973_init @ 0x1bde8 — choose the page and check the regulator.
 *
 * rail 1 -> page 0, rail 2 -> page 1. Then two consistency checks:
 * OPERATION (0x01) must read 0x80 and ON_OFF_CONFIG (0x02) 0x1e. If either is
 * missing, the original refuses to write — so do we.
 */
static int mcp2973_init(uint8_t slave_addr, uint8_t rail, int thread_id)
{
    uint8_t v = 0;

    if (rail == 2)
        v = 1;
    else if (rail != 1)
        v = 0;

    iic_single_byte_write(slave_addr, 0x00, v, thread_id);
    iic_single_byte_read(slave_addr, 0x00, &v, thread_id);

    v = 0;
    if (iic_single_byte_read(slave_addr, 0x01, &v, thread_id) < 0 || v != 0x80) {
        jas_log("mcp2973 0x%02x: OPERATION = 0x%02x, expected 0x80", slave_addr, v);
        return -1;
    }
    if (iic_single_byte_read(slave_addr, 0x02, &v, thread_id) < 0 || v != 0x1e) {
        jas_log("mcp2973 0x%02x: ON_OFF_CONFIG = 0x%02x, expected 0x1e", slave_addr, v);
        return -1;
    }
    return 0;
}

/*
 * mcp2973_set_vol @ 0x1be94: set the voltage, then read it back.
 *
 * The VOUT_COMMAND encoding is NOT that of READ_VOUT:
 *     write  0x21 : raw = mV / 5 - 0x31        860 mV -> 123
 *     read   0x8b : returns millivolts directly
 *
 * After the ASIC power cut done by power_on_operation, the regulators
 * restart at ~901 mV; they are set again because the firmware does so.
 */
int mcp2973_set_vol(uint8_t slave_addr, uint8_t rail, int mv, int thread_id)
{
    uint8_t buf[2];
    uint16_t readback = 0;
    int raw;

    switch (slave_addr) {
    case 0x60: case 0x62: case 0x64: case 0x66:
    case 0x69: case 0x6a: case 0x6b:
        break;
    default:
        jas_log("mcp2973: unknown address 0x%02x, nothing written", slave_addr);
        return 0;
    }

    if (mcp2973_init(slave_addr, rail, thread_id) < 0)
        return -1;

    raw = mv / 5 - 0x31;
    buf[1] = (uint8_t)(raw & 0xff);
    buf[0] = (uint8_t)((raw >> 8) & 0xff);
    iic_multi_byte_write(slave_addr, 0x21, buf, 2, thread_id);
    sleep(1);

    {
        uint8_t r[2] = { 0, 0 };
        if (iic_multi_byte_read(slave_addr, 0x8b, r, 2, thread_id) == 0)
            readback = (uint16_t)((r[0] << 8) | r[1]);
    }
    printf("  rail %u (0x%02x): setpoint %d mV, raw %d, read back %u mV\n",
           rail, slave_addr, mv, raw, readback);
    return 0;
}

/*
 * asic_pmbus_config @ 0x1c698 — with ITS BOUNDS, reproduced as they are.
 *
 * Outside these ranges the original writes nothing, and it is our only
 * protection against an aberrant value:
 *     rail0 [800, 1201)   rail1 [1000, 1251)   rail2 [1300, 1601)
 *
 * In mode 0x500 — ours — a single voltage is set: rail0, page 1.
 */
void asic_pmbus_config(uint16_t mode, int thread_id, unsigned asic,
                       uint16_t rail0, uint16_t rail1, uint16_t rail2)
{
    static const uint8_t adr_500[3] = { 0x6a, 0x60, 0x64 };

    if ((uint16_t)(rail0 - 800) >= 0x191 ||
        (uint16_t)(rail1 - 1000) >= 0xfb ||
        (uint16_t)(rail2 - 1300) >= 0x12d) {
        jas_log("pmbus: voltages out of range (%u/%u/%u), nothing written",
                rail0, rail1, rail2);
        return;
    }
    if (asic > 2)
        return;

    if (mode == 0x500) {
        mcp2973_set_vol(adr_500[asic], 1, rail0, thread_id);
    } else {
        jas_log("pmbus: mode 0x%03x not implemented yet, nothing written", mode);
    }
}

/*
 * OFF THE EXECUTION PATH (tools/atteignable.py): no live function calls
 * this one in the running firmware. Kept as a tool.
 */
/*
 * pmbus_set_current @ 0x1bffc — set the regulator's current limit.
 *
 * It is the ONLY function of the reimplementation that writes something
 * other than a voltage into a regulator, and its protection deserves a read
 * before using it: it reads the voltage back TWICE, a hundred milliseconds
 * apart, and refuses to write if either one is outside [vol_min, vol_max].
 * It is not gratuitous caution — it is the only way to make sure we are
 * talking to the intended regulator, and not to a silent I2C address that
 * would return 0x0000 or 0xffff.
 *
 * This guard is reproduced as is. Do not lighten it.
 *
 * Register 0x5f is not in the standard PMBus catalogue; it is a register
 * specific to the MCP2973, which the firmware calls "current_limit". It is
 * read and written on two bytes, most significant first — like VOUT_COMMAND.
 *
 * Same rail convention as mcp2973_init: rail 1 -> page 0, rail 2 -> page 1,
 * anything else is an error (the original writes "rail eror", typo
 * included).
 */
int pmbus_set_current(int thread_id, uint8_t slave_addr, uint8_t rail,
                      uint16_t vol_min, uint16_t vol_max, uint16_t limit_val)
{
    uint8_t buf[2] = { 0, 0 };
    uint8_t page;
    unsigned readback;
    int round;

    if (rail == 1)
        page = 0;
    else if (rail == 2)
        page = 1;
    else {
        printf("rail eror\n");
        return -1;
    }

    usleep(100000);
    iic_single_byte_write(slave_addr, 0x00, page, thread_id);

    /* The two control reads. The original writes them out in full, one
     * after the other, with two distinct error messages — "vol out of
     * range" then "vol out of range!!!". The exclamation marks are the only
     * way to know which of the two failed. */
    for (round = 0; round < 2; round++) {
        if (round)
            usleep(100000);
        buf[0] = buf[1] = 0;
        iic_multi_byte_read(slave_addr, 0x8b, buf, 2, thread_id);
        readback = (unsigned)((buf[0] << 8) | buf[1]);
        printf("slave %x, rail[%d] --> read_vol = %dmv\n", slave_addr, rail, readback);
        if (readback < vol_min || readback > vol_max) {
            printf(round ? "vol out of range!!!\n" : "vol out of range\n");
            return -1;
        }
    }

    buf[0] = buf[1] = 0;
    iic_multi_byte_read(slave_addr, 0x5f, buf, 2, thread_id);
    printf("slave %x, rail[%d] --> read_current_limit = %x\n",
           slave_addr, rail, (buf[0] << 8) | buf[1]);

    buf[0] = (uint8_t)(limit_val >> 8);
    buf[1] = (uint8_t)(limit_val & 0xff);
    iic_multi_byte_write(slave_addr, 0x5f, buf, 2, thread_id);

    buf[0] = buf[1] = 0;
    iic_multi_byte_read(slave_addr, 0x5f, buf, 2, thread_id);
    printf("slave %x, rail[%d] --> write_current_limit = %x\n",
           slave_addr, rail, (buf[0] << 8) | buf[1]);
    return 0;
}

/*
 * The three regulators, one per chip, as asic_pmbus_config addresses them in
 * mode 0x500 — ours. A single voltage is set there, rail0, page 1.
 */
static const uint8_t pmbus_addr[3] = { 0x6a, 0x60, 0x64 };

int jas_pmbus_read_vout(int thread_id, unsigned asic, uint16_t *raw)
{
    uint8_t buf[2] = { 0, 0 };

    if (asic > 2)
        return -1;
    if (iic_multi_byte_read(pmbus_addr[asic], 0x8b, buf, 2, thread_id) < 0)
        return -1;
    *raw = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

const uint8_t *jas_pmbus_addresses(void) { return pmbus_addr; }

/* ======================================================================
 * READING THE VOLTAGES — the counterpart of mcp2973_set_vol.
 *
 * TWO REGULATOR FAMILIES, AND TWO ENCODINGS. That is the trap:
 *
 *     MCP2973    write 0x21 : raw = mV / 5 - 0x31
 *     RAA228236  write 0x21 : the value AS IS
 *
 * Both read back READ_VOUT (0x8b), which returns millivolts. Setting an
 * MCP2973 setpoint on a RAA228236 would give 123 instead of 860.
 *
 * The PMBus mode decides the family: 0x500 and 0x450 are MCP2973s, 0x510
 * RAA228236s. Our machines are in 0x500.
 * ====================================================================== */

/* The RAA228236 is recognized by two identity registers. */
static int RAA228236_init(uint8_t slave_addr, uint8_t rail, int thread_id)
{
    uint8_t v = 0;

    if (rail == 2)
        v = 1;
    else if (rail != 1)
        jas_log("RAA228236: unknown rail %u, page 0 by default", rail);

    iic_single_byte_write(slave_addr, 0x00, v, thread_id);
    iic_single_byte_read(slave_addr, 0x00, &v, thread_id);

    v = 0;
    iic_single_byte_read(slave_addr, 0x01, &v, thread_id);
    if ((v & 0x7f) != 8) {
        jas_log("RAA228236 0x%02x: identity 0x01 = 0x%02x, expected 8", slave_addr, v);
        return -1;
    }
    iic_single_byte_read(slave_addr, 0x02, &v, thread_id);
    if ((v & 0xf7) != 0x16) {
        jas_log("RAA228236 0x%02x: identity 0x02 = 0x%02x, expected 0x16", slave_addr, v);
        return -1;
    }
    return 0;
}

int mcp2973_get_vol(uint8_t slave_addr, uint8_t rail, uint16_t *output, int thread_id)
{
    uint8_t buf[2] = { 0, 0 };

    switch (slave_addr) {
    case 0x60: case 0x62: case 0x64: case 0x66:
    case 0x69: case 0x6a: case 0x6b: case 0x6d:
        break;
    default:
        jas_log("mcp2973: unknown address 0x%02x, nothing read", slave_addr);
        return 0;
    }
    if (mcp2973_init(slave_addr, rail, thread_id) < 0)
        return -1;
    iic_multi_byte_read(slave_addr, 0x8b, buf, 2, thread_id);
    *output = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

int RAA228236_get_vol(uint8_t slave_addr, uint8_t rail, uint16_t *output, int thread_id)
{
    uint8_t buf[2] = { 0, 0 };

    if ((unsigned)(slave_addr - 0x60) >= 3) {
        jas_log("RAA228236: unknown address 0x%02x, nothing read", slave_addr);
        return 0;
    }
    if (RAA228236_init(slave_addr, rail, thread_id) < 0)
        return -1;
    iic_multi_byte_read(slave_addr, 0x8b, buf, 2, thread_id);
    *output = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

int RAA228236_set_vol(uint8_t slave_addr, uint8_t rail, int value, int thread_id)
{
    uint8_t buf[2];
    uint16_t readback = 0;

    if ((unsigned)(slave_addr - 0x60) >= 3) {
        jas_log("RAA228236: unknown address 0x%02x, nothing written", slave_addr);
        return 0;
    }
    if (RAA228236_init(slave_addr, rail, thread_id) < 0)
        return -1;

    /* AS IS, without the MCP2973 conversion. */
    buf[1] = (uint8_t)(value & 0xff);
    buf[0] = (uint8_t)((value >> 8) & 0xff);
    iic_multi_byte_write(slave_addr, 0x21, buf, 2, thread_id);
    sleep(1);

    buf[0] = buf[1] = 0;
    iic_multi_byte_read(slave_addr, 0x8b, buf, 2, thread_id);
    readback = (uint16_t)((buf[0] << 8) | buf[1]);
    jas_log("RAA228236 rail %u: setpoint %d, read back %u mV", rail, value, readback);
    return 0;
}

/*
 * set_asic_rail_vol — set a voltage through the multiplexer.
 *
 * It differs from mcp2973_set_vol on two points that matter: it first
 * switches the multiplexer (channel 0), and it chooses the page by MODIFYING
 * register 0 rather than overwriting it — `& 0xfc`, then `| 1` for rail 2.
 * The two most significant bits are thus preserved.
 *
 * Three seconds of waiting after the write, against one for mcp2973_set_vol.
 */
void set_asic_rail_vol(int thread_id, uint8_t asic_num, uint8_t rail_num,
                       uint8_t iic_slave_addr, uint16_t vol)
{
    uint8_t channel = 0, reg = 0, buf[2];
    uint16_t raw, readback = 0;

    iic_mux_select(iic_slave_addr, &channel, thread_id);

    iic_single_byte_read(iic_slave_addr, 0x00, &reg, thread_id);
    reg &= 0xfc;
    if (rail_num == 2)
        reg |= 1;
    iic_single_byte_write(iic_slave_addr, 0x00, reg, thread_id);

    raw = (uint16_t)(vol / 5 - 0x31);
    buf[1] = (uint8_t)(raw & 0xff);
    buf[0] = (uint8_t)(raw >> 8);
    iic_multi_byte_write(iic_slave_addr, 0x21, buf, 2, thread_id);
    sleep(3);

    buf[0] = buf[1] = 0;
    iic_multi_byte_read(iic_slave_addr, 0x8b, buf, 2, thread_id);
    readback = (uint16_t)((buf[0] << 8) | buf[1]);
    jas_log("asic %u rail %u: read back %u mV", asic_num, rail_num, readback);
}

/*
 * asic_pmbus_read_vol — the voltages of a board's three chips.
 *
 * read_vol holds nine entries: three rails per chip. In modes 0x500 and 0x510
 * a single rail is read per chip; in 0x450 all three are, on two regulators.
 */
void asic_pmbus_read_vol(uint16_t mode, int thread_id, uint16_t *read_vol)
{
    /* The addresses, per mode and per chip. 0 = no regulator at this index. */
    static const uint8_t adr_500[3]     = { 0x6a, 0x60, 0x64 };
    static const uint8_t adr_510[3]     = { 0x62, 0x60, 0x61 };
    static const uint8_t adr_450_a[3]   = { 0x6a, 0x62, 0x64 };
    static const uint8_t adr_450_b[3]   = { 0x6b, 0x63, 0x66 };
    unsigned chip;

    for (chip = 0; chip < 3; chip++) {
        if (!(ASIC_EN(thread_id) >> chip & 1))
            continue;
        switch (mode) {
        case 0x500:
            mcp2973_get_vol(adr_500[chip], 1, &read_vol[chip * 3], thread_id);
            break;
        case 0x510:
            RAA228236_get_vol(adr_510[chip], 1, &read_vol[chip * 3], thread_id);
            break;
        case 0x450:
            mcp2973_get_vol(adr_450_a[chip], 1, &read_vol[chip * 3 + 0], thread_id);
            mcp2973_get_vol(adr_450_b[chip], 1, &read_vol[chip * 3 + 1], thread_id);
            mcp2973_get_vol(adr_450_b[chip], 2, &read_vol[chip * 3 + 2], thread_id);
            break;
        default:
            jas_log("pmbus: unknown mode 0x%03x, nothing read", mode);
            return;
        }
    }
}

/* ------------------------------------------------------------------
 * LOCKING WRAPPERS. Three threads share the AXI windows (mining,
 * monitoring, display): a PL hard reset from the mining error path
 * during an I2C read of the monitoring thread would leave an AXI
 * transaction unacknowledged and stall the ARM core until a power
 * cut. The lock is recursive: main.c still wraps whole sequences,
 * these wrappers protect each isolated access.
 * ------------------------------------------------------------------ */

void iic_initialization_op(int thread_id)
{
    jas_bus_lock();
    iic_initialization_op_nolock(thread_id);
    jas_bus_unlock();
}

int iic_single_byte_write(uint8_t device_id, uint8_t address, uint8_t data, int thread_id)
{
    int r;
    jas_bus_lock();
    r = iic_single_byte_write_nolock(device_id, address, data, thread_id);
    jas_bus_unlock();
    return r;
}

int iic_single_byte_read(uint8_t device_id, uint8_t address, uint8_t *data, int thread_id)
{
    int r;
    jas_bus_lock();
    r = iic_single_byte_read_nolock(device_id, address, data, thread_id);
    jas_bus_unlock();
    return r;
}

int iic_mux_select(uint8_t device_id, const uint8_t *data, int thread_id)
{
    int r;
    jas_bus_lock();
    r = iic_mux_select_nolock(device_id, data, thread_id);
    jas_bus_unlock();
    return r;
}
