/*
 * Serial link to the ASICs — low level.
 *
 * The protocol is simple and fully visible in sl_single_write: a register
 * window mapped in memory, a command word, a data word, a trigger, and a busy
 * bit to wait on. There is no kernel driver: the programmable logic is
 * reached through /dev/mem.
 *
 *   base = g_mapBase[thread_id] + 0x20000
 *   +12  status register, bit 3 = busy
 *   +4   command word: asic_en | cmd<<8 | addr<<16
 *   +8   data word
 *   +64  trigger (write 1)
 *
 * The original builds the command word byte by byte, which makes it
 * independent of endianness; we keep that form rather than a cast, so the
 * reimplementation stays checkable line by line against the assembly.
 */

#include "jas_hal.h"
#include <sys/time.h>
#include "sl_transfer.h"
#include "jas_types.h"
#include <time.h>

#define SL_WINDOW   0x20000u   /* offset of the register window */
#define SL_STATUS   3          /* +12: bit 3 = transfer in progress     */
#define SL_CMD      1          /* +4                                    */
#define SL_DATA     2          /* +8                                    */
#define SL_TRIGGER  16         /* +64                                   */
#define SL_BUSY     0x8u

/* sl_transfer.c:639 */
void sl_delay(time_t sec, long ns)
{
    struct timespec ts;
    ts.tv_sec = sec;
    ts.tv_nsec = ns;
    nanosleep(&ts, NULL);
}

/* sl_transfer.c:469 */
/*
 * Waits until the link is free again. Returns 0 if it never is.
 */
static int sl_wait_idle(volatile uint32_t *r, int thread_id)
{
    struct timeval start, now;
    long us;
    int retried = 0;

    if (!(r[SL_STATUS] & SL_BUSY))
        return 1;                       /* the ordinary case, without a clock */

    gettimeofday(&start, NULL);
    for (;;) {
        if (!(r[SL_STATUS] & SL_BUSY))
            return 1;
        gettimeofday(&now, NULL);
        us = (now.tv_sec - start.tv_sec) * 1000000L
           + (now.tv_usec - start.tv_usec);
        if (us <= 2000000L)
            continue;
        if (retried) {
            jas_log("board %d: link stuck (state 0x%08x), write abandoned",
                    thread_id, r[SL_STATUS]);
            return 0;
        }
        jas_log("board %d: link stuck (state 0x%08x), recovered",
                thread_id, r[SL_STATUS]);
        jas_link_reset(thread_id);
        retried = 1;
        gettimeofday(&start, NULL);
    }
}

void sl_single_write_nolock(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                     uint16_t data, int thread_id)
{
    transfer_data_t t;
    volatile uint32_t *r;

    t.asic_en = asic_en;
    t.cmd     = cmd;
    t.addr0   = (uint8_t)(addr & 0xff);
    t.addr1   = (uint8_t)(addr >> 8);
    t.data0   = (uint8_t)(data & 0xff);
    t.data1   = (uint8_t)(data >> 8);

    r = jas_axi_base(thread_id);
    if (!r) return;
    r = (volatile uint32_t *)((volatile unsigned char *)r + SL_WINDOW);

    /*
     * DELIBERATE DIVERGENCE: the original waits here without bound. As long
     * as the firmware drives the machine, the busy bit always ends up
     * dropping; but a link left hanging (an interrupted transaction, a
     * process killed in the middle of an exchange) leaves it high for good,
     * and the next transfer would wait forever. The bound is wide (two
     * seconds, thousands of times a healthy transaction), so normal
     * behaviour is unchanged.
     */
    if (!sl_wait_idle(r, thread_id))
        return;

    r[SL_CMD]     = (uint32_t)t.asic_en | ((uint32_t)t.cmd   << 8)
                  | ((uint32_t)t.addr0 << 16) | ((uint32_t)t.addr1 << 24);
    r[SL_DATA]    = (uint32_t)t.data0   | ((uint32_t)t.data1 << 8);
    r[SL_TRIGGER] = 1;
}

/*
 * A grouped write is just a series of single writes to consecutive
 * addresses: that is why the hook placed on sl_multi_write saw all the
 * traffic, including that of the direct callers of sl_single_write.
 */
void sl_multi_write_nolock(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                    uint16_t *data, uint32_t data_len, int thread_id)
{
    uint16_t *end;

    if (data_len == 0)
        return;

    for (end = data + data_len; data < end; data++) {
        sl_single_write(asic_en, cmd, addr, *data, thread_id);
        addr = (uint16_t)(addr + 1);
    }
}

/*
 * Reading a register.
 *
 * A read is a command write followed by a wait on the status register, then
 * reading back the answer. Two bits matter:
 *
 *   bit 0   an answer is still expected
 *   bit 4   the link is in error
 *
 * Worth noting: the `timeout` parameter of the stock signature is NEVER used.
 * The wait is not bounded in time, it polls the status bits (the "1000000"
 * dag_calc passes is not a one-second delay).
 *
 * The answer contains the echo of the chip mask and of the command: if they
 * do not match what was asked, the value is rejected and 0 is returned to
 * the caller, without it being reported as an error.
 */
#define SL_RESP_HDR  4         /* +16: echo asic_en | cmd<<8 */
#define SL_RESP_DAT  5         /* +20: the data read */
#define SL_ACK       32        /* +128: acknowledgement */
#define SL_PENDING   0x1u
#define SL_LINKERR   0x10u

/*
 * "SOFT" READ — sl_single_read_soft @ 0x1d4bc.
 *
 * Three differences with sl_single_read, and each one matters:
 *
 *   1. it honours the DELAY passed as parameter (in microseconds);
 *   2. it does not test the link's error bit;
 *   3. it does NOT reset the link when it fails.
 *
 * That is what makes it usable to PROBE hardware that may not answer.
 * board_check uses it to discover which connectors carry a daughter board: it
 * writes 0x5e to register 0x30 and reads it back with 5 ms of patience. Using
 * the hard version there would reset the link at every empty connector.
 *
 * As with the hard version, an echo that does not match (asic_en, cmd)
 * returns a null datum — but here the return code stays 0: only the timeout
 * returns -1.
 */
int sl_single_read_soft_nolock(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                        uint16_t *data, uint64_t timeout_us, int thread_id)
{
    volatile uint32_t *r;
    struct timeval start, now;
    uint32_t hdr = 0, val = 0;
    int ret = -1;

    sl_single_write(asic_en, cmd, addr, 0, thread_id);

    r = jas_axi_base(thread_id);
    if (!r) { *data = 0; return -1; }
    r = (volatile uint32_t *)((volatile unsigned char *)r + SL_WINDOW);

    gettimeofday(&start, NULL);
    for (;;) {
        if (!(r[SL_STATUS] & SL_PENDING)) {
            r[SL_ACK] = 1;
            hdr = r[SL_RESP_HDR];
            val = r[SL_RESP_DAT];
            ret = 0;
            break;
        }
        gettimeofday(&now, NULL);
        {
            uint64_t elapsed = (uint64_t)(now.tv_sec - start.tv_sec) * 1000000u
                            + (uint64_t)(now.tv_usec - start.tv_usec);
            if (elapsed > timeout_us)
                break;                      /* no reset: that is the point */
        }
    }

    if (((hdr >> 8) & 0xff) != cmd || (hdr & 0xff) != asic_en)
        val = 0;
    *data = (uint16_t)val;
    return ret;
}


int sl_single_read_nolock(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                   uint16_t *data, uint64_t timeout, int thread_id)
{
    volatile uint32_t *r;
    uint32_t st, hdr;

    (void)timeout;                       /* unused by the original */

    sl_single_write(asic_en, cmd, addr, 0, thread_id);

    r = jas_axi_base(thread_id);
    if (!r) return -1;
    r = (volatile uint32_t *)((volatile unsigned char *)r + SL_WINDOW);

    for (;;) {
        st = r[SL_STATUS];
        if (!(st & SL_PENDING))
            break;
        if (st & SL_LINKERR) {
            jas_log("sl_single_read: link in error, board %d, register 0x%04x",
                    thread_id, addr);
            jas_link_reset(thread_id);
            *data = 0;
            return -1;
        }
    }

    r[SL_ACK] = 1;
    hdr = r[SL_RESP_HDR];
    if (((hdr >> 8) & 0xff) != cmd || (hdr & 0xff) != asic_en) {
        *data = 0;                       /* answer from another exchange */
        return 0;
    }
    *data = (uint16_t)r[SL_RESP_DAT];
    return 0;
}

/* Grouped read: single reads on consecutive addresses, the error being
 * remembered without breaking the loop. */
int sl_multi_read_nolock(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                  uint16_t *data, uint32_t data_len, uint64_t timeout,
                  int thread_id)
{
    uint16_t *end, tmp;
    int ret = 0;

    if (data_len == 0)
        return 0;

    for (end = data + data_len; data < end; data++) {
        if (sl_single_read(asic_en, cmd, addr, &tmp, timeout, thread_id) < 0)
            ret = -1;
        *data = tmp;
        addr = (uint16_t)(addr + 1);
    }
    return ret;
}

/* ------------------------------------------------------------------
 * LOCKING WRAPPERS. Three threads share the AXI windows (mining,
 * monitoring, display): a PL hard reset from the mining error path
 * during an I2C read of the monitoring thread would leave an AXI
 * transaction unacknowledged and stall the ARM core until a power
 * cut. The lock is recursive: main.c still wraps whole sequences,
 * these wrappers protect each isolated access.
 * ------------------------------------------------------------------ */

void sl_single_write(uint8_t asic_en, uint8_t cmd, uint16_t addr, uint16_t data, int thread_id)
{
    jas_bus_lock();
    sl_single_write_nolock(asic_en, cmd, addr, data, thread_id);
    jas_bus_unlock();
}

void sl_multi_write(uint8_t asic_en, uint8_t cmd, uint16_t addr, uint16_t *data, uint32_t data_len, int thread_id)
{
    jas_bus_lock();
    sl_multi_write_nolock(asic_en, cmd, addr, data, data_len, thread_id);
    jas_bus_unlock();
}

int sl_single_read_soft(uint8_t asic_en, uint8_t cmd, uint16_t addr, uint16_t *data, uint64_t timeout_us, int thread_id)
{
    int r;
    jas_bus_lock();
    r = sl_single_read_soft_nolock(asic_en, cmd, addr, data, timeout_us, thread_id);
    jas_bus_unlock();
    return r;
}

int sl_single_read(uint8_t asic_en, uint8_t cmd, uint16_t addr, uint16_t *data, uint64_t timeout, int thread_id)
{
    int r;
    jas_bus_lock();
    r = sl_single_read_nolock(asic_en, cmd, addr, data, timeout, thread_id);
    jas_bus_unlock();
    return r;
}

int sl_multi_read(uint8_t asic_en, uint8_t cmd, uint16_t addr, uint16_t *data, uint32_t data_len, uint64_t timeout, int thread_id)
{
    int r;
    jas_bus_lock();
    r = sl_multi_read_nolock(asic_en, cmd, addr, data, data_len, timeout, thread_id);
    jas_bus_unlock();
    return r;
}
