/*
 * Running test sequences through the DFT state machine.
 *
 * These are the functions that really put the memory to work: a sequence of
 * commands is loaded into the mesh's state machine, triggered, and we wait
 * for its verdict. The rest of the DFT block — primitives and readouts —
 * exists only to frame them.
 *
 * Two flags tell a normal end from an expired deadline: 0xd28 says the
 * sequence is finished, 0xd29 that it finished by expiry. Reading them in
 * this order is necessary — a sequence can declare itself finished while
 * having failed.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "sl_transfer.h"
#include "dft.h"
#include <string.h>

#define DFT_W(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0x8u))
#define DFT_R(mesh)  ((uint8_t)(((1u << ((mesh) & 0xf)) << 4) | 0xcu))
#define ASIC(i)      ((uint8_t)(1u << (i)))

#define REG_TIMEOUT     0xd03   /* deadline, in units of 50 ns */
#define REG_SEQ_DATA    0xd0a   /* the sequence, 22 words */
#define REG_MR_CONFIG   0xd20   /* register read configuration */
#define REG_SEQ_DONE    0xd28
#define REG_SEQ_TIMEOUT 0xd29
#define REG_SEQ_STROBE  0xd2a
#define REG_MRR_STROBE  0xd2d
#define REG_MRR_DONE    0xd30
#define REG_TRIM_0      0xd60
#define REG_TRIM_1      0xd61
#define REG_TRIM_2      0xd62
#define REG_TRIM_3      0xd63
#define REG_TRIM_APPLY  0xd6d

/* The four reference voltages, in the original's unexpected order. */
void dft_apply_trim(int thread_id, uint16_t i, uint16_t mesh,
                    const uint16_t *lr_trim)
{
    uint8_t a = ASIC(i), c = DFT_W(mesh);

    sl_single_write(a, c, REG_TRIM_3, lr_trim[1], thread_id);
    sl_single_write(a, c, REG_TRIM_2, lr_trim[0], thread_id);
    sl_single_write(a, c, REG_TRIM_1, lr_trim[3], thread_id);
    sl_single_write(a, c, REG_TRIM_0, lr_trim[2], thread_id);
    sl_single_write(a, c, REG_TRIM_APPLY, 1, thread_id);
}

/*
 * Reading the memory's mode registers ("mode register read").
 *
 * A FIXED wait, then a simple flag: the original does not loop here, it waits
 * once and concludes. This readout feeds the MRR verdict on which, in the
 * end, the acceptance of a bank depends.
 *
 * TEN MILLISECONDS, the factory value (the vendor profile carries
 * mrr_read_delay_ns = 10 000 000). The wait does not bound a read, it gives
 * the memory time to present its mode register: too short, and the `done`
 * flag may fall back on an incomplete readout, which decides which banks are
 * accepted, hence the integrity of the DAG. Do not shorten it without a
 * measurement.
 */
int dft_MRR_read(int thread_id, uint16_t i, uint16_t mesh,
                 uint16_t *lr_trim, uint16_t *ltc_r_mr_config)
{
    uint8_t a = ASIC(i);
    uint16_t done = 0;

    dft_apply_trim(thread_id, i, mesh, lr_trim);
    sl_multi_write(a, DFT_W(mesh), REG_MR_CONFIG, ltc_r_mr_config, 8, thread_id);
    sl_single_write(a, DFT_W(mesh), REG_MRR_STROBE, 1, thread_id);
    sl_delay(0, 10000000);      /* 10 ms — factory value, see header */
    sl_single_read(a, DFT_R(mesh), REG_MRR_DONE, &done, 1000000, thread_id);

    if (done)
        return 0;

    jas_trace("board %d asic %u mesh %u: MRR timeout", thread_id, i, mesh);
    return -1;
}

/*
 * Loads and runs a twenty-two-word sequence.
 *
 * timeout is in milliseconds on the caller's side; the state machine wants it
 * in units of fifty nanoseconds, hence the factor twenty thousand. The number
 * of wait rounds is computed to cover the deadline with a ten percent margin —
 * the software timing and the hardware's do not share the same time base.
 *
 * With a zero timeout, the sequence is started without waiting: the caller
 * will collect the result later.
 */
int dft_cmd_multi_write(int thread_id, uint16_t i, uint16_t mesh,
                        uint16_t *lr_trim, uint16_t *cmd_din, uint32_t timeout)
{
    uint8_t a = ASIC(i), cw = DFT_W(mesh), cr = DFT_R(mesh);
    uint32_t hw_timeout = timeout * 20000u;
    uint16_t done = 0, expired = 0;
    int tries;

    dft_apply_trim(thread_id, i, mesh, lr_trim);
    sl_multi_write(a, cw, REG_SEQ_DATA, cmd_din, 0x16, thread_id);

    if (timeout == 0) {
        sl_single_write(a, cw, REG_SEQ_STROBE, 1, thread_id);
        sl_delay(0, 1000);
        sl_single_write(a, cw, REG_SEQ_STROBE, 0, thread_id);
        return 0;
    }

    sl_multi_write(a, cw, REG_TIMEOUT, (uint16_t *)&hw_timeout, 2, thread_id);
    sl_single_write(a, cw, REG_SEQ_STROBE, 1, thread_id);
    sl_delay(0, 1000);

    for (tries = (int)(timeout + 1 + timeout / 10); tries > 0; tries--) {
        sl_single_read(a, cr, REG_SEQ_DONE, &done, 1000000, thread_id);
        if (!done) {
            sl_delay(0, 1000);
            continue;
        }
        /* Finished — remains to know whether by success or by deadline. */
        sl_single_read(a, cr, REG_SEQ_TIMEOUT, &expired, 1000000, thread_id);
        sl_single_write(a, cw, REG_SEQ_STROBE, 0, thread_id);
        if (expired) {
            jas_trace("board %d asic %u mesh %u: sequence timeout (hardware flag)",
                    thread_id, i, mesh);
            return -1;
        }
        return 0;
    }

    jas_trace("board %d asic %u mesh %u: sequence not answering", thread_id, i, mesh);
    sl_single_write(a, cw, REG_SEQ_STROBE, 0, thread_id);
    return -1;
}

/*
 * Writes a long command into the DFT state machine and waits for its verdict.
 *
 * A short command fits in two words (dft_write_single_cmd); this one takes
 * eleven, pushed one by one into numbered slots. The order is the original's
 * and it is not natural: slots 1 to 9 first, then slot 0, then a last write
 * of zero that leaves the validation flag high — that is what really starts
 * the test, and the state machine lowers it by itself.
 *
 * The wait delay is in units of 20000 cycles, and the resulting number of
 * polls — timeout/10 + timeout*2 + 1 — is taken as is: it has no obvious
 * interpretation, better not to make one up.
 */

#define REG_SLOT_INDEX  0xd07
#define REG_SLOT_DATA   0xd08
#define REG_SLOT_STROBE 0xd2b
#define REG_TIMEOUT     0xd03
#define REG_TEST_DONE   0xd28
#define S2M_WORDS       10

/* Pushes a 32-bit word into a numbered slot of the state machine. */
static void ltc_write_single_cmd(int thread_id, uint16_t i, uint16_t mesh,
                                 uint16_t *din, uint16_t slot)
{
    uint8_t a = ASIC(i), c = DFT_W(mesh);

    sl_single_write(a, c, REG_SLOT_INDEX, slot, thread_id);
    sl_multi_write(a, c, REG_SLOT_DATA, din, 2, thread_id);
    sl_single_write(a, c, REG_SLOT_STROBE, 1, thread_id);
    sl_single_write(a, c, REG_SLOT_STROBE, 0, thread_id);
}

/* Same, but leaves the flag high: that starts the test. */
static void ltc_write_single_cmd_wait_done(int thread_id, uint16_t i, uint16_t mesh,
                                           uint16_t *din, uint16_t slot)
{
    uint8_t a = ASIC(i), c = DFT_W(mesh);

    sl_single_write(a, c, REG_SLOT_INDEX, slot, thread_id);
    sl_multi_write(a, c, REG_SLOT_DATA, din, 2, thread_id);
    sl_single_write(a, c, REG_SLOT_STROBE, 1, thread_id);
}

int dft_cmd_s2m_write(int thread_id, uint16_t i, uint16_t mesh,
                      uint16_t *lr_trim, uint16_t *cmd_din, uint32_t timeout)
{
    uint8_t a = ASIC(i), cw = DFT_W(mesh), cr = DFT_R(mesh);
    uint32_t temp[S2M_WORDS + 1];
    uint32_t scaled = timeout * 20000u;
    int rounds;
    unsigned k;

    dft_apply_trim(thread_id, i, mesh, lr_trim);

    if (timeout)
        sl_multi_write(a, cw, REG_TIMEOUT, (uint16_t *)&scaled, 2, thread_id);

    memset(temp, 0, sizeof(temp));
    memcpy(temp, cmd_din, S2M_WORDS * sizeof(uint32_t));

    /* Slots 1 to 9, then slot 0 — the original's order. */
    for (k = 0; k < S2M_WORDS - 1; k++)
        ltc_write_single_cmd(thread_id, i, mesh, (uint16_t *)&temp[k],
                             (uint16_t)(k + 1));
    ltc_write_single_cmd(thread_id, i, mesh, (uint16_t *)&temp[S2M_WORDS - 1], 0);

    /* And the start. */
    ltc_write_single_cmd_wait_done(thread_id, i, mesh,
                                   (uint16_t *)&temp[S2M_WORDS], 0);
    sl_delay(0, 1000);

    if (!timeout)
        return 0;

    rounds = (int)(timeout / 10 + timeout * 2 + 1);
    while (rounds > 0) {
        uint16_t done = 0;

        sl_single_read(a, cr, REG_TEST_DONE, &done, 1000000, thread_id);
        if (done) {
            sl_single_write(a, cw, REG_SLOT_STROBE, 0, thread_id);
            return 0;
        }
        sl_delay(0, 1000);
        rounds--;
    }

    sl_single_write(a, cw, REG_SLOT_STROBE, 0, thread_id);
    jas_trace("board %d asic %u mesh %u: dft test gave no verdict (timeout)",
            thread_id, i, mesh);
    return -1;
}
