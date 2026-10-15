/*
 * The machine's serial numbers.
 *
 * WHERE THEY LIVE: in HIDDEN files of /media/configs, one per number.
 *
 *     .ms   machine number                64 hex characters
 *     .cs   control board number          64
 *     .pw   power supply number           32
 *     .hs1  hash board 1                  64
 *     .hs2  hash board 2                  empty on the X16-Q
 *     .hs3  hash board 3
 *
 * HOW THEY ARE PROTECTED. Each file holds hexadecimal text; once converted to
 * bytes, it is AES-128 in CBC mode. read_machine_sn does it in four calls:
 *
 *     read_file(path, buffer, 0);
 *     src = hexToString(buffer, &n);
 *     AES_init_ctx_iv(ctx, key, iv);
 *     AES_CBC_decrypt_buffer(ctx, msg, n);
 *     removeInvalidChars(msg);
 *
 * The key and the IV are two sixteen-byte constants lying side by side in the
 * binary, loaded by load_machine_info into tinyaes_key and tinyaes_iv. The IV
 * is the sequence 00 01 02 ... 0f, signature of the tiny-AES library from
 * which the firmware takes its variable names.
 *
 * The AES is rewritten from tiny-AES's public-domain reference rather than
 * copied out of a decompiler (an error in a substitution table would only
 * show at run time), and validated: the files of a machine decrypt to exactly
 * the strings the stock firmware's API announces for it.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jas_hal.h"

#define N_HASHBOARD 3

/* ====================================================================== *
 * AES-128, decryption only, CBC mode.
 * ====================================================================== */

static const uint8_t sbox_inv[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};

static const uint8_t sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t rcon[11] = {
    0x8d,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36
};

static uint8_t xt(uint8_t x) { return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b)); }

/* Multiplication in GF(2^8), for the inverse column mix. */
static uint8_t gmul(uint8_t a, uint8_t b)
{
    uint8_t r = 0, k;

    for (k = 0; k < 8; k++) {
        if (b & 1) r ^= a;
        a = xt(a);
        b >>= 1;
    }
    return r;
}

static void key_span(uint8_t rk[176], const uint8_t key[16])
{
    unsigned i;

    memcpy(rk, key, 16);
    for (i = 4; i < 44; i++) {
        uint8_t t[4];

        memcpy(t, rk + (i - 1) * 4, 4);
        if (i % 4 == 0) {
            uint8_t u = t[0];

            t[0] = (uint8_t)(sbox[t[1]] ^ rcon[i / 4]);
            t[1] = sbox[t[2]];
            t[2] = sbox[t[3]];
            t[3] = sbox[u];
        }
        rk[i * 4 + 0] = (uint8_t)(rk[(i - 4) * 4 + 0] ^ t[0]);
        rk[i * 4 + 1] = (uint8_t)(rk[(i - 4) * 4 + 1] ^ t[1]);
        rk[i * 4 + 2] = (uint8_t)(rk[(i - 4) * 4 + 2] ^ t[2]);
        rk[i * 4 + 3] = (uint8_t)(rk[(i - 4) * 4 + 3] ^ t[3]);
    }
}

static void block_decrypt(uint8_t s[16], const uint8_t rk[176])
{
    int round;
    unsigned i;
    uint8_t t;

    for (i = 0; i < 16; i++) s[i] ^= rk[160 + i];

    for (round = 9; round >= 0; round--) {
        /* row shift, inverse direction */
        t = s[13]; s[13] = s[9];  s[9]  = s[5];  s[5]  = s[1];  s[1]  = t;
        t = s[2];  s[2]  = s[10]; s[10] = t;
        t = s[6];  s[6]  = s[14]; s[14] = t;
        t = s[3];  s[3]  = s[7];  s[7]  = s[11]; s[11] = s[15]; s[15] = t;

        for (i = 0; i < 16; i++) s[i] = sbox_inv[s[i]];
        for (i = 0; i < 16; i++) s[i] ^= rk[round * 16 + i];

        if (round) {
            for (i = 0; i < 16; i += 4) {
                uint8_t a = s[i], b = s[i+1], c = s[i+2], d = s[i+3];

                s[i+0] = (uint8_t)(gmul(a,14) ^ gmul(b,11) ^ gmul(c,13) ^ gmul(d, 9));
                s[i+1] = (uint8_t)(gmul(a, 9) ^ gmul(b,14) ^ gmul(c,11) ^ gmul(d,13));
                s[i+2] = (uint8_t)(gmul(a,13) ^ gmul(b, 9) ^ gmul(c,14) ^ gmul(d,11));
                s[i+3] = (uint8_t)(gmul(a,11) ^ gmul(b,13) ^ gmul(c, 9) ^ gmul(d,14));
            }
        }
    }
}

static void cbc_decrypt(uint8_t *buf, size_t n, const uint8_t key[16],
                           const uint8_t iv[16])
{
    uint8_t rk[176], previous[16], cipher[16];
    size_t p;

    key_span(rk, key);
    memcpy(previous, iv, 16);
    for (p = 0; p + 16 <= n; p += 16) {
        unsigned i;

        memcpy(cipher, buf + p, 16);
        block_decrypt(buf + p, rk);
        for (i = 0; i < 16; i++)
            buf[p + i] ^= previous[i];
        memcpy(previous, cipher, 16);
    }
}

/* ====================================================================== *
 * The numbers themselves.
 * ====================================================================== */

/*
 * The key and the IV, taken at 0x2cfdf0 of the stock firmware — thirty-two
 * consecutive bytes that load_machine_info loads in two halves.
 */
static const uint8_t sn_key[16] = {
    0x58,0x9a,0x7d,0xc2,0x58,0xb3,0x2d,0x69,
    0xcf,0x2e,0xc6,0xa3,0xd6,0x28,0x3d,0x3c
};
static const uint8_t sn_iv[16] = {
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
};

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/*
 * Read a file, convert it from hexadecimal, decrypt it, and keep only the
 * printable part, as removeInvalidChars does: the plaintext is padded with
 * filler bytes that must be discarded.
 *
 * Returns 0 and leaves the destination empty if anything is missing: a
 * missing number must stay an empty string, as the stock API returns for
 * an empty .hs2.
 */
static int read_sn(const char *path, char *dest, size_t size)
{
    FILE *f;
    char hex[512];
    uint8_t raw[256];
    size_t n = 0, i, k;

    if (dest && size) dest[0] = 0;
    f = fopen(path, "r");
    if (!f)
        return 0;
    n = fread(hex, 1, sizeof(hex) - 1, f);
    fclose(f);
    hex[n] = 0;

    for (i = 0, k = 0; i + 1 < n && k < sizeof(raw); ) {
        int a = hexval((unsigned char)hex[i]);
        int b = hexval((unsigned char)hex[i + 1]);

        if (a < 0) { i++; continue; }          /* newline, space */
        if (b < 0) break;
        raw[k++] = (uint8_t)((a << 4) | b);
        i += 2;
    }
    if (k < 16 || (k % 16))
        return 0;

    cbc_decrypt(raw, k, sn_key, sn_iv);

    /*
     * CUT AT THE FIRST NON-ALPHANUMERIC CHARACTER, rather than filter:
     * removeInvalidChars (0x4297d) tests "__ctype_b_loc()[c] & 8" (glibc's
     * _ISalnum mask) and writes a zero as soon as it fails, so the string
     * stops there. Keeping all printable characters would let through a
     * filler byte that happens to be a letter.
     */
    for (i = 0, n = 0; i < k && n + 1 < size; i++) {
        uint8_t c = raw[i];
        int alnum = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z');

        if (!alnum)
            break;
        dest[n++] = (char)c;
    }
    dest[n] = 0;
    return n ? 1 : 0;
}

static struct {
    int  loaded;
    char machine[64];
    char ctrlboard[64];
    char pw[64];
    char hashboard[N_HASHBOARD][64];
} sn;

/*
 * THE LEADING LETTER: a machine number starts with Z, a control board with
 * K, a hash board with S. A failed decryption does not return an error but
 * bytes: without this check, a wrong key would produce a plausible string
 * that the API would show as a number. It is refused and reported instead.
 */
static void check_prefix(const char *what, char *val, char expected)
{
    if (!val[0] || val[0] == expected)
        return;
    jas_log("serial number %s: \"%s\" does not start with %c — suspicious"
            " decryption, ignored", what, val, expected);
    val[0] = 0;
}

static void load(void)
{
    unsigned k;

    if (sn.loaded)
        return;
    sn.loaded = 1;
    read_sn("/media/configs/.ms", sn.machine,   sizeof(sn.machine));
    read_sn("/media/configs/.cs", sn.ctrlboard, sizeof(sn.ctrlboard));
    read_sn("/media/configs/.pw", sn.pw,        sizeof(sn.pw));
    check_prefix("machine", sn.machine, 'Z');
    check_prefix("control board", sn.ctrlboard, 'K');
    for (k = 0; k < N_HASHBOARD; k++) {
        char path[40];

        snprintf(path, sizeof(path), "/media/configs/.hs%u", k + 1);
        read_sn(path, sn.hashboard[k], sizeof(sn.hashboard[k]));
        check_prefix("hash board", sn.hashboard[k], 'S');
    }
    jas_log("serial numbers: machine %s, control board %s, psu %s, hashboard %s",
            sn.machine[0]   ? sn.machine   : "(missing)",
            sn.ctrlboard[0] ? sn.ctrlboard : "(missing)",
            sn.pw[0]        ? sn.pw        : "(missing)",
            sn.hashboard[0][0] ? sn.hashboard[0] : "(missing)");
}

const char *jas_sn_machine(void)   { load(); return sn.machine; }
const char *jas_sn_ctrlboard(void) { load(); return sn.ctrlboard; }
const char *jas_sn_pw(void)        { load(); return sn.pw; }

const char *jas_sn_hashboard(unsigned k)
{
    load();
    return (k < N_HASHBOARD) ? sn.hashboard[k] : "";
}

/*
 * IDENTITY OF THE MACHINE IN THE NONCE SPACE.
 *
 * work.c and seed.c both compute:
 *
 *     extranonce = *jas_extranonce;                  (0 if never written)
 *     if (extranonce == 0) extranonce = *jas_dev_id;  (same)
 *     nonce = (extranonce << 48) + board*3*120 + chip*120;
 *
 * so without a seed every machine starts its ranges at 0 / 120 / 240, and
 * machines on a pool that pushes the same work to all the workers of a
 * wallet find the SAME nonces ("Duplicate share" refusals).
 *
 * THE SEED: the machine's serial number, the only truly unique identifier
 * available, folded with FNV-1a then brought into [1, 0xffff]. Two
 * constraints, both needed:
 *
 *   - NOT ZERO, otherwise the caller would believe the seed absent and fall
 *     back on the very fallback being fixed;
 *   - UNDER 16 BITS, so that `shift` stays at 48 and leaves the hardware the
 *     low 48 bits to search (at 40, eight would be taken away for nothing).
 *
 * Without a serial number, 0 is returned: better the known defect than a
 * seed all machines would share.
 */
uint64_t jas_identity_nonce(void)
{
    static uint64_t identity;
    static int done;
    const char *s;
    uint32_t h = 2166136261u;

    if (done)
        return identity;
    done = 1;

    load();
    s = sn.machine;
    if (!s || !*s) {
        jas_log("nonce: NO serial number — nonce range NOT separated, this"
                " machine may duplicate the others on a shared pool");
        return 0;
    }
    while (*s) {
        h ^= (uint32_t)(unsigned char)*s++;
        h *= 16777619u;
    }
    identity = (uint64_t)(h % 0xffffu) + 1u;
    jas_log("nonce: machine identity %llu (serial %s)",
            (unsigned long long)identity, sn.machine);
    return identity;
}
