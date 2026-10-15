/*
 * Keccak test vectors, to pass BEFORE writing anything that depends on it:
 * a DAG built on a wrong Keccak only shows as refused shares, hours later.
 *
 *   gcc -O2 -o /tmp/tk tools/test_keccak.c core/keccak.c && /tmp/tk
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int sha3_256(uint8_t *, size_t, const uint8_t *, size_t);
int sha3_512(uint8_t *, size_t, const uint8_t *, size_t);

static int checked(const char *what, const uint8_t *obtained, size_t n,
                   const char *expected)
{
    char hex[257];
    size_t i;
    for (i = 0; i < n; i++)
        sprintf(hex + 2 * i, "%02x", obtained[i]);
    hex[2 * n] = 0;
    if (strcmp(hex, expected) == 0) {
        printf("  ok    %s\n", what);
        return 1;
    }
    printf("  WRONG %s\n        got      %s\n        expected %s\n", what, hex, expected);
    return 0;
}

int main(void)
{
    uint8_t o[64];
    int good = 0, total = 0;

    /* Keccak-256 and Keccak-512 of the empty string, and of "abc". These are
     * ORIGINAL Keccak vectors, not SHA-3's: with 0x06 padding instead of
     * 0x01, none would pass. */
    total++; sha3_256(o, 32, (const uint8_t *)"", 0);
    good += checked("Keccak-256(\"\")", o, 32,
        "c5d2460186f7233c927e7db2dcc703c0e500b653ca82273b7bfad8045d85a470");

    total++; sha3_256(o, 32, (const uint8_t *)"abc", 3);
    good += checked("Keccak-256(\"abc\")", o, 32,
        "4e03657aea45a94fc7d47ba826c8d667c0d1e6e33a64a036ec44f58fa12d6c45");

    total++; sha3_512(o, 64, (const uint8_t *)"", 0);
    good += checked("Keccak-512(\"\")", o, 64,
        "0eab42de4c3ceb9235fc91acffe746b29c29a8c366b7c60e4e67c466f36a4304"
        "c00fa9caf9d87976ba469bcbe06713b435f091ef2769fb160cdab33d3670680e");

    total++; sha3_512(o, 64, (const uint8_t *)"abc", 3);
    good += checked("Keccak-512(\"abc\")", o, 64,
        "18587dc2ea106b9a1563e32b3312421ca164c7f1f07bc922a9c83d77cea3a1e5"
        "d0c69910739025372dc14ac9642629379540c17e2a65b19d77aa511a9d00bb96");

    /*
     * Beyond one rate block. These values come from Cryptodome, an independent
     * implementation, never from our own code: a made-up digest gives a test
     * that always fails or, worse, passes for the wrong reasons.
     *
     * The 136- and 72-byte cases are the trickiest: the input fills exactly
     * one block, so the padding takes a whole extra block.
     */
    {
        uint8_t m[500];

        memset(m, 'a', 200);
        total++; sha3_256(o, 32, m, 200);
        good += checked("Keccak-256(200 x 'a')", o, 32,
            "96ea54061def936c4be90b518992fdc6f12f535068a256229aca54267b4d084d");

        memset(m, 'b', 136);
        total++; sha3_256(o, 32, m, 136);
        good += checked("Keccak-256(136 x 'b') — exactly one rate block", o, 32,
            "121b76d0b19f3c2c7632310b92c54cddd59d16a6b5aafe84696426f10e5733bf");

        memset(m, 'c', 137);
        total++; sha3_256(o, 32, m, 137);
        good += checked("Keccak-256(137 x 'c')", o, 32,
            "75fa65a2b7c62a0b55ccf8961895f9019fa2c054a293bb2a7f2bfba9d750e9ab");

        memset(m, 'd', 72);
        total++; sha3_512(o, 64, m, 72);
        good += checked("Keccak-512(72 x 'd') — exactly one rate block", o, 64,
            "4a63dbbbb5a8446fa61447c69d32a2dcd9bf30ad8ab6c921898aec66e8639206"
            "d8b7048b9077d84666d788e169c9159fc02702ad07cb3161de4cf71c151ccf58");

        memset(m, 'e', 500);
        total++; sha3_512(o, 64, m, 500);
        good += checked("Keccak-512(500 x 'e')", o, 64,
            "e9290890ee17ad56d267a682ab368f4beb049f17138b7bf7315e35682d17ec8f"
            "698a2abfc93949ab58f881946e5d2c63b95f356e642fbc0b9a4535bdc939e24a");
    }

    printf("\n  %d/%d\n", good, total);
    return good == total ? 0 : 1;
}
