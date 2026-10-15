/*
 * Firmware types, taken from its DWARF — not reconstructions.
 * Using these definitions rather than numeric offsets makes the
 * reimplementation checkable and avoids alignment mistakes.
 */
#ifndef JAS_TYPES_H
#define JAS_TYPES_H

#include <stdint.h>

/* Command word of the serial link, as dropped into the AXI window. Built
 * byte by byte by the original, hence insensitive to endianness. */
typedef struct transfer_data {
    uint8_t asic_en;
    uint8_t cmd;
    uint8_t addr0;
    uint8_t addr1;
    uint8_t data0;
    uint8_t data1;
} transfer_data_t;

typedef struct { uint8_t b[32]; } ethash_h256_t;

/* A result reported by a chip: the nonce found and its mix digest, the one
 * the pool requires to validate a share without rebuilding the DAG. This is
 * what get_asic_result_operation_calc hands back to the caller. */
typedef struct gooddata {
    uint64_t      nonce;
    ethash_h256_t mixhash;
} gooddata;

typedef struct gooddata_detail {
    uint8_t  board_id;
    uint8_t  asic_id;
    uint8_t  alu_id;
    gooddata good_data;     /* at +8: the three identifiers are followed by
                             * alignment padding */
} gooddata_detail;

#endif
