/*
 * Shared state of the miner.
 *
 * These variables belong to the firmware as long as we have not taken over
 * the whole chain: in bridge mode they are its own, at their fixed addresses;
 * in standalone mode, ours. Declaring them as pointers rather than objects
 * lets the files of `core/` access them without ever naming an address — the
 * same rule as for the hardware.
 *
 * Names, types and dimensions come from the firmware's DWARF.
 */
#ifndef JAS_STATE_H
#define JAS_STATE_H

#include <stdint.h>

extern uint8_t  *jas_asic_num;              /* ASIC_NUM_G              0x387dd2 */
extern uint8_t  *jas_asic_en;               /* asic_en_g[3]            0x3e1f08 */
extern uint16_t *jas_board_en;              /* board_en_g              0x387cea */
extern uint16_t  jas_pll_lock[3];          /* pll_lock_g              0x3f92c0 */
extern uint16_t  jas_pll_stable[3];        /* pll_stable_g            0x3f92b8 */
extern uint16_t  jas_pll_clk_test[3][3];   /* pll_clk_test_g          0x3d000c */
extern uint16_t  jas_pmbus_mode;           /* pmbus_mode              0x387cb8 */
extern uint32_t  jas_asic_temp[3][3];      /* temperatures, initialized to 32 */
extern uint16_t  jas_sensor_board_en;      /* sensor_board_en_g       0x387c80 */
extern uint8_t  *jas_board_num;             /* BOARD_NUM_G             0x387dd3 */
extern uint16_t (*jas_spi_phase)[3][4];     /* var_spi_miso_phase      0x38c1c0 */
extern uint16_t (*jas_spi_status)[3];       /* spi_check_status_ori_g  0x494b38 */

/* State of the memory banks, 4080 per ASIC, 12240 per board.
 * The suffix tells the current state (_g) from the reference coming from the
 * memory test (_c), to which modify_remap_data reverts. */
extern uint16_t (*jas_bank_good)[12240];        /* bank_good_record_flag_g    0x3d0028 */
extern uint16_t (*jas_bank_partial)[12240];     /* bank_partial_record_flag_g 0x494b58 */
extern uint16_t (*jas_bank_bad)[12240];         /* bank_bad_record_flag_g     0x3f92c8 */
extern uint16_t (*jas_bank_partial_ref)[12240]; /* bank_partial_record_flag_c 0x482c58 */
extern uint16_t (*jas_bank_good_ref)[12240];    /* bank_good_record_flag_d    0x3e4100 */
extern uint16_t jas_map_cbsc_killed[3][12240]; /* carto diag: DFT-good bank killed by CBSC */
extern uint16_t jas_map_bank_alu[3][12240];    /* carto diag: nb of ALUs reaching the bank */

extern uint16_t (*jas_alu_result)[3][24];       /* alu_check_result_ori_g 0x482a98 */

/* Fields of fpga_data (0x52e3e8) still shared, and ALU mask per mesh.
 * fpga_data: +0 block_number, +8 cache_size, +16 dag_size, +24 extranonce,
 *            +32 target, +64 header, +96 seed, +128 alu_close_flag */
/* Good banks per chip after the crossbar scan, and the share attributable
 * to each mesh. DWARF dimensions: [3][3] and [3][3][4]. */
extern uint16_t *jas_good_bank_num_cbsc;        /* 0x38c118, 9 entries */
extern int64_t  *jas_asic_max_capacity;         /* 0x40b1c8, 9 entries */
extern float   (*jas_eff_real_value)[3][120];   /* 0x3e3020, 1080 flottants */
extern uint8_t  *jas_screening_status;          /* device_screening_result 0x3f71e0 */

/* Fields of dev_info_g that seed_update_operation handles. Named rather
 * than designated by their offset — the offset is in the comment. */
extern int      *jas_seed_switch;               /* dev_info_g + 0x3410 */
extern uint8_t  *jas_board_asic_num;            /* dev_info_g + 0x72, 3 entries */
extern uint16_t *jas_good_bank_num_cbsc_mesh;   /* 0x38c0d0, 36 entries */

extern uint64_t *jas_fpga_block_number;         /* fpga_data +0     */
extern uint64_t *jas_fpga_dag_size;             /* fpga_data +16    */
extern uint64_t *jas_extranonce;                /* fpga_data +24    */
extern uint8_t  *jas_fpga_target;               /* fpga_data +32    */
extern uint8_t  *jas_fpga_header;               /* fpga_data +64    */
extern uint8_t  *jas_fpga_seed;                 /* fpga_data +96    */
extern uint64_t *jas_dev_id;                    /* dev_info_g       */
extern uint8_t  *jas_alu_close_flag;            /* fpga_data +128   */
extern uint16_t (*jas_alu_en)[3][4][2];         /* alu_en_g 0x52f6d8 */

/* Raw result of the crossbar scan: [board][chip][alu][word of 16 banks].
 * 255 words cover the 4080 banks. */
extern uint16_t (*jas_cb_sc_result)[3][120][255];   /* cb_sc_result_ori_g 0x4a6a38 */

/* Memory enable masks, per quarter of 1020 banks (64 words each). */
extern uint16_t (*jas_mc_en)[3][4][64];             /* mc_en_g            0x52d1e8 */
extern uint16_t (*jas_mc_refresh_en)[3][4][64];     /* mc_refresh_en_g    0x3f5fe0 */
extern uint16_t (*jas_mc_monitor_en)[3][4][64];     /* mc_rf_moniter_en_g 0x52e4d8 */

/* A timing setting: the register, the addressing mode, the value, and the
 * number of words (1 for a single one, more for an array). */
typedef struct {
    uint16_t    reg;
    uint8_t     cmd;
    uint8_t     count;
    const void *value;
} jas_fix_param;

/* The table, terminated by an entry with a null reg. */
const jas_fix_param *jas_fix_params(void);

extern uint16_t (*jas_alu_flag)[360];        /* alu_check_result_record_flag_g */
extern uint16_t (*jas_good_alu_num)[3];      /* good_alu_sc_num     0x38c0a8 */
/* CAREFUL: good_alu_num and good_alu_sc_num are TWO distinct symbols,
 * twelve bytes apart. The sorting summary publishes the first one. */
extern uint16_t (*jas_good_alu_num_dft)[3];  /* good_alu_num        0x38c0bc */

#define ASIC_NUM   (*jas_asic_num)
#define ASIC_EN(b) (jas_asic_en[(b)])
#define BOARD_NUM  (*jas_board_num)
#define BOARD_EN   (*jas_board_en)


/* Serial number burnt into a chip: char asic_id[3][3][64] in dev_info_g,
 * offset 13504 — checked through the structure size (13504 + 3*3*64 + 24
 * bytes of mutex = 14104, the size announced by DWARF). */
#define JAS_ASIC_ID_LEN 64
char *jas_asic_id(int board, unsigned asic);

void efuse_read(uint8_t asic_en, uint16_t *data, int thread_id);
void sl_get_asic_id(void);


/* DFT test counts, per board and per chip. */
extern uint16_t (*jas_good_bank_num_pre)[3];   /* good_bank_num_pre   0x38c140 */
extern uint16_t (*jas_good_bank_num_post)[3];  /* good_bank_num_post  0x38c12c */
extern uint16_t (*jas_dp_fail_num)[3];         /* dp_fail_num         0x38c080 */
extern uint16_t (*jas_hb_fail_num)[3];         /* hb_fail_num         0x38c094 */
extern uint16_t *jas_mc_trim;                  /* var_mc_trim         0x387d48 */
extern uint8_t  *jas_dft_pretest_flag;         /* dft_pretest_flag_g  0x38c1a0 */


/* dev_info_g.asic_type, offset 2 — a type 3 forces a fresh DFT survey. */
int jas_asic_type(void);

/*
 * LOCAL reject rate of a chip (nonces with a wrong mix discarded BEFORE
 * submission, by ethash_nonce_check) — exposed for the periodic display in
 * surveillance.c, right after the chip's hashrate. It is the real silicon
 * health, distinct from the pool reject (always ~0). Returns the rate in %,
 * and sets *rej / *total (nonces judged) if the pointers are non-null.
 * Cumulative since the miner started. Defined in core/wrapper.c.
 */
double jas_reject_local_chip(unsigned board, unsigned asic,
                             unsigned *rej, unsigned *total);

#endif
