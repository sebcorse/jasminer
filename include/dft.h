/*
 * Built-in test block (DFT) — interface.
 * Signatures taken from the DWARF.
 */
#ifndef DFT_H
#define DFT_H
#include <stdint.h>

/* primitives */
void dft_reset(int thread_id, uint16_t i, uint16_t mesh);
void dft_set_id(int thread_id, uint16_t i, uint16_t mesh,
                uint16_t id_set, uint16_t id_mask_set);
void dft_set_pre_post_mode(int thread_id, uint16_t i, uint16_t mesh, uint16_t mode);
void dft_write_single_cmd(int thread_id, uint16_t i, uint16_t mesh,
                          uint16_t *single_cmd_din);
void dft_read_single_cmd(int thread_id, uint16_t i, uint16_t mesh,
                         uint16_t *ltc_single_read_data);
void dft_write_otp(int thread_id, uint16_t i, uint16_t mesh,
                   uint16_t *ap_repair_info);
void dft_conv_otp_row_data(uint16_t *ltc_row_data, uint32_t *ltc_otp_row_data);



/* test sequences */
void dft_apply_trim(int thread_id, uint16_t i, uint16_t mesh, const uint16_t *lr_trim);
int  dft_MRR_read(int thread_id, uint16_t i, uint16_t mesh,
                  uint16_t *lr_trim, uint16_t *ltc_r_mr_config);
int  dft_cmd_multi_write(int thread_id, uint16_t i, uint16_t mesh,
                         uint16_t *lr_trim, uint16_t *cmd_din, uint32_t timeout);

/* readouts */
int dft_get_row_repair_info(int thread_id, uint16_t i, uint16_t mesh,
                            uint16_t *ltc_row_data);
int dft_get_full_repair_info(int thread_id, uint16_t i, uint16_t mesh,
                             uint16_t *ltc_full_data);
/* Diagnostic: last column count read by dft_get_result_way (0 if no read
 * took place). Used to tell a real defect from a saturated counter. */
uint32_t dft_last_col_count(void);

int dft_get_result_way(int thread_id, uint16_t i, uint16_t mesh,
                       uint32_t *ltc_row_count);
int dft_read_HB_info(int thread_id, uint16_t i, uint16_t mesh,
                     uint16_t *lr_trim, uint16_t *single_cmd);
int dft_read_DP_info(int thread_id, uint16_t i, uint16_t mesh,
                     uint16_t *lr_trim, uint16_t *single_cmd1,
                     uint16_t *single_cmd2);

void dft_set_bist_l(int thread_id, uint16_t i, uint16_t mesh, uint16_t bist_l);
void dft_set_otp_mode(int thread_id, uint16_t i, uint16_t mesh, uint16_t mode);
void self_check_func(int thread_id);
int dft_cmd_s2m_write(int thread_id, uint16_t i, uint16_t mesh,
                      uint16_t *lr_trim, uint16_t *cmd_din, uint32_t timeout);
void close_mesh_bank(uint16_t mesh, uint16_t asic, int thread_id);
int  dft_short_pattern_test_110(int thread_id);
#endif
int dft_cmd_s2m_write(int thread_id, uint16_t i, uint16_t mesh,
                      uint16_t *lr_trim, uint16_t *cmd_din, uint32_t timeout);
