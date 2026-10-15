/*
 * Serial link to the ASICs — public interface of core/sl_transfer.c.
 * Signatures identical to the stock firmware's (DWARF), so the replacement is
 * a drop-in.
 */
#ifndef SL_TRANSFER_H
#define SL_TRANSFER_H

#include <stdint.h>
#include <time.h>

void sl_delay(time_t sec, long ns);                                    /* :639 */
void sl_single_write(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                     uint16_t data, int thread_id);                    /* :469 */
void sl_multi_write(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                    uint16_t *data, uint32_t data_len, int thread_id); /* :496 */
int  sl_single_read(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                    uint16_t *data, uint64_t timeout, int thread_id);  /* :508 */
/* Probe of hardware that may not answer: honours the delay (µs), does not
 * test the error bit, and above all does NOT RESET the link. */
int  sl_single_read_soft(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                         uint16_t *data, uint64_t timeout_us, int thread_id);
int  sl_multi_read(uint8_t asic_en, uint8_t cmd, uint16_t addr,
                   uint16_t *data, uint32_t data_len, uint64_t timeout,
                   int thread_id);
uint16_t sl_get_pll_from_freq(uint16_t freq);
void spi_check(int thread_id);                                         /* :1362 */
void init_alu_reg(int thread_id);
void alu_check(int thread_id);
int  config_remap_info(int thread_id, uint8_t asic, uint8_t remap_flag);/* :3082 */
void modify_remap_data(int thread_id, uint16_t asic);                  /* :3218 */
void dag_remap_data(int thread_id, uint16_t asic, uint8_t mesh_flag,
                    uint8_t bank_flag, uint8_t mesh_bank);             /* :3241 */


/*
 * asic_mask: one bit per chip. Only the chips in the mask are reprogrammed —
 * the original broadcast to the whole board, which in dual-coin mode made a
 * chip suffer the jobs of the neighbouring coin.
 */
void header_update_operation(int thread_id, uint8_t asic_mask);  /* :4873 */

/* Forget what work.c believes it knows about a chip's registers (after a
 * reset, an epoch change, or any reprogramming coming from elsewhere). */
void jas_work_memo_forget(int thread_id, uint8_t asic_mask);

/* Building the cache and the DAG */
struct jas_coin_ctx;
uint64_t dag_cache_init_addr(unsigned valid_banks, uint64_t cache_size);
uint64_t dag_asic_capacity(unsigned bad_banks);
void     dag_soft_reset(int board, unsigned asic);
int      dag_build_cache(int board, unsigned asic, const struct jas_coin_ctx *coin);
int      dag_build_dataset(int board, unsigned asic, const struct jas_coin_ctx *coin);
int      dag_calc_asic(int board, unsigned asic, const struct jas_coin_ctx *coin);

void config_asic_fix_param(int thread_id);
void alu_check_analysis(int thread_id);
void crossbar_check(int thread_id);
void crossbar_check_analysis(int thread_id);
void single_manual_write(uint8_t asic_en, uint32_t w_addr, uint16_t *w_data, int thread_id);
void single_manual_write_param(uint8_t asic_en, uint32_t w_addr, uint16_t *w_data,
                               int thread_id, unsigned delay_us, int probe_before);

/* Checking the cache in memory */
int dram_read_last_cache_node(uint8_t asic_en, uint16_t manual_sel,
                              uint32_t base_addr, uint64_t cache_size,
                              int thread_id, uint16_t node[32]);
int dram_check_cache(uint8_t asic_en, uint16_t manual_sel, uint32_t base_addr,
                     int thread_id, const struct jas_coin_ctx *coin,
                     const uint16_t *cpu_cache);

/* Collecting the results */
struct gooddata_detail;
int  get_asic_result_operation_calc(int thread_id, int asic, int mesh,
                                    struct gooddata_detail *detail);
int  get_asic_result_operation(int thread_id, struct gooddata_detail *detail);

int  single_manual_read(uint8_t asic_en, uint16_t manual_sel, uint32_t manual_addr,
                        uint16_t *manual_dat, int thread_id);
void dram_dump(uint8_t asic_en, uint16_t manual_sel, uint32_t base_addr,
               uint32_t jump_size, const char *file_name, int thread_id);
#endif
