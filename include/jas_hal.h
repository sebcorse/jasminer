/*
 * Hardware abstraction layer.
 *
 * Everything the reimplemented code does to the hardware goes through here,
 * and through nothing else. This rule is what lets the same files of `core/`
 * compile into two targets:
 *
 *   BRIDGE MODE      the library is injected into jasminer.elf through
 *                    LD_PRELOAD, and the HAL calls the firmware's functions
 *                    at their fixed address. Everything not yet
 *                    reimplemented keeps working.
 *
 *   STANDALONE MODE  our own binary, the HAL talks directly to the
 *                    programmable logic through /dev/mem.
 *
 * A function of `core/` containing a hard-coded address or a direct hardware
 * access would break this property: what is validated in bridge mode would no
 * longer be what ships in standalone mode.
 *
 * The signatures follow exactly those of the DWARF of jasminer.runtime.elf,
 * parameter names included, so that the reimplementation stays a reading and
 * not an interpretation.
 */

#ifndef JAS_HAL_H
#define JAS_HAL_H

#include <stdint.h>
#include <time.h>

/* --- serial link to the ASICs ---------------------------------------------
 * asic_en is a chip mask: 1 = ASIC0, 2 = ASIC1, 4 = ASIC2, 7 = all three.
 * cmd encodes the addressing mode: 0xf8 for broadcast, 0x18/0x28/0x38/0x48
 * to target one group of ALUs (a "mesh") in particular. */
void jas_write   (uint8_t asic_en, uint8_t cmd, uint16_t addr,
                  uint16_t *data, uint32_t data_len, int thread_id);
void jas_write1  (uint8_t asic_en, uint8_t cmd, uint16_t addr,
                  uint16_t data, int thread_id);
int  jas_read1   (uint8_t asic_en, uint8_t cmd, uint16_t addr,
                  uint16_t *data, uint64_t timeout, int thread_id);
int  jas_readn   (uint8_t asic_en, uint8_t cmd, uint16_t addr,
                  uint16_t *data, uint32_t data_len, uint64_t timeout,
                  int thread_id);

/* --- time and log --------------------------------------------------------- */
void jas_delay   (time_t sec, long ns);
void jas_log     (const char *fmt, ...);
void jas_trace   (const char *fmt, ...);  /* like jas_log, silent unless JASRE_TRACE */
void jas_dump_map(void);          /* banks/mesh map, gated by JASRE_MAP_DUMP */
void jas_hashrate_rt_sample(void);         /* sample of the accepted nonce stream */
void jas_hashrate_rt_get(double *rt, double *avg); /* real MH/s (accepted nonces) */
void jas_hashrate_rt_reset(void);          /* empty the rt window (active pool change) */

/* The configuration file: "-c" on the command line changes it. */
void        jas_config_set(const char *path);
const char *jas_config_file(void);

/* --- AXI register window --------------------------------------------------
 * Returns the mapped base for a board, or NULL if it is not available. In
 * bridge mode it is the one the firmware already set up (g_mapBase,
 * 0x494b4c); in standalone mode, ours through /dev/mem. This single function
 * is the only difference between the two modes: the protocol itself, in
 * core/sl_transfer.c, is identical. */
volatile uint32_t *jas_axi_base(int thread_id);

/* Reset line of the serial link (g_mapCLR, 0x40b1a8 in the firmware).
 * Used when the link reports an error: pull low, wait, release. */
void jas_link_reset(int thread_id);

/* --- HAL initialization ---------------------------------------------------
 * Bridge mode: checks the firmware signature before allowing any call.
 * Standalone mode: opens /dev/mem and maps the AXI windows.
 * Returns 0 if the hardware is usable. */
int  jas_hal_init(void);
const char *jas_hal_mode(void);


/*
 * Serial bus lock.
 *
 * The firmware serializes its accesses in sl_wrapper_work, around
 * dev_info_g.sl_dev_mutex, not in sl_single_write. Everything that talks to
 * the link in parallel with mining must therefore take THAT lock, otherwise
 * two transactions interleave on the same AXI window and corrupt each other.
 *
 * In bridge mode these functions lock the firmware's mutex; in standalone
 * mode, a recursive mutex of our own (see hal/hal_devmem.c).
 */
void jas_bus_lock(void);
void jas_bus_unlock(void);


/* Hard recovery of the link (sl_w_fpga_hard_rstn): two CLR+RSTN cycles. */
void jas_link_hard_reset(unsigned delay_s, int thread_id);   /* sl_w_fpga_hard_rstn   */
void jas_link_hard_reset_all(void);                         /* sl_w_fpga_hard_rstn_0 */
void jas_link_hard_reset_fast(int thread_id);               /* sl_w_fpga_hard_rstn_1 */

/* IIC windows (sensors + PMBus) and fans — axi_*_addr_map_init. */
extern void *jas_iic_p[3];
extern void *jas_fan_p[2];
int  jas_iic_map_init(int thread_id);
int  jas_fan_map_init(void);
extern void *jas_zynq_p;
int  jas_zynq_map_init(void);

/* IIC bus and PMBus (core/iic.c). Read-only on the voltage side for now. */
void iic_initialization_op(int thread_id);
int  iic_multi_byte_read(uint8_t device_id, uint8_t address,
                         uint8_t *data, int len, int thread_id);
int  iic_single_byte_write(uint8_t device_id, uint8_t address, uint8_t data, int thread_id);
int  iic_single_byte_read(uint8_t device_id, uint8_t address, uint8_t *data, int thread_id);
int  iic_multi_byte_write(uint8_t device_id, uint8_t address, const uint8_t *data,
                          int len, int thread_id);
int  mcp2973_set_vol(uint8_t slave_addr, uint8_t rail, int mv, int thread_id);
void asic_pmbus_config(uint16_t mode, int thread_id, unsigned asic,
                       uint16_t rail0, uint16_t rail1, uint16_t rail2);
int  pmbus_set_current(int thread_id, uint8_t slave_addr, uint8_t rail,
                      uint16_t vol_min, uint16_t vol_max, uint16_t limit_val);
int  iic_mux_select(uint8_t device_id, const uint8_t *data, int thread_id);
int  mcp2973_get_vol(uint8_t slave_addr, uint8_t rail, uint16_t *output, int thread_id);
int  RAA228236_get_vol(uint8_t slave_addr, uint8_t rail, uint16_t *output, int thread_id);
int  RAA228236_set_vol(uint8_t slave_addr, uint8_t rail, int value, int thread_id);
void set_asic_rail_vol(int thread_id, uint8_t asic_num, uint8_t rail_num,
                       uint8_t iic_slave_addr, uint16_t vol);
void asic_pmbus_read_vol(uint16_t mode, int thread_id, uint16_t *read_vol);
int  jas_pmbus_read_vout(int thread_id, unsigned asic, uint16_t *raw);
const uint8_t *jas_pmbus_addresses(void);

/* Hardware startup, in the firmware's order. */
void board_check(void);
void sensor_init(void);
void sensor_thread_create(void);
void get_temp_func(uint16_t thread_id);
void jas_sensors_configure(unsigned board);
void jas_sensors_read(unsigned board);
void get_temp_sensor_value(int *temp_value);
void fan_speed_set(int rate);
void fan_speed_read(int fan_speed[4]);

/* Fan regulation (core/ventilation.c). To be called at least once a
 * second: the FPGA goes back to 100 % without a refresh. */
int  fan_pwm_set(int pwm);
int  jas_pwm_current(void);
void set_PWM_according_to_temperature_x16(void);
int  temperature_filter(const int *temp, int board_id, int *max_temp,
                        int *min_temp, char *max_temp_asic_num);
extern uint16_t jas_tmp_sensor_en;
void power_on_operation(int thread_id);
void power_on_only(int thread_id);            /* turns back on WITHOUT computing asic_en */
void power_restart_operation(int thread_id);  /* back on + PLL, 0x11 not reread */
void power_off_operation(void);               /* off, all boards */
void find_borad_and_poweron(void);
void sl_wrapper_init(void);      /* the very first hardware code */
void *sl_wrapper_work(void *arg);  /* the mining loop */

/* Monitoring and thread startup (core/surveillance.c). */
void creat_pthread(void);
extern int jas_temp_max, jas_temp_min;
/* Fields of dev_info_g (+0x3458/+0x345c/+0x3468/+0x346c). Nobody raises
 * them yet: temperature_filter and pthread_device_status_monitor are missing. */
extern int *jas_alarm_temp_missing, *jas_alarm_temp_high;
extern int *jas_alarm_fan, *jas_alarm_ctrl_c;
extern int  jas_alarm_local[4];
void seed_init_bank_params(void);   /* fragment of seed_update_operation */

/* DAG construction progress bar (core/dag_calc.c). */
void dag_progress_bar(uint8_t slice);
int  dram_check_dag(int thread_id, uint8_t asic, uint8_t mesh_bad);
void dag_calc(void);
unsigned dag_calc_failures(void);   /* chips without a DAG, after dag_calc */
/* Rebuild PER CHIP, without the board preamble: the epoch change.
 * mask[board] = bits of the chips to rebuild. Returns the number of failures. */
unsigned dag_calc_mask(const uint8_t mask[3]);
/* Requests a DAG rebuild of the given chips; the dedicated thread takes
 * care of it. Called by the epoch handler of wrapper.c. */
void jas_dag_request_rebuild(int board, uint8_t mask);
/* Starts the thread running these rebuilds. To be called once at startup. */
void jas_dag_rebuild_start(void);
/* Per-chip startup: chips whose DAG end will be written after mining starts
 * (seed.c spares them), and the start signal. See core/dag_calc.c. */
uint8_t jas_dag_deferred(int board);
/* Chips whose DAG exceeds the memory beyond the reject threshold (70 %). */
uint8_t jas_dag_discarded(int board);
/* Switch marker file: /tmp/<name> (trial) or /media/configs/<name> (lasting). */
int     jas_marker(const char *name);
void    jas_dag_start_done(void);
void get_eff(int thread_id, int delay);  /* core/status.c */
void get_current_status(void);           /* core/status.c */
int  jas_asic_vol_rail(unsigned board, unsigned chip, unsigned rail);
void seed_update_operation(void);
extern uint8_t  jas_dag_percent;
extern uint16_t jas_dag_counter;
extern uint16_t jas_dag_done;
void jas_fan_mining(int active);   /* ventilation.c: 1 = ALUs mining */
void jas_standby_enter(void);              /* seed.c: ALUs off, DAG kept at rest */
void jas_standby_exit(void);              /* seed.c: DAG check, mining resumes */
extern volatile int jas_standby_requested;   /* wrapper.c: 1 = standby asked (API) */
extern volatile int jas_in_standby;         /* wrapper.c: state applied by the loop */
extern uint8_t  jas_dag_step;

/* Log in the firmware's format, in /tmp, served by log.html through
 * /var/log/run.log, the path the CGIs read hard-coded. Never in flash. */
void jas_applog(const char *line);
/* 1 if JASRE_TRACE is set: fine trace of pool traffic and nonces. */
int  jas_trace_active(void);

void jas_name_thread(const char *name);
/* Also declared in jas_coin.h: both layers call it. */
void jas_web_dag_publish(unsigned percent);
/* Refuses to go on if the real miner is running: two processes on the
 * serial bus freeze the machine. Returns -1 and explains. */
int  jas_require_session(void);

#endif

void jas_mining_counters(unsigned *seen, unsigned *submitted, unsigned *rej,
                          unsigned *stale);

/* asic_mask: one bit per chip. Only the chips in the mask are flushed. */
void purge_results(int thread_id, uint8_t asic_mask);

void jas_chips_ready(void);

unsigned jas_mining_rounds(void);
