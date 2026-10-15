/*
 * HAL in STANDALONE MODE: direct access to the programmable logic.
 *
 * The miner depends on no proprietary kernel driver: the firmware maps the
 * AXI windows itself from /dev/mem. Physical addresses read in
 * /proc/<pid>/maps of the running firmware:
 *
 *   0x42000000  1 MB   + 0x41200000 and 0x41300000 (4 KB each)
 *   0x43000000  1 MB   + 0x41210000 and 0x41310000
 *   0x44000000  1 MB   + 0x41220000 and 0x41320000
 *
 * Which board each line is does not follow the address order: see below.
 * The serial register window is at +0x20000 in each.
 */

#include <pthread.h>
#include <stdlib.h>
#include "jas_hal.h"
#include <dirent.h>
#include <limits.h>
#include <fcntl.h>
#include <stdarg.h>
#include <sys/prctl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include "jas_state.h"
#include "jas_ethash.h"

static const unsigned long board_phys[3] = { 0x42000000ul, 0x44000000ul, 0x43000000ul };

/*
 * Three windows per board, and the board order is NOT the address order.
 * The firmware's base_wr, base_rstn and base_clr tables (0x387d28,
 * 0x387d10, 0x387cf8) give the exact pairing:
 *
 *     board 0   0x42000000   rstn 0x41200000   clr 0x41300000
 *     board 1   0x44000000   rstn 0x41220000   clr 0x41320000
 *     board 2   0x43000000   rstn 0x41210000   clr 0x41310000
 *
 * CLR is the link recovery window: a low then high pulse puts the serial
 * link back in order after an error.
 */
static const unsigned long board_rstn[3] = { 0x41200000ul, 0x41220000ul, 0x41210000ul };
static const unsigned long board_clr[3]  = { 0x41300000ul, 0x41320000ul, 0x41310000ul };
#define BOARD_SPAN 0x100000ul

static volatile uint32_t *board_base[3];
static void *board_rstn_p[3], *board_clr_p[3];
static int devmem = -1;

static uint8_t  s_asic_num = 3;

/*
 * Which chips are active: read where the machine keeps it,
 * /media/configs/asic_enabled_mask.conf (7 for three chips, 5 when number
 * 1 has been set aside). Never hard-code 7: on a machine with a chip set
 * aside, spi_check would query the absent chip, each failed read would
 * reset the link, and the check would never advance.
 */
static uint8_t  s_asic_en[3]              = { 7, 7, 7 };

/* Reads the active-chip mask back, imposing nothing if the file is missing. */
static void load_asic_mask(void)
{
    const char *env = getenv("JASRE_ASIC_EN");
    FILE *f;
    unsigned v;

    if (env && *env) {
        s_asic_en[0] = (uint8_t)strtoul(env, NULL, 0);
        return;
    }
    f = fopen("/media/configs/asic_enabled_mask.conf", "r");
    if (!f)
        return;
    if (fscanf(f, "%u", &v) == 1 && v)
        s_asic_en[0] = (uint8_t)v;
    fclose(f);
}

int jas_hal_init(void)
{
    load_asic_mask();
    devmem = open("/dev/mem", O_RDWR | O_SYNC);
    if (devmem < 0) { perror("jasre: /dev/mem"); return -1; }

    for (int i = 0; i < 3; i++) {
        if (!board_phys[i]) continue;
        void *p = mmap(NULL, BOARD_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED,
                       devmem, (off_t)board_phys[i]);
        if (p == MAP_FAILED) { perror("jasre: mmap"); return -1; }
        board_base[i] = (volatile uint32_t *)p;

        /* The two side windows, as in sl_transfer_init: RSTN (PL reset) and
         * CLR (link recovery), see above. */
        board_rstn_p[i] = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED,
                               devmem, (off_t)board_rstn[i]);
        board_clr_p[i]  = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED,
                               devmem, (off_t)board_clr[i]);
        if (board_rstn_p[i] == MAP_FAILED) board_rstn_p[i] = NULL;
        if (board_clr_p[i]  == MAP_FAILED) board_clr_p[i]  = NULL;

        /* Same wording as the firmware: its log in flash is our control, and a
         * line-by-line comparison assumes the same words. */
        printf("board %d mmap base address ok.\n", i);
    }
    fflush(stdout);
    return 0;
}

/*
 * REFUSE TO TOUCH THE HARDWARE IF THE MINER IS RUNNING: two processes on
 * the same serial bus freeze the machine.
 */

/*
 * A SINGLE INSTANCE OF OURSELVES. A second instance of our own binary
 * attacks the AXI windows while the first holds the PL in reset: the ARM
 * core stalls on an unacknowledged transaction and the mains power must
 * be cut. Our bus lock, a mutex, does not cross process boundaries, so the
 * check is here. (A duplicate typically comes from a launch loop killed on
 * the workstation: neither the ssh session nor the remote miner dies.)
 *
 * The /proc/<pid>/exe links are compared, which designate the real
 * binary, never a command line, which would contain our own search
 * pattern.
 */
static int jas_other_instance(void)
{
    char own[PATH_MAX], other[PATH_MAX];
    ssize_t n;
    DIR *d;
    struct dirent *e;
    pid_t self = getpid();
    int found = 0;

    n = readlink("/proc/self/exe", own, sizeof(own) - 1);
    if (n <= 0)
        return 0;
    own[n] = 0;

    d = opendir("/proc");
    if (!d)
        return 0;
    while ((e = readdir(d)) != NULL) {
        char link[64];
        long pid = atol(e->d_name);

        if (pid <= 0 || (pid_t)pid == self)
            continue;
        snprintf(link, sizeof(link), "/proc/%ld/exe", pid);
        n = readlink(link, other, sizeof(other) - 1);
        if (n <= 0)
            continue;
        other[n] = 0;
        if (strcmp(other, own) == 0) {
            fprintf(stderr,
                    "\n  STOP: another instance is already running (pid %ld).\n"
                    "  Two processes on the same bus freeze the machine —\n"
                    "  only a mains power cut recovers it.\n\n",
                    pid);
            found = 1;
            break;
        }
    }
    closedir(d);
    return found;
}

/*
 * A SINGLE INSTANCE, and that is all: we ARE the miner (monitor ->
 * jasminer.sh -> /usr/bin/jasminer.elf), so there is no factory binary to
 * coexist with; only a second instance of ours is dangerous.
 */
/*
 * NAME THE THREADS: /proc/<pid>/task/<tid>/comm then tells at once which
 * thread is eating the processor (sixteen characters).
 */
void jas_name_thread(const char *name)
{
    prctl(PR_SET_NAME, name, 0, 0, 0);
}

int jas_require_session(void)
{
    return jas_other_instance() ? -1 : 0;
}

const char *jas_hal_mode(void) { return "standalone (/dev/mem)"; }

volatile uint32_t *jas_axi_base(int thread_id)
{
    if (thread_id < 0 || thread_id > 2) return 0;
    return board_base[thread_id];
}

/*
 * Each jas_log line is timestamped to the millisecond since launch, like
 * the firmware's own phase timestamps: comparing durations is then a
 * subtraction.
 */
/*
 * applog_driver @ 0x36c78: the log the web interface shows. The original
 * appends "[YYYY-MM-DD HH:MM:SS] message" to
 * /media/data/driver/driver_<boot-date>.log, opened and closed at each
 * line, under a mutex. We write to /var/log/run.log, the path the CGIs
 * read HARD-CODED.
 */
/*
 * The log's descriptor, at file level: jas_applog must be able to close and
 * reopen it when the cap is reached. Kept static INSIDE applog_fd, the first
 * version truncated a local copy and left a dead pointer behind it.
 */
static FILE *log_f;
static const char log_path[] = "/var/log/run.log";

/*
 * Reopen the log after messagebak.sh deleted it. The old descriptor is
 * closed; otherwise the erased inode would stay allocated until the end of
 * the process, on a 124 MB RAM disk.
 */
static void jas_applog_reopen(void)
{
    FILE *n;

    n = fopen(log_path, "a");
    if (!n)
        return;                 /* keep the old one rather than nothing */
    setvbuf(n, NULL, _IOLBF, 0);
    if (log_f)
        fclose(log_f);
    log_f = n;
}

static FILE *applog_fd(void)
{
    static int attempt;
    char path[96];
    FILE *f = log_f;

    /*
     * THE FILE CAN VANISH FROM UNDER US: /etc/init.d/messagebak.sh archives
     * /var/log/run.log to /media/data/log/ then deletes it as soon as it
     * exceeds 100 KB. A shell producer reopens the path at each line; we keep
     * a FILE * open and would go on writing to the erased inode (lost trace,
     * space still taken, empty Run Log tab, a machine that looks dead while
     * it mines). So the path is checked every four lines (about ninety stat
     * calls per hour at the steady six lines per minute) and the log is
     * reopened within a minute.
     */
    if (f) {
        static unsigned tic;

        if ((++tic & 0x3u) == 0) {
            struct stat st;

            if (stat(log_path, &st) != 0) {
                jas_applog_reopen();
                f = log_f;
            }
        }
        return f;
    }
    if (attempt)
        return f;
    attempt = 1;
    {
        /*
         * THE LOG GOES TO /var/log/run.log, where the web page reads it
         * (log.cgi: "cat /var/log/run.log").
         *
         * WITH A CAP, and it is not optional: "/" is a 124 MB ramdisk; an
         * unbounded log would fill it in a few days and nothing would start
         * any more.
         */
        snprintf(path, sizeof(path), "%s", log_path);
    }
    f = fopen(path, "a");
    if (!f)
        return NULL;
    setvbuf(f, NULL, _IOLBF, 0);
    log_f = f;
    return f;
}

/*
 * The log's cap: beyond it, start again from zero.
 *
 * Simple and brutal, but suited to a ramdisk: a rotation by renaming would
 * keep two files and double the footprint. The log serves day-to-day
 * diagnosis, not archiving.
 */
#define LOG_MAX (512u * 1024u)

static void log_bound(void)
{
    long size;

    if (!log_f)
        return;
    size = ftell(log_f);
    if (size < 0 || (unsigned long)size < LOG_MAX)
        return;
    fclose(log_f);
    log_f = fopen(log_path, "w");        /* truncates */
    if (log_f) {
        setvbuf(log_f, NULL, _IOLBF, 0);
        fprintf(log_f, " [log truncated at %u KB]\n", LOG_MAX / 1024u);
    }
}

/*
 * TRAFFIC TRACE: off by default. Like the original, which logs events
 * (disconnections, reconnections, warnings), not the traffic: a fine trace
 * multiplies the log's volume by hundreds, and messagebak.sh deletes
 * run.log as soon as it exceeds 100 KB, leaving the Run Log tab empty most
 * of the time. JASRE_TRACE=1 turns it back on.
 */
int jas_trace_active(void)
{
    static int state = -1;

    if (state < 0)
        state = getenv("JASRE_TRACE") ? 1 : 0;
    return state;
}

void jas_applog(const char *line)
{
    FILE *f = applog_fd();
    time_t t;
    struct tm *tm;

    if (!f)
        return;
    t = time(NULL);
    tm = localtime(&t);
    if (!tm)
        return;
    fprintf(f, " [%04d-%02d-%02d %02d:%02d:%02d] %s\n",
            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec, line);
    {
        static unsigned n;

        /* An ftell per line would be a waste: once every 256 lines is enough
         * for a 512 KB cap. */
        if ((++n & 0xff) == 0)
            log_bound();
    }
}

/*
 * jas_trace: like jas_log, but SILENT unless JASRE_TRACE is set.
 *
 * For per ALU / bank / mesh / phase detail: occasionally useful when
 * debugging, noise the rest of the time. The original only logs summaries;
 * we imitate it by sending those lines through here. See jas_trace_active.
 */
void jas_trace(const char *fmt, ...)
{
    char line[1024];
    va_list ap;

    if (!jas_trace_active())
        return;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    jas_applog(line);
}

void jas_log(const char *fmt, ...)
{
    static struct timeval t0;
    struct timeval now;
    char line[1024];
    long ms;
    va_list ap;

    if (!t0.tv_sec) gettimeofday(&t0, NULL);
    gettimeofday(&now, NULL);
    ms = (now.tv_sec - t0.tv_sec) * 1000L + (now.tv_usec - t0.tv_usec) / 1000;

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    fprintf(stderr, "[%3ld.%03ld] %s\n", ms / 1000, ms % 1000, line);
    jas_applog(line);
}

/*
 * Recovery of the serial link after an error, from sl_single_read's error
 * path: a one-second wait, then a low-high pulse on the CLR window, ten
 * microseconds on either side. Without it, a link left busy (state 0x19,
 * busy and in error) is never released: sl_single_write waits for the busy
 * bit to drop, without a bound, and the process spins forever.
 */
/*
 * Hard recovery: two full cycles on CLR and RSTN, in the original's order
 * (CLR first on the way down, RSTN first on the way up). It releases a
 * link that a CLR pulse alone does not recover (state back to 0x05, then
 * stuck again at 0x09 at the first write).
 */
/*
 * UNDER THE LOCK, AND THAT IS THE LOCK'S REASON TO BE.
 *
 * These functions hold the PL in reset (I2C block and data windows
 * included) for hundreds of milliseconds, even seconds. Any AXI access from
 * another thread during that window gets no acknowledgement and freezes the
 * ARM core until the mains power is cut.
 */
void jas_link_hard_reset(unsigned delay_s, int thread_id)
{
    volatile uint32_t *clr, *rstn;

    if (thread_id < 0 || thread_id > 2)
        return;
    if (!board_clr_p[thread_id] || !board_rstn_p[thread_id])
        return;

    clr  = (volatile uint32_t *)board_clr_p[thread_id];
    rstn = (volatile uint32_t *)board_rstn_p[thread_id];

    /* The lock AFTER the validity tests: leaving without unlocking would block
     * the rest of the program. */
    jas_bus_lock();
    for (int cycle = 0; cycle < 2; cycle++) {
        *clr = 0;  *rstn = 0;
        sleep(delay_s);
        *rstn = 1; *clr = 1;
        sleep(delay_s);
    }
    jas_bus_unlock();
}

/*
 * axi_iic_addr_map_init @ 0x1b010 and axi_fan_addr_map_init @ 0x1b6ac:
 * the IIC bus (temperature sensors AND PMBus, hence the voltages) and the
 * fans, mapped in sensor_init before powering on. Addresses from the ELF's
 * `axi_base_wr` table @ 0x388c90; as for the main windows, boards 1 and 2
 * are NOT in address order. The firmware logs "board 0 mmap IIC base
 * address ok" then "mmap fan base address ok".
 */
static const off_t iic_base[3]  = { 0x41600000, 0x41620000, 0x41610000 };
static const off_t fan_base[2]  = { 0x40000000, 0x41000000 };  /* 0-1 then 2-3 */

void *jas_iic_p[3];
void *jas_fan_p[2];
void *jas_zynq_p;      /* g_mapZYNQ @ 0x80007000: logic version */

/*
 * The Zynq window: /proc/<pid>/maps of the firmware lists it at
 * 0x80007000, one page. It is g_mapZYNQ, from which power_on_operation
 * reads the logic version announced in its banner.
 */
int jas_zynq_map_init(void)
{
    int fd;

    if (jas_zynq_p)
        return 0;
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -1;
    jas_zynq_p = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED,
                      fd, (off_t)0x80007000);
    close(fd);
    if (jas_zynq_p == MAP_FAILED) { jas_zynq_p = NULL; return -1; }
    return 0;
}

int jas_iic_map_init(int thread_id)
{
    int fd;

    if (thread_id < 0 || thread_id > 2)
        return -1;
    if (jas_iic_p[thread_id])
        return 0;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("board %d open /dev/mem fail.\n", thread_id);
        return -1;
    }
    printf("board %d open /dev/mem ok.\n", thread_id);
    jas_iic_p[thread_id] = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, iic_base[thread_id]);
    close(fd);
    if (jas_iic_p[thread_id] == MAP_FAILED) {
        jas_iic_p[thread_id] = NULL;
        jas_log("board %d: iic mapping failed", thread_id);
        return -1;
    }
    printf("board %d mmap IIC base address ok.\n", thread_id);
    return 0;
}

int jas_fan_map_init(void)
{
    int fd, i;

    if (jas_fan_p[0] && jas_fan_p[1])
        return 0;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("open /dev/mem fail.\n");
        return -1;
    }
    printf("open /dev/mem ok.\n");
    for (i = 0; i < 2; i++) {
        jas_fan_p[i] = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, fan_base[i]);
        if (jas_fan_p[i] == MAP_FAILED)
            jas_fan_p[i] = NULL;
    }
    close(fd);
    if (!jas_fan_p[0] || !jas_fan_p[1]) {
        jas_log("fan mapping failed");
        return -1;
    }
    printf("mmap fan base address ok.\n");
    return 0;
}

/*
 * sl_w_fpga_hard_rstn_0 @ 0x1d6e8: ALL the boards at once, the very first
 * thing board_check does, before knowing which connectors are populated:
 * two cycles on all the boards, RSTN released 100 ms before CLR.
 * Duration: 2 x (1 s + 100 ms + 1 s) = 4.2 s; the firmware logs "reset
 * fpga hard reset done" for the three boards in the same second.
 */
/*
 * UNDER THE LOCK, AND THAT IS THE LOCK'S REASON TO BE.
 *
 * These functions hold the PL in reset (I2C block and data windows
 * included) for hundreds of milliseconds, even seconds. Any AXI access from
 * another thread during that window gets no acknowledgement and freezes the
 * ARM core until the mains power is cut.
 */
void jas_link_hard_reset_all(void)
{
    jas_bus_lock();
    unsigned b;

    for (int cycle = 0; cycle < 2; cycle++) {
        for (b = 0; b < BOARD_NUM; b++)
            if (board_clr_p[b] && board_rstn_p[b]) {
                *(volatile uint32_t *)board_clr_p[b]  = 0;
                *(volatile uint32_t *)board_rstn_p[b] = 0;
            }
        sleep(1);

        for (b = 0; b < BOARD_NUM; b++)
            if (board_rstn_p[b])
                *(volatile uint32_t *)board_rstn_p[b] = 1;
        usleep(100000);

        for (b = 0; b < BOARD_NUM; b++)
            if (board_clr_p[b])
                *(volatile uint32_t *)board_clr_p[b] = 1;
        sleep(1);
    }
    jas_bus_unlock();
}

/*
 * sl_w_fpga_hard_rstn_1 @ 0x1d7a4: a single board, short delays.
 *
 * Same shape as the parameterized version, but in microseconds and with
 * RSTN going up 10 ms before CLR. board_check uses it BETWEEN two probing
 * attempts: a connector that did not answer must be woken up without
 * stalling the machine for two seconds at each try.
 *
 * Duration: 2 x (50 + 10 + 50) ms = 220 ms.
 */
/*
 * UNDER THE LOCK, AND THAT IS THE LOCK'S REASON TO BE.
 *
 * These functions hold the PL in reset (I2C block and data windows
 * included) for hundreds of milliseconds, even seconds. Any AXI access from
 * another thread during that window gets no acknowledgement and freezes the
 * ARM core until the mains power is cut.
 */
void jas_link_hard_reset_fast(int thread_id)
{
    volatile uint32_t *clr, *rstn;

    if (thread_id < 0 || thread_id > 2)
        return;
    if (!board_clr_p[thread_id] || !board_rstn_p[thread_id])
        return;

    clr  = (volatile uint32_t *)board_clr_p[thread_id];
    rstn = (volatile uint32_t *)board_rstn_p[thread_id];

    /* The lock AFTER the validity tests: see jas_link_hard_reset. */
    jas_bus_lock();
    for (int cycle = 0; cycle < 2; cycle++) {
        *clr = 0;  *rstn = 0;
        usleep(50000);
        *rstn = 1;
        usleep(10000);
        *clr = 1;
        usleep(50000);
    }
    jas_bus_unlock();
}

void jas_link_reset(int thread_id)
{
    volatile uint32_t *clr;

    if (thread_id < 0 || thread_id > 2 || !board_clr_p[thread_id])
        return;

    clr = (volatile uint32_t *)board_clr_p[thread_id];
    /* CLR alone does not pull RSTN down, but it resets the serial link: a
     * transfer from another thread during the pulse would be cut short. */
    jas_bus_lock();
    sleep(1);
    *clr = 0;
    usleep(10);
    *clr = 1;
    usleep(10);
    jas_bus_unlock();
}

/* --- shared state: ours, once the firmware is out of the way -------------- */

static uint16_t s_board_en                = 1;
static uint16_t s_spi_phase[3][3][4];
static uint16_t s_spi_status[3][3];

uint8_t  *jas_asic_num          = &s_asic_num;
uint8_t  *jas_asic_en           = s_asic_en;
uint16_t *jas_board_en          = &s_board_en;
uint16_t (*jas_spi_phase)[3][4] = s_spi_phase;
uint16_t (*jas_spi_status)[3]   = s_spi_status;

static uint16_t s_bank_good[3][12240], s_bank_partial[3][12240];
static uint16_t s_bank_bad[3][12240],  s_bank_partial_ref[3][12240];

uint16_t (*jas_bank_good)[12240]        = s_bank_good;
uint16_t (*jas_bank_partial)[12240]     = s_bank_partial;
uint16_t (*jas_bank_bad)[12240]         = s_bank_bad;
uint16_t (*jas_bank_partial_ref)[12240] = s_bank_partial_ref;
static uint16_t s_bank_good_ref[3][12240];
uint16_t (*jas_bank_good_ref)[12240]    = s_bank_good_ref;

static uint16_t s_alu_result[3][3][24];
uint16_t (*jas_alu_result)[3][24] = s_alu_result;

static uint64_t s_extranonce, s_dev_id;
static uint8_t  s_alu_close_flag;
static uint16_t s_alu_en[3][3][4][2];

uint64_t *jas_extranonce        = &s_extranonce;
uint64_t *jas_dev_id            = &s_dev_id;
uint8_t  *jas_alu_close_flag    = &s_alu_close_flag;
uint16_t (*jas_alu_en)[3][4][2] = s_alu_en;

static uint16_t s_cb_sc[3][3][120][255];
static uint16_t s_mc_en[3][3][4][64], s_mc_rf[3][3][4][64], s_mc_mon[3][3][4][64];

uint16_t (*jas_cb_sc_result)[3][120][255] = s_cb_sc;
uint16_t (*jas_mc_en)[3][4][64]           = s_mc_en;
uint16_t (*jas_mc_refresh_en)[3][4][64]   = s_mc_rf;
uint16_t (*jas_mc_monitor_en)[3][4][64]   = s_mc_mon;


/* ALU flags and good-ALU counts, used by core/alu_analysis.c and seed.c. */

/* The memory timing values were captured on a live machine (see
 * hal/fix_params.c). A wrong memory timing does not show at startup: it
 * shows as intermittent reads, hence as rejected shares. */
/* The 25 timing parameters live in hal/fix_params.c. */

static uint16_t s_alu_flag[3][360], s_good_alu[3][3];
uint16_t (*jas_alu_flag)[360]   = s_alu_flag;
uint16_t (*jas_good_alu_num)[3] = s_good_alu;
static uint16_t s_good_alu_dft[3][3];
uint16_t (*jas_good_alu_num_dft)[3] = s_good_alu_dft;

/* Three CONNECTORS to probe; board_check decides which carry a daughter
 * board. Value read in the ELF (BOARD_NUM_G @ 0x387dd3 = 3) and confirmed
 * by the firmware's log, which probes three. */
/* Bank counters and fpga_data fields, standalone version. */
static uint16_t s_good_bank_num_cbsc[9];
static uint16_t s_good_bank_num_cbsc_mesh[36];
static uint64_t s_fpga_block_number;
static uint64_t s_fpga_dag_size;
static ethash_light_t s_cache_zil;
static ethash_light_t s_cache_etc;
static uint8_t  s_coin_type;

uint16_t *jas_good_bank_num_cbsc      = s_good_bank_num_cbsc;
uint16_t *jas_good_bank_num_cbsc_mesh = s_good_bank_num_cbsc_mesh;
uint64_t *jas_fpga_block_number       = &s_fpga_block_number;
uint64_t *jas_fpga_dag_size           = &s_fpga_dag_size;
ethash_light_t *jas_cache_zil         = &s_cache_zil;
ethash_light_t *jas_cache_etc         = &s_cache_etc;
uint8_t  *jas_coin_type               = &s_coin_type;
static uint8_t s_fpga_target[32];
static uint8_t s_fpga_header[32];
uint8_t  *jas_fpga_target             = s_fpga_target;
uint8_t  *jas_fpga_header             = s_fpga_header;
static uint8_t s_fpga_seed[32];
uint8_t  *jas_fpga_seed               = s_fpga_seed;
int  jas_alarm_local[4];
int *jas_alarm_temp_missing = &jas_alarm_local[0];
int *jas_alarm_temp_high   = &jas_alarm_local[1];
int *jas_alarm_fan  = &jas_alarm_local[2];
int *jas_alarm_ctrl_c       = &jas_alarm_local[3];
static int64_t s_asic_max_capacity[9];
static float   s_eff_real_value[3][3][120];
static uint8_t s_screening_status[1880];
static int     s_seed_switch;
static uint8_t s_board_asic_num[3];
int64_t  *jas_asic_max_capacity       = s_asic_max_capacity;
float   (*jas_eff_real_value)[3][120] = s_eff_real_value;
uint8_t  *jas_screening_status        = s_screening_status;
int      *jas_seed_switch             = &s_seed_switch;
uint8_t  *jas_board_asic_num          = s_board_asic_num;

static uint8_t board_num_store = 3;
uint8_t *jas_board_num = &board_num_store;
uint16_t jas_sensor_board_en;   /* sensor_board_en_g @ 0x388c80 */

static char asic_id_store[3][3][JAS_ASIC_ID_LEN];

char *jas_asic_id(int board, unsigned asic)
{
    if (board < 0 || board >= 3 || asic >= 3)
        return NULL;
    return asic_id_store[board][asic];
}

/*
 * THE BUS LOCK, and why it is RECURSIVE.
 *
 * Three threads share the bus: mining, monitoring (I2C temperatures, once
 * per second) and display. On error, sl_single_read calls
 * sl_w_fpga_hard_rstn (`rstn = 0` then `rstn = 1`), which resets the WHOLE
 * PL, I2C block included: a temperature read in flight at that moment
 * never gets an acknowledgement, and the ARM core stalls for good (mains
 * power cut, nothing in REBOOT_STATUS). The original protects this path
 * with sl_dev_mutex (@0x533020, taken in sl_wrapper_work).
 *
 * RECURSIVE because both levels coexist: main.c takes the lock around
 * whole sequences (so a measurement is not cut in two), and the hardware
 * entry points each take it too; a plain mutex would deadlock.
 */
static pthread_mutex_t bus_mutex;

static void bus_mutex_init(void) __attribute__((constructor));
static void bus_mutex_init(void)
{
    pthread_mutexattr_t at;

    pthread_mutexattr_init(&at);
    pthread_mutexattr_settype(&at, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&bus_mutex, &at);
    pthread_mutexattr_destroy(&at);
}

void jas_bus_lock(void)   { pthread_mutex_lock(&bus_mutex); }
void jas_bus_unlock(void) { pthread_mutex_unlock(&bus_mutex); }

/* Standalone, these counts are ours. var_mc_trim receives the memory
 * settings; its starting value is the factory one, see below. */
static uint16_t gbn_pre[3][3], gbn_post[3][3], dpf[3][3], hbf[3][3];
/*
 * mc_trim: FACTORY VALUE, from the .data of the reference firmware
 * (var_mc_trim @0x387d48) = { 0x0000, 0x2000 }, i.e. the u32 8192 in
 * {hi16,lo16} packing, the same convention as ref_pre_period. It goes to
 * register 0x1c0c (7th u32 slot of the 0x1c00 block, right after the 6 of
 * ref_pre_period): a CYCLE COUNTER of the memory controller, active while
 * mining, despite the name "trim". JASRE_MC_TRIM=<u32> forces another
 * value (0 = none) for an A/B.
 */
static uint16_t mc_trim_store[2] = { 0x0000, 0x2000 };
static uint8_t  pretest_flag_store;
uint16_t (*jas_good_bank_num_pre)[3]  = gbn_pre;
uint16_t (*jas_good_bank_num_post)[3] = gbn_post;
uint16_t (*jas_dp_fail_num)[3]        = dpf;
uint16_t (*jas_hb_fail_num)[3]        = hbf;
uint16_t *jas_mc_trim                 = mc_trim_store;
uint8_t  *jas_dft_pretest_flag        = &pretest_flag_store;

/* Without the firmware, the chip type is not read yet: 0 = unknown, which
 * leaves the work mode to the configuration file. */
int jas_asic_type(void) { return 0; }

/*
 * THE CONFIGURATION PATH, in one place.
 *
 * It was hard-coded in pool.c, power.c and crossbar.c: three copies of the
 * same string, which a "-c" on the command line could not redirect.
 */
static char config_path[192] = "/media/configs/jasminer.json";

void jas_config_set(const char *path)
{
    if (path && *path) {
        snprintf(config_path, sizeof(config_path), "%s", path);
        jas_log("configuration: %s", config_path);
    }
}

const char *jas_config_file(void)
{
    return config_path;
}
