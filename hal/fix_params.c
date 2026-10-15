/*
 * Memory timing parameters: GENERATED TABLE.
 *
 * Twenty-five settings the firmware applies before any use of the memory:
 * read, write and refresh latencies of the embedded DRAM. In bridge mode they
 * are read from the firmware's globals; standalone we must carry them
 * ourselves, and this table freezes them.
 *
 * Captured from a running stock firmware (/proc/<pid>/mem), version of
 * 2024-10-14. They are platform values, not machine values, but they depend
 * on the firmware: capture them again after an update.
 *
 * WHAT HAPPENS IF THEY ARE MISSING: the memory stops answering. Manual writes
 * time out one time in two, reads never return anything, and the crossbar
 * scan ends reporting no bank at all. Nothing in these symptoms points at the
 * timings, hence this note.
 */

#include "jas_state.h"

static const uint16_t v_alu_thread = 0x0080;
static const uint16_t v_nonce_interval = 0x0168;
static const uint16_t v_req_interval = 0x0000;
static const uint16_t v_rdy_cycle = 0x0024;
static const uint16_t v_wen_cycle = 0x005d;
static const uint16_t v_cb_wait_delay = 0x0400;
/*
 * STOCK access latencies (lengthening them by 20 % recovered no bank: the
 * marginal banks are a hard defect, not a timing limit).
 */
static const uint16_t v_rd_ra2wl = 0x0003;
static const uint16_t v_rd_wl2sa = 0x0005;
static const uint16_t v_rd_sa2csl = 0x0003;
static const uint16_t v_rd_csl2wloff = 0x0002;
static const uint16_t v_rd_wloff2saoff = 0x0003;
static const uint16_t v_rd_csl2dout = 0x0005;
static const uint16_t v_wr_ra2wl = 0x0003;
static const uint16_t v_wr_wl2sa = 0x0005;
static const uint16_t v_wr_sa2csl = 0x0003;
static const uint16_t v_wr_csl2wloff = 0x0007;
static const uint16_t v_wr_wloff2saoff = 0x0003;
static const uint16_t v_ref_ra2wl = 0x0003;
static const uint16_t v_ref_wl2sa = 0x0005;
static const uint16_t v_ref_wloff2saoff = 0x0003;
static const uint16_t v_ref_ras = 0x0011;
static const uint16_t v_ref_start_row = 0x0000;
static const uint16_t v_ref_start_time[12] = { 0x001a, 0xb008, 0x001a, 0xb008, 0x001a, 0xb008, 0x001a, 0xb008, 0x001a, 0xb008, 0x001a, 0xb008 };
static const uint16_t v_ref_period[12] = { 0x0013, 0x12d0, 0x0013, 0x12d0, 0x0013, 0x12d0, 0x0013, 0x12d0, 0x0013, 0x12d0, 0x0013, 0x12d0 };
static const uint16_t v_ref_pre_period[12] = { 0x0000, 0x01f4, 0x0000, 0x01f4, 0x0000, 0x01f4, 0x0000, 0x01f4, 0x0000, 0x01f4, 0x0000, 0x01f4 };

static const jas_fix_param fix_params[] = {
    { 0x042 , 0xf8,  1, &v_alu_thread        },   /* alu_thread */
    { 0x2d9 , 0xf8,  1, &v_nonce_interval    },   /* nonce_interval */
    { 0x2da , 0xf8,  1, &v_req_interval      },   /* req_interval */
    { 0x307 , 0x18,  1, &v_rdy_cycle         },   /* rdy_cycle */
    { 0x308 , 0x18,  1, &v_wen_cycle         },   /* wen_cycle */
    { 0x5bc , 0xf8,  1, &v_cb_wait_delay     },   /* cb_wait_delay */
    { 0xcd0 , 0xf8,  1, &v_rd_ra2wl          },   /* rd_ra2wl */
    { 0xcd1 , 0xf8,  1, &v_rd_wl2sa          },   /* rd_wl2sa */
    { 0xcd2 , 0xf8,  1, &v_rd_sa2csl         },   /* rd_sa2csl */
    { 0xcd3 , 0xf8,  1, &v_rd_csl2wloff      },   /* rd_csl2wloff */
    { 0xcd4 , 0xf8,  1, &v_rd_wloff2saoff    },   /* rd_wloff2saoff */
    { 0xcd5 , 0xf8,  1, &v_rd_csl2dout       },   /* rd_csl2dout */
    { 0xcd6 , 0xf8,  1, &v_wr_ra2wl          },   /* wr_ra2wl */
    { 0xcd7 , 0xf8,  1, &v_wr_wl2sa          },   /* wr_wl2sa */
    { 0xcd8 , 0xf8,  1, &v_wr_sa2csl         },   /* wr_sa2csl */
    { 0xcd9 , 0xf8,  1, &v_wr_csl2wloff      },   /* wr_csl2wloff */
    { 0xcda , 0xf8,  1, &v_wr_wloff2saoff    },   /* wr_wloff2saoff */
    { 0xcdb , 0xf8,  1, &v_ref_ra2wl         },   /* ref_ra2wl */
    { 0xcdc , 0xf8,  1, &v_ref_wl2sa         },   /* ref_wl2sa */
    { 0xcdd , 0xf8,  1, &v_ref_wloff2saoff   },   /* ref_wloff2saoff */
    { 0xcde , 0xf8,  1, &v_ref_ras           },   /* ref_ras */
    { 0xcdf , 0xf8,  1, &v_ref_start_row     },   /* ref_start_row */
    { 0xce0 , 0xf8, 12, v_ref_start_time     },   /* ref_start_time */
    { 0xcec , 0xf8, 12, v_ref_period         },   /* ref_period */
    { 0x1c00, 0xf8, 12, v_ref_pre_period     },   /* ref_pre_period */
    { 0, 0, 0, 0 }
};

const jas_fix_param *jas_fix_params(void) { return fix_params; }
