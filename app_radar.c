/********************************************************************************************************
 * @file    app_radar.c
 *
 * @brief   Radar UART frame parsing + prediction + step-motor tracking (UI_RADAR_ENABLE)
 *
 *******************************************************************************************************/
#include "app_radar.h"

#ifdef UI_RADAR_ENABLE

#include "drivers.h"
#include "tc32.h"
#include "SineTable.h"
#include "StepMotor.h"

#include "app_config.h"
#include "app_ctrl.h"

#ifndef RADAR_GIMBAL_DEBUG_TRACK_PROC_ONLY
/** Set to 1 (e.g. in app_config.h) to drive gimbal only from mirrored proc_x/proc_y each frame — no prediction path. */
#define RADAR_GIMBAL_DEBUG_TRACK_PROC_ONLY 0
#endif

#define RADAR_INSTALL_HEIGHT_DEFAULT_MM (2.5f * 1000.0f)
#define RADAR_FRAME_LEN                 30
#define SAMPLE_COUNT                    1
#define STATIONARY_DXY_THRESHOLD_MM     5
#define RADAR_IDLE_TIMEOUT_US           (1000000u * 30u)  // 30s
#define RADAR_SESSION_REST_US           (1000000u * 60u)  // 1min
#define RADAR_SESSION_MAX_US            (1000000u * 15u)  // 10min

#define RADAR_TIME_SEC_PER_DAY          86400u

#define RADAR_PLAY_STATE_IDLE           0
#define RADAR_PLAY_STATE_ACTIVE         1
#define RADAR_PLAY_END_ONGOING          0xFFFFFFFFu

_attribute_data_retention_ volatile u8 g_uart_ndma_rx_byte[RADAR_FRAME_LEN];
_attribute_data_retention_ volatile u8 g_uart_ndma_rx_byte_cnt = 0;
_attribute_data_retention_ volatile u8 g_uart_ndma_rx_flag     = 0;

_attribute_data_retention_ static int32_t radar_x_sum      = 0;
_attribute_data_retention_ static int32_t radar_y_sum      = 0;
_attribute_data_retention_ static int32_t radar_v_sum      = 0;
_attribute_data_retention_ static u8      radar_sample_cnt = 0;
_attribute_data_retention_ static u32     radar_rand_seed  = 0x12345678UL;

typedef struct
{
    s16   prev_x_mm;
    s16   prev_y_mm;
    u8    has_prev;
    u8    stationary_frames;
    u8    has_last_motion_dir;
    float last_motion_dir_rad;

    // Cache recent points for stable motion direction (1s window)
    u8 motion_cache_head;
    u8 motion_cache_count;
} radar_prediction_state_t;

typedef struct
{
    s16 x_mm;
    s16 y_mm;
    u32 tick;
} radar_motion_point_t;

typedef struct
{
    s16 pan_deg10;
    s16 tilt_deg10;
} radar_gimbal_point_t;

typedef struct
{
    s32 x_mm;
    s32 y_mm;
} radar_boundary_point_t;

typedef enum
{
    RADAR_GIMBAL_FSM_IDLE = 0,
    RADAR_GIMBAL_FSM_RUN,
    RADAR_GIMBAL_FSM_WAIT_DONE,
} radar_gimbal_fsm_t;

#define RADAR_GIMBAL_SEQ_MAX_POINTS 16

/* 单段移动耗时（相邻两点之间），单位与 clock_time_exceed 第二参数一致（微秒） */
#define RADAR_GIMBAL_STEP_US_SLOW   400000u /* 0.4 s */
#define RADAR_GIMBAL_STEP_US_MED    200000u /* 0.2 s */
#define RADAR_GIMBAL_STEP_US_FAST   100000u /* 0.1 s */

/* 雷达速度单位 cm/s：0.5 m/s = 50，1 m/s = 100 */
#define RADAR_SPEED_LOW_CM_S        50
#define RADAR_SPEED_MED_CM_S        100

_attribute_data_retention_ static radar_prediction_state_t g_radar_pred = {0};

// Motion direction cache (keep last 1 second of points)
#define RADAR_MOTION_CACHE_WINDOW_US  1000000u
#define RADAR_MOTION_CACHE_MAX_POINTS 64
_attribute_data_retention_ static radar_motion_point_t g_radar_motion_cache[RADAR_MOTION_CACHE_MAX_POINTS] = {0};

static inline void RadarMotionCacheReset(void)
{
    g_radar_pred.has_last_motion_dir = 0;
    g_radar_pred.last_motion_dir_rad = 0.0f;
    g_radar_pred.motion_cache_head   = 0;
    g_radar_pred.motion_cache_count  = 0;
}

static inline u8 RadarMotionCacheIndexOldest(void)
{
    // oldest = head - count (mod N)
    u8 head  = g_radar_pred.motion_cache_head;
    u8 count = g_radar_pred.motion_cache_count;
    if (count == 0)
    {
        return 0;
    }
    s16 idx = (s16)head - (s16)count;
    while (idx < 0)
    {
        idx += RADAR_MOTION_CACHE_MAX_POINTS;
    }
    return (u8)idx;
}

static inline u8 RadarMotionCacheIndexNewest(void)
{
    // newest = head - 1 (mod N)
    u8 head = g_radar_pred.motion_cache_head;
    if (g_radar_pred.motion_cache_count == 0)
    {
        return 0;
    }
    return (u8)((head == 0) ? (RADAR_MOTION_CACHE_MAX_POINTS - 1) : (head - 1));
}

static void RadarMotionCachePrune(u32 now_tick)
{
    while (g_radar_pred.motion_cache_count)
    {
        u8  idx    = RadarMotionCacheIndexOldest();
        u32 age_us = (now_tick - g_radar_motion_cache[idx].tick) >> 4;
        if (age_us <= RADAR_MOTION_CACHE_WINDOW_US)
        {
            break;
        }
        g_radar_pred.motion_cache_count--;
    }
}

static void RadarMotionCachePush(u32 now_tick, s16 x_mm, s16 y_mm)
{
    RadarMotionCachePrune(now_tick);

    // de-dup: if newest is same point, refresh tick only
    if (g_radar_pred.motion_cache_count)
    {
        u8 newest = RadarMotionCacheIndexNewest();
        if (g_radar_motion_cache[newest].x_mm == x_mm && g_radar_motion_cache[newest].y_mm == y_mm)
        {
            g_radar_motion_cache[newest].tick = now_tick;
            return;
        }
    }

    g_radar_motion_cache[g_radar_pred.motion_cache_head].x_mm = x_mm;
    g_radar_motion_cache[g_radar_pred.motion_cache_head].y_mm = y_mm;
    g_radar_motion_cache[g_radar_pred.motion_cache_head].tick = now_tick;
    g_radar_pred.motion_cache_head                            = (u8)((g_radar_pred.motion_cache_head + 1) % RADAR_MOTION_CACHE_MAX_POINTS);

    if (g_radar_pred.motion_cache_count < RADAR_MOTION_CACHE_MAX_POINTS)
    {
        g_radar_pred.motion_cache_count++;
    }
}
_attribute_data_retention_ static radar_gimbal_point_t g_radar_seq_build[RADAR_GIMBAL_SEQ_MAX_POINTS]   = {0};
_attribute_data_retention_ static radar_gimbal_point_t g_radar_seq_active[RADAR_GIMBAL_SEQ_MAX_POINTS]  = {0};
_attribute_data_retention_ static radar_gimbal_point_t g_radar_seq_pending[RADAR_GIMBAL_SEQ_MAX_POINTS] = {0};
_attribute_data_retention_ static u8                   g_radar_seq_build_cnt                            = 0;
_attribute_data_retention_ static u8                   g_radar_seq_active_cnt                           = 0;
_attribute_data_retention_ static u8                   g_radar_seq_pending_cnt                          = 0;
_attribute_data_retention_ static u8                   g_radar_seq_active_idx                           = 0;
_attribute_data_retention_ static u8                   g_radar_seq_pending_valid                        = 0;
_attribute_data_retention_ static u8                   g_radar_seq_active_is_static                     = 0;
_attribute_data_retention_ static u8                   g_radar_seq_pending_is_static                    = 0;
_attribute_data_retention_ static u32                  g_radar_seq_step_interval_us                     = RADAR_GIMBAL_STEP_US_SLOW;
_attribute_data_retention_ static u32                  g_radar_seq_pending_step_us                      = RADAR_GIMBAL_STEP_US_SLOW;
_attribute_data_retention_ static u32                  g_radar_seq_step_tick                            = 0;
_attribute_data_retention_ static radar_gimbal_fsm_t   g_radar_gimbal_fsm                               = RADAR_GIMBAL_FSM_IDLE;
_attribute_data_retention_ static s16                  g_radar_busy_max_speed_abs                       = 0;
_attribute_data_retention_ static u8                   g_radar_static_loop_en                           = 0;
/* 下一段静止序列：1=逃逸点列，0=两随机点 (ax,ay)->(bx,by)；首次为两随机点 */
_attribute_data_retention_ static u8    g_radar_static_next_is_escape = 0;
_attribute_data_retention_ static s16   g_radar_static_ax_mm          = 0;
_attribute_data_retention_ static s16   g_radar_static_ay_mm          = 0;
_attribute_data_retention_ static s16   g_radar_static_bx_mm          = 0;
_attribute_data_retention_ static s16   g_radar_static_by_mm          = 0;
_attribute_data_retention_ static float g_radar_static_dir_rad        = 0.0f;
_attribute_data_retention_ static u32   g_radar_last_motion_tick      = 0;
_attribute_data_retention_ static u32   g_radar_session_start_tick    = 0;
_attribute_data_retention_ static u32   g_radar_rest_start_tick       = 0;
_attribute_data_retention_ static u8    g_radar_resting               = 0;

_attribute_data_retention_ static u32 g_radar_time_sec          = 0;
_attribute_data_retention_ static u32 g_radar_time_tick_acc_us  = 0;
_attribute_data_retention_ static u8  g_radar_time_valid        = 0;
_attribute_data_retention_ static s8  g_radar_time_tz_q15       = 0;
_attribute_data_retention_ static u8  g_radar_play_state        = RADAR_PLAY_STATE_IDLE;
_attribute_data_retention_ static u32 g_radar_play_active_start = 0;

_attribute_data_retention_ static u32 g_radar_play_start_sec[RADAR_TIME_MAX_RECORDS] = {0};
_attribute_data_retention_ static u32 g_radar_play_end_sec[RADAR_TIME_MAX_RECORDS]   = {0};
_attribute_data_retention_ static s8  g_radar_play_tz_q15[RADAR_TIME_MAX_RECORDS]    = {0};
_attribute_data_retention_ static u8  g_radar_play_record_count                      = 0;
_attribute_data_retention_ static u8  g_radar_play_record_next                       = 0;
_attribute_data_retention_ static u8  g_radar_boundary_configured                    = 0;
_attribute_data_retention_ static u8  g_radar_install_height_set                     = 0;

/* 边界多边形：默认是矩形，但支持修改为任意凸四边形（点顺序需为逆时针或顺时针一致） */
static const radar_boundary_point_t g_radar_boundary_quad_default[4] = {
    {-1000, 4000},
    {1000, 4000},
    {1000, 400},
    {-1000, 400},
};

_attribute_data_retention_ static radar_boundary_point_t g_radar_boundary_quad[4] = {
    {-600, 100},
    {600, 800},
    {800, 4000},
    {-1000, 3000},
};

#define RADAR_BOUNDARY_FLASH_MAGIC       0x52424452u  // "RBDR"
#define RADAR_INSTALL_HEIGHT_FLASH_MAGIC 0x52444948u  // "RDIH"
#define RADAR_PLAY_RECORD_FLASH_MAGIC    0x5244504Cu  // "RDPL"

typedef struct
{
    u32 magic;
    s32 x_mm[4];
    s32 y_mm[4];
    u32 crc;
} radar_boundary_flash_t;

typedef struct
{
    u32 magic;
    s32 height_mm;
    u8  height_set;
    u8  reserved[3];
    u32 crc;
} radar_install_height_flash_t;

typedef struct
{
    u32 magic;
    u8  record_count;
    u8  record_next;
    u8  reserved[2];
    u32 start_sec[RADAR_TIME_MAX_RECORDS];
    u32 end_sec[RADAR_TIME_MAX_RECORDS];
    s8  tz_q15[RADAR_TIME_MAX_RECORDS];
    u8  tz_reserved[3];
    u32 crc;
} radar_play_record_flash_t;

_attribute_data_retention_ static s32 g_radar_install_height_mm = (s32)RADAR_INSTALL_HEIGHT_DEFAULT_MM;

static inline s16 RadarSpeedAbs(s16 v_cm_s)
{
    return (s16)((v_cm_s < 0) ? (-v_cm_s) : v_cm_s);
}

static u32 radar_boundary_crc32(const u8 *data, u32 len)
{
    u32 crc = 0xFFFFFFFFu;
    for (u32 i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (u8 b = 0; b < 8; b++)
        {
            if (crc & 1u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void radar_play_records_reset_ram(void)
{
    for (u8 i = 0; i < RADAR_TIME_MAX_RECORDS; i++)
    {
        g_radar_play_start_sec[i] = 0;
        g_radar_play_end_sec[i]   = 0;
        g_radar_play_tz_q15[i]    = 0;
    }
    g_radar_play_record_count = 0;
    g_radar_play_record_next  = 0;
    g_radar_play_state        = RADAR_PLAY_STATE_IDLE;
    g_radar_play_active_start = 0;
}

static void radar_play_records_save_to_flash(void)
{
    radar_play_record_flash_t stored;
    stored.magic        = RADAR_PLAY_RECORD_FLASH_MAGIC;
    stored.record_count = g_radar_play_record_count;
    stored.record_next  = g_radar_play_record_next;
    stored.reserved[0]  = 0;
    stored.reserved[1]  = 0;

    for (u8 i = 0; i < RADAR_TIME_MAX_RECORDS; i++)
    {
        stored.start_sec[i] = g_radar_play_start_sec[i];
        stored.end_sec[i]   = g_radar_play_end_sec[i];
        stored.tz_q15[i]    = g_radar_play_tz_q15[i];
    }

    stored.tz_reserved[0] = 0;
    stored.tz_reserved[1] = 0;
    stored.tz_reserved[2] = 0;

    stored.crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));

    flash_erase_sector(RADAR_PLAY_RECORD_FLASH_ADDR);
    flash_write_page(RADAR_PLAY_RECORD_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
}

static void radar_play_records_load_from_flash(void)
{
    radar_play_record_flash_t stored;
    flash_read_page(RADAR_PLAY_RECORD_FLASH_ADDR, sizeof(stored), (u8 *)&stored);

    if (stored.magic != RADAR_PLAY_RECORD_FLASH_MAGIC)
    {
        radar_play_records_reset_ram();
        return;
    }

    u32 crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    if (crc != stored.crc)
    {
        radar_play_records_reset_ram();
        return;
    }

    if (stored.record_count > RADAR_TIME_MAX_RECORDS)
    {
        radar_play_records_reset_ram();
        return;
    }

    for (u8 i = 0; i < RADAR_TIME_MAX_RECORDS; i++)
    {
        g_radar_play_start_sec[i] = stored.start_sec[i];
        g_radar_play_end_sec[i]   = stored.end_sec[i];
        g_radar_play_tz_q15[i]    = stored.tz_q15[i];
    }

    g_radar_play_record_count = stored.record_count;
    g_radar_play_record_next  = stored.record_next;
    if (g_radar_play_record_next >= RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_next = 0;
    }
    g_radar_play_state        = RADAR_PLAY_STATE_IDLE;
    g_radar_play_active_start = 0;
}

static u8 radar_is_leap_year(u16 year)
{
    return (((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0)) ? 1 : 0;
}

static u16 radar_days_in_year(u16 year)
{
    return radar_is_leap_year(year) ? 366 : 365;
}

static u8 radar_days_in_month(u16 year, u8 month)
{
    static const u8 days_by_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12)
    {
        return 30;
    }
    if (month == 2 && radar_is_leap_year(year))
    {
        return 29;
    }
    return days_by_month[month - 1];
}

static void radar_epoch_to_datetime(u32  epoch_sec,
                                    u16 *year,
                                    u8  *month,
                                    u8  *day,
                                    u8  *hour,
                                    u8  *minute,
                                    u8  *second)
{
    u32 days = epoch_sec / RADAR_TIME_SEC_PER_DAY;
    u32 rem  = epoch_sec % RADAR_TIME_SEC_PER_DAY;

    u16 y = 1970;
    while (days >= radar_days_in_year(y))
    {
        days -= radar_days_in_year(y);
        y++;
    }

    u8 m = 1;
    while (days >= radar_days_in_month(y, m))
    {
        days -= radar_days_in_month(y, m);
        m++;
    }

    if (year)
    {
        *year = y;
    }
    if (month)
    {
        *month = m;
    }
    if (day)
    {
        *day = (u8)(days + 1);
    }
    if (hour)
    {
        *hour = (u8)(rem / 3600u);
    }
    rem %= 3600u;
    if (minute)
    {
        *minute = (u8)(rem / 60u);
    }
    if (second)
    {
        *second = (u8)(rem % 60u);
    }
}

static void radar_play_record_push(u32 start_sec, u32 end_sec)
{
    g_radar_play_start_sec[g_radar_play_record_next] = start_sec;
    g_radar_play_end_sec[g_radar_play_record_next]   = end_sec;
    g_radar_play_tz_q15[g_radar_play_record_next]    = g_radar_time_tz_q15;

    g_radar_play_record_next++;
    if (g_radar_play_record_next >= RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_next = 0;
    }
    if (g_radar_play_record_count < RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_count++;
    }
}

static void radar_play_record_start(void)
{
    if (!g_radar_time_valid)
    {
        return;
    }

    if (g_radar_play_state != RADAR_PLAY_STATE_ACTIVE)
    {
        g_radar_play_state        = RADAR_PLAY_STATE_ACTIVE;
        g_radar_play_active_start = g_radar_time_sec;
        radar_play_record_push(g_radar_play_active_start, RADAR_PLAY_END_ONGOING);
        radar_play_records_save_to_flash();
        LOG_D("radar_play_record_push");
    }
    else
    {
        // g_radar_play_end_sec[(g_radar_play_record_next + RADAR_TIME_MAX_RECORDS - 1) % RADAR_TIME_MAX_RECORDS] = RADAR_PLAY_END_ONGOING;
    }
}

static void radar_play_record_end(void)
{
    if (!g_radar_time_valid || g_radar_play_state != RADAR_PLAY_STATE_ACTIVE)
    {
        return;
    }
    LOG_D("radar_play_record_end");

    g_radar_play_end_sec[(g_radar_play_record_next + RADAR_TIME_MAX_RECORDS - 1) % RADAR_TIME_MAX_RECORDS] = g_radar_time_sec;
    g_radar_play_state                                                                                     = RADAR_PLAY_STATE_IDLE;
    radar_play_records_save_to_flash();
}

void app_radar_set_time_from_epoch(u32 epoch_sec, s8 tz_q15)
{
    g_radar_time_sec         = epoch_sec;
    g_radar_time_tz_q15      = tz_q15;
    g_radar_time_tick_acc_us = 0;
    g_radar_time_valid       = 1;

    u16 year  = 0;
    u8  month = 0;
    u8  day   = 0;
    u8  hour  = 0;
    u8  min   = 0;
    u8  sec   = 0;
    radar_epoch_to_datetime(epoch_sec, &year, &month, &day, &hour, &min, &sec);

    LOG_D("epoch_sec: %d, tz_q15: %d", epoch_sec, tz_q15);
    LOG_D("datetime: %04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, min, sec);
}

void app_radar_on_time_tick(void)
{
    if (!g_radar_time_valid)
    {
        return;
    }

    g_radar_time_tick_acc_us += 1000000u;
    if (g_radar_time_tick_acc_us >= 1000000u)
    {
        g_radar_time_tick_acc_us -= 1000000u;
        g_radar_time_sec++;
    }
}

int app_radar_get_play_records(u32 *out_buf, u8 *tz_buf, u8 max_records)
{
    if (!out_buf || !tz_buf || max_records == 0)
    {
        return 0;
    }

    u8 count = g_radar_play_record_count;
    if (count > max_records)
    {
        count = max_records;
    }

    u8 idx = g_radar_play_record_next;
    for (u8 i = 0; i < count; i++)
    {
        if (idx == 0)
        {
            idx = RADAR_TIME_MAX_RECORDS;
        }
        idx--;
        out_buf[i * 2]     = g_radar_play_start_sec[idx];
        out_buf[i * 2 + 1] = g_radar_play_end_sec[idx];
        tz_buf[i]          = (u8)g_radar_play_tz_q15[idx];
    }

    return count;
}

u8 app_radar_is_boundary_set(void)
{
    return g_radar_boundary_configured;
}

u8 app_radar_is_install_height_set(void)
{
    return g_radar_install_height_set;
}

int radar_boundary_load_from_flash(s32 x_mm[4], s32 y_mm[4])
{
    radar_boundary_flash_t stored;
    flash_read_page(RADAR_BOUNDARY_FLASH_ADDR, sizeof(stored), (u8 *)&stored);

    if (stored.magic != RADAR_BOUNDARY_FLASH_MAGIC)
    {
        return 0;
    }

    u32 crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    if (crc != stored.crc)
    {
        return 0;
    }

    for (int i = 0; i < 4; i++)
    {
        x_mm[i] = stored.x_mm[i];
        y_mm[i] = stored.y_mm[i];
    }

    return 1;
}

static int radar_install_height_load_from_flash(void)
{
    radar_install_height_flash_t stored;
    flash_read_page(RADAR_INSTALL_HEIGHT_FLASH_ADDR, sizeof(stored), (u8 *)&stored);

    if (stored.magic != RADAR_INSTALL_HEIGHT_FLASH_MAGIC)
    {
        return 0;
    }

    u32 crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    if (crc != stored.crc)
    {
        return 0;
    }

    if (stored.height_set)
    {
        g_radar_install_height_mm  = stored.height_mm;
        g_radar_install_height_set = 1;
    }

    return 1;
}

void app_radar_clear_install_height_and_boundary_flash(void)
{
    g_radar_install_height_mm   = (s32)RADAR_INSTALL_HEIGHT_DEFAULT_MM;
    g_radar_install_height_set  = 0;
    g_radar_boundary_configured = 0;

    for (int i = 0; i < 4; i++)
    {
        g_radar_boundary_quad[i].x_mm = g_radar_boundary_quad_default[i].x_mm;
        g_radar_boundary_quad[i].y_mm = g_radar_boundary_quad_default[i].y_mm;
    }

    radar_play_records_reset_ram();

    flash_erase_sector(RADAR_BOUNDARY_FLASH_ADDR);
    flash_erase_sector(RADAR_INSTALL_HEIGHT_FLASH_ADDR);
    flash_erase_sector(RADAR_PLAY_RECORD_FLASH_ADDR);
}

int app_radar_save_boundary_quad_to_flash(const s32 x_mm[4], const s32 y_mm[4])
{
    if (!x_mm || !y_mm)
    {
        return 0;
    }

    radar_boundary_flash_t stored;
    stored.magic = RADAR_BOUNDARY_FLASH_MAGIC;
    for (int i = 0; i < 4; i++)
    {
        stored.x_mm[i] = x_mm[i];
        stored.y_mm[i] = y_mm[i];
    }
    stored.crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));

    flash_erase_sector(RADAR_BOUNDARY_FLASH_ADDR);
    flash_write_page(RADAR_BOUNDARY_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
    LOG_D("radar_boundary_save_to_flash success");
    for (int i = 0; i < 4; i++)
    {
        LOG_D("x_mm[%d]: %d, y_mm[%d]: %d", i, x_mm[i], i, y_mm[i]);
    }
    return 1;
}

void app_radar_init(void)
{
    s32 x_mm[4];
    s32 y_mm[4];

    if (radar_boundary_load_from_flash(x_mm, y_mm))
    {
        LOG_D("radar_boundary_load_from_flash success");
        app_radar_set_boundary_quad(x_mm, y_mm);
        for (int i = 0; i < 4; i++)
        {
            LOG_D("x_mm[%d]: %d, y_mm[%d]: %d", i, x_mm[i], i, y_mm[i]);
        }
        g_radar_boundary_configured = 1;
    }
    else
    {
        LOG_D("radar_boundary_load_from_flash failed");
        app_radar_reset_boundary_default();
        g_radar_boundary_configured = 0;
    }

    if (radar_install_height_load_from_flash())
    {
        LOG_D("radar_install_height_load_from_flash success: %d", g_radar_install_height_mm);
    }
    else
    {
        LOG_D("radar_install_height_load_from_flash failed");
        g_radar_install_height_set = 0;
    }

    radar_play_records_load_from_flash();
}

static u8 RadarGimbalIsBusy(void)
{
    return (g_radar_gimbal_fsm != RADAR_GIMBAL_FSM_IDLE);
}

static s16 RadarGetSpeedForPrediction(s16 v_cm_s)
{
    s16 cur_abs = RadarSpeedAbs(v_cm_s);

    if (!RadarGimbalIsBusy())
    {
        g_radar_busy_max_speed_abs = cur_abs;
        return cur_abs;
    }

    if (cur_abs > g_radar_busy_max_speed_abs)
    {
        g_radar_busy_max_speed_abs = cur_abs;
    }

    return g_radar_busy_max_speed_abs;
}

void app_radar_point_to_pan_tilt(s32 x_mm, s32 y_mm, s32 height_mm, s16 *pan_deg10, s16 *tilt_deg10)
{
    // 根据x_mm和y_mm计算出r1_mm
    s32 r1_mm = app_radar_mysqrt_3(x_mm * x_mm + y_mm * y_mm);
    LOG_D("r1_mm: %d", r1_mm);
    // 在根据r_mm和h_mm计算出tilt_deg
    *tilt_deg10 = (s16)(lookup_atan2((float)r1_mm, (float)height_mm) * RAD_TO_DEG * 10.0f) - 900;
    LOG_D("tilt_deg10: %d", *tilt_deg10);
    // 根据y_mm和h_mm计算出r2_mm
    s32 r2_mm = app_radar_mysqrt_3(y_mm * y_mm + height_mm * height_mm);
    LOG_D("r2_mm: %d", r2_mm);
    // 根据r2_mm和x_mm计算出pan_deg
    *pan_deg10 = (s16)(lookup_atan2((float)x_mm, (float)r2_mm) * RAD_TO_DEG * 10.0f);
    LOG_D("pan_deg10: %d", *pan_deg10);
}

static void RadarSeqBuildReset(void)
{
    g_radar_seq_build_cnt = 0;
}

static void RadarSeqStartBuilt(void)
{
    memcpy(g_radar_seq_active, g_radar_seq_build, sizeof(radar_gimbal_point_t) * g_radar_seq_build_cnt);
    g_radar_seq_active_cnt     = g_radar_seq_build_cnt;
    g_radar_seq_active_idx     = 0;
    g_radar_seq_step_tick      = clock_time();
    g_radar_busy_max_speed_abs = 0;
    g_radar_gimbal_fsm         = RADAR_GIMBAL_FSM_RUN;
    if (g_radar_seq_active_cnt == 2)
    {
        StepMotor_GimbalSetSpeedUs(2400);
    }
    if (g_radar_seq_active_cnt == 3)
    {
        StepMotor_GimbalSetSpeedUs(1800);
    }
    if (g_radar_seq_active_cnt == 6)
    {
        StepMotor_GimbalSetSpeedUs(1500);
    }
}

float app_radar_mysqrt_3(float x)
{
    float xhalf = 0.5f * x;
    int   i     = *(int *)&x;

    if (!x)
        return 0;

    i = 0x5f375a86 - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);

    return 1 / x;
}

void RadarSessionStop(u8 reset)
{
    // StepMotor_StopAll();
    g_radar_pred.has_prev         = 0;
    g_radar_seq_build_cnt         = 0;
    g_radar_seq_active_cnt        = 0;
    g_radar_seq_active_idx        = 0;
    g_radar_seq_pending_cnt       = 0;
    g_radar_seq_pending_valid     = 0;
    g_radar_seq_active_is_static  = 0;
    g_radar_seq_pending_is_static = 0;
    g_radar_gimbal_fsm            = RADAR_GIMBAL_FSM_IDLE;
    g_radar_busy_max_speed_abs    = 0;
    g_radar_static_loop_en        = 0;
    g_radar_static_next_is_escape = 0;
    g_radar_seq_step_interval_us  = RADAR_GIMBAL_STEP_US_SLOW;
    g_radar_seq_pending_step_us   = RADAR_GIMBAL_STEP_US_SLOW;
    RadarMotionCacheReset();
    if (reset)
    {
        g_radar_session_start_tick = 0;
    }
}

u8 RadarSessionIsResting(void)
{
    return g_radar_resting;
}

static void RadarSessionEnterRest(u32 now_tick)
{
    RadarSessionStop(1);
    StepMotor_StopAll();
    g_radar_resting         = 1;
    g_radar_rest_start_tick = now_tick;
}

static void RadarSessionOnMotion(u32 now_tick)
{
    g_radar_last_motion_tick = now_tick;
    if (g_radar_resting)
    {
        return;
    }

    if (g_radar_session_start_tick == 0)
    {
        g_radar_session_start_tick = now_tick;
    }
    radar_play_record_start();
    gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
}

static u8 RadarSessionCanPredict(u32 now_tick)
{
    static u8 count   = 0;
    u32       elapsed = (now_tick - g_radar_rest_start_tick) >> 4;
    if (g_radar_resting)
    {
        // 休息了60秒，退出休息状态
        if (elapsed >= RADAR_SESSION_REST_US)
        {
            g_radar_resting         = 0;
            g_radar_rest_start_tick = 0;
            LOG_D("%ds rest, exit rest", elapsed);
        }
        else
        {
            return 0;
        }
    }

    // 工作了600秒，进入休息状态
    elapsed = (now_tick - g_radar_session_start_tick) >> 4;
    if (g_radar_session_start_tick &&
        elapsed >= RADAR_SESSION_MAX_US)
    {
        count++;
        g_radar_session_start_tick = clock_time();
        if (count >= 4 * 10)
        {
            count = 0;
            LOG_D("work %ds, stop radar", elapsed);
            radar_play_record_end();
            RadarSessionEnterRest(now_tick);
            StepMotor_GimbalResetStart();
            gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
        }

        return 0;
    }

    if (g_radar_last_motion_tick == 0)
    {
        return 0;
    }

    // 30s没有运动，则停止雷达解析
    elapsed = (now_tick - g_radar_last_motion_tick) >> 4;
    if (elapsed >= RADAR_IDLE_TIMEOUT_US)
    {
        LOG_D("%ds no motion, stop radar", elapsed);
        StepMotor_StopAll();
        radar_play_record_end();
        RadarSessionStop(0);
        gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
        return 0;
    }

    return 1;
}

static void RadarSeqBuildAppendPoint(s16 x_mm, s16 y_mm)
{
    if (g_radar_seq_build_cnt >= RADAR_GIMBAL_SEQ_MAX_POINTS)
    {
        return;
    }

    s16 pan_deg10  = 0;
    s16 tilt_deg10 = 0;
    app_radar_point_to_pan_tilt(x_mm, y_mm, g_radar_install_height_mm, &pan_deg10, &tilt_deg10);
    g_radar_seq_build[g_radar_seq_build_cnt].pan_deg10  = pan_deg10;
    g_radar_seq_build[g_radar_seq_build_cnt].tilt_deg10 = tilt_deg10;
    g_radar_seq_build_cnt++;
}

/** 静止预测序列：排队或启动时标记为 static，供运动序列抢占判断 */
static void RadarSeqCommitBuiltStatic(void)
{
    if (g_radar_seq_build_cnt == 0)
    {
        return;
    }

    if (!RadarGimbalIsBusy())
    {
        RadarSeqStartBuilt();
        g_radar_seq_active_is_static = 1;
    }
    else
    {
        memcpy(g_radar_seq_pending, g_radar_seq_build, sizeof(radar_gimbal_point_t) * g_radar_seq_build_cnt);
        g_radar_seq_pending_cnt       = g_radar_seq_build_cnt;
        g_radar_seq_pending_valid     = 1;
        g_radar_seq_pending_step_us   = g_radar_seq_step_interval_us;
        g_radar_seq_pending_is_static = 1;
    }
}

/** 运动预测序列：若当前正在播放静止序列（含 WAIT_DONE 等待下一段静止），立即打断并播放本序列 */
static void RadarSeqCommitBuiltMotion(void)
{
    if (g_radar_seq_build_cnt == 0)
    {
        return;
    }

    if (g_radar_seq_active_is_static && RadarGimbalIsBusy())
    {
        RadarSeqStartBuilt();
        g_radar_seq_active_is_static  = 0;
        g_radar_seq_pending_valid     = 0;
        g_radar_seq_pending_cnt       = 0;
        g_radar_seq_pending_is_static = 0;
        return;
    }

    if (!RadarGimbalIsBusy())
    {
        RadarSeqStartBuilt();
        g_radar_seq_active_is_static = 0;
    }
    else
    {
        memcpy(g_radar_seq_pending, g_radar_seq_build, sizeof(radar_gimbal_point_t) * g_radar_seq_build_cnt);
        g_radar_seq_pending_cnt       = g_radar_seq_build_cnt;
        g_radar_seq_pending_valid     = 1;
        g_radar_seq_pending_step_us   = g_radar_seq_step_interval_us;
        g_radar_seq_pending_is_static = 0;
    }
}

void app_radar_set_install_height_mm(s32 height_mm)
{
    if (height_mm < 500)
    {
        height_mm = 500;
    }
    else if (height_mm > 10000)
    {
        height_mm = 10000;
    }

    g_radar_install_height_mm  = height_mm;
    g_radar_install_height_set = 1;

    radar_install_height_flash_t stored;
    stored.magic      = RADAR_INSTALL_HEIGHT_FLASH_MAGIC;
    stored.height_mm  = g_radar_install_height_mm;
    stored.height_set = g_radar_install_height_set;
    stored.crc        = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));

    flash_erase_sector(RADAR_INSTALL_HEIGHT_FLASH_ADDR);
    flash_write_page(RADAR_INSTALL_HEIGHT_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
}

void app_radar_get_install_height_mm(s32 *height_mm)
{
    if (!height_mm)
    {
        return;
    }

    *height_mm = g_radar_install_height_mm;
}

void app_radar_get_boundary_quad_by_index(u8 index, s32 *x_mm, s32 *y_mm)
{
    if (!x_mm || !y_mm || index >= 4)
    {
        return;
    }

    *x_mm = g_radar_boundary_quad[index].x_mm;
    *y_mm = g_radar_boundary_quad[index].y_mm;
}

void app_radar_set_boundary_quad(s32 x_mm[4], s32 y_mm[4])
{
    if (!x_mm || !y_mm)
    {
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        g_radar_boundary_quad[i].x_mm = x_mm[i];
        g_radar_boundary_quad[i].y_mm = y_mm[i];
    }

    g_radar_boundary_configured = 1;
}

void app_radar_reset_boundary_default(void)
{
    for (int i = 0; i < 4; i++)
    {
        g_radar_boundary_quad[i] = g_radar_boundary_quad_default[i];
    }
    g_radar_boundary_configured = 0;
}

static s16 DecodeRadarSigned15(u8 low, u8 high)
{
    u16 raw = ((u16)high << 8) | low;
    s16 mag = (s16)(raw & 0x7FFF);

    if (raw & 0x8000)
    {
        return mag;
    }

    return (s16)(-mag);
}

static u32 RadarFastRand(void)
{
    radar_rand_seed = (radar_rand_seed * 1664525UL) + 1013904223UL;
    return radar_rand_seed;
}

static s32 RadarRandRangeI32(s32 min_v, s32 max_v)
{
    u32 span = (u32)(max_v - min_v + 1);
    return min_v + (s32)(RadarFastRand() % span);
}

/* 判断点是否在当前凸四边形内部（边顺序需保持一致：顺时针或逆时针） */
static u8 RadarPointInsideQuad(s32 x_mm, s32 y_mm)
{
    int sign = 0;

    for (int i = 0; i < 4; i++)
    {
        const radar_boundary_point_t *a = &g_radar_boundary_quad[i];
        const radar_boundary_point_t *b = &g_radar_boundary_quad[(i + 1) % 4];

        s32 edge_x = b->x_mm - a->x_mm;
        s32 edge_y = b->y_mm - a->y_mm;
        s32 px     = x_mm - a->x_mm;
        s32 py     = y_mm - a->y_mm;

        s64 cross = (s64)edge_x * (s64)py - (s64)edge_y * (s64)px;

        if (cross == 0)
        {
            continue;
        }

        int curSign = (cross > 0) ? 1 : -1;
        if (sign == 0)
        {
            sign = curSign;
        }
        else if (curSign != sign)
        {
            return 0;
        }
    }

    return 1;
}

static float RadarAtan2Safe(float y, float x)
{
    return lookup_atan2(y, x);
}

static float RadarWrapAngle(float a)
{
    while (a > 3.1415926f)
        a -= 6.2831852f;
    while (a < -3.1415926f)
        a += 6.2831852f;
    return a;
}

typedef struct
{
    float x;
    float y;
} radar_vec2f_t;

static radar_vec2f_t RadarVec2(float x, float y)
{
    radar_vec2f_t v;
    v.x = x;
    v.y = y;
    return v;
}

static radar_vec2f_t RadarVecSub(radar_vec2f_t a, radar_vec2f_t b)
{
    return RadarVec2(a.x - b.x, a.y - b.y);
}

static radar_vec2f_t RadarVecAdd(radar_vec2f_t a, radar_vec2f_t b)
{
    return RadarVec2(a.x + b.x, a.y + b.y);
}

static radar_vec2f_t RadarVecMul(radar_vec2f_t a, float s)
{
    return RadarVec2(a.x * s, a.y * s);
}

static float RadarDot(radar_vec2f_t a, radar_vec2f_t b)
{
    return a.x * b.x + a.y * b.y;
}

static float RadarCross(radar_vec2f_t a, radar_vec2f_t b)
{
    return a.x * b.y - a.y * b.x;
}

static radar_vec2f_t RadarClosestPointOnSegment(radar_vec2f_t p, radar_vec2f_t a, radar_vec2f_t b)
{
    radar_vec2f_t ab    = RadarVecSub(b, a);
    float         denom = RadarDot(ab, ab);
    if (denom <= 0.000001f)
    {
        return a;
    }

    float t = RadarDot(RadarVecSub(p, a), ab) / denom;
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    else if (t > 1.0f)
    {
        t = 1.0f;
    }
    return RadarVecAdd(a, RadarVecMul(ab, t));
}

static float RadarLen(radar_vec2f_t a)
{
    return app_radar_mysqrt_3(a.x * a.x + a.y * a.y);
}

static radar_vec2f_t RadarNormalize(radar_vec2f_t v)
{
    float l = RadarLen(v);
    if (l <= 0.000001f)
    {
        return RadarVec2(0.0f, 0.0f);
    }
    return RadarVec2(v.x / l, v.y / l);
}

static radar_vec2f_t RadarReflect(radar_vec2f_t v_unit, radar_vec2f_t edge_unit)
{
    // reflect around edge line: v' = v - 2*(v·n)*n, n = perp(edge)
    radar_vec2f_t n = RadarVec2(-edge_unit.y, edge_unit.x);
    n               = RadarNormalize(n);
    float vn        = RadarDot(v_unit, n);
    return RadarVec2(v_unit.x - 2.0f * vn * n.x, v_unit.y - 2.0f * vn * n.y);
}

static u8 RadarSegmentIntersect(radar_vec2f_t p,
                                radar_vec2f_t p2,
                                radar_vec2f_t q,
                                radar_vec2f_t q2,
                                float        *out_t,
                                float        *out_u)
{
    // Solve p + t*r = q + u*s for t,u in [0,1]
    radar_vec2f_t r   = RadarVecSub(p2, p);
    radar_vec2f_t s   = RadarVecSub(q2, q);
    float         rxs = RadarCross(r, s);
    radar_vec2f_t qmp = RadarVecSub(q, p);

    if (rxs > -0.000001f && rxs < 0.000001f)
    {
        return 0;  // parallel (ignore colinear for our use)
    }

    float t = RadarCross(qmp, s) / rxs;
    float u = RadarCross(qmp, r) / rxs;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f)
    {
        *out_t = t;
        *out_u = u;
        return 1;
    }

    return 0;
}

/* 原始雷达点无方向信息：若在四边形外，则投影到最近边（用于“拉回到合法区域”） */
static void RadarProjectToQuad(s32 x_mm, s32 y_mm, s16 *out_x_mm, s16 *out_y_mm, float *out_dist_mm)
{
    if (RadarPointInsideQuad(x_mm, y_mm))
    {
        *out_x_mm    = (s16)x_mm;
        *out_y_mm    = (s16)y_mm;
        *out_dist_mm = 0.0f;
        return;
    }

    float bestDist2 = -1.0f;
    float bestX     = (float)x_mm;
    float bestY     = (float)y_mm;

    for (int i = 0; i < 4; i++)
    {
        float ax = (float)g_radar_boundary_quad[i].x_mm;
        float ay = (float)g_radar_boundary_quad[i].y_mm;
        float bx = (float)g_radar_boundary_quad[(i + 1) % 4].x_mm;
        float by = (float)g_radar_boundary_quad[(i + 1) % 4].y_mm;

        float vx = bx - ax;
        float vy = by - ay;
        float wx = (float)x_mm - ax;
        float wy = (float)y_mm - ay;

        float denom = vx * vx + vy * vy;
        if (denom <= 0.0f)
        {
            continue;
        }

        float t = (vx * wx + vy * wy) / denom;
        if (t < 0.0f)
        {
            t = 0.0f;
        }
        else if (t > 1.0f)
        {
            t = 1.0f;
        }

        float projX = ax + t * vx;
        float projY = ay + t * vy;
        float dx    = projX - (float)x_mm;
        float dy    = projY - (float)y_mm;
        float dist2 = dx * dx + dy * dy;

        if (bestDist2 < 0.0f || dist2 < bestDist2)
        {
            bestDist2 = dist2;
            bestX     = projX;
            bestY     = projY;
        }
    }

    *out_x_mm    = (s16)bestX;
    *out_y_mm    = (s16)bestY;
    *out_dist_mm = (bestDist2 < 0.0f) ? 0.0f : app_radar_mysqrt_3(bestDist2);
}

/* 点在凸四边形外：取距离最近的边上的垂足 F，作 P' = 2F - P（沿垂线再伸出 |PF|）；若 P' 仍在外则回退为投影入域 */
static void RadarMirrorOutsideAcrossNearestEdge(s16 *inout_x, s16 *inout_y)
{
    s32 x = (s32)*inout_x;
    s32 y = (s32)*inout_y;

    if (RadarPointInsideQuad(x, y))
    {
        return;
    }

    float bestDist2 = -1.0f;
    float bestFx    = (float)x;
    float bestFy    = (float)y;

    for (int i = 0; i < 4; i++)
    {
        float ax = (float)g_radar_boundary_quad[i].x_mm;
        float ay = (float)g_radar_boundary_quad[i].y_mm;
        float bx = (float)g_radar_boundary_quad[(i + 1) % 4].x_mm;
        float by = (float)g_radar_boundary_quad[(i + 1) % 4].y_mm;

        float vx = bx - ax;
        float vy = by - ay;
        float wx = (float)x - ax;
        float wy = (float)y - ay;

        float denom = vx * vx + vy * vy;
        if (denom <= 0.0f)
        {
            continue;
        }

        float t = (vx * wx + vy * wy) / denom;
        if (t < 0.0f)
        {
            t = 0.0f;
        }
        else if (t > 1.0f)
        {
            t = 1.0f;
        }

        float projX = ax + t * vx;
        float projY = ay + t * vy;
        float dx    = projX - (float)x;
        float dy    = projY - (float)y;
        float dist2 = dx * dx + dy * dy;

        if (bestDist2 < 0.0f || dist2 < bestDist2)
        {
            bestDist2 = dist2;
            bestFx    = projX;
            bestFy    = projY;
        }
    }

    if (bestDist2 < 0.0f)
    {
        float pd;
        RadarProjectToQuad(x, y, inout_x, inout_y, &pd);
        return;
    }

    {
        float mx = 2.0f * bestFx - (float)x;
        float my = 2.0f * bestFy - (float)y;

        if (!RadarPointInsideQuad((s32)mx, (s32)my))
        {
            float pd;
            RadarProjectToQuad((s32)mx, (s32)my, inout_x, inout_y, &pd);
        }
        else
        {
            if (mx > 32767.0f)
            {
                mx = 32767.0f;
            }
            if (mx < -32768.0f)
            {
                mx = -32768.0f;
            }
            if (my > 32767.0f)
            {
                my = 32767.0f;
            }
            if (my < -32768.0f)
            {
                my = -32768.0f;
            }
            *inout_x = (s16)mx;
            *inout_y = (s16)my;
        }
    }
}

/* 原始雷达点外部处理策略 */
#define RADAR_RAW_OUTSIDE_DROP_DIST_MM 800.0f
#define RADAR_RAW_OUTSIDE_DROP_LIMIT   3
static _attribute_data_retention_ u8 g_radar_raw_outside_drop_cnt = 0;

/* 从 start 沿向量 v 走一步；若撞到边界则按撞到的那条边反射，并返回反射后的点与方向 */
static void RadarAdvanceReflectQuad(s32           start_x_mm,
                                    s32           start_y_mm,
                                    radar_vec2f_t v_step,
                                    s16          *out_x_mm,
                                    s16          *out_y_mm,
                                    float        *inout_dir_rad)
{
    radar_vec2f_t start = RadarVec2((float)start_x_mm, (float)start_y_mm);
    radar_vec2f_t end   = RadarVecAdd(start, v_step);

    // Fast path: end is inside
    if (RadarPointInsideQuad((s32)end.x, (s32)end.y))
    {
        *out_x_mm = (s16)end.x;
        *out_y_mm = (s16)end.y;
        return;
    }

    // Bounce loop: allow multiple reflections in one step (rare, but makes it robust)
    radar_vec2f_t cur_start = start;
    radar_vec2f_t cur_end   = end;
    radar_vec2f_t v_remain  = v_step;

    for (int bounce = 0; bounce < 4; bounce++)
    {
        float best_t    = 2.0f;
        int   best_edge = -1;
        float best_u    = 0.0f;

        // Find first intersection along segment
        for (int i = 0; i < 4; i++)
        {
            radar_vec2f_t a = RadarVec2((float)g_radar_boundary_quad[i].x_mm, (float)g_radar_boundary_quad[i].y_mm);
            radar_vec2f_t b = RadarVec2((float)g_radar_boundary_quad[(i + 1) % 4].x_mm, (float)g_radar_boundary_quad[(i + 1) % 4].y_mm);

            float t_seg = 0.0f;
            float u_seg = 0.0f;
            if (RadarSegmentIntersect(cur_start, cur_end, a, b, &t_seg, &u_seg))
            {
                // avoid choosing the segment start due to numeric issues
                if (t_seg > 0.00001f && t_seg < best_t)
                {
                    best_t    = t_seg;
                    best_u    = u_seg;
                    best_edge = i;
                }
            }
        }

        if (best_edge < 0 || best_t > 1.0f)
        {
            // Fallback: no exact intersection (numeric/degenerated case).
            // Choose nearest boundary edge and compute reflected prediction from it.
            int           nearest_edge = 0;
            float         best_dist2   = -1.0f;
            radar_vec2f_t nearest_proj = cur_end;

            for (int i = 0; i < 4; i++)
            {
                radar_vec2f_t a  = RadarVec2((float)g_radar_boundary_quad[i].x_mm, (float)g_radar_boundary_quad[i].y_mm);
                radar_vec2f_t b  = RadarVec2((float)g_radar_boundary_quad[(i + 1) % 4].x_mm, (float)g_radar_boundary_quad[(i + 1) % 4].y_mm);
                radar_vec2f_t q  = RadarClosestPointOnSegment(cur_end, a, b);
                radar_vec2f_t d  = RadarVecSub(q, cur_end);
                float         d2 = RadarDot(d, d);

                if (best_dist2 < 0.0f || d2 < best_dist2)
                {
                    best_dist2   = d2;
                    nearest_edge = i;
                    nearest_proj = q;
                }
            }

            radar_vec2f_t edge_a    = RadarVec2((float)g_radar_boundary_quad[nearest_edge].x_mm, (float)g_radar_boundary_quad[nearest_edge].y_mm);
            radar_vec2f_t edge_b    = RadarVec2((float)g_radar_boundary_quad[(nearest_edge + 1) % 4].x_mm, (float)g_radar_boundary_quad[(nearest_edge + 1) % 4].y_mm);
            radar_vec2f_t edge_unit = RadarNormalize(RadarVecSub(edge_b, edge_a));
            radar_vec2f_t seg_vec   = RadarVecSub(cur_end, cur_start);
            float         seg_len   = RadarLen(seg_vec);
            radar_vec2f_t v_unit    = RadarNormalize(seg_vec);
            radar_vec2f_t v_ref_u   = RadarNormalize(RadarReflect(v_unit, edge_unit));
            radar_vec2f_t out_pt;

            if (v_ref_u.x == 0.0f && v_ref_u.y == 0.0f)
            {
                v_ref_u = edge_unit;
            }

            *inout_dir_rad = RadarWrapAngle(RadarAtan2Safe(v_ref_u.x, v_ref_u.y));
            out_pt         = RadarVecAdd(nearest_proj, RadarVecMul(v_ref_u, seg_len));

            s16 px = (s16)out_pt.x;
            s16 py = (s16)out_pt.y;
            if (!RadarPointInsideQuad((s32)px, (s32)py))
            {
                float proj_dist = 0.0f;
                RadarProjectToQuad((s32)px, (s32)py, &px, &py, &proj_dist);
            }
            *out_x_mm = px;
            *out_y_mm = py;
            return;
        }

        // Intersection point
        radar_vec2f_t seg_vec = RadarVecSub(cur_end, cur_start);
        radar_vec2f_t hit     = RadarVecAdd(cur_start, RadarVecMul(seg_vec, best_t));

        // Remaining distance after hit
        float total_len  = RadarLen(seg_vec);
        float remain_len = (1.0f - best_t) * total_len;

        // Reflect direction based on hit edge
        radar_vec2f_t edge_a    = RadarVec2((float)g_radar_boundary_quad[best_edge].x_mm, (float)g_radar_boundary_quad[best_edge].y_mm);
        radar_vec2f_t edge_b    = RadarVec2((float)g_radar_boundary_quad[(best_edge + 1) % 4].x_mm, (float)g_radar_boundary_quad[(best_edge + 1) % 4].y_mm);
        radar_vec2f_t edge_unit = RadarNormalize(RadarVecSub(edge_b, edge_a));

        radar_vec2f_t v_unit  = RadarNormalize(seg_vec);
        radar_vec2f_t v_ref_u = RadarNormalize(RadarReflect(v_unit, edge_unit));

        // Update direction angle for subsequent predictions
        *inout_dir_rad = RadarWrapAngle(RadarAtan2Safe(v_ref_u.x, v_ref_u.y));

        // New end after reflection
        cur_start = hit;
        cur_end   = RadarVecAdd(hit, RadarVecMul(v_ref_u, remain_len));
        v_remain  = RadarVecSub(cur_end, cur_start);

        if (RadarPointInsideQuad((s32)cur_end.x, (s32)cur_end.y))
        {
            *out_x_mm = (s16)cur_end.x;
            *out_y_mm = (s16)cur_end.y;
            return;
        }
    }

    // If still outside after multiple bounces, just output current end (should be very rare)
    *out_x_mm = (s16)cur_end.x;
    *out_y_mm = (s16)cur_end.y;
}

static void RadarSeqBuildStationary2Points(s16 ax_mm, s16 ay_mm, s16 bx_mm, s16 by_mm)
{
    RadarSeqBuildAppendPoint(ax_mm, ay_mm);
    RadarSeqBuildAppendPoint(bx_mm, by_mm);
    app_ctrl_radar_dbg_send_pred_sta(ax_mm, ay_mm, bx_mm, by_mm);
}

static void RadarSeqBuildStationaryEscapePoints(s16 start_x_mm, s16 start_y_mm, float start_dir_rad, u8 point_count)
{
    s16   sx      = start_x_mm;
    s16   sy      = start_y_mm;
    float seq_dir = start_dir_rad;

    for (u8 i = 0; i < point_count; i++)
    {
        s32 dist = RadarRandRangeI32(100, 300);
        s32 nx32 = (s32)sx + (s32)((float)dist * lookup_sin(seq_dir));
        s32 ny32 = (s32)sy + (s32)((float)dist * lookup_cos(seq_dir));

        radar_vec2f_t v_step = RadarVec2((float)(nx32 - sx), (float)(ny32 - sy));
        RadarAdvanceReflectQuad((s32)sx, (s32)sy, v_step, &sx, &sy, &seq_dir);
        if (!RadarPointInsideQuad((s32)sx, (s32)sy))
        {
            float proj_dist = 0.0f;
            RadarProjectToQuad((s32)sx, (s32)sy, &sx, &sy, &proj_dist);
        }
        RadarSeqBuildAppendPoint(sx, sy);
        app_ctrl_radar_dbg_send_predseq((u8)(i + 1), sx, sy);
    }
}

void app_radar_gimbal_track_task(void)
{
    StepMotor_GimbalTask();
    switch (g_radar_gimbal_fsm)
    {
    case RADAR_GIMBAL_FSM_IDLE:
        break;

    case RADAR_GIMBAL_FSM_RUN:
        if (g_radar_seq_active_idx >= g_radar_seq_active_cnt)
        {
            g_radar_gimbal_fsm = RADAR_GIMBAL_FSM_WAIT_DONE;
            break;
        }

        if (!clock_time_exceed(g_radar_seq_step_tick, g_radar_seq_step_interval_us))
        {
            break;
        }

        g_radar_seq_step_tick = clock_time();
#if (UI_STEP_MOTOR_ENABLE)
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, g_radar_seq_active[g_radar_seq_active_idx].pan_deg10);
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, g_radar_seq_active[g_radar_seq_active_idx].tilt_deg10);
#endif
        // LOG_D("PAN,%d,TILT,%d,idx,%d",
        //       g_radar_seq_active[g_radar_seq_active_idx].pan_deg10,
        //       g_radar_seq_active[g_radar_seq_active_idx].tilt_deg10,
        //       g_radar_seq_active_idx);
        g_radar_seq_active_idx++;
        break;

    case RADAR_GIMBAL_FSM_WAIT_DONE:
        if (g_radar_seq_pending_valid && g_radar_seq_pending_cnt)
        {
            u8 pending_static = g_radar_seq_pending_is_static;
            // LOG_D("RADAR_GIMBAL_FSM_WAIT_DONE, %d", g_radar_seq_pending_cnt);
            memcpy(g_radar_seq_active, g_radar_seq_pending, sizeof(radar_gimbal_point_t) * g_radar_seq_pending_cnt);
            g_radar_seq_active_cnt        = g_radar_seq_pending_cnt;
            g_radar_seq_active_idx        = 0;
            g_radar_seq_pending_cnt       = 0;
            g_radar_seq_pending_valid     = 0;
            g_radar_seq_pending_is_static = 0;
            g_radar_seq_step_interval_us  = g_radar_seq_pending_step_us;
            g_radar_seq_step_tick         = clock_time();
            g_radar_busy_max_speed_abs    = 0;
            g_radar_seq_active_is_static  = pending_static;
            g_radar_gimbal_fsm            = RADAR_GIMBAL_FSM_RUN;
            if (g_radar_seq_active_cnt == 2)
            {
                StepMotor_GimbalSetSpeedUs(1600);
            }
            if (g_radar_seq_active_cnt == 3)
            {
                StepMotor_GimbalSetSpeedUs(1200);
            }
            if (g_radar_seq_active_cnt == 6)
            {
                StepMotor_GimbalSetSpeedUs(800);
            }
        }
        else if (g_radar_static_loop_en)
        {
            RadarSeqBuildReset();
            if (g_radar_static_next_is_escape)
            {
                RadarSeqBuildStationaryEscapePoints(g_radar_static_bx_mm, g_radar_static_by_mm, g_radar_static_dir_rad, 2);
                g_radar_static_next_is_escape = 0;
            }
            else
            {
                RadarSeqBuildStationary2Points(g_radar_static_ax_mm, g_radar_static_ay_mm, g_radar_static_bx_mm, g_radar_static_by_mm);
                g_radar_static_next_is_escape = 1;
            }
            g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_SLOW;
            RadarSeqStartBuilt();
            g_radar_seq_active_is_static = 1;
        }
        else
        {
            // LOG_D("RADAR_GIMBAL_FSM_WAIT_DONE,0");
            g_radar_seq_active_cnt       = 0;
            g_radar_seq_active_idx       = 0;
            g_radar_busy_max_speed_abs   = 0;
            g_radar_seq_active_is_static = 0;
            g_radar_gimbal_fsm           = RADAR_GIMBAL_FSM_IDLE;
        }
        break;

    default:
        g_radar_gimbal_fsm = RADAR_GIMBAL_FSM_IDLE;
        break;
    }
}

static void BuildPredictionPoint(s16   cur_x_mm,
                                 s16   cur_y_mm,
                                 s16   dx_mm,
                                 s16   dy_mm,
                                 float dir_rad,
                                 s32   distance_mm,
                                 s32   angle_offset_deg,
                                 s16  *out_x_mm,
                                 s16  *out_y_mm)
{
    float angle_rad = dir_rad + ((float)angle_offset_deg * DEG_TO_RAD);
    s32   local_x   = (s32)((float)distance_mm * lookup_sin(angle_rad));
    s32   local_y   = (s32)((float)distance_mm * lookup_cos(angle_rad));

    s32 scale_q8 = RadarRandRangeI32(384, 768);
    s32 pred_x   = (s32)cur_x_mm + ((s32)dx_mm * scale_q8 >> 8) + local_x;
    s32 pred_y   = (s32)cur_y_mm + ((s32)dy_mm * scale_q8 >> 8) + local_y;

    // 边界处理：从 cur -> pred 的线段若越界，则按撞到的边做镜面反射并得到最终点
    radar_vec2f_t v_step    = RadarVec2((float)(pred_x - cur_x_mm), (float)(pred_y - cur_y_mm));
    float         dummy_dir = dir_rad;
    RadarAdvanceReflectQuad((s32)cur_x_mm, (s32)cur_y_mm, v_step, out_x_mm, out_y_mm, &dummy_dir);

    // 保险：若仍在边界外，拉回到最近边界点
    if (!RadarPointInsideQuad((s32)*out_x_mm, (s32)*out_y_mm))
    {
        float proj_dist = 0.0f;
        RadarProjectToQuad((s32)*out_x_mm, (s32)*out_y_mm, out_x_mm, out_y_mm, &proj_dist);
    }
}

static void ReportPredictionSerialized(u32 now_tick, s16 x_mm, s16 y_mm, s16 v_cm_s)
{
    s16   proc_x        = x_mm;
    s16   proc_y        = y_mm;
    s16   dx_mm         = 0;
    s16   dy_mm         = 0;
    float motion_rad    = 0.0f;
    u8    motion_valid  = 0;
    u8    is_stationary = 1;

    if (x_mm == 0 && y_mm == 0)
    {
        x_mm = g_radar_pred.prev_x_mm;
        y_mm = g_radar_pred.prev_y_mm;
    }
    RadarMotionCachePush(now_tick, x_mm, y_mm);

    if (g_radar_pred.has_prev)
    {
        dx_mm = (s16)(x_mm - g_radar_pred.prev_x_mm);
        dy_mm = (s16)(y_mm - g_radar_pred.prev_y_mm);

        // Prefer direction derived from the 1s window edge points (oldest & newest)
        if (g_radar_pred.motion_cache_count >= 2)
        {
            u8  oldest = RadarMotionCacheIndexOldest();
            u8  newest = RadarMotionCacheIndexNewest();
            s16 dx_w   = (s16)(g_radar_motion_cache[newest].x_mm - g_radar_motion_cache[oldest].x_mm);
            s16 dy_w   = (s16)(g_radar_motion_cache[newest].y_mm - g_radar_motion_cache[oldest].y_mm);
            if ((abs(dx_w) > STATIONARY_DXY_THRESHOLD_MM) || (abs(dy_w) > STATIONARY_DXY_THRESHOLD_MM))
            {
                motion_rad   = lookup_atan2((float)dx_w, (float)dy_w);
                motion_valid = 1;
            }
        }

        if ((abs(dx_mm) > STATIONARY_DXY_THRESHOLD_MM) || (abs(dy_mm) > STATIONARY_DXY_THRESHOLD_MM))
        {
            is_stationary = 0;
            if (!motion_valid && (dx_mm || dy_mm))
            {
                motion_rad   = lookup_atan2((float)dx_mm, (float)dy_mm);
                motion_valid = 1;
            }
            if (motion_valid)
            {
                // Update sticky direction only while moving.
                g_radar_pred.last_motion_dir_rad = motion_rad;
                g_radar_pred.has_last_motion_dir = 1;
            }
        }
        else
        {
            // Keep last moving direction during stationary period.
            if (g_radar_pred.has_last_motion_dir)
            {
                motion_rad   = g_radar_pred.last_motion_dir_rad;
                motion_valid = 1;
            }
            else if (!motion_valid && (dx_mm || dy_mm))
            {
                motion_rad   = lookup_atan2((float)dx_mm, (float)dy_mm);
                motion_valid = 1;
            }
        }
    }

    {
        s16 motion_dir_deg10 = 0;
        if (motion_valid)
        {
            float d = motion_rad * (float)RAD_TO_DEG * 10.0f;
            if (d > 32767.0f)
            {
                d = 32767.0f;
            }
            else if (d < -32768.0f)
            {
                d = -32768.0f;
            }
            motion_dir_deg10 = (s16)d;
        }
        app_ctrl_radar_dbg_send_prev_raw(
            g_radar_pred.prev_x_mm, g_radar_pred.prev_y_mm, x_mm, y_mm, motion_valid, motion_dir_deg10);
    }
    g_radar_pred.prev_x_mm = x_mm;
    g_radar_pred.prev_y_mm = y_mm;
    g_radar_pred.has_prev  = 1;

    RadarSeqBuildReset();
    if (!RadarPointInsideQuad((s32)proc_x, (s32)proc_y))
    {
        RadarMirrorOutsideAcrossNearestEdge(&proc_x, &proc_y);
    }

#if RADAR_GIMBAL_DEBUG_TRACK_PROC_ONLY
    /* 调试：每帧只提交单点 (proc_x, proc_y)，不走静止/运动预测分支，用于观察电机对处理坐标的跟踪 */
    g_radar_static_loop_en        = 0;
    g_radar_static_next_is_escape = 0;
    RadarSeqBuildAppendPoint(proc_x, proc_y);
    g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_FAST;
    RadarSeqCommitBuiltMotion();
    return;
#else
    /* 以下内容：预测点序列控制电机 */

    if (is_stationary)
    {
        u8  was_static_loop = g_radar_static_loop_en;
        s16 ax, ay;
        s16 bx, by;

        BuildPredictionPoint(proc_x, proc_y, dx_mm, dy_mm, motion_rad, 300, 0, &ax, &ay);
        BuildPredictionPoint(proc_x, proc_y, dx_mm, dy_mm, motion_rad, RadarRandRangeI32(500, 500), RadarRandRangeI32(-20, 20), &bx, &by);
        g_radar_static_ax_mm   = ax;
        g_radar_static_ay_mm   = ay;
        g_radar_static_bx_mm   = bx;
        g_radar_static_by_mm   = by;
        g_radar_static_dir_rad = motion_rad;
        g_radar_static_loop_en = 1;

        if (!was_static_loop)
        {
            g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_SLOW;
            if (g_radar_static_next_is_escape)
            {
                RadarSeqBuildStationaryEscapePoints(g_radar_static_bx_mm, g_radar_static_by_mm, g_radar_static_dir_rad, 2);
                g_radar_static_next_is_escape = 0;
            }
            else
            {
                RadarSeqBuildStationary2Points(g_radar_static_ax_mm, g_radar_static_ay_mm, g_radar_static_bx_mm, g_radar_static_by_mm);
                g_radar_static_next_is_escape = 1;
            }
            RadarSeqCommitBuiltStatic();
        }
    }
    else
    {
        g_radar_static_loop_en = 0;
        s16   px, py;
        s16   sx;
        s16   sy;
        float seq_dir;
        s16   speed_abs;
        u8    seq_num;

        BuildPredictionPoint(proc_x, proc_y, dx_mm, dy_mm, motion_rad, RadarRandRangeI32(300, 300), RadarRandRangeI32(0, 0), &px, &py);

        sx        = px;
        sy        = py;
        seq_dir   = motion_rad;
        speed_abs = RadarGetSpeedForPrediction(v_cm_s);

        /* 静止或 v<0.5m/s：2 点 / 0.4s；0.5~1m/s：3 点 / 0.2s；≥1m/s：6 点 / 0.1s */
        if (speed_abs < RADAR_SPEED_LOW_CM_S)
        {
            seq_num                      = 2;
            g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_SLOW;
        }
        else if (speed_abs < RADAR_SPEED_MED_CM_S)
        {
            seq_num                      = 3;
            g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_MED;
        }
        else
        {
            seq_num                      = 6;
            g_radar_seq_step_interval_us = RADAR_GIMBAL_STEP_US_FAST;
        }

        for (u8 i = 0; i < seq_num; i++)
        {
            s32 dist = RadarRandRangeI32(300, 300);
            s32 nx32 = (s32)sx + (s32)((float)dist * lookup_sin(seq_dir));
            s32 ny32 = (s32)sy + (s32)((float)dist * lookup_cos(seq_dir));

            radar_vec2f_t v_step = RadarVec2((float)(nx32 - sx), (float)(ny32 - sy));
            RadarAdvanceReflectQuad((s32)sx, (s32)sy, v_step, &sx, &sy, &seq_dir);
            // 保险：若仍在边界外，拉回到最近边界点
            if (!RadarPointInsideQuad((s32)sx, (s32)sy))
            {
                float proj_dist = 0.0f;
                RadarProjectToQuad((s32)sx, (s32)sy, &sx, &sy, &proj_dist);
            }
            RadarSeqBuildAppendPoint(sx, sy);
            app_ctrl_radar_dbg_send_predseq((u8)(i + 1), sx, sy);
        }
        RadarSeqCommitBuiltMotion();
    }
#endif /* !RADAR_GIMBAL_DEBUG_TRACK_PROC_ONLY */
}

#define RADAR_FPS_TEST 0
#if RADAR_FPS_TEST
static u32 radar_start_time = 0;
#endif

void app_radar_parse_and_report_frame(void)
{
    if (!g_uart_ndma_rx_flag)
    {
        return;
    }
    g_uart_ndma_rx_flag = 0;
    // LOG_D("app_radar_parse_and_report_frame");

    if ((g_uart_ndma_rx_byte[0] != 0xAAU) || (g_uart_ndma_rx_byte[1] != 0xFFU) || (g_uart_ndma_rx_byte[2] != 0x03U) ||
        (g_uart_ndma_rx_byte[3] != 0x00U) || (g_uart_ndma_rx_byte[28] != 0x55U) || (g_uart_ndma_rx_byte[29] != 0xCCU))
    {
        LOG_D("radar frame invalid");
        return;
    }

    {
        u8  base  = 4;
        s16 x_mm  = DecodeRadarSigned15(g_uart_ndma_rx_byte[base + 0], g_uart_ndma_rx_byte[base + 1]);
        s16 y_mm  = DecodeRadarSigned15(g_uart_ndma_rx_byte[base + 2], g_uart_ndma_rx_byte[base + 3]);
        s16 v_cms = DecodeRadarSigned15(g_uart_ndma_rx_byte[base + 4], g_uart_ndma_rx_byte[base + 5]);

        radar_x_sum += x_mm;
        radar_y_sum += y_mm;
        radar_v_sum += v_cms;
        radar_sample_cnt++;

        if (radar_sample_cnt >= SAMPLE_COUNT)
        {
            // NOTE: preserves original behavior (uses bitshift, assumes SAMPLE_COUNT matches that logic)
            s16   x_avg      = (s16)(radar_x_sum >> (SAMPLE_COUNT >> 1));
            s16   y_avg      = (s16)(radar_y_sum >> (SAMPLE_COUNT >> 1));
            s16   v_avg      = (s16)(radar_v_sum >> (SAMPLE_COUNT >> 1));
            float proj_dist  = 0.0f;
            s16   x_in       = x_avg;
            s16   y_in       = y_avg;
            radar_x_sum      = 0;
            radar_y_sum      = 0;
            radar_v_sum      = 0;
            radar_sample_cnt = 0;

            // LOG_D("radar, x=%dmm y=%dmm v=%dcm/s", x_avg, y_avg, v_avg);
            u32 now_tick = clock_time();
            if (RadarSpeedAbs(v_avg) > 5)
            {
                RadarSessionOnMotion(now_tick);
            }

            if (RadarSessionCanPredict(now_tick))
            {
                ReportPredictionSerialized(now_tick, x_in, y_in, v_avg);
            }

#if RADAR_FPS_TEST
            u32 current_time = clock_time() >> 4;
            if (radar_start_time != 0)
            {
                u32 dt = (current_time - radar_start_time);
                if (dt > 0)
                {
                    u32 fps = 1000000 / dt;
                    LOG_D("radar fps: %d", fps);
                }
            }
            radar_start_time = current_time;
#endif
        }
    }
}

void app_radar_uart_init(void)
{
    uart_gpio_set(GPIO_PB6, GPIO_PB7);
    uart_init_baudrate(256000, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_dma_enable(0, 0);
    uart_ndma_clear_rx_index();
    uart_ndma_clear_tx_index();
    uart_ndma_irq_triglevel(1, 0);
    uart_irq_enable(1, 0);
}

void app_radar_uart_ndma_irq_proc(void)
{
    if (!uart_ndmairq_get())
    {
        return;
    }

    unsigned char rx_cnt = reg_uart_buf_cnt & 0x0f;

    if (g_uart_ndma_rx_flag)
    {
        while (rx_cnt--)
        {
            (void)uart_ndma_read_byte();
        }
        return;
    }

    while (rx_cnt--)
    {
        u8 data = uart_ndma_read_byte();
        if (g_uart_ndma_rx_byte_cnt >= sizeof(g_uart_ndma_rx_byte))
        {
            g_uart_ndma_rx_byte_cnt = 0;
        }
        g_uart_ndma_rx_byte[g_uart_ndma_rx_byte_cnt++] = data;

        if (g_uart_ndma_rx_byte_cnt >= 2 &&
            g_uart_ndma_rx_byte[g_uart_ndma_rx_byte_cnt - 2] == 0x55 &&
            g_uart_ndma_rx_byte[g_uart_ndma_rx_byte_cnt - 1] == 0xCC)
        {
            if (g_uart_ndma_rx_byte_cnt == 30 &&
                g_uart_ndma_rx_byte[0] == 0xAA &&
                g_uart_ndma_rx_byte[1] == 0xFF &&
                g_uart_ndma_rx_byte[2] == 0x03 &&
                g_uart_ndma_rx_byte[3] == 0x00)
            {
                g_uart_ndma_rx_flag = 1;
            }
            g_uart_ndma_rx_byte_cnt = 0;
        }
    }
}

#endif /* UI_RADAR_ENABLE */
