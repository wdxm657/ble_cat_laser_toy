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

/** 置 0 可关闭 ISR 计数与周期日志，发布前建议关闭。 */
#ifndef RADAR_RX_IRQ_DEBUG
#define RADAR_RX_IRQ_DEBUG 0
#endif

#define RADAR_INSTALL_HEIGHT_DEFAULT_MM   (1.8f * 1000.0f)
#define RADAR_FRAME_LEN                   30
#define SAMPLE_COUNT                      1
#define STATIONARY_DXY_THRESHOLD_MM       5.0
#define RADAR_IDLE_TIMEOUT_US             (1000000u * 30u)  // 30s
/** 与帧解析中「目标在移动」判定一致 (cm/s) */
#define RADAR_TARGET_MOVING_SPEED_THR_CMS 5

#define RADAR_TIME_SEC_PER_DAY            86400u

#define RADAR_PLAY_STATE_IDLE             0
#define RADAR_PLAY_STATE_ACTIVE           1
#define RADAR_PLAY_END_ONGOING            0xFFFFFFFFu

/* ========== 狩猎游戏状态机 ========== */
typedef enum
{
    HUNT_STATE_OFF = 0,          // 关机
    HUNT_STATE_IDLE,             // 逗宠等待 — radar on, 等待移动目标
    HUNT_STATE_ACTIVE,           // 狩猎中 — 计时+跟踪+镭射开启
    HUNT_STATE_STANDBY,          // 待机 — 15s无目标, 关闭激光+电机
    HUNT_STATE_PREY_ZONE_SLEEP,  // 30s休眠 — 目标在猎物点15s, 关闭雷达+激光+电机
    HUNT_STATE_COMPLETE_MOVING,  // 完成移动 — 移动光斑到猎物点
    HUNT_STATE_CELEBRATE,        // 停留10秒 — 在猎物点停留, 检查目标是否进入猎物点
    HUNT_STATE_SLEEP,            // 休眠 — 达成条件后休眠
} hunt_state_e;

#define HUNT_NO_TARGET_TIMEOUT_US   (15u * 1000000u)  // 15s 无目标 → 待机
#define HUNT_PREY_ZONE_RADIUS_MM    200               // 猎物点半径 20cm
#define HUNT_PREY_ZONE_TIMEOUT_US   (15u * 1000000u)  // 15s 目标在猎物点 → 30s休眠
#define HUNT_PREY_ZONE_SLEEP_DUR_US (30u * 1000000u)  // 30秒休眠
#define HUNT_CELEBRATION_DUR_US     (10u * 1000000u)  // 停留10秒

/* 狩猎配置与运行时变量 */
_attribute_data_retention_ static hunt_state_e g_hunt_state              = HUNT_STATE_OFF;
_attribute_data_retention_ static u16          g_hunt_duration_s         = 30;    // 单次狩猎时长(秒)
_attribute_data_retention_ static u8           g_hunt_count              = 3;     // 狩猎次数
_attribute_data_retention_ static u8           g_hunt_sleep_duration_min = 3;     // 休眠时长(分钟)
_attribute_data_retention_ static u8           g_hunt_completed          = 0;     // 已完成狩猎次数
_attribute_data_retention_ static u32          g_hunt_total_active_ms    = 0;     // 累计活跃狩猎时长(毫秒)
_attribute_data_retention_ static u32          g_hunt_session_tick       = 0;     // 当前狩猎开始tick
_attribute_data_retention_ static u32          g_hunt_session_acc_ms     = 0;     // 当前狩猎累计秒数
_attribute_data_retention_ static u32          g_hunt_acc_tick_last      = 0;     // 上次累计更新tick
_attribute_data_retention_ static u32          g_hunt_no_target_tick     = 0;     // 无目标计时开始tick
_attribute_data_retention_ static u32          g_hunt_prey_zone_tick     = 0;     // 目标在猎物点计时开始tick
_attribute_data_retention_ static u32          g_hunt_celebration_tick   = 0;     // 庆祝阶段开始tick
_attribute_data_retention_ static u32          g_hunt_sleep_end_tick     = 0;     // 休眠结束tick
_attribute_data_retention_ static u8           g_hunt_success            = 0;     // 当前狩猎是否成功
_attribute_data_retention_ static s16          g_prey_pan_deg10          = 0;     // 猎物点水平角 0°
_attribute_data_retention_ static s16          g_prey_tilt_deg10         = -700;  // 猎物点俯仰角 20°
/** 猎物点地面坐标缓存，仅在 g_prey_pan_deg10 / g_prey_tilt_deg10 变更时重算 */
_attribute_data_retention_ static s16 g_prey_px_mm    = 0;
_attribute_data_retention_ static s16 g_prey_py_mm    = 0;
_attribute_data_retention_ static u8  g_prey_xy_valid = 0;

/** pan/tilt 更新后标记缓存无效，下次 hunt_is_target_near_prey_point 时自动重算 px/py */
static inline void hunt_prey_point_invalidate_xy(void)
{
    g_prey_xy_valid = 0;
}

_attribute_data_retention_ volatile u8 g_uart_ndma_rx_byte[RADAR_FRAME_LEN];
_attribute_data_retention_ volatile u8 g_uart_ndma_rx_byte_cnt = 0;
_attribute_data_retention_ volatile u8 g_uart_ndma_rx_flag     = 0;

#if RADAR_RX_IRQ_DEBUG
volatile u32 g_radar_dbg_rx_drain_bytes      = 0;
volatile u32 g_radar_dbg_rx_drain_irq_cnt    = 0;
volatile u32 g_radar_dbg_rx_flag_set_cnt     = 0;
volatile u32 g_radar_dbg_rx_flag_cleared_cnt = 0;
volatile u32 g_radar_dbg_rx_uart_irq_cnt     = 0;
volatile u32 g_radar_dbg_rx_asm_bytes        = 0;
#endif

_attribute_data_retention_ static int32_t radar_x_sum      = 0;
_attribute_data_retention_ static int32_t radar_y_sum      = 0;
_attribute_data_retention_ static int32_t radar_v_sum      = 0;
_attribute_data_retention_ static u8      radar_sample_cnt = 0;
_attribute_data_retention_ static u32     radar_rand_seed  = 0x12345678UL;

typedef struct
{
    s16   prev_x_mm;
    s16   prev_y_mm;
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

/** 逗宠统计：轨迹点相对 newest 位移时累加 Δt 与 v×Δt（实现见后） */
static void radar_play_on_cache_displacement_ms(u32 dt_ms, u32 speed_cms_mag);

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

#define RADAR_GIMBAL_SEQ_MAX_POINTS            16

/* 单段移动耗时（相邻两点之间），单位与 clock_time_exceed 第二参数一致（微秒） */
#define RADAR_GIMBAL_STEP_US_SLOW              400000u /* 0.4 s */
#define RADAR_GIMBAL_STEP_US_MED               200000u /* 0.2 s */
#define RADAR_GIMBAL_STEP_US_FAST              100000u /* 0.1 s */

/*
 * 雷达直接跟踪：与 app_ctrl 边界移动（APP_CTRL_BOUNDARY_MOVE_SPEED_US）同量级步间隔，
 * 每帧只更新一个目标点，由 StepMotor_GimbalTask 连续逼近 —— 避免多点折线 + 分段等待造成的顿挫。
 */
#define RADAR_TRACK_GIMBAL_INTERVAL_DEFAULT_US 8000u

/*
 * 阶段二：沿估计运动方向在水平面内向前偏移 (mm)，使云台略超前目标。
 * 可在 app_config.h 中 #define RADAR_TRACK_LEAD_ENABLE 0 关闭。
 */
#ifndef RADAR_TRACK_LEAD_ENABLE
#define RADAR_TRACK_LEAD_ENABLE 1
#endif
/** 领跑距离 = BASE + |v_cm_s| * NUM / DEN (mm)，再夹到 MIN~MAX（与雷达 |v| 无关，低速也有 MIN 超前） */
#define RADAR_TRACK_LEAD_MM_BASE       600
#define RADAR_TRACK_LEAD_MM_PER_NUM    10
#define RADAR_TRACK_LEAD_MM_PER_DEN    1
#define RADAR_TRACK_LEAD_MM_MIN        200
#define RADAR_TRACK_LEAD_MM_MAX        2000
/** 目标判定静止时，沿最后运动方向保留的较小超前 (mm)，需 motion_valid（含粘滞方向） */
#define RADAR_TRACK_LEAD_MM_STATIC     600
/** 跟踪点与猫位置的最小间距约束，避免边界修正后贴近目标 */
#define RADAR_TRACK_ESCAPE_MIN_DIST_MM 600

_attribute_data_retention_ static radar_prediction_state_t g_radar_pred = {0};

// Motion direction cache (keep last 1 second of points)
#define RADAR_MOTION_CACHE_WINDOW_US  1000000u
#define RADAR_MOTION_CACHE_MAX_POINTS 64
/** 相邻轨迹点 tick 间隔换算为 ms 后允许的最大值，避免异常间隔拉爆速度/累加 */
#define RADAR_MOTION_STEP_DT_MS_MAX   120000u
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

    if (g_radar_pred.motion_cache_count)
    {
        u8 newest = RadarMotionCacheIndexNewest();
        if (g_radar_motion_cache[newest].x_mm != x_mm || g_radar_motion_cache[newest].y_mm != y_mm)
        {
            u32 prev_tick = g_radar_motion_cache[newest].tick;
            u32 dt_us     = (now_tick >= prev_tick) ? ((now_tick - prev_tick) >> 4) : 0u;
            u32 dt_ms     = dt_us / 1000u;
            if (dt_ms == 0u)
            {
                dt_ms = 1u;
            }
            if (dt_ms > RADAR_MOTION_STEP_DT_MS_MAX)
            {
                dt_ms = RADAR_MOTION_STEP_DT_MS_MAX;
            }
            {
                s32   dx_mm     = (s32)x_mm - (s32)g_radar_motion_cache[newest].x_mm;
                s32   dy_mm     = (s32)y_mm - (s32)g_radar_motion_cache[newest].y_mm;
                float dist2     = (float)dx_mm * (float)dx_mm + (float)dy_mm * (float)dy_mm;
                float dist_mm   = app_radar_mysqrt_3(dist2);
                u32   v_cms_mag = 0;
                if (dist_mm >= 5.0f)
                {
                    float v = dist_mm * 100.0f / (float)dt_ms;
                    if (v > 65535.0f)
                    {
                        v = 65535.0f;
                    }
                    v_cms_mag = (u32)v;

                    // BLE_LOG_D("add dt_ms %d, v_cms %d", dt_ms, v_cms_mag);
                    radar_play_on_cache_displacement_ms(dt_ms, v_cms_mag);
                }
            }
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

_attribute_data_retention_ static u32 g_radar_last_motion_tick = 0;
_attribute_data_retention_ static u8  g_radar_power_on         = 0;
_attribute_data_retention_ static u8  g_radar_uart_warmup_done = 0;
_attribute_data_retention_ static u32 g_radar_uart_warmup_tick = 0;
_attribute_data_retention_ static u8  g_radar_hold_on_mode     = 1;
_attribute_data_retention_ static u8  g_radar_working_mode     = 0;
/** 工作模式 0→1 后的第一段位移累加丢弃，避免刚进入 work 时与上一 newest 距离过大导致速度尖峰 */
_attribute_data_retention_ static u8 g_radar_drop_first_disp_after_work = 0;

static void radar_working_mode_set(u8 on)
{
    if (on)
    {
        if (!g_radar_working_mode)
        {
            BLE_LOG_D("g_radar_drop_first_disp_after_work");
            g_radar_drop_first_disp_after_work = 1;
        }
        g_radar_working_mode = 1;
    }
    else
    {
        g_radar_working_mode               = 0;
        g_radar_drop_first_disp_after_work = 0;
    }
}

_attribute_data_retention_ static u8 g_radar_power_log_last_state = 0;

_attribute_data_retention_ static u32 g_radar_time_sec          = 0;
_attribute_data_retention_ static u32 g_radar_time_tick_acc_us  = 0;
_attribute_data_retention_ static u32 g_radar_time_last_tick    = 0;
_attribute_data_retention_ static u8  g_radar_time_valid        = 0;
_attribute_data_retention_ static s8  g_radar_time_tz_q15       = 0;
_attribute_data_retention_ static u8  g_radar_play_state        = RADAR_PLAY_STATE_IDLE;
_attribute_data_retention_ static u32 g_radar_play_active_start = 0;
_attribute_data_retention_ static u8  g_radar_play_active_idx   = 0;

_attribute_data_retention_ static u32 g_radar_play_start_sec[RADAR_TIME_MAX_RECORDS]     = {0};
_attribute_data_retention_ static u32 g_radar_play_end_sec[RADAR_TIME_MAX_RECORDS]       = {0};
_attribute_data_retention_ static s8  g_radar_play_tz_q15[RADAR_TIME_MAX_RECORDS]        = {0};
_attribute_data_retention_ static u8  g_radar_play_hunt_result[RADAR_TIME_MAX_RECORDS]   = {0};
_attribute_data_retention_ static u32 g_radar_play_motion_sec[RADAR_TIME_MAX_RECORDS]    = {0};
_attribute_data_retention_ static u16 g_radar_play_avg_speed_cms[RADAR_TIME_MAX_RECORDS] = {0};
_attribute_data_retention_ static u8  g_radar_play_record_count                          = 0;
_attribute_data_retention_ static u8  g_radar_play_record_next                           = 0;
/** 最近一次雷达串口速度 (cm/s)，用于会话内「是否触发跟踪」等，非逗宠统计主路径 */
_attribute_data_retention_ static s16 g_radar_last_speed_cms = 0;
/** 当前进行中逗宠段：工作模式下轨迹点相对上一 newest 位移时累计的毫秒；speed_dt = Σ(v_cm/s×Δt_ms) 供时间加权平均 */
_attribute_data_retention_ static u32                g_radar_sess_motion_ms      = 0;
_attribute_data_retention_ static unsigned long long g_radar_sess_speed_dt_sum   = 0u;
_attribute_data_retention_ static u8                 g_radar_boundary_configured = 0;
_attribute_data_retention_ static u8                 g_radar_install_height_set  = 0;

/* 边界多边形：默认是矩形，但支持修改为任意凸四边形（点顺序需为逆时针或顺时针一致） */
static const radar_boundary_point_t g_radar_boundary_quad_default[4] = {
    {-6000, 6000},
    {6000, 6000},
    {6000, 0},
    {-6000, 0},
};

_attribute_data_retention_ static radar_boundary_point_t g_radar_boundary_quad[4] = {
    {-6000, 6000},
    {6000, 6000},
    {6000, 0},
    {-6000, 0},
};

#define RADAR_SHOULIE_POINT_AND_CONFIG_MAGIC 0x52445343u  // "RDSC"
#define RADAR_INSTALL_HEIGHT_FLASH_MAGIC     0x52444948u  // "RDIH"
#define RADAR_PLAY_RECORD_FLASH_MAGIC        0x5244504Du  // "RDPM" 含运动统计，与旧 RDPL 布局不兼容

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
    u8  hunt_result[RADAR_TIME_MAX_RECORDS];  // 狩猎结果: 0=未完成 1=完成 2=捕猎成功
    u32 motion_sec[RADAR_TIME_MAX_RECORDS];
    u16 avg_speed_cms[RADAR_TIME_MAX_RECORDS];
    u32 crc;
} radar_play_record_flash_t;

_attribute_data_retention_ static s32 g_radar_install_height_mm = (s32)RADAR_INSTALL_HEIGHT_DEFAULT_MM;

#define RADAR_LOW_FREQ_ON_US       (1000000u * 1u)   // 1s
#define RADAR_LOW_FREQ_OFF_US      (1000000u * 4u)   // 4s
#define RADAR_HOLD_ON_NO_MOTION_US (1000000u * 10u)  // 15s
#define RADAR_HOLD_ON_NO_MOTION_S  RADAR_HOLD_ON_NO_MOTION_US / (1000000u)
#define RADAR_WORK_MAX_US          (1000000u * 600u)      // 10min
#define RADAR_REST_EXIT_US         (1000000u * 60u)       // 1min
#define RADAR_UART_WARMUP_US       (1000000u * 7u / 10u)  // 700ms

static void app_radar_power_switch(u8 on)
{
    if (!on)
    {
        app_radar_uart_deinit();
        RadarSessionStop(1);
    }
    else
    {
        app_radar_uart_init();
    }
    g_radar_power_on = on ? 1 : 0;
}

static void app_radar_power_state_reset(void)
{
    if (g_radar_power_log_last_state == 0)
    {
        return;
    }
    BLE_LOG_D("app_radar_power_state_reset");
    g_radar_hold_on_mode = 1;
    radar_working_mode_set(1);
    g_radar_power_log_last_state = 0;
}

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

/* 猎物点 + 狩猎配置 flash 结构（magic "RDSC"） */
typedef struct
{
    u32 magic;       // RADAR_SHOULIE_POINT_AND_CONFIG_MAGIC
    s16 pan_deg10;   // 猎物点水平角 deg×10
    s16 tilt_deg10;  // 猎物点俯仰角 deg×10
    u16 duration_s;  // 单次狩猎时长(秒)
    u8  count;       // 狩猎次数
    u8  sleep_min;   // 休眠时长(分钟)
    u8  reserved[4];
    u32 crc;
} radar_prey_point_cfg_flash_t;

void radar_prey_point_cfg_save_to_flash(void)
{
    radar_prey_point_cfg_flash_t stored;
    stored.magic      = RADAR_SHOULIE_POINT_AND_CONFIG_MAGIC;
    stored.pan_deg10  = g_prey_pan_deg10;
    stored.tilt_deg10 = g_prey_tilt_deg10;
    stored.duration_s = g_hunt_duration_s;
    stored.count      = g_hunt_count;
    stored.sleep_min  = g_hunt_sleep_duration_min;
    memset(stored.reserved, 0, sizeof(stored.reserved));
    stored.crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    flash_erase_sector(RADAR_PREY_POINT_CFG_FLASH_ADDR);
    flash_write_page(RADAR_PREY_POINT_CFG_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
    BLE_LOG_D("prey+cfg saved to flash: pan=%d tilt=%d dur=%d cnt=%d slp=%d",
              stored.pan_deg10,
              stored.tilt_deg10,
              stored.duration_s,
              stored.count,
              stored.sleep_min);
}

static void radar_prey_point_cfg_load_from_flash(void)
{
    radar_prey_point_cfg_flash_t stored;
    flash_read_page(RADAR_PREY_POINT_CFG_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
    if (stored.magic != RADAR_SHOULIE_POINT_AND_CONFIG_MAGIC)
    {
        return;
    }
    u32 crc = radar_boundary_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    if (crc != stored.crc)
    {
        return;
    }
    g_prey_pan_deg10  = stored.pan_deg10;
    g_prey_tilt_deg10 = stored.tilt_deg10;
    hunt_prey_point_invalidate_xy();
    g_hunt_duration_s         = stored.duration_s;
    g_hunt_count              = stored.count;
    g_hunt_sleep_duration_min = stored.sleep_min;
    BLE_LOG_D("prey+cfg loaded from flash: pan=%d tilt=%d dur=%d cnt=%d slp=%d",
              g_prey_pan_deg10,
              g_prey_tilt_deg10,
              g_hunt_duration_s,
              g_hunt_count,
              g_hunt_sleep_duration_min);
}

/** 轨迹缓存刷新且 (x,y) 相对 newest 变化时：在工作模式 + 逗宠进行中累加 Δt(ms) 与 v×Δt（v 由位移/mm 与 Δt/ms 得到，cm/s） */
static void radar_play_on_cache_displacement_ms(u32 dt_ms, u32 speed_cms_mag)
{
    if (!g_radar_working_mode || g_radar_play_state != RADAR_PLAY_STATE_ACTIVE || dt_ms == 0u)
    {
        // BLE_LOG_D("skip cache disp: work %d state %d dt_ms %d", g_radar_working_mode, g_radar_play_state, dt_ms);
        return;
    }
    if (g_radar_drop_first_disp_after_work)
    {
        BLE_LOG_D("dsip after work");
        g_radar_drop_first_disp_after_work = 0;
        return;
    }
    if (g_radar_sess_motion_ms <= 0xFFFFFFFFu - dt_ms)
    {
        g_radar_sess_motion_ms += dt_ms;
    }
    {
        unsigned long long add = (unsigned long long)speed_cms_mag * (unsigned long long)dt_ms;
        if (g_radar_sess_speed_dt_sum <= (unsigned long long)(-1) - add)
        {
            g_radar_sess_speed_dt_sum += add;
        }
    }
}

static void radar_play_records_reset_ram(void)
{
    for (u8 i = 0; i < RADAR_TIME_MAX_RECORDS; i++)
    {
        g_radar_play_start_sec[i]     = 0;
        g_radar_play_end_sec[i]       = 0;
        g_radar_play_tz_q15[i]        = 0;
        g_radar_play_hunt_result[i]   = 0;
        g_radar_play_motion_sec[i]    = 0;
        g_radar_play_avg_speed_cms[i] = 0;
    }
    g_radar_play_record_count          = 0;
    g_radar_play_record_next           = 0;
    g_radar_play_state                 = RADAR_PLAY_STATE_IDLE;
    g_radar_play_active_start          = 0;
    g_radar_play_active_idx            = 0;
    g_radar_sess_motion_ms             = 0;
    g_radar_sess_speed_dt_sum          = 0u;
    g_radar_last_speed_cms             = 0;
    g_radar_drop_first_disp_after_work = 0;
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
        stored.start_sec[i]     = g_radar_play_start_sec[i];
        stored.end_sec[i]       = g_radar_play_end_sec[i];
        stored.tz_q15[i]        = g_radar_play_tz_q15[i];
        stored.hunt_result[i]   = g_radar_play_hunt_result[i];
        stored.motion_sec[i]    = g_radar_play_motion_sec[i];
        stored.avg_speed_cms[i] = g_radar_play_avg_speed_cms[i];
    }

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
        g_radar_play_start_sec[i]     = stored.start_sec[i];
        g_radar_play_end_sec[i]       = stored.end_sec[i];
        g_radar_play_tz_q15[i]        = stored.tz_q15[i];
        g_radar_play_hunt_result[i]   = stored.hunt_result[i];
        g_radar_play_motion_sec[i]    = stored.motion_sec[i];
        g_radar_play_avg_speed_cms[i] = stored.avg_speed_cms[i];
    }

    g_radar_play_record_count = stored.record_count;
    g_radar_play_record_next  = stored.record_next;
    if (g_radar_play_record_next >= RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_next = 0;
    }
    g_radar_play_state        = RADAR_PLAY_STATE_IDLE;
    g_radar_play_active_start = 0;
    g_radar_play_active_idx   = 0;
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

static u8 radar_play_record_push(u32 start_sec, u32 end_sec)
{
    u8 idx = g_radar_play_record_next;
    BLE_LOG_D("push record idx %d start %d end %d", idx, start_sec, end_sec);

    g_radar_play_start_sec[idx]     = start_sec;
    g_radar_play_end_sec[idx]       = end_sec;
    g_radar_play_tz_q15[idx]        = g_radar_time_tz_q15;
    g_radar_play_hunt_result[idx]   = HUNT_RESULT_INCOMPLETE;
    g_radar_play_motion_sec[idx]    = 0;
    g_radar_play_avg_speed_cms[idx] = 0;

    g_radar_play_record_next++;
    if (g_radar_play_record_next >= RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_next = 0;
    }
    if (g_radar_play_record_count < RADAR_TIME_MAX_RECORDS)
    {
        g_radar_play_record_count++;
    }

    return idx;
}

static inline u8 radar_play_record_is_complete(u32 end_sec)
{
    return (end_sec != RADAR_PLAY_END_ONGOING) ? 1 : 0;
}

static void radar_play_record_start(void)
{
    if (!g_radar_time_valid)
    {
        // BLE_LOG_D("radar_play_record_start failed");
        return;
    }

    if (g_radar_play_state != RADAR_PLAY_STATE_ACTIVE)
    {
        g_radar_sess_motion_ms    = 0;
        g_radar_sess_speed_dt_sum = 0u;
        g_radar_play_state        = RADAR_PLAY_STATE_ACTIVE;
        g_radar_play_active_start = g_radar_time_sec;
        g_radar_play_active_idx   = radar_play_record_push(g_radar_play_active_start, RADAR_PLAY_END_ONGOING);
        // radar_play_records_save_to_flash();
        BLE_LOG_D("radar_play_record_push");
        // gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
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
    BLE_LOG_D("radar_play_record_end");
    radar_working_mode_set(0);

    gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
    {
        u32                mot_ms  = g_radar_sess_motion_ms;
        u32                mot_sec = (mot_ms + 500u) / 1000u;
        u16                av      = 0;
        unsigned long long sum     = g_radar_sess_speed_dt_sum;
        if (mot_ms != 0u)
        {
            av = (u16)((sum + (unsigned long long)(mot_ms / 2u)) / (unsigned long long)mot_ms);
        }
        g_radar_play_motion_sec[g_radar_play_active_idx]    = mot_sec;
        g_radar_play_avg_speed_cms[g_radar_play_active_idx] = av;
    }
    g_radar_sess_motion_ms    = 0;
    g_radar_sess_speed_dt_sum = 0u;
    if (g_radar_play_hunt_result[g_radar_play_active_idx] == HUNT_RESULT_COMPLETE ||
        g_radar_play_hunt_result[g_radar_play_active_idx] == HUNT_RESULT_SUCCESS)
    {
        g_radar_play_end_sec[g_radar_play_active_idx] = g_radar_play_start_sec[g_radar_play_active_idx] + g_hunt_duration_s;
    }
    else
    {
        g_radar_play_end_sec[g_radar_play_active_idx] = g_radar_time_sec;
    }
    if (g_radar_play_end_sec[g_radar_play_active_idx] - g_radar_play_start_sec[g_radar_play_active_idx] < RADAR_HOLD_ON_NO_MOTION_S - 1)
    {
        BLE_LOG_D("end %d - start < %d %d idx %d", g_radar_play_end_sec[g_radar_play_active_idx], RADAR_HOLD_ON_NO_MOTION_S, g_radar_play_start_sec[g_radar_play_active_idx], g_radar_play_active_idx);
        g_radar_play_active_idx--;
        g_radar_play_state = RADAR_PLAY_STATE_IDLE;
        return;
    }

    g_radar_play_state = RADAR_PLAY_STATE_IDLE;
    radar_play_records_save_to_flash();
    app_ctrl_notify_play_record_changed();
}

void app_radar_set_time_from_epoch(u32 epoch_sec, s8 tz_q15)
{
    g_radar_time_sec         = epoch_sec;
    g_radar_time_tz_q15      = tz_q15;
    g_radar_time_tick_acc_us = 0;
    g_radar_time_last_tick   = clock_time();
    g_radar_time_valid       = 1;

    u16 year  = 0;
    u8  month = 0;
    u8  day   = 0;
    u8  hour  = 0;
    u8  min   = 0;
    u8  sec   = 0;
    radar_epoch_to_datetime(epoch_sec, &year, &month, &day, &hour, &min, &sec);

    BLE_LOG_D("epoch_sec: %d, tz_q15: %d", epoch_sec, tz_q15);
    BLE_LOG_D("datetime: %04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, min, sec);
}

void app_radar_on_time_tick(void)
{
    if (!g_radar_time_valid)
    {
        return;
    }

    u32 now_tick = clock_time();
    if (g_radar_time_last_tick == 0)
    {
        g_radar_time_last_tick = now_tick;
        return;
    }

    u32 elapsed_us         = (now_tick - g_radar_time_last_tick) >> 4;
    g_radar_time_last_tick = now_tick;

    g_radar_time_tick_acc_us += elapsed_us;
    while (g_radar_time_tick_acc_us >= 1000000u)
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

int app_radar_get_complete_play_records(u32 *out_buf, u8 *tz_buf, u32 *motion_sec_out, u16 *avg_speed_cms_out, u8 max_records)
{

    if (!out_buf || !tz_buf || max_records == 0)
    {
        return 0;
    }

    u8 count     = 0;
    u8 need_save = 0;
    u8 idx       = g_radar_play_record_next;
    for (u8 i = 0; i < g_radar_play_record_count && count < max_records; i++)
    {
        if (idx == 0)
        {
            idx = RADAR_TIME_MAX_RECORDS;
        }
        idx--;

        if (!radar_play_record_is_complete(g_radar_play_end_sec[idx]))
        {
            continue;
        }

        u32 start = g_radar_play_start_sec[idx];
        u32 end   = g_radar_play_end_sec[idx];

        /* 持续时间不足 RADAR_HOLD_ON_NO_MOTION_S 或时间戳回绕/异常：标记为无效并跳过，下次 clear 会清除 */
        if (end <= start || (end - start) < (RADAR_HOLD_ON_NO_MOTION_S - 1))
        {
            g_radar_play_end_sec[idx]       = 0;
            g_radar_play_motion_sec[idx]    = 0;
            g_radar_play_avg_speed_cms[idx] = 0;
            need_save                       = 1;
            continue;
        }

        out_buf[count * 2]     = start;
        out_buf[count * 2 + 1] = end;
        tz_buf[count]          = (u8)g_radar_play_tz_q15[idx];
        if (motion_sec_out)
        {
            motion_sec_out[count] = g_radar_play_motion_sec[idx];
        }
        if (avg_speed_cms_out)
        {
            avg_speed_cms_out[count] = g_radar_play_avg_speed_cms[idx];
        }
        BLE_LOG_D("count: %d, start: %d, end: %d, result: %d", count, start, end, g_radar_play_hunt_result[idx]);
        count++;
    }

    if (need_save)
    {
        radar_play_records_save_to_flash();
    }

    return count;
}

u8 app_radar_has_complete_play_records(void)
{
    BLE_LOG_D("app_radar_has_complete_play_records");
    u8 idx = g_radar_play_record_next;
    for (u8 i = 0; i < g_radar_play_record_count; i++)
    {
        if (idx == 0)
        {
            idx = RADAR_TIME_MAX_RECORDS;
        }
        idx--;

        if (radar_play_record_is_complete(g_radar_play_end_sec[idx]))
        {
            BLE_LOG_D("found complete record idx %d", idx);
            return 1;
        }
    }

    return 0;
}

void app_radar_clear_complete_play_records(void)
{
    u32 start_tmp[RADAR_TIME_MAX_RECORDS] = {0};
    u32 end_tmp[RADAR_TIME_MAX_RECORDS]   = {0};
    s8  tz_tmp[RADAR_TIME_MAX_RECORDS]    = {0};
    u8  res_tmp[RADAR_TIME_MAX_RECORDS]   = {0};
    u32 mot_tmp[RADAR_TIME_MAX_RECORDS]   = {0};
    u16 av_tmp[RADAR_TIME_MAX_RECORDS]    = {0};
    u8  keep_count                        = 0;

    u8 start = (u8)((g_radar_play_record_next + RADAR_TIME_MAX_RECORDS - g_radar_play_record_count) % RADAR_TIME_MAX_RECORDS);
    for (u8 i = 0; i < g_radar_play_record_count; i++)
    {
        u8 idx = (u8)((start + i) % RADAR_TIME_MAX_RECORDS);
        if (radar_play_record_is_complete(g_radar_play_end_sec[idx]))
        {
            continue;
        }

        start_tmp[keep_count] = g_radar_play_start_sec[idx];
        end_tmp[keep_count]   = g_radar_play_end_sec[idx];
        tz_tmp[keep_count]    = g_radar_play_tz_q15[idx];
        res_tmp[keep_count]   = g_radar_play_hunt_result[idx];
        mot_tmp[keep_count]   = g_radar_play_motion_sec[idx];
        av_tmp[keep_count]    = g_radar_play_avg_speed_cms[idx];
        keep_count++;
    }

    for (u8 i = 0; i < RADAR_TIME_MAX_RECORDS; i++)
    {
        g_radar_play_start_sec[i]     = 0;
        g_radar_play_end_sec[i]       = 0;
        g_radar_play_tz_q15[i]        = 0;
        g_radar_play_hunt_result[i]   = 0;
        g_radar_play_motion_sec[i]    = 0;
        g_radar_play_avg_speed_cms[i] = 0;
    }
    for (u8 i = 0; i < keep_count; i++)
    {
        g_radar_play_start_sec[i]     = start_tmp[i];
        g_radar_play_end_sec[i]       = end_tmp[i];
        g_radar_play_tz_q15[i]        = tz_tmp[i];
        g_radar_play_hunt_result[i]   = res_tmp[i];
        g_radar_play_motion_sec[i]    = mot_tmp[i];
        g_radar_play_avg_speed_cms[i] = av_tmp[i];
    }

    g_radar_play_record_count = keep_count;
    g_radar_play_record_next  = keep_count % RADAR_TIME_MAX_RECORDS;
    radar_play_records_save_to_flash();
}

u8 app_radar_is_install_height_set(void)
{
    return g_radar_install_height_set;
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

void app_radar_clear_install_height_and_record_flash(void)
{
    g_radar_install_height_mm  = (s32)RADAR_INSTALL_HEIGHT_DEFAULT_MM;
    g_radar_install_height_set = 0;

    for (int i = 0; i < 4; i++)
    {
        g_radar_boundary_quad[i].x_mm = g_radar_boundary_quad_default[i].x_mm;
        g_radar_boundary_quad[i].y_mm = g_radar_boundary_quad_default[i].y_mm;
    }

    radar_play_records_reset_ram();

    // flash_erase_sector(RADAR_BOUNDARY_FLASH_ADDR);
    flash_erase_sector(RADAR_INSTALL_HEIGHT_FLASH_ADDR);
    flash_erase_sector(RADAR_PLAY_RECORD_FLASH_ADDR);
    flash_erase_sector(RADAR_PREY_POINT_CFG_FLASH_ADDR);
}

void app_radar_init(void)
{
    s32 x_mm[4];
    s32 y_mm[4];

    app_radar_reset_boundary_default();

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
    radar_prey_point_cfg_load_from_flash();
}

void app_radar_point_to_pan_tilt(s32 x_mm, s32 y_mm, s32 height_mm, s16 *pan_deg10, s16 *tilt_deg10)
{
    // 根据x_mm和y_mm计算出pan_deg
    *pan_deg10 = (s16)(lookup_atan2((float)x_mm, (float)y_mm) * RAD_TO_DEG * 10.0f);
    // 根据x_mm和y_mm计算出r_mm
    s32 r_mm = app_radar_mysqrt_3(x_mm * x_mm + y_mm * y_mm);
    // 根据r_mm和h_mm计算出tilt_deg
    *tilt_deg10 = (s16)(lookup_atan2((float)r_mm, (float)height_mm) * RAD_TO_DEG * 10.0f) - 900;
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
    RadarMotionCacheReset();
    radar_play_record_end();
    (void)reset;
}

u8 app_radar_is_working_mode(void)
{
    return g_radar_working_mode;
}

static void RadarSessionOnMotion(u32 now_tick)
{
    g_radar_last_motion_tick = now_tick;
    // 狩猎状态机负责在 HUNT_ACTIVE 入口调用 radar_play_record_start()
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
    hunt_prey_point_invalidate_xy();

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

void app_radar_reset_boundary_default(void)
{
    for (int i = 0; i < 4; i++)
    {
        g_radar_boundary_quad[i] = g_radar_boundary_quad_default[i];
    }
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
}

#if (UI_STEP_MOTOR_ENABLE)
_attribute_data_retention_ static u32 g_radar_track_gimbal_interval_us = RADAR_TRACK_GIMBAL_INTERVAL_DEFAULT_US;

void app_radar_set_track_gimbal_interval_us(u32 interval_us)
{
    if (interval_us < STEP_MOTOR_MIN_INTERVAL_US_FASTEST)
    {
        interval_us = STEP_MOTOR_MIN_INTERVAL_US_FASTEST;
    }
    if (interval_us > 20000u)
    {
        interval_us = 20000u;
    }
    g_radar_track_gimbal_interval_us = interval_us;
}

/** 与蓝牙「移向限位」相同思路：固定步间隔 + 单一目标，电机顺滑跟随 */
#define M_PI     3.1415926
#define M_PI_3   (M_PI / 4.0f)
#define M_2_PI_3 (3.0f * M_PI / 4.0f)
static void RadarGimbalApplyTargetMm(s16 x_mm, s16 y_mm, float motion_rad)
{
    s16 pan_deg10  = 0;
    s16 tilt_deg10 = 0;

    app_radar_point_to_pan_tilt(x_mm, y_mm, g_radar_install_height_mm, &pan_deg10, &tilt_deg10);
    StepMotor_GimbalSetSpeedUs(g_radar_track_gimbal_interval_us);
    // 如果方向为pi/3-2pi/3，则需要将水平角和垂直角固定+5°
    if (motion_rad > M_PI_3 && motion_rad < M_2_PI_3)
    {
        pan_deg10 += 50;
        tilt_deg10 += 25;
    }
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, pan_deg10);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, tilt_deg10);
}
#else
void app_radar_set_track_gimbal_interval_us(u32 interval_us)
{
    (void)interval_us;
}
#endif

#if RADAR_TRACK_LEAD_ENABLE
/**
 * 在 (proc_x, proc_y) 基础上沿 motion_rad 向前偏移 lead_mm（与既有预测点 sin(x)/cos(y) 约定一致），
 * 越界时按反射前进修正并保证最小逃跑距离，再回到工作四边形内。
 */
static void RadarTrackComputeLeadMm(s16 proc_x, s16 proc_y, u8 motion_valid, u8 is_stationary, float motion_rad, s16 v_cm_s, s16 *out_x, s16 *out_y)
{
    s16 v_abs;
    s32 lead_mm;
    s32 rand1_mm = 0;
    s32 rand2_mm = 0;

    *out_x = proc_x;
    *out_y = proc_y;

    // if (!motion_valid)
    // {
    //     return;
    // }

    if (is_stationary)
    {
        /* 静止：沿用本帧给出的方向（含「粘滞」的最后运动方向），仅较小超前 + 200mm*/
        lead_mm  = (s32)RADAR_TRACK_LEAD_MM_STATIC;
        rand1_mm = RadarRandRangeI32(-200, 200);
        rand2_mm = RadarRandRangeI32(-200, 200);
    }
    else
    {
        v_abs   = RadarSpeedAbs(v_cm_s);
        lead_mm = (s32)RadarRandRangeI32(RADAR_TRACK_LEAD_MM_MIN, RADAR_TRACK_LEAD_MM_MIN + 200) + ((s32)v_abs * (s32)RADAR_TRACK_LEAD_MM_PER_NUM) / (s32)RADAR_TRACK_LEAD_MM_PER_DEN;
        if (lead_mm < (s32)RADAR_TRACK_LEAD_MM_MIN)
        {
            lead_mm = (s32)RADAR_TRACK_LEAD_MM_MIN;
        }
        else if (lead_mm > (s32)RADAR_TRACK_LEAD_MM_MAX)
        {
            lead_mm = (s32)RADAR_TRACK_LEAD_MM_MAX;
        }
    }

    {
        s32           tx              = (s32)proc_x + (s32)((float)lead_mm * lookup_sin(motion_rad)) + rand1_mm;
        s32           ty              = (s32)proc_y + (s32)((float)lead_mm * lookup_cos(motion_rad)) + rand2_mm;
        s16           candidate_x     = 0;
        s16           candidate_y     = 0;
        float         escape_dir_rad  = motion_rad;
        s32           escape_min_dist = (s32)RADAR_TRACK_ESCAPE_MIN_DIST_MM;
        float         dist_to_proc_mm = 0.0f;
        s32           step_dx         = 0;
        s32           step_dy         = 0;
        radar_vec2f_t step_unit       = RadarVec2(0.0f, 0.0f);

        if (tx > 32767)
        {
            tx = 32767;
        }
        else if (tx < -32768)
        {
            tx = -32768;
        }
        if (ty > 32767)
        {
            ty = 32767;
        }
        else if (ty < -32768)
        {
            ty = -32768;
        }
        candidate_x = (s16)tx;
        candidate_y = (s16)ty;

        /* 越界时按“沿运动向量前进并反射”处理，避免最近垂足投影导致贴边/贴猫。 */
        if (!RadarPointInsideQuad((s32)candidate_x, (s32)candidate_y))
        {
            radar_vec2f_t v_step = RadarVec2((float)((s32)candidate_x - (s32)proc_x), (float)((s32)candidate_y - (s32)proc_y));
            RadarAdvanceReflectQuad((s32)proc_x, (s32)proc_y, v_step, &candidate_x, &candidate_y, &escape_dir_rad);
            if (!RadarPointInsideQuad((s32)candidate_x, (s32)candidate_y))
            {
                float proj_dist = 0.0f;
                RadarProjectToQuad((s32)candidate_x, (s32)candidate_y, &candidate_x, &candidate_y, &proj_dist);
            }
        }

        step_dx         = (s32)candidate_x - (s32)proc_x;
        step_dy         = (s32)candidate_y - (s32)proc_y;
        dist_to_proc_mm = app_radar_mysqrt_3((float)(step_dx * step_dx + step_dy * step_dy));

        /* 边界修正后若离目标过近，强制推开到最小“逃跑”距离。 */
        if (dist_to_proc_mm < (float)escape_min_dist)
        {
            if (dist_to_proc_mm > 1.0f)
            {
                step_unit = RadarNormalize(RadarVec2((float)step_dx, (float)step_dy));
            }
            else
            {
                step_unit = RadarNormalize(RadarVec2(lookup_sin(escape_dir_rad), lookup_cos(escape_dir_rad)));
                if (step_unit.x == 0.0f && step_unit.y == 0.0f)
                {
                    step_unit = RadarVec2(0.0f, 1.0f);
                }
            }

            {
                radar_vec2f_t v_escape = RadarVecMul(step_unit, (float)escape_min_dist);
                RadarAdvanceReflectQuad((s32)proc_x, (s32)proc_y, v_escape, &candidate_x, &candidate_y, &escape_dir_rad);
                if (!RadarPointInsideQuad((s32)candidate_x, (s32)candidate_y))
                {
                    float proj_dist = 0.0f;
                    RadarProjectToQuad((s32)candidate_x, (s32)candidate_y, &candidate_x, &candidate_y, &proj_dist);
                }
            }
        }

        *out_x = candidate_x;
        *out_y = candidate_y;
    }
}
#endif /* RADAR_TRACK_LEAD_ENABLE */

static u32  tick_xy_mm = 0;
static void ReportPredictionSerialized(u32 now_tick, s16 x_mm, s16 y_mm, s16 v_cm_s)
{
    s16   proc_x        = x_mm;
    s16   proc_y        = y_mm;
    s16   dx_mm         = 0;
    s16   dy_mm         = 0;
    float motion_rad    = 0.0f;
    u8    motion_valid  = 0;
    u8    is_stationary = 1;
    u8    oldest        = 0;
    u8    newest        = 0;

    if (abs(g_radar_pred.prev_x_mm - x_mm) > STATIONARY_DXY_THRESHOLD_MM || abs(g_radar_pred.prev_y_mm - y_mm) > STATIONARY_DXY_THRESHOLD_MM)
    {
        gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
        motion_valid  = 1;
        is_stationary = 0;
        RadarSessionOnMotion(now_tick);
    }
    // 0.5s打印一次
    if (tick_xy_mm == 0 || clock_time_exceed(tick_xy_mm, 1000000))
    {
        tick_xy_mm = clock_time();
        BLE_LOG_D("x = %d,  y = %d", x_mm, y_mm);
    }
    g_radar_pred.prev_x_mm = x_mm;
    g_radar_pred.prev_y_mm = y_mm;
    RadarMotionCachePush(now_tick, x_mm, y_mm);

    // Prefer direction derived from the 1s window edge points (oldest & newest)
    if (g_radar_pred.motion_cache_count >= 2)
    {
        oldest     = RadarMotionCacheIndexOldest();
        newest     = RadarMotionCacheIndexNewest();
        s16 dx_w   = (s16)(g_radar_motion_cache[newest].x_mm - g_radar_motion_cache[oldest].x_mm);
        s16 dy_w   = (s16)(g_radar_motion_cache[newest].y_mm - g_radar_motion_cache[oldest].y_mm);
        motion_rad = lookup_atan2((float)dx_w, (float)dy_w);
    }
    else
    {
        return;
    }
    {
#if DEBUG_MODE
        // BLE_LOG_D("%d", motion_valid);
        // s16 motion_dir_deg10 = 0;
        // if (motion_valid)
        // {
        //     float d = motion_rad * (float)RAD_TO_DEG * 10.0f;
        //     if (d > 32767.0f)
        //     {
        //         d = 32767.0f;
        //     }
        //     else if (d < -32768.0f)
        //     {
        //         d = -32768.0f;
        //     }
        //     motion_dir_deg10 = (s16)d;
        // }
        // app_ctrl_radar_dbg_send_prev_raw(
        //     g_radar_motion_cache[oldest].x_mm, g_radar_motion_cache[oldest].y_mm, g_radar_motion_cache[newest].x_mm, g_radar_motion_cache[newest].y_mm, motion_valid, motion_dir_deg10);
#endif
    }

    if (!RadarPointInsideQuad((s32)proc_x, (s32)proc_y))
    {
        RadarMirrorOutsideAcrossNearestEdge(&proc_x, &proc_y);
    }

    /*
     * 跟踪：先保证目标在场地四边形内 (proc)，再在阶段二沿运动方向加超前量。
     */
    {
        s16 track_x = proc_x;
        s16 track_y = proc_y;
#if RADAR_TRACK_LEAD_ENABLE
        RadarTrackComputeLeadMm(proc_x, proc_y, motion_valid, is_stationary, motion_rad, v_cm_s, &track_x, &track_y);
#else
        (void)motion_valid;
        (void)motion_rad;
        (void)is_stationary;
        (void)v_cm_s;
#endif

        // app_ctrl_radar_dbg_send_predseq(1, track_x, track_y);
#if (UI_STEP_MOTOR_ENABLE)
        if (g_radar_hold_on_mode)
        {
            if (track_x == proc_x && track_y == proc_y)
            {
                BLE_LOG_D("hold on, no motion, skip gimbal move");
                return;
            }
            else
            {
                RadarGimbalApplyTargetMm(track_x, track_y, motion_rad);
            }
        }
#else
        (void)track_x;
        (void)track_y;
#endif
    }
}

#define RADAR_FPS_TEST 1
#if RADAR_FPS_TEST
static u32 radar_start_time = 0;
#endif

static void radar_uart_hw_drain_and_clear(void)
{
    unsigned char rx_cnt = reg_uart_buf_cnt & 0x0f;
    while (rx_cnt--)
    {
        (void)uart_ndma_read_byte();
    }
    uart_clear_parity_error();
}

static u8 g_radar_uart_inited = 0;

static void radar_uart_rx_state_reset(void)
{
    g_uart_ndma_rx_flag     = 0;
    g_uart_ndma_rx_byte_cnt = 0;
    memset((void *)g_uart_ndma_rx_byte, 0, sizeof(g_uart_ndma_rx_byte));
    uart_ndma_clear_rx_index();
    uart_ndma_clear_tx_index();
}
void app_radar_parse_and_report_frame(void)
{
    if (!g_radar_uart_inited)
    {
        return;
    }

    if (!g_radar_uart_warmup_done)
    {
        if (!clock_time_exceed(g_radar_uart_warmup_tick, RADAR_UART_WARMUP_US))
        {
            return;
        }

        radar_uart_rx_state_reset();
        radar_uart_hw_drain_and_clear();
        uart_irq_enable(1, 0);
        g_radar_uart_warmup_done = 1;
        return;
    }

    if (!g_uart_ndma_rx_flag)
    {
        return;
    }
    g_uart_ndma_rx_flag = 0;
#if RADAR_RX_IRQ_DEBUG
    g_radar_dbg_rx_flag_cleared_cnt++;
#endif
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

            g_radar_last_speed_cms = v_avg;
            // LOG_D("radar, x=%dmm y=%dmm v=%dcm/s", x_avg, y_avg, v_avg);
            u32 now_tick = clock_time();
            if (RadarSpeedAbs(v_avg) > RADAR_TARGET_MOVING_SPEED_THR_CMS)
            {
                // RadarSessionOnMotion(now_tick);
            }

            ReportPredictionSerialized(now_tick, x_in, y_in, v_avg);

#if RADAR_FPS_TEST
            // u32 current_time = clock_time() >> 4;
            // if (radar_start_time != 0)
            // {
            //     u32 dt = (current_time - radar_start_time);
            //     if (dt > 0)
            //     {
            //         u32 fps = 1000000 / dt;
            //         BLE_LOG_D("radar fps: %d", fps);
            //     }
            // }
            // radar_start_time = current_time;
#endif
        }
    }
}

u8 app_radar_has_recent_motion(u32 timeout_us)
{
    if (g_radar_last_motion_tick == 0)
    {
        return 0;
    }

    return clock_time_exceed(g_radar_last_motion_tick, timeout_us) ? 0 : 1;
}

void app_radar_set_enabled(u8 on)
{
    if (!on)
    {
        app_radar_power_state_reset();
        app_radar_power_switch(0);
        g_hunt_state           = HUNT_STATE_OFF;
        g_hunt_completed       = 0;
        g_hunt_total_active_ms = 0;
        g_hunt_session_acc_ms  = 0;
        g_hunt_no_target_tick  = 0;
        g_hunt_prey_zone_tick  = 0;
    }
    else
    {
        app_radar_power_state_reset();
        g_hunt_state           = HUNT_STATE_IDLE;
        g_hunt_completed       = 0;
        g_hunt_total_active_ms = 0;
        g_hunt_session_acc_ms  = 0;
        g_hunt_no_target_tick  = 0;
        g_hunt_prey_zone_tick  = 0;
        g_hunt_success         = 0;
        g_hunt_acc_tick_last   = 0;
        g_hunt_sleep_end_tick  = 0;
        gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
        radar_working_mode_set(1);
        g_radar_hold_on_mode = 1;
        app_radar_power_switch(1);
    }
}

/* ========== 狩猎游戏辅助函数 ========== */

/** 将猎物点角度 (pan/tilt deg10) 转换为地面坐标 (x, y mm) */
static void hunt_prey_point_to_xy(s32 height_mm, s16 *out_x_mm, s16 *out_y_mm)
{
    if (!out_x_mm || !out_y_mm)
        return;
    if (g_prey_tilt_deg10 >= 0)
    {
        *out_x_mm = 0;
        *out_y_mm = 0;
        return;
    }
    s32 r1_mm = (s32)((float)height_mm * lookup_tan((900 + g_prey_tilt_deg10) * DEG_TO_RAD_10));
    s32 x_mm  = (s32)((float)r1_mm * lookup_sin(g_prey_pan_deg10 * DEG_TO_RAD_10));
    s32 y_mm  = (s32)((float)r1_mm * lookup_cos(g_prey_pan_deg10 * DEG_TO_RAD_10));
    if (x_mm > 32767)
        x_mm = 32767;
    else if (x_mm < -32768)
        x_mm = -32768;
    if (y_mm > 32767)
        y_mm = 32767;
    else if (y_mm < -32768)
        y_mm = -32768;
    *out_x_mm = (s16)x_mm;
    *out_y_mm = (s16)y_mm;
    BLE_LOG_D("prey point pan %d tilt %d -> x %d y %d", g_prey_pan_deg10, g_prey_tilt_deg10, *out_x_mm, *out_y_mm);
}

/** 检查最新的雷达目标位置是否在猎物点半径范围内 */
static u32 last_log_tick = 0;
static u8  hunt_is_target_near_prey_point(void)
{
    if (g_radar_pred.motion_cache_count == 0)
        return 0;

    u8  newest = RadarMotionCacheIndexNewest();
    s16 tx     = g_radar_motion_cache[newest].x_mm;
    s16 ty     = g_radar_motion_cache[newest].y_mm;

    s32 height_mm = g_radar_install_height_mm;
    if (height_mm <= 0)
        height_mm = 2500;

    if (!g_prey_xy_valid)
    {
        hunt_prey_point_to_xy(height_mm, &g_prey_px_mm, &g_prey_py_mm);
        g_prey_xy_valid = 1;
    }

    s32 dx = (s32)tx - (s32)g_prey_px_mm;
    s32 dy = (s32)ty - (s32)g_prey_py_mm;
    // 0.5s打印一次
    if (last_log_tick == 0 || clock_time_exceed(last_log_tick, 500000))
    {
        last_log_tick = clock_time();
        // 在目标点附近时或等到目标点时才打印，避免过多无关日志
        if (g_hunt_prey_zone_tick != 0 || g_hunt_state == HUNT_STATE_CELEBRATE)
            BLE_LOG_D("target(%d,%d) prey(%d,%d) dx %d dy %d", tx, ty, g_prey_px_mm, g_prey_py_mm, dx, dy);
    }
    s32 d2 = dx * dx + dy * dy;

    return (d2 <= (s32)HUNT_PREY_ZONE_RADIUS_MM * (s32)HUNT_PREY_ZONE_RADIUS_MM) ? 1 : 0;
}

/** 休眠超时后进入Idle */
static u32 hunt_sleep_duration_us(void)
{
    return (u32)g_hunt_sleep_duration_min * 60u * 1000000u;
}

/* ========== 狩猎游戏状态机主函数 ========== */
void app_radar_task_power_schedule(void)
{
    u32 now_tick = clock_time();

    // OFF ──(开机)──→ IDLE ──(检测到目标)──→ ACTIVE
    //                                      │
    //                 ┌────────────────────┼────────────────────┐
    //                 ↓                    ↓                    ↓
    //            STANDBY             PREY_ZONE_SLEEP     COMPLETE_MOVING
    //        (15s无目标,激光+        (15s在猎物点,           (移动到猎物点)
    //         电机关闭,雷达保持)      雷达+激光+电机关闭)        │
    //                 │              30s后→IDLE               ↓
    //                 │                                 CELEBRATE
    //                 └──(目标返回)──→ ACTIVE          (停留10秒,检查)
    //                                                     │
    //                                           ┌─────────┴────────┐
    //                                           ↓                  ↓
    //                                        SLEEP               IDLE
    //                                   (达成次数或累计        (继续等待)
    //                                    时长,重置计数)
    //                                       │
    //                                       └── 休眠到期 → IDLE
    switch (g_hunt_state)
    {

    /* -------- 关机 -------- */
    case HUNT_STATE_OFF:
        app_radar_power_switch(0);
        g_radar_hold_on_mode = 0;
        radar_working_mode_set(0);
        return;

    /* -------- 逗宠等待: radar on, 检测到目标后启动狩猎 -------- */
    case HUNT_STATE_IDLE: {
        if (!g_radar_power_on)
        {
            BLE_LOG_D("HUNT: IDLE -> power on radar");
            g_radar_hold_on_mode = 1;
            radar_working_mode_set(1);
            app_radar_power_switch(1);
            return;
        }
        g_radar_hold_on_mode = 1;
        // 检测到移动目标 → 开始狩猎
        if (app_radar_has_recent_motion(HUNT_NO_TARGET_TIMEOUT_US))
        {
            BLE_LOG_D("HUNT: IDLE -> ACTIVE (target detected)");
            g_hunt_state          = HUNT_STATE_ACTIVE;
            g_hunt_session_acc_ms = 0;
            g_hunt_acc_tick_last  = now_tick;
            g_hunt_session_tick   = now_tick;
            g_hunt_success        = 0;
            g_hunt_no_target_tick = 0;
            g_hunt_prey_zone_tick = 0;
            // 启动逗宠记录
            radar_working_mode_set(1);
            RadarSessionOnMotion(now_tick);
            radar_play_record_start();  // 启动狩猎记录
            // gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
        }
        return;
    }

    /* -------- 狩猎中: 计时 + 跟踪 + 检测中断条件 -------- */
    case HUNT_STATE_ACTIVE: {
        if (!g_radar_power_on)
        {
            app_radar_power_switch(1);
        }
        g_radar_hold_on_mode = 1;

        // 累计狩猎时间 (每秒)
        if (g_hunt_acc_tick_last)
        {
            u32 elapsed_us = (now_tick - g_hunt_acc_tick_last) >> 4;
            if (elapsed_us >= 1000000u)
            {
                g_hunt_session_acc_ms = g_radar_sess_motion_ms;
                g_hunt_total_active_ms += 1000;  // 每秒增加1000ms
                g_hunt_acc_tick_last = now_tick;
                BLE_LOG_D("HUNT: active %ds/%ds total %ds",
                          g_hunt_session_acc_ms / 1000,
                          g_hunt_duration_s,
                          g_hunt_total_active_ms / 1000);
                BLE_LOG_D("last_motion %d", (g_radar_last_motion_tick / 1000000u) >> 4);
                // 调试的目标丢失累计时间打印
                if (g_hunt_no_target_tick != 0)
                {
                    BLE_LOG_D("HUNT: no target for %ds", ((now_tick - g_hunt_no_target_tick) / 1000000u) >> 4);
                }
                // 调试目标在猎物点附近累计时间打印
                if (g_hunt_prey_zone_tick != 0)
                {
                    BLE_LOG_D("HUNT: target in prey zone for %ds", ((now_tick - g_hunt_prey_zone_tick) / 1000000u) >> 4);
                }
            }
        }
        else
        {
            g_hunt_acc_tick_last = now_tick;
        }

        // 条件1: 1秒未检测到目标后开始计时，连续15s无目标 → 待机
        if (!app_radar_has_recent_motion(1000000))
        {
            if (g_hunt_no_target_tick == 0)
            {
                g_hunt_no_target_tick = now_tick;
                BLE_LOG_D("HUNT: no target detected, start timer %d", (g_hunt_no_target_tick / 1000000) >> 4);
            }
            else if (clock_time_exceed(g_hunt_no_target_tick, HUNT_NO_TARGET_TIMEOUT_US))
            {
                radar_play_record_end();  // 记录为未完成
                StepMotor_StopAll();
                gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
                g_radar_hold_on_mode  = 0;
                g_hunt_state          = HUNT_STATE_STANDBY;
                g_hunt_no_target_tick = 0;
                g_hunt_prey_zone_tick = 0;
                BLE_LOG_D("HUNT: ACTIVE -> STANDBY (15s no target) total_ms %d", g_hunt_total_active_ms / 1000);
                return;
            }
        }
        else
        {
            g_hunt_no_target_tick = 0;
        }

        // 条件2: 连续15s目标在猎物点半径20cm内 → 30s休眠
        if (hunt_is_target_near_prey_point())
        {
            if (g_hunt_prey_zone_tick == 0)
            {
                g_hunt_prey_zone_tick = now_tick;
                BLE_LOG_D("HUNT: target in prey zone, start timer %d", (g_hunt_prey_zone_tick / 1000000) >> 4);
            }
            else if (clock_time_exceed(g_hunt_prey_zone_tick, HUNT_PREY_ZONE_TIMEOUT_US))
            {
                radar_play_record_end();  // 记录为未完成
                radar_working_mode_set(0);
                StepMotor_StopAll();
                gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);
                app_radar_power_switch(0);
                g_radar_hold_on_mode  = 0;
                g_hunt_state          = HUNT_STATE_PREY_ZONE_SLEEP;
                g_hunt_no_target_tick = 0;
                g_hunt_prey_zone_tick = 0;
                BLE_LOG_D("HUNT: ACTIVE -> PREY_ZONE_SLEEP (target in prey zone 15s) total_ms %d", g_hunt_total_active_ms / 1000);
                return;
            }
        }
        else
        {
            g_hunt_prey_zone_tick = 0;
        }

        // 条件3: 狩猎计时到达预设值(固定时长，不按猫运动的时长来判断) → 完成移动
        if (clock_time_exceed(g_hunt_session_tick, (u32)g_hunt_duration_s * 1000000u))
        // if (g_hunt_session_acc_ms / 1000 >= (u32)g_hunt_duration_s)
        {
            BLE_LOG_D("HUNT: ACTIVE -> COMPLETE_MOVING (timer %ds)", g_hunt_duration_s);
            BLE_LOG_D("HUNT: stop tracking and move to prey point");
            radar_working_mode_set(0);
            StepMotor_StopAll();
            g_radar_hold_on_mode = 0;  // 停止雷达跟踪, 由云台移动到猎物点
            g_hunt_state         = HUNT_STATE_COMPLETE_MOVING;
            // 移动光斑到猎物点
            StepMotor_GimbalSetSpeedUs(12000);
            StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)g_prey_pan_deg10);
            StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)g_prey_tilt_deg10);
            gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
            return;
        }

        // 条件4：总逗宠时长若达到预设值则直接进入休眠
        if (g_hunt_total_active_ms / 1000 >= (u32)g_hunt_duration_s * (u32)g_hunt_count)
        {
            BLE_LOG_D("HUNT: ACTIVE -> SLEEP (total active %ds)", g_hunt_total_active_ms / 1000);
            radar_play_record_end();
            app_radar_power_switch(0);
            g_radar_hold_on_mode  = 0;
            g_hunt_state          = HUNT_STATE_SLEEP;
            g_hunt_sleep_end_tick = now_tick;
        }

        return;
    }

    /* -------- 待机: 关闭激光+电机, 雷达保持, 检测到目标则恢复 -------- */
    case HUNT_STATE_STANDBY: {
        if (!g_radar_power_on)
            app_radar_power_switch(1);
        g_radar_hold_on_mode = 0;  // 不跟踪

        // 检测到目标 → 恢复狩猎
        if (app_radar_has_recent_motion(HUNT_NO_TARGET_TIMEOUT_US))
        {
            BLE_LOG_D("HUNT: STANDBY -> ACTIVE (target returned)");
            g_hunt_session_acc_ms = 0;
            g_hunt_acc_tick_last  = now_tick;
            g_hunt_session_tick   = now_tick;
            g_hunt_success        = 0;
            g_hunt_no_target_tick = 0;
            g_hunt_prey_zone_tick = 0;
            g_radar_hold_on_mode  = 1;
            g_hunt_state          = HUNT_STATE_ACTIVE;
            RadarSessionOnMotion(now_tick);
            radar_play_record_start();  // 启动新狩猎记录
            radar_working_mode_set(1);
            gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
        }
        return;
    }

    /* -------- 30秒休眠 (目标在猎物点): 雷达+激光+电机关闭 -------- */
    case HUNT_STATE_PREY_ZONE_SLEEP: {
        if (g_hunt_prey_zone_tick == 0)
        {
            g_hunt_prey_zone_tick = now_tick;
        }
        app_radar_power_switch(0);
        g_radar_hold_on_mode = 0;
        gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);

        if (clock_time_exceed(g_hunt_prey_zone_tick, HUNT_PREY_ZONE_SLEEP_DUR_US))
        {
            BLE_LOG_D("HUNT: PREY_ZONE_SLEEP -> IDLE (30s up)");
            g_hunt_prey_zone_tick = 0;
            g_hunt_state          = HUNT_STATE_IDLE;
        }
        return;
    }

    /* -------- 移动到猎物点: 等待云台到达 -------- */
    case HUNT_STATE_COMPLETE_MOVING: {
        // 等待云台到达猎物点
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) && !StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
        {
            // s16 pan_cur  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
            // s16 tilt_cur = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
            // if (abs(pan_cur - g_prey_pan_deg10) <= 10 && abs(tilt_cur - g_prey_tilt_deg10) <= 10)
            // {
            BLE_LOG_D("HUNT: COMPLETE_MOVING -> CELEBRATE (at prey point)");
            // StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)g_prey_pan_deg10);
            // StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)g_prey_tilt_deg10);
            g_hunt_celebration_tick = now_tick;
            g_hunt_success          = 0;
            g_hunt_state            = HUNT_STATE_CELEBRATE;
            // }
        }
        return;
    }

    /* -------- 停留10秒: 检查目标是否进入猎物点半径 -------- */
    case HUNT_STATE_CELEBRATE: {
        gpio_write(GPIO_LED_WHITE, LED_ON_LEVEL);
        // 激光停在猎物点, 电机已停止
        StepMotor_StopAll();

        // 检查目标是否进入猎物点半径20cm → 记录捕猎成功
        if (!g_hunt_success && hunt_is_target_near_prey_point())
        {
            g_hunt_success = 1;
            // 标记当前记录为成功
            g_radar_play_hunt_result[g_radar_play_active_idx] = HUNT_RESULT_SUCCESS;
            BLE_LOG_D("HUNT: CELEBRATE -> SUCCESS! target entered prey zone");
        }

        // 10秒到 → 判定完成
        if (clock_time_exceed(g_hunt_celebration_tick, HUNT_CELEBRATION_DUR_US))
        {
            // 如果还没标记成功, 标记为完成(非成功)
            if (!g_hunt_success)
            {
                if (g_radar_play_hunt_result[g_radar_play_active_idx] == HUNT_RESULT_INCOMPLETE)
                {
                    g_radar_play_hunt_result[g_radar_play_active_idx] = HUNT_RESULT_COMPLETE;
                }
            }

            radar_play_record_end();

            g_hunt_completed++;
            BLE_LOG_D("HUNT: CELEBRATE done. completed=%d/%d, total_active=%ds/%ds",
                      g_hunt_completed,
                      g_hunt_count,
                      g_hunt_total_active_ms,
                      (u32)g_hunt_duration_s * (u32)g_hunt_count);
            gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);

            // 判断是否进入休眠
            if (g_hunt_completed >= g_hunt_count)
            {
                BLE_LOG_D("HUNT: CELEBRATE -> SLEEP (complete time limit reached)");
                app_radar_power_switch(0);
                g_radar_hold_on_mode  = 0;
                g_hunt_state          = HUNT_STATE_SLEEP;
                g_hunt_sleep_end_tick = now_tick;
            }
            else
            {
                BLE_LOG_D("HUNT: CELEBRATE -> IDLE");
                g_hunt_state = HUNT_STATE_IDLE;
            }
        }
        return;
    }

    /* -------- 休眠: 重置狩猎次数和累计时长, 到期后回到Idle -------- */
    case HUNT_STATE_SLEEP: {
        app_radar_power_switch(0);
        g_radar_hold_on_mode = 0;
        gpio_write(GPIO_LED_WHITE, !LED_ON_LEVEL);

        if (clock_time_exceed(g_hunt_sleep_end_tick, hunt_sleep_duration_us()))
        {
            BLE_LOG_D("HUNT: SLEEP -> IDLE (sleep %dmin done)", g_hunt_sleep_duration_min);
            // 重置狩猎完成次数和累计游戏时长
            g_hunt_completed       = 0;
            g_hunt_total_active_ms = 0;
            g_hunt_state           = HUNT_STATE_IDLE;
        }
        return;
    }

    default:
        g_hunt_state = HUNT_STATE_IDLE;
        return;
    }
}

u8 app_radar_is_power_on(void)
{
    return g_radar_power_on;
}

void app_radar_uart_init(void)
{
    if (g_radar_uart_inited)
    {
        return;
    }
    g_radar_uart_inited = 1;

    gpio_write(LEIDA_SWITCH, 1);
    uart_gpio_set(GPIO_PD6, GPIO_PD5);
    uart_init_baudrate(256000, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_dma_enable(0, 0);
    uart_ndma_irq_triglevel(1, 0);

    radar_uart_rx_state_reset();
    radar_uart_hw_drain_and_clear();
    uart_irq_enable(0, 0);

    g_radar_uart_warmup_done = 0;
    g_radar_uart_warmup_tick = clock_time();
}

void app_radar_uart_deinit(void)
{
    if (!g_radar_uart_inited)
    {
        return;
    }
    g_radar_uart_inited = 0;

    uart_irq_enable(0, 0);
    radar_uart_rx_state_reset();
    radar_uart_hw_drain_and_clear();

    // 关闭雷达串口,将串口引脚设置为GPIO并拉低以降低断电后功耗
    gpio_write(LEIDA_SWITCH, 0);
    gpio_set_func(GPIO_PD6, AS_GPIO);
    gpio_set_func(GPIO_PD5, AS_GPIO);
    gpio_set_output_en(GPIO_PD6, 1);
    gpio_set_output_en(GPIO_PD5, 1);
    gpio_write(GPIO_PD6, 0);
    gpio_write(GPIO_PD5, 0);

    g_radar_uart_warmup_done = 0;
    g_radar_uart_warmup_tick = 0;
}

#if RADAR_RX_IRQ_DEBUG
extern u8 app_get_power_state(void);

void app_radar_debug_rx_poll(void)
{
    static u32 last_tick;
    static u32 snap_drain;
    static u32 snap_clr;
    static u32 snap_set;
    static u32 snap_irq;
    static u32 snap_asm;

    if (!last_tick)
    {
        last_tick = clock_time();
        return;
    }
    if (!clock_time_exceed(last_tick, 1000000u))
    {
        return;
    }
    last_tick = clock_time();

    u32 d_drain = g_radar_dbg_rx_drain_bytes - snap_drain;
    u32 d_clr   = g_radar_dbg_rx_flag_cleared_cnt - snap_clr;
    u32 d_set   = g_radar_dbg_rx_flag_set_cnt - snap_set;
    u32 d_irq   = g_radar_dbg_rx_uart_irq_cnt - snap_irq;
    u32 d_asm   = g_radar_dbg_rx_asm_bytes - snap_asm;
    snap_drain  = g_radar_dbg_rx_drain_bytes;
    snap_clr    = g_radar_dbg_rx_flag_cleared_cnt;
    snap_set    = g_radar_dbg_rx_flag_set_cnt;
    snap_irq    = g_radar_dbg_rx_uart_irq_cnt;
    snap_asm    = g_radar_dbg_rx_asm_bytes;

    u8 app_on     = app_get_power_state();
    u8 rad_on     = app_radar_is_power_on();
    u8 parse_gate = (app_on && rad_on) ? 1U : 0U;

    LOG_D(
        "radar_rx dbg/s: d_irq=%u d_asm=%u d_set=%u d_clr=%u d_drain=%u f=%u app=%u rad=%u gate=%u",
        (unsigned)d_irq,
        (unsigned)d_asm,
        (unsigned)d_set,
        (unsigned)d_clr,
        (unsigned)d_drain,
        (unsigned)g_uart_ndma_rx_flag,
        (unsigned)app_on,
        (unsigned)rad_on,
        (unsigned)parse_gate);

    if (d_drain > 0U && d_clr == 0U && g_uart_ndma_rx_flag)
    {
        LOG_D("radar_rx STUCK? drain without clr");
    }
    if (g_uart_ndma_rx_flag && !parse_gate)
    {
        LOG_D("radar_rx note: f=1 gate=0 (app/rad flags)");
    }
}
#else
void app_radar_debug_rx_poll(void)
{
}
#endif

/**
 * 按帧头 AA FF 03 00 重新对齐，避免丢字节/噪声后永远等不到「恰好 30 字节且尾为 55 CC」的旧逻辑死锁。
 * 收满 30 字节后仅校验帧尾；帧头已在同步阶段保证。
 */
static void radar_uart_ndma_rx_push_byte(u8 data)
{
    if (g_uart_ndma_rx_byte_cnt < 4)
    {
        if (g_uart_ndma_rx_byte_cnt == 0)
        {
            if (data == 0xAA)
            {
                g_uart_ndma_rx_byte[0]  = data;
                g_uart_ndma_rx_byte_cnt = 1;
            }
        }
        else if (g_uart_ndma_rx_byte_cnt == 1)
        {
            if (data == 0xFF)
            {
                g_uart_ndma_rx_byte[1]  = data;
                g_uart_ndma_rx_byte_cnt = 2;
            }
            else
            {
                g_uart_ndma_rx_byte_cnt = (data == 0xAA) ? 1 : 0;
                if (data == 0xAA)
                {
                    g_uart_ndma_rx_byte[0] = 0xAA;
                }
            }
        }
        else if (g_uart_ndma_rx_byte_cnt == 2)
        {
            if (data == 0x03)
            {
                g_uart_ndma_rx_byte[2]  = data;
                g_uart_ndma_rx_byte_cnt = 3;
            }
            else if (data == 0xAA)
            {
                g_uart_ndma_rx_byte[0]  = 0xAA;
                g_uart_ndma_rx_byte_cnt = 1;
            }
            else
            {
                g_uart_ndma_rx_byte_cnt = 0;
            }
        }
        else
        {
            if (data == 0x00)
            {
                g_uart_ndma_rx_byte[3]  = data;
                g_uart_ndma_rx_byte_cnt = 4;
            }
            else if (data == 0xAA)
            {
                g_uart_ndma_rx_byte[0]  = 0xAA;
                g_uart_ndma_rx_byte_cnt = 1;
            }
            else
            {
                g_uart_ndma_rx_byte_cnt = 0;
            }
        }
        return;
    }

    g_uart_ndma_rx_byte[g_uart_ndma_rx_byte_cnt++] = data;
    if (g_uart_ndma_rx_byte_cnt == RADAR_FRAME_LEN)
    {
        if (g_uart_ndma_rx_byte[RADAR_FRAME_LEN - 2] == 0x55 && g_uart_ndma_rx_byte[RADAR_FRAME_LEN - 1] == 0xCC)
        {
            g_uart_ndma_rx_flag = 1;
#if RADAR_RX_IRQ_DEBUG
            g_radar_dbg_rx_flag_set_cnt++;
#endif
        }
        g_uart_ndma_rx_byte_cnt = 0;
    }
}

void app_radar_uart_ndma_irq_proc(void)
{
    if (!uart_ndmairq_get())
    {
        return;
    }

#if RADAR_RX_IRQ_DEBUG
    g_radar_dbg_rx_uart_irq_cnt++;
#endif

    unsigned char rx_cnt = reg_uart_buf_cnt & 0x0f;

    if (g_uart_ndma_rx_flag)
    {
#if RADAR_RX_IRQ_DEBUG
        g_radar_dbg_rx_drain_irq_cnt++;
        g_radar_dbg_rx_drain_bytes += (u32)rx_cnt;
#endif
        while (rx_cnt--)
        {
            (void)uart_ndma_read_byte();
        }
        return;
    }

    while (rx_cnt--)
    {
#if RADAR_RX_IRQ_DEBUG
        g_radar_dbg_rx_asm_bytes++;
#endif
        radar_uart_ndma_rx_push_byte(uart_ndma_read_byte());
    }
}

/* ========== 狩猎游戏公开 API ========== */

u16 app_hunt_get_duration_s(void)
{
    return g_hunt_duration_s;
}

void app_hunt_set_duration_s(u16 s)
{
    if (s < 10)
        s = 10;
    if (s > 600)
        s = 600;
    g_hunt_duration_s = s;
}

u8 app_hunt_get_count(void)
{
    return g_hunt_count;
}

void app_hunt_set_count(u8 cnt)
{
    if (cnt < 1)
        cnt = 1;
    // 最大值: ceil(600 / 单次狩猎时长)
    u8 max_cnt = (u8)((600u + (u32)g_hunt_duration_s - 1u) / (u32)g_hunt_duration_s);
    if (cnt > max_cnt)
        cnt = max_cnt;
    g_hunt_count = cnt;
}

u8 app_hunt_get_sleep_duration_min(void)
{
    return g_hunt_sleep_duration_min;
}

void app_hunt_set_sleep_duration_min(u8 min)
{
    if (min < 1)
        min = 1;
    if (min > 20)
        min = 20;
    g_hunt_sleep_duration_min = min;
}

void app_hunt_get_prey_point_deg10(s16 *pan_deg10, s16 *tilt_deg10)
{
    if (pan_deg10)
        *pan_deg10 = g_prey_pan_deg10;
    if (tilt_deg10)
        *tilt_deg10 = g_prey_tilt_deg10;
}

void app_hunt_set_prey_point_deg10(s16 pan_deg10, s16 tilt_deg10)
{
    // 水平限制 ±60°
    if (pan_deg10 > GIMBAL_PAN_LIMIT_DEG10_POS)
        pan_deg10 = GIMBAL_PAN_LIMIT_DEG10_POS;
    if (pan_deg10 < GIMBAL_PAN_LIMIT_DEG10_NEG)
        pan_deg10 = GIMBAL_PAN_LIMIT_DEG10_NEG;
    // 俯仰限制 -10°~-80°
    if (tilt_deg10 > GIMBAL_TILT_LIMIT_DEG10_POS)
        tilt_deg10 = GIMBAL_TILT_LIMIT_DEG10_POS;
    if (tilt_deg10 < GIMBAL_TILT_LIMIT_DEG10_NEG)
        tilt_deg10 = GIMBAL_TILT_LIMIT_DEG10_NEG;
    g_prey_pan_deg10  = pan_deg10;
    g_prey_tilt_deg10 = tilt_deg10;
    hunt_prey_point_invalidate_xy();
    radar_prey_point_cfg_save_to_flash();
}

// 随机移动循环状态机
static u8  g_prey_random_active = 0;
static u8  g_prey_random_state  = 0;  // 0=IDLE, 1=MOVING, 2=WAITING
static u32 g_prey_random_tick   = 0;

#define PREY_RANDOM_WAIT_US 000000u  // 到达后停留 0.5s

void app_hunt_prey_random_move(void)
{
    // 水平角度（±60°）俯仰角（15°~30°）间随机移动
    s16 pan_deg10  = (s16)RadarRandRangeI32(GIMBAL_PAN_LIMIT_DEG10_NEG, GIMBAL_PAN_LIMIT_DEG10_POS);
    s16 tilt_deg10 = (s16)RadarRandRangeI32(150 - 900, 450 - 900);  // 15°-45°

#if (UI_STEP_MOTOR_ENABLE)
    StepMotor_GimbalSetSpeedUs(1200);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)pan_deg10);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)tilt_deg10);
#endif
}

void app_hunt_prey_random_set_active(u8 active)
{
    if (active)
    {
        g_prey_random_active = 1;
        g_prey_random_state  = 0;  // IDLE → task will trigger first move
        g_prey_random_tick   = clock_time();
    }
    else
    {
        g_prey_random_active = 0;
        g_prey_random_state  = 0;
    }
}

void app_hunt_prey_save(u8 active)
{
    g_prey_random_active = 0;
    g_prey_random_state  = 0;
    hunt_prey_point_invalidate_xy();
    if (active)
    {
#if (UI_STEP_MOTOR_ENABLE)
        s16 pan  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
        s16 tilt = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)pan);
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)tilt);
        app_hunt_set_prey_point_deg10(pan, tilt);
        StepMotor_StopAll();
        BLE_LOG_D("prey random stop, auto-save: pan=%d, tilt=%d", pan, tilt);
#endif
    }
}

u8 app_hunt_prey_random_is_active(void)
{
    return g_prey_random_active;
}

void app_hunt_prey_random_task(void)
{
    if (!g_prey_random_active)
    {
        return;
    }

    switch (g_prey_random_state)
    {
    case 0:  // IDLE — 启动第一次移动
        app_hunt_prey_random_move();
        g_prey_random_state = 1;
        break;

    case 1:  // MOVING — 检查电机是否到达目标
#if (UI_STEP_MOTOR_ENABLE)
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) && !StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
        {
            g_prey_random_tick  = clock_time();
            g_prey_random_state = 2;
        }
#endif
        break;

    case 2:  // WAITING — 停留 PREY_RANDOM_WAIT_US 后继续下一随机点
        if (clock_time_exceed(g_prey_random_tick, PREY_RANDOM_WAIT_US))
        {
            app_hunt_prey_random_move();
            g_prey_random_state = 1;
        }
        break;
    }
}

u8 app_hunt_is_hunting(void)
{
    return (g_hunt_state == HUNT_STATE_ACTIVE ||
            g_hunt_state == HUNT_STATE_CELEBRATE ||
            g_hunt_state == HUNT_STATE_COMPLETE_MOVING)
               ? 1
               : 0;
}

u8 app_hunt_is_standby(void)
{
    return (g_hunt_state == HUNT_STATE_STANDBY ||
            g_hunt_state == HUNT_STATE_IDLE)
               ? 1
               : 0;
}

u8 app_hunt_is_sleeping(void)
{
    return (g_hunt_state == HUNT_STATE_SLEEP || g_hunt_state == HUNT_STATE_PREY_ZONE_SLEEP) ? 1 : 0;
}

int app_hunt_get_records_with_result(u32 *out_buf, u8 *tz_buf, u32 *motion_sec_out, u16 *avg_speed_cms_out, u8 *result_out, u8 max_records)
{
    if (!out_buf || !tz_buf || !result_out || max_records == 0)
    {
        return 0;
    }

    u8 count = g_radar_play_record_count;
    if (count > max_records)
        count = max_records;

    u8 idx = g_radar_play_record_next;
    BLE_LOG_D("get_records_with_result count=%d next_idx=%d", count, idx);
    for (u8 i = 0; i < count; i++)
    {
        if (idx == 0)
            idx = RADAR_TIME_MAX_RECORDS;
        idx--;

        out_buf[i * 2]     = g_radar_play_start_sec[idx];
        out_buf[i * 2 + 1] = g_radar_play_end_sec[idx];
        tz_buf[i]          = (u8)g_radar_play_tz_q15[idx];
        result_out[i]      = g_radar_play_hunt_result[idx];
        if (motion_sec_out)
            motion_sec_out[i] = g_radar_play_motion_sec[idx];
        if (avg_speed_cms_out)
            avg_speed_cms_out[i] = g_radar_play_avg_speed_cms[idx];
        g_radar_play_start_sec[i]     = 0;
        g_radar_play_end_sec[i]       = 0;
        g_radar_play_tz_q15[i]        = 0;
        g_radar_play_hunt_result[i]   = 0;
        g_radar_play_motion_sec[i]    = 0;
        g_radar_play_avg_speed_cms[i] = 0;
    }
    return count;
}

#endif /* UI_RADAR_ENABLE */
