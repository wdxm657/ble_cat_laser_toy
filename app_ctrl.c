/********************************************************************************************************
 * @file    app_ctrl.c
 *
 * @brief   Application control protocol implementation
 *          - Generic command channel over custom BLE service
 *          - Handles LED control, motor control, configuration, etc.
 *
 *******************************************************************************************************/

#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "app_config.h"
#include "app.h"
#include "app_att.h"
#include "app_ctrl.h"

#if (UI_STEP_MOTOR_ENABLE)
#include "StepMotor.h"
#endif

#include "app.h"
#include "app_adc_dbg.h"
#if (UI_RADAR_ENABLE)
#include "app_radar.h"
#endif

#include "SineTable.h"

// RX/TX buffers shared with ATT layer
u8 g_ctrlRxBuf[CTRL_RX_MAX_LEN] = {0};
u8 g_ctrlTxBuf[CTRL_TX_MAX_LEN] = {0};

// simple sequence generator for events/async notifications
static u8  g_ctrlSeq        = 0;
static u32 g_power_on_tick  = 0;
static u32 g_power_off_tick = 0;

#define POWER_CTRL_OFF_COOLDOWN_US (30000000u) / 1  // 30s

static volatile u8  s_ctrl_reboot_pending = 0;
static volatile u32 s_ctrl_reboot_tick    = 0;

static u8 app_ctrl_can_change_power(u8 target_on)
{
    u8 cur_on = app_get_power_state() ? 1 : 0;

    if (target_on)
    {
        // 开机不做额外限制
        return 1;
    }

    // target_on == 0: 关机限制
    if (cur_on)
    {
        // 开机后 30s 内禁止关机
        if (g_power_on_tick && !clock_time_exceed(g_power_on_tick, POWER_CTRL_OFF_COOLDOWN_US))
        {
            return 0;
        }
    }
    else
    {
        // 关机后 30s 内禁止重复关机
        if (g_power_off_tick && !clock_time_exceed(g_power_off_tick, POWER_CTRL_OFF_COOLDOWN_US))
        {
            return 0;
        }
    }

    return 1;
}

#if (UI_STEP_MOTOR_ENABLE)
typedef struct
{
    u8                active;
    u8                direction;
    s16               target_deg10;
    step_motor_axis_e axis;
    s32               dir_sign;
} app_ctrl_motor_dir_state_t;

static app_ctrl_motor_dir_state_t g_motor_dir_state = {0};
#endif

// simple example configuration structure (can be extended)
typedef struct
{
    u8  workMode;  // e.g. 0: normal, 1: low power, ...
    u16 motorDefaultOnMs;
} app_ctrl_cfg_t;

static app_ctrl_cfg_t g_ctrlCfg = {
    .workMode         = 0,
    .motorDefaultOnMs = 1000,
};

#if (UI_RADAR_ENABLE)
#define PLAY_RECORD_UPLOAD_DELAY_AFTER_CONN_US 2000000u

static u8  g_play_record_delay_active     = 0;
static u32 g_play_record_delay_start_tick = 0;

static u8 app_ctrl_play_record_upload_allowed(void)
{
    if (!g_play_record_delay_active)
    {
        return 1;
    }

    if (BLS_CONN_HANDLE == 0xFFFF)
    {
        return 0;
    }

    if (!clock_time_exceed(g_play_record_delay_start_tick, PLAY_RECORD_UPLOAD_DELAY_AFTER_CONN_US))
    {
        return 0;
    }

    g_play_record_delay_active = 0;
    BLE_LOG_D("play record upload delay done");
    return 1;
}

// ----------------------- 逐条逗宠记录上传状态机 -----------------------
// 每秒上传 1 条，收到 APP ACK 后再发下一条，避免低功耗蓝牙拥塞。

enum
{
    PLAY_UPLOAD_IDLE = 0,
    PLAY_UPLOAD_SEND_WAIT,  // 等待 1s 间隔后发送
    PLAY_UPLOAD_WAIT_ACK,   // 已发送，等待 APP ACK
};

static u8  g_play_upload_state                              = PLAY_UPLOAD_IDLE;
static u32 g_play_cache_records[RADAR_TIME_MAX_RECORDS * 2] = {0};
static u8  g_play_cache_timezones[RADAR_TIME_MAX_RECORDS]   = {0};
static u32 g_play_cache_motion[RADAR_TIME_MAX_RECORDS]      = {0};
static u16 g_play_cache_speed[RADAR_TIME_MAX_RECORDS]       = {0};
static u8  g_play_cache_result[RADAR_TIME_MAX_RECORDS]      = {0};
static u8  g_play_cache_total                               = 0;
static u8  g_play_cache_index                               = 0;
static u32 g_play_upload_tick                               = 0;

#define PLAY_RECORD_UPLOAD_INTERVAL_US    1000000u  // 每条记录间隔 1s
#define PLAY_RECORD_UPLOAD_ACK_TIMEOUT_US 2000000u  // ACK 超时 2s，超时后重传当前记录

/**
 * @brief 从缓存中发送当前索引的一条记录（EVENT），包含狩猎结果
 */
static void app_ctrl_upload_one_record_from_cache(void)
{
    u8  i          = g_play_cache_index;
    u32 start_sec  = g_play_cache_records[i * 2];
    u32 end_sec    = g_play_cache_records[i * 2 + 1];
    u32 duration   = (end_sec > start_sec) ? (end_sec - start_sec) : 0;
    u16 dur16      = (duration > 0xFFFFu) ? 0xFFFFu : (u16)duration;
    u32 msec       = g_play_cache_motion[i];
    u16 avs        = g_play_cache_speed[i];
    u8  result     = g_play_cache_result[i];
    u16 m16        = (msec > 0xFFFFu) ? 0xFFFFu : (u16)msec;
    u8  av8        = (avs > 255u) ? 255u : (u8)avs;

    u8 evt[13] = {0};
    evt[0]     = CTRL_STATUS_OK;
    // 由于APP端需要收到总数后才会发送ACK给设备，所以每次上传记录的total数量都需要是1
    evt[1]  = 1;
    evt[2]  = 0;
    evt[3]  = (u8)(start_sec & 0xFF);
    evt[4]  = (u8)((start_sec >> 8) & 0xFF);
    evt[5]  = (u8)((start_sec >> 16) & 0xFF);
    evt[6]  = (u8)((start_sec >> 24) & 0xFF);
    // duration_sec = end_sec - start_sec (u16 LE), 替换原 4 字节 end_sec 以将整帧控制在 20 字节内
    evt[7]  = (u8)(dur16 & 0xFF);
    evt[8]  = (u8)((dur16 >> 8) & 0xFF);
    evt[9]  = (u8)(m16 & 0xFF);
    evt[10] = (u8)((m16 >> 8) & 0xFF);
    evt[11] = av8;
    evt[12] = result;  // 狩猎结果: 0=未完成 1=完成 2=捕猎成功
    BLE_LOG_D("upload record %d/%d start:%d dur:%d tz:%d mot:%d av:%d res:%d",
              i + 1,
              g_play_cache_total,
              start_sec,
              dur16,
              g_play_cache_timezones[i],
              (u32)m16,
              (u32)av8,
              result);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_PLAY_RECORD_GET, g_ctrlSeq++, evt, sizeof(evt));

    // 记录发送时间戳，用于 ACK 超时重传判断
    g_play_upload_tick  = clock_time();
    g_play_upload_state = PLAY_UPLOAD_WAIT_ACK;
}

static void app_ctrl_try_upload_play_records(void)
{
    if (BLS_CONN_HANDLE == 0xFFFF)
    {
        return;
    }
    if (!app_ctrl_play_record_upload_allowed())
    {
        return;
    }

    // 如果正在上传中，不重新开始
    if (g_play_upload_state != PLAY_UPLOAD_IDLE)
    {
        return;
    }

    if (!app_radar_has_complete_play_records())
    {
        return;
    }

    BLE_LOG_D("app_hunt_get_records_with_result");
    int count = app_hunt_get_records_with_result(
        g_play_cache_records, g_play_cache_timezones, g_play_cache_motion, g_play_cache_speed, g_play_cache_result, RADAR_TIME_MAX_RECORDS);
    if (count <= 0)
    {
        return;
    }

    g_play_cache_total = (u8)count;
    g_play_cache_index = 0;

    // 一次性取出所有记录到缓存，然后清空雷达端已完成记录。
    // 后续依靠 ACK 超时重传机制确保每条记录都被 APP 收到后才会推进到下一条。
    app_radar_clear_complete_play_records();

    // 调度第一条：等待 1s 后发送
    g_play_upload_tick  = clock_time();
    g_play_upload_state = PLAY_UPLOAD_SEND_WAIT;
    BLE_LOG_D("play upload start: %d records", count);
}
#endif

#if (UI_RADAR_ENABLE)
#define RADAR_BOUNDARY_POINT_COUNT 4
#define RADAR_BOUNDARY_MIN_EDGE_MM 1000
#define RADAR_BOUNDARY_MIN_DIAG_MM 1000

#define RADAR_BOUNDARY_POINT_LU    0
#define RADAR_BOUNDARY_POINT_RU    1
#define RADAR_BOUNDARY_POINT_RD    2
#define RADAR_BOUNDARY_POINT_LD    3

enum
{
    CTRL_RADAR_BOUNDARY_OK = 0,
    CTRL_RADAR_BOUNDARY_ERR_EDGE_TOO_SHORT,
    CTRL_RADAR_BOUNDARY_ERR_ORDER,
    CTRL_RADAR_BOUNDARY_ERR_STATE,
    CTRL_RADAR_BOUNDARY_ERR_INDEX,
};

enum
{
    CTRL_RADAR_BOUNDARY_MODE_IDLE = 0,
    CTRL_RADAR_BOUNDARY_MODE_SETTING,
};

static s32 g_radar_boundary_x[RADAR_BOUNDARY_POINT_COUNT] = {-1000, 1000, 1000, -1000};
static s32 g_radar_boundary_y[RADAR_BOUNDARY_POINT_COUNT] = {4000, 4000, 800, 800};
static u8  g_radar_boundary_next_index                    = 0;
static u8  g_radar_boundary_ready                         = 0;
static u8  g_radar_boundary_mode                          = CTRL_RADAR_BOUNDARY_MODE_IDLE;
static u8  g_radar_boundary_active_index                  = 0xFF;
static u8  g_radar_boundary_point_mask                    = 0;
static s16 g_hieght_angle_10                              = 0;

static void app_ctrl_radar_boundary_reset(void)
{
    g_radar_boundary_next_index   = 0;
    g_radar_boundary_ready        = 0;
    g_radar_boundary_active_index = 0xFF;
    g_radar_boundary_point_mask   = 0;
}

static u8 app_ctrl_radar_boundary_check_order(const s32 x[4], const s32 y[4])
{
    s32 x0 = x[0];
    s32 y0 = y[0];  // left-up
    s32 x1 = x[1];
    s32 y1 = y[1];  // right-up
    s32 x2 = x[2];
    s32 y2 = y[2];  // right-down
    s32 x3 = x[3];
    s32 y3 = y[3];  // left-down

    if (!(y0 > y3 && y0 > y2 && y1 > y3 && y1 > y2))
    {
        BLE_LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        BLE_LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        BLE_LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        BLE_LOG_D("app_ctrl_radar_boundary_check_order failed: y0 > y3 && y0 > y2 && y1 > y3 && y1 > y2");
        return 0;
    }

    if (!(x0 < x1 && x3 < x2))
    {
        BLE_LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        BLE_LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        BLE_LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        BLE_LOG_D("app_ctrl_radar_boundary_check_order failed: x0 < x1 && x3 < x2");
        return 0;
    }

    return 1;
}

static u8 app_ctrl_radar_boundary_check_min_lengths(const s32 x[4], const s32 y[4], u8 *errDetail, u8 *shortPairMask)
{
    s32 dx;
    s32 dy;
    u8  pair_bit      = 0;
    u8  localPairMask = 0;

    // pair bit mapping:
    // bit0:(0,1) bit1:(0,2) bit2:(0,3) bit3:(1,2) bit4:(1,3) bit5:(2,3)
    for (u8 i = 0; i < 4; i++)
    {
        for (u8 j = (u8)(i + 1); j < 4; j++)
        {
            dx = x[j] - x[i];
            dy = y[j] - y[i];

            s64 len2 = (s64)dx * (s64)dx + (s64)dy * (s64)dy;
            if (len2 < (s64)RADAR_BOUNDARY_MIN_EDGE_MM * (s64)RADAR_BOUNDARY_MIN_EDGE_MM)
            {
                localPairMask |= (u8)(1u << pair_bit);
            }
            pair_bit++;
        }
    }

    if (shortPairMask)
    {
        *shortPairMask = localPairMask;
    }

    if (localPairMask != 0)
    {
        if (errDetail)
        {
            *errDetail = CTRL_RADAR_BOUNDARY_ERR_EDGE_TOO_SHORT;
        }
        return 0;
    }

    if (errDetail)
    {
        *errDetail = CTRL_RADAR_BOUNDARY_OK;
    }
    return 1;
}

static u8 app_ctrl_radar_boundary_commit(u8 *errDetail, u8 *shortPairMask)
{
    if (!g_radar_boundary_ready)
    {
        return 0;
    }

    if (!app_ctrl_radar_boundary_check_order(g_radar_boundary_x, g_radar_boundary_y))
    {
        if (errDetail)
        {
            *errDetail = CTRL_RADAR_BOUNDARY_ERR_ORDER;
        }
        app_radar_reset_boundary_default();
        g_radar_boundary_ready = 0;
        return 0;
    }
    BLE_LOG_D("app_ctrl_radar_boundary_check_order success");

    if (!app_ctrl_radar_boundary_check_min_lengths(g_radar_boundary_x, g_radar_boundary_y, errDetail, shortPairMask))
    {
        // app_radar_reset_boundary_default();
        g_radar_boundary_ready = 0;
        return 0;
    }
    BLE_LOG_D("app_ctrl_radar_boundary_check_min_lengths success");
    app_radar_set_boundary_quad(g_radar_boundary_x, g_radar_boundary_y);
    app_radar_save_boundary_quad_to_flash(g_radar_boundary_x, g_radar_boundary_y);
    g_radar_boundary_ready = 0;
    return 1;
}

static u8 g_last_charge_state   = 0xFF;
static u8 g_last_setting_state  = 0xFF;
static u8 g_last_hunting_state  = 0xFF;
static u8 g_last_standby_state  = 0xFF;
static u8 g_last_sleeping_state = 0xFF;

static void app_ctrl_calc_exclusive_mode_flags(u8 *hunting, u8 *standby, u8 *sleeping, u8 *setting)
{
    u8 s = app_ctrl_is_setting_mode() ? 1 : 0;
#if (UI_RADAR_ENABLE)
    u8 h  = app_hunt_is_hunting() ? 1 : 0;
    u8 st = app_hunt_is_standby() ? 1 : 0;
    u8 sl = app_hunt_is_sleeping() ? 1 : 0;
#else
    u8 h  = 0;
    u8 st = 0;
    u8 sl = 0;
#endif

    // 四种状态互斥：设置中 > 狩猎中 > 待机 > 休眠
    if (s)
    {
        h  = 0;
        st = 0;
        sl = 0;
    }
    else if (h)
    {
        st = 0;
        sl = 0;
    }
    else if (st)
    {
        sl = 0;
    }

    *hunting  = h;
    *standby  = st;
    *sleeping = sl;
    *setting  = s;
}

static u32 status_check_tick = 0;
void       app_ctrl_status_notify_task(void)
{
#if (UI_GEN_MOTOR_ENABLE) || (UI_RADAR_ENABLE)
    u8 changed = 0;

    u8  power_on     = app_get_power_state() ? 1 : 0;
    u8  boundary_set = 1;
    s32 height_mm    = 0;
    app_radar_get_install_height_mm(&height_mm);
    u16 install_height    = (U16_LO((s16)height_mm));
    u16 install_height_hi = (U16_HI((s16)height_mm));
    u8  charging          = app_adc_dbg_is_charging() ? 1 : 0;
    u8  setting_mode      = 0;
    u8  hunting_mode      = 0;
    u8  standby_mode      = 0;
    u8  sleeping_mode     = 0;
    app_ctrl_calc_exclusive_mode_flags(&hunting_mode, &standby_mode, &sleeping_mode, &setting_mode);
    /* 首次仅建立基线，不上报 */
    if (g_last_charge_state == 0xFF)
    {
        g_last_charge_state   = charging;
        g_last_setting_state  = setting_mode;
        g_last_hunting_state  = hunting_mode;
        g_last_standby_state  = standby_mode;
        g_last_sleeping_state = sleeping_mode;
        return;
    }

    if (charging != g_last_charge_state)
    {
        BLE_LOG_D("charging changed: %d -> %d", g_last_charge_state, charging);
        g_last_charge_state = charging;
        changed             = 1;
    }
    if (setting_mode != g_last_setting_state)
    {
        BLE_LOG_D("setting_mode changed: %d -> %d", g_last_setting_state, setting_mode);
        g_last_setting_state = setting_mode;
        changed              = 1;
    }
    if (hunting_mode != g_last_hunting_state)
    {
        BLE_LOG_D("hunting_mode changed: %d -> %d", g_last_hunting_state, hunting_mode);
        g_last_hunting_state = hunting_mode;
        changed              = 1;
    }
    if (standby_mode != g_last_standby_state)
    {
        BLE_LOG_D("standby_mode changed: %d -> %d", g_last_standby_state, standby_mode);
        g_last_standby_state = standby_mode;
        changed              = 1;
    }
    if (sleeping_mode != g_last_sleeping_state)
    {
        BLE_LOG_D("sleeping_mode changed: %d -> %d", g_last_sleeping_state, sleeping_mode);
        g_last_sleeping_state = sleeping_mode;
        changed               = 1;
    }
    if (changed)
    {
        // 状态最多允许1s更新1次，避免过于频繁地通知APP（尤其是充电状态可能会有较大波动）
        // if (clock_time_exceed(status_check_tick, 1000000))
        // {
        status_check_tick = clock_time();
        u8 pl[10]         = {CTRL_STATUS_OK, power_on, boundary_set, install_height, install_height_hi, charging, setting_mode, hunting_mode, standby_mode, sleeping_mode};
        app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_STATUS_GET, g_ctrlSeq++, pl, sizeof(pl));
        // }
    }
#endif
}

#define APP_CTRL_BOUNDARY_MOVE_SPEED_US        1200
#define APP_CTRL_BOUNDARY_MOVE_TOLERANCE_DEG10 5
#define APP_CTRL_BOUNDARY_PAN_GUARD_MM         300

static void app_ctrl_radar_boundary_move_to_point(u8 point_index)
{
#if (UI_STEP_MOTOR_ENABLE)
    s32 x_mm = g_radar_boundary_x[point_index];
    s32 y_mm = g_radar_boundary_y[point_index];
    // app_radar_get_boundary_quad_by_index(point_index, &x_mm, &y_mm);
    s32 height_mm = 0;
    app_radar_get_install_height_mm(&height_mm);

    // s16 tilt_deg10 = app_radar_height_to_tilt_deg10(height_mm, y_mm);
    s16 pan_deg10  = 0;
    s16 tilt_deg10 = 0;
    BLE_LOG_D("x: %d, y: %d, h: %d", x_mm, y_mm, height_mm);
    app_radar_point_to_pan_tilt(x_mm, y_mm, height_mm, &pan_deg10, &tilt_deg10);
    BLE_LOG_D("p: %d, t: %d", pan_deg10, tilt_deg10);
    StepMotor_GimbalSetSpeedUs(APP_CTRL_BOUNDARY_MOVE_SPEED_US);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, pan_deg10);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, tilt_deg10);
#else
    (void)point_index;
#endif
}

static u8 app_ctrl_radar_boundary_is_move_done(u8 point_index)
{
#if (UI_STEP_MOTOR_ENABLE)
    s32 x_mm = 0;
    s32 y_mm = 0;
    app_radar_get_boundary_quad_by_index(point_index, &x_mm, &y_mm);
    s32 height_mm = 0;
    app_radar_get_install_height_mm(&height_mm);

    s16 pan_target  = 0;
    s16 tilt_target = 0;
    app_radar_point_to_pan_tilt(x_mm, y_mm, height_mm, &pan_target, &tilt_target);
    BLE_LOG_D("pan_target: %d, tilt_target: %d", pan_target, tilt_target);
    s16 pan_cur  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
    s16 tilt_cur = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
    BLE_LOG_D("pan_cur: %d, tilt_cur: %d", pan_cur, tilt_cur);
    if ((abs(pan_cur - pan_target) <= APP_CTRL_BOUNDARY_MOVE_TOLERANCE_DEG10) &&
        (abs(tilt_cur - tilt_target) <= APP_CTRL_BOUNDARY_MOVE_TOLERANCE_DEG10) &&
        !StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) &&
        !StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
    {
        return 1;
    }
#endif
    (void)point_index;
    return 1;
}

static void app_ctrl_radar_boundary_store_point(u8 point_index, s16 x_mm, s16 y_mm)
{
    g_radar_boundary_x[point_index] = (s32)x_mm;
    g_radar_boundary_y[point_index] = (s32)y_mm;
    g_radar_boundary_point_mask |= (u8)(1u << point_index);
}

#endif

// ----------------------- helper: LED control -----------------------
static void app_ctrl_led_set(u8 ledId, u8 state)
{
#if (UI_LED_ENABLE)
    u8 level = (state ? LED_ON_LEVEL : !LED_ON_LEVEL);

    switch (ledId)
    {
    case 0:  // all
        gpio_write(GPIO_LED_BLUE, level);
        gpio_write(GPIO_LED_GREEN, level);
        gpio_write(GPIO_LED_RED, level);
        break;
    case 1:
        gpio_write(GPIO_LED_BLUE, level);
        break;
    case 2:
        gpio_write(GPIO_LED_GREEN, level);
        break;
    case 3:
        gpio_write(GPIO_LED_WHITE, level);
        break;
    case 4:
        gpio_write(GPIO_LED_RED, level);
        break;
    default:
        break;
    }
#else
    (void)ledId;
    (void)state;
#endif
}

void app_ctrl_notify_power_rejected_battery_temp_high(void)
{
    if (BLS_CONN_HANDLE == 0xFFFF)
    {
        return;
    }

    u8 pl[3] = {CTRL_STATUS_REJECT_ERROR, 0, CTRL_REASON_BATTERY_TEMP_HIGH};
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_POWER_CTRL, g_ctrlSeq++, pl, sizeof(pl));
}

// ----------------------- sending -----------------------
int app_ctrl_send(u8 msgType, u8 cmdId, u8 seq, u8 *payload, u16 payloadLen)
{
    u16 headerLen = 1 + 1 + 1 + 1 + 2;  // version + msgType + cmdId + seq + payloadLen
    u16 totalLen  = headerLen + payloadLen;

    if (totalLen > CTRL_TX_MAX_LEN)
    {
        // payload too long for one ATT Value, do not send and report error to caller
        return -1;
    }

    u8 *p = g_ctrlTxBuf;
    p[0]  = CTRL_PROTO_VERSION;
    p[1]  = msgType;
    p[2]  = cmdId;
    p[3]  = seq;
    p[4]  = U16_LO(payloadLen);
    p[5]  = U16_HI(payloadLen);

    if (payloadLen && payload)
    {
        memcpy(p + headerLen, payload, payloadLen);
    }
#if (DEBUG_MODE)
    {
        // tl_printf("app_ctrl_send");
        // for (u8 i = 0; i < totalLen; i++)
        // {
        //     tl_printf("0x%01x ", p[i]);
        // }
        // tl_printf("\r\n");
    }
#endif
    sleep_us(10000);
    if (BLS_CONN_HANDLE != 0xFFFF)
    {
        blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, CUSTOM_COUNTER_READ_DP_H, g_ctrlTxBuf, totalLen);
    }
    // memset(g_ctrlRxBuf, 0, sizeof(g_ctrlRxBuf));
    // memset(g_ctrlTxBuf, 0, sizeof(g_ctrlTxBuf));
    return 0;
}

// ----------------------- direct text/bytes to BLE (EVENT: CTRL_CMD_TEXT_CHUNK) -----------------------
// Send arbitrary bytes/text to PC visualizer via Ctrl TX notify (chunked to fit 20-byte ATT value).
static u8 g_textTxTransferId = 0;

void app_ctrl_text_send_bytes(const u8 *data, u16 len)
{
    if (!data || len == 0)
    {
        return;
    }
    if (BLS_CONN_HANDLE == 0xFFFF)
    {
        return;
    }

    u8 maxData = CTRL_TEXT_CHUNK_DATA_MAX;
    u8 total   = (u8)((len + maxData - 1) / maxData);
    if (total == 0)
    {
        total = 1;
    }

    u8 transferId = g_textTxTransferId++;

    for (u8 idx = 0; idx < total; idx++)
    {
        u16 off = (u16)idx * (u16)maxData;
        u16 rem = (len > off) ? (len - off) : 0;
        u8  dln = (rem > maxData) ? maxData : (u8)rem;

        // payload: [0]=transferId, [1]=chunkIndex, [2]=chunkTotal, [3]=dataLen, [4..]=data
        u8 pl[4 + CTRL_TEXT_CHUNK_DATA_MAX];
        pl[0] = transferId;
        pl[1] = idx;
        pl[2] = total;
        pl[3] = dln;
        if (dln)
        {
            memcpy(&pl[4], data + off, dln);
        }
        app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_TEXT_CHUNK, g_ctrlSeq++, pl, (u16)(4 + dln));

        // Space NOTIFYs so the stack copies g_ctrlTxBuf each time (same buffer for all sends).
        if (idx + 1 < total)
        {
            sleep_us(5000);
        }
    }
    sleep_us(10000);
}

void app_ctrl_radar_dbg_send_prev_raw(s16 prev_x, s16 prev_y, s16 raw_x, s16 raw_y, u8 motion_valid, s16 motion_dir_deg10)
{
    /* Binary EVENT on ctrl TX:
     * cmdId = CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY (0x57), payload:
     * [0]=sub(CTRL_RADAR_DBG_SUB_PREV_RAW),
     * [1..2]=prev_x(s16 LE), [3..4]=prev_y(s16 LE),
     * [5..6]=raw_x(s16 LE),  [7..8]=raw_y(s16 LE),
     * [9]=motion_valid(u8),  [10..11]=motion_dir_deg10(s16 LE, 0 if invalid). */
    u8  pl[12];
    s16 mdir = motion_valid ? motion_dir_deg10 : 0;
    pl[0]    = CTRL_RADAR_DBG_SUB_PREV_RAW;
    pl[1]    = (u8)(prev_x & 0xFF);
    pl[2]    = (u8)((prev_x >> 8) & 0xFF);
    pl[3]    = (u8)(prev_y & 0xFF);
    pl[4]    = (u8)((prev_y >> 8) & 0xFF);
    pl[5]    = (u8)(raw_x & 0xFF);
    pl[6]    = (u8)((raw_x >> 8) & 0xFF);
    pl[7]    = (u8)(raw_y & 0xFF);
    pl[8]    = (u8)((raw_y >> 8) & 0xFF);
    pl[9]    = motion_valid ? 1 : 0;
    pl[10]   = (u8)(mdir & 0xFF);
    pl[11]   = (u8)((mdir >> 8) & 0xFF);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
}

void app_ctrl_radar_dbg_send_pred_sta(s16 ax_mm, s16 ay_mm, s16 bx_mm, s16 by_mm)
{
    /* payload: [0]=sub, [1..8]=ax,ay,bx,by (s16 LE) */
    u8 pl[9];
    pl[0] = CTRL_RADAR_DBG_SUB_PRED_STA;
    pl[1] = (u8)(ax_mm & 0xFF);
    pl[2] = (u8)((ax_mm >> 8) & 0xFF);
    pl[3] = (u8)(ay_mm & 0xFF);
    pl[4] = (u8)((ay_mm >> 8) & 0xFF);
    pl[5] = (u8)(bx_mm & 0xFF);
    pl[6] = (u8)((bx_mm >> 8) & 0xFF);
    pl[7] = (u8)(by_mm & 0xFF);
    pl[8] = (u8)((by_mm >> 8) & 0xFF);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
}

void app_ctrl_radar_dbg_send_predseq(u8 idx, s16 x_mm, s16 y_mm)
{
    /* payload: [0]=sub, [1]=idx, [2..3]=x_mm(s16 LE), [4..5]=y_mm(s16 LE) */
    u8 pl[6];
    pl[0] = CTRL_RADAR_DBG_SUB_PREDSEQ;
    pl[1] = idx;
    pl[2] = (u8)(x_mm & 0xFF);
    pl[3] = (u8)((x_mm >> 8) & 0xFF);
    pl[4] = (u8)(y_mm & 0xFF);
    pl[5] = (u8)((y_mm >> 8) & 0xFF);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
}

static void app_ctrl_radar_dbg_send_boundary_pt(u8 corner_idx, s32 x_mm, s32 y_mm)
{
    /* 避免在 BLE/ATT 回调上下文里做大栈格式化输出导致异常复位：
     * 改为用二进制 EVENT 上报（sub=BOUNDARY_PT），由上位机/APP解析。 */
    s16 x16 = (x_mm > 32767) ? 32767 : (x_mm < -32768 ? -32768 : (s16)x_mm);
    s16 y16 = (y_mm > 32767) ? 32767 : (y_mm < -32768 ? -32768 : (s16)y_mm);

    /* payload[0]=sub, [1]=corner, [2..3]=x_mm(s16 LE), [4..5]=y_mm(s16 LE) */
    u8 pl[6];
    pl[0] = CTRL_RADAR_DBG_SUB_BOUNDARY_PT;
    pl[1] = corner_idx;
    pl[2] = (u8)(x16 & 0xFF);
    pl[3] = (u8)((x16 >> 8) & 0xFF);
    pl[4] = (u8)(y16 & 0xFF);
    pl[5] = (u8)((y16 >> 8) & 0xFF);

    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
}

void app_ctrl_radar_dbg_send_boundary_quad_all(void)
{
    u8 i;
    for (i = 0; i < 4; i++)
    {
        s32 x_mm = 0;
        s32 y_mm = 0;
        app_radar_get_boundary_quad_by_index(i, &x_mm, &y_mm);
        app_ctrl_radar_dbg_send_boundary_pt(i, x_mm, y_mm);
        /* Space NOTIFYs so the stack copies g_ctrlTxBuf each time (same buffer for all sends). */
        if (i < 3)
        {
            sleep_us(8000);
        }
    }
}

void app_ctrl_radar_boundary_enter(void)
{
#if (UI_RADAR_ENABLE)
    g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_SETTING;
#endif
}

u8 app_ctrl_is_setting_mode(void)
{
#if (UI_RADAR_ENABLE)
    return (g_radar_boundary_mode == CTRL_RADAR_BOUNDARY_MODE_SETTING);
#else
    return 0;
#endif
}

// ----------------------- command handlers -----------------------

static u32 app_ctrl_speed_to_interval_us(u8 speedLv, u32 defaultIntervalUs)
{
    switch (speedLv)
    {
    case 1:
        return 9000;
    case 2:
        return 12000;
    case 3:
        return 20000;
    default:
        return defaultIntervalUs;
    }
}

static void app_ctrl_calc_xy_from_angles(s32 pan_deg10, s32 tilt_deg10, s32 height_mm, s16 *out_x_mm, s16 *out_y_mm)
{
    BLE_LOG_D("pan_deg10: %d, tilt_deg10: %d, height_mm: %d", pan_deg10, tilt_deg10, height_mm);
    if (!out_x_mm || !out_y_mm)
    {
        return;
    }

    if (tilt_deg10 >= 0)
    {
        *out_x_mm = 0;
        *out_y_mm = 0;
        return;
    }

    s32 r1_mm = height_mm * lookup_tan((900 + tilt_deg10) * DEG_TO_RAD_10);
    s32 x_mm  = r1_mm * lookup_sin(pan_deg10 * DEG_TO_RAD_10);
    s32 y_mm  = r1_mm * lookup_cos(pan_deg10 * DEG_TO_RAD_10);
    BLE_LOG_D("x_mm: %d, y_mm: %d", x_mm, y_mm);

    if (x_mm > 32767)
    {
        x_mm = 32767;
    }
    else if (x_mm < -32768)
    {
        x_mm = -32768;
    }

    if (y_mm > 32767)
    {
        y_mm = 32767;
    }
    else if (y_mm < -32768)
    {
        y_mm = -32768;
    }

    *out_x_mm = (s16)x_mm;
    *out_y_mm = (s16)y_mm;
}

static int app_ctrl_handle_motor_ctrl(u8 seq, u8 *payload, u16 len)
{
    u8 rsp[8] = {CTRL_STATUS_OK, 0, 0, 0, 0, 0, 0, 0};
    u8 rspLen = 2;

#if (UI_STEP_MOTOR_ENABLE)
    if (len < 1)
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_CTRL, seq, rsp, rspLen);
        return -1;
    }

    u8 op = payload[0];

    switch (op)
    {
    case 0x00:  // stop all axis
        StepMotor_StopAll();
        break;

    case 0x01:  // move both axis to target angle
    {
        if (len < 6)
        {
            rsp[0] = CTRL_STATUS_PARAM_ERROR;
            app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_CTRL, seq, rsp, rspLen);
            return -1;
        }

        s16 panDeg10  = (s16)(payload[1] | (payload[2] << 8));
        s16 tiltDeg10 = (s16)(payload[3] | (payload[4] << 8));
        u8  speedLv   = payload[5];
        u32 intervalUs;

        intervalUs = app_ctrl_speed_to_interval_us(speedLv, 1200);

        StepMotor_GimbalSetSpeedUs(intervalUs);
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)panDeg10);
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)tiltDeg10);
        BLE_LOG_D("PAN: %d, TILT: %d,intervalUs: %d", panDeg10, tiltDeg10, intervalUs);
        break;
    }

    case 0x02:  // query current position (x,y)
    {
        s32 panDeg10  = StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
        s32 tiltDeg10 = StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);

        s32 height_mm = 0;
#if (UI_RADAR_ENABLE)
        app_radar_get_install_height_mm(&height_mm);
#endif
        if (height_mm <= 0)
        {
            height_mm = 2500;
        }

        s16 x_mm = 0;
        s16 y_mm = 0;
        app_ctrl_calc_xy_from_angles(panDeg10, tiltDeg10, height_mm, &x_mm, &y_mm);

        rsp[1] = op;
        rsp[2] = U16_LO((u16)x_mm);
        rsp[3] = U16_HI((u16)x_mm);
        rsp[4] = U16_LO((u16)y_mm);
        rsp[5] = U16_HI((u16)y_mm);
        rspLen = 6;
        break;
    }

    default:
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        break;
    }
#else
    (void)payload;
    (void)len;
    rsp[0] = CTRL_STATUS_UNSUPPORTED_CMD;
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_CTRL, seq, rsp, rspLen);
    return (rsp[0] == CTRL_STATUS_OK) ? 0 : -1;
}

static int app_ctrl_handle_motor_dir_ctrl(u8 seq, u8 *payload, u16 len)
{
    u8 rsp[8] = {CTRL_STATUS_OK, 0, 0};
    u8 rspLen = 2;

#if (UI_STEP_MOTOR_ENABLE)
    if (len < 1)
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
        return -1;
    }

    u8 op = payload[0];

    if (op == 0x00)
    {
        StepMotor_StopAll();
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN));
        StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT));
        g_motor_dir_state.active = 0;
        rsp[1]                   = 0x00;
        rsp[2]                   = op;
        rspLen                   = 2;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
        return 0;
    }

    if (op != 0x01)
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
        return -1;
    }

    if (len < 3)
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
        return -1;
    }

    u8 direction = payload[1];
    u8 speedLv   = payload[2];

    step_motor_axis_e axis;
    s32               dirSign;
    switch (direction)
    {
    case 0x00:  // up (tilt +)
        axis    = STEP_MOTOR_AXIS_TILT;
        dirSign = 1;
        break;
    case 0x01:  // down (tilt -)
        axis    = STEP_MOTOR_AXIS_TILT;
        dirSign = -1;
        break;
    case 0x02:  // left (pan -)
        axis    = STEP_MOTOR_AXIS_PAN;
        dirSign = -1;
        break;
    case 0x03:  // right (pan +)
        axis    = STEP_MOTOR_AXIS_PAN;
        dirSign = 1;
        break;
    default:
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
        return -1;
    }

    u32 intervalUs = app_ctrl_speed_to_interval_us(speedLv, 12000);
    StepMotor_GimbalSetSpeedUs(intervalUs);

    s32 curDeg10   = StepMotor_GimbalGetCurrentDeg10(axis);
    s32 limitDeg10 = curDeg10;

    if (axis == STEP_MOTOR_AXIS_PAN)
    {
        limitDeg10 = (dirSign > 0) ? GIMBAL_PAN_LIMIT_DEG10_POS : GIMBAL_PAN_LIMIT_DEG10_NEG;
    }
    else
    {
        limitDeg10 = (dirSign > 0) ? GIMBAL_TILT_LIMIT_DEG10_POS : GIMBAL_TILT_LIMIT_DEG10_NEG;
    }

    StepMotor_GimbalSetTargetDeg10(axis, limitDeg10);

    g_motor_dir_state.active       = 1;
    g_motor_dir_state.direction    = direction;
    g_motor_dir_state.target_deg10 = (s16)limitDeg10;
    g_motor_dir_state.axis         = axis;
    g_motor_dir_state.dir_sign     = dirSign;

    rsp[1] = direction;
    rsp[2] = op;
    rspLen = 3;
#else
    (void)payload;
    (void)len;
    rsp[0] = CTRL_STATUS_UNSUPPORTED_CMD;
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_DIR_CTRL, seq, rsp, rspLen);
    return (rsp[0] == CTRL_STATUS_OK) ? 0 : -1;
}

static int app_ctrl_handle_motor_set_zero(u8 seq, u8 *payload, u16 len)
{
    u8 rsp[4] = {CTRL_STATUS_OK, 0, 0, 0};
    u8 rspLen = 2;

#if (UI_STEP_MOTOR_ENABLE)
    if (len < 1)
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_SET_ZERO, seq, rsp, rspLen);
        return -1;
    }

    u8 axis = payload[0];
    if (axis == 0xFF)
    {
        StepMotor_GimbalSetZeroAll();
        BLE_LOG_D("Set Zero All");
    }
    else if (axis < STEP_MOTOR_AXIS_MAX)
    {
        StepMotor_GimbalSetZero((step_motor_axis_e)axis);
        BLE_LOG_D("Set Zero %d", axis);
    }
    else
    {
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_SET_ZERO, seq, rsp, rspLen);
        return -1;
    }

    rsp[1] = axis;
    rsp[2] = 0;
    rsp[3] = 0;
    rspLen = 4;
#else
    (void)payload;
    (void)len;
    rsp[0] = CTRL_STATUS_UNSUPPORTED_CMD;
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_MOTOR_SET_ZERO, seq, rsp, rspLen);
    return (rsp[0] == CTRL_STATUS_OK) ? 0 : -1;
}

void app_ctrl_motor_dir_task(void)
{
#if (UI_STEP_MOTOR_ENABLE)
    if (!g_motor_dir_state.active)
    {
        return;
    }

    s32 curDeg10 = StepMotor_GimbalGetCurrentDeg10(g_motor_dir_state.axis);
    s32 target   = (s32)g_motor_dir_state.target_deg10;

    if ((g_motor_dir_state.dir_sign > 0 && curDeg10 >= target - 10) ||
        (g_motor_dir_state.dir_sign < 0 && curDeg10 <= target + 10))
    {
        StepMotor_Stop(g_motor_dir_state.axis);

        u8 evt[4] = {0};
        evt[0]    = 0x01;  // reached
        evt[1]    = g_motor_dir_state.direction;

        g_motor_dir_state.active = 0;
        app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_MOTOR_DIR_CTRL, g_ctrlSeq++, evt, 2);
        return;
    }

#if (UI_RADAR_ENABLE)
    if (g_radar_boundary_mode == CTRL_RADAR_BOUNDARY_MODE_SETTING)
    {
        u8 point_index = g_radar_boundary_active_index;
        if (point_index < RADAR_BOUNDARY_POINT_COUNT)
        {
            s16 cur_x_mm  = 0;
            s16 cur_y_mm  = 0;
            s32 height_mm = 0;
            app_radar_get_install_height_mm(&height_mm);
            if (height_mm <= 0)
            {
                height_mm = 2500;
            }

            s16 tilt_deg10 = StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
            if (g_motor_dir_state.axis == STEP_MOTOR_AXIS_TILT &&
                g_motor_dir_state.direction == 0x00 && tilt_deg10 > g_hieght_angle_10)
            {
                StepMotor_Stop(g_motor_dir_state.axis);
                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)tilt_deg10);
                g_motor_dir_state.active = 0;

                u8 evt[4] = {0};
                evt[0]    = 0x03;  // max tilt for 6 m floor reach at install height
                evt[1]    = 0x00;  // direction: up
                evt[2]    = (u8)((u16)tilt_deg10 & 0xFF);
                evt[3]    = (u8)(((u16)tilt_deg10 >> 8) & 0xFF);
                app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_MOTOR_DIR_CTRL, g_ctrlSeq++, evt, sizeof(evt));
                return;
            }

            app_ctrl_calc_xy_from_angles(StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN),
                                         StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT),
                                         height_mm,
                                         &cur_x_mm,
                                         &cur_y_mm);

            u8 direction   = g_motor_dir_state.direction;
            u8 limit_index = 0xFF;

            if (direction == 0x03)
            {  // right
                if (point_index == RADAR_BOUNDARY_POINT_LU)
                {
                    limit_index = RADAR_BOUNDARY_POINT_RU;
                }
                else if (point_index == RADAR_BOUNDARY_POINT_LD)
                {
                    limit_index = RADAR_BOUNDARY_POINT_RD;
                }
            }
            else if (direction == 0x02)
            {  // left
                if (point_index == RADAR_BOUNDARY_POINT_RU)
                {
                    limit_index = RADAR_BOUNDARY_POINT_LU;
                }
                else if (point_index == RADAR_BOUNDARY_POINT_RD)
                {
                    limit_index = RADAR_BOUNDARY_POINT_LD;
                }
            }
            else if (direction == 0x00)
            {  // up
                if (point_index == RADAR_BOUNDARY_POINT_LD)
                {
                    limit_index = RADAR_BOUNDARY_POINT_LU;
                }
                else if (point_index == RADAR_BOUNDARY_POINT_RD)
                {
                    limit_index = RADAR_BOUNDARY_POINT_RU;
                }
            }
            else if (direction == 0x01)
            {  // down
                if (point_index == RADAR_BOUNDARY_POINT_LU)
                {
                    limit_index = RADAR_BOUNDARY_POINT_LD;
                }
                else if (point_index == RADAR_BOUNDARY_POINT_RU)
                {
                    limit_index = RADAR_BOUNDARY_POINT_RD;
                }
            }

            if (limit_index < RADAR_BOUNDARY_POINT_COUNT)
            {
                u8 need_stop = 0;

                if (direction == 0x03)
                {
                    s32 limit_x_mm = g_radar_boundary_x[limit_index];
                    if (cur_x_mm >= (limit_x_mm - APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        BLE_LOG_D("cur_x_mm: %d, limit_x_mm: %d", cur_x_mm, limit_x_mm);
                        need_stop = 1;
                    }
                }
                else if (direction == 0x02)
                {
                    s32 limit_x_mm = g_radar_boundary_x[limit_index];
                    if (cur_x_mm <= (limit_x_mm + APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        BLE_LOG_D("cur_x_mm: %d, limit_x_mm: %d", cur_x_mm, limit_x_mm);
                        need_stop = 1;
                    }
                }
                else if (direction == 0x00)
                {
                    s32 limit_y_mm = g_radar_boundary_y[limit_index];
                    if (cur_y_mm >= (limit_y_mm - APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        BLE_LOG_D("cur_y_mm: %d, limit_y_mm: %d", cur_y_mm, limit_y_mm);
                        need_stop = 1;
                    }
                }
                else if (direction == 0x01)
                {
                    s32 limit_y_mm = g_radar_boundary_y[limit_index];
                    if (cur_y_mm <= (limit_y_mm + APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        BLE_LOG_D("cur_y_mm: %d, limit_y_mm: %d", cur_y_mm, limit_y_mm);
                        need_stop = 1;
                    }
                }

                if (need_stop)
                {
                    StepMotor_Stop(g_motor_dir_state.axis);
                    StepMotor_GimbalSetTargetDeg10(g_motor_dir_state.axis, StepMotor_GimbalGetCurrentDeg10(g_motor_dir_state.axis));
                    g_motor_dir_state.active = 0;

                    u8 evt[4] = {0};
                    evt[0]    = 0x02;  // boundary guard
                    evt[1]    = direction;
                    evt[2]    = point_index;
                    evt[3]    = limit_index;
                    BLE_LOG_D("evt: %d, %d, %d, %d", evt[0], evt[1], evt[2], evt[3]);
                    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_MOTOR_DIR_CTRL, g_ctrlSeq++, evt, 4);
                }
            }
        }
    }
#endif
#endif
}

static int app_ctrl_handle_time_set(u8 seq, u8 *payload, u16 len)
{
    if (len != 5)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TIME_SET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u32 epoch_sec = ((u32)payload[0]) | ((u32)payload[1] << 8) | ((u32)payload[2] << 16) | ((u32)payload[3] << 24);
    s8  tz_q15    = (s8)payload[4];

#if (UI_RADAR_ENABLE)
    app_radar_set_time_from_epoch(epoch_sec, tz_q15);
#else
    (void)epoch_sec;
    (void)tz_q15;
#endif

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TIME_SET, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_power_ctrl(u8 seq, u8 *payload, u16 len)
{
    if (len < 1)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, CTRL_REASON_NONE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_POWER_CTRL, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 target_on = payload[0] ? 1 : 0;
    u8 cur_on    = app_get_power_state() ? 1 : 0;

    // payload: [0]=status, [1]=on_effective, [2]=reason
    u8 status       = CTRL_STATUS_OK;
    u8 on_effective = target_on;
    u8 reason       = CTRL_REASON_NONE;

    // 低电量禁止开机：bat_percent < 20 且未充电
    if (target_on && !cur_on)
    {
        u8 bat_percent = app_adc_dbg_get_bat_percent_exact();
        u8 is_charging = app_adc_dbg_is_charging() ? 1 : 0;
        if (bat_percent < 20)
        {
            status       = CTRL_STATUS_REJECT_ERROR;
            reason       = CTRL_REASON_LOW_BATTERY;
            on_effective = 0;
        }
    }

    // 电池温度过高禁止开机（>70°C；此时充电开关已由固件关闭）
    if (status == CTRL_STATUS_OK && target_on && !cur_on)
    {
        if (app_adc_dbg_is_ntc_temp_valid() && app_adc_dbg_get_ntc_temp_c() > 70)
        {
            status       = CTRL_STATUS_REJECT_ERROR;
            reason       = CTRL_REASON_BATTERY_TEMP_HIGH;
            on_effective = 0;
        }
    }

    // 冷却限制：开机/关机都限制 30s
    // 仅在“实际要改变电源状态”时生效：
    // - cur_on=0 且 target_on=1：禁止在离上次关机不足 30s 内开机
    // - cur_on=1 且 target_on=0：禁止在离上次开机不足 30s 内关机
    if (status == CTRL_STATUS_OK && target_on != cur_on)
    {
        if (target_on)
        {
            if (g_power_off_tick && !clock_time_exceed(g_power_off_tick, POWER_CTRL_OFF_COOLDOWN_US))
            {
                status       = CTRL_STATUS_REJECT_ERROR;
                reason       = CTRL_REASON_POWER_ON_COOLDOWN_30S;
                on_effective = cur_on;  // 被拒绝则回显当前真实状态
            }
        }
        else
        {
            if (g_power_on_tick && !clock_time_exceed(g_power_on_tick, POWER_CTRL_OFF_COOLDOWN_US))
            {
                status       = CTRL_STATUS_REJECT_ERROR;
                reason       = CTRL_REASON_POWER_OFF_COOLDOWN_30S;
                on_effective = cur_on;  // 被拒绝则回显当前真实状态
            }
        }
    }

    if (status != CTRL_STATUS_OK)
    {
        u8 rsp[3] = {status, on_effective, reason};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_POWER_CTRL, seq, rsp, sizeof(rsp));
        return -1;
    }

    // 允许：执行切换
    if (on_effective != cur_on)
    {
        if (on_effective)
        {
            g_power_on_tick  = clock_time();
            g_power_off_tick = 0;
        }
        else
        {
            g_power_off_tick = clock_time();
        }
    }

    LOG_D("pc: %d  payload: %d", on_effective, payload[0]);
    app_set_power_state(on_effective);

    u8 rsp[3] = {CTRL_STATUS_OK, on_effective, CTRL_REASON_NONE};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_POWER_CTRL, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_status_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    if (len != 0)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_STATUS_GET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 rsp[10] = {CTRL_STATUS_OK, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    rsp[1]     = app_get_power_state() ? 1 : 0;
#if (UI_RADAR_ENABLE)
    rsp[2]        = 1;
    s32 height_mm = 0;
    app_radar_get_install_height_mm(&height_mm);
    rsp[3] = (U16_LO((s16)height_mm));
    rsp[4] = (U16_HI((s16)height_mm));
    rsp[5] = app_adc_dbg_is_charging() ? 1 : 0;
    app_ctrl_calc_exclusive_mode_flags(&rsp[7], &rsp[8], &rsp[9], &rsp[6]);
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_STATUS_GET, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_play_record_get(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len != 0 && len != 1)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
        return -1;
    }

    // APP 确认已成功收到当前记录，推进到下一条或结束
    (void)payload;

    if (g_play_upload_state == PLAY_UPLOAD_WAIT_ACK)
    {
        // 清除掉上次发送的那一条记录
        if (g_play_cache_index + 1 < g_play_cache_total)
        {
            // 还有下一条：调度 1s 后发送
            g_play_cache_index++;
            g_play_upload_tick  = clock_time();
            g_play_upload_state = PLAY_UPLOAD_SEND_WAIT;
            BLE_LOG_D("play record ack, next %d/%d", g_play_cache_index + 1, g_play_cache_total);
        }
        else
        {
            // 所有记录已发送完毕
            g_play_cache_total  = 0;
            g_play_cache_index  = 0;
            g_play_upload_state = PLAY_UPLOAD_IDLE;
            BLE_LOG_D("play record all done");
        }
    }

    u8 remaining = 0;
    if (g_play_upload_state != PLAY_UPLOAD_IDLE)
    {
        remaining = g_play_cache_total - g_play_cache_index - 1;
    }

    u8 rsp[2] = {CTRL_STATUS_OK, remaining};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static int app_ctrl_handle_uid_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    if (len != 0)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_UID_GET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 uid[16] = {0};
    app_get_flash_uid(uid, sizeof(uid));

    for (u8 part = 0; part < 2; part++)
    {
        u8 rsp[10] = {CTRL_STATUS_OK, part, 0, 0, 0, 0, 0, 0, 0, 0};
        u8 base    = (u8)(part * 8);
        for (u8 i = 0; i < 8; i++)
        {
            rsp[2 + i] = uid[base + i];
        }

        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_UID_GET, seq, rsp, sizeof(rsp));
    }

    return 0;
}

// ----------------------- 新简化配置接口 handler -----------------------

/**
 * @brief 处理设置高度并进入配置模式命令（CMD = 0x59）
 *
 * 设备行为：
 * 1. 将高度限制在 500~10000mm 范围内
 * 2. 缓存高度值（不立即生效）
 * 3. 自动进入配置模式（SETTING状态）
 * 4. 清空之前缓存的坐标点数据
 */
static int app_ctrl_handle_radar_config_set_height(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 2)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_CONFIG_SET_HEIGHT, seq, rsp, sizeof(rsp));
        return -1;
    }

    s16 height_mm = (s16)(payload[0] | (payload[1] << 8));

    // 限制高度范围 800~2500mm
    if (height_mm < 800)
    {
        height_mm = 800;
    }
    else if (height_mm > 2500)
    {
        height_mm = 2500;
    }
    // 应用高度
    app_radar_set_install_height_mm(height_mm);

    BLE_LOG_D("RADAR_CONFIG_SET_HEIGHT: cached_height=%d mm", height_mm);

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_CONFIG_SET_HEIGHT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_CONFIG_SET_HEIGHT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- long text chunk handler -----------------------
// 设计用于发送长文本，采用分片方式：
// payload: [0]=transferId, [1]=chunkIndex, [2]=chunkTotal, [3]=dataLen, [4..] data
// 限制: 6(头) + 4(分片字段) + dataLen <= 20 => dataLen <= CTRL_TEXT_CHUNK_DATA_MAX

// 简单实现：使用一个固定长度缓冲区重组文本（防止占用过多 RAM）
#define CTRL_TEXT_MAX_TOTAL_LEN 20
static u8  g_textBuf[CTRL_TEXT_MAX_TOTAL_LEN];
static u16 g_textLen            = 0;
static u8  g_textTransferId     = 0xFF;
static u8  g_textExpectedChunks = 0;
static u8  g_textReceivedChunks = 0;

static void app_ctrl_text_reset(void)
{
    g_textLen            = 0;
    g_textTransferId     = 0xFF;
    g_textExpectedChunks = 0;
    g_textReceivedChunks = 0;
    memset(g_textBuf, 0, sizeof(g_textBuf));
}

static int app_ctrl_handle_text_chunk(u8 seq, u8 *payload, u16 len)
{
    if (len < 4)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TEXT_CHUNK, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 transferId = payload[0];
    u8 chunkIndex = payload[1];
    u8 chunkTotal = payload[2];
    u8 dataLen    = payload[3];

    if ((u16)(4 + dataLen) > len || dataLen > CTRL_TEXT_CHUNK_DATA_MAX)
    {
        u8 rsp[3] = {CTRL_STATUS_LEN_ERROR, transferId, chunkIndex};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TEXT_CHUNK, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 *data = &payload[4];

    // 首片：初始化重组状态
    if (chunkIndex == 0)
    {
        app_ctrl_text_reset();
        g_textTransferId     = transferId;
        g_textExpectedChunks = chunkTotal;
    }

    // 检查 transferId 一致性
    if (g_textTransferId != transferId)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, transferId, chunkIndex};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TEXT_CHUNK, seq, rsp, sizeof(rsp));
        return -1;
    }

    // 检查是否超出总长度缓冲区
    if ((u16)(g_textLen + dataLen) > CTRL_TEXT_MAX_TOTAL_LEN)
    {
        u8 rsp[3] = {CTRL_STATUS_LEN_ERROR, transferId, chunkIndex};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TEXT_CHUNK, seq, rsp, sizeof(rsp));
        app_ctrl_text_reset();
        return -1;
    }

    // 追加本片数据
    memcpy(g_textBuf + g_textLen, data, dataLen);
    g_textLen += dataLen;
    g_textReceivedChunks++;

    // 应答当前分片
    u8 rsp[3] = {CTRL_STATUS_OK, transferId, chunkIndex};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_TEXT_CHUNK, seq, rsp, sizeof(rsp));

    // 若已接收完所有分片，认为文本完整，可以进行业务处理
    if (g_textReceivedChunks == g_textExpectedChunks)
    {
        // 确保以 '\0' 结束，便于日志打印（如超出缓冲尾则已在上面截断返回错误）
        if (g_textLen < CTRL_TEXT_MAX_TOTAL_LEN)
        {
            g_textBuf[g_textLen] = 0;
        }
        else
        {
            g_textBuf[CTRL_TEXT_MAX_TOTAL_LEN - 1] = 0;
        }

        BLE_LOG_D("[CTRL][TEXT] id=%d, len=%d, text=\"%s\"",
                  g_textTransferId,
                  g_textLen,
                  g_textBuf);

        // 文本处理完后重置状态，等待下一次传输
        app_ctrl_text_reset();
    }

    return 0;
}

// ----------------------- handler: radar reset flash config -----------------------
static int app_ctrl_handle_radar_reset_flash_config(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
    app_radar_clear_install_height_and_boundary_flash();
    u8 rsp[1] = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_RESET_FLASH_CONFIG, seq, rsp, sizeof(rsp));
    return 0;
}

// ----------------------- handler: radar track speed -----------------------
static int app_ctrl_handle_radar_track_speed(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 2)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
        return -1;
    }
    u16 us = payload[0] | (payload[1] << 8);
    app_radar_set_track_gimbal_interval_us((u32)us);
    u8 rsp[3] = {CTRL_STATUS_OK, payload[0], payload[1]};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: radar debug get boundary -----------------------
static int app_ctrl_handle_radar_debug_get_boundary(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
#if (UI_RADAR_ENABLE)
    app_ctrl_radar_dbg_send_boundary_quad_all();
    return 0;
#else
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt settings enter -----------------------
static int app_ctrl_handle_hunt_settings_enter(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len != 0)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_ENTER, seq, rsp, sizeof(rsp));
        return -1;
    }
    // 进入狩猎设置模式
    app_ctrl_radar_boundary_enter();  // 复用边界设置模式标志
    // 光斑位于当前猎物点
    s16 pan = 0, tilt = 0;
    app_hunt_get_prey_point_deg10(&pan, &tilt);
    StepMotor_GimbalSetSpeedUs(1200);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, (s32)pan);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, (s32)tilt);
    u8 rsp[1] = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_ENTER, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_ENTER, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt settings exit -----------------------
static int app_ctrl_handle_hunt_settings_exit(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_EXIT, seq, rsp, sizeof(rsp));
        return -1;
    }
    u8 apply = payload[0];
    if (apply)
    {
        BLE_LOG_D("hunt settings applied");
    }
    // 清除设置模式
    g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_IDLE;
    u8 rsp[1]             = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_EXIT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_EXIT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt prey random -----------------------
static int app_ctrl_handle_hunt_prey_random(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_PREY_RANDOM, seq, rsp, sizeof(rsp));
        return -1;
    }
    u8 start = payload[0];
    if (start)
    {
        app_hunt_prey_random_move();
    }
    // 停止随机移动时不需额外动作; 当前停止后光斑停在当前位置
    u8 rsp[2] = {CTRL_STATUS_OK, start};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_PREY_RANDOM, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_PREY_RANDOM, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt prey set -----------------------
static int app_ctrl_handle_hunt_prey_set(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
#if (UI_RADAR_ENABLE) && (UI_STEP_MOTOR_ENABLE)
    // 当前云台位置设为猎物点
    s16 pan  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
    s16 tilt = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
    app_hunt_set_prey_point_deg10(pan, tilt);
    BLE_LOG_D("prey point set: pan=%d, tilt=%d", pan, tilt);
    u8 rsp[1] = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_PREY_SET, seq, rsp, sizeof(rsp));
    return 0;
#else
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_PREY_SET, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt set duration -----------------------
static int app_ctrl_handle_hunt_set_duration(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 2)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_DURATION, seq, rsp, sizeof(rsp));
        return -1;
    }
    u16 dur_s = (u16)(payload[0] | (payload[1] << 8));
    app_hunt_set_duration_s(dur_s);
    BLE_LOG_D("hunt duration set: %d s", app_hunt_get_duration_s());
    u8 rsp[3] = {CTRL_STATUS_OK, (u8)(app_hunt_get_duration_s() & 0xFF), (u8)((app_hunt_get_duration_s() >> 8) & 0xFF)};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_DURATION, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_DURATION, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt set count -----------------------
static int app_ctrl_handle_hunt_set_count(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_COUNT, seq, rsp, sizeof(rsp));
        return -1;
    }
    app_hunt_set_count(payload[0]);
    BLE_LOG_D("hunt count set: %d", app_hunt_get_count());
    u8 rsp[2] = {CTRL_STATUS_OK, app_hunt_get_count()};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_COUNT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_COUNT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt set sleep duration -----------------------
static int app_ctrl_handle_hunt_set_sleep_duration(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_SLEEP_DURATION, seq, rsp, sizeof(rsp));
        return -1;
    }
    app_hunt_set_sleep_duration_min(payload[0]);
    BLE_LOG_D("sleep duration set: %d min", app_hunt_get_sleep_duration_min());
    u8 rsp[2] = {CTRL_STATUS_OK, app_hunt_get_sleep_duration_min()};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_SLEEP_DURATION, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SET_SLEEP_DURATION, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt settings get -----------------------
static int app_ctrl_handle_hunt_settings_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
#if (UI_RADAR_ENABLE)
    u16 dur_s = app_hunt_get_duration_s();
    u8  count = app_hunt_get_count();
    u8  sleep = app_hunt_get_sleep_duration_min();
    u8  rsp[5] = {CTRL_STATUS_OK, (u8)(dur_s & 0xFF), (u8)((dur_s >> 8) & 0xFF), count, sleep};
    BLE_LOG_D("hunt settings get: dur=%d count=%d sleep=%d", dur_s, count, sleep);
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_GET, seq, rsp, sizeof(rsp));
    return 0;
#else
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_GET, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: device reboot -----------------------
static int app_ctrl_handle_device_reboot(u8 seq, u8 *payload, u16 len)
{
    if (len != 0)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_DEVICE_REBOOT, seq, rsp, sizeof(rsp));
        return -1;
    }
    u8 rsp[1] = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_DEVICE_REBOOT, seq, rsp, sizeof(rsp));
    s_ctrl_reboot_pending = 1;
    s_ctrl_reboot_tick    = clock_time();
    return 0;
}

// ----------------------- public APIs -----------------------
void app_ctrl_init(void)
{
    memset(g_ctrlRxBuf, 0, sizeof(g_ctrlRxBuf));
    memset(g_ctrlTxBuf, 0, sizeof(g_ctrlTxBuf));
    g_ctrlSeq = 0;
#if (UI_STEP_MOTOR_ENABLE)
    memset(&g_motor_dir_state, 0, sizeof(g_motor_dir_state));
#endif
#if (UI_RADAR_ENABLE)
    radar_boundary_load_from_flash(g_radar_boundary_x, g_radar_boundary_y);

    s32 h_mm = 0;
    app_radar_get_install_height_mm(&h_mm);
    if (h_mm <= 0)
    {
        h_mm = 2500;
    }
    g_hieght_angle_10 =
        (s16)(lookup_atan2(6000, h_mm) * RAD_TO_DEG * 10.0f - 900.0f);

    // 初始化狩猎游戏默认参数
    s16 pan = 0, tilt = 0;
    app_hunt_get_prey_point_deg10(&pan, &tilt);
#endif
}

void app_ctrl_on_ble_connected(void)
{
#if (UI_RADAR_ENABLE)
    BLE_LOG_D("app_ctrl_on_ble_connected");
    g_play_record_delay_active     = 1;
    g_play_record_delay_start_tick = clock_time();
#endif
}

void app_ctrl_notify_play_record_changed(void)
{
#if (UI_RADAR_ENABLE)
    app_ctrl_try_upload_play_records();
#endif
}

void app_ctrl_task(void)
{
#if (UI_RADAR_ENABLE)
    // 连接后延迟触发首次上传
    if (g_play_record_delay_active && BLS_CONN_HANDLE != 0xFFFF &&
        clock_time_exceed(g_play_record_delay_start_tick, PLAY_RECORD_UPLOAD_DELAY_AFTER_CONN_US) && !StepMotor_GimbalResetBusy())
    {
        LOG_D("app_ctrl_task upload play records");
        app_ctrl_try_upload_play_records();
    }

    // 逐条上传：每 1s 发送一条记录
    if (g_play_upload_state == PLAY_UPLOAD_SEND_WAIT &&
        clock_time_exceed(g_play_upload_tick, PLAY_RECORD_UPLOAD_INTERVAL_US))
    {
        app_ctrl_upload_one_record_from_cache();
    }

    // ACK 超时重传：5s 未收到 ACK 则重新发送当前记录
    if (g_play_upload_state == PLAY_UPLOAD_WAIT_ACK &&
        clock_time_exceed(g_play_upload_tick, PLAY_RECORD_UPLOAD_ACK_TIMEOUT_US))
    {
        BLE_LOG_D("play record ack timeout, retransmit %d/%d",
                  g_play_cache_index + 1,
                  g_play_cache_total);
        g_play_upload_tick  = clock_time();
        g_play_upload_state = PLAY_UPLOAD_SEND_WAIT;
    }
#endif

    // Soft reboot requested by BLE control command.
    // Do not reboot inside ATT write callback; respond first, then reboot shortly after.
    if (s_ctrl_reboot_pending && clock_time_exceed(s_ctrl_reboot_tick, 120000))
    {
        start_reboot();
    }
}

void app_ctrl_onRx(u8 *data, u16 len)
{
    if (len < 6)
    {
        // too short, ignore
        return;
    }

    u8  version = data[0];
    u8  msgType = data[1];
    u8  cmdId   = data[2];
    u8  seq     = data[3];
    u16 payLen  = data[4] | (data[5] << 8);

#if (DEBUG_MODE)
    tl_printf("app_ctrl_onRx");
    for (u8 i = 0; i < len; i++)
    {
        tl_printf("0x%01x ", data[i]);
    }
    tl_printf("\r\n");
#endif

#if (UI_STEP_MOTOR_ENABLE)
    if (cmdId == CTRL_CMD_MOTOR_DIR_CTRL)
    {
        g_motor_dir_state.active = 0;
    }
#endif

    if (version != CTRL_PROTO_VERSION)
    {
        // unsupported version, reply error
        u8 rsp[2] = {CTRL_STATUS_INTERNAL_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
        return;
    }

    if (msgType != CTRL_MSG_TYPE_CMD)
    {
        // only command supported from APP side
        return;
    }

    if ((u16)(6 + payLen) > len)
    {
        u8 rsp[2] = {CTRL_STATUS_LEN_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
        return;
    }

    u8 *payload = &data[6];

    switch (cmdId)
    {
    case CTRL_CMD_MOTOR_CTRL:
        BLE_LOG_D("CTRL_CMD_MOTOR_CTRL");
        app_ctrl_handle_motor_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_MOTOR_SET_ZERO:
        BLE_LOG_D("CTRL_CMD_MOTOR_SET_ZERO");
        app_ctrl_handle_motor_set_zero(seq, payload, payLen);
        break;
    case CTRL_CMD_MOTOR_DIR_CTRL:
        BLE_LOG_D("CTRL_CMD_MOTOR_DIR_CTRL");
        app_ctrl_handle_motor_dir_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_TIME_SET:
        BLE_LOG_D("CTRL_CMD_TIME_SET");
        app_ctrl_handle_time_set(seq, payload, payLen);
        break;
    case CTRL_CMD_PLAY_RECORD_GET:
        BLE_LOG_D("CTRL_CMD_PLAY_RECORD_GET");
        app_ctrl_handle_play_record_get(seq, payload, payLen);
        break;
    case CTRL_CMD_UID_GET:
        BLE_LOG_D("CTRL_CMD_UID_GET");
        app_ctrl_handle_uid_get(seq, payload, payLen);
        break;
    case CTRL_CMD_POWER_CTRL:
        BLE_LOG_D("CTRL_CMD_POWER_CTRL");
        app_ctrl_handle_power_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_STATUS_GET:
        BLE_LOG_D("CTRL_CMD_STATUS_GET");
        app_ctrl_handle_status_get(seq, payload, payLen);
        break;
    case CTRL_CMD_TEXT_CHUNK:
        BLE_LOG_D("CTRL_CMD_TEXT_CHUNK");
        app_ctrl_handle_text_chunk(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_RESET_FLASH_CONFIG:
        BLE_LOG_D("CTRL_CMD_RADAR_RESET_FLASH_CONFIG");
        app_ctrl_handle_radar_reset_flash_config(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_TRACK_SPEED:
        BLE_LOG_D("CTRL_CMD_RADAR_TRACK_SPEED");
        app_ctrl_handle_radar_track_speed(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY:
        BLE_LOG_D("CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY");
        app_ctrl_handle_radar_debug_get_boundary(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_CONFIG_SET_HEIGHT:
        BLE_LOG_D("CTRL_CMD_RADAR_CONFIG_SET_HEIGHT");
        app_ctrl_handle_radar_config_set_height(seq, payload, payLen);
        break;
    case CTRL_CMD_HUNT_SETTINGS_ENTER:
        BLE_LOG_D("CTRL_CMD_HUNT_SETTINGS_ENTER");
        app_ctrl_handle_hunt_settings_enter(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SETTINGS_EXIT:
        BLE_LOG_D("CTRL_CMD_HUNT_SETTINGS_EXIT");
        app_ctrl_handle_hunt_settings_exit(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_PREY_RANDOM:
        BLE_LOG_D("CTRL_CMD_HUNT_PREY_RANDOM");
        app_ctrl_handle_hunt_prey_random(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_PREY_SET:
        BLE_LOG_D("CTRL_CMD_HUNT_PREY_SET");
        app_ctrl_handle_hunt_prey_set(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SET_DURATION:
        BLE_LOG_D("CTRL_CMD_HUNT_SET_DURATION");
        app_ctrl_handle_hunt_set_duration(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SET_COUNT:
        BLE_LOG_D("CTRL_CMD_HUNT_SET_COUNT");
        app_ctrl_handle_hunt_set_count(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SET_SLEEP_DURATION:
        BLE_LOG_D("CTRL_CMD_HUNT_SET_SLEEP_DURATION");
        app_ctrl_handle_hunt_set_sleep_duration(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SETTINGS_GET:
        BLE_LOG_D("CTRL_CMD_HUNT_SETTINGS_GET");
        app_ctrl_handle_hunt_settings_get(seq, payload, payLen);
        break;

    case CTRL_CMD_DEVICE_REBOOT:
        BLE_LOG_D("CTRL_CMD_DEVICE_REBOOT");
        app_ctrl_handle_device_reboot(seq, payload, payLen);
        break;
    default: {
        u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
        break;
    }
    }
}
