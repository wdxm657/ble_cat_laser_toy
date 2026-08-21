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
static u8  g_ctrlSeq                   = 0;
static u32 g_power_on_tick             = 0;
static u32 g_power_off_tick            = 0;
static u8  g_power_on_cooldown_active  = 0;
static u8  g_power_off_cooldown_active = 0;

// Log TX CCC from app_att.c
extern u8 customCtrlLogCCC[2];

#define POWER_CTRL_COOLDOWN_US 30000000u  // 30s

static volatile u8  s_ctrl_reboot_pending = 0;
static volatile u32 s_ctrl_reboot_tick    = 0;

static void app_ctrl_power_cooldown_task(void)
{
    if (g_power_on_cooldown_active && clock_time_exceed(g_power_on_tick, POWER_CTRL_COOLDOWN_US))
    {
        g_power_on_cooldown_active = 0;
    }

    if (g_power_off_cooldown_active && clock_time_exceed(g_power_off_tick, POWER_CTRL_COOLDOWN_US))
    {
        g_power_off_cooldown_active = 0;
    }
}

static u8 app_ctrl_power_cooldown_active(u8 after_power_on)
{
    app_ctrl_power_cooldown_task();

    return after_power_on ? g_power_on_cooldown_active : g_power_off_cooldown_active;
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

// ----------------------- 简化逗宠记录上传 -----------------------
// 每 1s 检查雷达环形缓冲区是否有完整记录，有则上传最老的一条。
// 上传后等待 ACK（通过 0x35 删除命令确认），超时 2s 重传。
// 不从雷达缓冲区移除记录，直到收到 0x35 删除命令。

enum
{
    PLAY_UPLOAD_IDLE = 0,
    PLAY_UPLOAD_WAIT_ACK,
};

static u8  g_play_upload_state    = PLAY_UPLOAD_IDLE;
static u8  g_play_upload_id       = 0;     // 当前正在上传的记录 ID（0=无）
static u32 g_play_upload_tick     = 0;     // 上次发送/重传时间戳
static u32 g_play_upload_check_tick = 0;   // 上次检查时间戳
static u8  g_play_upload_pending  = 0;     // 标记需要立即检查（由 notify 设置）

#define PLAY_RECORD_UPLOAD_INTERVAL_US    1000000u  // 检查间隔 1s
#define PLAY_RECORD_UPLOAD_ACK_TIMEOUT_US 2000000u  // ACK 超时 2s

/**
 * @brief 从雷达缓冲区读取指定 ID 的记录数据，构建 EVENT 并发送。
 * @return 0=成功, -1=记录不存在
 */
static int app_ctrl_send_play_record(u8 record_id)
{
    u32 start_sec, end_sec, motion_sec;
    u16 avg_speed;
    u8  result;

    if (app_radar_get_record_data_by_id(record_id, &start_sec, &end_sec, &motion_sec, &avg_speed, &result) != 0)
    {
        return -1;
    }

    u32 duration = (end_sec > start_sec && end_sec != 0xFFFFFFFFu) ? (end_sec - start_sec) : 0;
    u16 dur16    = (duration > 0xFFFFu) ? 0xFFFFu : (u16)duration;
    u16 m16      = (motion_sec > 0xFFFFu) ? 0xFFFFu : (u16)motion_sec;
    u8  av8      = (avg_speed > 255u) ? 255u : (u8)avg_speed;

    u8 evt[14] = {0};
    evt[0]     = CTRL_STATUS_OK;
    evt[1]     = record_id;
    evt[2]     = 1;   // total = 1（逐条上传）
    evt[3]     = 0;   // index = 0
    evt[4]     = (u8)(start_sec & 0xFF);
    evt[5]     = (u8)((start_sec >> 8) & 0xFF);
    evt[6]     = (u8)((start_sec >> 16) & 0xFF);
    evt[7]     = (u8)((start_sec >> 24) & 0xFF);
    evt[8]     = (u8)(dur16 & 0xFF);
    evt[9]     = (u8)((dur16 >> 8) & 0xFF);
    evt[10]    = (u8)(m16 & 0xFF);
    evt[11]    = (u8)((m16 >> 8) & 0xFF);
    evt[12]    = av8;
    evt[13]    = result;

    BLE_LOG_D("upload record id=%d start:%d dur:%d mot:%d av:%d res:%d",
              record_id, start_sec, dur16, (u32)m16, (u32)av8, result);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_PLAY_RECORD_GET, g_ctrlSeq++, evt, sizeof(evt));
    return 0;
}

/**
 * @brief 检查并上传最老的完整记录。
 */
static void app_ctrl_play_record_check(void)
{
    if (BLS_CONN_HANDLE == 0xFFFF) return;
    if (!app_ctrl_play_record_upload_allowed()) return;
    if (g_play_upload_state != PLAY_UPLOAD_IDLE) return;

    u8 id = app_radar_find_oldest_complete_record_id();
    if (id == 0) return;

    if (app_ctrl_send_play_record(id) == 0)
    {
        g_play_upload_id    = id;
        g_play_upload_tick  = clock_time();
        g_play_upload_state = PLAY_UPLOAD_WAIT_ACK;
        BLE_LOG_D("play upload start: id=%d", id);
    }
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

static u8 g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_IDLE;

static u8 g_last_charge_state   = 0xFF;
static u8 g_last_setting_state  = 0xFF;
static u8 g_last_hunting_state  = 0xFF;
static u8 g_last_standby_state  = 0xFF;
static u8 g_last_sleeping_state = 0xFF;
static u8 g_last_power_on       = 0xFF;

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
        g_last_power_on       = power_on;
        g_last_charge_state   = charging;
        g_last_setting_state  = setting_mode;
        g_last_hunting_state  = hunting_mode;
        g_last_standby_state  = standby_mode;
        g_last_sleeping_state = sleeping_mode;
        return;
    }

    
    // 状态最多允许1s更新1次，避免过于频繁地通知APP（尤其是充电状态可能会有较大波动）
    if (clock_time_exceed(status_check_tick, 1000000))
    {
        if (charging != g_last_charge_state)
        {
            BLE_LOG_D("charging changed: %d -> %d", g_last_charge_state, charging);
            g_last_charge_state = charging;
            changed             = 1;
        }
        if (power_on != g_last_power_on)
        {
            BLE_LOG_D("power_on changed: %d -> %d", g_last_power_on, power_on);
            g_last_power_on = power_on;
            changed         = 1;
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
    }
    if (changed)
    {
        status_check_tick = clock_time();
        u8 pl[10]         = {CTRL_STATUS_OK, power_on, boundary_set, install_height, install_height_hi, charging, setting_mode, hunting_mode, standby_mode, sleeping_mode};
        app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_STATUS_GET, g_ctrlSeq++, pl, sizeof(pl));
    }
#endif
}

#define APP_CTRL_BOUNDARY_MOVE_SPEED_US        1200
#define APP_CTRL_BOUNDARY_MOVE_TOLERANCE_DEG10 5
#define APP_CTRL_BOUNDARY_PAN_GUARD_MM         300

#endif

// ----------------------- helper: LED control -----------------------
void app_ctrl_notify_power_rejected_battery_temp_high(void)
{
    if (BLS_CONN_HANDLE == 0xFFFF)
    {
        return;
    }

    u8 pl[3] = {CTRL_STATUS_REJECT_ERROR, 0, CTRL_REASON_BATTERY_TEMP_HIGH};
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_POWER_CTRL, g_ctrlSeq++, pl, sizeof(pl));
}

// ----------------------- response cache for duplicate seq -----------------------
#define RESP_CACHE_SIZE 4
typedef struct
{
    u8 used;
    u8 frame[CTRL_TX_MAX_LEN];
    u8 len;
} resp_cache_entry_t;

static resp_cache_entry_t g_resp_cache[RESP_CACHE_SIZE];
static u8                 g_resp_cache_idx = 0;

static void resp_cache_save(const u8 *frame, u8 len, u8 seq)
{
    resp_cache_entry_t *e = &g_resp_cache[g_resp_cache_idx];
    e->used               = 1;
    e->len                = (len <= CTRL_TX_MAX_LEN) ? len : CTRL_TX_MAX_LEN;
    memcpy(e->frame, frame, e->len);
    g_resp_cache_idx = (g_resp_cache_idx + 1) % RESP_CACHE_SIZE;
}

static int resp_cache_resend(u8 cmdId, u8 seq)
{
    for (u8 i = 0; i < RESP_CACHE_SIZE; i++)
    {
        if (g_resp_cache[i].used &&
            g_resp_cache[i].frame[2] == cmdId &&
            g_resp_cache[i].frame[3] == seq)
        {
            if (BLS_CONN_HANDLE != 0xFFFF)
            {
                blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, CUSTOM_COUNTER_READ_DP_H, g_resp_cache[i].frame, g_resp_cache[i].len);
            }
            return 1;
        }
    }
    return 0;
}

// ----------------------- log output via dedicated Log TX characteristic (0x03 UUID) -----------------------
void app_ctrl_log_send_bytes(const u8 *data, u16 len)
{
    if (!data || len == 0)
        return;
    if (BLS_CONN_HANDLE == 0xFFFF)
        return;
    if (!(customCtrlLogCCC[0] & 0x01))
        return;

    u16 maxChunk = 20;
    u16 offset   = 0;
    while (offset < len)
    {
        u16 chunkLen = (len - offset > maxChunk) ? maxChunk : (u16)(len - offset);
        blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, CUSTOM_COUNTER_LOG_DP_H, (u8 *)&data[offset], chunkLen);
        offset += chunkLen;
        if (offset < len)
            sleep_us(5000);
    }
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
    /* 缓存 RSP 帧用于重复 seq 重发 */
    if (msgType == CTRL_MSG_TYPE_RSP)
    {
        resp_cache_save(g_ctrlTxBuf, (u8)totalLen, seq);
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

/** 将环形扇区参数以单个 EVENT 上报（sub=0x05，共 13 字节）。 */
static void app_ctrl_radar_dbg_send_sector_region(void)
{
    const radar_sector_region_t *s = app_radar_get_sector_region();
    if (!s)
        return;

    /* payload[0]=sub(SECTOR), [1..2]=cx(s16 LE), [3..4]=cy(s16 LE),
     * [5..6]=ri(u16 LE), [7..8]=ro(u16 LE),
     * [9..10]=start_deg10(s16 LE), [11..12]=end_deg10(s16 LE) */
    u8 pl[13];
    pl[0]  = CTRL_RADAR_DBG_SUB_SECTOR;
    pl[1]  = (u8)(s->center_x_mm & 0xFF);
    pl[2]  = (u8)((s->center_x_mm >> 8) & 0xFF);
    pl[3]  = (u8)(s->center_y_mm & 0xFF);
    pl[4]  = (u8)((s->center_y_mm >> 8) & 0xFF);
    pl[5]  = (u8)(s->inner_radius_mm & 0xFF);
    pl[6]  = (u8)((s->inner_radius_mm >> 8) & 0xFF);
    pl[7]  = (u8)(s->outer_radius_mm & 0xFF);
    pl[8]  = (u8)((s->outer_radius_mm >> 8) & 0xFF);
    pl[9]  = (u8)(s->angle_start_deg10 & 0xFF);
    pl[10] = (u8)((s->angle_start_deg10 >> 8) & 0xFF);
    pl[11] = (u8)(s->angle_end_deg10 & 0xFF);
    pl[12] = (u8)((s->angle_end_deg10 >> 8) & 0xFF);

    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
}

/** 原四边形 4 角点上报告警——当前已改为环形扇区，用 app_ctrl_radar_dbg_send_sector_region 代替 */
void app_ctrl_radar_dbg_send_boundary_quad_all(void)
{
    app_ctrl_radar_dbg_send_sector_region();
}

/**
 * 发送预测序列点（sub=0x03）。
 * idx: 序列索引，1 表示新序列起始；x_mm, y_mm: 地面坐标 (mm)。
 * payload: [0]=sub(0x03), [1]=idx, [2..3]=x_mm(s16 LE), [4..5]=y_mm(s16 LE) -> 6 B
 */
void app_ctrl_radar_dbg_send_predseq(u8 idx, s16 x_mm, s16 y_mm)
{
    u8 pl[6];
    pl[0] = CTRL_RADAR_DBG_SUB_PREDSEQ;
    pl[1] = idx;
    pl[2] = (u8)(x_mm & 0xFF);
    pl[3] = (u8)((x_mm >> 8) & 0xFF);
    pl[4] = (u8)(y_mm & 0xFF);
    pl[5] = (u8)((y_mm >> 8) & 0xFF);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, g_ctrlSeq++, pl, sizeof(pl));
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

    StepMotor_GimbalSetSpeedUs(1200);

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
    app_ctrl_power_cooldown_task();

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
    // if (target_on && !cur_on)
    // {
    //     u8 bat_percent = app_adc_dbg_get_bat_percent_exact();
    //     u8 is_charging = app_adc_dbg_is_charging() ? 1 : 0;
    //     if (bat_percent < 20)
    //     {
    //         status       = CTRL_STATUS_REJECT_ERROR;
    //         reason       = CTRL_REASON_LOW_BATTERY;
    //         on_effective = 0;
    //     }
    // }

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
            if (app_ctrl_power_cooldown_active(0))
            {
                status       = CTRL_STATUS_REJECT_ERROR;
                reason       = CTRL_REASON_POWER_ON_COOLDOWN_30S;
                on_effective = cur_on;  // 被拒绝则回显当前真实状态
            }
        }
        else
        {
            if (app_ctrl_power_cooldown_active(1))
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
            g_power_on_tick              = clock_time();
            g_power_on_cooldown_active   = 1;
            g_power_off_cooldown_active  = 0;
        }
        else
        {
            g_power_off_tick             = clock_time();
            g_power_off_cooldown_active  = 1;
            g_power_on_cooldown_active   = 0;
        }
    }

    LOG_D("pc: %d  payload: %d", on_effective, payload[0]);
    app_set_power_state(on_effective);
    // 电源状态实际改变时持久化到 FLASH，重新上电后按此标志恢复开关机
    app_save_power_state_to_flash();

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
    BLE_LOG_D("status get: power=%d, height=%d mm, charging=%d, setting=%d, hunting=%d, standby=%d, sleeping=%d", rsp[1], height_mm, rsp[5], rsp[6], rsp[7], rsp[8], rsp[9]);
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_STATUS_GET, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_play_record_delete(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_DELETE, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 record_id = payload[0];
    int ret      = app_radar_delete_play_record_by_id(record_id);

    // ACK 处理：匹配当前正在上传的记录 ID，回到 IDLE 准备上传下一条
    if (g_play_upload_state == PLAY_UPLOAD_WAIT_ACK && g_play_upload_id == record_id)
    {
        g_play_upload_state = PLAY_UPLOAD_IDLE;
        g_play_upload_id    = 0;
        BLE_LOG_D("play record ack+del id=%d", record_id);
    }

    u8 remaining = 0;
    if (g_play_upload_state != PLAY_UPLOAD_IDLE)
    {
        remaining = 1;
    }

    u8 rsp[3] = {ret == 0 ? CTRL_STATUS_OK : CTRL_STATUS_PARAM_ERROR, remaining, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_DELETE, seq, rsp, sizeof(rsp));
    return ret;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_DELETE, seq, rsp, sizeof(rsp));
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

// ----------------------- handler: radar pan offset -----------------------
static int app_ctrl_handle_radar_pan_offset(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 4)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_PAN_OFFSET, seq, rsp, sizeof(rsp));
        return -1;
    }
    s16 pan_offset  = (s16)(payload[0] | (payload[1] << 8));
    s16 tilt_offset = (s16)(payload[2] | (payload[3] << 8));
    app_radar_set_pan_tilt_offset_deg10(pan_offset, tilt_offset);
    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_PAN_OFFSET, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_PAN_OFFSET, seq, rsp, sizeof(rsp));
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
    app_radar_clear_install_height_and_record_flash();
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

/** 固件版本号（大端：MAJOR.MINOR.PATCH） */
#define APP_FIRMWARE_VERSION_MAJOR 1
#define APP_FIRMWARE_VERSION_MINOR 0
#define APP_FIRMWARE_VERSION_PATCH 20
#define APP_FIRMWARE_VERSION       ((APP_FIRMWARE_VERSION_MAJOR << 16) | (APP_FIRMWARE_VERSION_MINOR << 8) | APP_FIRMWARE_VERSION_PATCH)


void app_get_firmware_version(void)
{
    BLE_LOG_D("[APP][VER] %d.%d.%d", APP_FIRMWARE_VERSION_MAJOR, APP_FIRMWARE_VERSION_MINOR, APP_FIRMWARE_VERSION_PATCH);
}

// ----------------------- handler: firmware version get -----------------------
static int app_ctrl_handle_fw_version_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
    u32 ver   = APP_FIRMWARE_VERSION;
    u8  pl[3] = {(u8)(ver & 0xFF), (u8)((ver >> 8) & 0xFF), (u8)((ver >> 16) & 0xFF)};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FW_VERSION_GET, seq, pl, sizeof(pl));
    return 0;
}

/** OTA 状态事件推送（设备主动发送） */
void app_ctrl_send_ota_status(u8 status)
{
    u8 pl[1] = {status};
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_OTA_STATUS_EVENT, 0, pl, sizeof(pl));
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
    BLE_LOG_D("hunt settings enter: prey point pan=%d deg10, tilt=%d deg10", pan, tilt);
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
    app_hunt_prey_save(apply);
    // 清除设置模式
    g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_IDLE;
    u8 rsp[1]             = {CTRL_STATUS_OK};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_EXIT, seq, rsp, sizeof(rsp));
    app_radar_set_enabled(1);  // 退出设置模式后自动启用雷达
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
// start=1: 开始随机移动循环（连续随机点，到达后停 0.5s 自动下一随机点）
// start=0: 停止循环，自动保存当前云台位置为猎物点
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
        // 开始循环随机移动（首次移动由 task 完成）
        app_hunt_prey_random_set_active(1);
    }
    else
    {
        // 停止并自动保存当前位置为猎物点
        app_hunt_prey_random_set_active(0);
    }
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

// ----------------------- handler: hunt settings set (combined) -----------------------
static int app_ctrl_handle_hunt_settings_set(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 4)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_SET, seq, rsp, sizeof(rsp));
        return -1;
    }
    u16 dur_s = (u16)(payload[0] | (payload[1] << 8));
    u8  count = payload[2];
    u8  sleep = payload[3];
    app_hunt_set_duration_s(dur_s);
    app_hunt_set_count(count);
    app_hunt_set_sleep_duration_min(sleep);
    radar_prey_point_cfg_save_to_flash();
    BLE_LOG_D("hunt settings set: dur=%d count=%d sleep=%d",
              app_hunt_get_duration_s(),
              app_hunt_get_count(),
              app_hunt_get_sleep_duration_min());
    u8 rsp[5] = {CTRL_STATUS_OK,
                 (u8)(app_hunt_get_duration_s() & 0xFF),
                 (u8)((app_hunt_get_duration_s() >> 8) & 0xFF),
                 app_hunt_get_count(),
                 app_hunt_get_sleep_duration_min()};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_SET, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_HUNT_SETTINGS_SET, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

// ----------------------- handler: hunt settings get -----------------------
static int app_ctrl_handle_hunt_settings_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    (void)len;
#if (UI_RADAR_ENABLE)
    u16 dur_s  = app_hunt_get_duration_s();
    u8  count  = app_hunt_get_count();
    u8  sleep  = app_hunt_get_sleep_duration_min();
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

// ----------------------- handler: assembly factory test enter -----------------------
static int app_ctrl_handle_factory_test_enter(u8 seq, u8 *payload, u16 len)
{
    if (len == 0)
    {
        u8 rsp[1] = {CTRL_STATUS_OK};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FACTORY_TEST_ENTER, seq, rsp, sizeof(rsp));
        app_factory_test_enter();
        return 0;
    }

    if (len != 2)
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FACTORY_TEST_ENTER, seq, rsp, sizeof(rsp));
        return -1;
    }

    if (!app_factory_test_is_active())
    {
        u8 rsp[1] = {CTRL_STATUS_REJECT_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FACTORY_TEST_ENTER, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 module = payload[0];
    u8 enable = payload[1] ? 1 : 0;

    switch (module)
    {
    case CTRL_FACTORY_TEST_MODULE_RADAR:
        app_factory_test_set_radar(enable);
        break;
    case CTRL_FACTORY_TEST_MODULE_MOTOR:
        app_factory_test_set_motor(enable);
        break;
    case CTRL_FACTORY_TEST_MODULE_LASER:
        app_factory_test_set_laser(enable);
        break;
    default:
    {
        u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FACTORY_TEST_ENTER, seq, rsp, sizeof(rsp));
        return -1;
    }
    }

    u8 rsp[3] = {CTRL_STATUS_OK, module, enable};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_FACTORY_TEST_ENTER, seq, rsp, sizeof(rsp));
    return 0;
}

// ----------------------- public APIs -----------------------
void app_ctrl_init(void)
{
    memset(g_ctrlRxBuf, 0, sizeof(g_ctrlRxBuf));
    memset(g_ctrlTxBuf, 0, sizeof(g_ctrlTxBuf));
    g_ctrlSeq                   = 0;
    g_power_on_tick             = 0;
    g_power_off_tick            = 0;
    g_power_on_cooldown_active  = 0;
    g_power_off_cooldown_active = 0;
#if (UI_STEP_MOTOR_ENABLE)
    memset(&g_motor_dir_state, 0, sizeof(g_motor_dir_state));
#endif
#if (UI_RADAR_ENABLE)
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
    BLE_LOG_D("app_ctrl_notify_play_record_changed");
    g_play_upload_pending = 1;  // 标记下次 app_ctrl_task 立即检查
#endif
}

void app_ctrl_task(void)
{
    app_ctrl_power_cooldown_task();

#if (UI_RADAR_ENABLE)
    u32 now = clock_time();

    // 连接后延迟触发首次检查
    if (g_play_record_delay_active && BLS_CONN_HANDLE != 0xFFFF &&
        clock_time_exceed(g_play_record_delay_start_tick, PLAY_RECORD_UPLOAD_DELAY_AFTER_CONN_US) && !StepMotor_GimbalResetBusy())
    {
        g_play_record_delay_active = 0;
        g_play_upload_pending      = 1;
        BLE_LOG_D("play record: BLE connected, will check");
    }

    // ACK 超时重传（先于 IDLE 检查）
    if (g_play_upload_state == PLAY_UPLOAD_WAIT_ACK &&
        clock_time_exceed(g_play_upload_tick, PLAY_RECORD_UPLOAD_ACK_TIMEOUT_US))
    {
        BLE_LOG_D("play record ack timeout, retransmit id=%d", g_play_upload_id);
        if (app_ctrl_send_play_record(g_play_upload_id) == 0)
        {
            g_play_upload_tick = now;
        }
        else
        {
            // 记录已不存在 → 回到 IDLE
            g_play_upload_state = PLAY_UPLOAD_IDLE;
            g_play_upload_id    = 0;
        }
    }

    // 每 1s 检查是否有新的完整记录需要上传
    if (g_play_upload_state == PLAY_UPLOAD_IDLE)
    {
        if (g_play_upload_pending || clock_time_exceed(g_play_upload_check_tick, PLAY_RECORD_UPLOAD_INTERVAL_US))
        {
            g_play_upload_pending    = 0;
            g_play_upload_check_tick = now;
            app_ctrl_play_record_check();
        }
    }

    // 猎物质点随机移动循环
    app_hunt_prey_random_task();
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

    /* 重复 seq 检测：相同 seq 表示 APP 重发，回复上次缓存的响应 */
    {
        static u8 s_last_seq = 0xFF;
        if (seq == s_last_seq)
        {
            BLE_LOG_D("[DUP_SEQ] cmd=0x%02x seq=%d resend cached response", cmdId, seq);
            if (!resp_cache_resend(cmdId, seq))
            {
                u8 rsp[1] = {CTRL_STATUS_OK};
                app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
            }
            return;
        }
        s_last_seq = seq;
    }

    u8 *payload = &data[6];

    switch (cmdId)
    {
    case CTRL_CMD_MOTOR_DIR_CTRL:
        BLE_LOG_D("CTRL_CMD_MOTOR_DIR_CTRL");
        app_ctrl_handle_motor_dir_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_TIME_SET:
        BLE_LOG_D("CTRL_CMD_TIME_SET");
        app_ctrl_handle_time_set(seq, payload, payLen);
        break;
    case CTRL_CMD_UID_GET:
        BLE_LOG_D("CTRL_CMD_UID_GET");
        app_ctrl_handle_uid_get(seq, payload, payLen);
        break;

    case CTRL_CMD_PLAY_RECORD_DELETE:
        BLE_LOG_D("CTRL_CMD_PLAY_RECORD_DELETE");
        app_ctrl_handle_play_record_delete(seq, payload, payLen);
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
    case CTRL_CMD_RADAR_PAN_OFFSET:
        BLE_LOG_D("CTRL_CMD_RADAR_PAN_OFFSET");
        app_ctrl_handle_radar_pan_offset(seq, payload, payLen);
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

    case CTRL_CMD_HUNT_SETTINGS_SET:
        BLE_LOG_D("CTRL_CMD_HUNT_SETTINGS_SET");
        app_ctrl_handle_hunt_settings_set(seq, payload, payLen);
        break;

    case CTRL_CMD_HUNT_SETTINGS_GET:
        BLE_LOG_D("CTRL_CMD_HUNT_SETTINGS_GET");
        app_ctrl_handle_hunt_settings_get(seq, payload, payLen);
        break;

    case CTRL_CMD_DEVICE_REBOOT:
        BLE_LOG_D("CTRL_CMD_DEVICE_REBOOT");
        app_ctrl_handle_device_reboot(seq, payload, payLen);
        break;
    case CTRL_CMD_FACTORY_TEST_ENTER:
        BLE_LOG_D("CTRL_CMD_FACTORY_TEST_ENTER");
        app_ctrl_handle_factory_test_enter(seq, payload, payLen);
        break;
    case CTRL_CMD_FW_VERSION_GET:
        BLE_LOG_D("CTRL_CMD_FW_VERSION_GET");
        app_ctrl_handle_fw_version_get(seq, payload, payLen);
        break;
    default: {
        u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
        break;
    }
    }
}
