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
#if (UI_RADAR_ENABLE)
#include "app_radar.h"
#endif

#include "SineTable.h"

// RX/TX buffers shared with ATT layer
u8 g_ctrlRxBuf[CTRL_RX_MAX_LEN] = {0};
u8 g_ctrlTxBuf[CTRL_TX_MAX_LEN] = {0};

// simple sequence generator for events/async notifications
static u8 g_ctrlSeq = 0;

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
        LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        LOG_D("y0=%d, y3=%d, y2=%d, y1=%d", y0, y3, y2, y1);
        LOG_D("app_ctrl_radar_boundary_check_order failed: y0 > y3 && y0 > y2 && y1 > y3 && y1 > y2");
        return 0;
    }

    if (!(x0 < x1 && x3 < x2))
    {
        LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        LOG_D("x0=%d, x1=%d, x3=%d, x2=%d", x0, x1, x3, x2);
        LOG_D("app_ctrl_radar_boundary_check_order failed: x0 < x1 && x3 < x2");
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
    LOG_D("app_ctrl_radar_boundary_check_order success");

    if (!app_ctrl_radar_boundary_check_min_lengths(g_radar_boundary_x, g_radar_boundary_y, errDetail, shortPairMask))
    {
        // app_radar_reset_boundary_default();
        g_radar_boundary_ready = 0;
        return 0;
    }
    LOG_D("app_ctrl_radar_boundary_check_min_lengths success");
    app_radar_set_boundary_quad(g_radar_boundary_x, g_radar_boundary_y);
    app_radar_save_boundary_quad_to_flash(g_radar_boundary_x, g_radar_boundary_y);
    g_radar_boundary_ready = 0;
    return 1;
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
    LOG_D("x: %d, y: %d, h: %d", x_mm, y_mm, height_mm);
    LOG_D("x: %d, y: %d, h: %d", x_mm, y_mm, height_mm);
    LOG_D("x: %d, y: %d, h: %d", x_mm, y_mm, height_mm);
    app_radar_point_to_pan_tilt(x_mm, y_mm, height_mm, &pan_deg10, &tilt_deg10);
    LOG_D("p: %d, t: %d", pan_deg10, tilt_deg10);
    LOG_D("p: %d, t: %d", pan_deg10, tilt_deg10);
    LOG_D("p: %d, t: %d", pan_deg10, tilt_deg10);

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
    LOG_D("pan_target: %d, tilt_target: %d", pan_target, tilt_target);
    LOG_D("pan_target: %d, tilt_target: %d", pan_target, tilt_target);
    LOG_D("pan_target: %d, tilt_target: %d", pan_target, tilt_target);
    s16 pan_cur  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
    s16 tilt_cur = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
    LOG_D("pan_cur: %d, tilt_cur: %d", pan_cur, tilt_cur);
    LOG_D("pan_cur: %d, tilt_cur: %d", pan_cur, tilt_cur);
    LOG_D("pan_cur: %d, tilt_cur: %d", pan_cur, tilt_cur);
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
    if (BLS_CONN_HANDLE != 0xFFFF)
    {
        blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, CUSTOM_COUNTER_READ_DP_H, g_ctrlTxBuf, totalLen);
    }

    return 0;
}

#if (UI_RADAR_ENABLE)
static void app_ctrl_radar_dbg_pack_s16(u8 *p, s16 v)
{
    u16 u = (u16)v;
    p[0]  = U16_LO(u);
    p[1]  = U16_HI(u);
}

void app_ctrl_radar_dbg_send_prev_raw(s16 prev_x, s16 prev_y, s16 raw_x, s16 raw_y, u8 motion_valid, s16 motion_dir_deg10)
{
    u8 pl[12];

    pl[0] = CTRL_RADAR_DBG_SUB_PREV_RAW;
    app_ctrl_radar_dbg_pack_s16(pl + 1, prev_x);
    app_ctrl_radar_dbg_pack_s16(pl + 3, prev_y);
    app_ctrl_radar_dbg_pack_s16(pl + 5, raw_x);
    app_ctrl_radar_dbg_pack_s16(pl + 7, raw_y);
    pl[9] = motion_valid ? 1 : 0;
    app_ctrl_radar_dbg_pack_s16(pl + 10, motion_valid ? motion_dir_deg10 : (s16)0);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_PRED_DEBUG, g_ctrlSeq++, pl, sizeof(pl));
}

void app_ctrl_radar_dbg_send_pred_sta(s16 ax_mm, s16 ay_mm, s16 bx_mm, s16 by_mm)
{
    u8 pl[9];

    pl[0] = CTRL_RADAR_DBG_SUB_PRED_STA;
    app_ctrl_radar_dbg_pack_s16(pl + 1, ax_mm);
    app_ctrl_radar_dbg_pack_s16(pl + 3, ay_mm);
    app_ctrl_radar_dbg_pack_s16(pl + 5, bx_mm);
    app_ctrl_radar_dbg_pack_s16(pl + 7, by_mm);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_PRED_DEBUG, g_ctrlSeq++, pl, sizeof(pl));
}

void app_ctrl_radar_dbg_send_predseq(u8 idx, s16 x_mm, s16 y_mm)
{
    u8 pl[6];

    pl[0] = CTRL_RADAR_DBG_SUB_PREDSEQ;
    pl[1] = idx;
    app_ctrl_radar_dbg_pack_s16(pl + 2, x_mm);
    app_ctrl_radar_dbg_pack_s16(pl + 4, y_mm);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_PRED_DEBUG, g_ctrlSeq++, pl, sizeof(pl));
}

static s16 app_ctrl_radar_mm_to_dbg_s16(s32 mm)
{
    if (mm > (s32)32767)
    {
        return (s16)32767;
    }
    if (mm < (s32)-32768)
    {
        return (s16)-32768;
    }
    return (s16)mm;
}

static void app_ctrl_radar_dbg_send_boundary_pt(u8 corner_idx, s32 x_mm, s32 y_mm)
{
    u8 pl[6];

    pl[0] = CTRL_RADAR_DBG_SUB_BOUNDARY_PT;
    pl[1] = corner_idx;
    app_ctrl_radar_dbg_pack_s16(pl + 2, app_ctrl_radar_mm_to_dbg_s16(x_mm));
    app_ctrl_radar_dbg_pack_s16(pl + 4, app_ctrl_radar_mm_to_dbg_s16(y_mm));
    LOG_D("send BOUNDARY_PT %d %d %d", corner_idx, x_mm, y_mm);
    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_RADAR_PRED_DEBUG, g_ctrlSeq++, pl, sizeof(pl));
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
#endif /* UI_RADAR_ENABLE */

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
static int app_ctrl_handle_led_ctrl(u8 seq, u8 *payload, u16 len)
{
    if (len < 2)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_LED_CTRL, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 ledId = payload[0];
    u8 state = payload[1];  // 0: off, 1: on

    app_ctrl_led_set(ledId, state);

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_LED_CTRL, seq, rsp, sizeof(rsp));
    return 0;
}

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
    LOG_D("pan_deg10: %d, tilt_deg10: %d, height_mm: %d", pan_deg10, tilt_deg10, height_mm);
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
    LOG_D("x_mm: %d, y_mm: %d", x_mm, y_mm);

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
        LOG_D("PAN: %d, TILT: %d,intervalUs: %d", panDeg10, tiltDeg10, intervalUs);
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
        LOG_D("Set Zero All");
    }
    else if (axis < STEP_MOTOR_AXIS_MAX)
    {
        StepMotor_GimbalSetZero((step_motor_axis_e)axis);
        LOG_D("Set Zero %d", axis);
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
                        need_stop = 1;
                    }
                }
                else if (direction == 0x02)
                {
                    s32 limit_x_mm = g_radar_boundary_x[limit_index];
                    LOG_D("cur_x_mm: %d, limit_x_mm: %d", cur_x_mm, limit_x_mm);
                    if (cur_x_mm <= (limit_x_mm + APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        need_stop = 1;
                    }
                }
                else if (direction == 0x00)
                {
                    s32 limit_y_mm = g_radar_boundary_y[limit_index];
                    if (cur_y_mm >= (limit_y_mm - APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
                        need_stop = 1;
                    }
                }
                else if (direction == 0x01)
                {
                    s32 limit_y_mm = g_radar_boundary_y[limit_index];
                    LOG_D("cur_y_mm: %d, limit_y_mm: %d", cur_y_mm, limit_y_mm);
                    if (cur_y_mm <= (limit_y_mm + APP_CTRL_BOUNDARY_PAN_GUARD_MM))
                    {
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
                    app_ctrl_send(CTRL_MSG_TYPE_EVENT, CTRL_CMD_MOTOR_DIR_CTRL, g_ctrlSeq++, evt, 4);
                }
            }
        }
    }
#endif
#endif
}

static int app_ctrl_handle_cfg_set(u8 seq, u8 *payload, u16 len)
{
    if (len < 2)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_CFG_SET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 key  = payload[0];
    u8 vLen = payload[1];

    if ((u16)(2 + vLen) > len)
    {
        u8 rsp[2] = {CTRL_STATUS_LEN_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_CFG_SET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 *val = &payload[2];

    switch (key)
    {
    case 0x01:  // workMode
        g_ctrlCfg.workMode = val[0];
        break;
    case 0x02:  // motorDefaultOnMs (u16)
        if (vLen >= 2)
        {
            g_ctrlCfg.motorDefaultOnMs = val[0] | (val[1] << 8);
        }
        break;
    default:
        // unsupported key
        break;
    }

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_CFG_SET, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_cfg_get(u8 seq, u8 *payload, u16 len)
{
    (void)payload;
    if (len < 1)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_CFG_GET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 key    = payload[0];
    u8 rsp[5] = {CTRL_STATUS_OK, key, 0, 0, 0};
    u8 rspLen = 2;

    switch (key)
    {
    case 0x01:       // workMode
        rsp[2] = 1;  // value length
        rsp[3] = g_ctrlCfg.workMode;
        rspLen = 4;
        break;
    case 0x02:       // motorDefaultOnMs
        rsp[2] = 2;  // value length
        rsp[3] = U16_LO(g_ctrlCfg.motorDefaultOnMs);
        rsp[4] = U16_HI(g_ctrlCfg.motorDefaultOnMs);
        rspLen = 5;
        break;
    default:
        rsp[0] = CTRL_STATUS_PARAM_ERROR;
        break;
    }

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_CFG_GET, seq, rsp, rspLen);
    return 0;
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
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_POWER_CTRL, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 on = payload[0] ? 1 : 0;
    app_set_radar_state(on);

    u8 rsp[2] = {CTRL_STATUS_OK, on};
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

    u8 rsp[5] = {CTRL_STATUS_OK, 0, 0, 0, 0};
    rsp[1]    = app_get_radar_state() ? 1 : 0;
#if (UI_RADAR_ENABLE)
    rsp[2]        = app_radar_is_boundary_set() ? 1 : 0;
    s32 height_mm = 0;
    app_radar_get_install_height_mm(&height_mm);
    rsp[3] = app_radar_is_install_height_set() ? (U16_LO((s16)height_mm)) : 0;
    rsp[4] = app_radar_is_install_height_set() ? (U16_HI((s16)height_mm)) : 0;
#endif

    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_STATUS_GET, seq, rsp, sizeof(rsp));
    return 0;
}

static int app_ctrl_handle_play_record_get(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    (void)payload;
    if (len < 1)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 max_req = payload[0];
    if (max_req > RADAR_TIME_MAX_RECORDS)
    {
        max_req = RADAR_TIME_MAX_RECORDS;
    }

    u32 records[RADAR_TIME_MAX_RECORDS * 2] = {0};
    u8  timezones[RADAR_TIME_MAX_RECORDS]   = {0};
    int count                               = app_radar_get_play_records(records, timezones, max_req);
    if (count <= 0)
    {
        u8 rsp[2] = {CTRL_STATUS_OK, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
        return 0;
    }

    // u8 payload_len = (u8)(1 + count * 9);
    // if (payload_len > (CTRL_TX_MAX_LEN - 6))
    // {
    //     payload_len = (CTRL_TX_MAX_LEN - 6);
    // }

    u8 rsp[12] = {0};
    rsp[0]     = CTRL_STATUS_OK;
    rsp[1]     = count;

    for (int i = 0; i < count; i++)
    {
        rsp[2]        = (u8)i;
        u32 start_sec = records[i * 2];
        u32 end_sec   = records[i * 2 + 1];
        rsp[3]        = (u8)(start_sec & 0xFF);
        rsp[4]        = (u8)((start_sec >> 8) & 0xFF);
        rsp[5]        = (u8)((start_sec >> 16) & 0xFF);
        rsp[6]        = (u8)((start_sec >> 24) & 0xFF);
        rsp[7]        = (u8)(end_sec & 0xFF);
        rsp[8]        = (u8)((end_sec >> 8) & 0xFF);
        rsp[9]        = (u8)((end_sec >> 16) & 0xFF);
        rsp[10]       = (u8)((end_sec >> 24) & 0xFF);
        rsp[11]       = timezones[i];
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_PLAY_RECORD_GET, seq, rsp, sizeof(rsp));
    };
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

static int app_ctrl_handle_radar_set_install_height(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 2)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_SET_INSTALL_HEIGHT, seq, rsp, sizeof(rsp));
        return -1;
    }

    s16 height_mm = (s16)(payload[0] | (payload[1] << 8));
    app_radar_set_install_height_mm((s32)height_mm);

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_SET_INSTALL_HEIGHT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_SET_INSTALL_HEIGHT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static int app_ctrl_handle_radar_boundary_enter(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    (void)payload;
    (void)len;

    g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_SETTING;
    app_ctrl_radar_boundary_reset();

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_ENTER, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_ENTER, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static int app_ctrl_handle_radar_boundary_select_point(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (len < 1)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, CTRL_RADAR_BOUNDARY_ERR_INDEX};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    if (g_radar_boundary_mode != CTRL_RADAR_BOUNDARY_MODE_SETTING)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, CTRL_RADAR_BOUNDARY_ERR_STATE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 point_index = payload[0];
    if (point_index >= RADAR_BOUNDARY_POINT_COUNT)
    {
        u8 rsp[2] = {CTRL_STATUS_PARAM_ERROR, CTRL_RADAR_BOUNDARY_ERR_INDEX};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    g_radar_boundary_active_index = point_index;
    app_ctrl_radar_boundary_move_to_point(point_index);

    u8 rsp[3] = {CTRL_STATUS_OK, point_index, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static u8 app_ctrl_radar_boundary_all_points_ready(void)
{
#if UI_RADAR_ENABLE
    return (g_radar_boundary_point_mask == 0x0F);
#else
    return 0;
#endif
}

static int app_ctrl_handle_radar_boundary_save_point(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    if (g_radar_boundary_mode != CTRL_RADAR_BOUNDARY_MODE_SETTING)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, CTRL_RADAR_BOUNDARY_ERR_STATE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    if (len < 1)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, CTRL_RADAR_BOUNDARY_ERR_INDEX};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    u8 point_index = payload[0];
    if (point_index >= RADAR_BOUNDARY_POINT_COUNT)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, point_index, CTRL_RADAR_BOUNDARY_ERR_INDEX};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }

    g_radar_boundary_active_index = point_index;

    if (!app_ctrl_radar_boundary_is_move_done(point_index))
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, point_index, CTRL_RADAR_BOUNDARY_ERR_STATE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
        return -1;
    }
    LOG_D("app_ctrl_radar_boundary_save_point idx=%d", point_index);

#if (UI_STEP_MOTOR_ENABLE)
    s16 pan_cur   = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_PAN);
    s16 tilt_cur  = (s16)StepMotor_GimbalGetCurrentDeg10(STEP_MOTOR_AXIS_TILT);
    s32 height_mm = 0;
    app_radar_get_install_height_mm(&height_mm);

    s16 x_mm = 0;
    s16 y_mm = 0;
    LOG_D("p: %d, t: %d, h: %d", pan_cur, tilt_cur, height_mm);
    LOG_D("p: %d, t: %d, h: %d", pan_cur, tilt_cur, height_mm);
    LOG_D("p: %d, t: %d, h: %d", pan_cur, tilt_cur, height_mm);
    app_ctrl_calc_xy_from_angles(pan_cur, tilt_cur, height_mm, &x_mm, &y_mm);
#else
    s16 x_mm = 0;
    s16 y_mm = 0;
#endif

    app_ctrl_radar_boundary_store_point(point_index, x_mm, y_mm);
    LOG_D("x: %d, y: %d", x_mm, y_mm);
    LOG_D("x: %d, y: %d", x_mm, y_mm);
    LOG_D("x: %d, y: %d", x_mm, y_mm);

    u8 ready  = app_ctrl_radar_boundary_all_points_ready() ? 1 : 0;
    u8 rsp[4] = {CTRL_STATUS_OK, point_index, ready, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
    return 0;

#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static int app_ctrl_handle_radar_boundary_commit(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    (void)payload;
    if (g_radar_boundary_mode != CTRL_RADAR_BOUNDARY_MODE_SETTING)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, CTRL_RADAR_BOUNDARY_ERR_STATE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
        return -1;
    }

    if (len != 0)
    {
        u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, 0, CTRL_RADAR_BOUNDARY_ERR_STATE};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
        return -1;
    }

    // if (!app_ctrl_radar_boundary_all_points_ready())
    // {
    //     u8 missing = 0xFF;
    //     for (u8 i = 0; i < RADAR_BOUNDARY_POINT_COUNT; i++)
    //     {
    //         if ((g_radar_boundary_point_mask & (1u << i)) == 0)
    //         {
    //             missing = i;
    //             break;
    //         }
    //     }
    //     u8 rsp[3] = {CTRL_STATUS_PARAM_ERROR, missing, CTRL_RADAR_BOUNDARY_ERR_INDEX};
    //     app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
    //     return -1;
    // }

    u8 errDetail           = CTRL_RADAR_BOUNDARY_OK;
    u8 shortPairMask       = 0;
    g_radar_boundary_ready = 1;

    if (app_ctrl_radar_boundary_commit(&errDetail, &shortPairMask))
    {
        // g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_IDLE;
        u8 rsp[4] = {CTRL_STATUS_OK, 1, 0, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
        // app_ctrl_radar_boundary_reset();
        return 0;
    }

    u8 rsp[4] = {CTRL_STATUS_OK, 0, errDetail, shortPairMask};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
    return -1;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_COMMIT, seq, rsp, sizeof(rsp));
    return -1;
#endif
}

static int app_ctrl_handle_radar_boundary_exit(u8 seq, u8 *payload, u16 len)
{
#if (UI_RADAR_ENABLE)
    (void)payload;
    (void)len;

    g_radar_boundary_mode = CTRL_RADAR_BOUNDARY_MODE_IDLE;
    app_ctrl_radar_boundary_reset();

    u8 rsp[2] = {CTRL_STATUS_OK, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_EXIT, seq, rsp, sizeof(rsp));
    return 0;
#else
    (void)payload;
    (void)len;
    u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
    app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_BOUNDARY_EXIT, seq, rsp, sizeof(rsp));
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

        LOG_D("[CTRL][TEXT] id=%d, len=%d, text=\"%s\"",
              g_textTransferId,
              g_textLen,
              g_textBuf);

        // 文本处理完后重置状态，等待下一次传输
        app_ctrl_text_reset();
    }

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
#endif
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
    case CTRL_CMD_LED_CTRL:
        LOG_D("CTRL_CMD_LED_CTRL");
        app_ctrl_handle_led_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_MOTOR_CTRL:
        LOG_D("CTRL_CMD_MOTOR_CTRL");
        app_ctrl_handle_motor_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_MOTOR_SET_ZERO:
        LOG_D("CTRL_CMD_MOTOR_SET_ZERO");
        app_ctrl_handle_motor_set_zero(seq, payload, payLen);
        break;
    case CTRL_CMD_MOTOR_DIR_CTRL:
        LOG_D("CTRL_CMD_MOTOR_DIR_CTRL");
        app_ctrl_handle_motor_dir_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_CFG_SET:
        LOG_D("CTRL_CMD_CFG_SET");
        app_ctrl_handle_cfg_set(seq, payload, payLen);
        break;
    case CTRL_CMD_CFG_GET:
        LOG_D("CTRL_CMD_CFG_GET");
        app_ctrl_handle_cfg_get(seq, payload, payLen);
        break;
    case CTRL_CMD_TIME_SET:
        LOG_D("CTRL_CMD_TIME_SET");
        app_ctrl_handle_time_set(seq, payload, payLen);
        break;
    case CTRL_CMD_PLAY_RECORD_GET:
        LOG_D("CTRL_CMD_PLAY_RECORD_GET");
        app_ctrl_handle_play_record_get(seq, payload, payLen);
        break;
    case CTRL_CMD_UID_GET:
        LOG_D("CTRL_CMD_UID_GET");
        app_ctrl_handle_uid_get(seq, payload, payLen);
        break;
    case CTRL_CMD_POWER_CTRL:
        LOG_D("CTRL_CMD_POWER_CTRL");
        app_ctrl_handle_power_ctrl(seq, payload, payLen);
        break;
    case CTRL_CMD_STATUS_GET:
        LOG_D("CTRL_CMD_STATUS_GET");
        app_ctrl_handle_status_get(seq, payload, payLen);
        break;
    case CTRL_CMD_TEXT_CHUNK:
        LOG_D("CTRL_CMD_TEXT_CHUNK");
        app_ctrl_handle_text_chunk(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_SET_INSTALL_HEIGHT:
        LOG_D("CTRL_CMD_RADAR_SET_INSTALL_HEIGHT");
        app_ctrl_handle_radar_set_install_height(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_BOUNDARY_ENTER:
        LOG_D("CTRL_CMD_RADAR_BOUNDARY_ENTER");
        app_ctrl_handle_radar_boundary_enter(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT:
        LOG_D("CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT");
        app_ctrl_handle_radar_boundary_select_point(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT:
        LOG_D("CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT");
        app_ctrl_handle_radar_boundary_save_point(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_BOUNDARY_EXIT:
        LOG_D("CTRL_CMD_RADAR_BOUNDARY_EXIT");
        app_ctrl_handle_radar_boundary_exit(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_BOUNDARY_COMMIT:
        LOG_D("CTRL_CMD_RADAR_BOUNDARY_COMMIT");
        app_ctrl_handle_radar_boundary_commit(seq, payload, payLen);
        break;
    case CTRL_CMD_RADAR_RESET_FLASH_CONFIG:
        LOG_D("CTRL_CMD_RADAR_RESET_FLASH_CONFIG");
        app_radar_clear_install_height_and_boundary_flash();
        {
            u8 rsp[1] = {CTRL_STATUS_OK};
            app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_RESET_FLASH_CONFIG, seq, rsp, sizeof(rsp));
        }
        break;
    case CTRL_CMD_RADAR_TRACK_SPEED:
#if (UI_RADAR_ENABLE)
        LOG_D("CTRL_CMD_RADAR_TRACK_SPEED");
        if (payLen < 2)
        {
            u8 rsp[1] = {CTRL_STATUS_PARAM_ERROR};
            app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
        }
        else
        {
            u16 us = payload[0] | (payload[1] << 8);
            app_radar_set_track_gimbal_interval_us((u32)us);
            u8 rsp[3] = {CTRL_STATUS_OK, payload[0], payload[1]};
            app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
        }
#else
    {
        u8 rsp[1] = {CTRL_STATUS_UNSUPPORTED_CMD};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_TRACK_SPEED, seq, rsp, sizeof(rsp));
    }
#endif
        break;
    case CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY:
#if (UI_RADAR_ENABLE)
        LOG_D("CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY");
        app_ctrl_radar_dbg_send_boundary_quad_all();
        // {
        //     u8 rsp[1] = {CTRL_STATUS_OK};
        //     app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, seq, rsp, sizeof(rsp));
        // }
#else
    {
        u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY, seq, rsp, sizeof(rsp));
    }
#endif
        break;
    default: {
        u8 rsp[2] = {CTRL_STATUS_UNSUPPORTED_CMD, 0};
        app_ctrl_send(CTRL_MSG_TYPE_RSP, cmdId, seq, rsp, sizeof(rsp));
        break;
    }
    }
}
