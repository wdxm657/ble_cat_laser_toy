/********************************************************************************************************
 * @file    app_ctrl.h
 *
 * @brief   Application control protocol header (generic business logic over custom BLE service)
 *
 *******************************************************************************************************/
#ifndef APP_CTRL_H_
#define APP_CTRL_H_

#include "tl_common.h"
#include "app_config.h"
#include "printf.h"
#include <string.h>

/**
 * @brief   Control protocol basic definitions
 */

// Protocol version
#define CTRL_PROTO_VERSION               0x01

// Message types
enum{
    CTRL_MSG_TYPE_CMD   = 0x01,
    CTRL_MSG_TYPE_RSP   = 0x02,
    CTRL_MSG_TYPE_EVENT = 0x03,
};

// Command IDs (can be extended freely)
enum{
    CTRL_CMD_LED_CTRL   = 0x10,   // control LED on/off/mode
    CTRL_CMD_LED_QUERY  = 0x11,   // query LED status
    CTRL_CMD_POWER_CTRL = 0x12,   // power on/off for auto play
    CTRL_CMD_STATUS_GET = 0x13,   // get device status (power/boundary/height)

    CTRL_CMD_MOTOR_CTRL      = 0x20,   // 2-axis motor control (target/speed/query/stop)
    CTRL_CMD_MOTOR_SET_ZERO  = 0x21,   // set motor logical zero (origin)
    CTRL_CMD_MOTOR_DIR_CTRL  = 0x22,   // direction control (up/down/left/right, optional speed, threshold notify)

    CTRL_CMD_CFG_SET         = 0x30,   // set configuration parameter
	CTRL_CMD_CFG_GET         = 0x31,   // get configuration parameter
    CTRL_CMD_TIME_SET        = 0x32,   // set device time (YYYY-MM-DD HH:MM:SS)
    CTRL_CMD_PLAY_RECORD_GET = 0x33,   // get play records (start/end time)
    CTRL_CMD_UID_GET         = 0x34,   // get flash UID (16 bytes, split into 2 responses)

    CTRL_CMD_RADAR_SET_INSTALL_HEIGHT  = 0x50,   // set radar install height (mm)
    CTRL_CMD_RADAR_BOUNDARY_ENTER       = 0x51,   // enter boundary setting mode
    CTRL_CMD_RADAR_BOUNDARY_SELECT_POINT = 0x52,  // select boundary point index
    CTRL_CMD_RADAR_BOUNDARY_SAVE_POINT  = 0x53,   // save current point (payload[0]=pointIndex)
    CTRL_CMD_RADAR_BOUNDARY_EXIT        = 0x54,   // exit boundary setting mode
    CTRL_CMD_RADAR_BOUNDARY_COMMIT      = 0x55,   // commit all 4 points when APP confirms ready
    CTRL_CMD_RADAR_RESET_FLASH_CONFIG   = 0x56,   // reset radar install height and boundary in flash
    /** APP -> device: request boundary quad (device emits 4x EVENT sub BOUNDARY_PT); RSP status only */
    CTRL_CMD_RADAR_DEBUG_GET_BOUNDARY  = 0x57,
    /** APP -> device: set radar track gimbal step interval; payload u16 LE interval_us (µs) */
    CTRL_CMD_RADAR_TRACK_SPEED         = 0x58,

	CTRL_CMD_TEXT_CHUNK = 0x40,   // long text transfer in chunks
};

/** payload[0] for CTRL_CMD_RADAR_PRED_DEBUG events */
enum{
    /** prev,raw 4xs16 + motion_valid(u8) + motion_dir_deg10(s16 LE), deg10=degrees*10, 0 if invalid -> 12 B */
    CTRL_RADAR_DBG_SUB_PREV_RAW = 0x01,
    CTRL_RADAR_DBG_SUB_PRED_STA = 0x02,   // ax,ay,bx,by (s16 LE) -> 9 B
    CTRL_RADAR_DBG_SUB_PREDSEQ  = 0x03,   // idx (u8), x,y (s16 LE) -> 6 B
    /** corner index 0..3, x_mm,y_mm (s16 LE, clamped from s32 in firmware) -> 6 B */
    CTRL_RADAR_DBG_SUB_BOUNDARY_PT = 0x04,
};

// Error codes for response payload[0]
enum{
    CTRL_STATUS_OK              = 0x00,
    CTRL_STATUS_LEN_ERROR       = 0x01,
    CTRL_STATUS_UNSUPPORTED_CMD = 0x02,
    CTRL_STATUS_PARAM_ERROR     = 0x03,
    CTRL_STATUS_INTERNAL_ERROR  = 0x04,
};

// ATT value max length for RX/TX.
// 受 MCU RAM 限制，此处仅支持单帧 20 字节（默认 MTU=23 时 ATT 有效负载为 20）。
#define CTRL_RX_MAX_LEN                 20
#define CTRL_TX_MAX_LEN                 20
// 每个 TEXT_CHUNK 分片内可携带的纯文本字节数（注意头+分片字段总长必须 ≤ 20）
#define CTRL_TEXT_CHUNK_DATA_MAX        10

// Global RX/TX buffers used by ATT layer & control layer
extern u8 g_ctrlRxBuf[CTRL_RX_MAX_LEN];
extern u8 g_ctrlTxBuf[CTRL_TX_MAX_LEN];


/**
 * @brief   Initialize control module
 */
void app_ctrl_init(void);

/**
 * @brief   Poll motor direction control limit and notify APP when reached
 */
void app_ctrl_motor_dir_task(void);

/**
 * @brief   Called by ATT write callback when CTRL_RX characteristic receives data
 *
 * @param[in] data - pointer to received data (already copied to g_ctrlRxBuf)
 * @param[in] len  - data length
 */
void app_ctrl_onRx(u8 *data, u16 len);
void app_ctrl_on_ble_connected(void);
void app_ctrl_notify_play_record_changed(void);
void app_ctrl_task(void);

/**
 * @brief   Send one control frame to APP via CTRL_TX characteristic
 *
 * @param[in] msgType    - CTRL_MSG_TYPE_xxx
 * @param[in] cmdId      - CTRL_CMD_xxx
 * @param[in] seq        - sequence number
 * @param[in] payload    - payload buffer
 * @param[in] payloadLen - payload length
 *
 * @return 0: success, other: fail
 */
int app_ctrl_send(u8 msgType, u8 cmdId, u8 seq, u8 *payload, u16 payloadLen);

/**
 * @brief   Send arbitrary bytes/text to PC visualizer via Ctrl TX notify (EVENT: CTRL_CMD_TEXT_CHUNK, 0x40).
 *          Best-effort: if not connected, the call returns without sending.
 */
void app_ctrl_text_send_bytes(const u8 *data, u16 len);

static inline void app_ctrl_text_send_str(const char *s)
{
    if (!s)
    {
        return;
    }
    app_ctrl_text_send_bytes((const u8 *)s, (u16)strlen(s));
}

#if DEBUG_MODE
/**
 * @brief   Use this macro at LOG sites to send the formatted string to PC over BLE.
 *          It does NOT hook printf, and does NOT use FIFO buffering.
 *
 * @note    Buffer is limited (local stack). Long lines will be truncated.
 *          Payload is chunked internally to fit 20-byte ATT values.
 */
#define BLE_LOG_D(fmt, ...)                                                                 \
    do                                                                                      \
    {                                                                                       \
        char _ble_log_buf[96];                                                              \
        tl_sprintf(_ble_log_buf, fmt "\r\n", ##__VA_ARGS__);                                \
        app_ctrl_text_send_bytes((const u8 *)_ble_log_buf, (u16)strlen(_ble_log_buf));      \
    } while (0)
#else
#define BLE_LOG_D(fmt, ...) ((void)0)
#endif
#if (UI_RADAR_ENABLE)
void app_ctrl_radar_dbg_send_prev_raw(s16 prev_x, s16 prev_y, s16 raw_x, s16 raw_y, u8 motion_valid, s16 motion_dir_deg10);
void app_ctrl_radar_dbg_send_pred_sta(s16 ax_mm, s16 ay_mm, s16 bx_mm, s16 by_mm);
void app_ctrl_radar_dbg_send_predseq(u8 idx, s16 x_mm, s16 y_mm);
void app_ctrl_radar_dbg_send_boundary_quad_all(void);
void app_ctrl_status_notify_task(void);
#endif

u8 app_ctrl_is_setting_mode(void);

#endif /* APP_CTRL_H_ */

