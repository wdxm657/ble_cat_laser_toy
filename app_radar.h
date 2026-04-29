/********************************************************************************************************
 * @file    app_radar.h
 *
 * @brief   Radar UART frame parsing + prediction + step-motor tracking (UI_RADAR_ENABLE)
 *
 *******************************************************************************************************/
#ifndef APP_RADAR_H_
#define APP_RADAR_H_

#include "tl_common.h"

#define DEG_TO_RAD             0.01745329252f
#define DEG_TO_RAD_10          (DEG_TO_RAD / 10.0f)
#define RAD_TO_DEG             57.2957795131f
#define RADAR_TIME_MAX_RECORDS 16

#ifdef UI_RADAR_ENABLE

void app_radar_uart_init(void);
void app_radar_uart_ndma_irq_proc(void);

/** 主循环中调用；可选调试统计（RADAR_RX_IRQ_DEBUG）。 */
void app_radar_debug_rx_poll(void);

void app_radar_parse_and_report_frame(void);
void app_radar_gimbal_track_task(void);

void app_radar_init(void);

void app_radar_on_time_tick(void);
void app_radar_set_time_from_epoch(u32 epoch_sec, s8 tz_q15);
int  app_radar_get_play_records(u32 *out_buf, u8 *tz_buf, u8 max_records);
int  app_radar_get_complete_play_records(u32 *out_buf, u8 *tz_buf, u8 max_records);
u8   app_radar_has_complete_play_records(void);
void app_radar_clear_complete_play_records(void);
void app_radar_set_install_height_mm(s32 height_mm);
void app_radar_get_install_height_mm(s32 *height_mm);
u8   app_radar_is_install_height_set(void);
u8   app_radar_is_boundary_set(void);
void app_radar_set_boundary_quad(s32 x_mm[4], s32 y_mm[4]);
void app_radar_get_boundary_quad_by_index(u8 index, s32 *x_mm, s32 *y_mm);
void app_radar_reset_boundary_default(void);
int  app_radar_save_boundary_quad_to_flash(const s32 x_mm[4], const s32 y_mm[4]);

u8 app_radar_is_boundary_set(void);
u8 app_radar_is_install_height_set(void);
u8 RadarSessionIsResting(void);
u8 app_radar_is_working_mode(void);

// Optional debug helper (used by commented test code in app.c)
float app_radar_mysqrt_3(float x);

void app_radar_point_to_pan_tilt(s32 x_mm, s32 y_mm, s32 height_mm, s16 *pan_deg10, s16 *tilt_deg10);
int  radar_boundary_load_from_flash(s32 x_mm[4], s32 y_mm[4]);
void app_radar_clear_install_height_and_boundary_flash(void);

/** 雷达跟踪时云台步进间隔 (µs)；无步进电机编译时为空操作 */
void app_radar_set_track_gimbal_interval_us(u32 interval_us);
u8   app_radar_has_recent_motion(u32 timeout_us);
void app_radar_set_enabled(u8 on);
void app_radar_task_power_schedule(void);
u8   app_radar_is_power_on(void);

#else
static inline void app_radar_debug_rx_poll(void)
{
}

static inline void app_radar_reset_boundary_default(void)
{
}

static inline void app_radar_clear_install_height_and_boundary_flash(void)
{
}
#endif /* UI_RADAR_ENABLE */

#endif /* APP_RADAR_H_ */
