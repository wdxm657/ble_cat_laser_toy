/********************************************************************************************************
 * @file    app_ui.h
 *
 * @brief   This is the header file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    12,2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#ifndef APP_UI_H_
#define APP_UI_H_



extern int 	key_not_released;
extern int	button_detect_en;
extern u32	button_detect_tick;
extern int	button_not_released;
extern int 	ota_is_working;


/**
 * @brief      this function is used to detect if key pressed or released.
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void proc_keyboard (u8 e, u8 *p, int n);


/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_SUSPEND_ENTER"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void task_sleep_enter(u8 e, u8 *p, int n);


/**
 * @brief		this function is used to detect if button pressed or released.
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void proc_button (u8 e, u8 *p, int n);


/**
 * @brief      this function is used to register the function for OTA start.
 * @param[in]  none
 * @return     none
 */
void app_enter_ota_mode(void);


/**
 * @brief       no matter whether the OTA result is successful or fail.
 *              code will run here to tell user the OTA result.
 * @param[in]   result    OTA result:success or fail(different reason)
 * @return      none
 */
void app_ota_result(int result);


void app_ui_led_task(void);
void app_ui_power_led_task(void);


/**
 * @brief      初始化 RGB 工作状态灯普通 IO 输出（在点灯前调用一次）
 * @param[in]  none
 * @return     none
 */
void app_ui_led_init(void);


/**
 * @brief      点亮/熄灭 RGB 三通道普通 IO（通道值>0 即点亮对应引脚）
 * @param[in]  r - 红通道亮度 0~255
 * @param[in]  g - 绿通道亮度 0~255
 * @param[in]  b - 蓝通道亮度 0~255
 * @return     none
 */
void app_ui_led_set_color(u8 r, u8 g, u8 b);


/**
 * @brief      RGB 工作状态灯预设颜色
 *             顺序：黑(关闭),红,橙,黄,绿,青,蓝,紫,灰,粉,白,棕
 */
typedef enum {
    LED_COLOR_OFF    = 0,   /* 黑(关闭) */
    LED_COLOR_RED,          /* 红 */
    LED_COLOR_ORANGE,       /* 橙 */
    LED_COLOR_YELLOW,       /* 黄 */
    LED_COLOR_GREEN,        /* 绿 */
    LED_COLOR_CYAN,         /* 青 */
    LED_COLOR_BLUE,         /* 蓝 */
    LED_COLOR_PURPLE,       /* 紫 */
    LED_COLOR_GRAY,         /* 灰 */
    LED_COLOR_PINK,         /* 粉 */
    LED_COLOR_WHITE,        /* 白 */
    LED_COLOR_BROWN,        /* 棕 */
    LED_COLOR_MAX,
} led_color_t;


/**
 * @brief      按预设颜色点亮 RGB 工作状态灯（LED_COLOR_OFF 熄灭）
 * @param[in]  color - 预设颜色，见 led_color_t
 * @return     none
 */
void app_ui_led_show(led_color_t color);


/**
 * @brief      测试代码：每秒切换一个预设颜色（调试用，测完删除）
 * @param[in]  none
 * @return     none
 */
void app_ui_led_test_cycle(void);

#endif /* APP_UI_H_ */
