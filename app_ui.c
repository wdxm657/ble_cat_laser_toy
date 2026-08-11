/********************************************************************************************************
 * @file    app_ui.c
 *
 * @brief   This is the source file for BLE SDK
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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "application/application.h"

#include "app.h"
#include "app_att.h"
#include "app_ui.h"
#include "app_ctrl.h"
#include "app_adc_dbg.h"

/*----------------------------------------------------------------------------*/
/*------------- OTA  Function                                 ----------------*/
/*----------------------------------------------------------------------------*/
_attribute_data_retention_ int ota_is_working = 0;
#if (BLE_OTA_SERVER_ENABLE)

/**
 * @brief      this function is used to register the function for OTA start.
 * @param[in]  none
 * @return     none
 */
void app_enter_ota_mode(void)
{
    ota_is_working = 1;
    BLE_LOG_D("[APP][OTA] Enter OTA mode");
    app_ctrl_send_ota_status(1);  // 更新中
}

/**
 * @brief       no matter whether the OTA result is successful or fail.
 *              code will run here to tell user the OTA result.
 * @param[in]   result    OTA result:success or fail(different reason)
 * @return      none
 */
void app_ota_result(int result)
{
    if (result == OTA_SUCCESS)
    {  // OTA success
        BLE_LOG_D("[APP][OTA] SUCCESSF");
        app_ctrl_send_ota_status(2);  // 更新成功
        sleep_ms(500);
    }
    else
    {  // OTA fail
        BLE_LOG_D("[APP][OTA] FAILED %d", result);
        app_ctrl_send_ota_status(3);  // 更新失败
        sleep_ms(500);
    }
}

#endif


#define LED_BLINK_INTERVAL_US 500000

static u32 g_led_blink_tick = 0;
static u8  g_led_blink_on   = 0;

static void app_ui_led_blink_update(void)
{
    u32 now = clock_time();
    if (clock_time_exceed(g_led_blink_tick, LED_BLINK_INTERVAL_US))
    {
        g_led_blink_tick = now;
        g_led_blink_on   = !g_led_blink_on;
    }
}

/*=================== RGB 工作状态灯：普通 IO → PWM 调光/调色 ===================
 * 引脚：GPIO_LED_BLUE(PA5) / GPIO_LED_RED(PA6) / GPIO_LED_GREEN(PA7)
 * ⚠  PWM 通道与引脚的对应关系由 B80 数据手册「GPIO 功能复用表」决定，
 *     以下默认值为 Telink 常规映射（PWM0→PA5, PWM1→PA6, PWM2→PA7），
 *     烧录前请对照手册确认；如需调整，修改下面三个 LED_PWM_*_ID 宏即可，
 *     也可在 app_config.h 中提前定义进行覆盖。
 */
#ifndef LED_RGB_PWM_CLK_HZ
#define LED_RGB_PWM_CLK_HZ      2000000u   /* PWM 计数时钟 2MHz */
#endif
#ifndef LED_RGB_PWM_CYCLE
#define LED_RGB_PWM_CYCLE       256u       /* 周期 256 tick → 7.8kHz；占空比 0~255 */
#endif
#ifndef LED_PWM_BLUE_ID
#define LED_PWM_BLUE_ID         PWM0_ID    /* GPIO_LED_BLUE  PA5 */
#endif
#ifndef LED_PWM_RED_ID
#define LED_PWM_RED_ID          PWM1_ID    /* GPIO_LED_RED   PA6 */
#endif
#ifndef LED_PWM_GREEN_ID
#define LED_PWM_GREEN_ID        PWM2_ID    /* GPIO_LED_GREEN PA7 */
#endif

/* pwm_id(0~5) 与 gpio 功能码 PWM0(3)~PWM5(8) 一一对应 */
static gpio_func_e app_ui_led_pwm_func(pwm_id id)
{
    return (gpio_func_e)(PWM0 + (int)id);
}

/* duty: 0~255，映射到占空比 0~100%；LED_ON_LEVEL==0（低电平点灯）时反相 */
static void app_ui_led_pwm_set(pwm_id id, u8 duty)
{
    u32 cmp = ((u32)LED_RGB_PWM_CYCLE * duty) / 255u;
    if (cmp > LED_RGB_PWM_CYCLE)
    {
        cmp = LED_RGB_PWM_CYCLE;
    }
#if (LED_ON_LEVEL == 0)
    cmp = LED_RGB_PWM_CYCLE - cmp;
#endif
    pwm_set_cmp(id, (u16)cmp);
}

static void app_ui_led_pwm_channel_init(pwm_id id, GPIO_PinTypeDef pin)
{
#if (UI_LED_ENABLE)
    pwm_set_cycle_and_duty(id, (u16)LED_RGB_PWM_CYCLE, 0);  /* 周期 + 初始熄灭 */
    gpio_set_func(pin, app_ui_led_pwm_func(id));            /* 引脚复用为 PWMx */
    gpio_set_output_en(pin, 1);
    pwm_start(id);
#endif
}

/**
 * @brief  初始化 RGB 工作状态灯的 PWM 输出。
 *         必须在任何点灯操作之前调用一次（例如低电检测点灯前）。
 */
void app_ui_led_init(void)
{
#if (UI_LED_ENABLE)
    static u8 s_inited = 0;
    if (s_inited)
    {
        return;
    }
    s_inited = 1;

    pwm_set_clk(CLOCK_SYS_CLOCK_HZ, (int)LED_RGB_PWM_CLK_HZ);
    app_ui_led_pwm_channel_init(LED_PWM_BLUE_ID, GPIO_LED_BLUE);
    app_ui_led_pwm_channel_init(LED_PWM_RED_ID, GPIO_LED_RED);
    app_ui_led_pwm_channel_init(LED_PWM_GREEN_ID, GPIO_LED_GREEN);
    app_ui_led_show(LED_COLOR_OFF);  /* 默认熄灭 */
#endif
}

/**
 * @brief  设置 RGB 三通道亮度，用于调光/调色/开关。
 * @param[in] r 红通道亮度 0~255
 * @param[in] g 绿通道亮度 0~255
 * @param[in] b 蓝通道亮度 0~255
 */
void app_ui_led_set_color(u8 r, u8 g, u8 b)
{
#if (UI_LED_ENABLE)
    app_ui_led_pwm_set(LED_PWM_RED_ID, r);
    app_ui_led_pwm_set(LED_PWM_GREEN_ID, g);
    app_ui_led_pwm_set(LED_PWM_BLUE_ID, b);
#endif
}

/* 预设颜色表：索引与 led_color_t 一一对应 */
static const u8 s_led_rgb_tbl[LED_COLOR_MAX][3] = {
    {  0,   0,   0},   /* OFF 黑(关闭) */
    {255,   0,   0},   /* RED 红 */
    {255, 128,   0},   /* ORANGE 橙 */
    {255, 255,   0},   /* YELLOW 黄 */
    {  0, 255,   0},   /* GREEN 绿 */
    {  0, 255, 255},   /* CYAN 青 */
    {  0,   0, 255},   /* BLUE 蓝 */
    {255,   0, 255},   /* PURPLE 紫 */
    {128, 128, 128},   /* GRAY 灰 */
    {255, 128, 128},   /* PINK 粉 */
    {255, 255, 255},   /* WHITE 白 */
    {165,  42,  42},   /* BROWN 棕 */
};

/**
 * @brief  按预设颜色点亮 RGB 工作状态灯（LED_COLOR_OFF 熄灭）。
 *         预设颜色定义于 s_led_rgb_tbl，如需自定义直接用 app_ui_led_set_color。
 */
void app_ui_led_show(led_color_t color)
{
    if (color >= LED_COLOR_MAX)
    {
        color = LED_COLOR_OFF;
    }
    app_ui_led_set_color(s_led_rgb_tbl[color][0],
                         s_led_rgb_tbl[color][1],
                         s_led_rgb_tbl[color][2]);
}

/* 点亮指定颜色，on=0 时熄灭 */
static void app_ui_led_set_on(led_color_t color, u8 on)
{
    app_ui_led_show(on ? color : LED_COLOR_OFF);
}

/*================ 测试代码：每秒切换一个颜色（调试用，测完删除） ================*/
void app_ui_led_test_cycle(void)
{
    static u32       s_tick  = 0;
    static led_color_t s_color = LED_COLOR_RED;   /* 从红开始循环 */

    if (clock_time_exceed(s_tick, 1000000))       /* 每秒 */
    {
        s_tick = clock_time();
        app_ui_led_show(s_color);
        s_color = (led_color_t)(s_color + 1);
        if (s_color >= LED_COLOR_MAX)
        {
            s_color = LED_COLOR_RED;              /* 循环，跳过 OFF */
        }
    }
}

static void app_ui_power_led_set(GPIO_PinTypeDef pin, u8 on)
{
#if (UI_LED_ENABLE)
    gpio_write(pin, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
#endif
}

void app_ui_led_task(void)
{
// - 工作状态灯（红/蓝/绿）
// - 熄灭
//     - 代表物理关机状态
#if (UI_LED_ENABLE)
    app_ui_led_blink_update();

    if (ota_is_working)
    {
        /* 白灯闪烁：OTA 升级中 */
        app_ui_led_set_on(LED_COLOR_WHITE, g_led_blink_on);
        return;
    }

    // - 红色灯闪烁：设置模式
    if (app_ctrl_is_setting_mode())
    {
        app_ui_led_set_on(LED_COLOR_RED, g_led_blink_on);
        return;
    }
    if (app_get_power_state())
    {
        /* 绿灯：软件开机状态，有连接常亮 / 无连接闪烁 */
        app_ui_led_set_on(LED_COLOR_GREEN,
                          (blc_ll_getCurrentState() == BLS_LINK_STATE_CONN) ? 1 : g_led_blink_on);
    }
    else
    {
        /* 蓝灯：软件关机状态，有连接常亮 / 无连接闪烁 */
        app_ui_led_set_on(LED_COLOR_BLUE,
                          (blc_ll_getCurrentState() == BLS_LINK_STATE_CONN) ? 1 : g_led_blink_on);
    }
#endif
}

void app_ui_power_led_task(void)
{
#if (UI_LED_ENABLE)
    app_ui_led_blink_update();

    /* 电源状态灯独立于工作状态灯，不受电源开关影响 */
    u8 charging    = app_adc_dbg_is_charging();
    u8 bat_percent = app_adc_dbg_get_bat_percent();

    if (bat_percent >= 80)
    {
        app_ui_power_led_set(GPIO_CHARGE_LED_GREEN, 1);
        app_ui_power_led_set(GPIO_CHARGE_LED_RED, 0);
    }
    else if (charging)
    {
        app_ui_power_led_set(GPIO_CHARGE_LED_GREEN, 0);
        app_ui_power_led_set(GPIO_CHARGE_LED_RED, 1);
    }
    else if (bat_percent < 20)
    {
        app_ui_power_led_set(GPIO_CHARGE_LED_GREEN, 0);
        app_ui_power_led_set(GPIO_CHARGE_LED_RED, g_led_blink_on);
    }
    else
    {
        app_ui_power_led_set(GPIO_CHARGE_LED_GREEN, 0);
        app_ui_power_led_set(GPIO_CHARGE_LED_RED, 0);
    }
#endif
}

#if (UI_KEYBOARD_ENABLE)

int        key_not_released;
u8         key_type;
static u32 keyScanTick = 0;

extern u32 scan_pin_need;

#define CONSUMER_KEY 1
#define KEYBOARD_KEY 2

/**
 * @brief		this function is used to process keyboard matrix status change.
 * @param[in]	none
 * @return      none
 */
void key_change_proc(void)
{
#if (PM_DEEPSLEEP_ENABLE)
    extern u32 latest_user_event_tick;
    latest_user_event_tick = clock_time();  // record latest key change time
#endif

    u8 key0       = kb_event.keycode[0];
    u8 key_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    key_not_released = 1;
    if (kb_event.cnt == 2)  // two key press, do  not process
    {
        LOG_D("[APP][KEY] Two key press, do not process");
    }
    else if (kb_event.cnt == 1)
    {
        LOG_D("[APP][KEY] One key press, process");
        if (key0 >= CR_VOL_UP)  // volume up/down
        {
            key_type = CONSUMER_KEY;
            u16 consumer_key;
            if (key0 == CR_VOL_UP)
            {  // volume up
                consumer_key = MKEY_VOL_UP;
            }
            else if (key0 == CR_VOL_DN)
            {  // volume down
                consumer_key = MKEY_VOL_DN;
            }
            blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, HID_CONSUME_REPORT_INPUT_DP_H, (u8 *)&consumer_key, 2);
        }
        else
        {
            // normal keyboard key
            key_type   = KEYBOARD_KEY;
            key_buf[2] = key0;
            blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, HID_NORMAL_KB_REPORT_INPUT_DP_H, key_buf, 8);
        }
    }
    else  // kb_event.cnt == 0,  key release
    {
        key_not_released = 0;
        if (key_type == CONSUMER_KEY)
        {
            u16 consumer_key = 0;
            blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, HID_CONSUME_REPORT_INPUT_DP_H, (u8 *)&consumer_key, 2);
        }
        else if (key_type == KEYBOARD_KEY)
        {
            key_buf[2] = 0;
            blc_gatt_pushHandleValueNotify(BLS_CONN_HANDLE, HID_NORMAL_KB_REPORT_INPUT_DP_H, key_buf, 8);  // release
        }
    }
}

/**
 * @brief      this function is used to detect if key pressed or released.
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */

void proc_keyboard(u8 e, u8 *p, int n)
{
    if (clock_time_exceed(keyScanTick, 8000))
    {
        keyScanTick = clock_time();
    }
    else
    {
        return;
    }

    kb_event.keycode[0] = 0;
    // LOG_D("[APP][KEY] Scan key");
    int det_key = kb_scan_key(0, 1);

    if (det_key)
    {
        LOG_D("[APP][KEY] Key press, process");
        key_change_proc();
    }
}

#endif  // end of UI_KEYBOARD_ENABLE

/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_SUSPEND_ENTER"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void task_sleep_enter(u8 e, u8 *p, int n)
{
#if (BLE_APP_PM_ENABLE)
    (void)e;
    (void)p;
    (void)n;

    /* 记录每次准备进入 suspend 的上下文，便于定位“莫名其妙冷启动重启”前发生了什么 */
    // LOG_D("[APP][PM] SUSPEND_ENTER state=0x%02x, wakeupTickIn=%dus, power_on=%d",
    //       blc_ll_getCurrentState(),
    //       (int)(bls_pm_getSystemWakeupTick() - clock_time()),
    //       app_get_power_state());

    if (blc_ll_getCurrentState() == BLS_LINK_STATE_CONN && ((u32)(bls_pm_getSystemWakeupTick() - clock_time())) > 80 * CLOCK_16M_SYS_TIMER_CLK_1MS)
    {                                           // suspend time > 30ms.add gpio wakeup
        bls_pm_setWakeupSource(PM_WAKEUP_PAD);  // gpio CORE wakeup suspend
    }
#endif
}
