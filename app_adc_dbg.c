#include "tl_common.h"
#include "drivers.h"
#include "adc.h"
#include "app_config.h"
#include "app_adc_dbg.h"
#include "app.h"
#include "app_ctrl.h"

#define APP_ADC_SAMPLE_INTERVAL_US       10000u
#define APP_ADC_REPORT_INTERVAL_US       250000u
#define APP_BAT_DISCHARGE_STEP_S         48u * 1000000u
#define APP_BAT_CHARGE_STEP_S            5u * 1000000u
#define APP_BAT_PERCENT_STABLE_US        5000000u
#define APP_BAT_FLASH_SAVE_INTERVAL_US   30000000u
#define APP_BAT_PERCENT_DEFAULT_NO_FLASH 100u
#define APP_CHARGE_GPIO_DEBOUNCE_SAMPLES 2u /* 10ms * 2 = 20ms，与按键去抖一致 */

#define BAT_PERCENT_FLASH_MAGIC          0x42415450u /* "BATP" */

typedef struct
{
    u32 magic;
    u8  percent;
    u8  reserved[3];
    u32 crc;
} bat_percent_flash_t;

typedef struct
{
    s8  temp_c;
    u16 res_10ohm;
} ntc_point_t;

static const ntc_point_t g_ntc_table[] = {
    {-20, 6891},
    {-19, 6563},
    {-18, 6252},
    {-17, 5958},
    {-16, 5680},
    {-15, 5416},
    {-14, 5166},
    {-13, 4929},
    {-12, 4704},
    {-11, 4491},
    {-10, 4288},
    {-9, 4096},
    {-8, 3914},
    {-7, 3740},
    {-6, 3576},
    {-5, 3419},
    {-4, 3270},
    {-3, 3129},
    {-2, 2994},
    {-1, 2866},
    {0, 2744},
    {1, 2628},
    {2, 2517},
    {3, 2412},
    {4, 2312},
    {5, 2216},
    {6, 2125},
    {7, 2038},
    {8, 1955},
    {9, 1876},
    {10, 1801},
    {11, 1729},
    {12, 1660},
    {13, 1594},
    {14, 1531},
    {15, 1472},
    {16, 1414},
    {17, 1360},
    {18, 1307},
    {19, 1257},
    {20, 1209},
    {21, 1164},
    {22, 1120},
    {23, 1078},
    {24, 1038},
    {25, 1000},
    {26, 963},
    {27, 928},
    {28, 894},
    {29, 861},
    {30, 830},
    {31, 801},
    {32, 772},
    {33, 745},
    {34, 719},
    {35, 693},
    {36, 669},
    {37, 646},
    {38, 624},
    {39, 602},
    {40, 582},
    {41, 562},
    {42, 543},
    {43, 525},
    {44, 508},
    {45, 491},
    {46, 474},
    {47, 459},
    {48, 444},
    {49, 429},
    {50, 416},
    {51, 402},
    {52, 389},
    {53, 377},
    {54, 365},
    {55, 353},
    {56, 342},
    {57, 332},
    {58, 321},
    {59, 311},
    {60, 302},
    {61, 293},
    {62, 284},
    {63, 275},
    {64, 267},
    {65, 259},
    {66, 251},
    {67, 244},
    {68, 236},
    {69, 230},
    {70, 223},
};

typedef struct
{
    u16 mv;
    u8  percent;
} bat_point_t;

/* 单节锂电池电量曲线 */
static const bat_point_t g_bat_table[] = {
    // {4200, 100},
    {4150, 100},
    {4100, 95},
    {4060, 90},
    {4020, 85},
    {3980, 75},
    {3950, 70},
    {3920, 65},
    {3890, 60},
    {3860, 55},
    {3840, 50},
    {3820, 45},
    {3800, 40},
    {3780, 35},
    {3760, 30},
    {3740, 25},
    {3710, 20},
    {3670, 15},
    {3630, 10},
    {3580, 8},
    {3500, 5},
    {3400, 3},
    {3200, 1},
    {3000, 0}};

static u32 s_adc_sample_tick;
static u32 s_adc_init_tick;
static u32 s_adc_report_tick;
static u32 s_mv_bat_sum;
static u32 s_mv_ntc_sum;
static u16 s_sample_cnt;
static u16 s_bat_mv;
static u8  s_bat_percent;
static u8  last_s_bat_percent;
static u8  s_bat_percent_inited;
static u8  s_bat_prev_charging;
static u32 s_bat_rate_acc_us;
static u8  s_charge_state_stable;
static u8  s_usb_det_stable;
static u8  s_charge_state_cnt;
static u8  s_usb_det_cnt;
static s8  s_ntc_temp_c;
static u8  s_ntc_temp_valid;
static u8  s_charge_switch_on = 1;
static u32 s_bat_flash_save_tick;
static u8  s_bat_flash_valid;
static u32 s_log_tick;
static u8  s_temp_protection_active;
static u8  s_temp_protection_reason;
static u8  s_temp_sleep_requested;

static u32 app_adc_dbg_flash_crc32(const u8 *data, u32 len)
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

static u8 app_adc_dbg_bat_percent_load_from_flash(u8 *percent_out)
{
    bat_percent_flash_t stored;

    if (!percent_out)
    {
        return 0;
    }

    flash_read_page(BAT_PERCENT_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
    if (stored.magic != BAT_PERCENT_FLASH_MAGIC)
    {
        return 0;
    }
    u32 crc = app_adc_dbg_flash_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    if (crc != stored.crc)
    {
        return 0;
    }
    if (stored.percent > 100)
    {
        return 0;
    }
    *percent_out = stored.percent;
    return 1;
}

void app_adc_dbg_bat_percent_save_to_flash(void)
{
    bat_percent_flash_t stored;
    stored.magic       = BAT_PERCENT_FLASH_MAGIC;
    stored.percent     = s_bat_percent;
    stored.reserved[0] = 0;
    stored.reserved[1] = 0;
    stored.reserved[2] = 0;
    stored.crc         = app_adc_dbg_flash_crc32((const u8 *)&stored, sizeof(stored) - sizeof(stored.crc));
    flash_erase_sector(BAT_PERCENT_FLASH_ADDR);
    flash_write_page(BAT_PERCENT_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
}

static void app_adc_dbg_gpio_debounce(u8 raw, u8 *stable, u8 *cnt)
{
    if (raw == *stable)
    {
        *cnt = 0;
        return;
    }
    if (++(*cnt) >= APP_CHARGE_GPIO_DEBOUNCE_SAMPLES)
    {
        *stable = raw;
        *cnt    = 0;
    }
}

static void app_adc_dbg_charge_gpio_update(void)
{
    u8 charge_raw = gpio_read(CHARGE_STATE);
    u8 usb_raw    = gpio_read(USB_DET);

    app_adc_dbg_gpio_debounce(charge_raw, &s_charge_state_stable, &s_charge_state_cnt);
    app_adc_dbg_gpio_debounce(usb_raw, &s_usb_det_stable, &s_usb_det_cnt);
}

static int ntc_temp_from_res_10ohm(u16 r_10ohm)
{
    const int n = sizeof(g_ntc_table) / sizeof(g_ntc_table[0]);

    if (r_10ohm >= g_ntc_table[0].res_10ohm)
    {
        return g_ntc_table[0].temp_c;
    }
    if (r_10ohm <= g_ntc_table[n - 1].res_10ohm)
    {
        return g_ntc_table[n - 1].temp_c;
    }

    for (int i = 0; i < n - 1; i++)
    {
        u16 r_hi = g_ntc_table[i].res_10ohm;
        u16 r_lo = g_ntc_table[i + 1].res_10ohm;
        if (r_10ohm <= r_hi && r_10ohm >= r_lo)
        {
            s8  t_hi   = g_ntc_table[i].temp_c;
            s8  t_lo   = g_ntc_table[i + 1].temp_c;
            s16 dt     = (s16)t_lo - (s16)t_hi;
            u16 dr     = (u16)(r_hi - r_lo);
            u16 offset = (u16)(r_hi - r_10ohm);
            if (dr == 0)
            {
                return t_hi;
            }
            return (int)((s32)t_hi + (s32)dt * (s32)offset / (s32)dr);
        }
    }

    return g_ntc_table[n - 1].temp_c;
}

u8 app_adc_dbg_is_charging(void)
{
    /* CHARGE_STATE: 充电中为低电平；usb_det: 插入为高电平（经去抖后的稳定值） */
    return (s_charge_state_stable == 0 && s_usb_det_stable == 1) ? 1 : 0;
}

u8 app_adc_dbg_is_charge_switch_on(void)
{
    return s_charge_switch_on;
}

s8 app_adc_dbg_get_ntc_temp_c(void)
{
    return s_ntc_temp_c;
}

u8 app_adc_dbg_is_ntc_temp_valid(void)
{
    return s_ntc_temp_valid;
}

static void app_adc_dbg_charge_switch_set(u8 on)
{
    s_charge_switch_on = on ? 1 : 0;
    gpio_write(CHARGE_SWITCH, s_charge_switch_on);
}

static void app_adc_dbg_temp_charge_manage(void)
{
    if (!s_ntc_temp_valid)
    {
        return;
    }

    if (s_usb_det_stable)
    {
        if (s_charge_switch_on)
        {
            /*  USB 插入且充电开关开启时：>45°或小于0° 关充电 */
            if (s_ntc_temp_c > 45 || s_ntc_temp_c < 0)
            {
                app_adc_dbg_charge_switch_set(0);
            }
        }else{
            /*  USB 插入且充电开关关闭时*/
            if (s_ntc_temp_c >= 0 && s_ntc_temp_c <= 25)
            {
                // 低电流充电
                gpio_write(Set_Charg_I, 1);
                app_adc_dbg_charge_switch_set(1);
            }
            else if (s_ntc_temp_c > 25 && s_ntc_temp_c <= 45)
            {
                // 正常电流充电
                gpio_write(Set_Charg_I, 0);
                app_adc_dbg_charge_switch_set(1);
            }
        }
    }

    // 大于60°或小于-10°关机，如果未插电时则进入低功耗，插电唤醒
    u8 temp_protection_reason = 0;
    if (s_ntc_temp_c > 60)
    {
        temp_protection_reason = CTRL_REASON_BATTERY_TEMP_HIGH;
    }
    else if (s_ntc_temp_c < -10)
    {
        temp_protection_reason = CTRL_REASON_BATTERY_TEMP_LOW;
    }

    if (temp_protection_reason)
    {
        if (app_get_power_state())
        {
            app_set_power_state(0);
            app_save_power_state_to_flash();
        }

        if (!s_temp_protection_active || s_temp_protection_reason != temp_protection_reason)
        {
            s_temp_protection_active = 1;
            s_temp_protection_reason = temp_protection_reason;
            if (temp_protection_reason == CTRL_REASON_BATTERY_TEMP_HIGH)
            {
                app_ctrl_notify_power_rejected_battery_temp_high();
            }
            else
            {
                app_ctrl_notify_power_rejected_battery_temp_low();
            }
        }

        if (!s_usb_det_stable && !s_temp_sleep_requested)
        {
            s_temp_sleep_requested = 1;
            BLE_LOG_D("battery temp protection: request deep sleep, temp=%dC", s_ntc_temp_c);
            app_request_deep_sleep();
        }
    }
    else
    {
        s_temp_protection_active = 0;
        s_temp_protection_reason = 0;
        s_temp_sleep_requested = 0;
    }
}

static u8 app_adc_dbg_bat_percent_from_mv(u16 mv_bat)
{
    int n = (int)(sizeof(g_bat_table) / sizeof(g_bat_table[0]));

    if (mv_bat >= g_bat_table[0].mv)
    {
        return g_bat_table[0].percent;
    }
    if (mv_bat <= g_bat_table[n - 1].mv)
    {
        return g_bat_table[n - 1].percent;
    }

    for (int i = 0; i < n - 1; i++)
    {
        u16 mv_hi = g_bat_table[i].mv;
        u16 mv_lo = g_bat_table[i + 1].mv;
        if ((mv_bat <= mv_hi) && (mv_bat >= mv_lo))
        {
            u8  p_hi   = g_bat_table[i].percent;
            u8  p_lo   = g_bat_table[i + 1].percent;
            u16 d_mv   = (u16)(mv_hi - mv_lo);
            u16 offset = (u16)(mv_hi - mv_bat);
            if (d_mv == 0)
            {
                return p_hi;
            }
            u32 delta = (u32)(p_hi - p_lo) * (u32)offset;
            return (u8)(p_hi - (u8)((delta + d_mv / 2u) / d_mv));
        }
    }

    return g_bat_table[n - 1].percent;
}

u16 app_adc_dbg_get_bat_mv(void)
{
    return s_bat_mv;
}

u8 app_adc_dbg_get_bat_percent(void)
{
    // 只给前端 0 20 40 60 80 100
    u8 bat = s_bat_percent / 20 * 20;
    if (bat < 100)
    {
        bat+=20;
    }
    return bat;
}

u8 app_adc_dbg_get_bat_percent_exact(void)
{
    return s_bat_percent;
}

u8 app_adc_dbg_is_bat_percent_stable(void)
{
    return (s_bat_percent_inited && clock_time_exceed(s_adc_init_tick, APP_BAT_PERCENT_STABLE_US)) ? 1 : 0;
}

static u8 app_adc_dbg_bat_percent_apply_rate_limit(u8 target_percent, u8 is_charging)
{
    u32 step_us = (is_charging ? APP_BAT_CHARGE_STEP_S : APP_BAT_DISCHARGE_STEP_S);

    if (!s_bat_percent_inited)
    {
        s_bat_percent_inited = 1;
        s_bat_prev_charging  = is_charging;
        s_bat_rate_acc_us    = 0;
        return s_bat_percent;
    }

    if (s_bat_prev_charging != is_charging)
    {
        s_bat_prev_charging = is_charging;
        s_bat_rate_acc_us   = 0;
    }

    if (is_charging)
    {
        if (target_percent < s_bat_percent)
        {
            target_percent = s_bat_percent;
        }
        if (target_percent == s_bat_percent)
        {
            s_bat_rate_acc_us = 0;
            return s_bat_percent;
        }

        s_bat_rate_acc_us += APP_ADC_REPORT_INTERVAL_US;
        while ((s_bat_percent < target_percent) && (s_bat_rate_acc_us >= step_us))
        {
            s_bat_percent++;
            s_bat_rate_acc_us -= step_us;
        }
    }
    else
    {
        if (target_percent > s_bat_percent)
        {
            target_percent = s_bat_percent;
        }
        if (target_percent == s_bat_percent)
        {
            s_bat_rate_acc_us = 0;
            return s_bat_percent;
        }

        s_bat_rate_acc_us += APP_ADC_REPORT_INTERVAL_US;
        while ((s_bat_percent > target_percent) && (s_bat_rate_acc_us >= step_us))
        {
            s_bat_percent--;
            s_bat_rate_acc_us -= step_us;
        }
    }

    return s_bat_percent;
}

static u32 app_adc_dbg_sample_pin(adc_input_pin_def_e pin)
{
    adc_base_init(pin);
    return adc_sample_and_get_result();
}

void app_adc_dbg_init(void)
{
    u8 flash_percent = 0;

    adc_init();
    adc_power_on_sar_adc(1);

    s_bat_flash_valid = app_adc_dbg_bat_percent_load_from_flash(&flash_percent);
    if (!s_bat_flash_valid)
    {
        flash_percent     = APP_BAT_PERCENT_DEFAULT_NO_FLASH;
        s_bat_flash_valid = 1;
    }

    // 初始化计时器
    s_adc_init_tick       = clock_time();
    s_adc_sample_tick     = 0;
    s_adc_report_tick     = 0;
    s_mv_bat_sum          = 0;
    s_mv_ntc_sum          = 0;
    s_sample_cnt          = 0;
    s_bat_mv              = 0;
    s_bat_percent         = flash_percent;
    s_bat_flash_save_tick = 0;
    s_bat_percent_inited  = 0;
    s_bat_prev_charging   = 0xFF;
    s_bat_rate_acc_us     = 0;
    s_charge_state_stable = gpio_read(CHARGE_STATE);
    s_usb_det_stable      = gpio_read(USB_DET);
    s_charge_state_cnt    = 0;
    s_usb_det_cnt         = 0;
    s_ntc_temp_c          = 0;
    s_ntc_temp_valid      = 0;
    s_charge_switch_on    = 1;
    s_temp_protection_active = 0;
    s_temp_protection_reason = 0;
    s_temp_sleep_requested = 0;
}

static u8 first_5ms_flag;

void app_adc_dbg_poll(void)
{
    u32 now = clock_time();

    if (s_adc_sample_tick == 0)
    {
        s_adc_sample_tick = now;
    }
    if (s_adc_report_tick == 0)
    {
        s_adc_report_tick = now;
    }

    if (clock_time_exceed(s_adc_sample_tick, APP_ADC_SAMPLE_INTERVAL_US))
    {
        app_adc_dbg_charge_gpio_update();

        u32 mv_bat = app_adc_dbg_sample_pin(ADC_GPIO_PB2);
        u32 mv_ntc = app_adc_dbg_sample_pin(ADC_GPIO_PC4);

        s_mv_bat_sum += mv_bat;
        s_mv_ntc_sum += mv_ntc;
        s_sample_cnt++;
        s_adc_sample_tick = now;
    }

    if (clock_time_exceed(s_adc_report_tick, APP_ADC_REPORT_INTERVAL_US))
    {
        u32 mv_bat_avg = 0;
        u32 mv_ntc_avg = 0;

        if (s_sample_cnt > 0)
        {
            mv_bat_avg = s_mv_bat_sum / s_sample_cnt;
            mv_ntc_avg = s_mv_ntc_sum / s_sample_cnt;
        }

        int ntc_temp_c    = 0;
        u16 ntc_res_10ohm = 0;
        s_ntc_temp_valid  = 0;
        if (mv_ntc_avg > 0 && mv_ntc_avg < 3300)
        {
            u32 r10          = (1000u * mv_ntc_avg) / (3300u - mv_ntc_avg);
            ntc_res_10ohm    = (r10 > 0xFFFFu) ? 0xFFFFu : (u16)r10;
            ntc_temp_c       = ntc_temp_from_res_10ohm(ntc_res_10ohm);
            s_ntc_temp_c     = (s8)ntc_temp_c;
            s_ntc_temp_valid = 1;
        }

        /* 分压比 660k / 100k => 电池真实电压 = ADC 电压 * 6.6 */
        mv_bat_avg = (mv_bat_avg * 66u + 5u) / 10u;

        u8 is_charging = app_adc_dbg_is_charging();
        u8 bat_percent_raw;
        bat_percent_raw = app_adc_dbg_bat_percent_from_mv((u16)mv_bat_avg);

        // ADC 初始化 5s 前使用 flash 上次电量，避免 raw 不稳定
        if (clock_time_exceed(s_adc_init_tick, APP_BAT_PERCENT_STABLE_US))
        {
            u8 bat_percent = app_adc_dbg_bat_percent_apply_rate_limit(bat_percent_raw, is_charging);
            s_bat_percent  = bat_percent;
            if (!first_5ms_flag)
            {
                s_bat_percent  = bat_percent_raw;
                first_5ms_flag = 1;
                BLE_LOG_D("first_5ms_flag %d", bat_percent_raw);
                app_adc_dbg_bat_percent_save_to_flash();
            }

            // if (app_adc_dbg_is_charging() && s_bat_percent < 5)
            // {  // 充电中，软件开机状态，则软件关机
            //     if (app_get_power_state()){
            //         BLE_LOG_D("s_bat_percent  < 5 close");
            //         app_set_power_state(0);
            //     }
            // }
        }
        else if (!s_bat_flash_valid)
        {
            BLE_LOG_D("Using default bat percent %d%% as flash is invalid", bat_percent_raw);
            s_bat_percent = bat_percent_raw;
        }

        s_bat_mv = (mv_bat_avg > 0xFFFFu) ? 0xFFFFu : (u16)mv_bat_avg;
        if (clock_time_exceed(s_bat_flash_save_tick, 1000000))
        {
            last_s_bat_percent = s_bat_percent;
            // BLE_LOG_D("bat=%d bat_pc=%d is_char=%d bat_raw=%d", mv_bat_avg, s_bat_percent, is_charging, bat_percent_raw);
            BLE_LOG_D("ntc=%d NTC_R=%d(ohm) T=%dC", mv_ntc_avg, ntc_res_10ohm, ntc_temp_c);
            s_bat_flash_save_tick = now;
        }
        if (first_5ms_flag)
        {
            app_adc_dbg_temp_charge_manage();
        }

        s_mv_bat_sum      = 0;
        s_mv_ntc_sum      = 0;
        s_sample_cnt      = 0;
        s_adc_report_tick = now;
    }
}
