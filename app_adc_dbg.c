#include "tl_common.h"
#include "drivers.h"
#include "adc.h"
#include "app_config.h"
#include "app_adc_dbg.h"
#include "app_ctrl.h"

#define APP_ADC_SAMPLE_INTERVAL_US 10000u
#define APP_ADC_REPORT_INTERVAL_US 250000u
#define APP_BAT_DISCHARGE_STEP_S   210u
#define APP_BAT_CHARGE_STEP_S      30u

typedef struct
{
    s8  temp_c;
    u16 res_10ohm;
} ntc_point_t;

static const ntc_point_t g_ntc_table[] = {
    {-30, 11952},
    {-29, 11330},
    {-28, 10745},
    {-27, 10193},
    {-26, 9673},
    {-25, 9183},
    {-24, 8721},
    {-23, 8285},
    {-22, 7873},
    {-21, 7485},
    {-20, 7118},
    {-19, 6771},
    {-18, 6443},
    {-17, 6133},
    {-16, 5840},
    {-15, 5562},
    {-14, 5300},
    {-13, 5051},
    {-12, 4816},
    {-11, 4593},
    {-10, 4381},
    {-9, 4181},
    {-8, 3991},
    {-7, 3811},
    {-6, 3640},
    {-5, 3477},
    {-4, 3323},
    {-3, 3177},
    {-2, 3038},
    {-1, 2905},
    {0, 2780},
    {1, 2660},
    {2, 2546},
    {3, 2438},
    {4, 2335},
    {5, 2237},
    {6, 2144},
    {7, 2055},
    {8, 1970},
    {9, 1890},
    {10, 1813},
    {11, 1739},
    {12, 1669},
    {13, 1602},
    {14, 1539},
    {15, 1478},
    {16, 1420},
    {17, 1364},
    {18, 1311},
    {19, 1261},
    {20, 1212},
    {21, 1166},
    {22, 1122},
    {23, 1079},
    {24, 1039},
    {25, 1000},
    {26, 963},
    {27, 927},
    {28, 893},
    {29, 861},
    {30, 830},
    {31, 800},
    {32, 771},
    {33, 743},
    {34, 717},
    {35, 692},
    {36, 667},
    {37, 644},
    {38, 622},
    {39, 600},
    {40, 580},
    {41, 560},
    {42, 541},
    {43, 523},
    {44, 505},
    {45, 488},
    {46, 472},
    {47, 457},
    {48, 442},
    {49, 427},
    {50, 413},
    {51, 400},
    {52, 387},
    {53, 375},
    {54, 363},
    {55, 351},
    {56, 340},
    {57, 330},
    {58, 319},
    {59, 309},
    {60, 300},
    {61, 291},
    {62, 282},
    {63, 273},
    {64, 265},
    {65, 257},
    {66, 249},
    {67, 242},
    {68, 235},
    {69, 228},
    {70, 221},
    {71, 215},
    {72, 209},
    {73, 203},
    {74, 197},
    {75, 191},
    {76, 186},
    {77, 180},
    {78, 175},
    {79, 170},
    {80, 166},
    {81, 161},
    {82, 157},
    {83, 152},
    {84, 148},
    {85, 144},
    {86, 140},
    {87, 137},
    {88, 133},
    {89, 129},
    {90, 126},
    {91, 123},
    {92, 119},
    {93, 116},
    {94, 113},
    {95, 110},
    {96, 107},
    {97, 105},
    {98, 102},
    {99, 99},
    {100, 97},
    {101, 95},
    {102, 92},
    {103, 90},
    {104, 88},
    {105, 85},
    {106, 83},
    {107, 81},
    {108, 79},
    {109, 77},
    {110, 76},
    {111, 74},
    {112, 72},
    {113, 70},
    {114, 69},
    {115, 67},
    {116, 65},
    {117, 64},
    {118, 62},
    {119, 61},
    {120, 60},
    {121, 59},
    {122, 57},
    {123, 56},
    {124, 54},
    {125, 53},
};

typedef struct
{
    u16 mv;
    u8  percent;
} bat_point_t;

/* 单节锂电池电量曲线 */
static const bat_point_t g_bat_table[] = {
    {4200, 100},
    {4150, 95},
    {4100, 90},
    {4050, 80},
    {3980, 70},
    {3920, 60},
    {3860, 50},
    {3800, 40},
    {3740, 30},
    {3680, 20},
    {3600, 10},
    {3500, 5},
    {3200, 0},
};

static u32 s_adc_sample_tick;
static u32 s_adc_report_tick;
static u32 s_mv_bat_sum;
static u32 s_mv_ntc_sum;
static u16 s_sample_cnt;
static u16 s_bat_mv;
static u8  s_bat_percent;
static u8  s_bat_percent_inited;
static u8  s_bat_prev_charging;
static u32 s_bat_rate_acc_us;

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
    /* CHARGE_STATE: 充电中为低电平；CHARGE_STY: 充满为低电平 */
    u8 charge_state = gpio_read(CHARGE_STATE);
    u8 charge_full  = gpio_read(CHARGE_STY);
    return (charge_state == 0 && charge_full != 0) ? 1 : 0;
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
    return s_bat_percent;
}

static u8 app_adc_dbg_bat_percent_apply_rate_limit(u8 target_percent, u8 is_charging)
{
    u32 step_us = (is_charging ? APP_BAT_CHARGE_STEP_S : APP_BAT_DISCHARGE_STEP_S) * 1000000u;

    if (!s_bat_percent_inited)
    {
        s_bat_percent_inited = 1;
        s_bat_prev_charging  = is_charging;
        s_bat_rate_acc_us    = 0;
        s_bat_percent        = target_percent;
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
    adc_init();
    adc_power_on_sar_adc(1);

    s_adc_sample_tick    = 0;
    s_adc_report_tick    = 0;
    s_mv_bat_sum         = 0;
    s_mv_ntc_sum         = 0;
    s_sample_cnt         = 0;
    s_bat_mv             = 0;
    s_bat_percent        = 0;
    s_bat_percent_inited = 0;
    s_bat_prev_charging  = 0xFF;
    s_bat_rate_acc_us    = 0;
}

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
        if (mv_ntc_avg > 0 && mv_ntc_avg < 3300)
        {
            u32 r10       = (1000u * mv_ntc_avg) / (3300u - mv_ntc_avg);
            ntc_res_10ohm = (r10 > 0xFFFFu) ? 0xFFFFu : (u16)r10;
            ntc_temp_c    = ntc_temp_from_res_10ohm(ntc_res_10ohm);
        }

        /* 分压比 660k / 100k => 电池真实电压 = ADC 电压 * 6.6 */
        mv_bat_avg = (mv_bat_avg * 66u + 5u) / 10u;

        u8 bat_percent_raw = app_adc_dbg_bat_percent_from_mv((u16)mv_bat_avg);
        u8 is_charging     = app_adc_dbg_is_charging();
        u8 bat_percent     = app_adc_dbg_bat_percent_apply_rate_limit(bat_percent_raw, is_charging);

        s_bat_mv = (mv_bat_avg > 0xFFFFu) ? 0xFFFFu : (u16)mv_bat_avg;

        // BLE_LOG_D("bat=%d bat_pc=%d is_char=%d ntc=%d NTC_R=%d0(ohm) T=%dC n=%d bat_raw=%d",
        //           mv_bat_avg,
        //           bat_percent,
        //           is_charging,
        //           mv_ntc_avg,
        //           ntc_res_10ohm,
        //           ntc_temp_c,
        //           s_sample_cnt,
        //           bat_percent_raw);

        s_mv_bat_sum      = 0;
        s_mv_ntc_sum      = 0;
        s_sample_cnt      = 0;
        s_adc_report_tick = now;
    }
}
