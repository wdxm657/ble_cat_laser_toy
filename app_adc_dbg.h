#pragma once

#include "app_config.h"

void app_adc_dbg_init(void);
void app_adc_dbg_poll(void);
u16  app_adc_dbg_get_bat_mv(void);
u8   app_adc_dbg_get_bat_percent(void);
u8   app_adc_dbg_is_charging(void);
u8   app_adc_dbg_is_charge_switch_on(void);
s8   app_adc_dbg_get_ntc_temp_c(void);
u8   app_adc_dbg_is_ntc_temp_valid(void);
