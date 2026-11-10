/********************************************************************************************************
 * @file    app_vib_data.h
 *
 * @brief   Latest WM_LT_TOY advertising data cached in RAM and persisted in Flash.
 *******************************************************************************************************/
#ifndef APP_VIB_DATA_H_
#define APP_VIB_DATA_H_

#include "tl_common.h"

typedef struct
{
	u8 valid;
	u8 power_on;
	u8 battery_low;
	s8 rssi;
	u8 mac[6];
} app_vib_data_t;

void app_vib_data_init(void);
void app_vib_data_update(const u8 mac[6], s8 rssi, u8 power_on, u8 battery_low);
void app_vib_data_task(void);
void app_vib_data_clear(void);
u8 app_vib_data_get(app_vib_data_t *data);

#endif /* APP_VIB_DATA_H_ */
