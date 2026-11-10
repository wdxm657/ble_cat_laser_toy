/********************************************************************************************************
 * @file    app_vib_data.c
 *
 * @brief   Latest WM_LT_TOY advertising data cached in RAM and persisted in Flash.
 *******************************************************************************************************/
#include "tl_common.h"
#include "drivers.h"

#include "app_config.h"
#include "app_vib_data.h"

#include <string.h>

#define APP_VIB_FLASH_MAGIC   0x56494244u /* "VIBD" */
#define APP_VIB_FLASH_VERSION 0x02u

typedef struct
{
	u32 magic;
	u8 version;
	u8 power_on;
	u8 battery_low;
	s8 rssi;
	u8 mac[6];
	u8 reserved;
	u8 checksum;
} app_vib_flash_record_t;

static app_vib_data_t s_vib_data;
static u8 s_flash_save_pending;

static u8 app_vib_checksum(const u8 *data, u32 len)
{
	u8 checksum = 0x5Au;

	while (len--)
	{
		checksum ^= *data++;
	}
	return checksum;
}

void app_vib_data_init(void)
{
	app_vib_flash_record_t stored;

	memset(&s_vib_data, 0, sizeof(s_vib_data));
	s_flash_save_pending = 0;
	flash_read_page(VIB_DATA_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
	if (stored.magic != APP_VIB_FLASH_MAGIC ||
	    stored.version != APP_VIB_FLASH_VERSION ||
	    stored.checksum != app_vib_checksum((const u8 *)&stored, sizeof(stored) - 1u))
	{
		LOG_D("[VIB_DATA] flash_load valid=0");
		return;
	}

	s_vib_data.valid = 1;
	s_vib_data.power_on = stored.power_on ? 1 : 0;
	s_vib_data.battery_low = stored.battery_low ? 1 : 0;
	s_vib_data.rssi = stored.rssi;
	memcpy(s_vib_data.mac, stored.mac, sizeof(s_vib_data.mac));
	LOG_D("[VIB_DATA] flash_load valid=1 power=%d battery_low=%d rssi=%d",
	      s_vib_data.power_on, s_vib_data.battery_low, s_vib_data.rssi);
}

void app_vib_data_update(const u8 mac[6], s8 rssi, u8 power_on, u8 battery_low)
{
	u8 changed = !s_vib_data.valid ||
	             s_vib_data.power_on != (power_on ? 1 : 0) ||
	             s_vib_data.battery_low != (battery_low ? 1 : 0) ||
	             memcmp(s_vib_data.mac, mac, sizeof(s_vib_data.mac)) != 0;

	s_vib_data.valid = 1;
	s_vib_data.power_on = power_on ? 1 : 0;
	s_vib_data.battery_low = battery_low ? 1 : 0;
	s_vib_data.rssi = rssi;
	memcpy(s_vib_data.mac, mac, sizeof(s_vib_data.mac));

	/* RSSI changes on almost every packet, so it is refreshed in RAM but does
	 * not by itself trigger a sector erase. This avoids excessive Flash wear. */
	if (!changed)
	{
		return;
	}

	s_flash_save_pending = 1;
}

void app_vib_data_task(void)
{
	app_vib_flash_record_t stored;

	if (!s_flash_save_pending)
	{
		return;
	}
	s_flash_save_pending = 0;
	memset(&stored, 0, sizeof(stored));
	stored.magic = APP_VIB_FLASH_MAGIC;
	stored.version = APP_VIB_FLASH_VERSION;
	stored.power_on = s_vib_data.power_on;
	stored.battery_low = s_vib_data.battery_low;
	stored.rssi = s_vib_data.rssi;
	memcpy(stored.mac, s_vib_data.mac, sizeof(stored.mac));
	stored.checksum = app_vib_checksum((const u8 *)&stored, sizeof(stored) - 1u);
	flash_erase_sector(VIB_DATA_FLASH_ADDR);
	flash_write_page(VIB_DATA_FLASH_ADDR, sizeof(stored), (u8 *)&stored);
	LOG_D("[VIB_DATA] flash_save power=%d battery_low=%d rssi=%d",
	      stored.power_on, stored.battery_low, stored.rssi);
}

void app_vib_data_clear(void)
{
	s_flash_save_pending = 0;
	memset(&s_vib_data, 0, sizeof(s_vib_data));
	flash_erase_sector(VIB_DATA_FLASH_ADDR);
	LOG_D("[VIB_DATA] flash_clear valid=0");
}

u8 app_vib_data_get(app_vib_data_t *data)
{
	if (!data || !s_vib_data.valid)
	{
		return 0;
	}

	*data = s_vib_data;
	return 1;
}
