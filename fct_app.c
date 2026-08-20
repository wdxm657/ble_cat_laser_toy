/********************************************************************************************************
 * @file    app.c
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


#include "app.h"
#include "app_att.h"
#include "app_ui.h"
#include "app_buffer.h"
#include "adc.h"
#include "app_ctrl.h"
#include "fct_uart.h"

#define 	ADV_IDLE_ENTER_DEEP_TIME			60  //60 s
#define 	CONN_IDLE_ENTER_DEEP_TIME			60  //60 s


#define MY_APP_ADV_CHANNEL			BLT_ENABLE_ADV_ALL
#define MY_ADV_INTERVAL_MIN			ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX			ADV_INTERVAL_35MS

#define MY_RF_POWER_INDEX			RF_POWER_P2p87dBm

#define MY_DIRECT_ADV_TIME			10000000

#define	BLE_DEVICE_ADDRESS_TYPE 	BLE_DEVICE_ADDRESS_PUBLIC

u32	advertise_begin_tick;

#if (PM_DEEPSLEEP_ENABLE)
int device_in_connection_state;
u8  sendTerminate_before_enterDeep = 0;
u32 latest_user_event_tick;
#endif


own_addr_type_t app_own_address_type = OWN_ADDRESS_PUBLIC;

/**
 * @brief	BLE Advertising data
 */
const u8 tbl_advData[] = {
    12,
    DT_COMPLETE_LOCAL_NAME,
    'W',
    '2',
    'M',
    'L',
    'a',
    's',
    'e',
    'r',
    'T',
    'O',
    'Y',
    2,
    DT_FLAGS,
    0x05,  // BLE limited discoverable mode and BR/EDR not supported
    3,
    DT_APPEARANCE,
    0x80,
    0x01,  // 384, Generic Remote Control, Generic category
    5,
    DT_INCOMPLETE_LIST_16BIT_SERVICE_UUID,
    0x12,
    0x18,
    0x0F,
    0x18,  // incomplete list of service class UUIDs (0x1812, 0x180F)
};

/**
 * @brief	BLE Scan Response Packet data
 */
u8 tbl_scanRsp[] = {
    12,
    DT_COMPLETE_LOCAL_NAME,
    'W',
    '2',
    'M',
    'L',
    'a',
    's',
    'e',
    'r',
    'T',
    'O',
    'Y',
    17,
    DT_MANUFACTURER_SPECIFIC_DATA,
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x06,
    0x07,
    0x08,
    0x09,
    0x0A,
    0x0B,
    0x0C,
    0x0D,
    0x0E,
    0x0F,
};


/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_ADV_DURATION_TIMEOUT"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void app_switch_to_undirected_adv(u8 e, u8 *p, int n)
{
	(void)e;(void)p;(void)n;
	bls_ll_setAdvParam( MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
						ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type,
						0,  NULL,
						MY_APP_ADV_CHANNEL,
						ADV_FP_NONE);


	bls_ll_setAdvEnable(BLC_ADV_ENABLE);  //must: set ADV enable
}




/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_CONNECT"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void task_connect(u8 e, u8 *p, int n)
{

	bls_l2cap_requestConnParamUpdate (CONN_INTERVAL_10MS, CONN_INTERVAL_10MS, 99, CONN_TIMEOUT_4S);  // 1 S

	#if (PM_DEEPSLEEP_ENABLE)
		latest_user_event_tick = clock_time();
		device_in_connection_state = 1;
	#endif

	#if (UI_LED_ENABLE)
		gpio_write(GPIO_LED_RED, LED_ON_LEVEL);
	#endif
}


/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_TERMINATE"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void task_terminate(u8 e,u8 *p, int n) //*p is terminate reason
{
	#if (PM_DEEPSLEEP_ENABLE)
		device_in_connection_state = 0;
	#endif

	tlk_contr_evt_terminate_t *pEvt = (tlk_contr_evt_terminate_t *)p;
	if(pEvt->terminate_reason == HCI_ERR_CONN_TIMEOUT){

	}
	else if(pEvt->terminate_reason == HCI_ERR_REMOTE_USER_TERM_CONN){

	}
	else if(pEvt->terminate_reason == HCI_ERR_CONN_TERM_MIC_FAILURE){

	}
	else{

	}

	#if (PM_DEEPSLEEP_ENABLE)
		 //user has push terminate pkt to ble TX buffer before deepsleep
		if(sendTerminate_before_enterDeep == 1){
			sendTerminate_before_enterDeep = 2;
		}
	#endif


	#if (UI_LED_ENABLE)
		gpio_write(GPIO_LED_RED, !LED_ON_LEVEL);  //light off
	#endif

	advertise_begin_tick = clock_time();
}




/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_SUSPEND_EXIT"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void task_suspend_exit (u8 e, u8 *p, int n)
{
	(void)e;(void)p;(void)n;
	rf_set_power_level_index (MY_RF_POWER_INDEX);
}


/**
 * @brief      power management code for application
 * @param[in]  none
 * @return     none
 */
void blt_pm_proc(void)
{
}


#if (APP_BATT_CHECK_ENABLE)  //battery check must do before OTA relative operation

_attribute_data_retention_	u32	lowBattDet_tick   = 0;

/**
 * @brief		this function is used to process battery power.
 * 				The low voltage protection threshold 2.0V is an example and reference value. Customers should
 * 				evaluate and modify these thresholds according to the actual situation. If users have unreasonable designs
 * 				in the hardware circuit, which leads to a decrease in the stability of the power supply network, the
 * 				safety thresholds must be increased as appropriate.
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ void user_battery_power_check(u16 alarm_vol_mv)
{
	/*For battery-powered products, as the battery power will gradually drop, when the voltage is low to a certain
	  value, it will cause many problems.
		a) When the voltage is lower than operating voltage range of chip, chip can no longer guarantee stable operation.
		b) When the battery voltage is low, due to the unstable power supply, the write and erase operations
			of Flash may have the risk of error, causing the program firmware and user data to be modified abnormally,
			and eventually causing the product to fail. */
	u8 battery_check_returnValue = 0;
	if(analog_read(USED_DEEP_ANA_REG) & LOW_BATT_FLG){
		battery_check_returnValue = app_battery_power_check(alarm_vol_mv+200);
	}
	else{
		battery_check_returnValue = app_battery_power_check(alarm_vol_mv);
	}
	if(battery_check_returnValue)
	{
		analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG) & (~LOW_BATT_FLG));  //clr
	}
	else
	{
		#if (UI_LED_ENABLE)  //led indicate
			for(int k=0;k<3;k++){
				gpio_write(GPIO_LED_BLUE, LED_ON_LEVEL);
				sleep_us(200000);
				gpio_write(GPIO_LED_BLUE, !LED_ON_LEVEL);
				sleep_us(200000);
			}
		#endif

		if(analog_read(USED_DEEP_ANA_REG) & LOW_BATT_FLG){
			printf("[APP][BAT] The battery voltage is lower than %dmV, shut down!!!\n", (alarm_vol_mv + 200));
		} else {
			printf("[APP][BAT] The battery voltage is lower than %dmV, shut down!!!\n", alarm_vol_mv);
		}


		analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG) | LOW_BATT_FLG);  //mark

		#if (UI_KEYBOARD_ENABLE)
		u32 pin[] = KB_DRIVE_PINS;
		for (int i=0; i<(sizeof (pin)/sizeof(*pin)); i++)
		{
			cpu_set_gpio_wakeup (pin[i], Level_High, 1);  //drive pin pad high wakeup deepsleep
		}

		cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  //deepsleep
		#endif
	}
}

#endif

u8 g_flash_uid[16];

static u8 g_fct_power_on = 1;

u8 app_get_power_state(void)
{
    return g_fct_power_on;
}

void app_set_power_state(u8 on)
{
    g_fct_power_on = on ? 1 : 0;
}

void app_save_power_state_to_flash(void)
{
}

void app_restore_power_state_from_flash(void)
{
    g_fct_power_on = 1;
}

void app_factory_test_enter(void)
{
}

u8 app_factory_test_is_active(void)
{
    return 1;
}

void app_factory_test_set_radar(u8 on)
{
    (void)on;
}

void app_factory_test_set_motor(u8 on)
{
    (void)on;
}

void app_factory_test_set_laser(u8 on)
{
    (void)on;
}

void app_get_flash_uid(u8 *uid, u8 len)
{
    if (!uid || len == 0)
    {
        return;
    }

    u8 copy_len = (len > sizeof(g_flash_uid)) ? sizeof(g_flash_uid) : len;
    memcpy(uid, g_flash_uid, copy_len);
}

void app_uart_ndma_irq_proc(void);

enum
{
    FCT_GPIO_PC0 = 0,
    FCT_GPIO_PC1,
    FCT_GPIO_PC2,
    FCT_GPIO_PC3,
    FCT_GPIO_PB7,
    FCT_GPIO_PB6,
    FCT_GPIO_PB5,
    FCT_GPIO_PB4,
    FCT_GPIO_PC6,
    FCT_GPIO_PD7,
};

static const GPIO_PinTypeDef g_fct_gpio_table[] = {
    GPIO_PC0, GPIO_PC1, GPIO_PC2, GPIO_PC3,
    GPIO_PB7, GPIO_PB6, GPIO_PB5, GPIO_PB4,
    GPIO_PC6, GPIO_PD7,
};

static u16 g_fct_bat_mv;
static u16 g_fct_ntc_mv;
static u8  g_fct_key_stable;
static u8  g_fct_key_candidate;
static u8  g_fct_usb_stable;
static u8  g_fct_usb_candidate;
static u32 g_fct_key_tick;
static u32 g_fct_usb_tick;
static u32 g_fct_adc_tick;
static u32 g_fct_adc_report_tick;
static u32 g_fct_bat_mv_sum;
static u32 g_fct_ntc_mv_sum;
static u16 g_fct_adc_sample_count;
static u32 g_fct_led_tick;
static u8  g_fct_led_step;

static void fct_gpio_output_init(GPIO_PinTypeDef pin, u8 level)
{
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, level);
}

static void fct_app_hw_init(void)
{
    fct_gpio_output_init(GPIO_LED_RED, !LED_ON_LEVEL);
    fct_gpio_output_init(GPIO_LED_GREEN, !LED_ON_LEVEL);
    fct_gpio_output_init(GPIO_LED_BLUE, !LED_ON_LEVEL);
    fct_gpio_output_init(GPIO_CHARGE_LED_GREEN, !LED_ON_LEVEL);
    fct_gpio_output_init(GPIO_CHARGE_LED_RED, !LED_ON_LEVEL);

	// 常开NTC电压AD检测开关
    gpio_write(V_NTC_CON, 1);
    // 常开电池电压AD检测开关
    gpio_write(V_BAT_CON, 1);
    gpio_write(CHARGE_SWITCH, 1);

    gpio_set_func(GPIO_KEY, AS_GPIO);
    gpio_set_input_en(GPIO_KEY, 1);
    gpio_set_output_en(GPIO_KEY, 0);
    gpio_setup_up_down_resistor(GPIO_KEY, PM_PIN_PULLUP_10K);

    gpio_set_func(USB_DET, AS_GPIO);
    gpio_set_input_en(USB_DET, 1);
    gpio_set_output_en(USB_DET, 0);
    gpio_setup_up_down_resistor(CHARGE_STATE, PM_PIN_PULLUP_10K);

    for (u8 i = 0; i < sizeof(g_fct_gpio_table) / sizeof(g_fct_gpio_table[0]); i++)
    {
        fct_gpio_output_init(g_fct_gpio_table[i], 0);
    }

    g_fct_key_stable = gpio_read(GPIO_KEY) ? 0 : 1;
    g_fct_key_candidate = g_fct_key_stable;
    g_fct_usb_stable = gpio_read(USB_DET) ? 1 : 0;
    g_fct_usb_candidate = g_fct_usb_stable;
    g_fct_key_tick = clock_time();
    g_fct_usb_tick = clock_time();
    g_fct_adc_tick = 0;
    g_fct_adc_report_tick = 0;
    g_fct_bat_mv_sum = 0;
    g_fct_ntc_mv_sum = 0;
    g_fct_adc_sample_count = 0;
    g_fct_led_tick = 0;
    g_fct_led_step = 0;
}

static void fct_app_led_task(void)
{
    if (g_fct_led_tick && !clock_time_exceed(g_fct_led_tick, 1000000))
    {
        return;
    }

    g_fct_led_tick = clock_time();

    gpio_write(GPIO_LED_RED, g_fct_led_step == 0 ? LED_ON_LEVEL : !LED_ON_LEVEL);
    gpio_write(GPIO_LED_GREEN, g_fct_led_step == 1 ? LED_ON_LEVEL : !LED_ON_LEVEL);
    gpio_write(GPIO_LED_BLUE, g_fct_led_step == 2 ? LED_ON_LEVEL : !LED_ON_LEVEL);
    gpio_write(GPIO_CHARGE_LED_GREEN, (g_fct_led_step & 1) ? LED_ON_LEVEL : !LED_ON_LEVEL);
    gpio_write(GPIO_CHARGE_LED_RED, (g_fct_led_step & 1) ? !LED_ON_LEVEL : LED_ON_LEVEL);
    g_fct_led_step = (u8)((g_fct_led_step + 1) % 3);
}

static u16 fct_app_adc_sample(adc_input_pin_def_e pin)
{
    adc_base_init(pin);
    return (u16)adc_sample_and_get_result();
}

static void fct_app_adc_task(void)
{
    u32 now = clock_time();

    if (g_fct_adc_tick == 0)
    {
        g_fct_adc_tick = now;
    }
    if (g_fct_adc_report_tick == 0)
    {
        g_fct_adc_report_tick = now;
    }

    /* Match app_adc_dbg.c: sample every 10ms and update the cached average every 500ms. */
    if (clock_time_exceed(g_fct_adc_tick, 10000))
    {
        g_fct_bat_mv_sum += fct_app_adc_sample(ADC_GPIO_PB2);
        g_fct_ntc_mv_sum += fct_app_adc_sample(ADC_GPIO_PC4);
        g_fct_adc_sample_count++;
        g_fct_adc_tick = now;
    }

    if (!clock_time_exceed(g_fct_adc_report_tick, 500000))
    {
        return;
    }

    if (g_fct_adc_sample_count)
    {
        u32 bat_adc_mv = g_fct_bat_mv_sum / g_fct_adc_sample_count;
        u32 ntc_adc_mv = g_fct_ntc_mv_sum / g_fct_adc_sample_count;

        /* app_adc_dbg.c uses a 660k/100k divider: battery = ADC * 6.6. */
        bat_adc_mv = (bat_adc_mv * 66u + 5u) / 10u;
        g_fct_bat_mv = (bat_adc_mv > 0xFFFFu) ? 0xFFFFu : (u16)bat_adc_mv;
        g_fct_ntc_mv = (ntc_adc_mv > 0xFFFFu) ? 0xFFFFu : (u16)ntc_adc_mv;
    }

    g_fct_bat_mv_sum = 0;
    g_fct_ntc_mv_sum = 0;
    g_fct_adc_sample_count = 0;
    g_fct_adc_report_tick = now;
}

static void fct_app_key_task(void)
{
    u8 raw = gpio_read(GPIO_KEY) ? 0 : 1;
    if (raw == g_fct_key_candidate)
    {
        if (clock_time_exceed(g_fct_key_tick, 20000) && raw != g_fct_key_stable)
        {
            g_fct_key_stable = raw;
            fct_uart_send_key(raw);
        }
    }
    else
    {
        g_fct_key_candidate = raw;
        g_fct_key_tick = clock_time();
    }
}

static void fct_app_usb_task(void)
{
    u8 raw = gpio_read(USB_DET) ? 1 : 0;

    if (raw == g_fct_usb_candidate)
    {
        if (clock_time_exceed(g_fct_usb_tick, 20000) && raw != g_fct_usb_stable)
        {
            g_fct_usb_stable = raw;
            fct_uart_send_usb(raw);
        }
    }
    else
    {
        g_fct_usb_candidate = raw;
        g_fct_usb_tick = clock_time();
    }
}

void fct_app_gpio_set(u8 gpio_id, u8 level)
{
    if (gpio_id >= sizeof(g_fct_gpio_table) / sizeof(g_fct_gpio_table[0]) || level > 1)
    {
        return;
    }
    gpio_write(g_fct_gpio_table[gpio_id], level ? LED_ON_LEVEL : !LED_ON_LEVEL);
    fct_uart_send_gpio(gpio_id, level);
}

void fct_app_gpio_set_all(u8 level)
{
    if (level > 1)
    {
        return;
    }

    for (u8 i = 0; i < sizeof(g_fct_gpio_table) / sizeof(g_fct_gpio_table[0]); i++)
    {
        gpio_write(g_fct_gpio_table[i], level ? LED_ON_LEVEL : !LED_ON_LEVEL);
    }
}

void fct_app_uid_send(void)
{
    fct_uart_send_event(FCT_EVT_UID, g_flash_uid, sizeof(g_flash_uid));
}

u16 fct_app_get_bat_mv(void)
{
    return g_fct_bat_mv;
}

u16 fct_app_get_ntc_mv(void)
{
    return g_fct_ntc_mv;
}

void fct_app_status_send(u8 seq)
{
    u8 payload[5] = {
        (u8)g_fct_bat_mv, (u8)(g_fct_bat_mv >> 8),
        (u8)g_fct_ntc_mv, (u8)(g_fct_ntc_mv >> 8),
        g_fct_key_stable,
    };
    fct_uart_send_event(FCT_EVT_ADC, payload, sizeof(payload));
    (void)seq;
}

void fct_app_enter_low_power(void)
{
    gpio_write(GPIO_LED_RED, !LED_ON_LEVEL);
    gpio_write(GPIO_LED_GREEN, !LED_ON_LEVEL);
    gpio_write(GPIO_LED_BLUE, !LED_ON_LEVEL);
    gpio_write(GPIO_CHARGE_LED_GREEN, !LED_ON_LEVEL);
    gpio_write(GPIO_CHARGE_LED_RED, !LED_ON_LEVEL);
    cpu_set_gpio_wakeup(GPIO_KEY, Level_Low, 1);
    gpio_setup_up_down_resistor(GPIO_KEY, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(CHARGE_SWITCH, PM_PIN_PULLUP_10K);
    cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);
}

void app_uart_ndma_irq_proc(void)
{
    fct_uart_ndma_irq_proc();
}

/**
 * @brief		user initialization when MCU power on or wake_up from deepSleep mode
 * @param[in]	none
 * @return      none
 */
_attribute_no_inline_ void user_init_normal(void)
{

//////////////////////////// basic hardware Initialization  Begin //////////////////////////////////

	/* random number generator must be initiated before any BLE stack initialization.
	 * When deepSleep retention wakeUp, no need initialize again */
	random_generator_init();

	blc_readFlashSize_autoConfigCustomFlashSector();

	/* attention that this function must be called after "blc_readFlashSize_autoConfigCustomFlashSector" !!!*/
	blc_app_loadCustomizedParameters_normal();


	/* attention that this function must be called after "blc_app_loadCustomizedParameters_normal" !!!
	   The reason is that the low battery check need the ADC calibration parameter, and this parameter
	   is loaded in blc_app_loadCustomizedParameters_normal.
	 */
	#if (APP_BATT_CHECK_ENABLE)
	/*The SDK must do a quick low battery detect during user initialization instead of waiting
	  until the main_loop. The reason for this process is to avoid application errors that the device
	  has already working at low power.
	  Considering the working voltage of MCU and the working voltage of flash, if the Demo is set below 2.0V,
	  the chip will alarm and deep sleep (Due to PM does not work in the current version of B92, it does not go
	  into deepsleep), and once the chip is detected to be lower than 2.0V, it needs to wait until the voltage rises to 2.2V,
	  the chip will resume normal operation. Consider the following points in this design:
		At 2.0V, when other modules are operated, the voltage may be pulled down and the flash will not
		work normally. Therefore, it is necessary to enter deepsleep below 2.0V to ensure that the chip no
		longer runs related modules;
		When there is a low voltage situation, need to restore to 2.2V in order to make other functions normal,
		this is to ensure that the power supply voltage is confirmed in the charge and has a certain amount of
		power, then start to restore the function can be safer.*/
		user_battery_power_check(VBAT_ALARM_THRES_MV);
	#endif


	#if (APP_FLASH_PROTECTION_ENABLE)
		app_flash_protection_operation(FLASH_OP_EVT_APP_INITIALIZATION, 0, 0);
		blc_appRegisterStackFlashOperationCallback(app_flash_protection_operation); //register flash operation callback for stack
	#endif



//////////////////////////// basic hardware Initialization  End //////////////////////////////////




//////////////////////////// BLE stack Initialization  Begin //////////////////////////////////
	//////////// Controller Initialization  Begin /////////////////////////
	u8  mac_public[6];
	u8  mac_random_static[6];
	/* for 512K Flash, flash_sector_mac_address equals to 0x7F000, for 128K  Flash, flash_sector_mac_address equals to 0x1F000 */
	blc_initMacAddress(flash_sector_mac_address, mac_public, mac_random_static);


	#if(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
		app_own_address_type = OWN_ADDRESS_PUBLIC;
	#elif(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
		app_own_address_type = OWN_ADDRESS_RANDOM;
		blc_ll_setRandomAddr(mac_random_static);
	#endif

	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(mac_public);				//mandatory
	blc_ll_initAdvertising_module();//legacy advertising module: mandatory for BLE slave
	blc_ll_initSlaveRole_module();//slave module: mandatory for BLE slave,

	blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS, ACL_CONN_MAX_TX_OCTETS);
	blc_ll_initAclConnTxFifo(app_acl_txfifo, ACL_TX_FIFO_SIZE, ACL_TX_FIFO_NUM);
	blc_ll_initAclConnRxFifo(app_acl_rxfifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);


	//////////// Controller Initialization  End /////////////////////////




	//////////// Host Initialization  Begin /////////////////////////
	/* GAP initialization must be done before any other host feature initialization !!! */
	blc_gap_peripheral_init();    //gap initialization

	/* GATT Initialization */
	my_gatt_init();


	/* L2CAP Initialization */
	blc_l2cap_register_handler(blc_l2cap_packet_receive);
	blc_l2cap_initRxDataBuffer(app_l2cap_rx_fifo, L2CAP_RX_BUFF_SIZE);

	/* SMP Initialization */
	/* SMP Initialization may involve flash write/erase(when one sector stores too much information,
	 *   is about to exceed the sector threshold, this sector must be erased, and all useful information
	 *   should re_stored) , so it must be done after battery check */
	#if (BLE_APP_SECURITY_ENABLE)
		bls_smp_configPairingSecurityInfoStorageAddr(flash_sector_smp_storage);
		blc_smp_param_setBondingDeviceMaxNumber(4);  	//default is 4, can not bigger than this value
												    	//and this func must call before bls_smp_enableParing
		blc_smp_peripheral_init();
		blc_smp_setSecurityLevel(Unauthenticated_Pairing_with_Encryption);

	#else
		blc_smp_setSecurityLevel(No_Security);
	#endif


	//////////// Host Initialization  End /////////////////////////

	//////////// Service Initialization  Begin /////////////////////////
	#if(BLE_OTA_SERVER_ENABLE)
		/* OTA module initialization must be called after "blc_ota_setNewFirmwareStorageAddress"(if used), and before any other OTA API.*/
		blc_ota_initOtaServer_module();
		blc_ota_setOtaProcessTimeout(60); //set OTA whole process timeout
		blc_ota_registerOtaStartCmdCb(app_enter_ota_mode);
		blc_ota_registerOtaResultIndicationCb(app_ota_result);
	#endif
	//////////// Service Initialization  End   /////////////////////////

//////////////////////////// BLE stack Initialization  End //////////////////////////////////


//////////////////////////// User Configuration for BLE application ////////////////////////////
	////////////////// config ADV packet /////////////////////
	u8 adv_param_status = BLE_SUCCESS;
	#if(BLE_APP_SECURITY_ENABLE)
		u8 bond_number = blc_smp_param_getCurrentBondingDeviceNumber(); 		//get bonded device number
		smp_param_save_t  bondInfo;
		if(bond_number)   //at least 1 bonding device exist
		{
			//get the latest bonding device (index: bond_number-1 )
			bls_smp_param_loadByIndex( bond_number - 1, &bondInfo);

			/* set direct ADV
			 * bondInfo.peer_addr_type & bondInfo.peer_addr is the address in the air packet of "CONNECT_IND" PDU stored in Flash.
			 * if peer address is IDA(identity address), bondInfo.peer_addr is OK used here.
			 * if peer address is RPA(resolved private address), bondInfo.peer_addr is one RPA peer device has used, it has a correct relation
			 * with peer IRK, so it can match to peer device at any time even peer device changes it's RPA. */
			adv_param_status = bls_ll_setAdvParam( MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
											ADV_TYPE_CONNECTABLE_DIRECTED_LOW_DUTY, app_own_address_type,
											bondInfo.peer_addr_type,  bondInfo.peer_addr,
											MY_APP_ADV_CHANNEL,
											ADV_FP_NONE);

			/* If IRK distributed by peer device is valid, peer device may use RPA(resolved private address) at any time,
			 * even if it used IDA(identity address) in first pairing phase.
			 * So here must add peer IRK to resolving list and enable address resolution, since local device should check if
			 * "CONNECT_IND" PDU is sent by the device directed to.
			 * attention: local RPA not used, so parameter "local_irk" set to NULL */
			if(blc_app_isIrkValid(bondInfo.peer_irk)){
				ll_resolvingList_add(bondInfo.peer_id_adrType, bondInfo.peer_id_addr, bondInfo.peer_irk, NULL);
				ll_resolvingList_setAddrResolutionEnable(1);
			}

			//it is recommended that direct ADV only last for several seconds, then switch to undirected adv
			bls_ll_setAdvDuration(MY_DIRECT_ADV_TIME, 1);
			bls_app_registerEventCallback (BLT_EV_FLAG_ADV_DURATION_TIMEOUT, &app_switch_to_undirected_adv);

		}
		else   //set undirected adv
	#endif
		{
			adv_param_status = bls_ll_setAdvParam(  MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
											 ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type,
											 0,  NULL,
											 MY_APP_ADV_CHANNEL,
											 ADV_FP_NONE);
		}

	if(adv_param_status != BLE_SUCCESS){
		printf("[APP][INI] ADV parameters error 0x%x!!!\n", adv_param_status);
	}

	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
    unsigned int flash_mid;
    int          flag = flash_read_mid_uid_with_check(&flash_mid, g_flash_uid);
    for (int i = 0; i < 16; i++)
    {
        tbl_scanRsp[15 + i] = g_flash_uid[i];
    }
    if (flag == 0)
    {  // reading flash UID error
        LOG_D("[APP][INI] reading flash UID error");
    }
    bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));
	bls_ll_setAdvEnable(BLC_ADV_ENABLE);  //ADV enable

	/* set RF power index, user must set it after every suspend wake_up, because relative setting will be reset in suspend */
	rf_set_power_level_index (MY_RF_POWER_INDEX);

	bls_app_registerEventCallback (BLT_EV_FLAG_CONNECT, &task_connect);
	bls_app_registerEventCallback (BLT_EV_FLAG_TERMINATE, &task_terminate);
	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_EXIT, &task_suspend_exit);


	///////////////////// Power Management initialization///////////////////
	#if(BLE_APP_PM_ENABLE)
		#if (SAVE_RAM_CODE_ENABLE)
			blc_ll_initPowerManagement_module_save_ram();        //pm module:      	 optional
		#else
			blc_ll_initPowerManagement_module();        //pm module:      	 optional
		#endif
		#if (PM_DEEPSLEEP_RETENTION_ENABLE)
			#if (!SAVE_RAM_CODE_ENABLE)
				blc_ll_initDeepsleepRetention_module();//Remove it if need save ramcode, and add DeepsleepRetentionEarlyWakeupTiming.
																	  //Note:The user needs to DeepsleepRetentionEarlyWakeupTiming according to the timing from boot to the end of the user_init_deepRetn.Refer to handbook for details.
			#endif
			bls_pm_setSuspendMask (SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);
			blc_pm_setDeepsleepRetentionThreshold(95, 95);
			#if (MCU_CORE_B80)
				blc_pm_setDeepsleepRetentionEarlyWakeupTiming(650);
			#elif (MCU_CORE_B80B)
				blc_pm_setDeepsleepRetentionEarlyWakeupTiming(550);
			#endif
		#else
			bls_pm_setSuspendMask (SUSPEND_DISABLE);
		#endif

		bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_ENTER, &task_sleep_enter);
	#else
		bls_pm_setSuspendMask (SUSPEND_DISABLE);
	#endif


	#if (UI_KEYBOARD_ENABLE)
		/////////// keyboard gpio wakeup init ////////
		u32 pin[] = KB_DRIVE_PINS;
		for (int i=0; i<(sizeof (pin)/sizeof(*pin)); i++)
		{
			cpu_set_gpio_wakeup (pin[i], Level_High,1);  //drive pin pad high wakeup deepsleep
		}

		bls_app_registerEventCallback (BLT_EV_FLAG_GPIO_EARLY_WAKEUP, &proc_keyboard);
	#endif
////////////////////////////////////////////////////////////////////////////////////////////////

	/* Check if any Stack(Controller & Host) Initialization error after all BLE initialization done.
	 * attention that code will stuck in "while(1)" if any error detected in initialization, user need find what error happens and then fix it */
	blc_app_checkControllerHostInitialization();

	advertise_begin_tick = clock_time();
	adc_init();
	adc_power_on_sar_adc(1);
	fct_app_hw_init();
	fct_uart_init();
}
#if (PM_DEEPSLEEP_RETENTION_ENABLE)
/**
 * @brief		user initialization when MCU wake_up from deepSleep_retention mode
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_
void user_init_deepRetn(void)
{

	blc_app_loadCustomizedParameters_deepRetn();
	//////////// LinkLayer Initialization  Begin /////////////////////////
	blc_ll_initBasicMCU();                      //mandatory

//////////////////////////// User Configuration for BLE application ////////////////////////////

	/* set rf power index, user must set it after every suspend wakeup, cause relative setting will be reset in suspend */
	rf_set_power_level_index (MY_RF_POWER_INDEX);
	blc_ll_recoverDeepRetention();
	irq_enable();

	#if (UI_KEYBOARD_ENABLE)
		/////////// keyboard GPIO wake_up initialization ////////
		u32 pin[] = KB_DRIVE_PINS;
		for (int i=0; i<(sizeof (pin)/sizeof(*pin)); i++)
		{
			cpu_set_gpio_wakeup (pin[i], Level_High,1);  //drive pin high level wake_up deepsleep
		}
	#endif
////////////////////////////////////////////////////////////////////////////////////////////////
}
#endif

#if (APP_FLASH_PROTECTION_ENABLE)

/**
 * @brief      flash protection operation, including all locking & unlocking for application
 * 			   handle all flash write & erase action for this demo code. use should add more more if they have more flash operation.
 * @param[in]  flash_op_evt - flash operation event, including application layer action and stack layer action event(OTA write & erase)
 * 			   attention 1: if you have more flash write or erase action, you should should add more type and process them
 * 			   attention 2: for "end" event, no need to pay attention on op_addr_begin & op_addr_end, we set them to 0 for
 * 			   			    stack event, such as stack OTA write new firmware end event
 * @param[in]  op_addr_begin - operating flash address range begin value
 * @param[in]  op_addr_end - operating flash address range end value
 * 			   attention that, we use: [op_addr_begin, op_addr_end)
 * 			   e.g. if we write flash sector from 0x10000 to 0x20000, actual operating flash address is 0x10000 ~ 0x1FFFF
 * 			   		but we use [0x10000, 0x20000):  op_addr_begin = 0x10000, op_addr_end = 0x20000
 * @return     none
 */
_attribute_data_retention_ u16  flash_lockBlock_cmd = 0;
void app_flash_protection_operation(u8 flash_op_evt, u32 op_addr_begin, u32 op_addr_end)
{
	if(flash_op_evt == FLASH_OP_EVT_APP_INITIALIZATION)
	{
		/* ignore "op addr_begin" and "op addr_end" for initialization event
		 * must call "flash protection_init" first, will choose correct flash protection relative API according to current internal flash type in MCU */
		flash_protection_init();

		/* just sample code here, protect all flash area for old firmware and OTA new firmware.
		 * user can change this design if have other consideration */
		u32  app_lockBlock = 0xffffffff;
		#if (BLE_OTA_SERVER_ENABLE)
			u32 multiBootAddress = blc_ota_getCurrentUsedMultipleBootAddress();
			if(multiBootAddress == MULTI_BOOT_ADDR_0x10000){
				app_lockBlock = FLASH_LOCK_FW_LOW_128K;
			}
			else if(multiBootAddress == MULTI_BOOT_ADDR_0x20000){
				app_lockBlock = FLASH_LOCK_FW_LOW_256K;
			}
			else if(multiBootAddress == MULTI_BOOT_ADDR_0x40000){
				/* attention that 512K capacity flash can not lock all 512K area, should leave some upper sector
				 * for system data(SMP storage data & calibration data & MAC address) and user data
				 * will use a approximate value */
				app_lockBlock = FLASH_LOCK_FW_LOW_512K;
			}
		#else
			app_lockBlock = FLASH_LOCK_FW_LOW_256K; //just demo value, user can change this value according to application
		#endif


		flash_lockBlock_cmd = flash_change_app_lock_block_to_flash_lock_block(app_lockBlock);

		if(blc_flashProt.init_err){
			printf("[FLASH][PROT] flash protection initialization error!!!\n");
		}

		printf("[FLASH][PROT] initialization, lock flash\n");
		flash_lock(flash_lockBlock_cmd);
	}
#if (BLE_OTA_SERVER_ENABLE)
	else if(flash_op_evt == FLASH_OP_EVT_STACK_OTA_CLEAR_OLD_FW_BEGIN)
	{
		/* OTA clear old firmware begin event is triggered by stack, in "blc ota_initOtaServer_module", rebooting from a successful OTA.
		 * Software will erase whole old firmware for potential next new OTA, need unlock flash if any part of flash address from
		 * "op addr_begin" to "op addr_end" is in locking block area.
		 * In this sample code, we protect whole flash area for old and new firmware, so here we do not need judge "op addr_begin" and "op addr_end",
		 * must unlock flash */
		printf("[FLASH][PROT] OTA clear old FW begin, unlock flash\n");
		flash_unlock();
	}
	else if(flash_op_evt == FLASH_OP_EVT_STACK_OTA_CLEAR_OLD_FW_END)
	{
		/* ignore "op addr_begin" and "op addr_end" for END event
		 * OTA clear old firmware end event is triggered by stack, in "blc ota_initOtaServer_module", erasing old firmware data finished.
		 * In this sample code, we need lock flash again, because we have unlocked it at the begin event of clear old firmware */
		printf("[FLASH][PROT] OTA clear old FW end, restore flash locking\n");
		flash_lock(flash_lockBlock_cmd);
	}
	else if(flash_op_evt == FLASH_OP_EVT_STACK_OTA_WRITE_NEW_FW_BEGIN)
	{
		/* OTA write new firmware begin event is triggered by stack, when receive first OTA data PDU.
		 * Software will write data to flash on new firmware area,  need unlock flash if any part of flash address from
		 * "op addr_begin" to "op addr_end" is in locking block area.
		 * In this sample code, we protect whole flash area for old and new firmware, so here we do not need judge "op addr_begin" and "op addr_end",
		 * must unlock flash */
		printf( "[FLASH][PROT] OTA write new FW begin, unlock flash\n");
		flash_unlock();
	}
	else if(flash_op_evt == FLASH_OP_EVT_STACK_OTA_WRITE_NEW_FW_END)
	{
		/* ignore "op addr_begin" and "op addr_end" for END event
		 * OTA write new firmware end event is triggered by stack, after OTA end or an OTA error happens, writing new firmware data finished.
		 * In this sample code, we need lock flash again, because we have unlocked it at the begin event of write new firmware */
		printf("[FLASH][PROT] OTA write new FW end, restore flash locking\n");
		flash_lock(flash_lockBlock_cmd);
	}
#endif
	/* add more flash protection operation for your application if needed */
}


#endif




/**
 * @brief     BLE main loop
 * @param[in]  none.
 * @return     none.
 */
void main_loop (void)
{

	////////////////////////////////////// BLE entry /////////////////////////////////
	blc_sdk_main_loop();

	fct_uart_task();
	fct_app_led_task();
	fct_app_adc_task();
	fct_app_key_task();
	fct_app_usb_task();

	////////////////////////////////////// PM Process /////////////////////////////////
	blt_pm_proc();

}
