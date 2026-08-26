#ifndef FCT_UART_H
#define FCT_UART_H

#include "tl_common.h"

#define FCT_UART_BAUDRATE       115200
#define FCT_UART_PROTO_VERSION  0x01
#define FCT_UART_MSG_CMD        0x01
#define FCT_UART_MSG_RSP        0x02
#define FCT_UART_MSG_EVT        0x03
#define FCT_UART_MAX_PAYLOAD    64

enum
{
    FCT_CMD_GPIO_SET = 0x10,
    FCT_CMD_GPIO_ALL_SET = 0x11,
    FCT_CMD_UID_READ = 0x20,
    FCT_CMD_BAT_ADC_READ = 0x21,
    FCT_CMD_NTC_ADC_READ = 0x22,
    FCT_CMD_FW_VERSION_READ = 0x23,
    FCT_CMD_LOW_POWER = 0x30,
    FCT_CMD_STATUS_GET = 0x40,
};

enum
{
    FCT_EVT_ADC = 0x80,
    FCT_EVT_KEY = 0x81,
    FCT_EVT_GPIO = 0x82,
    FCT_EVT_UID = 0x83,
    FCT_EVT_USB = 0x84,
};

enum
{
    FCT_STATUS_OK = 0x00,
    FCT_STATUS_LEN_ERROR = 0x01,
    FCT_STATUS_UNSUPPORTED = 0x02,
    FCT_STATUS_PARAM_ERROR = 0x03,
};

void fct_uart_init(void);
void fct_uart_ndma_irq_proc(void);
void fct_uart_task(void);
void fct_uart_send_event(u8 cmd_id, const u8 *payload, u16 payload_len);
void fct_uart_send_key(u8 pressed);
void fct_uart_send_usb(u8 inserted);
void fct_uart_send_gpio(u8 gpio_id, u8 level);

#endif
