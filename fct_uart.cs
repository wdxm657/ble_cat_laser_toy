#include "tl_common.h"
#include "drivers.h"
#include "app_config.h"
#include "fct_uart.h"

extern void fct_app_gpio_set(u8 gpio_id, u8 level);
extern void fct_app_gpio_set_all(u8 level);
extern void fct_app_enter_low_power(void);
extern void fct_app_status_send(u8 seq);
extern u16 fct_app_get_bat_mv(void);
extern u16 fct_app_get_ntc_mv(void);
extern u8 g_flash_uid[16];

#define FCT_FRAME_HEAD0 0x55
#define FCT_FRAME_HEAD1 0xAA
#define FCT_FRAME_OVERHEAD 10
#define FCT_FRAME_MAX_SIZE (FCT_FRAME_OVERHEAD + FCT_UART_MAX_PAYLOAD)

static volatile u8 g_fct_rx_frame[FCT_FRAME_MAX_SIZE];
static volatile u8 g_fct_rx_frame_len;
static volatile u8 g_fct_rx_expected_len;
static volatile u8 g_fct_rx_frame_ready;
static u8 g_fct_seq;

static u16 fct_crc16(const u8 *data, u16 len)
{
    u16 crc = 0xFFFF;
    for (u16 i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (u8 b = 0; b < 8; b++)
        {
            crc = (crc & 1) ? (u16)((crc >> 1) ^ 0xA001) : (u16)(crc >> 1);
        }
    }
    return crc;
}

static void fct_uart_wait_tx_idle(void)
{
    u32 tick = clock_time();
    while (uart_tx_is_busy() && !clock_time_exceed(tick, 100000))
    {
    }
}

static void fct_uart_send_frame(u8 msg_type, u8 cmd_id, u8 seq, const u8 *payload, u16 payload_len)
{
    if (payload_len > FCT_UART_MAX_PAYLOAD)
    {
        return;
    }

    u8 frame[10 + FCT_UART_MAX_PAYLOAD];
    u16 i = 0;
    frame[i++] = FCT_FRAME_HEAD0;
    frame[i++] = FCT_FRAME_HEAD1;
    frame[i++] = FCT_UART_PROTO_VERSION;
    frame[i++] = msg_type;
    frame[i++] = cmd_id;
    frame[i++] = seq;
    frame[i++] = (u8)payload_len;
    frame[i++] = (u8)(payload_len >> 8);
    if (payload_len && payload)
    {
        memcpy(&frame[i], payload, payload_len);
        i = (u16)(i + payload_len);
    }
    u16 crc = fct_crc16(&frame[2], (u16)(6 + payload_len));
    frame[i++] = (u8)crc;
    frame[i++] = (u8)(crc >> 8);

    fct_uart_wait_tx_idle();

    for (u16 n = 0; n < i; n++)
    {
        uart_ndma_send_byte(frame[n]);
        sleep_us(120);
    }
    fct_uart_wait_tx_idle();
}

static void fct_uart_send_rsp(u8 cmd_id, u8 seq, u8 status, const u8 *payload, u16 payload_len)
{
    u8 rsp[FCT_UART_MAX_PAYLOAD];
    rsp[0] = status;
    if (payload_len > FCT_UART_MAX_PAYLOAD - 1)
    {
        payload_len = FCT_UART_MAX_PAYLOAD - 1;
    }
    if (payload_len && payload)
    {
        memcpy(&rsp[1], payload, payload_len);
    }
    fct_uart_send_frame(FCT_UART_MSG_RSP, cmd_id, seq, rsp, (u16)(payload_len + 1));
}

void fct_uart_send_event(u8 cmd_id, const u8 *payload, u16 payload_len)
{
    fct_uart_send_frame(FCT_UART_MSG_EVT, cmd_id, g_fct_seq++, payload, payload_len);
}

void fct_uart_send_key(u8 pressed)
{
    fct_uart_send_event(FCT_EVT_KEY, &pressed, 1);
}

void fct_uart_send_usb(u8 inserted)
{
    fct_uart_send_event(FCT_EVT_USB, &inserted, 1);
}

void fct_uart_send_gpio(u8 gpio_id, u8 level)
{
    u8 payload[2] = {gpio_id, level};
    fct_uart_send_event(FCT_EVT_GPIO, payload, sizeof(payload));
}

void fct_uart_init(void)
{
    uart_gpio_set(GPIO_PD3, GPIO_PD2);
    uart_reset();
    uart_init_baudrate(FCT_UART_BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_dma_enable(0, 0);
    uart_ndma_clear_rx_index();
    uart_ndma_clear_tx_index();
    uart_ndma_irq_triglevel(1, 0);
    uart_clear_parity_error();
    g_fct_rx_frame_len = 0;
    g_fct_rx_expected_len = 0;
    g_fct_rx_frame_ready = 0;
    g_fct_seq = 0;
    uart_irq_enable(1, 0);
}

static void fct_uart_rx_push_byte(u8 data)
{
    if (g_fct_rx_frame_ready)
    {
        return;
    }

    if (g_fct_rx_frame_len == 0)
    {
        if (data == FCT_FRAME_HEAD0)
        {
            g_fct_rx_frame[0] = data;
            g_fct_rx_frame_len = 1;
        }
        return;
    }

    if (g_fct_rx_frame_len == 1)
    {
        if (data == FCT_FRAME_HEAD1)
        {
            g_fct_rx_frame[1] = data;
            g_fct_rx_frame_len = 2;
        }
        else
        {
            g_fct_rx_frame_len = (data == FCT_FRAME_HEAD0) ? 1 : 0;
            if (g_fct_rx_frame_len)
            {
                g_fct_rx_frame[0] = data;
            }
        }
        return;
    }

    if (g_fct_rx_frame_len >= FCT_FRAME_MAX_SIZE)
    {
        g_fct_rx_frame_len = 0;
        g_fct_rx_expected_len = 0;
        return;
    }

    g_fct_rx_frame[g_fct_rx_frame_len++] = data;

    if (g_fct_rx_frame_len == 8)
    {
        u16 payload_len = (u16)g_fct_rx_frame[6] | ((u16)g_fct_rx_frame[7] << 8);
        if (g_fct_rx_frame[2] != FCT_UART_PROTO_VERSION ||
            g_fct_rx_frame[3] != FCT_UART_MSG_CMD ||
            payload_len > FCT_UART_MAX_PAYLOAD)
        {
            g_fct_rx_frame_len = 0;
            g_fct_rx_expected_len = 0;
            return;
        }
        g_fct_rx_expected_len = (u8)(FCT_FRAME_OVERHEAD + payload_len);
    }

    if (g_fct_rx_expected_len && g_fct_rx_frame_len == g_fct_rx_expected_len)
    {
        g_fct_rx_frame_ready = 1;
    }
}

void fct_uart_ndma_irq_proc(void)
{
    if (!uart_ndmairq_get())
    {
        return;
    }
    u8 count = (u8)(reg_uart_buf_cnt & 0x0F);
    while (count--)
    {
        fct_uart_rx_push_byte(uart_ndma_read_byte());
    }
}

static void fct_handle_frame(const u8 *frame)
{
    u8 cmd = frame[4];
    u8 seq = frame[5];
    u16 len = (u16)frame[6] | ((u16)frame[7] << 8);
    const u8 *payload = &frame[8];

    if (cmd == FCT_CMD_GPIO_SET)
    {
        if (len != 2)
        {
            fct_uart_send_rsp(cmd, seq, FCT_STATUS_LEN_ERROR, NULL, 0);
        }
        else
        {
            fct_app_gpio_set(payload[0], payload[1]);
            fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, NULL, 0);
        }
    }
    else if (cmd == FCT_CMD_GPIO_ALL_SET)
    {
        if (len != 1 || payload[0] > 1)
        {
            fct_uart_send_rsp(cmd, seq, FCT_STATUS_PARAM_ERROR, NULL, 0);
        }
        else
        {
            fct_app_gpio_set_all(payload[0]);
            fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, NULL, 0);
        }
    }
    else if (cmd == FCT_CMD_UID_READ)
    {
        fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, g_flash_uid, sizeof(g_flash_uid));
    }
    else if (cmd == FCT_CMD_BAT_ADC_READ || cmd == FCT_CMD_NTC_ADC_READ)
    {
        u16 value = (cmd == FCT_CMD_BAT_ADC_READ) ?
                    fct_app_get_bat_mv() : fct_app_get_ntc_mv();
        u8 value_payload[2] = {(u8)value, (u8)(value >> 8)};
        fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, value_payload, sizeof(value_payload));
    }
    else if (cmd == FCT_CMD_LOW_POWER)
    {
        fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, NULL, 0);
        fct_app_enter_low_power();
    }
    else if (cmd == FCT_CMD_STATUS_GET)
    {
        fct_app_status_send(seq);
    }
    else
    {
        fct_uart_send_rsp(cmd, seq, FCT_STATUS_UNSUPPORTED, NULL, 0);
    }
}

void fct_uart_task(void)
{
    u8 frame[FCT_FRAME_MAX_SIZE];
    u8 frame_len;

    if (!g_fct_rx_frame_ready)
    {
        return;
    }

    uart_irq_enable(0, 0);
    frame_len = g_fct_rx_frame_len;
    memcpy(frame, (const void *)g_fct_rx_frame, frame_len);
    g_fct_rx_frame_len = 0;
    g_fct_rx_expected_len = 0;
    g_fct_rx_frame_ready = 0;
    uart_irq_enable(1, 0);

    if (frame_len < FCT_FRAME_OVERHEAD)
    {
        return;
    }

    u16 payload_len = (u16)frame[6] | ((u16)frame[7] << 8);
    u16 rx_crc = (u16)frame[frame_len - 2] | ((u16)frame[frame_len - 1] << 8);
    u16 calc_crc = fct_crc16(&frame[2], (u16)(6 + payload_len));
    if (frame_len != (u8)(FCT_FRAME_OVERHEAD + payload_len) || rx_crc != calc_crc)
    {
        return;
    }

    fct_handle_frame(frame);
}
