#include "tl_common.h"
#include "drivers.h"
#include "app_config.h"
#include "fct_uart.h"

extern void fct_app_gpio_set(u8 gpio_id, u8 level);
extern void fct_app_enter_low_power(void);
extern void fct_app_status_send(u8 seq);
extern u8 g_flash_uid[16];

#define FCT_FRAME_HEAD0 0x55
#define FCT_FRAME_HEAD1 0xAA
#define FCT_FRAME_OVERHEAD 10
#define FCT_RX_BUFFER_SIZE 128

static volatile u8 g_fct_rx_buf[FCT_RX_BUFFER_SIZE];
static volatile u8 g_fct_rx_len;
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

static void fct_drop_rx(u16 count)
{
    if (count >= g_fct_rx_len)
    {
        g_fct_rx_len = 0;
        return;
    }
    memmove((void *)g_fct_rx_buf, (const void *)&g_fct_rx_buf[count], g_fct_rx_len - count);
    g_fct_rx_len = (u8)(g_fct_rx_len - count);
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

    uart_irq_enable(0, 0);
    fct_uart_wait_tx_idle();
    uart_ndma_clear_tx_index();

    for (u16 n = 0; n < i; n++)
    {
        uart_ndma_send_byte(frame[n]);
        sleep_us(120);
    }
    fct_uart_wait_tx_idle();
    uart_irq_enable(1, 0);
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

void fct_uart_send_adc(u16 bat_mv, u16 ntc_mv)
{
    u8 payload[4] = {(u8)bat_mv, (u8)(bat_mv >> 8), (u8)ntc_mv, (u8)(ntc_mv >> 8)};
    fct_uart_send_event(FCT_EVT_ADC, payload, sizeof(payload));
}

void fct_uart_send_key(u8 pressed)
{
    fct_uart_send_event(FCT_EVT_KEY, &pressed, 1);
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
    g_fct_rx_len = 0;
    g_fct_seq = 0;
    uart_irq_enable(1, 0);
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
        u8 data = uart_ndma_read_byte();
        if (g_fct_rx_len < sizeof(g_fct_rx_buf))
        {
            g_fct_rx_buf[g_fct_rx_len++] = data;
        }
        else
        {
            g_fct_rx_len = 0;
            if (data == FCT_FRAME_HEAD0)
            {
                g_fct_rx_buf[g_fct_rx_len++] = data;
            }
        }
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
    else if (cmd == FCT_CMD_UID_READ)
    {
        fct_uart_send_rsp(cmd, seq, FCT_STATUS_OK, g_flash_uid, sizeof(g_flash_uid));
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
    while (g_fct_rx_len >= FCT_FRAME_OVERHEAD)
    {
        u16 start = 0;
        while ((start + 1) < g_fct_rx_len &&
               !(g_fct_rx_buf[start] == FCT_FRAME_HEAD0 && g_fct_rx_buf[start + 1] == FCT_FRAME_HEAD1))
        {
            start++;
        }
        if ((start + 1) >= g_fct_rx_len)
        {
            g_fct_rx_len = (g_fct_rx_buf[g_fct_rx_len - 1] == FCT_FRAME_HEAD0) ? 1 : 0;
            if (g_fct_rx_len)
            {
                g_fct_rx_buf[0] = FCT_FRAME_HEAD0;
            }
            return;
        }
        if (start)
        {
            fct_drop_rx(start);
        }
        if (g_fct_rx_len < FCT_FRAME_OVERHEAD)
        {
            return;
        }

        u16 payload_len = (u16)g_fct_rx_buf[6] | ((u16)g_fct_rx_buf[7] << 8);
        u16 frame_len = (u16)(10 + payload_len);
        if (g_fct_rx_buf[2] != FCT_UART_PROTO_VERSION ||
            g_fct_rx_buf[3] != FCT_UART_MSG_CMD ||
            payload_len > FCT_UART_MAX_PAYLOAD ||
            frame_len > sizeof(g_fct_rx_buf))
        {
            fct_drop_rx(1);
            continue;
        }
        if (g_fct_rx_len < frame_len)
        {
            return;
        }

        u16 rx_crc = (u16)g_fct_rx_buf[frame_len - 2] | ((u16)g_fct_rx_buf[frame_len - 1] << 8);
        u16 calc_crc = fct_crc16((const u8 *)&g_fct_rx_buf[2], (u16)(6 + payload_len));
        if (rx_crc == calc_crc)
        {
            fct_handle_frame((const u8 *)g_fct_rx_buf);
            fct_drop_rx(frame_len);
        }
        else
        {
            fct_drop_rx(1);
        }
    }
}
