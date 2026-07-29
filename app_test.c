/********************************************************************************************************
 * @file    app_test.c
 *
 * @brief   PCBA FCT test helper for ble_cat_laser_toy.
 *
 * This file is intentionally a small production-test module instead of a full
 * copy of app.c. The final FCT protocol is not available yet, so the temporary
 * protocol is ASCII line based and easy for the factory fixture to adapt:
 *
 *   FCT control UART: 115200 8N1, TX=PD3, RX=PD2
 *   UART-under-test: 9600 8N1 software UART, TX=PD6, RX=PD5
 *   Line ending: '\r', '\n', or "\r\n"
 *
 * Commands:
 *   PING                  -> OK PONG
 *   VER?                  -> OK VER PCBA_TEST,1
 *   VBAT?                 -> OK VBAT pin_mv=<mV> bat_mv=<mV>
 *   NTC?                  -> OK NTC pin_mv=<mV>
 *   MOTOR FWD             -> OK MOTOR FWD START, then OK MOTOR FWD DONE
 *   MOTOR REV             -> OK MOTOR REV START, then OK MOTOR REV DONE
 *   UART TEST             -> test software UART TX=PD6, RX=PD5 loopback/echo
 *   GPIO <name> <0|1>     -> OK GPIO <name> <0|1>
 *   GPIO ALL 0            -> OK GPIO ALL 0
 *   SLEEP                 -> OK SLEEP ENTER, then enter deep sleep
 *
 * GPIO names:
 *   LED_BLUE, LED_GREEN, LED_RED, LED_ALL,
 *   BAT_GREEN, BAT_RED, BAT_ALL,
 *   LASER, RADAR, VBAT_EN, NTC_EN, CHARGE_LIMIT
 *
 * Integration note:
 *   Call app_pcba_test_init() during product-test firmware initialization,
 *   app_pcba_test_uart_irq_proc() from the UART IRQ handler, and
 *   app_pcba_test_task() from the main loop.
 *******************************************************************************************************/

#include "tl_common.h"
#include "drivers.h"
#include "app_config.h"
#include "StepMotor.h"

#ifndef APP_PCBA_TEST_ENABLE
#define APP_PCBA_TEST_ENABLE 1
#endif

#if (APP_PCBA_TEST_ENABLE)

#define PCBA_TEST_UART_BAUDRATE          115200
#define PCBA_TEST_UART_TX_PIN            GPIO_PD3
#define PCBA_TEST_UART_RX_PIN            GPIO_PD2
#define PCBA_TEST_AUX_UART_BAUDRATE      9600
#define PCBA_TEST_AUX_UART_TX_PIN        GPIO_PD6
#define PCBA_TEST_AUX_UART_RX_PIN        GPIO_PD5
#define PCBA_TEST_AUX_UART_BIT_US        (1000000 / PCBA_TEST_AUX_UART_BAUDRATE)
#define PCBA_TEST_AUX_UART_TIMEOUT_US    (200 * 1000)
#define PCBA_TEST_RX_LINE_MAX            64
#define PCBA_TEST_MOTOR_STEP_INTERVAL_US 1000
#define PCBA_TEST_KEY_DEBOUNCE_US        (20 * 1000)

static const char s_aux_uart_test_pattern[] = "PCBA_UART_TEST\r\n";

typedef enum
{
    PCBA_TEST_MOTOR_IDLE = 0,
    PCBA_TEST_MOTOR_FWD,
    PCBA_TEST_MOTOR_REV,
} pcba_test_motor_state_e;

static volatile u8 s_rx_line_ready = 0;
static volatile u8 s_rx_line_len   = 0;
static volatile u8 s_rx_line[PCBA_TEST_RX_LINE_MAX];
static volatile u8 s_rx_work_len = 0;
static volatile u8 s_rx_work[PCBA_TEST_RX_LINE_MAX];

static pcba_test_motor_state_e s_motor_state = PCBA_TEST_MOTOR_IDLE;
static u32                     s_key_tick    = 0;
static u8                      s_key_state   = 0;

static void pcba_uart_send_byte(u8 b)
{
    while (uart_tx_is_busy())
    {
    }
    uart_ndma_send_byte(b);
}

static void pcba_uart_send_str(const char *s)
{
    while (s && *s)
    {
        pcba_uart_send_byte((u8)*s++);
    }
}

static void pcba_uart_send_u32(u32 value)
{
    char buf[11];
    u8   i = 0;

    if (value == 0)
    {
        pcba_uart_send_byte('0');
        return;
    }

    while (value && i < sizeof(buf))
    {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i)
    {
        pcba_uart_send_byte((u8)buf[--i]);
    }
}

static void pcba_uart_send_line(const char *s)
{
    pcba_uart_send_str(s);
    pcba_uart_send_str("\r\n");
}

static void pcba_uart_send_error(const char *reason)
{
    pcba_uart_send_str("ERR ");
    pcba_uart_send_line(reason ? reason : "UNKNOWN");
}

static void pcba_aux_uart_init(void)
{
    gpio_set_func(PCBA_TEST_AUX_UART_TX_PIN, AS_GPIO);
    gpio_set_input_en(PCBA_TEST_AUX_UART_TX_PIN, 0);
    gpio_set_output_en(PCBA_TEST_AUX_UART_TX_PIN, 1);
    gpio_write(PCBA_TEST_AUX_UART_TX_PIN, 1);

    gpio_set_func(PCBA_TEST_AUX_UART_RX_PIN, AS_GPIO);
    gpio_set_input_en(PCBA_TEST_AUX_UART_RX_PIN, 1);
    gpio_set_output_en(PCBA_TEST_AUX_UART_RX_PIN, 0);
    gpio_setup_up_down_resistor(PCBA_TEST_AUX_UART_RX_PIN, PM_PIN_PULLUP_10K);
}

static void pcba_aux_uart_send_byte(u8 b)
{
    u8 i;

    gpio_write(PCBA_TEST_AUX_UART_TX_PIN, 0);
    sleep_us(PCBA_TEST_AUX_UART_BIT_US);

    for (i = 0; i < 8; i++)
    {
        gpio_write(PCBA_TEST_AUX_UART_TX_PIN, (b >> i) & 0x01);
        sleep_us(PCBA_TEST_AUX_UART_BIT_US);
    }

    gpio_write(PCBA_TEST_AUX_UART_TX_PIN, 1);
    sleep_us(PCBA_TEST_AUX_UART_BIT_US);
}

static void pcba_aux_uart_send_str(const char *s)
{
    while (s && *s)
    {
        pcba_aux_uart_send_byte((u8)*s++);
    }
}

static u8 pcba_aux_uart_recv_byte(u8 *out, u32 timeout_us)
{
    u8  i;
    u8  value = 0;
    u32 start_tick;

    if (!out)
    {
        return 0;
    }

    start_tick = clock_time();
    while (gpio_read(PCBA_TEST_AUX_UART_RX_PIN))
    {
        if (clock_time_exceed(start_tick, timeout_us))
        {
            return 0;
        }
    }

    sleep_us(PCBA_TEST_AUX_UART_BIT_US + (PCBA_TEST_AUX_UART_BIT_US / 2));
    for (i = 0; i < 8; i++)
    {
        if (gpio_read(PCBA_TEST_AUX_UART_RX_PIN))
        {
            value |= BIT(i);
        }
        sleep_us(PCBA_TEST_AUX_UART_BIT_US);
    }

    *out = value;
    sleep_us(PCBA_TEST_AUX_UART_BIT_US);
    return 1;
}

static u8 pcba_char_upper(u8 c)
{
    if (c >= 'a' && c <= 'z')
    {
        return (u8)(c - 'a' + 'A');
    }
    return c;
}

static u8 pcba_streq(const char *a, const char *b)
{
    while (*a && *b)
    {
        if (pcba_char_upper((u8)*a) != pcba_char_upper((u8)*b))
        {
            return 0;
        }
        a++;
        b++;
    }

    return (*a == '\0' && *b == '\0') ? 1 : 0;
}

static char *pcba_next_token(char **cursor)
{
    char *p;
    char *start;

    if (!cursor || !*cursor)
    {
        return 0;
    }

    p = *cursor;
    while (*p == ' ' || *p == '\t')
    {
        p++;
    }

    if (*p == '\0')
    {
        *cursor = p;
        return 0;
    }

    start = p;
    while (*p && *p != ' ' && *p != '\t')
    {
        p++;
    }

    if (*p)
    {
        *p++ = '\0';
    }

    *cursor = p;
    return start;
}

static u8 pcba_parse_onoff(const char *s, u8 *out)
{
    if (!s || !out)
    {
        return 0;
    }

    if (pcba_streq(s, "1") || pcba_streq(s, "ON"))
    {
        *out = 1;
        return 1;
    }

    if (pcba_streq(s, "0") || pcba_streq(s, "OFF"))
    {
        *out = 0;
        return 1;
    }

    return 0;
}

static void pcba_gpio_output_init(GPIO_PinTypeDef pin, u8 on)
{
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

static void pcba_set_output(GPIO_PinTypeDef pin, u8 on)
{
    gpio_write(pin, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

static void pcba_set_charge_limit(u8 on)
{
    gpio_write(Set_Charg_I, on ? 1 : 0);
}

static void pcba_all_test_outputs_off(void)
{
#if (UI_LED_ENABLE)
    pcba_set_output(GPIO_LED_BLUE, 0);
    pcba_set_output(GPIO_LED_GREEN, 0);
    pcba_set_output(GPIO_LED_RED, 0);
    pcba_set_output(GPIO_LED_WHITE, 0);
    pcba_set_output(GPIO_CHARGE_LED_GREEN, 0);
    pcba_set_output(GPIO_CHARGE_LED_RED, 0);
    gpio_write(V_BAT_CON, 0);
    gpio_write(V_NTC_CON, 0);
    gpio_write(LEIDA_SWITCH, 0);
    pcba_set_charge_limit(0);
#endif
}

static void pcba_gpio_init(void)
{
#if (UI_LED_ENABLE)
    pcba_gpio_output_init(GPIO_LED_BLUE, 0);
    pcba_gpio_output_init(GPIO_LED_GREEN, 0);
    pcba_gpio_output_init(GPIO_LED_RED, 0);
    pcba_gpio_output_init(GPIO_LED_WHITE, 0);
    pcba_gpio_output_init(GPIO_CHARGE_LED_GREEN, 0);
    pcba_gpio_output_init(GPIO_CHARGE_LED_RED, 0);
    pcba_gpio_output_init(V_BAT_CON, 1);
    pcba_gpio_output_init(V_NTC_CON, 1);
    pcba_gpio_output_init(LEIDA_SWITCH, 0);
    pcba_gpio_output_init(Set_Charg_I, 0);

    gpio_set_func(GPIO_KEY, AS_GPIO);
    gpio_set_input_en(GPIO_KEY, 1);
    gpio_set_output_en(GPIO_KEY, 0);
    gpio_setup_up_down_resistor(GPIO_KEY, PM_PIN_PULLUP_10K);
#endif
}

static u32 pcba_adc_sample_pin_mv(adc_input_pin_def_e pin)
{
    adc_base_init(pin);
    sleep_us(100);
    return adc_sample_and_get_result();
}

static void pcba_cmd_vbat(void)
{
    u32 pin_mv = pcba_adc_sample_pin_mv(ADC_GPIO_PB2);
    u32 bat_mv = (pin_mv * 66u + 5u) / 10u; /* Same divider compensation as app_adc_dbg.c. */

    pcba_uart_send_str("OK VBAT pin_mv=");
    pcba_uart_send_u32(pin_mv);
    pcba_uart_send_str(" bat_mv=");
    pcba_uart_send_u32(bat_mv);
    pcba_uart_send_str("\r\n");
}

static void pcba_cmd_ntc(void)
{
    u32 pin_mv = pcba_adc_sample_pin_mv(ADC_GPIO_PC4);

    pcba_uart_send_str("OK NTC pin_mv=");
    pcba_uart_send_u32(pin_mv);
    pcba_uart_send_str("\r\n");
}

static void pcba_cmd_uart_test(void)
{
    const char *p = s_aux_uart_test_pattern;
    u8          rx;
    u8          index = 0;

    pcba_aux_uart_send_str(s_aux_uart_test_pattern);

    while (*p)
    {
        if (!pcba_aux_uart_recv_byte(&rx, PCBA_TEST_AUX_UART_TIMEOUT_US))
        {
            pcba_uart_send_str("ERR UART_TIMEOUT index=");
            pcba_uart_send_u32(index);
            pcba_uart_send_str("\r\n");
            return;
        }

        if (rx != (u8)*p)
        {
            pcba_uart_send_str("ERR UART_MISMATCH index=");
            pcba_uart_send_u32(index);
            pcba_uart_send_str(" exp=");
            pcba_uart_send_u32((u8)*p);
            pcba_uart_send_str(" got=");
            pcba_uart_send_u32(rx);
            pcba_uart_send_str("\r\n");
            return;
        }

        p++;
        index++;
    }

    pcba_uart_send_line("OK UART TEST");
}

static void pcba_motor_start(pcba_test_motor_state_e state)
{
#if (UI_STEP_MOTOR_ENABLE)
    step_motor_dir_e dir = (state == PCBA_TEST_MOTOR_FWD) ? STEP_MOTOR_DIR_CW : STEP_MOTOR_DIR_CCW;

    if (s_motor_state != PCBA_TEST_MOTOR_IDLE)
    {
        pcba_uart_send_error("MOTOR_BUSY");
        return;
    }

    StepMotor_StopAll();
    StepMotor_MoveSteps(STEP_MOTOR_AXIS_PAN, dir, STEP_MOTOR_STEPS_PER_REV, PCBA_TEST_MOTOR_STEP_INTERVAL_US);
    StepMotor_MoveSteps(STEP_MOTOR_AXIS_TILT, dir, STEP_MOTOR_STEPS_PER_REV, PCBA_TEST_MOTOR_STEP_INTERVAL_US);
    s_motor_state = state;

    pcba_uart_send_line((state == PCBA_TEST_MOTOR_FWD) ? "OK MOTOR FWD START" : "OK MOTOR REV START");
#else
    (void)state;
    pcba_uart_send_error("MOTOR_DISABLED");
#endif
}

static void pcba_motor_task(void)
{
#if (UI_STEP_MOTOR_ENABLE)
    if (s_motor_state == PCBA_TEST_MOTOR_IDLE)
    {
        return;
    }

    StepMotor_Task();

    if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) && !StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
    {
        pcba_test_motor_state_e done = s_motor_state;
        s_motor_state                = PCBA_TEST_MOTOR_IDLE;
        StepMotor_StopAll();
        pcba_uart_send_line((done == PCBA_TEST_MOTOR_FWD) ? "OK MOTOR FWD DONE" : "OK MOTOR REV DONE");
    }
#endif
}

static u8 pcba_gpio_handle_one(const char *name, u8 on)
{
#if (UI_LED_ENABLE)
    if (pcba_streq(name, "LED_BLUE"))
    {
        pcba_set_output(GPIO_LED_BLUE, on);
    }
    else if (pcba_streq(name, "LED_GREEN"))
    {
        pcba_set_output(GPIO_LED_GREEN, on);
    }
    else if (pcba_streq(name, "LED_RED"))
    {
        pcba_set_output(GPIO_LED_RED, on);
    }
    else if (pcba_streq(name, "LED_ALL"))
    {
        pcba_set_output(GPIO_LED_BLUE, on);
        pcba_set_output(GPIO_LED_GREEN, on);
        pcba_set_output(GPIO_LED_RED, on);
    }
    else if (pcba_streq(name, "BAT_GREEN"))
    {
        pcba_set_output(GPIO_CHARGE_LED_GREEN, on);
    }
    else if (pcba_streq(name, "BAT_RED"))
    {
        pcba_set_output(GPIO_CHARGE_LED_RED, on);
    }
    else if (pcba_streq(name, "BAT_ALL"))
    {
        pcba_set_output(GPIO_CHARGE_LED_GREEN, on);
        pcba_set_output(GPIO_CHARGE_LED_RED, on);
    }
    else if (pcba_streq(name, "LASER"))
    {
        pcba_set_output(GPIO_LED_WHITE, on);
    }
    else if (pcba_streq(name, "RADAR"))
    {
        gpio_write(LEIDA_SWITCH, on ? 1 : 0);
    }
    else if (pcba_streq(name, "VBAT_EN"))
    {
        gpio_write(V_BAT_CON, on ? 1 : 0);
    }
    else if (pcba_streq(name, "NTC_EN"))
    {
        gpio_write(V_NTC_CON, on ? 1 : 0);
    }
    else if (pcba_streq(name, "CHARGE_LIMIT"))
    {
        pcba_set_charge_limit(on);
    }
    else if (pcba_streq(name, "ALL"))
    {
        if (on)
        {
            pcba_set_output(GPIO_LED_BLUE, 1);
            pcba_set_output(GPIO_LED_GREEN, 1);
            pcba_set_output(GPIO_LED_RED, 1);
            pcba_set_output(GPIO_LED_WHITE, 1);
            pcba_set_output(GPIO_CHARGE_LED_GREEN, 1);
            pcba_set_output(GPIO_CHARGE_LED_RED, 1);
            gpio_write(V_BAT_CON, 1);
            gpio_write(V_NTC_CON, 1);
            gpio_write(LEIDA_SWITCH, 1);
        }
        else
        {
            pcba_all_test_outputs_off();
        }
    }
    else
    {
        return 0;
    }

    return 1;
#else
    (void)name;
    (void)on;
    return 0;
#endif
}

static void pcba_cmd_gpio(char *cursor)
{
    char *name = pcba_next_token(&cursor);
    char *val  = pcba_next_token(&cursor);
    u8    on   = 0;

    if (!name || !pcba_parse_onoff(val, &on))
    {
        pcba_uart_send_error("GPIO_USAGE");
        return;
    }

    if (!pcba_gpio_handle_one(name, on))
    {
        pcba_uart_send_error("GPIO_NAME");
        return;
    }

    pcba_uart_send_str("OK GPIO ");
    pcba_uart_send_str(name);
    pcba_uart_send_byte(' ');
    pcba_uart_send_byte(on ? '1' : '0');
    pcba_uart_send_str("\r\n");
}

void app_pcba_test_enter_deep_sleep(void)
{
    pcba_all_test_outputs_off();
#if (UI_STEP_MOTOR_ENABLE)
    StepMotor_StopAll();
#endif

    cpu_set_gpio_wakeup(GPIO_KEY, Level_Low, 1);
    gpio_setup_up_down_resistor(GPIO_KEY, PM_PIN_PULLUP_10K);
    cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);
}

static void pcba_key_sleep_task(void)
{
    u8 key_down = gpio_read(GPIO_KEY) ? 0 : 1;

    switch (s_key_state)
    {
    case 0:
        if (key_down)
        {
            s_key_state = 1;
            s_key_tick  = clock_time();
        }
        break;

    case 1:
        if (!key_down)
        {
            s_key_state = 0;
        }
        else if (clock_time_exceed(s_key_tick, PCBA_TEST_KEY_DEBOUNCE_US))
        {
            s_key_state = 2;
        }
        break;

    case 2:
        if (!key_down)
        {
            s_key_state = 3;
            s_key_tick  = clock_time();
        }
        break;

    case 3:
        if (key_down)
        {
            s_key_state = 2;
        }
        else if (clock_time_exceed(s_key_tick, PCBA_TEST_KEY_DEBOUNCE_US))
        {
            pcba_uart_send_line("OK SLEEP KEY");
            app_pcba_test_enter_deep_sleep();
        }
        break;

    default:
        s_key_state = 0;
        break;
    }
}

static void pcba_dispatch_line(char *line)
{
    char *cursor = line;
    char *cmd    = pcba_next_token(&cursor);

    if (!cmd)
    {
        return;
    }

    if (pcba_streq(cmd, "PING"))
    {
        pcba_uart_send_line("OK PONG");
    }
    else if (pcba_streq(cmd, "VER?"))
    {
        pcba_uart_send_line("OK VER PCBA_TEST,1");
    }
    else if (pcba_streq(cmd, "VBAT?") || pcba_streq(cmd, "BAT?"))
    {
        pcba_cmd_vbat();
    }
    else if (pcba_streq(cmd, "NTC?"))
    {
        pcba_cmd_ntc();
    }
    else if (pcba_streq(cmd, "MOTOR"))
    {
        char *dir = pcba_next_token(&cursor);
        if (pcba_streq(dir ? dir : "", "FWD"))
        {
            pcba_motor_start(PCBA_TEST_MOTOR_FWD);
        }
        else if (pcba_streq(dir ? dir : "", "REV"))
        {
            pcba_motor_start(PCBA_TEST_MOTOR_REV);
        }
        else
        {
            pcba_uart_send_error("MOTOR_USAGE");
        }
    }
    else if (pcba_streq(cmd, "UART"))
    {
        char *op = pcba_next_token(&cursor);
        if (pcba_streq(op ? op : "", "TEST"))
        {
            pcba_cmd_uart_test();
        }
        else
        {
            pcba_uart_send_error("UART_USAGE");
        }
    }
    else if (pcba_streq(cmd, "GPIO"))
    {
        pcba_cmd_gpio(cursor);
    }
    else if (pcba_streq(cmd, "SLEEP"))
    {
        pcba_uart_send_line("OK SLEEP ENTER");
        app_pcba_test_enter_deep_sleep();
    }
    else if (pcba_streq(cmd, "HELP"))
    {
        pcba_uart_send_line("OK HELP PING VER? VBAT? NTC? MOTOR FWD|REV UART TEST GPIO <NAME> <0|1> SLEEP");
    }
    else
    {
        pcba_uart_send_error("UNKNOWN_CMD");
    }
}

void app_pcba_test_uart_irq_proc(void)
{
    if (!uart_ndmairq_get())
    {
        return;
    }

    while ((reg_uart_buf_cnt & 0x0f) != 0)
    {
        u8 b = uart_ndma_read_byte();

        if (b == '\r' || b == '\n')
        {
            if (s_rx_work_len && !s_rx_line_ready)
            {
                u8 len = s_rx_work_len;
                if (len >= PCBA_TEST_RX_LINE_MAX)
                {
                    len = PCBA_TEST_RX_LINE_MAX - 1;
                }

                memcpy((u8 *)s_rx_line, (u8 *)s_rx_work, len);
                s_rx_line[len] = '\0';
                s_rx_line_len  = len;
                s_rx_line_ready = 1;
            }
            s_rx_work_len = 0;
            continue;
        }

        if (s_rx_work_len < (PCBA_TEST_RX_LINE_MAX - 1))
        {
            s_rx_work[s_rx_work_len++] = b;
        }
        else
        {
            s_rx_work_len = 0;
            pcba_uart_send_error("RX_OVERFLOW");
        }
    }

    uart_clear_parity_error();
}

void app_pcba_test_init(void)
{
    pcba_gpio_init();
    pcba_aux_uart_init();

#if (UI_STEP_MOTOR_ENABLE)
    StepMotor_Init();
    StepMotor_StopAll();
#endif

    uart_gpio_set(PCBA_TEST_UART_TX_PIN, PCBA_TEST_UART_RX_PIN);
    uart_init_baudrate(PCBA_TEST_UART_BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_dma_enable(0, 0);
    uart_ndma_irq_triglevel(1, 0);
    uart_ndma_clear_rx_index();
    uart_ndma_clear_tx_index();
    uart_irq_enable(1, 0);

    pcba_uart_send_line("READY PCBA_TEST");
}

void app_pcba_test_task(void)
{
    if (s_rx_line_ready)
    {
        char line[PCBA_TEST_RX_LINE_MAX];
        u8   len = s_rx_line_len;

        if (len >= PCBA_TEST_RX_LINE_MAX)
        {
            len = PCBA_TEST_RX_LINE_MAX - 1;
        }

        memcpy((u8 *)line, (u8 *)s_rx_line, len);
        line[len]       = '\0';
        s_rx_line_ready = 0;

        pcba_dispatch_line(line);
    }

    pcba_motor_task();
    pcba_key_sleep_task();
}

#endif /* APP_PCBA_TEST_ENABLE */
