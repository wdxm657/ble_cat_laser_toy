#include "StepMotor.h"
#include "app_config.h"

#if (UI_STEP_MOTOR_ENABLE)

#define STEP_MOTOR_PHASE_COUNT 8
#define STEP_MOTOR_S32_MAX     2147483647L
#define STEP_MOTOR_S32_MIN     (-2147483647L - 1L)

typedef struct
{
    GPIO_PinTypeDef pin_a;
    GPIO_PinTypeDef pin_b;
    GPIO_PinTypeDef pin_c;
    GPIO_PinTypeDef pin_d;
} step_motor_pin_map_t;

typedef struct
{
    u8  phase_idx;
    u32 next_step_tick;
    u32 target_interval_us;
    s32 position_steps;
    s32 zero_steps;
    s32 target_steps;
} step_motor_axis_state_t;

typedef enum
{
    MOTOR_RESET_IDLE = 0,
    MOTOR_RESET_TILT_DOWN,
    MOTOR_RESET_WAIT_TILT_DOWN,
    MOTOR_RESET_TILT_UP,
    MOTOR_RESET_WAIT_TILT_UP,
    MOTOR_RESET_TILT_SET_ZERO,
    MOTOR_RESET_PAN_LEFT,
    MOTOR_RESET_WAIT_PAN_LEFT,
    MOTOR_RESET_PAN_RIGHT,
    MOTOR_RESET_WAIT_PAN_RIGHT,
    MOTOR_RESET_PAN_RIGHT_1,
    MOTOR_RESET_WAIT_PAN_RIGHT_1,
    MOTOR_RESET_PAN_LEFT_1,
    MOTOR_RESET_WAIT_PAN_LEFT_1,
    MOTOR_RESET_PAN_SET_ZERO,
    MOTOR_RESET_DONE,
} step_motor_reset_state_e;

/* Half-step sequence: A, AB, B, BC, C, CD, D, DA. */
static const u8 s_phase_table[STEP_MOTOR_PHASE_COUNT][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

static const step_motor_pin_map_t s_motor_pin[STEP_MOTOR_AXIS_MAX] = {
    {GPIO_STEP_MOTOR_1_A, GPIO_STEP_MOTOR_1_B, GPIO_STEP_MOTOR_1_C, GPIO_STEP_MOTOR_1_D},
    {GPIO_STEP_MOTOR_2_A, GPIO_STEP_MOTOR_2_B, GPIO_STEP_MOTOR_2_C, GPIO_STEP_MOTOR_2_D},
};

static step_motor_axis_state_t g_axis[STEP_MOTOR_AXIS_MAX] = {0};
static u32                      g_gimbal_interval_us = 1200;
static u8                       g_auto_enabled = 1;
static step_motor_reset_state_e g_reset_state = MOTOR_RESET_IDLE;
static u8                       g_reset_busy = 0;

static inline u32 StepMotor_ClampIntervalUs(u32 interval_us)
{
    return (interval_us < STEP_MOTOR_MIN_INTERVAL_US_FASTEST) ?
           STEP_MOTOR_MIN_INTERVAL_US_FASTEST : interval_us;
}

static inline s32 StepMotor_ClampDeg10(step_motor_axis_e axis, s32 deg_x10)
{
    if (axis == STEP_MOTOR_AXIS_PAN)
    {
        if (deg_x10 > GIMBAL_PAN_LIMIT_DEG10_POS)
        {
            return GIMBAL_PAN_LIMIT_DEG10_POS;
        }
        if (deg_x10 < GIMBAL_PAN_LIMIT_DEG10_NEG)
        {
            return GIMBAL_PAN_LIMIT_DEG10_NEG;
        }
        return deg_x10;
    }

    if (deg_x10 > GIMBAL_TILT_LIMIT_DEG10_POS)
    {
        return GIMBAL_TILT_LIMIT_DEG10_POS;
    }
    if (deg_x10 < GIMBAL_TILT_LIMIT_DEG10_NEG)
    {
        return GIMBAL_TILT_LIMIT_DEG10_NEG;
    }
    return deg_x10;
}

static inline s32 StepMotor_Deg10ToSteps(s32 deg_x10)
{
    s32 numerator = deg_x10 * (s32)STEP_MOTOR_STEPS_PER_REV;
    s32 denominator = DEGx10(360);

    if (numerator >= 0)
    {
        return (numerator + denominator / 2) / denominator;
    }
    return (numerator - denominator / 2) / denominator;
}

static inline s32 StepMotor_StepsToDeg10(s32 steps)
{
    return (s32)(((s32)steps * DEGx10(360)) / (s32)STEP_MOTOR_STEPS_PER_REV);
}

static inline void StepMotor_WritePin(GPIO_PinTypeDef pin, u8 on)
{
#if (STEP_MOTOR_PIN_ACTIVE_LEVEL)
    gpio_write(pin, on);
#else
    gpio_write(pin, !on);
#endif
}

static void StepMotor_WritePhase(step_motor_axis_e axis, u8 phase)
{
    const u8 *pattern = s_phase_table[phase & 0x07];

    StepMotor_WritePin(s_motor_pin[axis].pin_a, pattern[0]);
    StepMotor_WritePin(s_motor_pin[axis].pin_b, pattern[1]);
    StepMotor_WritePin(s_motor_pin[axis].pin_c, pattern[2]);
    StepMotor_WritePin(s_motor_pin[axis].pin_d, pattern[3]);
}

static void StepMotor_ConfigOne(step_motor_axis_e axis)
{
    gpio_set_func(s_motor_pin[axis].pin_a, AS_GPIO);
    gpio_set_func(s_motor_pin[axis].pin_b, AS_GPIO);
    gpio_set_func(s_motor_pin[axis].pin_c, AS_GPIO);
    gpio_set_func(s_motor_pin[axis].pin_d, AS_GPIO);

    gpio_set_output_en(s_motor_pin[axis].pin_a, 1);
    gpio_set_output_en(s_motor_pin[axis].pin_b, 1);
    gpio_set_output_en(s_motor_pin[axis].pin_c, 1);
    gpio_set_output_en(s_motor_pin[axis].pin_d, 1);

    gpio_set_input_en(s_motor_pin[axis].pin_a, 0);
    gpio_set_input_en(s_motor_pin[axis].pin_b, 0);
    gpio_set_input_en(s_motor_pin[axis].pin_c, 0);
    gpio_set_input_en(s_motor_pin[axis].pin_d, 0);
}

static void StepMotor_GimbalSetTargetDeg10Internal(step_motor_axis_e axis,
                                                    s32 target_deg_x10,
                                                    u8 bypass_limit)
{
    step_motor_axis_state_t *state;
    s32 adjusted_target_deg_x10;
    s32 target_steps;

    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    state = &g_axis[axis];
    adjusted_target_deg_x10 = bypass_limit ? target_deg_x10 :
                              StepMotor_ClampDeg10(axis, target_deg_x10);
    target_steps = state->zero_steps + StepMotor_Deg10ToSteps(adjusted_target_deg_x10);
    state->target_steps = target_steps;
    state->target_interval_us = g_gimbal_interval_us;
}

void StepMotor_Init(void)
{
    u8 axis;
    u32 now = clock_time();

    g_gimbal_interval_us = 1200;
    for (axis = 0; axis < STEP_MOTOR_AXIS_MAX; axis++)
    {
        StepMotor_ConfigOne((step_motor_axis_e)axis);
        g_axis[axis].phase_idx = 0;
        g_axis[axis].next_step_tick = now;
        g_axis[axis].target_interval_us = g_gimbal_interval_us;
        g_axis[axis].position_steps = 0;
        g_axis[axis].zero_steps = 0;
        g_axis[axis].target_steps = 0;
        StepMotor_Stop((step_motor_axis_e)axis);
    }

    g_reset_state = MOTOR_RESET_IDLE;
    g_reset_busy = 0;
}

void StepMotor_MoveSteps(step_motor_axis_e axis,
                         step_motor_dir_e dir,
                         u32 steps,
                         u32 step_interval_us)
{
    step_motor_axis_state_t *state;
    s32 signed_steps;
    s32 target_steps;

    if ((axis >= STEP_MOTOR_AXIS_MAX) || (steps == 0) || (steps > (u32)STEP_MOTOR_S32_MAX))
    {
        return;
    }

    state = &g_axis[axis];
    signed_steps = (s32)steps;
    if (dir == STEP_MOTOR_DIR_CCW)
    {
        signed_steps = -signed_steps;
    }

    if (((signed_steps > 0) &&
         (state->position_steps > (s32)(STEP_MOTOR_S32_MAX - signed_steps))) ||
        ((signed_steps < 0) &&
         (state->position_steps < (s32)(STEP_MOTOR_S32_MIN - signed_steps))))
    {
        return;
    }

    target_steps = state->position_steps + signed_steps;
    state->target_steps = target_steps;
    state->target_interval_us = StepMotor_ClampIntervalUs((step_interval_us == 0) ?
                                                          g_gimbal_interval_us : step_interval_us);
}

void StepMotor_Stop(step_motor_axis_e axis)
{
    step_motor_axis_state_t *state;

    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    state = &g_axis[axis];
    StepMotor_WritePin(s_motor_pin[axis].pin_a, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_b, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_c, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_d, 0);
    state->target_steps = state->position_steps;
    state->next_step_tick = clock_time();
}

void StepMotor_StopAll(void)
{
    StepMotor_Stop(STEP_MOTOR_AXIS_PAN);
    StepMotor_Stop(STEP_MOTOR_AXIS_TILT);
}

void StepMotor_CtrlSetAutoEnabled(u8 enable)
{
    g_auto_enabled = enable ? 1 : 0;
}

u8 StepMotor_CtrlCanEnterAuto(void)
{
    return g_auto_enabled;
}

u8 StepMotor_CtrlIsAutoEnabled(void)
{
    return g_auto_enabled;
}

u8 StepMotor_IsRunning(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return 0;
    }
    return (g_axis[axis].position_steps != g_axis[axis].target_steps);
}

void StepMotor_Task(void)
{
    u8 axis;
    u32 now = clock_time();

    for (axis = 0; axis < STEP_MOTOR_AXIS_MAX; axis++)
    {
        step_motor_axis_state_t *state = &g_axis[axis];
        s32 delta_steps = state->target_steps - state->position_steps;

        if (delta_steps == 0)
        {
            continue;
        }

        if (!clock_time_exceed(state->next_step_tick, state->target_interval_us))
        {
            continue;
        }

        /* One phase transition per axis per task pass; never build a catch-up burst. */
        state->next_step_tick = now;
        if (delta_steps > 0)
        {
            state->phase_idx = (state->phase_idx + 1) & 0x07;
            state->position_steps++;
        }
        else
        {
            state->phase_idx = (state->phase_idx + 7) & 0x07;
            state->position_steps--;
        }
        StepMotor_WritePhase((step_motor_axis_e)axis, state->phase_idx);
    }
}

void StepMotor_GimbalSetSpeedUs(u32 step_interval_us)
{
    g_gimbal_interval_us = StepMotor_ClampIntervalUs(step_interval_us);
}

void StepMotor_GimbalSetTargetDeg10(step_motor_axis_e axis, s32 target_deg_x10)
{
    StepMotor_GimbalSetTargetDeg10Internal(axis, target_deg_x10, 0);
}

void StepMotor_GimbalMoveDeltaDeg10(step_motor_axis_e axis, s32 delta_deg_x10)
{
    s32 target_deg10;

    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    target_deg10 = StepMotor_StepsToDeg10(g_axis[axis].target_steps -
                                           g_axis[axis].zero_steps);
    StepMotor_GimbalSetTargetDeg10(axis, target_deg10 + delta_deg_x10);
}

s32 StepMotor_GimbalGetCurrentDeg10(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return 0;
    }
    return StepMotor_StepsToDeg10(g_axis[axis].position_steps - g_axis[axis].zero_steps);
}

void StepMotor_GimbalSetZero(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    g_axis[axis].zero_steps = g_axis[axis].position_steps;
    g_axis[axis].target_steps = g_axis[axis].position_steps;
}

void StepMotor_GimbalSetZeroAll(void)
{
    StepMotor_GimbalSetZero(STEP_MOTOR_AXIS_PAN);
    StepMotor_GimbalSetZero(STEP_MOTOR_AXIS_TILT);
}

void StepMotor_GimbalHome(void)
{
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, 0);
    StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, 0);
}

void StepMotor_GimbalTask(void)
{
    StepMotor_Task();
}

void StepMotor_GimbalResetStart(void)
{
    StepMotor_GimbalSetSpeedUs(800);
    g_reset_busy = 1;
    g_reset_state = MOTOR_RESET_TILT_DOWN;
    LOG_D("[MOTOR] reset start");
}

u8 StepMotor_GimbalResetBusy(void)
{
    return g_reset_busy;
}

void StepMotor_GimbalResetTask(void)
{
    if (!g_reset_busy)
    {
        return;
    }

    StepMotor_GimbalTask();

    switch (g_reset_state)
    {
    case MOTOR_RESET_TILT_DOWN:
        LOG_D("[MOTOR] reset tilt down to -180");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_TILT, DEGx10(-180), 1);
        g_reset_state = MOTOR_RESET_WAIT_TILT_DOWN;
        break;

    case MOTOR_RESET_WAIT_TILT_DOWN:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
        {
            g_reset_state = MOTOR_RESET_TILT_UP;
        }
        break;

    case MOTOR_RESET_TILT_UP:
        LOG_D("[MOTOR] reset tilt up to -102");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_TILT, DEGx10(-102), 1);
        g_reset_state = MOTOR_RESET_WAIT_TILT_UP;
        break;

    case MOTOR_RESET_WAIT_TILT_UP:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
        {
            g_reset_state = MOTOR_RESET_TILT_SET_ZERO;
        }
        break;

    case MOTOR_RESET_TILT_SET_ZERO:
        StepMotor_GimbalSetZero(STEP_MOTOR_AXIS_TILT);
        g_reset_state = MOTOR_RESET_PAN_LEFT;
        LOG_D("[MOTOR] reset tilt zero set");
        break;

    case MOTOR_RESET_PAN_LEFT:
        LOG_D("[MOTOR] reset pan left to -180");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_PAN, DEGx10(-180), 1);
        g_reset_state = MOTOR_RESET_WAIT_PAN_LEFT;
        break;

    case MOTOR_RESET_WAIT_PAN_LEFT:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN))
        {
            g_reset_state = MOTOR_RESET_PAN_RIGHT;
        }
        break;

    case MOTOR_RESET_PAN_RIGHT:
        LOG_D("[MOTOR] reset pan right to -95");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_PAN, DEGx10(-95), 1);
        g_reset_state = MOTOR_RESET_WAIT_PAN_RIGHT;
        break;

    case MOTOR_RESET_WAIT_PAN_RIGHT:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN))
        {
            g_reset_state = app_factory_test_is_active() ?
                            MOTOR_RESET_PAN_RIGHT_1 : MOTOR_RESET_PAN_SET_ZERO;
        }
        break;

    case MOTOR_RESET_PAN_RIGHT_1:
        LOG_D("[MOTOR] reset pan right to 0");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_PAN, 0, 1);
        g_reset_state = MOTOR_RESET_WAIT_PAN_RIGHT_1;
        break;

    case MOTOR_RESET_WAIT_PAN_RIGHT_1:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN))
        {
            g_reset_state = MOTOR_RESET_PAN_LEFT_1;
        }
        break;

    case MOTOR_RESET_PAN_LEFT_1:
        LOG_D("[MOTOR] reset pan left to -97");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_PAN, DEGx10(-97), 1);
        g_reset_state = MOTOR_RESET_WAIT_PAN_LEFT_1;
        break;

    case MOTOR_RESET_WAIT_PAN_LEFT_1:
        if (!StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN))
        {
            g_reset_state = MOTOR_RESET_PAN_SET_ZERO;
        }
        break;

    case MOTOR_RESET_PAN_SET_ZERO:
        StepMotor_GimbalSetZero(STEP_MOTOR_AXIS_PAN);
        g_reset_state = MOTOR_RESET_DONE;
        LOG_D("[MOTOR] reset pan zero set");
        break;

    case MOTOR_RESET_DONE:
        g_reset_busy = 0;
        g_reset_state = MOTOR_RESET_IDLE;
        StepMotor_StopAll();
        LOG_D("[MOTOR] reset done");
        break;

    default:
        g_reset_busy = 0;
        g_reset_state = MOTOR_RESET_IDLE;
        break;
    }
}

void StepMotor_GimbalDemoTask(void)
{
#define DEMO_STEP_DEG10 100
#define DEMO_UPDATE_US  500000u
#define DEMO_MIN_DEG10 DEGx10(-50)
#define DEMO_MAX_DEG10 DEGx10(50)

    static u8  s_ramp_down = 0;
    static u8  s_tick_init = 0;
    static u32 s_update_tick = 0;
    static s16 s_target_deg10 = DEMO_MIN_DEG10;

    StepMotor_GimbalTask();

    if (!s_tick_init)
    {
        s_tick_init = 1;
        s_update_tick = clock_time();
    }
    else if (clock_time_exceed(s_update_tick, DEMO_UPDATE_US))
    {
        s_update_tick = clock_time();
        if (!s_ramp_down)
        {
            if (s_target_deg10 < DEMO_MAX_DEG10)
            {
                s_target_deg10 += DEMO_STEP_DEG10;
                if (s_target_deg10 > DEMO_MAX_DEG10)
                {
                    s_target_deg10 = DEMO_MAX_DEG10;
                }
                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, s_target_deg10);
                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, s_target_deg10);
            }
            else
            {
                s_ramp_down = 1;
            }
        }
        else if (s_target_deg10 > DEMO_MIN_DEG10)
        {
            s_target_deg10 -= DEMO_STEP_DEG10;
            if (s_target_deg10 < DEMO_MIN_DEG10)
            {
                s_target_deg10 = DEMO_MIN_DEG10;
            }
            StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, s_target_deg10);
            StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, s_target_deg10);
        }
        else
        {
            s_ramp_down = 0;
        }
    }

#undef DEMO_STEP_DEG10
#undef DEMO_UPDATE_US
#undef DEMO_MIN_DEG10
#undef DEMO_MAX_DEG10
}

#endif
