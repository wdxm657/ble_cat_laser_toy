#include "StepMotor.h"

#if (UI_STEP_MOTOR_ENABLE)

#define STEP_MOTOR_PHASE_COUNT      8
#define GIMBAL_DISPATCH_CHUNK_STEPS 16

typedef struct
{
    GPIO_PinTypeDef pin_a;
    GPIO_PinTypeDef pin_b;
    GPIO_PinTypeDef pin_c;
    GPIO_PinTypeDef pin_d;
} step_motor_pin_map_t;

typedef struct
{
    u8  run;
    u8  dir;
    u8  phase_idx;
    u8  _rsv;
    u32 next_tick;
    u32 interval_us;
    u32 remain_steps;
} step_motor_ctrl_t;

typedef struct
{
    s32 target_steps;
} gimbal_axis_ctrl_t;

/* half-step 8-beat sequence: A,AB,B,BC,C,CD,D,DA */
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
    MOTOR_RESET_PAN_SET_ZERO,
    MOTOR_RESET_DONE,
} step_motor_reset_state_e;

static step_motor_ctrl_t        g_motor_ctrl[STEP_MOTOR_AXIS_MAX]      = {0};
static gimbal_axis_ctrl_t       g_gimbal_ctrl[STEP_MOTOR_AXIS_MAX]     = {0};
static s32                      g_axis_pos_steps[STEP_MOTOR_AXIS_MAX]  = {0};
static s32                      g_axis_zero_steps[STEP_MOTOR_AXIS_MAX] = {0};
static u32                      g_gimbal_interval_us                   = 1200;
static u8                       g_auto_enabled                         = 1;
static step_motor_reset_state_e g_reset_state                          = MOTOR_RESET_IDLE;
static u8                       g_reset_busy                           = 0;

static inline u32 StepMotor_ClampIntervalUs(u32 interval_us)
{
    if (interval_us < STEP_MOTOR_MIN_INTERVAL_US_FASTEST)
    {
        return STEP_MOTOR_MIN_INTERVAL_US_FASTEST;
    }
    return interval_us;
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
    s32 num = (s32)deg_x10 * (s32)STEP_MOTOR_STEPS_PER_REV;
    s32 den = DEGx10(360);

    /* 四舍五入，降低量化误差导致的小范围抖动 */
    if (num >= 0)
    {
        return (num + (den / 2)) / den;
    }
    else
    {
        return (num - (den / 2)) / den;
    }
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
    const u8 *p = s_phase_table[phase & 0x07];

    StepMotor_WritePin(s_motor_pin[axis].pin_a, p[0]);
    StepMotor_WritePin(s_motor_pin[axis].pin_b, p[1]);
    StepMotor_WritePin(s_motor_pin[axis].pin_c, p[2]);
    StepMotor_WritePin(s_motor_pin[axis].pin_d, p[3]);
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

static inline u8 StepMotor_AxisReached(step_motor_axis_e axis, s32 target_deg10)
{
    s32 current_deg10 = StepMotor_GimbalGetCurrentDeg10(axis);
    LOG_D("[MOTOR] axis current/target: %d, %d", current_deg10, target_deg10);
    return (!StepMotor_IsRunning(axis) &&
            (current_deg10 == target_deg10));
}

static void StepMotor_GimbalSetTargetDeg10Internal(step_motor_axis_e axis, s32 target_deg_x10, u8 bypass_limit)
{
    s32 target_steps;
    s32 adjusted_target_deg_x10;

    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    adjusted_target_deg_x10          = bypass_limit ? target_deg_x10 : StepMotor_ClampDeg10(axis, target_deg_x10);
    target_steps                     = StepMotor_Deg10ToSteps(adjusted_target_deg_x10);
    g_gimbal_ctrl[axis].target_steps = g_axis_zero_steps[axis] + target_steps;
}

void StepMotor_Init(void)
{
    u8 i;

    g_gimbal_interval_us = 1200;

    for (i = 0; i < STEP_MOTOR_AXIS_MAX; i++)
    {
        StepMotor_ConfigOne((step_motor_axis_e)i);

        g_motor_ctrl[i].run          = 0;
        g_motor_ctrl[i].dir          = STEP_MOTOR_DIR_CW;
        g_motor_ctrl[i].phase_idx    = 0;
        g_motor_ctrl[i].next_tick    = 0;
        g_motor_ctrl[i].interval_us  = g_gimbal_interval_us;
        g_motor_ctrl[i].remain_steps = 0;

        g_axis_pos_steps[i]           = 0;
        g_axis_zero_steps[i]          = 0;
        g_gimbal_ctrl[i].target_steps = 0;

        StepMotor_Stop((step_motor_axis_e)i);
    }

    g_reset_state = MOTOR_RESET_IDLE;
    g_reset_busy  = 0;
}

void StepMotor_MoveSteps(step_motor_axis_e axis, step_motor_dir_e dir, u32 steps, u32 step_interval_us)
{
    if ((axis >= STEP_MOTOR_AXIS_MAX) || (steps == 0))
    {
        return;
    }

    g_motor_ctrl[axis].dir          = (u8)dir;
    g_motor_ctrl[axis].remain_steps = steps;
    g_motor_ctrl[axis].interval_us  = StepMotor_ClampIntervalUs((step_interval_us == 0) ? g_gimbal_interval_us : step_interval_us);
    g_motor_ctrl[axis].next_tick    = clock_time();
    g_motor_ctrl[axis].run          = 1;
}

void StepMotor_Stop(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    StepMotor_WritePin(s_motor_pin[axis].pin_a, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_b, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_c, 0);
    StepMotor_WritePin(s_motor_pin[axis].pin_d, 0);

    g_motor_ctrl[axis].run          = 0;
    g_motor_ctrl[axis].remain_steps = 0;
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
    return (g_auto_enabled);
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

    return g_motor_ctrl[axis].run;
}

void StepMotor_Task(void)
{
    u8 i;

    for (i = 0; i < STEP_MOTOR_AXIS_MAX; i++)
    {
        step_motor_ctrl_t *ctrl = &g_motor_ctrl[i];

        if (!ctrl->run)
        {
            continue;
        }

        if (!clock_time_exceed(ctrl->next_tick, ctrl->interval_us))
        {
            continue;
        }

        ctrl->next_tick = clock_time();

        if (ctrl->dir == STEP_MOTOR_DIR_CW)
        {
            ctrl->phase_idx = (ctrl->phase_idx + 1) & 0x07;
            g_axis_pos_steps[i]++;
        }
        else
        {
            ctrl->phase_idx = (ctrl->phase_idx + 7) & 0x07;
            g_axis_pos_steps[i]--;
        }

        StepMotor_WritePhase((step_motor_axis_e)i, ctrl->phase_idx);

        if (ctrl->remain_steps > 0)
        {
            ctrl->remain_steps--;
            if (ctrl->remain_steps == 0)
            {
                g_motor_ctrl[i].run          = 0;
                g_motor_ctrl[i].remain_steps = 0;
            }
        }
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
    s32 cur_deg10;

    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    cur_deg10 = StepMotor_StepsToDeg10(g_gimbal_ctrl[axis].target_steps);
    StepMotor_GimbalSetTargetDeg10(axis, cur_deg10 + delta_deg_x10);
}

s32 StepMotor_GimbalGetCurrentDeg10(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return 0;
    }

    return StepMotor_StepsToDeg10(g_axis_pos_steps[axis] - g_axis_zero_steps[axis]);
}

void StepMotor_GimbalSetZero(step_motor_axis_e axis)
{
    if (axis >= STEP_MOTOR_AXIS_MAX)
    {
        return;
    }

    g_axis_zero_steps[axis]          = g_axis_pos_steps[axis];
    g_gimbal_ctrl[axis].target_steps = g_axis_pos_steps[axis];
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
    u8 i;

    StepMotor_Task();

    for (i = 0; i < STEP_MOTOR_AXIS_MAX; i++)
    {
        s32 delta_steps;
        u32 dispatch_steps;

        if (StepMotor_IsRunning((step_motor_axis_e)i))
        {
            continue;
        }

        delta_steps = g_gimbal_ctrl[i].target_steps - g_axis_pos_steps[i];
        if (delta_steps == 0)
        {
            continue;
        }

        if (delta_steps > 0)
        {
            dispatch_steps = (u32)((delta_steps > GIMBAL_DISPATCH_CHUNK_STEPS) ? GIMBAL_DISPATCH_CHUNK_STEPS : delta_steps);
            StepMotor_MoveSteps((step_motor_axis_e)i, STEP_MOTOR_DIR_CW, dispatch_steps, g_gimbal_interval_us);
        }
        else
        {
            s32 abs_steps  = -delta_steps;
            dispatch_steps = (u32)((abs_steps > GIMBAL_DISPATCH_CHUNK_STEPS) ? GIMBAL_DISPATCH_CHUNK_STEPS : abs_steps);
            StepMotor_MoveSteps((step_motor_axis_e)i, STEP_MOTOR_DIR_CCW, dispatch_steps, g_gimbal_interval_us);
        }
    }
}

void StepMotor_GimbalResetStart(void)
{
    StepMotor_GimbalSetSpeedUs(1200);
    g_reset_busy  = 1;
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
        LOG_D("[MOTOR] reset tilt up to -105");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_TILT, DEGx10(-100), 1);
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
        LOG_D("[MOTOR] reset pan right to -100");
        StepMotor_GimbalSetTargetDeg10Internal(STEP_MOTOR_AXIS_PAN, DEGx10(-90), 1);
        g_reset_state = MOTOR_RESET_WAIT_PAN_RIGHT;
        break;

    case MOTOR_RESET_WAIT_PAN_RIGHT:
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
        g_reset_busy  = 0;
        g_reset_state = MOTOR_RESET_IDLE;
        StepMotor_StopAll();
        LOG_D("[MOTOR] reset done");
        break;

    default:
        g_reset_busy  = 0;
        g_reset_state = MOTOR_RESET_IDLE;
        break;
    }
}

void StepMotor_GimbalDemoTask(void)
{
    enum
    {
        DEMO_RAMP_UP = 0,
        DEMO_RAMP_DOWN,
        DEMO_WAIT_RESTART,
    };

    /* 每 0.1s 调整 1°（单位：deg10） */
#define DEMO_STEP_DEG10 100
#define DEMO_UPDATE_US  500000
#define DEMO_WAIT_US    0
#define DEMO_MIN_DEG10  DEGx10(-50)
#define DEMO_MAX_DEG10  DEGx10(50)

    static u8  s_state        = DEMO_RAMP_UP;
    static u32 s_update_tick  = 0;
    static u32 s_wait_tick    = 0;
    static s16 s_target_deg10 = DEMO_MIN_DEG10;

    StepMotor_GimbalTask();

    switch (s_state)
    {
    case DEMO_RAMP_UP:
        if (clock_time_exceed(s_update_tick, DEMO_UPDATE_US))
        {
            s_update_tick = clock_time();
            // if (StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) || StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
            // {
            //     break;
            // }

            if (s_target_deg10 < DEMO_MAX_DEG10)
            {
                s_target_deg10 += DEMO_STEP_DEG10;
                if (s_target_deg10 > DEMO_MAX_DEG10)
                {
                    s_target_deg10 = DEMO_MAX_DEG10;
                }

                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, s_target_deg10);
                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, s_target_deg10);
                // LOG_D("PAN,%d,TILT,%d", s_target_deg10, s_target_deg10);
            }
            else
            {
                s_state = DEMO_RAMP_DOWN;
            }
        }
        break;

    case DEMO_RAMP_DOWN:
        if (clock_time_exceed(s_update_tick, DEMO_UPDATE_US))
        {
            s_update_tick = clock_time();
            // if (StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) || StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
            // {
            //     break;
            // }

            if (s_target_deg10 > DEMO_MIN_DEG10)
            {
                s_target_deg10 -= DEMO_STEP_DEG10;
                if (s_target_deg10 < DEMO_MIN_DEG10)
                {
                    s_target_deg10 = DEMO_MIN_DEG10;
                }

                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, s_target_deg10);
                StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, s_target_deg10);
                // LOG_D("PAN,%d,TILT,%d", s_target_deg10, s_target_deg10);
            }
            else
            {
                s_wait_tick = clock_time();
                s_state     = DEMO_RAMP_UP;
            }
        }
        break;

    case DEMO_WAIT_RESTART:
        // if (clock_time_exceed(s_wait_tick, DEMO_WAIT_US) &&
        //     !StepMotor_IsRunning(STEP_MOTOR_AXIS_PAN) &&
        //     !StepMotor_IsRunning(STEP_MOTOR_AXIS_TILT))
        // {
        // s_target_deg10 = DEMO_MIN_DEG10;
        // StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_PAN, s_target_deg10);
        // StepMotor_GimbalSetTargetDeg10(STEP_MOTOR_AXIS_TILT, s_target_deg10);
        s_update_tick = clock_time();
        s_state       = DEMO_RAMP_UP;
        // }
        break;

    default:
        s_target_deg10 = DEMO_MIN_DEG10;
        s_update_tick  = clock_time();
        s_wait_tick    = s_update_tick;
        s_state        = DEMO_RAMP_UP;
        break;
    }

#undef DEMO_STEP_DEG10
#undef DEMO_UPDATE_US
#undef DEMO_WAIT_US
#undef DEMO_MIN_DEG10
#undef DEMO_MAX_DEG10
}

#endif
