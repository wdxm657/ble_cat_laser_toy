#pragma once

#include "tl_common.h"
#include "drivers.h"

#ifndef UI_STEP_MOTOR_ENABLE
#define UI_STEP_MOTOR_ENABLE 0
#endif

#if (UI_STEP_MOTOR_ENABLE)

#ifndef GPIO_STEP_MOTOR_1_A
#define GPIO_STEP_MOTOR_1_A GPIO_PC7
#endif
#ifndef GPIO_STEP_MOTOR_1_B
#define GPIO_STEP_MOTOR_1_B GPIO_PC6
#endif
#ifndef GPIO_STEP_MOTOR_1_C
#define GPIO_STEP_MOTOR_1_C GPIO_PC5
#endif
#ifndef GPIO_STEP_MOTOR_1_D
#define GPIO_STEP_MOTOR_1_D GPIO_PC4
#endif

#ifndef GPIO_STEP_MOTOR_2_A
#define GPIO_STEP_MOTOR_2_A GPIO_PC3
#endif
#ifndef GPIO_STEP_MOTOR_2_B
#define GPIO_STEP_MOTOR_2_B GPIO_PC2
#endif
#ifndef GPIO_STEP_MOTOR_2_C
#define GPIO_STEP_MOTOR_2_C GPIO_PC1
#endif
#ifndef GPIO_STEP_MOTOR_2_D
#define GPIO_STEP_MOTOR_2_D GPIO_PC0
#endif

#ifndef STEP_MOTOR_PIN_ACTIVE_LEVEL
#define STEP_MOTOR_PIN_ACTIVE_LEVEL 1
#endif

#define DEGx10(_deg)                       (_deg * 10)
#define STEP_MOTOR_STEPS_PER_REV           1616u
#define STEP_MOTOR_MIN_INTERVAL_US_FASTEST 750u

/* angle precision: 0.1 degree (deg10) */
#define GIMBAL_PAN_LIMIT_DEG10_POS         DEGx10(80)
#define GIMBAL_PAN_LIMIT_DEG10_NEG         DEGx10(-80)
#define GIMBAL_TILT_LIMIT_DEG10_POS        DEGx10(0)
#define GIMBAL_TILT_LIMIT_DEG10_NEG        DEGx10(-75)

typedef enum
{
    STEP_MOTOR_AXIS_PAN  = 0,
    STEP_MOTOR_AXIS_TILT = 1,
    STEP_MOTOR_AXIS_MAX,
} step_motor_axis_e;

typedef enum
{
    STEP_MOTOR_DIR_CW  = 1,
    STEP_MOTOR_DIR_CCW = 0,
} step_motor_dir_e;

void StepMotor_Init(void);
void StepMotor_MoveSteps(step_motor_axis_e axis, step_motor_dir_e dir, u32 steps, u32 step_interval_us);
void StepMotor_Stop(step_motor_axis_e axis);
void StepMotor_StopAll(void);
u8   StepMotor_IsRunning(step_motor_axis_e axis);
void StepMotor_Task(void);

void StepMotor_CtrlSetAutoEnabled(u8 enable);
u8   StepMotor_CtrlCanEnterAuto(void);
u8   StepMotor_CtrlIsAutoEnabled(void);

/*********************** Gmbal control layer ************************/
void StepMotor_GimbalSetSpeedUs(u32 step_interval_us);
void StepMotor_GimbalSetTargetDeg10(step_motor_axis_e axis, s32 target_deg_x10);
void StepMotor_GimbalMoveDeltaDeg10(step_motor_axis_e axis, s32 delta_deg_x10);
s32  StepMotor_GimbalGetCurrentDeg10(step_motor_axis_e axis);
void StepMotor_GimbalSetZero(step_motor_axis_e axis);
void StepMotor_GimbalSetZeroAll(void);
void StepMotor_GimbalHome(void);
void StepMotor_GimbalTask(void);
void StepMotor_GimbalDemoTask(void);

/* reset flow: tilt down 75 -> up 75, pan left 80 -> right 80, then set zero */
void StepMotor_GimbalResetStart(void);
void StepMotor_GimbalResetTask(void);
u8   StepMotor_GimbalResetBusy(void);

#endif
