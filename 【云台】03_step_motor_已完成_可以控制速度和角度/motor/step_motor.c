#include "step_motor.h"

#include <limits.h>

#define TIMER_MAX_PERIOD                   (65535U)
#define TIMER_MIN_PERIOD                   (800U)

static volatile uint32_t step_remain_1;
static volatile uint32_t step_remain_2;
static volatile bool step_running_1;
static volatile bool step_running_2;

static bool StepMotor_IsValidId(uint8_t stepper_id)
{
    return (stepper_id == STEP_MOTOR_1) ||
           (stepper_id == STEP_MOTOR_2);
}

static bool StepMotor_StartSteps(uint32_t steps, uint8_t stepper_id)
{
    if (!StepMotor_IsValidId(stepper_id) || (steps == 0U) ||
        step_motor_is_busy(stepper_id)) {
        return false;
    }

    if (stepper_id == STEP_MOTOR_1) {
        step_remain_1 = steps;
    } else {
        step_remain_2 = steps;
    }

    step_motor_start(stepper_id);
    return true;
}

void step_motor_init(void)
{
    /* Release reset/sleep and select the high-torque decay mode. */
    DL_GPIO_setPins(STEP_MOTOR1_A_PORT,
        STEP_MOTOR1_A_RST1_PIN | STEP_MOTOR1_A_DIR1_PIN);
    DL_GPIO_setPins(STEP_MOTOR1_B_PORT,
        STEP_MOTOR1_B_SLP1_PIN | STEP_MOTOR1_B_DCY1_PIN);

    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_RST2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_SLP2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DCY2_PIN);

    step_remain_1 = 0U;
    step_remain_2 = 0U;
    step_running_1 = false;
    step_running_2 = false;

    NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
    NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);
}

void step_motor_dir_set(uint8_t direction, uint8_t stepper_id)
{
    if (stepper_id == STEP_MOTOR_1) {
        if (direction == 0U) {
            DL_GPIO_clearPins(STEP_MOTOR1_A_PORT, STEP_MOTOR1_A_DIR1_PIN);
        } else {
            DL_GPIO_setPins(STEP_MOTOR1_A_PORT, STEP_MOTOR1_A_DIR1_PIN);
        }
    } else if (stepper_id == STEP_MOTOR_2) {
        if (direction == 0U) {
            DL_GPIO_clearPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        } else {
            DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        }
    }
}

void step_motor_start(uint8_t stepper_id)
{
    if (stepper_id == STEP_MOTOR_1) {
        step_running_1 = true;
        NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM1_INST);
    } else if (stepper_id == STEP_MOTOR_2) {
        step_running_2 = true;
        NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM2_INST);
    }
}

void step_motor_stop(uint8_t stepper_id)
{
    if (stepper_id == STEP_MOTOR_1) {
        DL_Timer_stopCounter(DCC_100_PWM1_INST);
        step_remain_1 = 0U;
        step_running_1 = false;
    } else if (stepper_id == STEP_MOTOR_2) {
        DL_Timer_stopCounter(DCC_100_PWM2_INST);
        step_remain_2 = 0U;
        step_running_2 = false;
    }
}

void step_set_speed(uint8_t speed, uint8_t stepper_id)
{
    uint32_t frequency = (uint32_t) (speed / STEP_MOTOR_STEP_ANGLE_DEG);
    frequency = frequency > 0U ? frequency : 1U;

    if (stepper_id == STEP_MOTOR_1) {
        uint32_t period = DCC_100_PWM1_INST_CLK_FREQ / frequency;
        period = period <= TIMER_MAX_PERIOD ? period : TIMER_MAX_PERIOD;
        period = period >= TIMER_MIN_PERIOD ? period : TIMER_MIN_PERIOD;
        DL_Timer_setLoadValue(DCC_100_PWM1_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM1_INST, period / 2U,
            GPIO_DCC_100_PWM1_C0_IDX);
    } else if (stepper_id == STEP_MOTOR_2) {
        uint32_t period = DCC_100_PWM2_INST_CLK_FREQ / frequency;
        period = period <= TIMER_MAX_PERIOD ? period : TIMER_MAX_PERIOD;
        period = period >= TIMER_MIN_PERIOD ? period : TIMER_MIN_PERIOD;
        DL_Timer_setLoadValue(DCC_100_PWM2_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM2_INST, period / 2U,
            GPIO_DCC_100_PWM2_C0_IDX);
    }
}

void step_motor_set_angle(uint8_t angle, uint8_t stepper_id)
{
    uint32_t steps = (uint32_t) (angle / STEP_MOTOR_STEP_ANGLE_DEG);
    (void) StepMotor_StartSteps(steps, stepper_id);
}

bool step_motor_move_steps(int32_t signed_steps, uint8_t stepper_id)
{
    uint32_t steps;

    if (!StepMotor_IsValidId(stepper_id) ||
        (signed_steps == 0) || (signed_steps == INT32_MIN) ||
        step_motor_is_busy(stepper_id)) {
        return false;
    }

    if (signed_steps > 0) {
        step_motor_dir_set(1U, stepper_id);
        steps = (uint32_t) signed_steps;
    } else {
        step_motor_dir_set(0U, stepper_id);
        steps = (uint32_t) (-signed_steps);
    }

    return StepMotor_StartSteps(steps, stepper_id);
}

bool step_motor_move_relative(float angle_deg, uint8_t stepper_id)
{
    float magnitude;
    uint32_t steps;
    int32_t signed_steps;

    if (!StepMotor_IsValidId(stepper_id) || (angle_deg == 0.0f)) {
        return false;
    }

    magnitude = angle_deg > 0.0f ? angle_deg : -angle_deg;
    if (magnitude > ((float) INT32_MAX * STEP_MOTOR_STEP_ANGLE_DEG)) {
        return false;
    }

    steps = (uint32_t) ((magnitude / STEP_MOTOR_STEP_ANGLE_DEG) + 0.5f);
    if (steps == 0U) {
        steps = 1U;
    }

    signed_steps = (int32_t) steps;
    if (angle_deg < 0.0f) {
        signed_steps = -signed_steps;
    }

    return step_motor_move_steps(signed_steps, stepper_id);
}

bool step_motor_is_busy(uint8_t stepper_id)
{
    if (stepper_id == STEP_MOTOR_1) {
        return step_running_1;
    }
    if (stepper_id == STEP_MOTOR_2) {
        return step_running_2;
    }
    return false;
}

uint32_t step_motor_get_remaining_steps(uint8_t stepper_id)
{
    if (stepper_id == STEP_MOTOR_1) {
        return step_remain_1;
    }
    if (stepper_id == STEP_MOTOR_2) {
        return step_remain_2;
    }
    return 0U;
}

void DCC_100_PWM1_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(DCC_100_PWM1_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        /* One LOAD event means one complete PWM pulse was emitted. */
        if (step_remain_1 > 0U) {
            step_remain_1--;
        }
        if (step_remain_1 == 0U) {
            step_motor_stop(STEP_MOTOR_1);
        }
        break;
    default:
        break;
    }
}

void DCC_100_PWM2_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(DCC_100_PWM2_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        /* One LOAD event means one complete PWM pulse was emitted. */
        if (step_remain_2 > 0U) {
            step_remain_2--;
        }
        if (step_remain_2 == 0U) {
            step_motor_stop(STEP_MOTOR_2);
        }
        break;
    default:
        break;
    }
}
