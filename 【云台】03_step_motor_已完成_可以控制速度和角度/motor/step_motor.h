#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * Motor 1: PA28 PWM, PA31 DIR, PB20 DCY, PB24 SLP, PA7 RST
 * Motor 2: PA12 PWM, PA13 DIR, PA14 DCY, PA15 SLP, PA16 RST
 * The configured driver advances 0.05625 degrees per pulse.
 */
#define STEP_MOTOR_1                       (1U)
#define STEP_MOTOR_2                       (2U)
#define STEP_MOTOR_STEP_ANGLE_DEG          (0.05625f)

void step_motor_init(void);
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id);
void step_motor_start(uint8_t stepper_id);
void step_motor_stop(uint8_t stepper_id);
void step_set_speed(uint8_t speed, uint8_t stepper_id);

/* Legacy unsigned-angle API. It keeps the direction selected by dir_set(). */
void step_motor_set_angle(uint8_t angle, uint8_t stepper_id);

/*
 * Non-blocking relative movement used by the gimbal controller.
 * Positive movement selects DIR=1; negative movement selects DIR=0.
 * Returns false when the motor is already moving or the request is invalid.
 */
bool step_motor_move_steps(int32_t signed_steps, uint8_t stepper_id);
bool step_motor_move_relative(float angle_deg, uint8_t stepper_id);

bool step_motor_is_busy(uint8_t stepper_id);
uint32_t step_motor_get_remaining_steps(uint8_t stepper_id);

#endif /* STEP_MOTOR_H */
