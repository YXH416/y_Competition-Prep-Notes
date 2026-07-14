#include "gimbal_control.h"

#include <stddef.h>

#include "step_motor.h"

static bool filter_initialized;
static float filtered_center_x;
static float filtered_center_y;
static float filtered_distance_mm;
static float yaw_position_deg;
static float pitch_position_deg;
static uint32_t update_sequence;

static float Gimbal_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t Gimbal_RoundToInt(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }
    return (int32_t) (value - 0.5f);
}

static uint32_t Gimbal_EstimateTargetDistance(const K230_TargetData *target)
{
    float distance_x = GIMBAL_DEFAULT_TARGET_DISTANCE_MM;
    float distance_y = GIMBAL_DEFAULT_TARGET_DISTANCE_MM;
    float distance;

    if (target->width > 0) {
        distance_x = (GIMBAL_CAMERA_FOCAL_X_PX * GIMBAL_TARGET_WIDTH_MM) /
                     (float) target->width;
    }
    if (target->height > 0) {
        distance_y = (GIMBAL_CAMERA_FOCAL_Y_PX * GIMBAL_TARGET_HEIGHT_MM) /
                     (float) target->height;
    }

    distance = (distance_x + distance_y) * 0.5f;
    distance = Gimbal_ClampFloat(distance,
        GIMBAL_MIN_TARGET_DISTANCE_MM, GIMBAL_MAX_TARGET_DISTANCE_MM);
    return (uint32_t) (distance + 0.5f);
}

static bool Gimbal_CommandAxis(float requested_angle_deg,
    uint8_t motor_id, float soft_limit_deg, float *position_deg,
    int32_t *command_mdeg)
{
    float limited_angle;
    float candidate_position;
    int32_t steps;
    float actual_angle;

    if ((position_deg == NULL) || (command_mdeg == NULL) ||
        step_motor_is_busy(motor_id)) {
        return false;
    }

    limited_angle = Gimbal_ClampFloat(requested_angle_deg,
        -GIMBAL_MAX_CORRECTION_DEG, GIMBAL_MAX_CORRECTION_DEG);
    candidate_position = *position_deg + limited_angle;

    if (candidate_position > soft_limit_deg) {
        limited_angle = soft_limit_deg - *position_deg;
    } else if (candidate_position < -soft_limit_deg) {
        limited_angle = -soft_limit_deg - *position_deg;
    }

    steps = Gimbal_RoundToInt(
        limited_angle / STEP_MOTOR_STEP_ANGLE_DEG);
    if (steps == 0) {
        *command_mdeg = 0;
        return false;
    }

    if (!step_motor_move_steps(steps, motor_id)) {
        return false;
    }

    actual_angle = (float) steps * STEP_MOTOR_STEP_ANGLE_DEG;
    *position_deg += actual_angle;
    *command_mdeg = Gimbal_RoundToInt(actual_angle * 1000.0f);
    return true;
}

void GimbalControl_Init(void)
{
    filter_initialized = false;
    filtered_center_x = GIMBAL_CAMERA_WIDTH_PX * 0.5f;
    filtered_center_y = GIMBAL_CAMERA_HEIGHT_PX * 0.5f;
    filtered_distance_mm = GIMBAL_DEFAULT_TARGET_DISTANCE_MM;
    yaw_position_deg = 0.0f;
    pitch_position_deg = 0.0f;
    update_sequence = 0U;

    step_set_speed(GIMBAL_MOTOR_SPEED_DEG_PER_S, GIMBAL_YAW_MOTOR_ID);
    step_set_speed(GIMBAL_MOTOR_SPEED_DEG_PER_S, GIMBAL_PITCH_MOTOR_ID);
}

bool GimbalControl_Update(
    const K230_TargetData *target, GimbalControlDiagnostics *diagnostics)
{
    float measured_distance;
    float desired_center_x;
    float desired_center_y;
    float error_x;
    float error_y;
    float yaw_angle = 0.0f;
    float pitch_angle = 0.0f;
    bool yaw_sent = false;
    bool pitch_sent = false;
    int32_t yaw_command_mdeg = 0;
    int32_t pitch_command_mdeg = 0;

    if ((target == NULL) || (diagnostics == NULL) ||
        (target->center_x < 0) || (target->center_y < 0) ||
        ((float) target->center_x >= GIMBAL_CAMERA_WIDTH_PX) ||
        ((float) target->center_y >= GIMBAL_CAMERA_HEIGHT_PX)) {
        return false;
    }

    measured_distance = (float) Gimbal_EstimateTargetDistance(target);
    if (!filter_initialized) {
        filtered_center_x = (float) target->center_x;
        filtered_center_y = (float) target->center_y;
        filtered_distance_mm = measured_distance;
        filter_initialized = true;
    } else {
        filtered_center_x += GIMBAL_FILTER_ALPHA *
            ((float) target->center_x - filtered_center_x);
        filtered_center_y += GIMBAL_FILTER_ALPHA *
            ((float) target->center_y - filtered_center_y);
        filtered_distance_mm += GIMBAL_FILTER_ALPHA *
            (measured_distance - filtered_distance_mm);
    }

    /*
     * A displaced laser must aim along (target - laser_offset). Therefore the
     * camera should keep the target at a shifted pixel center, not at the
     * optical image center. The shift changes with target distance.
     */
    desired_center_x = (GIMBAL_CAMERA_WIDTH_PX * 0.5f) +
        ((GIMBAL_CAMERA_FOCAL_X_PX * GIMBAL_LASER_CAMERA_OFFSET_X_MM) /
         filtered_distance_mm);
    desired_center_y = (GIMBAL_CAMERA_HEIGHT_PX * 0.5f) +
        ((GIMBAL_CAMERA_FOCAL_Y_PX * GIMBAL_LASER_CAMERA_OFFSET_Y_MM) /
         filtered_distance_mm);

    error_x = filtered_center_x - desired_center_x;
    error_y = filtered_center_y - desired_center_y;

    if ((error_x > (float) GIMBAL_DEADBAND_X_PX) ||
        (error_x < (float) -GIMBAL_DEADBAND_X_PX)) {
        yaw_angle = (error_x / GIMBAL_CAMERA_FOCAL_X_PX) *
            GIMBAL_DEGREES_PER_RADIAN *
            GIMBAL_YAW_KP * GIMBAL_YAW_MOTOR_SIGN;
        yaw_sent = Gimbal_CommandAxis(yaw_angle, GIMBAL_YAW_MOTOR_ID,
            GIMBAL_YAW_SOFT_LIMIT_DEG, &yaw_position_deg,
            &yaw_command_mdeg);
    }

    if ((error_y > (float) GIMBAL_DEADBAND_Y_PX) ||
        (error_y < (float) -GIMBAL_DEADBAND_Y_PX)) {
        pitch_angle = (error_y / GIMBAL_CAMERA_FOCAL_Y_PX) *
            GIMBAL_DEGREES_PER_RADIAN *
            GIMBAL_PITCH_KP * GIMBAL_PITCH_MOTOR_SIGN;
        pitch_sent = Gimbal_CommandAxis(pitch_angle, GIMBAL_PITCH_MOTOR_ID,
            GIMBAL_PITCH_SOFT_LIMIT_DEG, &pitch_position_deg,
            &pitch_command_mdeg);
    }

    update_sequence++;
    diagnostics->sequence = update_sequence;
    diagnostics->target_center_x_px = Gimbal_RoundToInt(filtered_center_x);
    diagnostics->target_center_y_px = Gimbal_RoundToInt(filtered_center_y);
    diagnostics->desired_center_x_px = Gimbal_RoundToInt(desired_center_x);
    diagnostics->desired_center_y_px = Gimbal_RoundToInt(desired_center_y);
    diagnostics->error_x_px = Gimbal_RoundToInt(error_x);
    diagnostics->error_y_px = Gimbal_RoundToInt(error_y);
    diagnostics->estimated_distance_mm =
        (uint32_t) (filtered_distance_mm + 0.5f);
    diagnostics->yaw_command_mdeg = yaw_command_mdeg;
    diagnostics->pitch_command_mdeg = pitch_command_mdeg;
    diagnostics->yaw_position_mdeg =
        Gimbal_RoundToInt(yaw_position_deg * 1000.0f);
    diagnostics->pitch_position_mdeg =
        Gimbal_RoundToInt(pitch_position_deg * 1000.0f);
    diagnostics->yaw_command_sent = yaw_sent;
    diagnostics->pitch_command_sent = pitch_sent;
    diagnostics->yaw_busy = step_motor_is_busy(GIMBAL_YAW_MOTOR_ID);
    diagnostics->pitch_busy = step_motor_is_busy(GIMBAL_PITCH_MOTOR_ID);

    return true;
}
