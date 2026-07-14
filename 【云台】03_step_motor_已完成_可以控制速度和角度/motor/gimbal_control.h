#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "k230.h"

#define GIMBAL_CHANGE_NAME \
    "CHANGE_20260714_01_K230_LASER_GIMBAL_TRACKING"

/* ---------------- Camera and target calibration ---------------- */

/* These values must match the image coordinates sent by the K230 script. */
#define GIMBAL_CAMERA_WIDTH_PX                 (640.0f)
#define GIMBAL_CAMERA_HEIGHT_PX                (480.0f)
#define GIMBAL_CAMERA_FOCAL_X_PX               (554.0f)
#define GIMBAL_CAMERA_FOCAL_Y_PX               (579.0f)
#define GIMBAL_CAMERA_HORIZONTAL_FOV_DEG       (60.0f)
#define GIMBAL_CAMERA_VERTICAL_FOV_DEG         (45.0f)

/* A4 target dimensions for distance estimation in landscape orientation. */
#define GIMBAL_TARGET_WIDTH_MM                 (297.0f)
#define GIMBAL_TARGET_HEIGHT_MM                (210.0f)
#define GIMBAL_DEFAULT_TARGET_DISTANCE_MM      (1000.0f)
#define GIMBAL_MIN_TARGET_DISTANCE_MM          (300.0f)
#define GIMBAL_MAX_TARGET_DISTANCE_MM          (3000.0f)

/*
 * REQUIRED MEASUREMENT:
 * Measure the laser-to-camera optical-center distance before field testing.
 * The default ratios assume the laser is directly to the camera's right.
 * Positive X is image-right and positive Y is image-down. For a vertical
 * installation, set X_RATIO to 0 and Y_RATIO to +1 or -1.
 */
#define GIMBAL_LASER_CAMERA_DISTANCE_MM        (0.0f)
#define GIMBAL_LASER_OFFSET_X_RATIO            (1.0f)
#define GIMBAL_LASER_OFFSET_Y_RATIO            (0.0f)
#define GIMBAL_LASER_CAMERA_OFFSET_X_MM        \
    (GIMBAL_LASER_CAMERA_DISTANCE_MM * GIMBAL_LASER_OFFSET_X_RATIO)
#define GIMBAL_LASER_CAMERA_OFFSET_Y_MM        \
    (GIMBAL_LASER_CAMERA_DISTANCE_MM * GIMBAL_LASER_OFFSET_Y_RATIO)
#define GIMBAL_LASER_OFFSET_CALIBRATED         (0U)

/* ---------------- Gimbal axis calibration ---------------- */

/* Initial assumption: motor 1 is yaw and motor 2 is pitch. */
#define GIMBAL_YAW_MOTOR_ID                    (1U)
#define GIMBAL_PITCH_MOTOR_ID                  (2U)
#define GIMBAL_YAW_MOTOR_SIGN                  (1.0f)
#define GIMBAL_PITCH_MOTOR_SIGN                (1.0f)
#define GIMBAL_MOTOR_SPEED_DEG_PER_S           (90U)

/* Closed-loop tuning. All moves are relative to the startup position. */
#define GIMBAL_FILTER_ALPHA                    (0.35f)
#define GIMBAL_DEGREES_PER_RADIAN              (57.29578f)
#define GIMBAL_YAW_KP                          (0.65f)
#define GIMBAL_PITCH_KP                        (0.65f)
#define GIMBAL_DEADBAND_X_PX                   (4)
#define GIMBAL_DEADBAND_Y_PX                   (4)
#define GIMBAL_MAX_CORRECTION_DEG              (2.0f)
#define GIMBAL_YAW_SOFT_LIMIT_DEG              (45.0f)
#define GIMBAL_PITCH_SOFT_LIMIT_DEG            (30.0f)

/* Print one diagnostic line after this many accepted K230 frames. */
#define GIMBAL_DEBUG_FRAME_DIVIDER             (5U)

typedef struct {
    uint32_t sequence;
    int32_t target_center_x_px;
    int32_t target_center_y_px;
    int32_t desired_center_x_px;
    int32_t desired_center_y_px;
    int32_t error_x_px;
    int32_t error_y_px;
    uint32_t estimated_distance_mm;
    int32_t yaw_command_mdeg;
    int32_t pitch_command_mdeg;
    int32_t yaw_position_mdeg;
    int32_t pitch_position_mdeg;
    bool yaw_command_sent;
    bool pitch_command_sent;
    bool yaw_busy;
    bool pitch_busy;
} GimbalControlDiagnostics;

void GimbalControl_Init(void);

/*
 * Process one valid K230 target and issue non-blocking relative motor moves.
 * Returns false when the target coordinates are outside the configured image.
 */
bool GimbalControl_Update(
    const K230_TargetData *target, GimbalControlDiagnostics *diagnostics);

#endif /* GIMBAL_CONTROL_H */
