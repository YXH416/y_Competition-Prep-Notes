/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include <stdio.h>

#include "gimbal_control.h"
#include "k230.h"
#include "step_motor.h"
#include "uart.h"

static void PrintStartupConfiguration(void)
{
    char buffer[192];

    (void) snprintf(buffer, sizeof(buffer),
        "[CHANGE] %s\r\n", GIMBAL_CHANGE_NAME);
    UART_send_string(PRINT_INST, buffer);

    (void) snprintf(buffer, sizeof(buffer),
        "[CFG] image=%ldx%ld focal=(%ld,%ld) A4=(%ld,%ld)mm\r\n",
        (long) GIMBAL_CAMERA_WIDTH_PX,
        (long) GIMBAL_CAMERA_HEIGHT_PX,
        (long) GIMBAL_CAMERA_FOCAL_X_PX,
        (long) GIMBAL_CAMERA_FOCAL_Y_PX,
        (long) GIMBAL_TARGET_WIDTH_MM,
        (long) GIMBAL_TARGET_HEIGHT_MM);
    UART_send_string(PRINT_INST, buffer);

    (void) snprintf(buffer, sizeof(buffer),
        "[CFG] laser_distance=%ldum offset=(%ld,%ld)um "
        "motors=(yaw:%u,pitch:%u) signs=(%ld,%ld)\r\n",
        (long) (GIMBAL_LASER_CAMERA_DISTANCE_MM * 1000.0f),
        (long) (GIMBAL_LASER_CAMERA_OFFSET_X_MM * 1000.0f),
        (long) (GIMBAL_LASER_CAMERA_OFFSET_Y_MM * 1000.0f),
        (unsigned int) GIMBAL_YAW_MOTOR_ID,
        (unsigned int) GIMBAL_PITCH_MOTOR_ID,
        (long) GIMBAL_YAW_MOTOR_SIGN,
        (long) GIMBAL_PITCH_MOTOR_SIGN);
    UART_send_string(PRINT_INST, buffer);

#if GIMBAL_LASER_OFFSET_CALIBRATED == 0U
    UART_send_string(PRINT_INST,
        "[CAL_WARN] Measure laser-camera X/Y offsets before accuracy test.\r\n");
#endif

    UART_send_string(PRINT_INST,
        "[WAIT] K230 frame: $<len>,15,<x>,<y>,<w>,<h>,#\r\n");
}

static void PrintAimDiagnostics(const GimbalControlDiagnostics *diagnostics)
{
    char buffer[224];

    (void) snprintf(buffer, sizeof(buffer),
        "[AIM] n=%lu target=(%ld,%ld) desired=(%ld,%ld) "
        "error=(%ld,%ld) distance=%lumm cmd=(%ld,%ld)mdeg "
        "pos=(%ld,%ld)mdeg sent=(%u,%u) busy=(%u,%u)\r\n",
        (unsigned long) diagnostics->sequence,
        (long) diagnostics->target_center_x_px,
        (long) diagnostics->target_center_y_px,
        (long) diagnostics->desired_center_x_px,
        (long) diagnostics->desired_center_y_px,
        (long) diagnostics->error_x_px,
        (long) diagnostics->error_y_px,
        (unsigned long) diagnostics->estimated_distance_mm,
        (long) diagnostics->yaw_command_mdeg,
        (long) diagnostics->pitch_command_mdeg,
        (long) diagnostics->yaw_position_mdeg,
        (long) diagnostics->pitch_position_mdeg,
        diagnostics->yaw_command_sent ? 1U : 0U,
        diagnostics->pitch_command_sent ? 1U : 0U,
        diagnostics->yaw_busy ? 1U : 0U,
        diagnostics->pitch_busy ? 1U : 0U);
    UART_send_string(PRINT_INST, buffer);
}

static void PrintK230Errors(const K230_Stats *stats)
{
    char buffer[144];

    (void) snprintf(buffer, sizeof(buffer),
        "[K230_ERR] complete=%lu valid=%lu parse=%lu overflow=%lu dropped=%lu\r\n",
        (unsigned long) stats->completed_frames,
        (unsigned long) stats->valid_frames,
        (unsigned long) stats->parse_errors,
        (unsigned long) stats->overflow_errors,
        (unsigned long) stats->dropped_frames);
    UART_send_string(PRINT_INST, buffer);
}

int main(void)
{
    K230_TargetData target;
    K230_Stats stats;
    GimbalControlDiagnostics diagnostics;
    uint32_t last_parse_errors = 0U;
    uint32_t last_overflow_errors = 0U;
    uint32_t last_dropped_frames = 0U;

    SYSCFG_DL_init();
    step_motor_init();
    K230_Init();
    GimbalControl_Init();
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);

    PrintStartupConfiguration();

    while (1) {
        if (K230_PollTarget(&target)) {
            if (GimbalControl_Update(&target, &diagnostics)) {
                if ((diagnostics.sequence % GIMBAL_DEBUG_FRAME_DIVIDER) == 0U) {
                    PrintAimDiagnostics(&diagnostics);
                }
            } else {
                UART_send_string(PRINT_INST,
                    "[AIM_ERR] K230 center is outside configured image size.\r\n");
            }
        }

        K230_GetStats(&stats);
        if ((stats.parse_errors != last_parse_errors) ||
            (stats.overflow_errors != last_overflow_errors) ||
            (stats.dropped_frames != last_dropped_frames)) {
            PrintK230Errors(&stats);
            last_parse_errors = stats.parse_errors;
            last_overflow_errors = stats.overflow_errors;
            last_dropped_frames = stats.dropped_frames;
        }
    }
}
