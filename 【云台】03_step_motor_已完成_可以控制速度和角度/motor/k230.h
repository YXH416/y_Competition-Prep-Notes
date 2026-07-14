#ifndef K230_H
#define K230_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * K230 UART wiring:
 *   K230 TXD (IO9)  -> MSPM0 PA22 (UART2 RX)
 *   K230 RXD (IO10) -> MSPM0 PA21 (UART2 TX)
 *   K230 GND        -> MSPM0 GND
 *   K230 5V         -> board 5V/VCC
 *
 * Target tracking frame:
 *   $<length>,15,<x>,<y>,<width>,<height>,#
 * x and y are the top-left corner of the target box.
 */

#define K230_TARGET_TRACKING_FUNCTION_ID    (15U)
#define K230_FRAME_MAX_LENGTH               (64U)

typedef struct {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    int32_t center_x;
    int32_t center_y;
} K230_TargetData;

typedef struct {
    uint32_t completed_frames;
    uint32_t valid_frames;
    uint32_t parse_errors;
    uint32_t overflow_errors;
    uint32_t dropped_frames;
} K230_Stats;

/* Call once after SYSCFG_DL_init(). */
void K230_Init(void);

/*
 * Call repeatedly in the main loop. Returns true when one complete and valid
 * target frame is available. The received frame is consumed by this call.
 */
bool K230_PollTarget(K230_TargetData *target);

/* Public parser for direct frame parsing and protocol-level tests. */
bool K230_ParseFrame(
    const uint8_t *frame, uint8_t length, K230_TargetData *target);

void K230_GetStats(K230_Stats *stats);

#endif /* K230_H */
