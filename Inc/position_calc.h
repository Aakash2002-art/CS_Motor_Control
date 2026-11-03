/**
 * @file    position_calc.h
 * @brief   Encoder angle unwrapping and unit conversions.
 *
 * The MA782 sits on the FINAL gearbox output, so one native revolution (65536 raw)
 * equals one gearbox-output revolution; no gear ratio is applied in software.
 * 16 raw = 1 virtual count (4096 virtual counts per output revolution).
 */
#ifndef POSITION_CALC_H
#define POSITION_CALC_H

#include "app_types.h"

/** Result of feeding a new angle sample to the unwrapper. */
typedef enum
{
    POSITION_UPDATE_OK = 0,
    POSITION_UPDATE_JUMP        /**< step larger than the plausible maximum; sample rejected */
} PositionUpdateStatus_t;

/** Unwrapped, origin-relative position tracker (sign: positive = delivery direction). */
typedef struct
{
    uint16_t last_angle;
    int32_t  position_raw;
    bool     invert;
} PositionCalc_t;

void                   PositionCalc_Reset(PositionCalc_t *tracker, uint16_t origin_angle, bool invert);
PositionUpdateStatus_t PositionCalc_Update(PositionCalc_t *tracker, uint16_t angle);
int32_t                PositionCalc_AngleDelta(uint16_t newer, uint16_t older);
int32_t                PositionCalc_DegreesToRaw(float32_t degrees);
int32_t                PositionCalc_RawToCount(int32_t raw);
float32_t              PositionCalc_RawToCountsFloat(int32_t raw);

#endif /* POSITION_CALC_H */
