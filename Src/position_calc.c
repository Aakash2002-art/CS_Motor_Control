/**
 * @file    position_calc.c
 * @brief   Encoder angle unwrapping and unit conversions.
 */
#include "position_calc.h"
#include "app_config.h"
#include <math.h>

#define ANGLE_MODULUS       (65536)
#define ANGLE_HALF_RANGE    (32768U)

/**
 * @brief Start tracking from a measured origin angle: position 0, remember the encoder sign.
 */
void PositionCalc_Reset(PositionCalc_t *tracker, uint16_t origin_angle, bool invert)
{
    tracker->last_angle = origin_angle;
    tracker->position_raw = 0;
    tracker->invert = invert;
}

/**
 * @brief Shortest signed difference between two 16-bit angles (circular).
 */
int32_t PositionCalc_AngleDelta(uint16_t newer, uint16_t older)
{
    const uint16_t difference = (uint16_t)(newer - older);
    int32_t delta;

    if (difference >= ANGLE_HALF_RANGE)
    {
        delta = (int32_t)difference - ANGLE_MODULUS;
    }
    else
    {
        delta = (int32_t)difference;
    }
    return delta;
}

/**
 * @brief Add the shortest circular angle step to the unwrapped position. A step larger than
 *         APP_ENCODER_MAX_STEP_RAW is rejected as an encoder jump. The sign is applied here (encoder_invert).
 */
PositionUpdateStatus_t PositionCalc_Update(PositionCalc_t *tracker, uint16_t angle)
{
    PositionUpdateStatus_t status = POSITION_UPDATE_OK;
    int32_t delta = PositionCalc_AngleDelta(angle, tracker->last_angle);

    /* A suspect sample is never substituted by the last good one while moving. */
    if ((delta > APP_ENCODER_MAX_STEP_RAW) || (delta < -APP_ENCODER_MAX_STEP_RAW))
    {
        status = POSITION_UPDATE_JUMP;
    }
    else
    {
        tracker->last_angle = angle;
        if (tracker->invert)
        {
            delta = -delta;
        }
        tracker->position_raw += delta;
    }
    return status;
}

/**
 * @brief Degrees to native MA782 counts (65536 per output revolution), rounded.
 */
int32_t PositionCalc_DegreesToRaw(float32_t degrees)
{
    /* Controlled in the native domain: 10.61 deg is about 1931.49 raw, not 1936. */
    return (int32_t)lroundf(degrees * ((float32_t)APP_ENCODER_RAW_PER_REV / APP_FULL_REV_DEGREES));
}

/**
 * @brief Native counts to rounded virtual counts (16 raw = 1 count, 4096 per revolution).
 */
int32_t PositionCalc_RawToCount(int32_t raw)
{
    int32_t count;

    if (raw >= 0)
    {
        count = (raw + (APP_ENCODER_RAW_PER_COUNT / 2)) / APP_ENCODER_RAW_PER_COUNT;
    }
    else
    {
        count = (raw - (APP_ENCODER_RAW_PER_COUNT / 2)) / APP_ENCODER_RAW_PER_COUNT;
    }
    return count;
}

/**
 * @brief Native counts to virtual counts as a float.
 */
float32_t PositionCalc_RawToCountsFloat(int32_t raw)
{
    return (float32_t)raw / (float32_t)APP_ENCODER_RAW_PER_COUNT;
}
