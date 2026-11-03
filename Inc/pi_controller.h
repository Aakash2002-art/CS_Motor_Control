/**
 * @file    pi_controller.h
 * @brief   Reusable PI controller with output clamping and conditional anti-windup.
 */
#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "app_types.h"

/** PI controller state and parameters. */
typedef struct
{
    float32_t kp;
    float32_t ki;
    float32_t integral;
    float32_t integral_min;
    float32_t integral_max;
    float32_t output_min;
    float32_t output_max;
} PiController_t;

void      PiController_Init(PiController_t *controller, float32_t kp, float32_t ki,
                            float32_t integral_min, float32_t integral_max,
                            float32_t output_min, float32_t output_max);
void      PiController_Reset(PiController_t *controller);
float32_t PiController_Update(PiController_t *controller, float32_t error,
                              float32_t dt_seconds, float32_t bias);

#endif /* PI_CONTROLLER_H */
