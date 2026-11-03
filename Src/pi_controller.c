/**
 * @file    pi_controller.c
 * @brief   PI controller implementation.
 *
 * Output = clamp(bias + kp * error + integral). The integral only advances when
 * the output is inside its limits, or when the error is driving it back out of
 * saturation (conditional anti-windup).
 */
#include "pi_controller.h"

/**
 * @brief Limit a value to [low, high].
 */
static float32_t PiController_Clamp(float32_t value, float32_t low, float32_t high)
{
    float32_t result = value;

    if (result < low)
    {
        result = low;
    }
    else if (result > high)
    {
        result = high;
    }
    else
    {
        /* within limits */
    }
    return result;
}

/**
 * @brief Set gains and limits and clear the integral.
 */
void PiController_Init(PiController_t *controller, float32_t kp, float32_t ki,
                       float32_t integral_min, float32_t integral_max,
                       float32_t output_min, float32_t output_max)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->integral = 0.0F;
    controller->integral_min = integral_min;
    controller->integral_max = integral_max;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

/**
 * @brief Clear the integral term (start of a move).
 */
void PiController_Reset(PiController_t *controller)
{
    controller->integral = 0.0F;
}

/**
 * @brief One PI step: output = clamp(bias + kp*error + integral, output limits). The integral is advanced only
 *         when the result is inside the limits or the error is pulling it back out of saturation (anti-windup).
 */
float32_t PiController_Update(PiController_t *controller, float32_t error,
                              float32_t dt_seconds, float32_t bias)
{
    const float32_t candidate = PiController_Clamp(controller->integral + (controller->ki * error * dt_seconds),
                                                   controller->integral_min, controller->integral_max);
    const float32_t unsaturated = bias + (controller->kp * error) + candidate;
    const bool within_limits = (unsaturated >= controller->output_min) &&
                               (unsaturated <= controller->output_max);
    const bool exiting_high = (unsaturated > controller->output_max) && (error < 0.0F);
    const bool exiting_low = (unsaturated < controller->output_min) && (error > 0.0F);

    if (within_limits || exiting_high || exiting_low)
    {
        controller->integral = candidate;
    }

    return PiController_Clamp(bias + (controller->kp * error) + controller->integral,
                              controller->output_min, controller->output_max);
}
