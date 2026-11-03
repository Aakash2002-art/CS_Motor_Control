/**
 * @file    app_state.c
 * @brief   Application flags and the shared "start one move" sequence.
 */
#include "app_state.h"
#include "app_config.h"
#include "bsp_hw.h"
#include "fault_manager.h"
#include "ma782_encoder.h"
#include "motion_control.h"
#include <math.h>

static bool g_ready = false;
static bool g_qualified = false;

/**
 * @brief Set the "hardware and MA782 configuration are OK" flag (set once at start-up).
 */
void AppState_SetReady(bool ready)
{
    g_ready = ready;
}

/**
 * @brief True once start-up checks passed.
 */
bool AppState_IsReady(void)
{
    return g_ready;
}

/**
 * @brief Record that a loaded g10.61 move has passed since reboot (needed by r and test/vol).
 */
void AppState_SetQualified(bool qualified)
{
    g_qualified = qualified;
}

/**
 * @brief True if the qualification flag is set.
 */
bool AppState_IsQualified(void)
{
    return g_qualified;
}

/**
 * @brief Verify the MA782 volatile configuration. The control tick is paused
 *        because the SPI bus is shared with the interrupt-driven sampler.
 */
bool AppMove_CheckEncoderConfig(void)
{
    bool pass;

    Bsp_ControlTickSuspend();
    pass = Ma782_ConfigurationPass();
    Bsp_ControlTickResume();
    return pass;
}

/**
 * @brief Start one RELATIVE move from a newly measured stationary origin.
 */
AppMoveStatus_t AppMove_Start(float32_t degrees)
{
    AppMoveStatus_t status = APP_MOVE_OK;
    const float32_t magnitude = fabsf(degrees);
    uint16_t origin = 0U;

    if (!g_ready)
    {
        status = APP_MOVE_NOT_READY;
    }
    else if (FaultManager_IsLatched())
    {
        status = APP_MOVE_FAULT_LATCHED;
    }
    else if (MotionControl_IsMoveActive())
    {
        status = APP_MOVE_BUSY;
    }
    else if ((!isfinite(degrees)) || (magnitude < APP_MOVE_MIN_DEGREES) || (magnitude > APP_MOVE_MAX_DEGREES))
    {
        status = APP_MOVE_BAD_ARGUMENT;
    }
    else if (!AppMove_CheckEncoderConfig())
    {
        FaultManager_Latch(FAULT_ENCODER_CONFIG);
        status = APP_MOVE_ENCODER_CONFIG;
    }
    else if (!MotionControl_MeasureStationary(&origin))
    {
        FaultManager_Latch(FAULT_NOT_STATIONARY);
        status = APP_MOVE_NOT_STATIONARY;
    }
    else if (MotionControl_StartMove(degrees, origin) != MOTION_START_OK)
    {
        status = APP_MOVE_BUSY;
    }
    else
    {
        /* move armed */
    }
    return status;
}
