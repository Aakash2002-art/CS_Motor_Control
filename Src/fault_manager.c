/**
 * @file    fault_manager.c
 * @brief   Fault latching implementation. Only the FIRST fault is retained.
 */
#include "fault_manager.h"
#include "motor_driver.h"
#include "app_config.h"

static volatile FaultCode_t g_fault_code = FAULT_NONE;

static const char *const g_fault_text[FAULT_COUNT] =
{
    "none",
    "DRV8411A nFAULT asserted (OCP/UVLO/thermal)",
    "motor current above overcurrent limit",
    "MA782 configuration/status invalid",
    "implausible encoder jump",
    "feedback service gap > 5 ms",
    "peak overtravel > +1 count; reverse correction forbidden",
    "output rollback exceeds raw jitter allowance",
    "endpoint over +1 count during verify; reverse correction forbidden",
    "endpoint unstable during verify",
    "settled overshoot; reverse correction forbidden",
    "stopped >32 counts short; inspect braking/load",
    "forward catch-up limit reached; load/jam inspection required",
    "no forward progress after load-adaptive catch-up; real stall/load limit",
    "stall: no progress with motor current present",
    "no meaningful forward progress for 1200 ms; real stall/load limit",
    "move timeout",
    "output not stationary",
    "direction calibration had no reliable motion/settling",
    "user abort",
    "volumetric/repeat schedule violation"
};

/**
 * @brief Clear the fault latch.
 */
void FaultManager_Init(void)
{
    g_fault_code = FAULT_NONE;
}

/**
 * @brief Latch a fault (first one wins) and disable the bridge. ISR safe.
 */
void FaultManager_Latch(FaultCode_t code)
{
    MotorDriver_Stop();
    if (g_fault_code == FAULT_NONE)
    {
        g_fault_code = code;
    }
}

/**
 * @brief True if a fault is latched.
 */
bool FaultManager_IsLatched(void)
{
    return (g_fault_code != FAULT_NONE);
}

/**
 * @brief The first fault code that was latched (FAULT_NONE if none).
 */
FaultCode_t FaultManager_GetCode(void)
{
    return g_fault_code;
}

/**
 * @brief Human-readable text for a fault code.
 */
const char *FaultManager_GetText(FaultCode_t code)
{
    const char *text = "unknown";

    if ((uint32_t)code < (uint32_t)FAULT_COUNT)
    {
        text = g_fault_text[code];
    }
    return text;
}

/**
 * @brief Clear the latch. Refused while the driver still reports nFAULT.
 */
bool FaultManager_Clear(void)
{
    bool cleared = false;

    if ((APP_IGNORE_DRIVER_FAULTS != 0) || !MotorDriver_IsFaultPinAsserted())
    {
        MotorDriver_ClearFaultFlag();
        g_fault_code = FAULT_NONE;
        cleared = true;
    }
    return cleared;
}

/**
 * @brief nFAULT falling-edge interrupt handler body.
 */
void FaultManager_OnDriverFaultInterrupt(void)
{
    if (APP_IGNORE_DRIVER_FAULTS == 0)
    {
        MotorDriver_NotifyFaultEdge();
        FaultManager_Latch(FAULT_DRIVER_NFAULT);
    }
}

/**
 * @brief Polled backup for a missed edge: nFAULT held low at start-up or after a clear.
 */
void FaultManager_CheckDriverFault(void)
{
    if (MotorDriver_IsFaultPinAsserted())
    {
        FaultManager_OnDriverFaultInterrupt();
    }
}
