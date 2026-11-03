/**
 * @file    fault_manager.h
 * @brief   Fault latching and reporting. The fault response is always DISABLE
 *          (bridge coasts); it makes no claim of mechanical holding.
 */
#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include "app_types.h"

/** All latched fault causes. FAULT_NONE means no fault. */
typedef enum
{
    FAULT_NONE = 0,
    FAULT_DRIVER_NFAULT,
    FAULT_DRIVER_OVERCURRENT,
    FAULT_ENCODER_CONFIG,
    FAULT_ENCODER_JUMP,
    FAULT_FEEDBACK_GAP,
    FAULT_OVERTRAVEL,
    FAULT_ROLLBACK,
    FAULT_VERIFY_OVER_TARGET,
    FAULT_VERIFY_UNSTABLE,
    FAULT_SETTLED_OVERSHOOT,
    FAULT_STOPPED_SHORT,
    FAULT_PULSE_LIMIT,
    FAULT_NO_PROGRESS_PULSES,
    FAULT_STALL_CURRENT,
    FAULT_NO_PROGRESS_TIMEOUT,
    FAULT_MOVE_TIMEOUT,
    FAULT_NOT_STATIONARY,
    FAULT_CALIBRATION,
    FAULT_USER_ABORT,
    FAULT_SCHEDULE,
    FAULT_COUNT
} FaultCode_t;

void        FaultManager_Init(void);
void        FaultManager_Latch(FaultCode_t code);
bool        FaultManager_IsLatched(void);
FaultCode_t FaultManager_GetCode(void);
const char *FaultManager_GetText(FaultCode_t code);
bool        FaultManager_Clear(void);
void        FaultManager_OnDriverFaultInterrupt(void);
void        FaultManager_CheckDriverFault(void);

#endif /* FAULT_MANAGER_H */
