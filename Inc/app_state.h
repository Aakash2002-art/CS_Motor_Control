/**
 * @file    app_state.h
 * @brief   Application-level readiness and qualification flags, plus the shared move starter.
 */
#ifndef APP_STATE_H
#define APP_STATE_H

#include "app_types.h"

/** Result of AppMove_Start(). */
typedef enum
{
    APP_MOVE_OK = 0,
    APP_MOVE_NOT_READY,
    APP_MOVE_FAULT_LATCHED,
    APP_MOVE_BUSY,
    APP_MOVE_BAD_ARGUMENT,
    APP_MOVE_ENCODER_CONFIG,
    APP_MOVE_NOT_STATIONARY
} AppMoveStatus_t;

void            AppState_SetReady(bool ready);
bool            AppState_IsReady(void);
void            AppState_SetQualified(bool qualified);
bool            AppState_IsQualified(void);

bool            AppMove_CheckEncoderConfig(void);
AppMoveStatus_t AppMove_Start(float32_t degrees);

#endif /* APP_STATE_H */
