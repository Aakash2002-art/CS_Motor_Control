/**
 * @file    motor_driver.h
 * @brief   DRV8411A bridge-A drive layer: run, brake, coast, stop and diagnostics.
 *
 * This module knows the DRV8411A truth table and nothing about positions or
 * control algorithms. All duty values are percent of the motor supply voltage.
 *
 *   AIN1 AIN2  Bridge        Used for
 *    0    0    Hi-Z (coast)  coast / settle / disabled (sleeps after ~1 ms)
 *    1    PWM  forward slow decay (PWM low = drive, PWM high = brake)
 *    PWM  1    reverse slow decay
 *    1    1    low-side brake
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "app_types.h"

/** Electrical state of the bridge. */
typedef enum
{
    MOTOR_STATE_COAST = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_REVERSE,
    MOTOR_STATE_BRAKE
} MotorState_t;

void         MotorDriver_Init(void);
void         MotorDriver_RunForward(float32_t duty_percent);
void         MotorDriver_RunReverse(float32_t duty_percent);
void         MotorDriver_Brake(void);
void         MotorDriver_Coast(void);
void         MotorDriver_Stop(void);

MotorState_t MotorDriver_GetState(void);
float32_t    MotorDriver_GetDutyPercent(void);

void         MotorDriver_NotifyFaultEdge(void);
bool         MotorDriver_IsFaultFlagSet(void);
bool         MotorDriver_IsFaultPinAsserted(void);
void         MotorDriver_ClearFaultFlag(void);

float32_t    MotorDriver_ReadCurrentMilliamps(void);
float32_t    MotorDriver_ExpectedStallMilliamps(float32_t duty_percent);

#endif /* MOTOR_DRIVER_H */
