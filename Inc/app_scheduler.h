/**
 * @file    app_scheduler.h
 * @brief   Repeatability test and volumetric delivery scheduler.
 *
 * Volumetric schedule: 10 moves x 10.61 deg, 4 s START-to-START, then 180 s after the
 * 10th move completes, for 10 batches (100 moves). A move that cannot keep the
 * cadence aborts the test; a move is never interrupted merely to catch up.
 * The scheduler does NOT estimate delivered volume: use the balance.
 */
#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "app_types.h"
#include "motion_control.h"

void Scheduler_Init(void);
bool Scheduler_StartRepeat(float32_t degrees);
bool Scheduler_StartVolumetric(void);
bool Scheduler_IsActive(void);
bool Scheduler_IsRepeatActive(void);
bool Scheduler_IsVolumetricActive(void);
void Scheduler_Cancel(void);
void Scheduler_Poll(void);
void Scheduler_OnMoveResult(const MoveResult_t *result);
void Scheduler_PrintRunTag(void);
void Scheduler_PrintStatus(void);

#endif /* APP_SCHEDULER_H */
