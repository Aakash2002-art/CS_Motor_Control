/**
 * @file    motion_control.h
 * @brief   Position-control algorithm: velocity-profiled position PI, active brake,
 *          coast settling, verification and forward-only hold.
 *
 * Sequence of one move (all drive is in the commanded direction only):
 *   RUN    position PI (outer) -> velocity PI (inner) -> duty, with decelerating profile
 *   BRAKE  active brake (both bridge inputs high)
 *   SETTLE coast with the bridge disabled
 *   PULSE  short load-adaptive catch-up pulse in the move direction (if short of target)
 *   VERIFY 100 ms window: position must stay within +/-1 count, span <= 12 raw
 *   HOLD   bridge disabled; forward-only catch-up if the output drifts below -1 count
 *
 * Overshoot beyond +1 count, rollback, jumps, stalls and timeouts latch a fault.
 * Reverse drive is never used to correct a forward move.
 *
 * MotionControl_Tick() is called from the 2 kHz control interrupt. All other
 * functions are called from the main loop.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "app_types.h"
#include "fault_manager.h"

/** Phase of the motion state machine. */
typedef enum
{
    MOTION_PHASE_IDLE = 0,
    MOTION_PHASE_RUN,
    MOTION_PHASE_PULSE,
    MOTION_PHASE_BRAKE,
    MOTION_PHASE_SETTLE,
    MOTION_PHASE_VERIFY,
    MOTION_PHASE_HOLD,
    MOTION_PHASE_FAULTED
} MotionPhase_t;

/** Result of a start request. */
typedef enum
{
    MOTION_START_OK = 0,
    MOTION_START_BUSY,
    MOTION_START_INVALID,
    MOTION_START_FAULTED
} MotionStartStatus_t;

/** Outcome of one completed (or faulted) move. */
typedef struct
{
    uint32_t    elapsed_ms;
    int32_t     final_raw;        /**< progress in the move direction, native counts */
    int32_t     final_count;      /**< rounded virtual count                          */
    int32_t     target_raw;
    int32_t     target_count;
    float32_t   requested_degrees;
    uint16_t    pulse_count;
    bool        passed;
    FaultCode_t fault;
} MoveResult_t;

/** One buffered trace sample. */
typedef struct
{
    uint32_t ms;
    int32_t  raw;
    uint8_t  phase;
    uint8_t  duty_percent;
} TraceEntry_t;

void                MotionControl_Init(void);

void                MotionControl_SetEncoderInvert(bool invert);
bool                MotionControl_GetEncoderInvert(void);

bool                MotionControl_MeasureStationary(uint16_t *angle);
void                MotionControl_GetCalibrationDiag(bool *jump, bool *settled, bool *nfault, float32_t *peak_ma);
int16_t             MotionControl_GetLastStationarySample(uint32_t index);
void                MotionControl_GetLastStationarySpan(int32_t *minimum, int32_t *maximum);
MotionStartStatus_t MotionControl_StartMove(float32_t degrees, uint16_t origin_angle);
void                MotionControl_Tick(void);
void                MotionControl_Release(void);
void                MotionControl_AbortByUser(void);
bool                MotionControl_CalibrateDirection(float32_t duty_percent, int32_t *observed_raw);

MotionPhase_t       MotionControl_GetPhase(void);
const char         *MotionControl_GetPhaseName(MotionPhase_t phase);
bool                MotionControl_IsMoveActive(void);
bool                MotionControl_TakeResult(MoveResult_t *result);
bool                MotionControl_IsResultPending(void);
uint32_t            MotionControl_GetMoveStartUs(void);
uint32_t            MotionControl_GetMoveEndUs(void);
int32_t             MotionControl_GetProgressRaw(void);
int32_t             MotionControl_GetTargetRaw(void);
float32_t           MotionControl_GetCurrentMilliamps(void);

void                MotionControl_SetTraceEnabled(bool enabled);
bool                MotionControl_IsTraceEnabled(void);
uint16_t            MotionControl_GetTraceCount(void);
bool                MotionControl_GetTraceEntry(uint16_t index, TraceEntry_t *entry);
bool                MotionControl_IsTraceFull(void);
uint32_t            MotionControl_GetTickCounter(void);

#endif /* MOTION_CONTROL_H */
