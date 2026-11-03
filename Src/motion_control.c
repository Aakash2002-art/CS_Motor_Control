/**
 * @file    motion_control.c
 * @brief   Position-control algorithm implementation (see motion_control.h).
 *
 * All positions in this file are kept in the "progress domain": native encoder
 * counts measured from the move origin, multiplied by the move direction sign,
 * so that progress always increases towards the target.
 *
 * MOVE STATE MACHINE (runs in the 2 kHz TIM6 interrupt, MotionControl_Tick):
 *
 *   StartMove -> RUN   speed loop (position PI -> speed reference -> velocity PI -> duty).
 *                      Starts with a breakaway kick; brakes when the remaining distance is
 *                      within the stop-prediction lead (speed * APP_STOP_PREDICTION_S + lead).
 *             -> BRAKE   both bridge inputs high for APP_BRAKE_US
 *             -> SETTLE  coast for APP_SETTLE_US, then SettledDecision:
 *                          within +/-1 count -> VERIFY
 *                          short of target   -> PULSE (short forward catch-up pulse) -> BRAKE -> SETTLE ...
 *                          past +1 count     -> fault (reverse correction is forbidden)
 *             -> VERIFY  stay within +/-1 count with a small span for APP_VERIFY_US
 *             -> HOLD    bridge off; forward-only catch-up pulse if the output drifts short
 *
 *   Any limit violation (overtravel, rollback, encoder jump, sample gap, stall, timeout, ...)
 *   latches a fault, coasts the bridge and posts a failed result.
 *
 * Units: raw = native MA782 counts (65536 per output revolution), count = virtual count
 * (16 raw, 4096 per revolution), cps = virtual counts per second, duty = percent of VM.
 * See docs/FIRMWARE_GUIDE.md and docs/FLOWCHARTS.md for the full description.
 */
#include "motion_control.h"
#include "app_config.h"
#include "bsp_hw.h"
#include "ma782_encoder.h"
#include "motor_driver.h"
#include "pi_controller.h"
#include "position_calc.h"
#include <math.h>

/* ------------------------------------------------------------------------- */
/* Private constants                                                          */
/* ------------------------------------------------------------------------- */
#define PULSE_DESIRED_FRACTION      (0.40F)
#define PULSE_DESIRED_MIN_RAW       (3.0F)
#define PULSE_DESIRED_MAX_RAW       (20.0F)
#define PULSE_RATIO_MIN             (0.50F)
#define PULSE_RATIO_MAX             (2.50F)
#define BREAKAWAY_RAMP_STEP         (1.0F)    /* extra % duty per speed-loop period while the shaft has not moved */
#define FAST_SPEED_WINDOW_US        (2000U)
#define PULSE_INCREMENT_US          (500.0F)
#define SPEED_FILTER_KEEP           (0.5F)
#define MICROSECONDS_PER_SECOND     (1000000.0F)
#define CALIBRATION_MAX_US          (80000U)
#define CALIBRATION_MAX_TRAVEL_RAW  (8192)
#define CALIBRATION_MIN_TRAVEL_RAW  (32)
#define CALIBRATION_BRAKE_MS        (12U)
#define CALIBRATION_SAMPLE_US       (500U)
#define CURRENT_SAMPLE_PERIOD_US    (8000U)
#define MS_PER_US_DIVISOR           (1000U)

/* ------------------------------------------------------------------------- */
/* Module state                                                               */
/* ------------------------------------------------------------------------- */
static volatile MotionPhase_t g_phase = MOTION_PHASE_IDLE;
static volatile bool          g_result_pending = false;
static volatile uint32_t      g_tick_counter = 0U;

static MoveResult_t    g_result;
static PositionCalc_t  g_position;
static PiController_t  g_velocity_pi;
static PiController_t  g_position_pi;

static bool            g_encoder_invert = APP_ENCODER_INVERT_DEFAULT;
static bool            g_move_completed = false;
static bool            g_breakaway_done = true;
static float32_t       g_kick_drive = 0.0F;
static int32_t         g_fast_prev_raw = 0;
static uint32_t        g_fast_prev_us = 0U;
static float32_t       g_fast_speed_cps = 0.0F;
static bool            g_brake_from_pulse = false;

static int32_t         g_direction_sign = 1;
static int32_t         g_target_raw = 0;
static int32_t         g_target_count = 0;
static int32_t         g_progress_raw = 0;
static int32_t         g_peak_raw = 0;
static int32_t         g_progress_mark_raw = 0;
static int32_t         g_speed_prev_raw = 0;
static int32_t         g_verify_min_raw = 0;
static int32_t         g_verify_max_raw = 0;
static int32_t         g_pulse_start_raw = 0;
static int32_t         g_pulse_travel_stop_raw = 0;

static uint32_t        g_move_start_us = 0U;
static uint32_t        g_move_end_us = 0U;
static uint32_t        g_phase_start_us = 0U;
static uint32_t        g_last_sample_us = 0U;
static uint32_t        g_last_speed_us = 0U;
static uint32_t        g_last_current_us = 0U;
static uint32_t        g_progress_mark_us = 0U;
static uint32_t        g_last_trace_us = 0U;

static uint32_t        g_pulse_us = APP_PULSE_INITIAL_US;
static uint16_t        g_pulse_count = 0U;
static uint8_t         g_no_progress_pulses = 0U;

static float32_t       g_requested_degrees = 0.0F;
static float32_t       g_speed_cps = 0.0F;
static float32_t       g_current_ma = 0.0F;

static bool            g_trace_enabled = false;
static bool            g_trace_full = false;
static uint16_t        g_trace_count = 0U;
static TraceEntry_t    g_trace[APP_TRACE_CAPACITY];

/* ------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* ------------------------------------------------------------------------- */
static float32_t MotionControl_Bound(float32_t value, float32_t low, float32_t high)
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
        /* within bounds */
    }
    return result;
}

/**
 * @brief Drive the bridge in the direction of the current move at the given duty (percent of VM).
 */
static void MotionControl_ApplyDrive(float32_t duty_percent)
{
    if (g_direction_sign > 0)
    {
        MotorDriver_RunForward(duty_percent);
    }
    else
    {
        MotorDriver_RunReverse(duty_percent);
    }
}

/**
 * @brief True if the progress is within +/-APP_ACCEPT_RAW (1 virtual count) of the target.
 */
static bool MotionControl_WithinTolerance(int32_t progress_raw)
{
    return (progress_raw >= (g_target_raw - APP_ACCEPT_RAW)) &&
           (progress_raw <= (g_target_raw + APP_ACCEPT_RAW));
}

/**
 * @brief Append a (time, progress, phase, drive) sample to the trace buffer when tracing is on.
 *         "force" records regardless of the trace period (used at phase changes).
 */
static void MotionControl_AddTrace(uint32_t now_us, bool force)
{
    if (g_trace_enabled)
    {
        if (force || ((now_us - g_last_trace_us) >= APP_TRACE_PERIOD_US))
        {
            g_last_trace_us = now_us;
            if (g_trace_count >= APP_TRACE_CAPACITY)
            {
                g_trace_full = true;
            }
            else
            {
                TraceEntry_t *entry = &g_trace[g_trace_count];

                entry->ms = (now_us - g_move_start_us) / MS_PER_US_DIVISOR;
                entry->raw = g_progress_raw;
                entry->phase = (uint8_t)g_phase;
                entry->duty_percent = (uint8_t)lroundf(MotorDriver_GetDutyPercent());
                ++g_trace_count;
            }
        }
    }
}

/**
 * @brief Fill the move result (time, final raw/count, target, pulses, pass/fail, fault) for the main loop to print.
 */
static void MotionControl_PostResult(uint32_t now_us, bool passed)
{
    g_move_end_us = now_us;
    g_result.elapsed_ms = (now_us - g_move_start_us) / MS_PER_US_DIVISOR;
    g_result.final_raw = g_progress_raw;
    g_result.final_count = PositionCalc_RawToCount(g_progress_raw);
    g_result.target_raw = g_target_raw;
    g_result.target_count = g_target_count;
    g_result.requested_degrees = g_requested_degrees;
    g_result.pulse_count = g_pulse_count;
    g_result.passed = passed;
    g_result.fault = FaultManager_GetCode();
    g_result_pending = true;
}

/**
 * @brief Enter the FAULTED phase. A result is posted if the move had not completed.
 */
static void MotionControl_EnterFault(uint32_t now_us)
{
    MotorDriver_Stop();
    if (!g_move_completed)
    {
        MotionControl_AddTrace(now_us, true);
        MotionControl_PostResult(now_us, false);
        g_move_completed = true;
    }
    g_phase = MOTION_PHASE_FAULTED;
}

/**
 * @brief Latch a fault, stop the bridge and post a failed result.
 */
static void MotionControl_Fail(FaultCode_t code, uint32_t now_us)
{
    FaultManager_Latch(code);
    MotionControl_EnterFault(now_us);
}

/**
 * @brief End of the run or of a catch-up pulse: apply the brake (both bridge inputs high) and enter the BRAKE phase.
 */
static void MotionControl_BeginBrake(uint32_t now_us)
{
    g_brake_from_pulse = (g_phase == MOTION_PHASE_PULSE);
    MotorDriver_Brake();
    g_phase = MOTION_PHASE_BRAKE;
    g_phase_start_us = Bsp_GetMicros();
    MotionControl_AddTrace(now_us, true);
}

/**
 * @brief Start one bounded catch-up pulse in the move direction.
 */
static void MotionControl_StartPulse(uint32_t now_us)
{
    if (g_pulse_count >= APP_MAX_PULSES)
    {
        MotionControl_Fail(FAULT_PULSE_LIMIT, now_us);
    }
    else
    {
        const int32_t target_stop_raw = g_target_raw - APP_PULSE_BRAKE_LEAD_RAW;

        g_pulse_start_raw = g_progress_raw;
        /* Pulse time may grow to break stiction, but encoder travel always cuts it short. */
        g_pulse_travel_stop_raw = g_progress_raw + APP_PULSE_TRAVEL_LIMIT_RAW;
        if (g_pulse_travel_stop_raw > target_stop_raw)
        {
            g_pulse_travel_stop_raw = target_stop_raw;
        }
        ++g_pulse_count;
        g_phase = MOTION_PHASE_PULSE;
        MotionControl_ApplyDrive(APP_PULSE_DRIVE_PERCENT);
        g_phase_start_us = Bsp_GetMicros();   /* after any driver wake-up delay */
        MotionControl_AddTrace(now_us, true);
    }
}

/**
 * @brief Adapt the next pulse length after a pulse-and-brake cycle.
 *        Only the pulse TIME is adapted, never the direction or the drive ceiling (APP_PULSE_DRIVE_PERCENT).
 */
static bool MotionControl_AdaptPulse(uint32_t now_us)
{
    bool ok = true;
    const int32_t moved_raw = g_progress_raw - g_pulse_start_raw;

    if (moved_raw < APP_PULSE_NO_PROGRESS_RAW)
    {
        ++g_no_progress_pulses;
        if ((g_no_progress_pulses == 2U) && (g_pulse_us < 3000U))
        {
            g_pulse_us = 3000U;
        }
        else if ((g_no_progress_pulses == 4U) && (g_pulse_us < 5000U))
        {
            g_pulse_us = 5000U;
        }
        else if ((g_no_progress_pulses == 6U) && (g_pulse_us < 8000U))
        {
            g_pulse_us = 8000U;
        }
        else if ((g_no_progress_pulses >= 8U) && (g_pulse_us < 10000U))
        {
            g_pulse_us = 10000U;
        }
        else
        {
            g_pulse_us = (uint32_t)MotionControl_Bound((float32_t)g_pulse_us + PULSE_INCREMENT_US,
                                                       (float32_t)APP_PULSE_MIN_US,
                                                       (float32_t)APP_PULSE_MAX_US);
        }
        if (g_no_progress_pulses >= APP_MAX_NO_PROGRESS_PULSES)
        {
            MotionControl_Fail(FAULT_NO_PROGRESS_PULSES, now_us);
            ok = false;
        }
    }
    else
    {
        const float32_t remaining_raw = (float32_t)(g_target_raw - g_progress_raw);
        const float32_t desired_raw = MotionControl_Bound(remaining_raw * PULSE_DESIRED_FRACTION,
                                                          PULSE_DESIRED_MIN_RAW, PULSE_DESIRED_MAX_RAW);
        const float32_t ratio = MotionControl_Bound(desired_raw / (float32_t)moved_raw,
                                                    PULSE_RATIO_MIN, PULSE_RATIO_MAX);

        g_no_progress_pulses = 0U;
        g_pulse_us = (uint32_t)MotionControl_Bound((float32_t)g_pulse_us * ratio,
                                                   (float32_t)APP_PULSE_MIN_US,
                                                   (float32_t)APP_PULSE_MAX_US);
    }
    return ok;
}

/**
 * @brief Decide what to do once the output has settled after brake/coast.
 */
static void MotionControl_SettledDecision(uint32_t now_us)
{
    bool proceed = true;

    if (g_brake_from_pulse)
    {
        proceed = MotionControl_AdaptPulse(now_us);
    }

    if (proceed)
    {
        const int32_t remaining_raw = g_target_raw - g_progress_raw;

        if (g_progress_raw > (g_target_raw + APP_ACCEPT_RAW))
        {
            MotionControl_Fail(FAULT_SETTLED_OVERSHOOT, now_us);
        }
        else if (MotionControl_WithinTolerance(g_progress_raw))
        {
            g_phase = MOTION_PHASE_VERIFY;
            g_verify_min_raw = g_progress_raw;
            g_verify_max_raw = g_progress_raw;
            g_phase_start_us = now_us;
            MotionControl_AddTrace(now_us, true);
        }
        else if (remaining_raw > (APP_STOPPED_SHORT_LIMIT_COUNTS * APP_ENCODER_RAW_PER_COUNT))
        {
            MotionControl_Fail(FAULT_STOPPED_SHORT, now_us);
        }
        else
        {
            MotionControl_StartPulse(now_us);
        }
    }
}

/**
 * @brief Outer position PI + inner velocity PI, run every APP_SPEED_LOOP_PERIOD_US.
 */
static void MotionControl_RunSpeedLoop(uint32_t now_us)
{
    const uint32_t elapsed_us = now_us - g_last_speed_us;

    if (elapsed_us >= APP_SPEED_LOOP_PERIOD_US)
    {
        const float32_t dt_s = (float32_t)elapsed_us / MICROSECONDS_PER_SECOND;
        const float32_t instant_cps = (float32_t)(g_progress_raw - g_speed_prev_raw) /
                                      ((float32_t)APP_ENCODER_RAW_PER_COUNT * dt_s);
        const float32_t remaining_counts = (float32_t)(g_target_raw - g_progress_raw) /
                                           (float32_t)APP_ENCODER_RAW_PER_COUNT;
        float32_t profile_cps;
        float32_t position_cps;
        float32_t reference_cps;
        float32_t duty;

        g_speed_prev_raw = g_progress_raw;
        g_last_speed_us = now_us;
        g_speed_cps = (SPEED_FILTER_KEEP * g_speed_cps) + ((1.0F - SPEED_FILTER_KEEP) * instant_cps);

        /* Static friction: hold a breakaway kick (ramping up if needed) until the shaft first moves,
           then hand over to the PI loop so it does not spend 60-80 ms creeping up to breakaway. */
        if (!g_breakaway_done)
        {
            if (g_progress_raw >= APP_BREAKAWAY_DETECT_RAW)
            {
                g_breakaway_done = true;
            }
            else
            {
                g_kick_drive = fminf(g_kick_drive + BREAKAWAY_RAMP_STEP, APP_MAX_DRIVE_PERCENT);
                MotionControl_ApplyDrive(g_kick_drive);
                return;
            }
        }

        /* STEP 1 - speed profile: cruise speed, limited by a deceleration envelope
           v = sqrt(2 * a * distance_to_go), and by APP_APPROACH_CPS inside the approach zone. */
        profile_cps = sqrtf(fmaxf(0.0F, 2.0F * APP_DECEL_CPS2 *
                                        fmaxf(0.0F, remaining_counts - APP_PROFILE_STOP_MARGIN_COUNTS)));
        profile_cps = fminf(APP_CRUISE_CPS, profile_cps);
        if (remaining_counts < APP_APPROACH_ZONE_COUNTS)
        {
            profile_cps = fminf(profile_cps, APP_APPROACH_CPS);
        }

        /* STEP 2 - outer position PI: remaining distance (counts) -> speed demand (counts/s).
           The speed reference is the smaller of the profile and this demand. */
        position_cps = PiController_Update(&g_position_pi, remaining_counts, dt_s, 0.0F);
        reference_cps = fminf(profile_cps, position_cps);

        /* Keep a small positive approach speed so a loaded mechanism cannot stall short of the brake zone. */
        if ((remaining_counts > APP_BRAKE_MIN_LEAD_COUNTS) && (reference_cps < APP_MIN_LOADED_APPROACH_CPS))
        {
            reference_cps = APP_MIN_LOADED_APPROACH_CPS;
        }

        /* STEP 3 - inner velocity PI: speed error (counts/s) -> duty (% of VM) around the
           APP_START_DRIVE_PERCENT feed-forward bias. A duty of 0 coasts (it must not brake). */
        duty = PiController_Update(&g_velocity_pi, reference_cps - g_speed_cps, dt_s, APP_START_DRIVE_PERCENT);

        if ((remaining_counts > APP_BRAKE_MIN_LEAD_COUNTS) && (remaining_counts < APP_APPROACH_ZONE_COUNTS) &&
            (fabsf(g_speed_cps) < APP_LOW_SPEED_RECOVERY_CPS))
        {
            duty = fmaxf(duty, APP_LOW_SPEED_RECOVERY_DRIVE);
        }
        MotionControl_ApplyDrive(duty);
    }
}

/**
 * @brief Read and unwrap the encoder, update peak/progress bookkeeping.
 * @return false if a fault was raised.
 */
static bool MotionControl_SampleEncoder(uint32_t now_us)
{
    bool ok = true;

    if ((now_us - g_last_sample_us) > APP_MAX_SAMPLE_GAP_US)
    {
        MotionControl_Fail(FAULT_FEEDBACK_GAP, now_us);
        ok = false;
    }
    else
    {
        const uint16_t angle = Ma782_ReadAngleRaw();

        g_last_sample_us = now_us;
        if (PositionCalc_Update(&g_position, angle) != POSITION_UPDATE_OK)
        {
            MotionControl_Fail(FAULT_ENCODER_JUMP, now_us);
            ok = false;
        }
        else
        {
            g_progress_raw = g_direction_sign * g_position.position_raw;
            if (g_progress_raw > g_peak_raw)
            {
                g_peak_raw = g_progress_raw;
            }
            if (g_progress_raw > (g_progress_mark_raw + 2))
            {
                g_progress_mark_raw = g_progress_raw;
                g_progress_mark_us = now_us;
            }
            MotionControl_AddTrace(now_us, false);
        }
    }
    return ok;
}

/**
 * @brief Protection checks that apply on every tick.
 * @return false if a fault was raised.
 */
static bool MotionControl_CheckLimits(uint32_t now_us)
{
    bool ok = true;
    const MotorState_t motor_state = MotorDriver_GetState();
    const bool driving = (motor_state == MOTOR_STATE_FORWARD) || (motor_state == MOTOR_STATE_REVERSE);

    /* 1) motor current (IPROPI): overcurrent fault; skipped when APP_IGNORE_DRIVER_FAULTS is set. */
    if (driving && ((now_us - g_last_current_us) >= CURRENT_SAMPLE_PERIOD_US))
    {
        g_last_current_us = now_us;
        g_current_ma = MotorDriver_ReadCurrentMilliamps();
        if ((APP_IGNORE_DRIVER_FAULTS == 0) && (g_current_ma > APP_OVERCURRENT_MA))
        {
            MotionControl_Fail(FAULT_DRIVER_OVERCURRENT, now_us);
            ok = false;
        }
    }

    /* 2) position limits: peak past target + 1 count (reverse correction is forbidden) and rollback
       from the peak by more than APP_REVERSE_LIMIT_RAW (gear recoil allowance). */
    if (ok && (g_peak_raw > (g_target_raw + APP_ACCEPT_RAW)))
    {
        MotionControl_Fail(FAULT_OVERTRAVEL, now_us);
        ok = false;
    }

    if (ok && (g_phase != MOTION_PHASE_HOLD) &&
        ((g_peak_raw - g_progress_raw) > APP_REVERSE_LIMIT_RAW))
    {
        MotionControl_Fail(FAULT_ROLLBACK, now_us);
        ok = false;
    }

    /* 3) time limits: overall move timeout, no-progress watchdog, and current-based stall detection. */
    if (ok && !g_move_completed)
    {
        const uint32_t since_start_us = now_us - g_move_start_us;
        const uint32_t since_progress_us = now_us - g_progress_mark_us;
        const bool moving_phase = (g_phase == MOTION_PHASE_RUN) || (g_phase == MOTION_PHASE_PULSE);

        if (since_start_us > APP_MOVE_TIMEOUT_US)
        {
            MotionControl_Fail(FAULT_MOVE_TIMEOUT, now_us);
            ok = false;
        }
        else if (moving_phase && (since_progress_us > APP_STALL_US))
        {
            MotionControl_Fail(FAULT_NO_PROGRESS_TIMEOUT, now_us);
            ok = false;
        }
        else if ((APP_IGNORE_DRIVER_FAULTS == 0) && moving_phase &&
                 (since_progress_us > APP_STALL_FAST_US) && driving &&
                 (g_current_ma >= (APP_STALL_CURRENT_FRACTION *
                                   MotorDriver_ExpectedStallMilliamps(MotorDriver_GetDutyPercent()))))
        {
            MotionControl_Fail(FAULT_STALL_CURRENT, now_us);
            ok = false;
        }
        else
        {
            /* no limit violated */
        }
    }
    return ok;
}

/**
 * @brief Verify window finished successfully: post the result and enter HOLD.
 */
static void MotionControl_CompleteMove(uint32_t now_us)
{
    MotorDriver_Stop();
    if (!g_move_completed)
    {
        MotionControl_PostResult(now_us, true);
        g_move_completed = true;
    }
    g_peak_raw = g_progress_raw;
    g_phase = MOTION_PHASE_HOLD;
    MotionControl_AddTrace(now_us, true);
}

/**
 * @brief VERIFY phase: output must stay within +/-1 count with a span below APP_STABLE_SPAN_RAW for APP_VERIFY_US.
 *         Over target faults, short of target starts a forward catch-up pulse, unstable faults.
 */
static void MotionControl_RunVerify(uint32_t now_us)
{
    if (g_progress_raw > (g_target_raw + APP_ACCEPT_RAW))
    {
        MotionControl_Fail(FAULT_VERIFY_OVER_TARGET, now_us);
    }
    else if (g_progress_raw < (g_target_raw - APP_ACCEPT_RAW))
    {
        /* Load or compliance pulled the output short: bounded catch-up in the move direction only. */
        MotionControl_StartPulse(now_us);
    }
    else
    {
        if (g_progress_raw < g_verify_min_raw)
        {
            g_verify_min_raw = g_progress_raw;
        }
        if (g_progress_raw > g_verify_max_raw)
        {
            g_verify_max_raw = g_progress_raw;
        }
        if ((g_verify_max_raw - g_verify_min_raw) > APP_STABLE_SPAN_RAW)
        {
            MotionControl_Fail(FAULT_VERIFY_UNSTABLE, now_us);
        }
        else if ((now_us - g_phase_start_us) >= APP_VERIFY_US)
        {
            MotionControl_CompleteMove(now_us);
        }
        else
        {
            /* still verifying */
        }
    }
}

/**
 * @brief Short-window (about 2 ms) speed for the brake point. The 8 ms filtered speed lags a stick-slip
 *        burst, so short moves would otherwise brake too late.
 */
static float32_t MotionControl_FastSpeed(uint32_t now_us)
{
    const uint32_t elapsed_us = now_us - g_fast_prev_us;

    if (elapsed_us >= FAST_SPEED_WINDOW_US)
    {
        g_fast_speed_cps = (float32_t)(g_progress_raw - g_fast_prev_raw) * MICROSECONDS_PER_SECOND /
                           ((float32_t)APP_ENCODER_RAW_PER_COUNT * (float32_t)elapsed_us);
        g_fast_prev_raw = g_progress_raw;
        g_fast_prev_us = now_us;
    }
    return g_fast_speed_cps;
}

/**
 * @brief Per-tick state machine. RUN: brake when the remaining distance is within the stop-prediction lead,
 *         otherwise run the speed loop. PULSE: brake on travel limit or near target. BRAKE: coast after APP_BRAKE_US.
 *         SETTLE: decide after APP_SETTLE_US. VERIFY: stability check. HOLD: forward-only catch-up if drifted short.
 */
static void MotionControl_RunPhase(uint32_t now_us)
{
    switch (g_phase)
    {
        case MOTION_PHASE_RUN:
        {
            const float32_t speed_for_lead = fmaxf(0.0F, fmaxf(g_speed_cps, MotionControl_FastSpeed(now_us)));
            const float32_t lead_counts = fmaxf(APP_BRAKE_MIN_LEAD_COUNTS,
                                                (speed_for_lead * APP_STOP_PREDICTION_S) + APP_BRAKE_MIN_LEAD_COUNTS);
            const int32_t lead_raw = (int32_t)ceilf(lead_counts * (float32_t)APP_ENCODER_RAW_PER_COUNT);

            if ((g_target_raw - g_progress_raw) <= lead_raw)
            {
                MotionControl_BeginBrake(now_us);
            }
            else
            {
                MotionControl_RunSpeedLoop(now_us);
            }
            break;
        }

        case MOTION_PHASE_PULSE:
            /* Stop near the target or after limited measured travel. */
            if ((g_progress_raw >= (g_target_raw - APP_PULSE_BRAKE_LEAD_RAW)) ||
                (g_progress_raw >= g_pulse_travel_stop_raw))
            {
                MotionControl_BeginBrake(now_us);
            }
            break;

        case MOTION_PHASE_BRAKE:
            if ((now_us - g_phase_start_us) >= APP_BRAKE_US)
            {
                MotorDriver_Coast();
                g_phase = MOTION_PHASE_SETTLE;
                g_phase_start_us = Bsp_GetMicros();
                MotionControl_AddTrace(now_us, true);
            }
            break;

        case MOTION_PHASE_SETTLE:
            if ((now_us - g_phase_start_us) >= APP_SETTLE_US)
            {
                MotionControl_SettledDecision(now_us);
            }
            break;

        case MOTION_PHASE_VERIFY:
            MotionControl_RunVerify(now_us);
            break;

        case MOTION_PHASE_HOLD:
            if (g_progress_raw < (g_target_raw - APP_ACCEPT_RAW))
            {
                MotionControl_StartPulse(now_us);
            }
            break;

        case MOTION_PHASE_IDLE:
        case MOTION_PHASE_FAULTED:
        default:
            break;
    }
}

/**
 * @brief Stationary check with the bridge disabled. Caller has suspended the control tick.
 */
static int32_t g_last_stationary_min;
static int32_t g_last_stationary_max;
static int16_t g_last_stationary_samples[APP_STATIONARY_SAMPLES];

/**
 * @brief Diagnostic: delta (raw) of one sample of the last stationary check.
 */
int16_t MotionControl_GetLastStationarySample(uint32_t index)
{
    return (index < APP_STATIONARY_SAMPLES) ? g_last_stationary_samples[index] : 0;
}

/**
 * @brief Bridge off, wait, then take APP_STATIONARY_SAMPLES angle samples; stable if the span is within
 *         APP_STABLE_SPAN_RAW. Returns the mean angle. Caller must have suspended the control tick.
 *         Stores min/max and every sample for diagnostics.
 */
static bool MotionControl_MeasureStationaryLocked(uint16_t *angle)
{
    uint16_t origin;
    int32_t minimum = 0;
    int32_t maximum = 0;
    int32_t total = 0;
    uint32_t index;

    MotorDriver_Stop();
    Bsp_DelayMs(APP_STATIONARY_PREWAIT_MS);
    origin = Ma782_ReadAngleRaw();

    for (index = 0U; index < APP_STATIONARY_SAMPLES; ++index)
    {
        int32_t delta;

        Bsp_DelayMs(APP_STATIONARY_SAMPLE_MS);
        delta = PositionCalc_AngleDelta(Ma782_ReadAngleRaw(), origin);
        g_last_stationary_samples[index] = (int16_t)delta;
        if (delta < minimum)
        {
            minimum = delta;
        }
        if (delta > maximum)
        {
            maximum = delta;
        }
        total += delta;
        Bsp_WatchdogRefresh();
    }

    *angle = (uint16_t)((int32_t)origin +
                        (int32_t)lroundf((float32_t)total / (float32_t)APP_STATIONARY_SAMPLES));
    g_last_stationary_min = minimum;
    g_last_stationary_max = maximum;
    return ((maximum - minimum) <= APP_STABLE_SPAN_RAW);
}

static bool g_cal_jump;
static bool g_cal_settled;
static bool g_cal_nfault;
static float32_t g_cal_peak_ma;

/**
 * @brief Diagnostic: flags and peak current recorded during the last direction test.
 */
void MotionControl_GetCalibrationDiag(bool *jump, bool *settled, bool *nfault, float32_t *peak_ma)
{
    *jump = g_cal_jump;
    *settled = g_cal_settled;
    *nfault = g_cal_nfault;
    *peak_ma = g_cal_peak_ma;
}

/**
 * @brief Diagnostic: min and max delta (raw) of the last stationary check.
 */
void MotionControl_GetLastStationarySpan(int32_t *minimum, int32_t *maximum)
{
    *minimum = g_last_stationary_min;
    *maximum = g_last_stationary_max;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void MotionControl_Init(void)
{
    g_phase = MOTION_PHASE_IDLE;
    g_result_pending = false;
    g_move_completed = true;
    g_encoder_invert = APP_ENCODER_INVERT_DEFAULT;
    PiController_Init(&g_velocity_pi, APP_VELOCITY_KP, APP_VELOCITY_KI,
                      -APP_START_DRIVE_PERCENT, APP_MAX_DRIVE_PERCENT, 0.0F, APP_MAX_DRIVE_PERCENT);
    PiController_Init(&g_position_pi, APP_POSITION_KP, APP_POSITION_KI,
                      -APP_POSITION_INTEGRAL_LIMIT, APP_POSITION_INTEGRAL_LIMIT, 0.0F, APP_CRUISE_CPS);
}

/**
 * @brief Set the encoder sign (true = shaft turns the delivery direction when the raw angle decreases).
 */
void MotionControl_SetEncoderInvert(bool invert)
{
    g_encoder_invert = invert;
}

/**
 * @brief Current encoder sign.
 */
bool MotionControl_GetEncoderInvert(void)
{
    return g_encoder_invert;
}

/**
 * @brief Public stationary check: suspends the tick, ends any hold (including a hold pulse in progress, so the
 *         resumed tick sees no false sample gap), runs the check and resumes the tick. Returns true if stable.
 */
bool MotionControl_MeasureStationary(uint16_t *angle)
{
    bool stable;

    Bsp_ControlTickSuspend();
    /* A new stationary measurement ends any forward-only hold (including a hold catch-up pulse in
       progress), so the origin cannot move afterwards and the suspended tick sees no false sample gap. */
    if (g_move_completed && (g_phase != MOTION_PHASE_FAULTED))
    {
        g_phase = MOTION_PHASE_IDLE;
    }
    stable = MotionControl_MeasureStationaryLocked(angle);
    Bsp_ControlTickResume();
    return stable;
}

/**
 * @brief Arm one relative move from a measured stationary origin: set the target in raw counts, reset trackers and
 *         PI controllers, then enter RUN with the breakaway kick. Refused if faulted, busy or the angle is invalid.
 */
MotionStartStatus_t MotionControl_StartMove(float32_t degrees, uint16_t origin_angle)
{
    MotionStartStatus_t status = MOTION_START_OK;
    const float32_t magnitude = fabsf(degrees);

    if (FaultManager_IsLatched())
    {
        status = MOTION_START_FAULTED;
    }
    else if (MotionControl_IsMoveActive())
    {
        status = MOTION_START_BUSY;
    }
    else if ((!isfinite(degrees)) || (magnitude < APP_MOVE_MIN_DEGREES) || (magnitude > APP_MOVE_MAX_DEGREES))
    {
        status = MOTION_START_INVALID;
    }
    else
    {
        uint32_t now_us;

        Bsp_ControlTickSuspend();
        MotorDriver_Stop();

        g_requested_degrees = degrees;
        g_direction_sign = (degrees >= 0.0F) ? 1 : -1;
        /* Control in the native MA782 domain; the commanded angle is not quantised to 4096 counts. */
        g_target_raw = PositionCalc_DegreesToRaw(magnitude);
        g_target_count = (int32_t)lroundf((float32_t)g_target_raw / (float32_t)APP_ENCODER_RAW_PER_COUNT);

        PositionCalc_Reset(&g_position, origin_angle, g_encoder_invert);
        PiController_Reset(&g_velocity_pi);
        PiController_Reset(&g_position_pi);

        g_progress_raw = 0;
        g_peak_raw = 0;
        g_progress_mark_raw = 0;
        g_speed_prev_raw = 0;
        g_verify_min_raw = 0;
        g_verify_max_raw = 0;
        g_pulse_start_raw = 0;
        g_pulse_travel_stop_raw = 0;
        g_speed_cps = 0.0F;
        g_current_ma = 0.0F;
        g_pulse_us = APP_PULSE_INITIAL_US;
        g_pulse_count = 0U;
        g_no_progress_pulses = 0U;
        g_brake_from_pulse = false;
        g_move_completed = false;
        g_result_pending = false;
        g_trace_count = 0U;
        g_trace_full = false;

        now_us = Bsp_GetMicros();
        g_move_start_us = now_us;
        g_move_end_us = 0U;
        g_last_sample_us = now_us;
        g_last_speed_us = now_us;
        g_last_current_us = now_us;
        g_progress_mark_us = now_us;
        g_last_trace_us = now_us;

        g_breakaway_done = false;
        g_kick_drive = APP_BREAKAWAY_DRIVE_PERCENT;
        g_fast_prev_raw = 0;
        g_fast_prev_us = now_us;
        g_fast_speed_cps = 0.0F;

        g_phase = MOTION_PHASE_RUN;
        MotionControl_ApplyDrive(APP_BREAKAWAY_DRIVE_PERCENT);
        MotionControl_AddTrace(now_us, true);
        Bsp_ControlTickResume();
    }
    return status;
}

/**
 * @brief 2 kHz control tick (interrupt context).
 */
void MotionControl_Tick(void)
{
    const MotionPhase_t phase = g_phase;

    ++g_tick_counter;

    if ((phase != MOTION_PHASE_IDLE) && (phase != MOTION_PHASE_FAULTED))
    {
        const uint32_t now_us = Bsp_GetMicros();

        if (FaultManager_IsLatched())
        {
            MotionControl_EnterFault(now_us);
        }
        else
        {
            bool ok;

            /* Pulse deadline is checked before the SPI sample to bound pulse elongation. */
            if ((g_phase == MOTION_PHASE_PULSE) && ((now_us - g_phase_start_us) >= g_pulse_us))
            {
                MotionControl_BeginBrake(now_us);
            }

            ok = MotionControl_SampleEncoder(now_us);
            if (ok)
            {
                ok = MotionControl_CheckLimits(now_us);
            }
            if (ok)
            {
                MotionControl_RunPhase(now_us);
            }
        }
    }
}

/**
 * @brief Leave HOLD/IDLE with the bridge disabled.
 */
void MotionControl_Release(void)
{
    Bsp_ControlTickSuspend();
    MotorDriver_Stop();
    if (!MotionControl_IsMoveActive())
    {
        /* A FAULTED phase is only left once the fault latch has been cleared. */
        if ((g_phase != MOTION_PHASE_FAULTED) || (!FaultManager_IsLatched()))
        {
            g_phase = MOTION_PHASE_IDLE;
        }
    }
    Bsp_ControlTickResume();
}

/**
 * @brief Immediate disable. A move in progress latches a real movement fault.
 */
void MotionControl_AbortByUser(void)
{
    if (MotionControl_IsMoveActive())
    {
        FaultManager_Latch(FAULT_USER_ABORT);
    }
    else
    {
        MotionControl_Release();
    }
}

/**
 * @brief Optional bring-up: short drive in the delivery direction to measure encoder sign.
 */
bool MotionControl_CalibrateDirection(float32_t duty_percent, int32_t *observed_raw)
{
    bool success = false;
    uint16_t origin = 0U;

    *observed_raw = 0;
    Bsp_ControlTickSuspend();

    if (MotionControl_MeasureStationaryLocked(&origin))
    {
        int32_t sum = 0;
        uint16_t previous = origin;
        bool jump = false;
        const uint32_t start_us = Bsp_GetMicros();
        uint16_t settled = 0U;

        g_phase = MOTION_PHASE_IDLE;
        g_cal_peak_ma = 0.0F;
        g_cal_nfault = false;
        MotorDriver_RunForward(duty_percent);

        while (((Bsp_GetMicros() - start_us) < CALIBRATION_MAX_US) && (!jump) &&
               (sum < CALIBRATION_MAX_TRAVEL_RAW) && (sum > -CALIBRATION_MAX_TRAVEL_RAW))
        {
            const uint16_t angle = Ma782_ReadAngleRaw();
            const int32_t delta = PositionCalc_AngleDelta(angle, previous);

            previous = angle;
            {
                const float32_t ma = MotorDriver_ReadCurrentMilliamps();

                if (ma > g_cal_peak_ma)
                {
                    g_cal_peak_ma = ma;
                }
                if (MotorDriver_IsFaultPinAsserted())
                {
                    g_cal_nfault = true;
                }
            }
            if ((delta > APP_ENCODER_MAX_STEP_RAW) || (delta < -APP_ENCODER_MAX_STEP_RAW))
            {
                jump = true;
            }
            else
            {
                sum += delta;
            }
            Bsp_DelayUs(CALIBRATION_SAMPLE_US);
        }

        MotorDriver_Brake();
        Bsp_DelayMs(CALIBRATION_BRAKE_MS);
        MotorDriver_Coast();

        *observed_raw = sum;
        g_cal_jump = jump;
        g_cal_settled = (!jump) && MotionControl_MeasureStationaryLocked(&settled);
        if (g_cal_settled &&
            ((sum >= CALIBRATION_MIN_TRAVEL_RAW) || (sum <= -CALIBRATION_MIN_TRAVEL_RAW)))
        {
            g_encoder_invert = (sum < 0);
            success = true;
        }
        else
        {
            FaultManager_Latch(FAULT_CALIBRATION);
        }
    }
    else
    {
        FaultManager_Latch(FAULT_NOT_STATIONARY);
    }

    Bsp_ControlTickResume();
    return success;
}

/**
 * @brief Current motion phase.
 */
MotionPhase_t MotionControl_GetPhase(void)
{
    return g_phase;
}

/**
 * @brief Phase name for the console and trace.
 */
const char *MotionControl_GetPhaseName(MotionPhase_t phase)
{
    const char *name;

    switch (phase)
    {
        case MOTION_PHASE_RUN:    name = "RUN";             break;
        case MOTION_PHASE_PULSE:  name = "FORWARD_PULSE";   break;
        case MOTION_PHASE_BRAKE:  name = "BRAKE";           break;
        case MOTION_PHASE_SETTLE: name = "SETTLE_DISABLED"; break;
        case MOTION_PHASE_VERIFY: name = "VERIFY_DISABLED"; break;
        case MOTION_PHASE_HOLD:   name = "HOLD";            break;
        case MOTION_PHASE_FAULTED: name = "FAULT";          break;
        case MOTION_PHASE_IDLE:
        default:                  name = "IDLE";            break;
    }
    return name;
}

/**
 * @brief True from move start until the verify window has completed (or a fault).
 */
bool MotionControl_IsMoveActive(void)
{
    const MotionPhase_t phase = g_phase;
    const bool in_move_phase = (phase == MOTION_PHASE_RUN) || (phase == MOTION_PHASE_PULSE) ||
                               (phase == MOTION_PHASE_BRAKE) || (phase == MOTION_PHASE_SETTLE) ||
                               (phase == MOTION_PHASE_VERIFY);

    return in_move_phase && (!g_move_completed);
}

/**
 * @brief Fetch the finished-move result once (clears the pending flag).
 */
bool MotionControl_TakeResult(MoveResult_t *result)
{
    bool available = false;

    if (g_result_pending)
    {
        Bsp_ControlTickSuspend();
        *result = g_result;
        g_result_pending = false;
        available = true;
        Bsp_ControlTickResume();
    }
    return available;
}

/**
 * @brief True if a result is waiting to be printed.
 */
bool MotionControl_IsResultPending(void)
{
    return g_result_pending;
}

/**
 * @brief Microsecond timestamp when the current/last move started (volumetric cadence).
 */
uint32_t MotionControl_GetMoveStartUs(void)
{
    return g_move_start_us;
}

/**
 * @brief Microsecond timestamp when the last move ended.
 */
uint32_t MotionControl_GetMoveEndUs(void)
{
    return g_move_end_us;
}

/**
 * @brief Progress of the current move in raw counts (16 raw = 1 virtual count).
 */
int32_t MotionControl_GetProgressRaw(void)
{
    return g_progress_raw;
}

/**
 * @brief Target of the current move in raw counts.
 */
int32_t MotionControl_GetTargetRaw(void)
{
    return g_target_raw;
}

/**
 * @brief Last measured motor current estimate (IPROPI based).
 */
float32_t MotionControl_GetCurrentMilliamps(void)
{
    return g_current_ma;
}

/**
 * @brief Enable or disable the move trace (printed after each move).
 */
void MotionControl_SetTraceEnabled(bool enabled)
{
    g_trace_enabled = enabled;
}

/**
 * @brief True if tracing is on.
 */
bool MotionControl_IsTraceEnabled(void)
{
    return g_trace_enabled;
}

/**
 * @brief Number of samples in the trace buffer.
 */
uint16_t MotionControl_GetTraceCount(void)
{
    return g_trace_count;
}

/**
 * @brief Read trace sample number "index"; false if out of range.
 */
bool MotionControl_GetTraceEntry(uint16_t index, TraceEntry_t *entry)
{
    bool valid = false;

    if (index < g_trace_count)
    {
        *entry = g_trace[index];
        valid = true;
    }
    return valid;
}

/**
 * @brief True if the trace buffer overflowed (later samples dropped).
 */
bool MotionControl_IsTraceFull(void)
{
    return g_trace_full;
}

/**
 * @brief Control-tick heartbeat counter (the main loop feeds the watchdog only while it changes).
 */
uint32_t MotionControl_GetTickCounter(void)
{
    return g_tick_counter;
}
