/**
 * @file    app_scheduler.c
 * @brief   Repeatability test and volumetric delivery scheduler implementation.
 */
#include "app_scheduler.h"
#include "app_config.h"
#include "app_state.h"
#include "bsp_hw.h"
#include "console.h"
#include "fault_manager.h"
#include "motor_driver.h"
#include <math.h>

#define QUALIFICATION_TOLERANCE_DEG  (0.001F)
#define PREFLIGHT_EXTRA_US           (50000UL)
#define MS_DIVISOR                   (1000UL)

typedef enum
{
    VOL_IDLE = 0,
    VOL_WAIT_PREFLIGHT,
    VOL_MOVE_ACTIVE
} VolState_t;

/* Repeatability test state */
static bool      g_repeat_active = false;
static uint8_t   g_repeat_index = 0U;
static float32_t g_repeat_degrees = 0.0F;
static uint32_t  g_repeat_due_us = 0U;

/* Volumetric test state */
static bool       g_vol_active = false;
static VolState_t g_vol_state = VOL_IDLE;
static uint8_t    g_vol_batch = 0U;
static uint8_t    g_vol_move = 0U;
static uint32_t   g_vol_next_start_us = 0U;
static uint32_t   g_vol_overall_start_ms = 0U;
static uint32_t   g_vol_completed = 0U;
static int32_t    g_vol_last_start_error_us = 0;
static bool       g_vol_timing_violation = false;

/**
 * @brief Signed wrap-safe difference (later - earlier) of two microsecond stamps.
 */
static int32_t Scheduler_TimeDiffUs(uint32_t later, uint32_t earlier)
{
    const uint32_t difference = later - earlier;
    int32_t result;

    if (difference <= 0x7FFFFFFFUL)
    {
        result = (int32_t)difference;
    }
    else
    {
        result = -(int32_t)(0UL - difference);
    }
    return result;
}

/**
 * @brief Clear the volumetric-schedule state (does not touch the motor).
 */
static void Scheduler_FinishVolumetric(void)
{
    g_vol_active = false;
    g_vol_state = VOL_IDLE;
    g_vol_timing_violation = false;
}

/**
 * @brief Print the banner shown when the volumetric schedule stops, with the reason and progress.
 */
static void Scheduler_PrintStopBanner(const char *reason)
{
    Console_PrintNewLine();
    Console_PrintLine("############################################################");
    Console_PrintLine("# VOLUMETRIC SCHEDULE STOPPED");
    Console_PrintString("# Reason                  : ");
    Console_PrintLine(reason);
    Console_PrintString("# Completed movements     : ");
    Console_PrintUint(g_vol_completed);
    Console_PrintNewLine();
    Console_PrintString("# Current batch/move      : ");
    Console_PrintUint((uint32_t)g_vol_batch + 1U);
    Console_PrintString("/");
    Console_PrintUint(APP_VOL_BATCHES);
    Console_PrintString(" batch, ");
    Console_PrintUint((uint32_t)g_vol_move + 1U);
    Console_PrintString("/");
    Console_PrintUint(APP_VOL_MOVES_PER_BATCH);
    Console_PrintNewLine();
    Console_PrintLine("############################################################");
    Console_PrintNewLine();
}

/**
 * @brief Stop the volumetric schedule. If a move is in progress that is a real movement fault (latched);
 *         otherwise the bridge is simply released.
 */
static void Scheduler_StopVolumetric(const char *reason)
{
    if (g_vol_active)
    {
        Scheduler_FinishVolumetric();
        if (MotionControl_IsMoveActive())
        {
            /* An abort during motion is a real movement fault. */
            FaultManager_Latch(FAULT_SCHEDULE);
        }
        else
        {
            MotionControl_Release();
        }
        Scheduler_PrintStopBanner(reason);
    }
}

/**
 * @brief Reset the repeat and volumetric schedulers to idle.
 */
void Scheduler_Init(void)
{
    g_repeat_active = false;
    g_vol_active = false;
    g_vol_state = VOL_IDLE;
}

/**
 * @brief Start the repeatability test (APP_REPEAT_CYCLES moves with a rest after each completion).
 *         Returns false if the first move could not start.
 */
bool Scheduler_StartRepeat(float32_t degrees)
{
    g_repeat_index = 1U;
    g_repeat_degrees = degrees;
    g_repeat_active = true;
    if (AppMove_Start(degrees) != APP_MOVE_OK)
    {
        g_repeat_active = false;
    }
    return g_repeat_active;
}

/**
 * @brief Start the volumetric schedule (batches of moves, fixed start-to-start period, long wait between batches).
 *         Refused unless the system is ready, idle, fault-free, the MA782 config is valid and a loaded
 *         g10.61 qualification has passed since reboot.
 */
bool Scheduler_StartVolumetric(void)
{
    bool started = false;

    if (g_vol_active)
    {
        Console_PrintLine("[TEST] volumetric schedule already running");
    }
    else if ((!AppState_IsReady()) || FaultManager_IsLatched() || MotionControl_IsMoveActive() || g_repeat_active)
    {
        Console_PrintLine("[TEST] BLOCKED - require ready, idle, no fault/repeat");
    }
    else if (!AppMove_CheckEncoderConfig())
    {
        Console_PrintLine("[TEST] BLOCKED - MA782 configuration/status invalid");
    }
    else if (!AppState_IsQualified())
    {
        Console_PrintLine("[TEST] BLOCKED - no loaded g10.61 qualification since reboot");
    }
    else
    {
        g_vol_active = true;
        g_vol_state = VOL_WAIT_PREFLIGHT;
        g_vol_batch = 0U;
        g_vol_move = 0U;
        g_vol_completed = 0U;
        g_vol_overall_start_ms = Bsp_GetMillis();
        g_vol_last_start_error_us = 0;
        g_vol_timing_violation = false;
        /* The first preflight needs time; every later start is anchored to the previous start. */
        g_vol_next_start_us = Bsp_GetMicros() + APP_VOL_PREFLIGHT_US + PREFLIGHT_EXTRA_US;
        Console_PrintLine("[TEST] VOLUMETRIC START 10 batches x 10 moves, 10.61 deg, 4 s start-to-start, 180 s inter-batch");
        started = true;
    }
    return started;
}

/**
 * @brief True if either a repeat or a volumetric schedule is running.
 */
bool Scheduler_IsActive(void)
{
    return g_repeat_active || g_vol_active;
}

/**
 * @brief True while the repeatability test is running.
 */
bool Scheduler_IsRepeatActive(void)
{
    return g_repeat_active;
}

/**
 * @brief True while the volumetric schedule is running.
 */
bool Scheduler_IsVolumetricActive(void)
{
    return g_vol_active;
}

/**
 * @brief The 'x' command: cancel any schedule (no move in progress).
 */
void Scheduler_Cancel(void)
{
    g_repeat_active = false;
    if (g_vol_active)
    {
        Scheduler_StopVolumetric("cancelled by user");
    }
}

/**
 * @brief Volumetric bookkeeping after each move: abort on a fault or a cadence violation, otherwise set the next
 *         start time (4 s after the ACTUAL previous start, or 180 s after the 10th move completes) and advance batch/move.
 */
static void Scheduler_OnVolumetricResult(const MoveResult_t *result)
{
    if (g_vol_state == VOL_MOVE_ACTIVE)
    {
        if (!result->passed)
        {
            Console_PrintString("[TEST] FAULT Batch ");
            Console_PrintUint((uint32_t)g_vol_batch + 1U);
            Console_PrintString("/");
            Console_PrintUint(APP_VOL_BATCHES);
            Console_PrintString(" Move ");
            Console_PrintUint((uint32_t)g_vol_move + 1U);
            Console_PrintString("/");
            Console_PrintUint(APP_VOL_MOVES_PER_BATCH);
            Console_PrintNewLine();
            Scheduler_FinishVolumetric();
            Console_PrintLine("[TEST] volumetric schedule aborted on first movement fault");
        }
        else
        {
            ++g_vol_completed;

            if (g_vol_timing_violation)
            {
                Scheduler_FinishVolumetric();
                Console_PrintLine("[TEST] STOP - actual movement start exceeded volumetric scheduling tolerance");
            }
            else if ((uint32_t)(g_vol_move + 1U) < APP_VOL_MOVES_PER_BATCH)
            {
                /* Next move: exactly 4 s after the ACTUAL previous movement start. */
                ++g_vol_move;
                g_vol_next_start_us = MotionControl_GetMoveStartUs() + APP_VOL_PERIOD_US;
                g_vol_state = VOL_WAIT_PREFLIGHT;
                if (Scheduler_TimeDiffUs(Bsp_GetMicros(), g_vol_next_start_us) >= 0)
                {
                    Scheduler_FinishVolumetric();
                    Console_PrintLine("[TEST] STOP - previous loaded move exceeded 4 s start-to-start cadence");
                }
            }
            else
            {
                Console_PrintString("[TEST] Batch ");
                Console_PrintUint((uint32_t)g_vol_batch + 1U);
                Console_PrintLine(" COMPLETE.");

                if ((uint32_t)(g_vol_batch + 1U) >= APP_VOL_BATCHES)
                {
                    Scheduler_FinishVolumetric();
                    Console_PrintString("[TEST] VOLUMETRIC COMPLETE movements=");
                    Console_PrintUint(g_vol_completed);
                    Console_PrintString(" total_time_ms=");
                    Console_PrintUint(Bsp_GetMillis() - g_vol_overall_start_ms);
                    Console_PrintNewLine();
                }
                else
                {
                    /* The full 180 s is measured from completion of the 10th movement. */
                    ++g_vol_batch;
                    g_vol_move = 0U;
                    g_vol_next_start_us = MotionControl_GetMoveEndUs() + APP_VOL_INTER_BATCH_WAIT_US;
                    g_vol_state = VOL_WAIT_PREFLIGHT;
                    Console_PrintString("[TEST] Waiting ");
                    Console_PrintUint(APP_VOL_INTER_BATCH_WAIT_US / 1000000UL);
                    Console_PrintLine(" seconds after move 10 completion before next batch start.");
                }
            }
        }
    }
}

/**
 * @brief Called for every finished move. A passed standalone g10.61 qualifies repeat/volumetric runs until reboot;
 *         then drives the active schedule (repeat dwell or volumetric sequencing).
 */
void Scheduler_OnMoveResult(const MoveResult_t *result)
{
    /* A successful standalone loaded g10.61 qualifies repeat/volumetric runs until reboot. */
    if ((!g_repeat_active) && (!g_vol_active) && result->passed &&
        (fabsf(result->requested_degrees - APP_VOL_TEST_DEGREES) < QUALIFICATION_TOLERANCE_DEG))
    {
        AppState_SetQualified(true);
    }

    if (g_vol_active)
    {
        Scheduler_OnVolumetricResult(result);
    }
    else if (g_repeat_active)
    {
        if (!result->passed)
        {
            g_repeat_active = false;   /* repeatability stops at the first fault */
        }
        else if (g_repeat_index >= APP_REPEAT_CYCLES)
        {
            g_repeat_active = false;
            Console_PrintLine("[REP] COMPLETE 10/10");
        }
        else
        {
            g_repeat_due_us = Bsp_GetMicros() + APP_REPEAT_DWELL_US;
        }
    }
    else
    {
        /* standalone move */
    }
}

/**
 * @brief Start the next repeatability cycle once the post-completion dwell has elapsed.
 */
static void Scheduler_PollRepeat(void)
{
    if (g_repeat_active && (!g_vol_active) && (!MotionControl_IsMoveActive()) &&
        (Scheduler_TimeDiffUs(Bsp_GetMicros(), g_repeat_due_us) >= 0))
    {
        ++g_repeat_index;
        if (AppMove_Start(g_repeat_degrees) != APP_MOVE_OK)
        {
            g_repeat_active = false;
            Console_PrintLine("[REP] stopped: next cycle could not start");
        }
    }
}

/**
 * @brief Volumetric preflight: shortly before each scheduled start check the encoder config and that the output is
 *         stationary, wait (feeding the watchdog) for the exact start time, then start the move. Never starts late.
 */
static void Scheduler_PollVolumetric(void)
{
    const uint32_t now_us = Bsp_GetMicros();
    const uint32_t preflight_at_us = g_vol_next_start_us - APP_VOL_PREFLIGHT_US;

    if (g_vol_active && (!MotionControl_IsMoveActive()) && (g_vol_state == VOL_WAIT_PREFLIGHT) &&
        (Scheduler_TimeDiffUs(now_us, preflight_at_us) >= 0))
    {
        uint16_t origin = 0U;

        if (FaultManager_IsLatched() || g_repeat_active || (!AppMove_CheckEncoderConfig()))
        {
            Scheduler_StopVolumetric("preflight readiness/configuration failed");
        }
        else if (!MotionControl_MeasureStationary(&origin))
        {
            Scheduler_StopVolumetric("output not stationary during scheduled preflight");
        }
        else if (Scheduler_TimeDiffUs(g_vol_next_start_us, Bsp_GetMicros()) < -(int32_t)APP_VOL_START_TOLERANCE_US)
        {
            /* Never start late just to keep the test running. */
            Scheduler_StopVolumetric("scheduled start missed during preflight");
        }
        else
        {
            while (Scheduler_TimeDiffUs(Bsp_GetMicros(), g_vol_next_start_us) < 0)
            {
                Bsp_WatchdogRefresh();
            }
            if (MotionControl_StartMove(APP_VOL_TEST_DEGREES, origin) != MOTION_START_OK)
            {
                Scheduler_StopVolumetric("scheduled movement could not start");
            }
            else
            {
                g_vol_last_start_error_us = Scheduler_TimeDiffUs(MotionControl_GetMoveStartUs(), g_vol_next_start_us);
                g_vol_timing_violation = (g_vol_last_start_error_us > (int32_t)APP_VOL_START_TOLERANCE_US) ||
                                         (g_vol_last_start_error_us < -(int32_t)APP_VOL_START_TOLERANCE_US);
                g_vol_state = VOL_MOVE_ACTIVE;
            }
        }
    }
}

/**
 * @brief Main-loop hook: run both schedulers.
 */
void Scheduler_Poll(void)
{
    Scheduler_PollRepeat();
    Scheduler_PollVolumetric();
}

/**
 * @brief Append the batch/move or cycle tag to the [RUN] line when a schedule is active.
 */
void Scheduler_PrintRunTag(void)
{
    if (g_vol_active)
    {
        Console_PrintString(" batch=");
        Console_PrintUint((uint32_t)g_vol_batch + 1U);
        Console_PrintString(" move=");
        Console_PrintUint((uint32_t)g_vol_move + 1U);
    }
    else if (g_repeat_active)
    {
        Console_PrintString(" cycle=");
        Console_PrintUint(g_repeat_index);
    }
    else
    {
        /* standalone move: no tag */
    }
}

/**
 * @brief Add the volumetric progress to the "s" status output.
 */
void Scheduler_PrintStatus(void)
{
    if (g_vol_active)
    {
        Console_PrintString("[STATUS] volumetric batch=");
        Console_PrintUint((uint32_t)g_vol_batch + 1U);
        Console_PrintString("/");
        Console_PrintUint(APP_VOL_BATCHES);
        Console_PrintString(" move=");
        Console_PrintUint((uint32_t)g_vol_move + 1U);
        Console_PrintString("/");
        Console_PrintUint(APP_VOL_MOVES_PER_BATCH);
        Console_PrintString(" completed=");
        Console_PrintUint(g_vol_completed);
        Console_PrintNewLine();
    }
}
