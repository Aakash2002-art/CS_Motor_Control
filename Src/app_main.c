/**
 * @file    app_main.c
 * @brief   Application initialisation and main loop.
 *
 * Execution model:
 *   - TIM6 interrupt (2 kHz)  : MotionControl_Tick() - encoder sampling and control
 *   - EXTI13 interrupt        : DRV8411A nFAULT latch
 *   - main loop               : commands, result reporting, scheduler, watchdog
 * Nothing is printed while a move is active.
 */
#include "app_main.h"
#include "app_commands.h"
#include "app_config.h"
#include "app_scheduler.h"
#include "app_state.h"
#include "bsp_hw.h"
#include "console.h"
#include "fault_manager.h"
#include "ma782_encoder.h"
#include "motion_control.h"
#include "motor_driver.h"

static bool     g_fault_reported = false;
static uint32_t g_last_heartbeat = 0U;

/**
 * @brief TIM6 2 kHz interrupt callback: run one motion-control step (encoder sample, limits, phase logic).
 */
static void App_ControlTick(void)
{
    MotionControl_Tick();
}

/**
 * @brief Print the buffered per-move trace (ms, progress in raw counts, phase, drive %). Printed only after the
 *         move so that console output never delays the 2 kHz control loop.
 */
static void App_PrintTrace(void)
{
    uint16_t index;
    TraceEntry_t entry;

    for (index = 0U; MotionControl_GetTraceEntry(index, &entry); ++index)
    {
        Console_PrintString("[TRACE] ms=");
        Console_PrintUint(entry.ms);
        Console_PrintString(" raw=");
        Console_PrintInt(entry.raw);
        Console_PrintString(" phase=");
        Console_PrintString(MotionControl_GetPhaseName((MotionPhase_t)entry.phase));
        Console_PrintString(" drive=");
        Console_PrintUint(entry.duty_percent);
        Console_PrintNewLine();
        Bsp_WatchdogRefresh();
    }
    if (MotionControl_IsTraceFull())
    {
        Console_PrintLine("[TRACE] buffer full; later samples dropped");
    }
}

/**
 * @brief Compact per-run line: run identity, elapsed time and final rounded encoder count.
 */
static void App_PrintResult(const MoveResult_t *result)
{
    Console_PrintString("[RUN]");
    Scheduler_PrintRunTag();
    Console_PrintString(" time_ms=");
    Console_PrintUint(result->elapsed_ms);
    Console_PrintString(" final_encoder_count=");
    Console_PrintInt(result->final_count);
    Console_PrintNewLine();

    if (!result->passed)
    {
        Console_PrintString("[FAULT] ");
        Console_PrintLine(FaultManager_GetText(result->fault));
        g_fault_reported = true;
    }
    if (MotionControl_IsTraceEnabled())
    {
        App_PrintTrace();
    }
}

/**
 * @brief If a move just finished (or faulted), print its result line and tell the scheduler.
 */
static void App_HandleResult(void)
{
    MoveResult_t result;

    if (MotionControl_TakeResult(&result))
    {
        App_PrintResult(&result);
        Scheduler_OnMoveResult(&result);
    }
}

/**
 * @brief Report a fault that was latched outside a move (hold, idle, nFAULT).
 */
static void App_ReportIdleFault(void)
{
    if (FaultManager_IsLatched())
    {
        const MotionPhase_t phase = MotionControl_GetPhase();
        const bool tick_processed = (phase == MOTION_PHASE_FAULTED) || (phase == MOTION_PHASE_IDLE);

        if ((!g_fault_reported) && tick_processed && (!MotionControl_IsResultPending()))
        {
            Console_PrintString("[FAULT] ");
            Console_PrintLine(FaultManager_GetText(FaultManager_GetCode()));
            g_fault_reported = true;
        }
    }
    else
    {
        g_fault_reported = false;
    }
}

/**
 * @brief The independent watchdog is only fed while the control tick is alive.
 */
static void App_ServiceWatchdog(void)
{
    const uint32_t heartbeat = MotionControl_GetTickCounter();

    if (heartbeat != g_last_heartbeat)
    {
        g_last_heartbeat = heartbeat;
        Bsp_WatchdogRefresh();
    }
}

/**
 * @brief One-time start-up: board peripherals, drivers, fault manager, motion control; read and force the MA782
 *         configuration (FW=12, RD=0, volatile only); mark the system ready if it passes; start the control tick and watchdog.
 */
void App_Init(void)
{
    Ma782Config_t config;

    Bsp_Init();
    MotorDriver_Init();
    FaultManager_Init();
    MotionControl_Init();
    Scheduler_Init();
    Bsp_SetControlTickCallback(App_ControlTick);
    Bsp_SetFaultCallback(FaultManager_OnDriverFaultInterrupt);

    Console_PrintNewLine();
    Console_PrintLine("STM32U575 + DRV8411A + MA782 V1.0 POSITION CONTROL (forward-only, +/-1 virtual count)");

    /* Configure and verify the MA782 before the control tick starts using the SPI bus. */
    Ma782_ConfigureVolatile();
    Bsp_DelayMs(20U);
    AppState_SetReady(Ma782_ReadConfiguration(&config));
    Console_PrintString("[MA782-CONFIG] FW=");
    Console_PrintUint(config.filter_window);
    Console_PrintString(" required=");
    Console_PrintUint(APP_MA782_REQUIRED_FW);
    Console_PrintString(" RD=");
    Console_PrintUint(config.rotation_direction);
    Console_PrintString(" status=0x");
    Console_PrintHex(config.status_register);
    Console_PrintString(" -> ");
    Console_PrintLine(config.config_ok ? "PASS" : "CHECK");

    Bsp_WatchdogStart();
    Bsp_ControlTickStart();

    if (!AppState_IsReady())
    {
        FaultManager_Latch(FAULT_ENCODER_CONFIG);
    }
    FaultManager_CheckDriverFault();
    Commands_PrintHelp();
}

/**
 * @brief Main loop (never returns): feed the watchdog while the control tick is alive, poll the serial commands,
 *         report finished moves, run the repeat/volumetric scheduler and report faults latched outside a move.
 */
void App_Run(void)
{
    for (;;)
    {
        App_ServiceWatchdog();
        Commands_Poll();
        App_HandleResult();
        Scheduler_Poll();
        App_ReportIdleFault();
    }
}
