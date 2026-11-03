/**
 * @file    app_commands.c
 * @brief   Serial command interpreter implementation.
 */
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
#include "position_calc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define COMMANDS_PER_CALL_CHARS     (8U)
#define DT_MIN_PERCENT              (5.0F)
#define DT_MAX_PERCENT              (APP_DRIVER_HARD_LIMIT_PERCENT)
#define ANGLE_DUMP_SAMPLES          (10U)
#define ANGLE_DUMP_PERIOD_MS        (2U)

static char     g_command[APP_COMMAND_MAX_LENGTH];
static uint8_t  g_command_length = 0U;
static bool     g_discard_line = false;

/**
 * @brief Print the command list (the "?" command).
 */
void Commands_PrintHelp(void)
{
    Console_PrintLine("STM32U575 + DRV8411A + MA782 position control; forward-only moves; drive limits per motor profile");
    Console_PrintLine("mr      MA782 configuration/status");
    Console_PrintLine("dt25    optional: re-measure encoder sign; DETACH reservoir first; also re-arms after a fault");
    Console_PrintLine("pw40    bench: steady 2 s drive (pw-40 = reverse); prints travel, ADC raw and nFAULT");
    Console_PrintLine("an      print 10 raw MA782 angle readings (wiring/magnet check)");
    Console_PrintLine("ast     100 ms stationary check with motor disabled");
    Console_PrintLine("g10.61  one loaded qualification move; g-10.61 = explicit reverse move");
    Console_PrintLine("r10.61  10 relative moves, 4 s rest AFTER each completion; repeatability only");
    Console_PrintLine("test / vol  volumetric: 10 x 10.61 deg @4 s START-to-START, wait 180 s, 10 batches");
    Console_PrintLine("trace1 / trace0  buffered 10 ms trace, printed after the move");
    Console_PrintLine("x       immediate disable/abort; a move in progress latches a fault");
    Console_PrintLine("clear   fault acknowledge (needs stationary output and valid MA782 config)");
    Console_PrintLine("s       status; ? help");
}

/**
 * @brief Parse a command argument with strtof. Accepts only text that is entirely a finite number
 *         (rejects trailing characters, NaN and infinity).
 */
static bool Commands_ParseNumber(const char *text, float32_t *value)
{
    bool valid = false;

    if (text[0] != '\0')
    {
        char *end = NULL;
        const float32_t parsed = strtof(text, &end);

        if ((end != NULL) && (*end == '\0') && isfinite(parsed))
        {
            *value = parsed;
            valid = true;
        }
    }
    return valid;
}

/**
 * @brief Translate the status returned by AppMove_Start into a console message (APP_MOVE_OK prints nothing).
 */
static void Commands_PrintMoveBlocked(AppMoveStatus_t status)
{
    switch (status)
    {
        case APP_MOVE_NOT_READY:
            Console_PrintLine("[BLOCKED] hardware/config not ready");
            break;
        case APP_MOVE_FAULT_LATCHED:
            Console_PrintLine("[BLOCKED] fault latched; clear or dt first");
            break;
        case APP_MOVE_BUSY:
            Console_PrintLine("[BLOCKED] motion already active");
            break;
        case APP_MOVE_BAD_ARGUMENT:
            Console_PrintLine("[ERR] use g0.1..g360 (negative = reverse)");
            break;
        case APP_MOVE_ENCODER_CONFIG:
            Console_PrintLine("[FAULT] MA782 configuration/status invalid");
            break;
        case APP_MOVE_NOT_STATIONARY:
            Console_PrintLine("[FAULT] output not stationary before move");
            break;
        case APP_MOVE_OK:
        default:
            break;
    }
}

/**
 * @brief "s" command: one-screen status - ready/fault/scheduler flags, encoder sign, qualification,
 *         motion phase, progress/target (raw), measured current, nFAULT pin and any latched fault text.
 */
static void Commands_PrintStatus(void)
{
    const MotionPhase_t phase = MotionControl_GetPhase();

    Console_PrintString("[STATUS] ready=");
    Console_PrintUint(AppState_IsReady() ? 1U : 0U);
    Console_PrintString(" fault_latched=");
    Console_PrintUint(FaultManager_IsLatched() ? 1U : 0U);
    Console_PrintString(" repeat_active=");
    Console_PrintUint(Scheduler_IsRepeatActive() ? 1U : 0U);
    Console_PrintString(" volumetric_active=");
    Console_PrintUint(Scheduler_IsVolumetricActive() ? 1U : 0U);
    Console_PrintString(" encoder_invert=");
    Console_PrintUint(MotionControl_GetEncoderInvert() ? 1U : 0U);
    Console_PrintString(" qualified=");
    Console_PrintUint(AppState_IsQualified() ? 1U : 0U);
    Console_PrintString(" phase=");
    Console_PrintLine(MotionControl_GetPhaseName(phase));

    Console_PrintString("[STATUS] progress_raw=");
    Console_PrintInt(MotionControl_GetProgressRaw());
    Console_PrintString(" target_raw=");
    Console_PrintInt(MotionControl_GetTargetRaw());
    Console_PrintString(" current_mA=");
    Console_PrintFloat(MotionControl_GetCurrentMilliamps(), 1U);
    Console_PrintString(" nfault_pin=");
    Console_PrintUint(MotorDriver_IsFaultPinAsserted() ? 1U : 0U);
    Console_PrintNewLine();

    Scheduler_PrintStatus();
    if (FaultManager_IsLatched())
    {
        Console_PrintLine(FaultManager_GetText(FaultManager_GetCode()));
    }
}

/**
 * @brief "mr" command: read the MA782 FW/RD/STATUS registers and report PASS/CHECK.
 *         The control tick is suspended because the SPI bus is shared with the interrupt-driven sampler.
 */
static void Commands_PrintMa782(void)
{
    Ma782Config_t config;

    Bsp_ControlTickSuspend();
    (void)Ma782_ReadConfiguration(&config);
    Bsp_ControlTickResume();

    Console_PrintNewLine();
    Console_PrintLine("---------- MA782 CONFIG ----------");
    Console_PrintString("FW reg       = 0x");
    Console_PrintHex(config.fw_register);
    Console_PrintNewLine();
    Console_PrintString("FW           = ");
    Console_PrintUint(config.filter_window);
    Console_PrintNewLine();
    Console_PrintString("Required FW  = ");
    Console_PrintUint(APP_MA782_REQUIRED_FW);
    Console_PrintNewLine();
    Console_PrintString("RD           = ");
    Console_PrintUint(config.rotation_direction);
    Console_PrintNewLine();
    Console_PrintString("Status       = 0x");
    Console_PrintHex(config.status_register);
    Console_PrintNewLine();
    Console_PrintString("CONFIG       = ");
    Console_PrintLine(config.config_ok ? "PASS" : "CHECK");
    Console_PrintLine("----------------------------------");
    Console_PrintNewLine();
}

/**
 * @brief "x" command. During a move it latches a user-abort fault (bridge off). When idle it releases the
 *         bridge and cancels any repeat/volumetric schedule.
 */
static void Commands_HandleAbort(void)
{
    if (MotionControl_IsMoveActive())
    {
        MotionControl_AbortByUser();
    }
    else
    {
        MotionControl_Release();
        Scheduler_Cancel();
        Console_PrintLine("[STOP] disabled; repeat/volumetric schedule cancelled");
    }
}

/**
 * @brief Re-arm requirement shared by clear and dt: valid config and stationary output.
 */
static bool Commands_RearmChecksPass(void)
{
    uint16_t angle = 0U;

    return AppState_IsReady() && AppMove_CheckEncoderConfig() && MotionControl_MeasureStationary(&angle);
}

/**
 * @brief "clear" command: acknowledge a latched fault. Only accepted when the encoder configuration is valid,
 *         the output is stationary and (unless APP_IGNORE_DRIVER_FAULTS) nFAULT is released.
 */
static void Commands_HandleClear(void)
{
    if (Commands_RearmChecksPass() && FaultManager_Clear())
    {
        MotionControl_Release();
        Console_PrintLine("[CLEAR] accepted; prior successful loaded qualification retained");
    }
    else
    {
        Console_PrintLine("[CLEAR] rejected; require ready/config/stationary output and nFAULT released");
    }
}

/**
 * @brief "z" / "ast" command: 100 ms stationary check with the bridge off.
 *         Prints PASS/FAIL with min/max/span in raw counts and, on FAIL, the 50 individual samples.
 */
static void Commands_HandleStationary(void)
{
    uint16_t angle = 0U;
    const bool stable = MotionControl_MeasureStationary(&angle);

    Console_PrintString("[STATIONARY] ");
    Console_PrintString(stable ? "PASS" : "FAIL");
    {
        int32_t minimum;
        int32_t maximum;

        MotionControl_GetLastStationarySpan(&minimum, &maximum);
        Console_PrintString(" min=");
        Console_PrintInt(minimum);
        Console_PrintString(" max=");
        Console_PrintInt(maximum);
        Console_PrintString(" span=");
        Console_PrintInt(maximum - minimum);
        Console_PrintString(" limit=");
        Console_PrintInt(APP_STABLE_SPAN_RAW);
        Console_PrintString(" angle=");
        Console_PrintUint(angle);
        Console_PrintNewLine();
        if (!stable)
        {
            uint32_t index;

            Console_PrintString("[STATIONARY] samples(2ms):");
            for (index = 0U; index < APP_STATIONARY_SAMPLES; ++index)
            {
                Console_PrintString(" ");
                Console_PrintInt(MotionControl_GetLastStationarySample(index));
            }
            Console_PrintNewLine();
        }
    }
}

/**
 * @brief Bench test: steady drive for 2 s (pw40 = forward, pw-40 = reverse). Measure motor volts/current
 *        with a meter and read the raw ADC, nFAULT and shaft travel printed every 250 ms.
 */
static void Commands_HandleBenchDrive(float32_t percent)
{
    const float32_t magnitude = fabsf(percent);
    uint32_t step;
    int32_t travel = 0;
    uint16_t previous;

    if (MotionControl_IsMoveActive() || (magnitude < 1.0F) || (magnitude > APP_DRIVER_HARD_LIMIT_PERCENT))
    {
        Console_PrintLine("[ERR] pw needs idle state, 1..100 (negative = reverse)");
        return;
    }

    Bsp_ControlTickSuspend();
    MotorDriver_Stop();
    previous = Ma782_ReadAngleRaw();
    Console_PrintLine("[PW] driving 2 s; measure motor terminals now");
    if (percent > 0.0F)
    {
        MotorDriver_RunForward(magnitude);
    }
    else
    {
        MotorDriver_RunReverse(magnitude);
    }

    for (step = 0U; step < 8U; ++step)
    {
        uint32_t index;

        for (index = 0U; index < 125U; ++index)
        {
            const uint16_t angle = Ma782_ReadAngleRaw();

            travel += PositionCalc_AngleDelta(angle, previous);
            previous = angle;
            Bsp_DelayUs(2000U);
            Bsp_WatchdogRefresh();
        }
        Console_PrintString("[PW] t_ms=");
        Console_PrintUint((step + 1U) * 250U);
        Console_PrintString(" travel_raw=");
        Console_PrintInt(travel);
        Console_PrintString(" adc_raw=");
        Console_PrintUint(Bsp_AdcReadCurrentRaw());
        Console_PrintString(" nfault=");
        Console_PrintUint(MotorDriver_IsFaultPinAsserted() ? 1U : 0U);
        Console_PrintNewLine();
    }

    MotorDriver_Coast();
    Bsp_ControlTickResume();
    Console_PrintLine("[PW] done");
}

/**
 * @brief "dt<percent>" command: measure the encoder sign. Drives forward for at most 80 ms, brakes, lets the
 *         output settle and sets encoder_invert from the sign of the observed travel (negative = inverted).
 *         On failure prints the observed delta, jump/settled flags and the driver nFAULT/current seen during the pulse.
 *         Also re-arms (clears a latched fault) first if the re-arm checks pass.
 */
static void Commands_HandleDirectionTest(float32_t percent)
{
    bool allowed = true;
    int32_t observed = 0;

    if (FaultManager_IsLatched())
    {
        if (Commands_RearmChecksPass() && FaultManager_Clear())
        {
            MotionControl_Release();
        }
        else
        {
            Console_PrintLine("[DT] BLOCKED - cannot re-arm (config/stationary/nFAULT)");
            allowed = false;
        }
    }

    if (allowed)
    {
        Console_PrintLine("[DT] detached-reservoir direction check: delivery direction, max 80 ms");
        if (MotionControl_CalibrateDirection(percent, &observed))
        {
            Console_PrintString("[DT] PASS encoder_invert=");
            Console_PrintUint(MotionControl_GetEncoderInvert() ? 1U : 0U);
            Console_PrintString(" observed_raw_delta=");
            Console_PrintInt(observed);
            Console_PrintNewLine();
        }
        else
        {
            bool jump;
            bool settled;
            bool nfault;
            float32_t peak_ma;

            MotionControl_GetCalibrationDiag(&jump, &settled, &nfault, &peak_ma);
            Console_PrintString("[DT] observed_raw_delta=");
            Console_PrintInt(observed);
            Console_PrintString(" jump=");
            Console_PrintUint(jump ? 1U : 0U);
            Console_PrintString(" settled=");
            Console_PrintUint(settled ? 1U : 0U);
            Console_PrintString(" nfault_during_drive=");
            Console_PrintUint(nfault ? 1U : 0U);
            Console_PrintString(" peak_mA=");
            Console_PrintFloat(peak_ma, 1U);
            Console_PrintNewLine();
            Console_PrintString("[FAULT] ");
            Console_PrintLine(FaultManager_GetText(FaultManager_GetCode()));
        }
    }
}

/**
 * @brief Bring-up aid: print 10 raw MA782 angle readings (motor must be idle).
 */
static void Commands_HandleAngle(void)
{
    uint16_t samples[ANGLE_DUMP_SAMPLES];
    uint32_t index;
    int32_t minimum = 0;
    int32_t maximum = 0;

    Bsp_ControlTickSuspend();
    for (index = 0U; index < ANGLE_DUMP_SAMPLES; ++index)
    {
        samples[index] = Ma782_ReadAngleRaw();
        Bsp_DelayMs(ANGLE_DUMP_PERIOD_MS);
    }
    Bsp_ControlTickResume();

    Console_PrintString("[ANGLE] raw:");
    for (index = 0U; index < ANGLE_DUMP_SAMPLES; ++index)
    {
        const int32_t delta = PositionCalc_AngleDelta(samples[index], samples[0]);

        Console_PrintString(" ");
        Console_PrintUint(samples[index]);
        if (delta < minimum)
        {
            minimum = delta;
        }
        if (delta > maximum)
        {
            maximum = delta;
        }
    }
    Console_PrintString("  span=");
    Console_PrintInt(maximum - minimum);
    Console_PrintString(" (limit ");
    Console_PrintInt(APP_STABLE_SPAN_RAW);
    Console_PrintLine(")");
}

/**
 * @brief "g<degrees>" command: start one relative move (negative = explicit reverse move).
 */
static void Commands_HandleMove(float32_t degrees)
{
    const AppMoveStatus_t status = AppMove_Start(degrees);

    Commands_PrintMoveBlocked(status);
}

/**
 * @brief "r<degrees>" command: start the 10-cycle repeatability test. Requires a successful loaded g10.61 first.
 */
static void Commands_HandleRepeat(float32_t degrees)
{
    if (FaultManager_IsLatched() || (!AppState_IsQualified()) ||
        (fabsf(degrees) < APP_MOVE_MIN_DEGREES) || (fabsf(degrees) > APP_MOVE_MAX_DEGREES))
    {
        Console_PrintLine("[BLOCKED] require prior loaded qualification, valid angle, no fault");
    }
    else
    {
        (void)Scheduler_StartRepeat(degrees);
    }
}

/**
 * @brief Route one complete command line to its handler. While a repeat or volumetric schedule runs only
 *         "s" and "?" are accepted ("x" is handled earlier, character by character, so it works mid-move).
 */
static void Commands_Dispatch(const char *command)
{
    float32_t value = 0.0F;

    if (strcmp(command, "?") == 0)
    {
        Commands_PrintHelp();
    }
    else if (strcmp(command, "s") == 0)
    {
        Commands_PrintStatus();
    }
    else if (Scheduler_IsVolumetricActive())
    {
        Console_PrintLine("[BUSY] x stops volumetric schedule");
    }
    else if (Scheduler_IsRepeatActive())
    {
        Console_PrintLine("[BUSY] x stops repeatability");
    }
    else if (strcmp(command, "mr") == 0)
    {
        Commands_PrintMa782();
    }
    else if (strcmp(command, "trace1") == 0)
    {
        MotionControl_SetTraceEnabled(true);
        Console_PrintLine("[TRACE] ON");
    }
    else if (strcmp(command, "trace0") == 0)
    {
        MotionControl_SetTraceEnabled(false);
        Console_PrintLine("[TRACE] OFF");
    }
    else if (strcmp(command, "clear") == 0)
    {
        Commands_HandleClear();
    }
    else if ((strcmp(command, "an") == 0) && (!MotionControl_IsMoveActive()))
    {
        Commands_HandleAngle();
    }
    else if ((strcmp(command, "ast") == 0) || (strcmp(command, "z") == 0))
    {
        Commands_HandleStationary();
    }
    else if ((strcmp(command, "test") == 0) || (strcmp(command, "vol") == 0))
    {
        (void)Scheduler_StartVolumetric();
    }
    else if ((strncmp(command, "pw", 2U) == 0) && Commands_ParseNumber(&command[2], &value))
    {
        Commands_HandleBenchDrive(value);
    }
    else if ((strncmp(command, "dt", 2U) == 0) && Commands_ParseNumber(&command[2], &value))
    {
        if ((value < DT_MIN_PERCENT) || (value > DT_MAX_PERCENT) || ((value - floorf(value)) > 0.0F))
        {
            Console_PrintLine("[ERR] dt integer 5..max drive");
        }
        else
        {
            Commands_HandleDirectionTest(value);
        }
    }
    else if ((command[0] == 'g') && Commands_ParseNumber(&command[1], &value))
    {
        Commands_HandleMove(value);
    }
    else if ((command[0] == 'r') && Commands_ParseNumber(&command[1], &value))
    {
        Commands_HandleRepeat(value);
    }
    else
    {
        Console_PrintLine("[ERR] unknown/invalid command; ? for help");
    }
}

/**
 * @brief Poll the serial port. At most a few characters are processed per call so
 *        that console traffic cannot starve the rest of the main loop.
 */
void Commands_Poll(void)
{
    uint8_t count;
    char character;

    for (count = 0U; (count < COMMANDS_PER_CALL_CHARS) && Console_ReadChar(&character); ++count)
    {
        if ((character == 'x') || (character == 'X'))
        {
            Commands_HandleAbort();
            g_command_length = 0U;
            g_discard_line = true;
        }
        else if (character == '\r')
        {
            /* ignored */
        }
        else if (character == '\n')
        {
            if ((!g_discard_line) && (g_command_length > 0U))
            {
                g_command[g_command_length] = '\0';
                if (!MotionControl_IsMoveActive())
                {
                    Commands_Dispatch(g_command);
                }
            }
            g_command_length = 0U;
            g_discard_line = false;
        }
        else if (MotionControl_IsMoveActive() || g_discard_line)
        {
            /* Never parse or print during motion. */
            g_discard_line = true;
        }
        else
        {
            char lowered = character;

            if ((lowered >= 'A') && (lowered <= 'Z'))
            {
                lowered = (char)(lowered - 'A' + 'a');
            }
            if (g_command_length >= (APP_COMMAND_MAX_LENGTH - 1U))
            {
                g_discard_line = true;
                g_command_length = 0U;
            }
            else
            {
                g_command[g_command_length] = lowered;
                ++g_command_length;
            }
        }
    }
}
