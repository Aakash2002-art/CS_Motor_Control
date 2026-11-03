/**
 * @file    motor_driver.c
 * @brief   DRV8411A bridge-A drive layer implementation.
 */
#include "motor_driver.h"
#include "app_config.h"
#include "bsp_hw.h"

/* ------------------------------------------------------------------------- */
/* Module state                                                               */
/* ------------------------------------------------------------------------- */
static MotorState_t   g_state = MOTOR_STATE_COAST;
static float32_t      g_duty_percent = 0.0F;
static uint32_t       g_coast_since_us = 0U;
static volatile bool  g_fault_flag = false;

/* ------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Clamp a duty request to the allowed range. NaN maps to zero.
 */
static float32_t MotorDriver_LimitDuty(float32_t duty_percent)
{
    float32_t limited = duty_percent;

    if (!(limited > 0.0F))
    {
        limited = 0.0F;
    }
    else if (limited > APP_DRIVER_HARD_LIMIT_PERCENT)
    {
        limited = APP_DRIVER_HARD_LIMIT_PERCENT;
    }
    else
    {
        /* value already within range */
    }
    return limited;
}

/**
 * @brief Write both AIN compare values, honouring the output-swap option.
 */
static void MotorDriver_WriteInputs(uint32_t ain1_compare, uint32_t ain2_compare)
{
#if (APP_COMPLEMENTARY_DRIVE != 0)
    /* AIN1 = PWM1 (high while CNT < c), AIN2 = PWM2 (high while CNT >= c).
       Only the coast (0,0) and brake (period,period) patterns come through here. */
    const uint32_t period = Bsp_PwmGetPeriodCounts();

    if ((ain1_compare == 0U) && (ain2_compare == 0U))
    {
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN1, 0U);        /* AIN1 low */
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN2, period);    /* AIN2 low : coast */
    }
    else
    {
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN1, period);    /* AIN1 high */
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN2, 0U);        /* AIN2 high : brake */
    }
#else
    if (APP_MOTOR_SWAP_OUTPUTS)
    {
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN1, ain2_compare);
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN2, ain1_compare);
    }
    else
    {
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN1, ain1_compare);
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN2, ain2_compare);
    }
#endif
}
/**
 * @brief After a long coast the DRV8411A is in autosleep. Wait for it to wake
 *        once the inputs have been raised, before any timing-critical drive.
 */
static void MotorDriver_WakeIfNeeded(MotorState_t previous_state)
{
    if (previous_state == MOTOR_STATE_COAST)
    {
        if ((Bsp_GetMicros() - g_coast_since_us) >= APP_DRIVER_SLEEP_ENTRY_US)
        {
            Bsp_DelayUs(APP_DRIVER_WAKE_DELAY_US);
        }
    }
}

/**
 * @brief Slow-decay drive in one direction.
 */
static void MotorDriver_Drive(MotorState_t direction, float32_t duty_percent)
{
    const MotorState_t previous_state = g_state;
    const uint32_t period = Bsp_PwmGetPeriodCounts();
    const float32_t duty = MotorDriver_LimitDuty(duty_percent);
    const uint32_t on_counts = (uint32_t)((((float32_t)period) * duty / 100.0F) + 0.5F);
    uint32_t pwm_compare = period - on_counts;

    if (pwm_compare > period)
    {
        pwm_compare = period;
    }

#if (APP_COMPLEMENTARY_DRIVE != 0)
    {
        /* Differential duty d: AIN1 high for (0.5 + d/2) of the period, AIN2 is its inverse.
           duty 0 = 50 % chop with zero net voltage, duty 100 = AIN1 always high. */
        const bool forward = (direction == MOTOR_STATE_FORWARD) != (APP_MOTOR_SWAP_OUTPUTS != 0);
        const uint32_t compare = forward ? ((period / 2U) + (on_counts / 2U))
                                         : ((period / 2U) - (on_counts / 2U));

        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN1, compare);
        Bsp_PwmSetCompare(BSP_PWM_CHANNEL_AIN2, compare);
    }
#else
    if (on_counts == 0U)
    {
        /* Zero demand must not brake: slow-decay with no on-time shorts the motor and stops it dead,
           which made the speed loop stop-and-go. Free-wheel instead. */
        MotorDriver_Coast();
        return;
    }
    if (direction == MOTOR_STATE_FORWARD)
    {
        MotorDriver_WriteInputs(period, pwm_compare);
    }
    else
    {
        MotorDriver_WriteInputs(pwm_compare, period);
    }
#endif

    g_state = direction;
    g_duty_percent = duty;
    MotorDriver_WakeIfNeeded(previous_state);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void MotorDriver_Init(void)
{
    MotorDriver_WriteInputs(0U, 0U);
    g_state = MOTOR_STATE_COAST;
    g_duty_percent = 0.0F;
    g_coast_since_us = Bsp_GetMicros() - APP_DRIVER_SLEEP_ENTRY_US;
    g_fault_flag = false;
}

/**
 * @brief Drive forward (delivery direction) at the given duty, percent of VM.
 */
void MotorDriver_RunForward(float32_t duty_percent)
{
    MotorDriver_Drive(MOTOR_STATE_FORWARD, duty_percent);
}

/**
 * @brief Drive reverse at the given duty, percent of VM.
 */
void MotorDriver_RunReverse(float32_t duty_percent)
{
    MotorDriver_Drive(MOTOR_STATE_REVERSE, duty_percent);
}

/**
 * @brief Active brake: both bridge inputs high (motor terminals shorted through the low-side FETs).
 */
void MotorDriver_Brake(void)
{
    const uint32_t period = Bsp_PwmGetPeriodCounts();
    const MotorState_t previous_state = g_state;

    MotorDriver_WriteInputs(period, period);
    g_state = MOTOR_STATE_BRAKE;
    g_duty_percent = 0.0F;
    MotorDriver_WakeIfNeeded(previous_state);
}

/**
 * @brief Free-wheel: both inputs low (the driver enters autosleep after a short time). Records the coast start time.
 */
void MotorDriver_Coast(void)
{
    if (g_state != MOTOR_STATE_COAST)
    {
        MotorDriver_WriteInputs(0U, 0U);
        g_state = MOTOR_STATE_COAST;
        g_duty_percent = 0.0F;
        g_coast_since_us = Bsp_GetMicros();
    }
}

/**
 * @brief Disable the bridge (same as coast). Safe to call from interrupt context.
 */
void MotorDriver_Stop(void)
{
    MotorDriver_Coast();
}

/**
 * @brief Current bridge state (forward/reverse/brake/coast).
 */
MotorState_t MotorDriver_GetState(void)
{
    return g_state;
}

/**
 * @brief Last applied duty in percent.
 */
float32_t MotorDriver_GetDutyPercent(void)
{
    return g_duty_percent;
}

/**
 * @brief Record that nFAULT fell (called from the EXTI handler).
 */
void MotorDriver_NotifyFaultEdge(void)
{
    g_fault_flag = true;
}

/**
 * @brief True if a nFAULT edge has been seen since the flag was cleared.
 */
bool MotorDriver_IsFaultFlagSet(void)
{
    return g_fault_flag;
}

/**
 * @brief True while nFAULT is held low.
 */
bool MotorDriver_IsFaultPinAsserted(void)
{
    return Bsp_IsNFaultAsserted();
}

/**
 * @brief Clear the nFAULT edge flag.
 */
void MotorDriver_ClearFaultFlag(void)
{
    g_fault_flag = false;
}

/**
 * @brief Motor current from the IPROPI mirror: raw ADC -> volts -> / sense resistor -> / mirror ratio.
 *         Depends on APP_IPROPI_SENSE_OHMS matching the EVM; the value has not been verified on this hardware.
 */
float32_t MotorDriver_ReadCurrentMilliamps(void)
{
    const float32_t raw = (float32_t)Bsp_AdcReadCurrentRaw();
    const float32_t volts = (raw / APP_ADC_FULL_SCALE) * APP_ADC_REFERENCE_VOLTS;
    const float32_t mirror_amps = volts / APP_IPROPI_SENSE_OHMS;

    return (mirror_amps / APP_IPROPI_AMPS_PER_AMP) * 1000.0F;
}

/**
 * @brief Expected stall current at a given duty: duty/100 * VM / R (used by the stall detector).
 */
float32_t MotorDriver_ExpectedStallMilliamps(float32_t duty_percent)
{
    const float32_t duty = MotorDriver_LimitDuty(duty_percent);

    return (duty / 100.0F) * (APP_MOTOR_SUPPLY_VOLTS / APP_MOTOR_RESISTANCE_OHMS) * 1000.0F;
}
