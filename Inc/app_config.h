/**
 * @file    app_config.h
 * @brief   Central configuration of geometry, timing, control and protection constants.
 *
 * HOW THIS FILE IS ORGANISED
 *   1. Motor profile (top): pick the motor with APP_MOTOR_SELECT. Every value that depends on the
 *      motor (supply voltage, drive limits, PI gains, speed profile, brake/stop settings, ...)
 *      is defined once per motor as MP_APP_<name> and mapped to APP_<name> further down.
 *   2. Everything else is common to all motors (geometry, timing, scheduler, MA782 settings).
 *   Switches: APP_COMPLEMENTARY_DRIVE (bridge drive scheme), APP_IGNORE_DRIVER_FAULTS (bring-up).
 *   Full description: docs/FIRMWARE_GUIDE.md.
 *
 * Values marked "INITIAL TUNING VALUE" are starting points derived from the
 * validated NodeMCU/TMC4671 firmware V4.5 and the motor datasheet. They are NOT
 * final: they must be tuned on the real reservoir/plunger assembly.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "app_types.h"

/* ------------------------------------------------------------------------- */
/* Motor profile: select ONE motor. Everything motor-dependent comes from here */
/* ------------------------------------------------------------------------- */
#define APP_MOTOR_CONSTAR       (1)   /**< Constar 3.0 V, 3.5:1 gearbox, MA782 on output (original NodeMCU/TMC4671 tuning) */
#define APP_MOTOR_MAXON_DCX08M  (2)   /**< Maxon DCX08M EB KL 2.4 V, GPX08 4:1 gearhead, MA782 on output                   */
#define APP_MOTOR_SELECT        APP_MOTOR_CONSTAR

#if (APP_MOTOR_SELECT == APP_MOTOR_CONSTAR)
#define MP_APP_MOTOR_SUPPLY_VOLTS           (3.0F)
#define MP_APP_MOTOR_RESISTANCE_OHMS        (13.6F)   /* 3.0 V / 220 mA stall (datasheet) */
#define MP_APP_OVERCURRENT_MA               (300.0F)
#define MP_APP_ENCODER_INVERT_DEFAULT       (true)    /* measured on STM32: dt PASS encoder_invert=1 */
#define MP_APP_MAX_DRIVE_PERCENT            (25.0F)
#define MP_APP_START_DRIVE_PERCENT          (12.0F)
#define MP_APP_LOW_SPEED_RECOVERY_DRIVE     (16.0F)
#define MP_APP_PULSE_DRIVE_PERCENT          (25.0F)
#define MP_APP_VELOCITY_KI                  (0.150F)
#define MP_APP_APPROACH_CPS                 (300.0F)
#define MP_APP_STOP_PREDICTION_S            (0.005F)
#define MP_APP_BRAKE_MIN_LEAD_COUNTS        (1.0F)
#define MP_APP_REVERSE_LIMIT_RAW            (24)
#define MP_APP_STOPPED_SHORT_LIMIT_COUNTS   (32)
/* Plant measured from pw runs (Constar, 3 V): about 113 counts/s per % duty above a ~8 % breakaway offset. */
#define MP_APP_VELOCITY_KP                  (0.004F)
#define MP_APP_POSITION_KP                  (20.0F)
#define MP_APP_CRUISE_CPS                   (800.0F)
#define MP_APP_DECEL_CPS2                   (8000.0F)
#define MP_APP_APPROACH_ZONE_COUNTS         (20.0F)
#define MP_APP_VERIFY_US                    (50000U)
#define MP_APP_BREAKAWAY_DRIVE_PERCENT      (20.0F)   /* kick until the first motion; measured breakaway about 16-18 % */
#elif (APP_MOTOR_SELECT == APP_MOTOR_MAXON_DCX08M)
#define MP_APP_MOTOR_SUPPLY_VOLTS           (2.4F)
#define MP_APP_MOTOR_RESISTANCE_OHMS        (4.13F)   /* datasheet; stall 581 mA */
#define MP_APP_OVERCURRENT_MA               (700.0F)
#define MP_APP_ENCODER_INVERT_DEFAULT       (true)    /* measured: every dt pass gave encoder_invert=1 */
#define MP_APP_MAX_DRIVE_PERCENT            (70.0F)
#define MP_APP_START_DRIVE_PERCENT          (24.0F)
#define MP_APP_LOW_SPEED_RECOVERY_DRIVE     (32.0F)
#define MP_APP_PULSE_DRIVE_PERCENT          (100.0F)
#define MP_APP_VELOCITY_KI                  (0.300F)
#define MP_APP_APPROACH_CPS                 (60.0F)
#define MP_APP_STOP_PREDICTION_S            (0.025F)
#define MP_APP_BRAKE_MIN_LEAD_COUNTS        (25.0F)
#define MP_APP_REVERSE_LIMIT_RAW            (400)     /* gearhead backlash about 330 raw */
#define MP_APP_STOPPED_SHORT_LIMIT_COUNTS   (80)
#define MP_APP_VELOCITY_KP                  (0.025F)
#define MP_APP_POSITION_KP                  (8.0F)
#define MP_APP_CRUISE_CPS                   (300.0F)
#define MP_APP_DECEL_CPS2                   (1500.0F)
#define MP_APP_APPROACH_ZONE_COUNTS         (30.0F)
#define MP_APP_VERIFY_US                    (100000U)
#define MP_APP_BREAKAWAY_DRIVE_PERCENT      (24.0F)
#else
#error "APP_MOTOR_SELECT must be APP_MOTOR_CONSTAR or APP_MOTOR_MAXON_DCX08M"
#endif

/* ------------------------------------------------------------------------- */
/* Encoder geometry (MA782 sits on the FINAL gearbox output)                  */
/* ------------------------------------------------------------------------- */
#define APP_ENCODER_RAW_PER_REV         (65536)   /**< 16-bit native angle counts per output revolution */
#define APP_ENCODER_RAW_PER_COUNT       (16)      /**< native counts per virtual count (4096 per rev)   */
#define APP_ENCODER_COUNTS_PER_REV      (4096)    /**< virtual counts per output revolution             */
#define APP_ENCODER_MAX_STEP_RAW        (512)     /**< larger one-sample step = implausible jump        */
#define APP_ENCODER_INVERT_DEFAULT      (MP_APP_ENCODER_INVERT_DEFAULT)   /**< from the motor profile */
#define APP_FULL_REV_DEGREES            (360.0F)

/* ------------------------------------------------------------------------- */
/* Position acceptance (+/-1 virtual count, evaluated in the native domain)   */
/* ------------------------------------------------------------------------- */
#define APP_ACCEPT_COUNTS               (1)
#define APP_ACCEPT_RAW                  (APP_ENCODER_RAW_PER_COUNT * APP_ACCEPT_COUNTS)
#define APP_REVERSE_LIMIT_RAW           (MP_APP_REVERSE_LIMIT_RAW)   /**< from the motor profile */
#define APP_STABLE_SPAN_RAW             (12)      /**< max span of the verify window                    */

/** BRING-UP ONLY: 1 = ignore DRV8411A nFAULT and IPROPI current faults (motion/encoder checks stay). Set 0 to restore. */
#define APP_IGNORE_DRIVER_FAULTS        (1)
#define APP_STOPPED_SHORT_LIMIT_COUNTS  (MP_APP_STOPPED_SHORT_LIMIT_COUNTS)   /**< from the motor profile */
/* ------------------------------------------------------------------------- */
/* Timing                                                                     */
/* ------------------------------------------------------------------------- */
#define APP_CONTROL_TICK_US             (500U)       /**< control tick period (2 kHz)                   */
#define APP_SPEED_LOOP_PERIOD_US        (8000U)      /**< velocity/position PI period                   */
#define APP_MAX_SAMPLE_GAP_US           (5000U)      /**< feedback service gap fault                    */
#define APP_BRAKE_US                    (12000U)     /**< active brake duration                         */
#define APP_SETTLE_US                   (20000U)     /**< driver-disabled (coast) settling              */
#define APP_VERIFY_US                   (MP_APP_VERIFY_US)    /**< verify window                                 */
#define APP_MOVE_TIMEOUT_US             (30000000UL) /**< move timeout                                  */
#define APP_STALL_US                    (1200000UL)  /**< no-progress watchdog                          */
#define APP_STALL_FAST_US               (400000UL)   /**< no-progress + current present = stall         */
#define APP_STATIONARY_PREWAIT_MS       (20U)
#define APP_STATIONARY_SAMPLES          (50U)
#define APP_STATIONARY_SAMPLE_MS        (2U)

/* ------------------------------------------------------------------------- */
/* Drive limits (percent of the motor supply voltage, duty cycle)             */
/* ------------------------------------------------------------------------- */
/* Drive limits come from the motor profile at the top of this file. */
#define APP_MAX_DRIVE_PERCENT           (MP_APP_MAX_DRIVE_PERCENT)   /**< from the motor profile */
#define APP_BREAKAWAY_DRIVE_PERCENT     (MP_APP_BREAKAWAY_DRIVE_PERCENT)   /**< initial kick until motion starts */
#define APP_BREAKAWAY_DETECT_RAW        (6)       /**< progress that counts as breakaway                */
#define APP_START_DRIVE_PERCENT         (MP_APP_START_DRIVE_PERCENT)   /**< from the motor profile */
#define APP_LOW_SPEED_RECOVERY_DRIVE    (MP_APP_LOW_SPEED_RECOVERY_DRIVE)   /**< from the motor profile */
#define APP_LOW_SPEED_RECOVERY_CPS      (5.0F)

/* ------------------------------------------------------------------------- */
/* Velocity profile (virtual counts per second)                               */
/* ------------------------------------------------------------------------- */
#define APP_CRUISE_CPS                  (MP_APP_CRUISE_CPS)
#define APP_APPROACH_CPS                (MP_APP_APPROACH_CPS)   /**< from the motor profile */
#define APP_MIN_LOADED_APPROACH_CPS     (30.0F)
#define APP_DECEL_CPS2                  (MP_APP_DECEL_CPS2)
#define APP_STOP_PREDICTION_S           (MP_APP_STOP_PREDICTION_S)   /**< from the motor profile */
#define APP_PROFILE_STOP_MARGIN_COUNTS  (3.0F)
#define APP_APPROACH_ZONE_COUNTS        (MP_APP_APPROACH_ZONE_COUNTS)
#define APP_BRAKE_MIN_LEAD_COUNTS       (MP_APP_BRAKE_MIN_LEAD_COUNTS)   /**< from the motor profile */
/* ------------------------------------------------------------------------- */
/* PI gains. INITIAL TUNING VALUES - tune on the loaded mechanism.            */
/* ------------------------------------------------------------------------- */
#define APP_VELOCITY_KP                 (MP_APP_VELOCITY_KP)  /**< percent duty per (count/s)                       */
#define APP_VELOCITY_KI                 (MP_APP_VELOCITY_KI)   /**< from the motor profile */
#define APP_POSITION_KP                 (MP_APP_POSITION_KP)    /**< (count/s) per count of position error            */
#define APP_POSITION_KI                 (2.0F)    /**< (count/s) per (count*s)                          */
#define APP_POSITION_INTEGRAL_LIMIT     (APP_APPROACH_CPS)

/* ------------------------------------------------------------------------- */
/* Forward-only catch-up pulses (no reverse drive is ever used to correct)    */
/* ------------------------------------------------------------------------- */
#define APP_DRIVER_HARD_LIMIT_PERCENT   (100.0F)  /**< absolute driver clamp; speed loop is limited by APP_MAX_DRIVE_PERCENT */
#define APP_PULSE_DRIVE_PERCENT         (MP_APP_PULSE_DRIVE_PERCENT)   /**< from the motor profile */
#define APP_PULSE_BRAKE_LEAD_RAW        (8)
#define APP_PULSE_TRAVEL_LIMIT_RAW      (20)
#define APP_PULSE_MIN_US                (250U)
#define APP_PULSE_INITIAL_US            (500U)
#define APP_PULSE_MAX_US                (12000U)
#define APP_MAX_PULSES                  (160U)
#define APP_MAX_NO_PROGRESS_PULSES      (20U)
#define APP_PULSE_NO_PROGRESS_RAW       (3)

/* ------------------------------------------------------------------------- */
/* Power stage (DRV8411A) and motor                                           */
/* ------------------------------------------------------------------------- */
#define APP_PWM_FREQUENCY_HZ            (20000U)  /**< DRV8411A accepts 0..100 kHz                      */
/** 1 = TMC4671-style drive: AIN1/AIN2 complementary PWM (AIN2 = NOT AIN1), differential voltage = duty % of VM.
    0 = slow-decay (AIN1 held, AIN2 chopped). The NodeMCU/TMC4671 setup used the complementary scheme. */
#define APP_COMPLEMENTARY_DRIVE         (0)
#define APP_MOTOR_SWAP_OUTPUTS          (false)   /**< true swaps AIN1/AIN2 roles (delivery polarity)   */
#define APP_DRIVER_WAKE_DELAY_US        (150U)    /**< DRV8411A sleep->active 100 us max + margin       */
#define APP_DRIVER_SLEEP_ENTRY_US       (700U)    /**< coast longer than this may have entered sleep    */
#define APP_MOTOR_SUPPLY_VOLTS          (MP_APP_MOTOR_SUPPLY_VOLTS)   /**< from the motor profile */
#define APP_MOTOR_RESISTANCE_OHMS       (MP_APP_MOTOR_RESISTANCE_OHMS)   /**< from the motor profile */
#define APP_IPROPI_SENSE_OHMS           (1000.0F) /**< VERIFY against the EVM schematic (R at IPROPI)   */
#define APP_IPROPI_AMPS_PER_AMP         (0.0002F) /**< DRV8411A current mirror: 200 uA per A            */
#define APP_ADC_REFERENCE_VOLTS         (3.3F)
#define APP_ADC_FULL_SCALE              (4095.0F)
#define APP_OVERCURRENT_MA              (MP_APP_OVERCURRENT_MA)   /**< from the motor profile */
#define APP_STALL_CURRENT_FRACTION      (0.5F)    /**< of expected stall current at the present duty    */

/* ------------------------------------------------------------------------- */
/* MA782 required volatile configuration                                      */
/* ------------------------------------------------------------------------- */
#define APP_MA782_REQUIRED_FW           (12U)
#define APP_MA782_REQUIRED_RD           (0U)

/* ------------------------------------------------------------------------- */
/* Volumetric delivery scheduler                                              */
/* ------------------------------------------------------------------------- */
#define APP_VOL_TEST_DEGREES            (10.61F)
#define APP_VOL_PERIOD_US               (4000000UL)
#define APP_VOL_MOVES_PER_BATCH         (10U)
#define APP_VOL_BATCHES                 (10U)
#define APP_VOL_INTER_BATCH_WAIT_US     (180000000UL)
#define APP_VOL_PREFLIGHT_US            (160000UL)
#define APP_VOL_START_TOLERANCE_US      (25000UL)
#define APP_REPEAT_CYCLES               (10U)
#define APP_REPEAT_DWELL_US             (4000000UL)

/* ------------------------------------------------------------------------- */
/* Command limits, console and watchdog                                       */
/* ------------------------------------------------------------------------- */
#define APP_MOVE_MIN_DEGREES            (0.1F)
#define APP_MOVE_MAX_DEGREES            (360.0F)
#define APP_COMMAND_MAX_LENGTH          (48U)
#define APP_UART_BAUD                   (115200U)
#define APP_TRACE_CAPACITY              (1200U)
#define APP_TRACE_PERIOD_US             (10000UL)
#define APP_WATCHDOG_RELOAD_MS          (2000U)

#endif /* APP_CONFIG_H */