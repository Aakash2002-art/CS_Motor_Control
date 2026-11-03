# STM32U575 Position-Control Firmware - Complete Guide

**Audience:** someone who has never seen this project. After reading this document you should understand
what the system is, how the old NodeMCU firmware was converted, how the new firmware works, what was changed
during bring-up and why, what has been proven on the bench, and what is still open.

**Companion files**

| File | Content |
|---|---|
| `docs/FLOWCHARTS.md` (and `flowcharts.html`) | 14 flowcharts of how the code runs |
| `README.md` | Original build / wiring / validation notes written before hardware bring-up |
| `Src/*.c`, `Inc/*.h` | Source code, every function has a comment block |
| `Inc/app_config.h` | All constants, with the motor profile at the top |
| Parent folder `..\TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count.ino` | Original NodeMCU + TMC4671 firmware (reference) |
| Parent folder `Constar_motor_12bit.txt` | Reference results of the NodeMCU firmware on the Constar motor |

**Snapshot date:** 2026-11-03. **Safety note:** this is bench development firmware. It is not validated for patient
or production use. Delivered volume must be checked independently with a balance.

---

## 1. What the system does

A small DC gear-motor drives a mechanism (a reservoir plunger in the final application). The software must
rotate the **gearbox output shaft by a commanded angle** and stop with an error of at most **+/-1 encoder count**.

* The output angle is measured by an **MA782 magnetic angle sensor** mounted on the gearbox **output** shaft.
* One output revolution = **4096 virtual counts** (the sensor delivers 16 bit = 65536 native counts per revolution;
  16 native counts = 1 virtual count). One virtual count = 0.0879 degree.
* The angle command is **relative**: every move starts from the angle the shaft has at that moment.
  `g10.61` rotates the output by 10.61 degrees (about 121 counts) - one "unit of delivery" in the application.
* The firmware is **forward-only**: it never uses reverse drive to correct a move. If the shaft goes more than
  +1 count past the target, the move is declared failed. Explicit reverse moves exist only as a command (`g-10.61`).

---

## 2. History: the original NodeMCU + TMC4671 system

| Item | Original system |
|---|---|
| Controller | NodeMCU 1.0 (ESP8266), Arduino sketch `TMC4671_MA782_V4_5_...ino`, about 1080 lines |
| Motor driver chain | **TMC4671** motion-control IC (SPI) -> its PWM outputs `UX1_H` / `UX1_L` -> **DRV8411EVM** inputs `AIN1` / `AIN2` |
| Motor | Constar 0837M5, 3 V DC, plus an additional 3.5:1 reduction |
| Encoder | MA782 on the final output (SPI, mode 0, 16-bit frames), filter window FW=12 |
| Control | **All control loops were software on the NodeMCU** (speed PI, position profile, braking, catch-up pulses). The TMC4671 was used only as a voltage generator: `MODE_UQ_UD_EXT` (open-loop voltage), `UQ` limited to **25 %** of full scale, ENI pin to enable the outputs |
| TMC4671 PWM settings | about 25 kHz (`PWM_MAXCNT` 3999), dead time 10/10, centered PWM (`PWM_SV_CHOP` 7), motor type DC |
| Brake | TMC polarity register pattern (`PWM_POLARITIES` = 3) = both bridge inputs high |
| Result | Moves 10.61 degrees in about 240-490 ms, final count 121 (or 120), `+/-1` count |

Reference data: `Constar_motor_12bit.txt` contains 3 sets of 10 repeatability cycles of 10.61 degrees. All 30 cycles
passed, final count 121 (27 times) or 120 (3 times), time per move 239-487 ms, no catch-up pulses ("kicks=0").
The original README also quotes a longer run of 181 cycles (error 0 x161, +/-1 x20, average about 335 ms).

---

## 3. The new system

| Item | New system |
|---|---|
| Controller | **NUCLEO-U575ZI-Q** (STM32U575ZI, Cortex-M33, 160 MHz), STM32CubeIDE project, HAL + own modules in C |
| TMC4671 | **Removed.** The STM32 timer TIM1 generates the PWM for `AIN1` / `AIN2` directly |
| Driver | DRV8411A evaluation board (DRV8411AEVM, bridge A only) |
| Motor | Constar (current focus, VM = 3.0 V) or Maxon DCX08M 2.4 V + GPX08 4:1 (second profile) |
| Encoder | Same MA782, now on SPI1 (PA5/PA6/PA7, CS PA4) |
| Console | USART1 over the ST-LINK virtual COM port, 115200 8N1, same command set as the NodeMCU |

### 3.1 Software structure

| Layer | File | Role |
|---|---|---|
| Board | `bsp_hw.c/.h` | The only module that touches the HAL: TIM1 PWM, TIM2 1 MHz clock, TIM6 2 kHz tick, SPI1, ADC1, USART1, EXTI13, IWDG |
| Drive | `motor_driver.c/.h` | DRV8411A bridge: forward, reverse, brake, coast; duty limit; sleep/wake delay; nFAULT flag; IPROPI current |
| Drive | `ma782_encoder.c/.h` | MA782 SPI frames, register access, volatile FW=12 / RD=0 configuration (NVM is never written) |
| Algorithm | `motion_control.c/.h` | Move state machine, speed loop, brake, catch-up pulses, verify, hold, stationary check, direction test, trace |
| Algorithm | `position_calc.c/.h` | Angle unwrapping, jump check, degrees to raw, raw to count |
| Algorithm | `pi_controller.c/.h` | PI controller with limits and conditional anti-windup |
| Application | `fault_manager.c/.h` | First-fault latch, fault texts |
| Application | `app_commands.c`, `app_scheduler.c`, `app_state.c`, `app_main.c`, `console.c` | Serial commands, repeat/volumetric scheduler, readiness flags, main loop, printing without stdio |
| Configuration | `app_config.h` | All constants with units; motor profile at the top |
| Cube-generated | `main.c`, `stm32u5xx_*.c`, `system_stm32u5xx.c`, `syscalls.c`, `sysmem.c` | Clock setup, interrupt table, runtime support |

### 3.2 Execution model

* **TIM6 interrupt, 2 kHz (every 500 us):** `MotionControl_Tick()` samples the encoder, checks limits and runs the move
  state machine. It sets the PWM compare values.
* **EXTI13 interrupt:** the DRV8411A nFAULT line (currently ignored, see section 9).
* **Main loop (`App_Run`):** reads serial commands, prints finished moves, runs the repeat / volumetric scheduler,
  feeds the independent watchdog. Nothing is printed while a move is active.
* Blocking routines (stationary check, direction test, MA782 register access) suspend the 2 kHz interrupt with
  `Bsp_ControlTickSuspend()` because they share the SPI bus; they are only run when no move is active.

See `docs/FLOWCHARTS.md` sections 1-6.

---

## 4. Conversion map: NodeMCU function -> STM32 module

| NodeMCU sketch | STM32 firmware | Notes |
|---|---|---|
| `tmcWrite`, `tmcRead`, `verifyTmc`, `initializeBridge` | `bsp_hw.c` `Bsp_InitPwm`, `motor_driver.c` | TMC SPI is gone. PWM is TIM1 CH1/CH2 at 20 kHz |
| `setForwardVoltage(percent)`, `UQ_UD_EXT` | `MotorDriver_RunForward / RunReverse(duty)` | `UQ` (percent of 32767) became PWM duty on `AIN1`/`AIN2` |
| `stopBridge` | `MotorDriver_Stop` / `MotorDriver_Coast` | ENI low became both inputs low |
| `brakeBridge` | `MotorDriver_Brake` | both inputs high |
| `maFrame`, `maReadReg`, `maWriteReg`, `maConfigureVolatile`, `maConfigurationPass` | `ma782_encoder.c` | same registers, SPI1 hardware instead of bit-banging |
| `stationaryAngle` | `MotionControl_MeasureStationaryLocked` | same 20 ms wait, 50 samples, span <= 12 raw |
| `sampleMotion` | `MotionControl_SampleEncoder` + `CheckLimits` | now in a 2 kHz interrupt (NodeMCU polled at 1 kHz in `loop`) |
| `updateSpeedControl` | `MotionControl_RunSpeedLoop` | position PI -> speed reference -> velocity PI, every 8 ms |
| `tickMotion`, `beginBrake`, `startPulse`, `settledDecision` | `MotionControl_Tick`, `RunPhase`, `BeginBrake`, `StartPulse`, `SettledDecision` | same phases RUN, BRAKE, SETTLE, PULSE, VERIFY |
| `startMove`, `armMoveFromOrigin` | `AppMove_Start`, `MotionControl_StartMove` | |
| `latchFault` | `FaultManager_Latch`, `MotionControl_Fail` | first fault wins, bridge coasts |
| `calibrateDirection` (`dt`) | `MotionControl_CalibrateDirection` | measures the encoder sign |
| `startVolumetricTest ... pollVolumetricTest` | `app_scheduler.c` | 10 moves x 4 s start-to-start, 180 s between 10 batches |
| `handleCommand`, `pollSerial`, `printResult`, `printHelp` | `app_commands.c`, `app_main.c`, `console.c` | same commands and output format |
| `setup` / `loop` | `App_Init` / `App_Run` | |

### 4.1 What is the same

Command set and `[RUN]` output format; relative moves from a measured stationary start; forward-only logic; the
phase sequence RUN -> BRAKE (12 ms) -> SETTLE (20 ms) -> PULSE* -> VERIFY (+/-1 count) -> HOLD; encoder geometry
(4096 counts per output revolution); MA782 FW=12, RD=0 volatile; fault philosophy (latch the first fault, coast).

### 4.2 What is different

| Topic | NodeMCU + TMC4671 | STM32 |
|---|---|---|
| PWM | TMC4671, centered, about 25 kHz, **complementary** `UX1_H`/`UX1_L` -> `AIN1`/`AIN2` | TIM1, edge-aligned, 20 kHz. Default **slow decay**: `AIN1` held, `AIN2` chopped. TMC-style complementary drive is available as a switch (`APP_COMPLEMENTARY_DRIVE`) |
| Drive units | `UQ` percent of the TMC voltage range, max 25 % | duty percent of VM; the old percentages do **not** transfer one-to-one (section 8.4) |
| Sampling | polled about every 1 ms in `loop` | 2 kHz hardware interrupt |
| Zero demand | `UQ`=0 (outputs still switching) | zero duty coasts (a brake here caused stop-and-go, section 8.5) |
| Fault pins | nFAULT jumpered to ground | nFAULT interrupt, internal pull-up (checks currently disabled) |
| Motor current | not measured | IPROPI via ADC (scaling **not verified**) |
| Motor profile | one motor | `APP_MOTOR_SELECT` picks Constar or Maxon values |
| Diagnostics | trace | trace, `pw`, `dt` diagnostics, stationary samples |

---

## 5. Hardware and wiring

### 5.1 Pin map (see `README.md` section 2 for more detail)

| Signal | STM32 pin | Goes to |
|---|---|---|
| AIN1 (TIM1_CH1) | PE9 | DRV8411AEVM AIN1 |
| AIN2 (TIM1_CH2) | PE11 | DRV8411AEVM AIN2 |
| nFAULT (EXTI13) | PF13 | EVM nFAULT (needs a pull-up; internal one is a backup) |
| IPROPI (ADC1_IN8) | PA3 | EVM IPROPI test point |
| MA782 SCLK / MISO / MOSI / CS | PA5 / PA6 / PA7 / PA4 | MA782 |
| Console | PA9 / PA10 | ST-LINK virtual COM port |
| GND | GND | common ground for EVM, MA782, motor supply |

### 5.2 DRV8411AEVM requirements (learned the hard way, see section 8.7)

1. **VREF sets the driver's current limit.** Potentiometer **R27** sets VREF. A low VREF makes the driver chop the
   motor current: the output pins still show the full voltage but the motor gets almost no torque. The guide says:
   "If no current regulation is needed, set VREF to 3.3 V." In the current setup **VREF = 1.3 V** and the Constar
   motor runs correctly. The trip level must stay above the motor's stall current.
2. **Supply below 3.3 V:** the EVM's on-board 3.3 V LDO cannot regulate (VM = 3 V). The EVM needs an external
   3.3 V supply for its logic (the user confirmed the board needs two supplies when the jumper is removed).
3. **J9 shunts:** the on-board MSP430 drives AIN1/AIN2/nSLEEP through the J9 shunts. Remove the shunts so only the
   STM32 drives the inputs.
4. Bridge A only: motor on J10 (AOUT1/AOUT2); J7 open.
5. DRV8411A inputs: `00` coast, `10` forward, `01` reverse, `11` brake. Autosleep after about 1 ms of `00`.

---

## 6. How a move works (step by step)

All numbers are for the current Constar profile. See `docs/FLOWCHARTS.md` sections 5-9 for diagrams.

1. **Command:** `g10.61` -> `Commands_Dispatch` -> `AppMove_Start`.
2. **Checks:** system ready, no latched fault, no move active, angle 0.1..360, MA782 configuration valid.
3. **Stationary measurement:** bridge off, wait 20 ms, 50 angle samples over 100 ms, span must be <= 12 raw.
   The mean angle is the move origin.
4. **Arm:** `target_raw = degrees x 65536 / 360` (10.61 degrees = 1931 raw = 120.7 counts), direction from the sign,
   trackers and PI controllers reset, phase RUN, **breakaway kick** applied.
5. **RUN (2 kHz tick, speed loop every 8 ms):**
   * Breakaway: drive at the kick duty (20 %), +1 % per 8 ms until progress >= 6 raw (static friction).
   * Profile speed = min(cruise, sqrt(2 x decel x remaining)); limited to the approach speed in the last zone.
   * Position PI turns the remaining distance (counts) into a speed demand; reference speed = min(profile, demand).
   * Velocity PI turns the speed error into duty around the start-drive bias; duty 0 coasts.
   * Brake point: when remaining distance <= speed x stop-prediction + brake lead. The speed used is the larger of the
     8 ms filtered speed and a 2 ms fast estimate (the filter lags a stick-slip burst).
6. **BRAKE:** both inputs high for 12 ms. **SETTLE:** coast for 20 ms.
7. **Decision:** within +/-1 count -> VERIFY; short -> catch-up PULSE; past +1 count -> fault (no reverse correction).
8. **PULSE:** short forward pulse (time adapts to the load, 0.25-12 ms, travel limited to 20 raw), then BRAKE and
   SETTLE again. Up to 160 pulses.
9. **VERIFY:** bridge off for 50 ms: must stay within +/-1 count with span <= 12 raw.
10. **HOLD:** move complete, result posted: `[RUN] time_ms=... final_encoder_count=...`. If the output later drifts short
    the hold pulses forward again. A new command ends the hold first.

Control equations (units in brackets):

* Speed `v` [counts/s] = change of progress / (16 raw per count x time), filtered 50 % old + 50 % new.
* Position PI: `v_demand = Kp_pos x remaining + integral` [counts/s].
* Velocity PI: `duty = start_drive + Kp_vel x (v_ref - v) + integral` [% of VM], clamped to 0..max drive.
* Brake lead: `lead [counts] = v x stop_prediction [s] + min_lead [counts]`.

---

## 7. Configuration guide (`Inc/app_config.h`)

### 7.1 Motor profile

Change one line to select the motor, then rebuild:

```c
#define APP_MOTOR_SELECT   APP_MOTOR_CONSTAR        /* or APP_MOTOR_MAXON_DCX08M */
```

Every motor-dependent value is defined once per motor as `MP_APP_<name>` and mapped to `APP_<name>`.

| Parameter | NodeMCU original | **Constar profile (now)** | Maxon profile |
|---|---|---|---|
| Supply VM (V) | 3.0 | 3.0 | 2.4 |
| Motor resistance (ohm) | 13.6 | 13.6 | 4.13 |
| Overcurrent limit (mA) | n/a | 300 | 700 |
| Encoder sign default | by `dt` | true (measured) | true (measured) |
| Max drive (%) | 25 | 25 | 70 |
| Start drive / feed-forward (%) | 12 | 12 | 24 |
| Low-speed recovery drive (%) | 16 | 16 | 32 |
| Breakaway kick (%) | - | 20 | 24 |
| Pulse drive (%) | 25 | 25 | 100 |
| Velocity Kp (%/(count/s)) | 0.025 | **0.004** | 0.025 |
| Velocity Ki (%/count) | 0.08 | **0.15** | 0.30 |
| Position Kp ((count/s)/count) | 8 | **20** | 8 |
| Cruise speed (counts/s) | 300 | **800** | 300 |
| Approach speed (counts/s) | 80 | **300** | 60 |
| Approach zone (counts) | 30 | **20** | 30 |
| Decel (counts/s^2) | 1500 | **8000** | 1500 |
| Stop prediction (s) | 0.012 | **0.005** | 0.025 |
| Brake min lead (counts) | 2 | **1** | 25 |
| Rollback allowance (raw) | 24 | 24 | 400 |
| Stopped-short limit (counts) | 32 | 32 | 80 |
| Verify window (ms) | 100 | **50** | 100 |

Bold = changed from the NodeMCU value during this bring-up. The Maxon values were tuned **before** the VREF problem
was found (section 8.7) and must be re-verified.

### 7.2 Common settings

| Setting | Value | Meaning |
|---|---|---|
| `APP_CONTROL_TICK_US` | 500 | control tick period (2 kHz) |
| `APP_SPEED_LOOP_PERIOD_US` | 8000 | PI loops period |
| `APP_MAX_SAMPLE_GAP_US` | 5000 | feedback gap fault |
| `APP_BRAKE_US` / `APP_SETTLE_US` | 12000 / 20000 | brake and coast times |
| `APP_STABLE_SPAN_RAW` | 12 | max span in stationary and verify windows |
| `APP_ACCEPT_COUNTS` | 1 | +/-1 virtual count acceptance |
| `APP_PWM_FREQUENCY_HZ` | 20000 | PWM frequency |
| `APP_MOVE_TIMEOUT_US` / `APP_STALL_US` | 30 s / 1.2 s | timeout and no-progress watchdog |
| `APP_PULSE_*` | see file | catch-up pulse limits (travel 20 raw, 0.25-12 ms, 160 pulses) |
| `APP_VOL_*`, `APP_REPEAT_*` | 10.61 deg, 4 s, 10 x 10, 180 s | schedulers |

### 7.3 Switches

| Switch | Default | Effect |
|---|---|---|
| `APP_COMPLEMENTARY_DRIVE` | 0 | 0 = slow decay (AIN1 held, AIN2 chopped). 1 = AIN2 is the inverse of AIN1 (TMC4671 style) |
| `APP_IGNORE_DRIVER_FAULTS` | 1 | 1 = nFAULT, IPROPI overcurrent and current-based stall faults are **disabled** (bring-up). Set 0 to restore |
| `APP_MOTOR_SWAP_OUTPUTS` | false | swap the delivery direction if the motor turns the wrong way |

---

## 8. Chronology of the bring-up (what was found and changed)

The firmware was written to compile and had never run on hardware. This is the order in which problems were found.
**Dead ends are listed on purpose** so that nobody repeats them.

### 8.1 Stationary check failing, then passing
* Symptom: `z` printed `[STATIONARY] FAIL` with spans of 1000-2000 raw.
* Added: min/max/span and all 50 samples to the output. The samples were random jumps held for about 12 ms.
* Cause: the MA782 board was not seated properly. After fixing the placement the span was 1-3 raw.

### 8.2 nFAULT asserted during the direction test
* Added `APP_IGNORE_DRIVER_FAULTS` (nFAULT, IPROPI overcurrent, current-based stall) so that bring-up could continue.
* The real cause turned out much later (section 8.7).

### 8.3 Diagnostic tools added
* `dt` prints observed travel, jump, settled, nFAULT, peak current on failure.
* `pw<percent>`: steady 2 s drive that prints travel, ADC and nFAULT every 250 ms (negative = reverse).
* Stationary check prints min/max/span and samples.

### 8.4 Dead end: scaling the drive percentages
* The NodeMCU used `UQ` = 25 %. On the STM32 the same 25 % duty did not move the motor.
* I scaled the limits x4, x2 and tried a TMC-style complementary drive. **None of this was the real problem.**
  The motor was current-starved by VREF (section 8.7). The complementary drive code stayed in as a switch.

### 8.5 Encoder sign bug
* Symptom: every move failed with "rollback", progress went negative.
* Cause: `APP_ENCODER_INVERT_DEFAULT` was false; the sign is only set by `dt`, which had been skipped.
  Every `dt` that passed measured `encoder_invert = 1`.
* Fix: default set to true (measured, for both motors).

### 8.6 Motor profiles introduced
* Constar (3 V, 13.6 ohm) and Maxon DCX08M (2.4 V, 4.13 ohm) values from their datasheets, and a profile selector.
  The datasheet files are in the project folder (`DCX08M_2.4V GPX08 4_1 ENX 8 MAG_256.pdf`).

### 8.7 ROOT CAUSE of the weak / erratic motion: DRV8411AEVM VREF and supply
* Symptoms: motor moved only in jerks, needed 45-55 % duty to break away, "hard stops" and "sticky spots" at certain
  angles (about 28 degrees and 79 degrees), motor did nothing at 100 % duty with the Constar, driver output pins
  measured the full 3 V.
* Cause (from `sleu121.pdf`, the EVM user guide): on the DRV8411A EVM **VREF (potentiometer R27) sets the current
  regulation limit**, and with a supply under 3.3 V the EVM needs an external 3.3 V for its logic.
* Fix by the user: correct the EVM supply / jumper and set **VREF = 1.3 V**. The Constar motor then ran freely
  (`pw25` gave 64000 raw in 2 s).
* Lesson: when the motor "does not move" while the output pins show voltage, check the **current limit** first.

### 8.8 First good results (Constar, VM 3 V, VREF 1.3 V, original NodeMCU constants)
* `g10.61` / `g-10.61` x 20: **all 120 counts**, 0.65-1.4 s per move.
* Forward `g5` 57, `g20` 227, `g45` 513, `g90` 1023, `g180` 2047, `g360` 4095 (14.4 s).
* Reverse `g-20` 227, `g-60` 682, `g-90` 1023, `g-180` 2048, `g-360` 4095. `g-5` gave 58 and faulted (overtravel).

### 8.9 Stop-and-go and slow moves: tuning round 1 (**tested**)
The user saw the motor stop and restart during one move, with the current dropping to 0 mA. The plant was measured from
the `pw` runs (Constar, 3 V): speed is about **113 counts/s per % duty above a breakaway offset of about 8 %**
(`pw25` = 1960 counts/s, `pw50` = 4780 counts/s). Findings and changes:

| Finding | Change |
|---|---|
| Velocity Kp 0.025 gave a loop gain above 1 (100 counts/s error swings duty by 2.5 % = 280 counts/s): the duty bounced between 0 and 21 % | Kp 0.025 -> **0.004**, Ki 0.08 -> **0.15** |
| Zero duty in slow decay = both inputs high = **brake**, which stopped the motor dead | `MotorDriver_Drive`: zero duty now **coasts** |
| Last 30 counts limited to 80 counts/s, position Kp 8, cruise 300: a 10.61 degree move took 0.7-1.4 s | cruise 800, approach 300 in a 20-count zone, decel 8000, position Kp 20, verify 100 -> 50 ms |
| A command arriving during a hold pulse suspended the tick for 120 ms and raised a false "feedback gap" fault | `MotionControl_MeasureStationary` now sets any finished-move phase to idle first |

Result (tested): 10.61 degrees x 20, **all 120 counts**, times 242-795 ms (mostly 370-630 ms); `g5` 56, `g20` 227,
`g45` 512, `g90` 1023 in 1.4 s, **`g360` 4096 in 5.4 s**. Remaining problems: `g2` overshot (27 vs 23 counts, fault)
and the catch-up pulses made many moves slower than the 200-250 ms target.

### 8.10 Tuning round 2 (**NOT YET TESTED - firmware hold point**)
Analysis of the round-1 traces: the motor does not move until about 16-18 % duty, then jumps (160 raw in 10 ms); the
brake fired about 100 raw too early (stop prediction 12 ms against a measured stopping time of about 4.3 ms),
and the catch-up pulses (about 6 raw per 22 ms cycle) crawled through the remainder. Changes made:

| Change | Where |
|---|---|
| **Breakaway kick**: start at 20 %, +1 % per 8 ms until the first 6 raw of motion, then PI | `MotionControl_RunSpeedLoop`, `StartMove`, `APP_BREAKAWAY_*` |
| **Fast speed (2 ms window)** also used for the brake point | `MotionControl_FastSpeed` |
| Stop prediction 12 -> **5 ms**, brake min lead 2 -> **1 count** | Constar profile |
| Catch-up pulses larger: ratio cap 1.6 -> 2.5, desired size cap 10 -> 20 raw, travel limit 8 -> 20 raw | `motion_control.c`, `APP_PULSE_TRAVEL_LIMIT_RAW` |

Targets of round 2: 10.61 degrees completed in about 200-250 ms with final count 120-121, `g2` without overtravel.
**These changes have not been run on hardware yet.** They were written from the measured plant and trace analysis.

### 8.11 Other changes made during the session
* `APP_COMPLEMENTARY_DRIVE` option (TMC-style complementary PWM). Implemented but **only tested while VREF was wrong**,
  so it has not been evaluated properly.
* `APP_DRIVER_HARD_LIMIT_PERCENT` (100 %): the driver clamps to 100 % while the speed loop is limited by the max drive.
* Fixed a missing Maxon entry that would have broken the build if Maxon had been selected.
* Comments added to every function and an overview at the top of `motion_control.c`.

---

## 9. Serial command reference

Terminal: 115200 8N1, line ending newline. Send `?` for the list.

| Command | Action |
|---|---|
| `?` | help |
| `s` | status: ready, fault, scheduler, encoder sign, qualification, phase, progress, current, nFAULT pin |
| `mr` | MA782 configuration (FW=12, RD=0, status) - must show CONFIG PASS |
| `z` or `ast` | stationary check, prints PASS/FAIL min max span (plus samples on FAIL) |
| `an` | print 10 raw angle readings (wiring / magnet check) |
| `g<deg>` | one relative move; `g-10.61` is an explicit reverse move |
| `r<deg>` | repeatability: 10 moves with 4 s rest after each completion (needs a prior loaded `g10.61`) |
| `test` or `vol` | volumetric schedule: 10 batches x 10 moves, 4 s start-to-start, 180 s between batches |
| `trace1` / `trace0` | print a 10 ms trace after each move: `ms raw phase drive` |
| `clear` | acknowledge a fault (needs stationary output and valid MA782 config) |
| `x` | abort immediately (during a move this latches a fault) |
| `dt<N>` | measure encoder sign (drives forward N % for at most 80 ms); `N` from 5 to the driver limit |
| `pw<N>` | **bench**: steady drive for 2 s at N % (negative = reverse), prints travel/ADC/nFAULT every 250 ms. Do not run into end stops |

Output per move: `[RUN] time_ms=<ms> final_encoder_count=<count>` and, on failure, `[FAULT] <reason>`.
`time_ms` includes the brake (12 ms), settle (20 ms) and verify (50 ms) windows, so about 82 ms of every
reported time is fixed.

---

## 10. Fault reference

Only the first fault is kept; the bridge coasts; `clear` re-arms.

| `[FAULT]` text | Meaning | Common causes seen |
|---|---|---|
| no meaningful forward progress for 1200 ms | stall or no power | no motor current (VREF!), jam, end stop |
| peak overtravel > +1 count | shaft went past target + 1 count | brake too late, stick-slip burst on a short move |
| output rollback exceeds raw jitter allowance | progress fell back from its peak by more than the allowance | gearhead recoil, encoder sign wrong |
| stopped >32 counts short | first brake left it too far away | brake point too early |
| implausible encoder jump | one-tick step > 512 raw | encoder glitch / electrical noise |
| feedback service gap > 5 ms | control tick starved | a command suspended the tick (fixed for the hold case) |
| endpoint unstable during verify | span > 12 raw while verifying | bounce after stop |
| output not stationary | stationary check failed | encoder not seated, shaft moving |
| direction calibration had no reliable motion | `dt` saw < 32 raw or no settling | motor not turning, stuck |
| MA782 configuration/status invalid | FW/RD/status registers wrong | wiring, power |
| DRV8411A nFAULT asserted / motor current above overcurrent | driver reports a fault | **ignored while `APP_IGNORE_DRIVER_FAULTS` = 1** |

---

## 11. What has been verified

| Item | Status |
|---|---|
| Encoder reading, stationary check, config registers | verified |
| PWM on AIN1/AIN2 (20 kHz) and motor rotation, Constar at 3 V, VREF 1.3 V | verified |
| Encoder sign (inverted) | verified (`dt` PASS) |
| `g10.61` forward and reverse, +/-1 count | verified, 40 moves |
| Angles 5 to 360 degrees, forward and reverse, +/-1 count | verified (rounds 8.8 and 8.9) |
| Move time 200-250 ms for 10.61 degrees | **not yet reached** (best 242 ms, typical 370-630 ms before round 2) |
| `g2`, `g-5` (small moves) | overshoot seen, round 2 should address it, **untested** |
| Round 2 changes (section 8.10) | **untested** |
| Volumetric schedule (`test` / `vol`) | **never run on this hardware** |
| Repeatability (`r10.61`) | **never run on this hardware** |
| nFAULT / IPROPI / overcurrent / stall-current faults | **disabled**; IPROPI scaling **not verified** (ADC reads about 550 raw regardless of motion) |
| Complementary drive | untested with correct VREF |
| Maxon motor | profile exists, **needs re-verification** after the VREF fix |
| Watchdog | present, not stress-tested |

---

## 12. Known issues and suggested next steps (research stage)

1. **Run round 2** (section 8.10): `z`, `trace1`, then 10 cycles of `g10.61` / `g-10.61`, then `g2`, `g5`, `g20`, `g45`,
   `g90`, `g360` with their reverses. Compare the time against the 200-250 ms goal and check +/-1 count.
2. If the brake now fires too late, raise `MP_APP_STOP_PREDICTION_S`; if the shaft stops short and needs many pulses,
   lower it or the lead. The trace phases show which.
3. **Compare `APP_COMPLEMENTARY_DRIVE` = 1** with the default now that the EVM current limit is correct. The NodeMCU
   used complementary switching and its motion was smoother.
4. **Fix IPROPI:** read the sense resistor from the EVM schematic (`APP_IPROPI_SENSE_OHMS`), verify `current_mA`
   with a meter, then set `APP_IGNORE_DRIVER_FAULTS` to 0 to restore nFAULT and current protection.
5. **Run `r10.61` and `test`** to check repeatability and the 4 s cadence. A move that takes longer than about 4 s aborts
   the volumetric test by design.
6. **Re-profile the Maxon motor** with VREF set correctly; its values are from before the root cause was found.
7. Fix the console printing while a hold is active more thoroughly if "feedback gap" faults reappear.
8. **`build.ps1` link step fails** with "multiple definition" of the syscalls (it compiles `syscalls.c`/`sysmem.c` from two folders). All own modules compile with `-Wall -Wextra`, 0 warnings, so the code is fine; build with STM32CubeIDE (the project in `STM32CubeIDE/`) until the script is fixed.
9. Do not claim MISRA compliance: the README lists the scope and the known deviations, no checker has been run.

---

## 13. Troubleshooting

| Symptom | First thing to check |
|---|---|
| Output pins show VM but the motor does not turn | **VREF / current limit (R27)**, then motor leads, then supply current |
| Motor turns but encoder shows no travel | coupling to the gearbox, magnet on the output shaft (`an` command) |
| Every move says "rollback" | encoder sign: run `dt20` or set `APP_ENCODER_INVERT_DEFAULT` |
| `z` fails with spans of hundreds of raw | MA782 not seated, magnet gap, supply noise |
| "no meaningful forward progress" and shaft stops at the same angle | end stop, jam, or driver current limit |
| Jerky motion, current falling to 0 mA | speed-loop gains too high or zero duty braking (fixed in 8.9) |
| `g` is refused with "fault latched" | run `clear`, then `z` |
| Nothing is printed after a reset | board still in debug halt: press reset; check the COM port number |
| Moves go the wrong way | `APP_MOTOR_SWAP_OUTPUTS` |
| Two `g` commands in a row hit an end stop | a forward move cannot be repeated forever; retract with `g-<same angle>` first |

---

## 14. Glossary

| Term | Meaning |
|---|---|
| raw | native MA782 count, 65536 per output revolution |
| count / virtual count | 16 raw, 4096 per revolution, 0.0879 degree |
| cps | counts per second |
| duty | PWM duty in percent of the motor supply voltage (VM) |
| progress | unwrapped distance moved since the start of the move, positive toward the target |
| slow decay | AIN1 held high, AIN2 chopped: drive when AIN2 is low, brake when it is high |
| complementary | AIN2 is always the inverse of AIN1: drive/reverse switching (fast decay) |
| breakaway | the duty needed to overcome static friction and start the shaft |
| kick | the initial drive applied until the shaft starts moving |
| catch-up pulse | a short forward drive to cover the last few counts |
| qualification | a successful standalone `g10.61` since reboot, needed for `r` and `test` |
| VREF | EVM reference voltage that sets the DRV8411A current regulation limit |
| IPROPI | DRV8411A current-mirror output used to estimate motor current |

---

## 15. Files created or changed during this bring-up

| File | Change |
|---|---|
| `Inc/app_config.h` | motor profile, `APP_COMPLEMENTARY_DRIVE`, `APP_IGNORE_DRIVER_FAULTS`, breakaway and driver-limit constants, retuned Constar values |
| `Src/motion_control.c` | breakaway kick, fast speed, hold-race fix, diagnostics (stationary samples, calibration diag), pulse constants, comments |
| `Src/motor_driver.c` | zero duty coasts, complementary drive, hard limit |
