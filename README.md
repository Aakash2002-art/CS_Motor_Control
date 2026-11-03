# STM32U575 + DRV8411A + MA782 — Position Control Firmware

Replaces the NodeMCU + TMC4671 firmware (V4.5). Same motor, MA782 (on the final gearbox output), 3.5:1 gearbox.
Target: commanded relative move, **±1 virtual count** (4096/rev) at the output, forward-only correction, repeatable.

> **Status (2026-11-03): bench-tested with the Constar motor (VM 3 V).**
> **Read `docs/FIRMWARE_GUIDE.md` first**: it explains the conversion from the NodeMCU firmware, the bring-up history
> (including what was found wrong and why), what has been verified, and what is still open.
> Flowcharts of how the code runs: `docs/FLOWCHARTS.md` (or open `docs/flowcharts.html`).
> This README was written before hardware bring-up. Its wiring and build sections are still valid, but sections
> that describe tuning values and "not hardware-tested" are superseded by the guide. Two things changed on the
> hardware side: the DRV8411AEVM **VREF (R27) sets the current limit** and must be set correctly (currently 1.3 V),
> and the EVM needs an external 3.3 V logic supply when VM is below 3.3 V.
> The firmware is bench development firmware, **not validated for patient delivery**.

## 1. Architecture

| Layer | File | Role |
|---|---|---|
| Board | `bsp_hw.c/.h` | Only module that touches the HAL (TIM1 PWM, TIM2 µs clock, TIM6 2 kHz tick, SPI1, ADC1, USART1, EXTI, IWDG) |
| **Drive** | `motor_driver.c/.h` | DRV8411A bridge A: `RunForward`, `RunReverse`, `Brake`, `Coast`, `Stop`, duty limit, sleep/wake handling, nFAULT flag, IPROPI current |
| **Drive** | `ma782_encoder.c/.h` | MA782 SPI mode 0, 16-bit frames, register access, volatile FW=12 / RD=0 config (NVM is never written) |
| Application | `position_calc.c/.h` | Angle unwrap, jump check, degrees→raw, raw→virtual count |
| Application | `pi_controller.c/.h` | PI with clamp and conditional anti-windup |
| **Algorithm** | `motion_control.c/.h` | Position PI → velocity PI → duty; brake; coast settle; forward pulses; verify; hold |
| Application | `fault_manager.c/.h` | First-fault latch, fault texts, nFAULT handling |
| Application | `app_commands.c`, `app_scheduler.c`, `app_state.c`, `app_main.c`, `console.c` | Serial commands, repeat/volumetric scheduler, reporting, main loop |
| Config | `app_config.h` | **All** constants, with units |

Execution model: TIM6 interrupt (2 kHz) runs `MotionControl_Tick()`; EXTI13 latches nFAULT; the main loop handles
commands, results, scheduler and the watchdog. Nothing is printed while a move is active.

### Move sequence (forward-only — reverse drive is never used to correct a move)
`RUN` (position PI outer, velocity PI inner, decelerating profile) → `BRAKE` (12 ms, both inputs high) →
`SETTLE` (20 ms coast) → [`PULSE` load-adaptive forward catch-up, repeat] → `VERIFY` (100 ms, within ±1 count,
span ≤ 12 raw) → `HOLD` (bridge off; forward-only catch-up if output drifts below −1 count).
Overshoot > +1 count, rollback > 24 raw, encoder jump, feedback gap > 5 ms, stall, timeout, overcurrent,
nFAULT → **latched fault, bridge coasts**. Reverse moves exist only as an explicit command (`g-10.61`).

## 2. Wiring (NUCLEO-U575ZI-Q ↔ DRV8411AEVM ↔ MA782)

Pins were cross-checked against the STM32Cube U5 Nucleo examples (TIM1_CH2/PE11, SPI1 PA5/6/7, USART1 PA9/10).
**Verify them against your board's solder bridges and the EVM schematic before powering.**

| Signal | STM32 pin | Connect to |
|---|---|---|
| AIN1 (TIM1_CH1) | PE9 | EVM AIN1 (J9 header / test point) |
| AIN2 (TIM1_CH2) | PE11 | EVM AIN2 |
| nFAULT (EXTI13) | PF13 | EVM nFAULT, **remove the ground jumper**, pull-up to 3.3 V (internal pull-up enabled as backup) |
| AIPROPI (ADC1_IN8) | PA3 | EVM AIPROPI test point |
| SCLK / MISO / MOSI | PA5 / PA6 / PA7 | MA782 SCLK / MISO / MOSI |
| /CS | PA4 | MA782 /CS |
| Console | PA9 TX / PA10 RX | ST-LINK virtual COM port (USART1), 115200 8N1 |
| GND | GND | EVM GND, MA782 GND, motor supply GND (common) |

EVM changes (DRV8411AEVM, MD063):
1. **Remove the J9 shunts** for AIN1/AIN2 so the on-board MSP430 does not drive the inputs; feed PWM from the STM32.
2. **VM = 3.0 V < 3.3 V: the EVM's 3.3 V LDO cannot regulate.** Supply the EVM's digital 3.3 V externally (Nucleo 3V3).
3. Bridge A only; leave **J7 open** (parallel mode off); motor on **J10 (AOUT1/AOUT2)**.
4. R27 sets VREF (current regulation). Start with VREF = 3.3 V (regulation off), then set a limit above the
   expected running current.
5. **IPROPI**: `APP_IPROPI_SENSE_OHMS` in `app_config.h` (default 1 kΩ) must match the EVM resistor to ground;
   DRV8411A mirror = 200 µA/A. Overcurrent (300 mA) and the stall-current check depend on it.
6. MA782: VDD and VDDIO = 3.3 V, EN to GND if unused, ASC to GND.

DRV8411A bridge truth table used: coast `00`; forward slow-decay `AIN1=1, AIN2=PWM` (PWM low = drive, high = brake);
reverse `AIN1=PWM, AIN2=1`; brake `11`. All-low enters autosleep after ≈0.7–1.5 ms; the driver inserts a
150 µs wake delay when it leaves a long coast. If the delivery direction is wrong, set `APP_MOTOR_SWAP_OUTPUTS`.

## 3. Build

**STM32CubeIDE 1.15.1 (tested):** *File ▸ Import ▸ General ▸ Existing Projects into Workspace* → select the
`STM32CubeIDE` folder of this directory → *Project ▸ Build* (Debug). The HAL and CMSIS sources are copied under
`Drivers\` so the project is self-contained. Flash with the on-board ST-LINK (TrustZone disabled, option byte
TZEN=0, the Nucleo default).

**Command line:** `powershell -ExecutionPolicy Bypass -File build.ps1` (uses the GNU Arm toolchain bundled with
CubeIDE; outputs `Build\*.elf/.bin`).

Open a terminal at 115200 8N1 on the ST-LINK COM port; send `?` for help.

## 4. Commands (same set as the NodeMCU firmware)

`mr`, `dt5..dt25` (optional encoder-sign check; detach reservoir), `ast`, `g<deg>` (negative = reverse), `r<deg>`,
`test`/`vol`, `trace1`/`trace0`, `x`, `clear`, `s`, `?`.
Output per run: `[RUN] [batch=B move=M | cycle=N] time_ms=T final_encoder_count=C`, plus `[FAULT] <reason>`.
Encoder sign defaults to `APP_ENCODER_INVERT_DEFAULT`; `dt` re-measures it (not required if wiring is unchanged
and moves go the right way).

## 5. Bring-up order (do not skip)
1. Power with motor **disconnected**. Check `mr`: FW=12, RD=0, CONFIG PASS. Check `s`: `nfault_pin=0`.
2. Scope AIN1/AIN2 at 0 % (both low), then confirm PWM period 50 µs.
3. Connect the motor, **reservoir detached**. `ast` → PASS. `dt10` → record `encoder_invert` and the physical
   direction; if wrong, set `APP_MOTOR_SWAP_OUTPUTS` and re-run.
4. `trace1`, `g5` repeatedly; compare against the NodeMCU log (target about 335 ms / 10.61° average, errors 0/±1).
5. Attach the reservoir; `g10.61` (qualifies `r`/`test`), then `r10.61`, then `test`.
6. Check IPROPI (`s` shows `current_mA`) against a meter before relying on the stall check.

## 6. Tuning (initial values come from the validated NodeMCU firmware and the motor datasheet)
`app_config.h`: `APP_VELOCITY_KP/KI`, `APP_POSITION_KP/KI`, `APP_START_DRIVE_PERCENT` (feed-forward),
`APP_CRUISE_CPS`, `APP_APPROACH_CPS`, `APP_DECEL_CPS2`, pulse limits, `APP_CONTROL_TICK_US`,
`APP_SPEED_LOOP_PERIOD_US`. The 25 % drive ceiling assumes the same 3 V supply as the TMC4671 setup; the TMC4671
UQ scaling and a duty cycle are not identical, so confirm speed on the bench. Do not raise the ceiling to hide a stall.

## 7. Validation plan (compare with the NodeMCU baseline: 181 cycles, error 0 ×161 / ±1 ×20, avg ≈335 ms)
| Test | Method | Pass criterion |
|---|---|---|
| Encoder accuracy | `ast` ×20; rotate by hand, compare `s` progress vs. dial | span ≤ 12 raw, no jumps |
| Motor direction | `dt10`, visual | forward drives the delivery direction |
| PWM | Scope PE9/PE11 | 20 kHz, 0–25 % only, no shoot-through glitches, brake = both high |
| Gear ratio | `g360` unloaded, one output revolution | final count 4096 ±1 (360° = one output rev) |
| Position accuracy | `g10.61` ×100 loaded | every final count = target ±1 |
| PI stability | `trace1`, inspect | no oscillation, monotonic approach, no rollback |
| Stopping | brake/settle trace | no overshoot > +1; settles within 20 ms |
| Repeatability | `r10.61` ×10 batches | no fault, error distribution ≤ NodeMCU |
| Faults | block shaft; short nFAULT to GND; unplug MA782; send `x` mid-move | latched fault, bridge off, correct text |
| Endurance | `test` (100 moves) ×N | no fault, cadence within 25 ms, independent balance check of delivered mass |

## 8. MISRA C:2012 scope and deviations
Applied to all own modules except `bsp_hw.c` (HAL access) and Cube-derived files (`main.c` clock setup,
`stm32u5xx_*.c`, `system_stm32u5xx.c`). No MISRA checker was available, so **compliance is by construction and
review only, not certified**; run cppcheck-premium/PC-lint before claiming it. Known deviations:
- Dir 4.9 / macros: `app_config.h` uses object-like macros for configuration constants.
- Rule 11.5/11.8 and 11.3: HAL buffer/pointer casts only inside `bsp_hw.c`.
- Rule 15.5 (advisory): a few functions keep a single exit; remaining multi-branch `if/else` chains are used instead.
- Rule 21.7/21.6: `strtof` is used for command parsing; no `stdio`/dynamic memory anywhere.
- Rule 10.x: signed/unsigned comparisons of µs timestamps use the helper `Scheduler_TimeDiffUs`.
- Float is used (Cortex-M33 FPU); `-Wfloat-equal` clean.

## 9. Known limitations / risks
- Not hardware-tested; gains are starting values; loaded friction unknown.
- ±1 count (0.088°) is close to MA782 noise (0.015–0.2° RMS, 8–12 bit effective). Achievable accuracy depends on the sensor as much as the loop.
- Reverse correction is intentionally absent; a mechanism that drives the output past +1 count faults.
- `APP_IPROPI_SENSE_OHMS` is a placeholder until read from the EVM schematic.
- Hold is forward-only catch-up (bridge off otherwise); it does not resist back-drive between catch-ups.

## 10. CubeMX reference file
`STM32U575_Volumetric_Firmware_REFERENCE_ONLY.ioc` mirrors the pin map, clock tree (160 MHz) and peripheral settings used by `bsp_hw.c` so they can be viewed in STM32CubeMX. It is **view-only**: do **not** press *Generate Code* - CubeMX would create `MX_*_Init` functions, MSP code and interrupt handlers that clash with `bsp_hw.c` and `stm32u5xx_it.c`. It was written by hand, not produced or validated by CubeMX, so if it opens with warnings, trust `bsp_hw.c`.
