# STM32U575 Position-Control Firmware - Flowcharts

How the firmware works, one diagram per topic. Written for someone who has not seen the code.
View this file in VS Code (Markdown preview with Mermaid), GitHub, or open `flowcharts.html` in a browser.
Function names in the boxes are the real function names in `Src/`.
Companion document: `FIRMWARE_GUIDE.md`.

Units used in the diagrams: **raw** = native MA782 count (65536 per output revolution),
**count** = virtual count (16 raw, 4096 per revolution), **duty** = percent of the motor supply voltage (VM).

---

## 1. Big picture - who runs what

Three execution contexts. Only the 2 kHz interrupt touches the motor during a move; the main loop only
talks to the user and never blocks the control loop.

```mermaid
flowchart TB
    subgraph PC["PC (serial terminal 115200 8N1)"]
        U["User types g10.61, z, trace1, ..."]
    end
    subgraph MAIN["Main loop (App_Run)"]
        M1["Commands_Poll<br/>read characters, build a line"]
        M2["Commands_Dispatch<br/>run the command"]
        M3["App_HandleResult<br/>print finished move"]
        M4["Scheduler_Poll<br/>repeat / volumetric test"]
        M5["App_ServiceWatchdog<br/>feed IWDG while the tick is alive"]
    end
    subgraph ISR1["TIM6 interrupt, 2 kHz (every 500 us)"]
        T1["MotionControl_Tick"]
        T2["SampleEncoder: read MA782 angle"]
        T3["CheckLimits: faults"]
        T4["RunPhase: state machine"]
        T5["MotorDriver: set PWM"]
    end
    subgraph ISR2["EXTI13 interrupt"]
        F1["nFAULT falling edge<br/>FaultManager_OnDriverFaultInterrupt"]
    end
    subgraph HW["Hardware"]
        ENC["MA782 magnetic encoder<br/>on gearbox output (SPI1)"]
        DRV["DRV8411A bridge<br/>AIN1 / AIN2 from TIM1 PWM"]
        MOT["DC motor + gearbox"]
    end
    U --> M1 --> M2
    M2 -->|"g, r, test: start move"| T1
    T1 --> T2 --> T3 --> T4 --> T5
    ENC -->|angle| T2
    T5 --> DRV --> MOT
    MOT -->|shaft turns| ENC
    T4 -->|result posted| M3 --> U
    F1 -->|latch fault, coast| DRV
    M4 --> M2
```

---

## 2. Power-up (App_Init)

```mermaid
flowchart TD
    A["Reset"] --> B["main.c: HAL_Init + SystemClock 160 MHz"]
    B --> C["App_Init"]
    C --> D["Bsp_Init<br/>GPIO, TIM1 PWM, TIM2 1 MHz clock,<br/>SPI1, ADC1, USART1, nFAULT EXTI"]
    D --> E["MotorDriver_Init: bridge coasting"]
    E --> F["FaultManager_Init, MotionControl_Init"]
    F --> G["Ma782_ConfigureVolatile<br/>force FW=12, RD=0 (RAM only, no NVM write)"]
    G --> H{"Ma782_ReadConfiguration:<br/>FW=12, RD=0, no error bits?"}
    H -- yes --> I["AppState_SetReady(true)"]
    H -- no --> J["not ready: every move is refused<br/>message: MA782 configuration/status invalid"]
    I --> K["Bsp_ControlTickStart: 2 kHz tick starts"]
    J --> K
    K --> L["Bsp_WatchdogStart (IWDG, 2 s)"]
    L --> M["print banner, then App_Run (main loop)"]
```

---

## 3. Main loop (App_Run)

```mermaid
flowchart TD
    S(["loop forever"]) --> W["App_ServiceWatchdog<br/>refresh IWDG only if the control tick counter changed"]
    W --> C["Commands_Poll<br/>at most 8 characters per pass"]
    C --> R["App_HandleResult<br/>move finished?"]
    R -->|yes| R1["print [RUN] time_ms, final_encoder_count<br/>print [FAULT] if failed<br/>print trace if trace1"]
    R1 --> R2["Scheduler_OnMoveResult<br/>qualify g10.61, advance repeat/volumetric"]
    R -->|no| P
    R2 --> P["Scheduler_Poll"]
    P --> F["App_ReportIdleFault<br/>fault latched outside a move: print once"]
    F --> S
```

---

## 4. Serial command handling

A command line ends with newline. The letter `x` is special: it is acted on immediately, even in the middle of a move.

```mermaid
flowchart TD
    A["Commands_Poll: character received"] --> B{"character"}
    B -- "x or X" --> X["Commands_HandleAbort<br/>move active: latch FAULT_USER_ABORT<br/>idle: release bridge, cancel schedules"]
    B -- "CR" --> I["ignored"]
    B -- "LF" --> L{"line not empty and no move active?"}
    B -- "other" --> Q["append to line buffer"]
    L -- yes --> D["Commands_Dispatch"]
    L -- no --> N["drop the line"]
    D --> K{"which command?"}
    K -- "?" --> H["Commands_PrintHelp"]
    K -- "s" --> S["Commands_PrintStatus"]
    K -- "mr" --> MR["Commands_PrintMa782"]
    K -- "z or ast" --> Z["Commands_HandleStationary"]
    K -- "an" --> AN["Commands_HandleAngle"]
    K -- "trace1 / trace0" --> TR["MotionControl_SetTraceEnabled"]
    K -- "clear" --> CL["Commands_HandleClear"]
    K -- "dtNN" --> DT["Commands_HandleDirectionTest"]
    K -- "pwNN" --> PW["Commands_HandleBenchDrive<br/>steady 2 s drive, bench test"]
    K -- "g deg" --> G["Commands_HandleMove<br/>AppMove_Start"]
    K -- "r deg" --> RP["Commands_HandleRepeat<br/>Scheduler_StartRepeat"]
    K -- "test or vol" --> V["Scheduler_StartVolumetric"]
    K -- "anything else" --> E["print [ERR] unknown command"]
```

---

## 5. Starting a move (AppMove_Start)

```mermaid
flowchart TD
    A["g10.61 -> AppMove_Start(degrees)"] --> B{"ready?"}
    B -- no --> E1["APP_MOVE_NOT_READY"]
    B -- yes --> C{"fault latched?"}
    C -- yes --> E2["APP_MOVE_FAULT_LATCHED<br/>message: clear or dt first"]
    C -- no --> D{"move already active?"}
    D -- yes --> E3["APP_MOVE_BUSY"]
    D -- no --> F{"0.1 <= angle <= 360 and finite?"}
    F -- no --> E4["APP_MOVE_BAD_ARGUMENT"]
    F -- yes --> G["AppMove_CheckEncoderConfig<br/>read MA782 registers (tick suspended)"]
    G -- invalid --> E5["latch FAULT_ENCODER_CONFIG"]
    G -- ok --> H["MotionControl_MeasureStationary<br/>bridge off, 20 ms wait, 50 samples, span <= 12 raw"]
    H -- unstable --> E6["latch FAULT_NOT_STATIONARY"]
    H -- stable --> I["MotionControl_StartMove(degrees, origin)"]
    I --> J["target_raw = degrees x 65536 / 360<br/>direction = sign of degrees<br/>reset trackers and PI controllers"]
    J --> K["phase = RUN<br/>drive = breakaway kick"]
    K --> L(["move armed: the 2 kHz tick now controls it"])
```

Each move is **relative** to a freshly measured stationary start angle, not to an absolute position.

---

## 6. One control tick (MotionControl_Tick, every 500 us)

```mermaid
flowchart TD
    A(["TIM6 interrupt"]) --> B{"phase is IDLE or FAULTED?"}
    B -- yes --> Z(["return"])
    B -- no --> C{"fault latched?"}
    C -- yes --> F1["MotionControl_EnterFault<br/>bridge off, post failed result"]
    C -- no --> D{"phase = PULSE and pulse time elapsed?"}
    D -- yes --> D1["BeginBrake"]
    D -- no --> E
    D1 --> E["SampleEncoder"]
    E --> E1{"gap since last sample > 5 ms?"}
    E1 -- yes --> E2["fault: feedback gap"]
    E1 -- no --> E3["read MA782 angle<br/>PositionCalc_Update: unwrap, apply sign"]
    E3 --> E4{"step > 512 raw?"}
    E4 -- yes --> E5["fault: encoder jump"]
    E4 -- no --> E6["progress_raw = direction x position<br/>update peak and progress marks"]
    E6 --> G["CheckLimits"]
    G --> G1{"limit violated?"}
    G1 -- yes --> G2["fault (see section 10)"]
    G1 -- no --> H["RunPhase: state machine (section 7)"]
```

---

## 7. Move state machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> RUN: StartMove
    RUN --> BRAKE: remaining distance within stop lead
    BRAKE --> SETTLE: after APP_BRAKE_US (12 ms), coast
    SETTLE --> VERIFY: within plus/minus 1 count
    SETTLE --> PULSE: short of target
    SETTLE --> FAULTED: past +1 count, or more than the short-stop limit short
    PULSE --> BRAKE: travel limit, near target, or pulse time elapsed
    VERIFY --> HOLD: stable for APP_VERIFY_US
    VERIFY --> PULSE: drifted short
    VERIFY --> FAULTED: over target or unstable
    HOLD --> PULSE: output drifted short of target
    HOLD --> IDLE: new stationary measurement or release
    RUN --> FAULTED: limit violated
    PULSE --> FAULTED: limit violated
    FAULTED --> IDLE: clear (stationary, config valid)
```

Phase meaning:

| Phase | What the motor is doing | Ends when |
|---|---|---|
| RUN | speed loop drives toward the target | remaining distance <= brake lead |
| BRAKE | both bridge inputs high (motor shorted) for 12 ms | timer |
| SETTLE | bridge off (coast) for 20 ms so the shaft stops | timer, then decision |
| PULSE | short forward pulse to catch up (never reverse) | time, 20 raw travel, or near target |
| VERIFY | bridge off, shaft must stay within +/-1 count (span <= 12 raw) | window elapsed |
| HOLD | bridge off, move is complete and reported | next command |

---

## 8. RUN phase - the speed loop (every 8 ms)

```mermaid
flowchart TD
    A["RunPhase, phase = RUN"] --> B["speed_for_lead = max(filtered speed, 2 ms fast speed)"]
    B --> C["lead = speed x APP_STOP_PREDICTION_S + APP_BRAKE_MIN_LEAD counts"]
    C --> D{"target - progress <= lead?"}
    D -- yes --> E["BeginBrake -> BRAKE phase"]
    D -- no --> F["RunSpeedLoop"]
    F --> G{"8 ms elapsed?"}
    G -- no --> Z(["return, keep last duty"])
    G -- yes --> H["measure speed = delta progress / delta time<br/>filter: 50 % old + 50 % new"]
    H --> I{"breakaway done?<br/>(progress >= 6 raw)"}
    I -- no --> J["drive = kick duty<br/>+1 % per 8 ms, up to max drive<br/>(static friction)"]
    I -- yes --> K["STEP 1 profile speed:<br/>min(cruise, sqrt(2 x decel x distance))<br/>capped at approach speed in the last zone"]
    J --> Z
    K --> L["STEP 2 position PI:<br/>remaining counts -> speed demand"]
    L --> M["reference = min(profile, position demand)<br/>keep a small minimum approach speed"]
    M --> N["STEP 3 velocity PI:<br/>reference - measured speed -> duty<br/>bias = start drive, clamp 0..max"]
    N --> O{"duty = 0?"}
    O -- yes --> P["MotorDriver_Coast (do not brake)"]
    O -- no --> Q["MotorDriver_RunForward / RunReverse(duty)"]
    P --> Z
    Q --> Z
```

---

## 9. After the brake - settle decision and catch-up pulses

```mermaid
flowchart TD
    A["SETTLE finished (20 ms of coast)"] --> B{"came from a pulse?"}
    B -- yes --> C["AdaptPulse: change the next pulse time"]
    B -- no --> D
    C --> D{"progress vs target"}
    D -- "more than +1 count past" --> F1["fault: settled overshoot<br/>(forward-only: no reverse correction)"]
    D -- "within +/-1 count" --> V["enter VERIFY"]
    D -- "short by more than short-stop limit" --> F2["fault: stopped short"]
    D -- "short by a little" --> P["StartPulse"]
    P --> P1{"pulse count >= 160?"}
    P1 -- yes --> F3["fault: pulse limit"]
    P1 -- no --> P2["drive forward at APP_PULSE_DRIVE_PERCENT<br/>stop at +20 raw travel or near target"]
    P2 --> P3["BRAKE (12 ms) -> SETTLE (20 ms) -> back to the top"]
    A2["AdaptPulse rules"] --> A3["moved < 3 raw: longer pulse<br/>(3, 5, 8, 10 ms steps, 20 tries max)"]
    A2 --> A4["moved enough: scale pulse time so that<br/>movement = 40 % of remaining, 3..20 raw"]
```

---

## 10. Faults

Only the **first** fault is kept. The response is always the same: **bridge coasts**, the move result is posted as failed,
and further moves are refused until `clear`.

```mermaid
flowchart TD
    subgraph SRC["Where faults come from"]
        S1["CheckLimits (every tick)"]
        S2["SampleEncoder: gap, jump"]
        S3["SettledDecision / RunVerify"]
        S4["nFAULT interrupt (ignored while APP_IGNORE_DRIVER_FAULTS = 1)"]
        S5["Command: x during a move, dt failure, not stationary"]
    end
    SRC --> L["FaultManager_Latch(code)<br/>MotorDriver_Stop, keep first code"]
    L --> E["MotionControl_EnterFault<br/>post failed result, phase = FAULTED"]
    E --> P["main loop prints [RUN] ... and [FAULT] text"]
    P --> C{"user: clear"}
    C --> C1["Commands_RearmChecksPass:<br/>ready, MA782 config ok, output stationary"]
    C1 -- ok --> C2["FaultManager_Clear -> IDLE"]
    C1 -- fail --> C3["[CLEAR] rejected"]
```

CheckLimits tests, in order:

| Check | Fault | Typical cause |
|---|---|---|
| IPROPI current > limit (skipped when faults ignored) | `FAULT_DRIVER_OVERCURRENT` | stall, wrong sense resistor |
| peak > target + 1 count | `FAULT_OVERTRAVEL` | stop too late / burst |
| peak - progress > reverse limit | `FAULT_ROLLBACK` | gear recoil bigger than the allowance |
| move time > 30 s | `FAULT_MOVE_TIMEOUT` | too slow |
| no 2-raw progress for 1.2 s | `FAULT_NO_PROGRESS_TIMEOUT` | stuck / no power / jam |

---

## 11. Motor driver - from duty to the two bridge inputs

```mermaid
flowchart TD
    A["MotorDriver_RunForward / RunReverse(duty)"] --> B["limit duty to 0..100 %<br/>on_counts = duty x period"]
    B --> C{"APP_COMPLEMENTARY_DRIVE"}
    C -- "0 (slow decay, current default)" --> D{"on_counts = 0?"}
    D -- yes --> E["MotorDriver_Coast<br/>AIN1 = 0, AIN2 = 0"]
    D -- no --> F["forward: AIN1 held high, AIN2 = PWM<br/>(AIN2 low = drive, high = brake)<br/>reverse: swapped"]
    C -- "1 (TMC4671 style)" --> G["AIN1 = PWM1, AIN2 = PWM2 on the same compare<br/>AIN2 is always the inverse of AIN1<br/>net voltage = duty x VM, 0 % = 50 % chop"]
    H["MotorDriver_Brake"] --> H1["AIN1 = 1, AIN2 = 1 (both outputs low)"]
    I["MotorDriver_Coast / Stop"] --> I1["AIN1 = 0, AIN2 = 0<br/>driver sleeps after about 1 ms"]
    F --> W["wake delay 150 us if the driver was asleep"]
    G --> W
```

DRV8411A inputs: `00` coast, `10` forward, `01` reverse, `11` brake.

---

## 12. Stationary check and direction test

```mermaid
flowchart TD
    subgraph STAT["z / ast: stationary check"]
        A1["tick suspended, bridge off"] --> A2["wait 20 ms"]
        A2 --> A3["origin = angle"]
        A3 --> A4["50 samples, 2 ms apart<br/>delta from origin, track min and max"]
        A4 --> A5{"max - min <= 12 raw?"}
        A5 -- yes --> A6["PASS, mean angle returned"]
        A5 -- no --> A7["FAIL, print all samples"]
    end
    subgraph DIR["dtNN: direction test (encoder sign)"]
        B1["stationary check"] --> B2["drive forward at NN % for at most 80 ms<br/>sum the angle steps; abort on a jump"]
        B2 --> B3["brake 12 ms, coast, stationary check again"]
        B3 --> B4{"settled and travel >= 32 raw?"}
        B4 -- yes --> B5["encoder_invert = (travel < 0)<br/>PASS"]
        B4 -- no --> B6["FAULT_CALIBRATION<br/>print travel, jump, settled, nFAULT, peak mA"]
    end
```

---

## 13. Volumetric and repeatability schedulers

```mermaid
flowchart TD
    A["test / vol"] --> B{"ready, idle, no fault,<br/>MA782 ok, g10.61 qualified since reboot?"}
    B -- no --> X["[TEST] BLOCKED"]
    B -- yes --> C["batch = 0, move = 0<br/>first start in 210 ms"]
    C --> D["WAIT_PREFLIGHT"]
    D --> E["160 ms before the start time:<br/>check MA782 config and stationary"]
    E -- fail --> STOP["stop schedule with a reason"]
    E -- ok --> F["wait (feeding watchdog) until the exact start time"]
    F --> G["MotionControl_StartMove(10.61 deg)"]
    G --> H["MOVE_ACTIVE"]
    H --> I{"result"}
    I -- failed --> STOP
    I -- passed --> J{"start error > 25 ms<br/>or cadence missed?"}
    J -- yes --> STOP
    J -- no --> K{"move < 10 of batch?"}
    K -- yes --> L["next start = previous ACTUAL start + 4 s"]
    L --> D
    K -- no --> M{"batch < 10?"}
    M -- yes --> N["next start = move end + 180 s<br/>next batch"]
    N --> D
    M -- no --> DONE["[TEST] VOLUMETRIC COMPLETE"]
```

Repeatability (`r<deg>`): 10 moves, 4 s of rest after each completion, stops at the first fault.

---

## 14. NodeMCU + TMC4671 to STM32 - where each function went

```mermaid
flowchart LR
    subgraph OLD["NodeMCU (ESP8266) + TMC4671"]
        O1["tmcWrite / tmcRead / initializeBridge<br/>setForwardVoltage / stopBridge / brakeBridge"]
        O2["maFrame / maReadReg / maWriteReg<br/>maConfigurationPass"]
        O3["stationaryAngle"]
        O4["sampleMotion, updateSpeedControl,<br/>tickMotion, beginBrake, startPulse, settledDecision"]
        O5["startMove / armMoveFromOrigin"]
        O6["latchFault"]
        O7["calibrateDirection"]
        O8["handleCommand / pollSerial / printResult"]
        O9["startVolumetricTest ... pollVolumetricTest"]
        O10["setup / loop"]
    end
    subgraph NEW["STM32U575 + DRV8411A"]
        N1["motor_driver.c + bsp_hw.c<br/>PWM straight to AIN1 / AIN2 (no TMC)"]
        N2["ma782_encoder.c + bsp_hw.c (SPI1)"]
        N3["MotionControl_MeasureStationaryLocked"]
        N4["motion_control.c: SampleEncoder, RunSpeedLoop,<br/>MotionControl_Tick, RunPhase, BeginBrake,<br/>StartPulse, SettledDecision"]
        N5["AppMove_Start (app_state.c)<br/>MotionControl_StartMove"]
        N6["fault_manager.c: FaultManager_Latch"]
        N7["MotionControl_CalibrateDirection<br/>(dt command)"]
        N8["app_commands.c, console.c, app_main.c"]
        N9["app_scheduler.c"]
        N10["App_Init / App_Run"]
    end
    O1 --> N1
    O2 --> N2
    O3 --> N3
    O4 --> N4
    O5 --> N5
    O6 --> N6
    O7 --> N7
    O8 --> N8
    O9 --> N9
    O10 --> N10
```

What the TMC4671 used to do: generate the PWM and apply a requested voltage (`UQ_UD_EXT`, open-loop voltage mode)
to the DRV8411A inputs. All control loops (speed PI, position logic, braking, pulses) were already software on the NodeMCU.
On the STM32 the PWM is generated by timer TIM1 directly and the TMC4671 is removed.
