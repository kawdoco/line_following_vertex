<div align="center">

# 🤖 Line Follower Robot — ESP32 v8.0
### *Intelligent Auto-Tune Edition*

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue?logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%2F%20Arduino-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20IDE-teal)](https://www.arduino.cc/en/software)
[![Motor Driver](https://img.shields.io/badge/Motor%20Driver-TB6612FNG-orange)](https://www.pololu.com/product/713)
[![Sensor](https://img.shields.io/badge/Sensor-QTR--8RC-green)](https://www.pololu.com/product/961)
[![License](https://img.shields.io/badge/License-MIT-brightgreen)](LICENSE)
[![Version](https://img.shields.io/badge/Version-v8.0-blueviolet)](CHANGELOG.md)
[![Competition](https://img.shields.io/badge/🏆%20Award-3rd%20Place%20%7C%20BCI%20Campus%20Line%20Following%20Competition-gold)](#-competition-achievement)

---

> **A production-grade, competition-winning autonomous line-following robot built on the ESP32, featuring a fully self-tuning PID controller, real-time Wi-Fi telemetry dashboard, dual-profile intelligent auto-tuning via Coordinate Descent (Twiddle), hardware look-ahead anticipatory braking, and persistent crash-recovery — all engineered from scratch.**

---

### 🏆 Competition Achievement

| Event | Organiser | Result | Prize |
|---|---|---|---|
| Line Following Robot Competition | BCI Campus | **3rd Place — 2nd Runner-Up** | **LKR 10,000** |

<br>

![Award Ceremony](/robot_image.jpg)
*Award ceremony — 3rd place podium finish at the BCI Campus Line Following Competition*

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Competition Achievement](#-competition-achievement)
- [Key Features & Innovations](#-key-features--innovations)
- [Hardware Architecture](#-hardware-architecture)
- [Circuit Connections](#-circuit-connections)
- [Software Architecture](#-software-architecture)
- [PID Configuration & Tuning](#-pid-configuration--tuning)
- [Wi-Fi Web Interface](#-wi-fi-web-interface)
- [Auto-Tune System (AT8)](#-auto-tune-system-at8)
- [Bug Fixes & Optimisations](#-bug-fixes--optimisations)
- [Installation & Flashing](#-installation--flashing)
- [Usage Guide](#-usage-guide)
- [File Structure](#-file-structure)
- [Tech Stack](#-tech-stack)
- [Gallery](#-gallery)
- [License](#-license)

---

## 🔍 Overview

This project is a high-performance autonomous line-following robot designed for competitive robotics. The firmware, written entirely in C++ for the Arduino/ESP32 ecosystem, has evolved through **8 major versions** culminating in the **Intelligent Auto-Tune Edition (v8.0)** — a system that autonomously discovers its own optimal PID parameters using Coordinate Descent (Twiddle), adapts to different track geometries in real-time, and persists its learned state across power cycles using non-volatile storage.

The robot operates at an overvolted **11.4V** (boosted from a 7.4V 2S Li-ion pack via an MP1584 buck-boost converter), driving N20 6V motors beyond their nominal rating to achieve competitive speeds while maintaining precise line tracking.

---

## ✨ Key Features & Innovations

### 1. 🧠 Intelligent Dual-Profile Adaptive Auto-Tune (AT8)

The most significant engineering achievement of this project. Rather than relying on manually tuned PID constants, the robot autonomously discovers optimal parameters using a **Coordinate Descent (Twiddle)** algorithm with two independent PID profiles:

- **Straight Profile** (`kpS`, `kdS`) — tuned for high-speed low-error segments
- **90° Turn Profile** (`kpT`, `kdT`) — tuned for aggressive cornering

A terrain state machine (using sensor error magnitude + look-ahead bits) continuously classifies the robot's current environment as `STRAIGHT`, `TURN_90`, or `UNKNOWN`. IAE (Integral of Absolute Error) fitness evaluation is **terrain-filtered**, meaning corner dynamics never contaminate straight evaluations and vice versa — a key innovation that dramatically improves parameter quality.

### 2. 📈 Progressive Speed Escalation

Rather than targeting a fixed speed, the AT8 system escalates PWM output in steps (`AT8_SPD_STEP = 10 PWM/step`) starting from `AT8_SPD_START = 200`, with **no artificial ceiling**. The robot keeps pushing speed until the Twiddle convergence fails or the mechanical ceiling is reached. This means the robot autonomously discovers the fastest speed it can reliably handle.

### 3. 📡 Hardware Look-Ahead (74HC165 + Dedicated IR Sensors)

Four dedicated IR sensors (S1–S4) connected via a **74HC165 parallel-load shift register** provide anticipatory look-ahead at two distances:
- **6 cm ahead** — triggers smooth geometry-based speed scaling and ABS-style braking
- **2 cm ahead** — acts as a fail-safe hard brake if speed is still excessive

This architecture completely separates junction detection from line-following sensor processing, eliminating the latency that would otherwise cause the robot to overshoot corners.

### 4. 🚗 ABS-Style Electronic Braking

Inspired by automotive ABS, the braking system uses the **TB6612FNG's active electronic brake mode** (IN1=IN2=HIGH, PWM=255 — which shorts motor terminals to lock the shaft) in timed lock/release cycles before junctions. The cycle timing (`ABS_LOCK_TICKS = 10`, `ABS_RELEASE_TICKS = 5`) was derived from the N20 motor's measured lock and release latencies (~50 ms each), ensuring safe operation without gear strip.

### 5. 🔁 3-Mode Hybrid PID Controller

The robot implements three independent PID modes selected dynamically by current error magnitude:

| Mode | Track Condition | Error Range | Default Kp | Default Kd | Base PWM |
|---|---|---|---|---|---|
| **Mode 0** | Straight | `err < 800` | 0.55 | 9.0 | 110 |
| **Mode 1** | Gentle Curve | `800 ≤ err < 1800` | 0.90 | 14.0 | 70 |
| **Mode 2** | Sharp Corner | `err ≥ 1800` | 1.40 | 22.0 | 48 |

The D-term uses **Exponential Moving Average (EMA) filtering** (`D_EMA_ALPHA = 0.25`) to suppress high-frequency derivative noise, which would otherwise be amplified by the high motor voltage.

### 6. 📡 Real-Time Wi-Fi Telemetry Dashboard

The robot hosts a **Wi-Fi Access Point** and serves a full-featured web dashboard at `http://192.168.4.1`. Live telemetry is pushed over WebSocket at 10 Hz without polling:
- Live PID error, output, and PWM values
- All 8 QTR sensor readings
- Auto-tune phase, terrain, fitness score, and dp-convergence indicator
- Full event log streamed in real-time
- Remote PID parameter update and save to NVS
- Remote calibration and run/stop control

The web task runs on **Core 0** while the control loop runs on **Core 1** — Xtensa dual-core isolation prevents web activity from ever introducing control-loop jitter.

### 7. 💾 Crash-Recovery State Persistence (NVS)

Every speed escalation step and the final result are checkpointed to ESP32 Non-Volatile Storage (`Preferences` library, namespace `"at8"`). On any unexpected restart (power loss, watchdog, E-brake), the auto-tuner resumes from the last saved checkpoint rather than restarting from scratch — a critical feature for long tuning sessions.

### 8. 📊 Wobble/Oscillation RMS Analysis

A rolling 80-sample RMS window continuously monitors error magnitude. If `RMS > WOBBLE_HEAVY_RMS (330)` during the settle phase, a micro-step Kp reduction fires (`WOBBLE_MICRO_STEP = 0.005`) to prevent runaway oscillation at the elevated 11.4V supply — a hardware-aware safety mechanism that standard Ziegler-Nichols auto-tune does not provide.

### 9. 🔒 Welford Online Variance — Straight Profile Lock

Welford's numerically stable online algorithm computes running variance of the straight-segment error. When variance falls below `3200 units²` for three consecutive 200-sample windows, the straight profile is **permanently locked**, freeing the tuner to focus exclusively on the turn profile. This prevents the turn-tuning phase from accidentally degrading already-optimal straight-line performance.

### 10. ⚡ Emergency Hardware Brake (GPIO 34 ISR)

A physical emergency brake button on GPIO 34 is serviced by an `IRAM_ATTR` ISR (runs directly from IRAM, immune to flash cache misses), which sets a flag polled every 5 ms tick. On trigger: TB6612FNG active electronic brake, 200 ms hold, then coast-stop, NVS checkpoint save, and state transition to IDLE — all within a single tick cycle.

### 11. 🗺️ Node Map / Grid Memory (FIFO)

A 16-entry FIFO records junction history (available directions bitmask, directions tried, entry direction). When a U-turn is detected and the robot revisits a junction within an 80-tick window, previously attempted directions are excluded, enabling **systematic dead-end avoidance** without an external map or GPS.

### 12. 🔹 45° Dummy Line Filter

An angular-rate filter (`DUMMY45_RATE_THR = 900 units/tick`) detects diagonal line artifacts that would otherwise register as false junction crossings. When a rate spike is confirmed over 4 consecutive ticks, junction detection is suppressed for 25 ticks — eliminating a major source of false positives on complex competition tracks.

### 13. ⚙️ Velocity Ramp (Asymmetric Accel/Decel)

All PWM transitions are smoothed through an asymmetric ramp: `VEL_ACCEL_RATE = 25 PWM/tick` and `VEL_DECEL_RATE = 25 PWM/tick` (~3000–5000 PWM/s). This prevents the mechanical shock and wheel slip that would otherwise occur when jumping between PID modes at high speed, and protects the drivetrain during emergency braking.

### 14. 🔋 Overvolted Drivetrain with MP1584 Dual Power Rails

The power system uses two MP1584 buck converters operating in opposite modes:
- **Power Boost rail**: 7.4V → 11.4V for motors (overvolted for maximum torque/speed)
- **Step-Down rail**: 7.4V → 5V for logic and sensors

The 2S Li-ion pack is managed by a **2S BMS** with the **TP5100 2A charging module** supporting direct 12V charger input.

---

## 🔧 Hardware Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                    POWER SYSTEM                               │
│  [2× 18650 3.7V] → [2S BMS] → 7.4V                          │
│       7.4V → [MP1584 Boost]  → 11.4V (Motors)               │
│       7.4V → [MP1584 Buck]   →   5V  (Logic/Sensors)        │
│  [TP5100 Charger Module]     ← 12V DC input                  │
└───────────────────────────────────────────────────────────────┘
              │                        │
              ▼ 11.4V                  ▼ 5V
┌─────────────────────┐    ┌──────────────────────────────────┐
│    TB6612FNG        │    │    ESP32 DevKitV1 (Control MCU)  │
│  Motor Driver       │    │    Dual-Core 240 MHz Xtensa       │
│  IN1/IN2/PWM A&B    │    │    Core 1: PID Control Loop      │
│  → Left N20 Motor   │    │    Core 0: Wi-Fi / WebSocket     │
│  → Right N20 Motor  │    │    NVS: PID + AT8 Persistence    │
└─────────────────────┘    └──────────────────────────────────┘
                                        │
              ┌─────────────────────────┼──────────────────────┐
              ▼                         ▼                      ▼
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│   QTR-8RC (REGLETA) │  │  74HC165N Shift Reg │  │   Emergency Brake   │
│   8× IR Sensors     │  │  4× Look-Ahead IR   │  │   GPIO 34 (IRAM ISR)│
│   Line Position     │  │  6cm + 2cm Detect   │  │   Active LOW        │
│   Error Calculation │  │  SPI-style Readout  │  │   Ext. 10kΩ Pull-Up │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
```

> 📄 **Full Hardware Diagram**: See [`docs/hardware_diagram.pdf`](docs/hardware_diagram.pdf) (EasyEDA schematic, Rev 1.0, 2026-05-05).
> To view: open in any PDF reader or import into [EasyEDA](https://easyeda.com) using the source schematic.

---

## 🔌 Circuit Connections

### ESP32 → TB6612FNG Motor Driver

| ESP32 GPIO | TB6612FNG Pin | Function |
|---|---|---|
| GPIO 25 | PWMA | Left motor PWM (LEDC Ch A) |
| GPIO 26 | AIN1 | Left motor direction 1 |
| GPIO 27 | AIN2 | Left motor direction 2 |
| GPIO 2  | PWMB | Right motor PWM (LEDC Ch B) |
| GPIO 12 | BIN1 | Right motor direction 1 |
| GPIO 15 | BIN2 | Right motor direction 2 |
| 11.4V   | VM   | Motor supply voltage |
| 3.3V    | VCC  | Logic supply |

### ESP32 → QTR-8RC (Line Sensor Array)

| ESP32 GPIO | QTR-8RC Sensor | Position |
|---|---|---|
| GPIO 23 | S0 | Leftmost |
| GPIO 22 | S1 | — |
| GPIO 21 | S2 | — |
| GPIO 17 | S3 | — |
| GPIO 16 | S4 | — |
| GPIO 14 | S5 | — |
| GPIO 13 | S6 | — |
| GPIO 4  | S7 | Rightmost |
| GPIO 33 | IR LED EN | Sensor IR enable |

### ESP32 → 74HC165 (Look-Ahead Shift Register)

| ESP32 GPIO | 74HC165 Pin | Function |
|---|---|---|
| GPIO 5  | SH/LD (Pin 1) | Parallel load latch |
| GPIO 18 | CLK (Pin 2)   | Clock |
| GPIO 32 | Q7 (Pin 9)    | Serial data out |

### Miscellaneous

| ESP32 GPIO | Component | Notes |
|---|---|---|
| GPIO 34 | Emergency Brake Button | Input-only. **External 10kΩ pull-up to 3.3V required.** Active LOW. |

> ⚠️ GPIO 34 on the ESP32 has **no internal pull-up resistor**. An external 10kΩ resistor from GPIO 34 to 3.3V is **mandatory** for the emergency brake to function.

---

## 🏗️ Software Architecture

The firmware is structured around FreeRTOS tasks pinned to specific cores to guarantee real-time control loop timing:

```
┌──────────────────────────────────────────────────────────────┐
│  ESP32 — FreeRTOS Dual-Core Task Allocation                  │
├─────────────────────────┬────────────────────────────────────┤
│  Core 1 (Priority 10)   │  Core 0 (Priority 3)              │
│  taskControl()          │  taskWeb()                        │
│  ─────────────────────  │  ─────────────────────────────    │
│  • QTR sensor read      │  • WebSocket broadcast @ 10 Hz   │
│  • 74HC165 look-ahead   │  • AT success broadcast           │
│  • Error calculation    │  • PID fetch broadcast            │
│  • 3-Mode PID output    │  • Ring-buffer log push           │
│  • Motor drive          │  • HTTP request handling          │
│  • AT8 Twiddle engine   │  • Wi-Fi AP management            │
│  • Navigation state SM  │                                   │
│  • E-Brake ISR poll     │  loop() — deliberately empty      │
│  • NVS checkpointing    │  (vTaskDelay 1000ms only)         │
│  5 ms tick (200 Hz)     │                                   │
└─────────────────────────┴────────────────────────────────────┘
```

### State Machines

**Robot State** (`State` enum):
`IDLE` → `CALIBRATING` → `RUNNING` ↔ `AUTO_TUNING` → `STOPPED`

**Navigation State** (`NavState` enum):
`LINE_FOLLOW` ↔ `DASHED_FWD` ↔ `PIVOT_SEARCH` ↔ `AT_INTERSECTION` ↔ `INTER_TURN_L/R` ↔ `DEAD_END_UTURN`

**AT8 Phase** (`AT8Phase` enum):
`INIT` → `STR_TUNE` → `STR_LOCKED` → `TRN_TUNE` → `SPD_STEP` → `COMPLETE` / `ABORTED`

---

## ⚙️ PID Configuration & Tuning

### Competition-Tuned Default Parameters (11.4V, N20 Motors)

```cpp
// Mode 0 — Straight sections
PIDParam Mode0 = { kP: 0.55, kI: 0.000, kD: 9.0,  base: 110, top: 140, low: 60 };

// Mode 1 — Gentle curves (|error| 800–1800)
PIDParam Mode1 = { kP: 0.90, kI: 0.000, kD: 14.0, base: 70,  top: 90,  low: 40 };

// Mode 2 — Sharp corners (|error| ≥ 1800)
PIDParam Mode2 = { kP: 1.40, kI: 0.000, kD: 22.0, base: 48,  top: 68,  low: 28 };
```

### Key Tuning Constants

| Parameter | Value | Description |
|---|---|---|
| `QTR_SETPOINT` | 3500 | Target sensor position (centre of 8-sensor array × 1000) |
| `HYBRID_MODE1_ERR` | 800.0 | Mode 0→1 error threshold |
| `HYBRID_MODE2_ERR` | 1800.0 | Mode 1→2 error threshold |
| `D_EMA_ALPHA` | 0.25 | D-term EMA filter coefficient |
| `MOTOR_DEADBAND` | 25 | Minimum PWM to overcome motor stiction |
| `VEL_ACCEL_RATE` | 25 | PWM ramp acceleration rate (units/tick) |
| `VEL_DECEL_RATE` | 25 | PWM ramp deceleration rate (units/tick) |
| `LEDC_HZ` | 20000 | Motor PWM frequency (20 kHz, inaudible) |

### Adjusting PID via Web UI

All three PID mode parameters can be updated live without reflashing:
1. Connect to `LineFollower` Wi-Fi network (password: `robot1234`)
2. Open `http://192.168.4.1` in a browser
3. Adjust Kp, Ki, Kd, Base, Top, Low for any mode
4. Click **Save to NVS** — values persist across power cycles

---

## 🌐 Wi-Fi Web Interface

| Property | Value |
|---|---|
| SSID | `LineFollower` |
| Password | `robot1234` |
| IP Address | `192.168.4.1` |
| Protocol | WebSocket + HTTP |
| Update Rate | 10 Hz telemetry push |

### Web UI Commands

| Command | Action |
|---|---|
| `CALIBRATE` | Run 3-pass sensor calibration sweep |
| `START` / `STOP` | Begin/end line following |
| `AT_START` | Launch Auto-Tune sequence |
| `AT_STOP` | Abort Auto-Tune |
| `AT8_RESET_NVS` | Clear AT8 checkpoint (force cold start) |
| `SET_PID` | Update PID parameters live |
| `MANUAL_BRAKE` | Trigger ABS braking sequence |
| `SET_AT_KP_START` | Set Twiddle Kp initial seed per mode |
| `SET_AT_KP_STEP` | Set Twiddle dpKp initial step per mode |

---

## 🤖 Auto-Tune System (AT8)

The AT8 Intelligent Auto-Tune system is the core engineering innovation of this project.

### How It Works

```
1. INIT       Read NVS checkpoint (resume) or start fresh
      ↓
2. STR_TUNE   Twiddle optimises kpS, kdS on straight segments
              (IAE measured only during STRAIGHT terrain)
      ↓
3. STR_LOCKED Welford variance < 3200 for 3 windows → lock straight profile
      ↓
4. TRN_TUNE   Twiddle optimises kpT, kdT on 90° corners
              (IAE measured only during TURN_90 terrain)
      ↓
5. SPD_STEP   Both profiles converged → save NVS checkpoint → +10 PWM
              → repeat from STR_TUNE at new speed
      ↓
6. COMPLETE   Mechanical ceiling reached → apply final result → normal run
```

### Twiddle Coordinate Descent Parameters

| Parameter | Value | Description |
|---|---|---|
| `TWIDDLE_EVAL_TICKS` | 200 | Terrain-filtered IAE evaluation window |
| `TWIDDLE_SETTLE_TICKS` | 60 | Dead ticks before IAE starts (robot stabilises) |
| `TWIDDLE_DP_GROW` | 1.05× | Grow dp on improvement |
| `TWIDDLE_DP_SHRINK` | 0.95× | Shrink dp on no improvement |
| `TWIDDLE_DONE_SUM` | 0.005 | dpKp + dpKd < this → converged |
| `AT8_SPD_START` | 200 | Initial PWM for escalation |
| `AT8_SPD_STEP` | 10 | PWM increment per speed step |

---

## 🛠️ Bug Fixes & Optimisations

The following issues were identified, root-caused, and resolved during development. Each fix is tagged in the source code for traceability.

### [BUG2-FIX] — Immutable Auto-Tune Kp Start / Step

**Problem**: `AT_KP_START` and `AT_KP_STEP` were `#define` constants, making them impossible to update from the Web UI at runtime. This forced reflashing to adjust auto-tune sensitivity for different track conditions.

**Fix**: Converted to runtime-mutable global arrays `gAtKpStart[3]` and `gAtKpStep[3]`, one entry per PID mode. The Web UI commands `SET_AT_KP_START` and `SET_AT_KP_STEP` now update these live.

---

### [FIX-PERMODE] — Per-Mode AT Checkpoint Mismatch on Resume

**Problem**: The NVS checkpoint did not store which PID mode (`M0`/`M1`/`M2`) initiated the auto-tune session. On resume after a power cycle, the tuner would load checkpoint values that belonged to a different mode, contaminating the starting parameters for the resumed session.

**Fix**: Added `at8mod` key to the NVS checkpoint. `at8NvsLoad()` now accepts a `reqMode` parameter and returns `false` (triggering a fresh start) if the saved mode does not match the requested mode.

---

### [FIX] — velRamp() Incorrect Direction at Reverse Speeds

**Problem**: The velocity ramp function compared `target > current` directly, which produced incorrect rate selection when PWM values were negative (reverse drive). A decelerating reverse command was incorrectly classified as acceleration.

**Fix**: Changed rate selection to compare `abs(target) > abs(current)` — acceleration is defined as increasing shaft speed regardless of direction, and deceleration as decreasing it.

---

### [U3-FIX] — Pivot Search Premature Timeout

**Problem**: The pivot search (line-lost recovery spin) had a fixed tick timeout. On slower tracks or when the line was genuinely far, the robot would abort the search before recovering the line and enter an irrecoverable state.

**Fix**: Raised `UTURN_MAX_TICKS` from a conservative limit to 250, and changed the pivot search to spin indefinitely toward `lastKnownSide` with no tick ceiling during active recovery.

---

### [U4-FIX] — Hard Look-Ahead Speed Caps Causing Abrupt Deceleration

**Problem**: Look-ahead speed targets (`LA_BRAKE_6CM_SPEED`, `LA_PREP_2CM_SPEED`) were applied as hard instantaneous caps, causing sudden PWM drops that destabilised the PID controller and occasionally stalled motors.

**Fix**: Replaced hard caps with geometry-based smooth scaling — `effBase` is linearly interpolated (`lerp`) toward the target speed over `LA_GEOM_RAMP_TICKS = 30` ticks based on sensor proximity, giving the velocity ramp time to absorb the transition.

---

### [U7-FIX] — 45° Diagonal Line False Junction Detections

**Problem**: Competition tracks with diagonal join lines generated sudden large error rate spikes (`|de/dt| > 900 units/tick`) that the junction detector misidentified as valid intersection crossings, causing incorrect turns.

**Fix**: The `DUMMY45` filter monitors `|de/dt|` and, when a spike is sustained for `DUMMY45_CONFIRM_TICKS = 4` ticks, suppresses junction detection for a `DUMMY45_COOL_TICKS = 25` tick cooldown window.

---

### [U9-FIX] — Hybrid Braking Premature Recovery

**Problem**: The hybrid braking system restored full speed too aggressively after a braking event, causing the robot to repeatedly brake and recover on the same corner rather than smoothly cornering.

**Fix**: Introduced a hysteresis recovery condition: speed is only restored when `|error| < HYBRID_RECOVER_ERR_THR (20)` for `HYBRID_RECOVER_TICKS = 10` **consecutive** ticks, preventing premature recovery from a transient error dip.

---

### [U10-FIX] — Mode Switching Shock at PID Mode Boundaries

**Problem**: Switching between PID modes 0, 1, and 2 instantly changed the `base` PWM target (e.g. 110 → 70 → 48), which bypassed the velocity ramp and sent a sharp PWM transient to the motors, causing wheel judder at high speeds.

**Fix**: The velocity ramp (`velRamp()`) is applied **after** mode selection. The `current` PWM value is carried across mode transitions, so the ramp absorbs the base-speed change over multiple ticks rather than applying it instantaneously.

---

### [AT8-2] — Uncontrolled Oscillation at 11.4V During Auto-Tune

**Problem**: Standard Ziegler-Nichols relay testing at 11.4V caused the robot to exhibit heavy uncontrolled oscillation (`RMS > 330`) during settle phases due to the motor system's high gain at elevated voltage.

**Fix**: The rolling 80-sample RMS wobble detector fires a `WOBBLE_MICRO_STEP = 0.005` Kp reduction each tick that the robot remains in heavy wobble, gradually damping the oscillation before the next IAE measurement window opens.

---

### [FIX] — NVS AT8 Checkpoint Written with Zero Values on Broadcast Race

**Problem**: The AT8 success broadcast task occasionally read `gAtResult.currentKp = 0` before the control task had written the final values, saving a zero checkpoint to NVS.

**Fix**: The broadcast function checks `sentKp > 0.0f || sentKu > 0.0f` before serialising. If values are zero, the broadcast is re-queued for the next 50 ms tick, ensuring the checkpoint is only written once valid values are confirmed.

---

## 🚀 Installation & Flashing

### Prerequisites

- [Arduino IDE 2.x](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- ESP32 Arduino Core (`esp32` by Espressif, v2.x or later)

### Required Libraries

Install via Arduino Library Manager or PlatformIO:

```
AsyncTCP               by dvarrel (or me-no-dev)
ESPAsyncWebServer      by lacamera / me-no-dev
ArduinoJson            by Benoit Blanchon  (v6.x)
```

### Flashing Steps

1. Clone this repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/line-follower-robot.git
   cd line-follower-robot
   ```

2. Open `src/main.cpp` (or the `.ino` file) in Arduino IDE.

3. Select board: **ESP32 Dev Module** (or ESP32 DevKitV1).

4. Set partition scheme to **Huge APP** (required for firmware size).

5. Upload speed: **921600** baud recommended.

6. Flash:
   ```
   Sketch → Upload  (Ctrl+U)
   ```

7. Open Serial Monitor at **115200 baud** to verify startup logs.

### First-Run Checklist

- [ ] Connect to `LineFollower` Wi-Fi
- [ ] Open `http://192.168.4.1` in browser
- [ ] Click **Calibrate** — robot will sweep left/right over the line
- [ ] Verify all 8 sensors show valid calibration ranges (min/max spread > 100µs)
- [ ] Click **Start** to begin line following
- [ ] (Optional) Click **AT Start** to run intelligent auto-tune

---

## 📖 Usage Guide

### Normal Operation

| Step | Action |
|---|---|
| 1 | Power on. Robot enters `IDLE` state. |
| 2 | Connect phone/laptop to `LineFollower` Wi-Fi. |
| 3 | Open `http://192.168.4.1`. |
| 4 | Press **Calibrate** and sweep the robot over the line (3 passes auto). |
| 5 | Place robot on the start position. |
| 6 | Press **Start**. |

### Auto-Tune Operation

| Step | Action |
|---|---|
| 1 | Complete calibration first. |
| 2 | Place robot on a track with both straights and 90° corners. |
| 3 | Press **AT Start** and select PID mode (M0/M1/M2). |
| 4 | Monitor the telemetry panel — watch terrain, fitness, and dpSum. |
| 5 | When `AT8Phase = COMPLETE`, press **Stop**. New PID values are in NVS. |
| 6 | Press **Save PID** to confirm to the main PID namespace. |

### Emergency Stop

Press the physical button on GPIO 34 at any time. The TB6612FNG will apply active electronic brake immediately. All state is saved to NVS.

---

## 📁 File Structure

```
line-follower-robot/
├── src/
│   └── main.cpp               # Full firmware (v8.0 Intelligent AT Edition)
├── docs/
│   └── hardware_diagram.pdf   # EasyEDA schematic (Rev 1.0)
├── /
│   ├── robot_image.jpg        # Robot photo
│   └── competition_certificate.jpg  # Award certificate
├── README.md
└── LICENSE
```

---

## 🧰 Tech Stack

### Microcontroller & Firmware

| Component | Details |
|---|---|
| MCU | ESP32 DevKitV1 (Dual-Core Xtensa LX6, 240 MHz) |
| Framework | Arduino / ESP-IDF via Arduino Core |
| Language | C++ (embedded, no heap allocation in control loop) |
| RTOS | FreeRTOS (dual-core task pinning) |
| NVS | ESP32 `Preferences` library (key-value flash storage) |
| Web Server | ESPAsyncWebServer (non-blocking, event-driven) |
| WebSocket | AsyncTCP WebSocket (real-time bi-directional) |
| JSON | ArduinoJson v6 (stack-allocated `StaticJsonDocument`) |

### Hardware Components

| Component | Part Number | Role |
|---|---|---|
| Microcontroller | ESP32 DevKitV1 | Main compute, Wi-Fi AP, dual-core RTOS |
| Motor Driver | TB6612FNG | Dual H-bridge, active electronic brake |
| Line Sensor | QTR-8RC (REGLETA) | 8-element reflectance array, RC timing |
| Shift Register | SN74HC165N | 4× look-ahead IR sensors parallel-to-serial |
| Motors | N20 6V DC (×2) | Driven at 11.4V for max speed |
| Buck-Boost | MP1584 (×2) | 7.4V→11.4V boost; 7.4V→5V step-down |
| Battery | 18650 3.7V Li-ion (×2) | 2S configuration, 7.4V nominal |
| BMS | 2S BMS Module | Overcharge/over-discharge protection |
| Charger | TP5100 2A Module | 12V DC input charging |
| Emergency Brake | Momentary button + 10kΩ | GPIO 34, active LOW, IRAM ISR |

---

## 🖼️ Gallery

| Competition Day | Robot on Track | Hardware Top View |
|---|---|---|
| ![Award](/robot_image.jpg) | *(Competition run)* | *(Top view)* |

> Full resolution images available in the `/` directory.

---

## 📜 Changelog

### v8.0 — Intelligent Auto-Tune Edition
- Dual-profile Twiddle/Coordinate Descent auto-tuner (AT8-1 through AT8-5)
- NVS crash-recovery state persistence
- 80-sample RMS wobble/oscillation detector with micro-step Kp reduction
- Welford online variance straight-profile locking
- Progressive speed escalation with no artificial PWM ceiling
- Emergency brake ISR on GPIO 34 (IRAM_ATTR)
- Per-mode AT checkpoint with mode-match guard on resume

### v7.0
- Upgraded hybrid braking with recovery hysteresis (U9)
- Velocity smoothing with asymmetric accel/decel ramp (U10)
- Node map / FIFO grid memory for junction history (U5)

### v6.x — v5.x
- Hardware look-ahead (74HC165 shift register)
- ABS-style electronic braking cycles
- 45° dummy line filter
- Ziegler-Nichols relay auto-tune foundation
- 3-mode hybrid PID controller

---

## 📄 License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.

---

<div align="center">

**Built with precision. Tuned with intelligence. Competed with confidence.**

*Line Follower Robot v8.0 — Intelligent Auto-Tune Edition*

[![GitHub Stars](https://img.shields.io/github/stars/YOUR_USERNAME/line-follower-robot?style=social)](https://github.com/YOUR_USERNAME/line-follower-robot)

</div>
