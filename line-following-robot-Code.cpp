/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  LINE FOLLOWER ROBOT — ESP32 v8.0  (INTELLIGENT AT EDITION)        ║
 * ║  TB6612FNG + QTR-8RC + 74HC165 + N20 6V Motors @ 11.40V (OVERV.)  ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  VM = 11.40V  |  Dual-Profile Adaptive AT  |  3-Mode Dynamic PID   ║
 * ║  Wi-Fi AP:  SSID="LineFollower"  PASS="robot1234"                   ║
 * ║  Web UI:    http://192.168.4.1                                      ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  v8.0 — INTELLIGENT AUTO-TUNE EDITION (based on v7.0)              ║
 * ║                                                                     ║
 * ║  [AT8-1] STATE PERSISTENCE & RESUME:                                ║
 * ║    NVS namespace "at8" saves speed, both PID profiles, lock         ║
 * ║    state and phase every escalation step. On crash/restart the      ║
 * ║    tuner resumes from last checkpoint — never from scratch.         ║
 * ║                                                                     ║
 * ║  [AT8-2] OSCILLATION / WOBBLE ANALYSIS:                             ║
 * ║    Rolling RMS error window (80 ticks) distinguishes "hunting        ║
 * ║    oscillation" (desirable for ZN) from "heavy wobble" (unstable).  ║
 * ║    Micro-step Kp reduction fires when RMS > WOBBLE_HEAVY_RMS during ║
 * ║    settle phase, preventing catastrophic over-correction at 11.4V.  ║
 * ║                                                                     ║
 * ║  [AT8-3] DUAL-PROFILE ADAPTIVE TUNING (Straight + 90° Turn):       ║
 * ║    Terrain state machine (STRAIGHT / TURN_90 / UNKNOWN) from sensor ║
 * ║    error magnitude + look-ahead bits. Two independent PID profiles   ║
 * ║    (kpS/kdS, kpT/kdT). Welford online variance triggers a lock when ║
 * ║    straight error variance < threshold for 3 consecutive windows;   ║
 * ║    locked straight values are frozen while tuning focuses on turns. ║
 * ║                                                                     ║
 * ║  [AT8-4] PROGRESSIVE SPEED ESCALATION + BIDIRECTIONAL TWIDDLE:     ║
 * ║    Coordinate Descent (Twiddle) optimises IAE fitness score.        ║
 * ║    Tries param+dp; if fitness improves → keep & grow dp; otherwise  ║
 * ║    tries param-dp; neither → restore & shrink dp. Convergence       ║
 * ║    (sum(dp) < threshold) triggers speed step. No PWM cap — robot    ║
 * ║    escalates until mechanical ceiling is reached.                   ║
 * ║                                                                     ║
 * ║  [AT8-5] EMERGENCY INTERRUPT BRAKE (GPIO 34, active LOW):           ║
 * ║    IRAM_ATTR ISR sets gEBrakeFlag. Control loop checks flag every    ║
 * ║    5 ms tick before any other AT logic. On trigger: TB6612FNG        ║
 * ║    active electronic brake (IN1=IN2=HIGH, PWM=255), 200 ms hold,    ║
 * ║    then coast-stop + checkpoint save + IDLE transition.             ║
 * ║                                                                     ║
 * ║  ALL v7.0 FEATURES FULLY RETAINED (U1–U10, ZN fallback AT, NVS,    ║
 * ║  3-mode PID, stub-filter, hybrid braking, sensor-turn, EMA D-term). ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Forward declarations
enum class State : uint8_t;
enum class NavState : uint8_t;
enum class LookAheadState : uint8_t;
// [AT8] — must be forward-declared before at8NvsSave/at8NvsLoad use them as param types
enum class AT8Phase : uint8_t;
enum class ATTerrain : uint8_t;
enum class TWParam : uint8_t;
enum class TWDir : uint8_t;
enum class TWEval : uint8_t;

// ══════════════════════════════════════════════════════════════════════
//  PIN MAP  (unchanged from v6.7)
// ══════════════════════════════════════════════════════════════════════
#define PIN_PWMA 25
#define PIN_AIN1 26
#define PIN_AIN2 27
#define PIN_PWMB 2
#define PIN_BIN1 12
#define PIN_BIN2 15
const uint8_t QTR_PIN[8] = { 23, 22, 21, 17, 16, 14, 13, 4 };
#define PIN_QTR_IR 33
#define PIN_SR_PL 5
#define PIN_SR_CP 18
#define PIN_SR_Q7 32
// [AT8-5] Emergency brake button — input-only GPIO, active LOW
// Wiring: 10 kΩ pull-up to 3.3 V, momentary button to GND.
// GPIO 34 has no internal pull-up so external resistor is mandatory.
#define PIN_EBRAKE 34

// ══════════════════════════════════════════════════════════════════════
//  LEDC (PWM)
// ══════════════════════════════════════════════════════════════════════
#define LEDC_HZ 20000
#define LEDC_BITS 8

// ══════════════════════════════════════════════════════════════════════
//  FLOOR TUNING PARAMETERS  (5 ms tick)
// ══════════════════════════════════════════════════════════════════════
#define INTER_TURN_MIN_TICKS 25
#define INTER_TURN_TICKS 70
#define UTURN_MIN_TICKS 80
#define UTURN_MAX_TICKS 250  // [U3] raised — recovery spin unlimited

#define DASHED_COAST_MAX_SPD 90
#define D_EMA_ALPHA 0.25f

// ══════════════════════════════════════════════════════════════════════
//  STUB-FILTER CONSTANTS  (unchanged)
// ══════════════════════════════════════════════════════════════════════
#define STUB_MIN_ARM_WIDE_TICKS 10
#define STUB_MIN_ARM_WIDE_TICKS_HINTED 8
#define STUB_MIN_FWD_TICKS 2

// ══════════════════════════════════════════════════════════════════════
//  SENSOR / LINE CONSTANTS  (unchanged)
// ══════════════════════════════════════════════════════════════════════
#define QTR_TIMEOUT_US 1500
#define QTR_LOST_THR 400
#define SINGLE_SENSOR_MIN 150
#define QTR_SETPOINT 3500
#define HYBRID_MODE1_ERR 800.0f
#define HYBRID_MODE2_ERR 1800.0f
#define ANTISTALL_ERR_THR 600.0f
#define SMOOTH_ALPHA 0.12f
#define MOTOR_DEADBAND 25
#define LINE_LOST_MAX_TICKS 30
#define LINE_SEARCH_TICKS 150  // kept for dashed timeout; pivot-search no longer stops
#define SEARCH_PHASE_TICKS 40
#define WS_FAILSAFE_MS 5000

// ══════════════════════════════════════════════════════════════════════
//  [v5.5] HARDWARE LOOK-AHEAD SPEED TARGETS
//  [U4] These are now used as the smooth-scale endpoints, not hard caps.
//  Smooth scaling uses effBase lerp toward these targets based on proximity.
// ══════════════════════════════════════════════════════════════════════
#define LA_BRAKE_6CM_SPEED 60  // target speed when outer sensors (S1/S4) trigger
#define LA_PREP_2CM_SPEED 50   // target speed when inner sensors (S2/S3) trigger

// [U4] Geometry-based smooth scaling constants
#define LA_GEOM_RAMP_TICKS 30  // ticks over which speed scales down smoothly

// ══════════════════════════════════════════════════════════════════════
//  [ABS] AGGRESSIVE ABS JUNCTION BRAKING CONSTANTS
//  N20 motor lock ~50ms, release ~50ms → 100ms minimum cycle.
//  150ms cycles (30 ticks @5ms) used for motor/gear safety.
//  6cm trigger: 3 cycles = 450ms hard stop before junction.
//  2cm fail-safe: 1 cycle hard stop if speed is still too high.
// ══════════════════════════════════════════════════════════════════════
#define ABS_LOCK_TICKS      10   // ticks motors held in BRAKE per cycle (75ms)
#define ABS_RELEASE_TICKS   5   // ticks motors run fwd at low PWM per cycle (75ms)
#define ABS_CYCLES_6CM       1   // ABS cycles for 6cm sensor trigger
#define ABS_CYCLES_2CM       0   // ABS cycles for 2cm fail-safe trigger
#define ABS_RELEASE_PWM     40   // forward PWM during ABS release phase
#define ABS_2CM_SPD_MARGIN  15   // 2cm triggers hard brake if effBase > M1.base+this

// ══════════════════════════════════════════════════════════════════════
//  [v5.6] HYBRID FALLBACK BRAKING THRESHOLD
// ══════════════════════════════════════════════════════════════════════
#define PRED_BRAKE_DRATE_THRESHOLD 150.0f

// ══════════════════════════════════════════════════════════════════════
//  [U9] HYBRID BRAKING UPGRADE CONSTANTS
// ══════════════════════════════════════════════════════════════════════
#define HYBRID_RECOVER_ERR_THR 20.0f     // below this → restore full speed
#define HYBRID_RECOVER_TICKS 10            // must be below err thr for N ticks
#define HYBRID_DECEL_ERR_RATE_THR 300.0f  // spike rate threshold (units/tick)

// ══════════════════════════════════════════════════════════════════════
//  JUNCTION / NAVIGATION CONSTANTS  (unchanged)
// ══════════════════════════════════════════════════════════════════════
#define CROSS_BLACK_THR 700
#define CROSS_DEBOUNCE_TICKS 8
#define TURN_SENSE_THR CROSS_BLACK_THR
#define CROSS_PASSTHRU_TICKS 20
#define INTER_PASSTHRU_TICKS 15
#define DASHED_STRAIGHT_THR 350.0f
#define DASHED_FWD_TIMEOUT 80

#define DIR_LEFT 0x01
#define DIR_FWD 0x02
#define DIR_RIGHT 0x04

#define END_DETECT_FULL_THR 950
#define END_DETECT_TICKS 6

#define CROSS_BLANK_TICKS 30
#define POST_CROSS_CONFIRM_TICKS 300

// ══════════════════════════════════════════════════════════════════════
//  [U5] NODE MAP / GRID MEMORY
// ══════════════════════════════════════════════════════════════════════
#define NODE_MAP_SIZE 16      // FIFO size — old nodes auto-evicted
#define NODE_MATCH_WINDOW 80  // ticks after U-turn to search for revisit

// ══════════════════════════════════════════════════════════════════════
//  [U7] 45-DEGREE DUMMY LINE FILTER CONSTANTS
// ══════════════════════════════════════════════════════════════════════
#define DUMMY45_RATE_THR 900.0f  // |de/dt| threshold (units/tick)
#define DUMMY45_CONFIRM_TICKS 4  // sustained ticks to confirm diagonal
#define DUMMY45_COOL_TICKS 25    // ticks to suppress cross after filter

// ══════════════════════════════════════════════════════════════════════
//  [U10] VELOCITY SMOOTHING CONSTANTS
// ══════════════════════════════════════════════════════════════════════
#define VEL_ACCEL_RATE 25  // PWM/tick acceleration  (~3000 PWM/s)
#define VEL_DECEL_RATE 25  // PWM/tick deceleration  (~5000 PWM/s)

// ══════════════════════════════════════════════════════════════════════
//  [AT] ZIEGLER-NICHOLS AUTO-TUNE CONSTANTS  (v6.6 base; U2 raises max)
// ══════════════════════════════════════════════════════════════════════
// [BUG2-FIX] Converted from #define to runtime-mutable globals so the
// Web UI can update them via SET_AT_KP_START / SET_AT_KP_STEP commands.
// [FIX-PERMODE] Per-mode ZN/Twiddle Kp start and dp-step — M0/M1/M2 independently tunable
// gAtKpStart[m] : initial Twiddle Kp candidate seed for mode m (fresh start)
// gAtKpStep[m]  : initial Twiddle dpKp step for mode m (controls search granularity)
float gAtKpStart[3] = { 0.020f, 0.020f, 0.020f };
float gAtKpStep[3]  = { 0.020f, 0.020f, 0.020f };
#define AT_KP_MAX 3.00f
#define AT_KP_SETTLE_TICKS 300
#define AT_DETECT_WINDOW_TICKS 400
#define AT_CROSSINGS_REQUIRED 8
#define AT_ZN_KD_FACTOR 0.06f
#define AT_SAFETY_STALL_TICKS 800
#define AT_STALL_ERR_THR 2500.0f
#define AT_VIOLENT_OSC_THR 220
#define AT_SOFT_START_RAMP 8
#define AT_BASE_SPEED_MIN 50
#define AT_BASE_SPEED_MAX 255  // [U2] raised from 150 → hardware PWM limit
#define AT_BASE_SPEED_DEFAULT 50
#define AT_SAFE_START_PWM 50
#define AT_BASE_RAMP_RATE 5
#define AT_STALL_ERR_THR_M12 3000.0f
#define AT_CROSS_ERR_GATE 500.0f

// ══════════════════════════════════════════════════════════════════════
//  [AT8] INTELLIGENT DUAL-PROFILE ADAPTIVE AUTO-TUNE CONSTANTS
// ══════════════════════════════════════════════════════════════════════

// ── [AT8-5] Emergency Brake ───────────────────────────────────────────
// No extra constants needed — PIN_EBRAKE defined in PIN MAP above.

// ── [AT8-4] Twiddle / Coordinate Descent ─────────────────────────────
// Each candidate evaluation = TWIDDLE_SETTLE_TICKS (robot stabilises)
// + TWIDDLE_EVAL_TICKS (IAE accumulation).  Only ticks spent in the
// target terrain (STRAIGHT or TURN_90) count toward TWIDDLE_EVAL_TICKS,
// so corner contamination of straight eval and vice-versa is impossible.
#define TWIDDLE_SETTLE_TICKS 60      // dead ticks before IAE starts
#define TWIDDLE_EVAL_TICKS 200       // terrain-filtered IAE window
#define TWIDDLE_DP_GROW 1.05f        // grow dp after improvement
#define TWIDDLE_DP_SHRINK 0.95f      // shrink dp after no improvement
#define TWIDDLE_DONE_SUM 0.005f      // dpKp+dpKd < this → converged
#define TWIDDLE_KP_INIT_STEP 0.020f  // initial Kp delta (small — 11.4V)
#define TWIDDLE_KD_INIT_STEP 0.400f  // initial Kd delta
#define TWIDDLE_KP_MIN 0.010f        // hard lower clamp on Kp candidate
#define TWIDDLE_KP_MAX 3.000f        // hard upper clamp
#define TWIDDLE_KD_MIN 0.100f        // hard lower clamp on Kd candidate
#define TWIDDLE_KD_MAX 50.000f       // hard upper clamp

// ── [AT8-3] Terrain Detection ─────────────────────────────────────────
// STRAIGHT confirmed: |err| < STR_ERR_THR AND no LA turn bits active
//                     for STR_HOLD consecutive ticks
// TURN_90  confirmed: |err| > TRN_ERR_THR OR inner LA bits (s2||s3)
//                     for TRN_HOLD consecutive ticks
#define TERRAIN_STR_ERR_THR 480.0f  // units below = straight zone
#define TERRAIN_STR_HOLD 40         // ticks to confirm straight
#define TERRAIN_TRN_ERR_THR 860.0f  // units above = turn zone
#define TERRAIN_TRN_HOLD 12         // ticks to confirm turn

// ── [AT8-2] Wobble / Oscillation Analysis ─────────────────────────────
// Rolling circular buffer of WOBBLE_WINDOW error samples.
// RMS computed each tick. If RMS > WOBBLE_HEAVY_RMS AND we are in the
// SETTLE phase (robot should be calming down), Kp is micro-stepped down
// by WOBBLE_MICRO_STEP to prevent runaway oscillation at high voltage.
#define WOBBLE_WINDOW 80          // samples in rolling RMS window
#define WOBBLE_HEAVY_RMS 330.0f   // RMS above this = heavy wobble
#define WOBBLE_MICRO_STEP 0.005f  // Kp reduction per wobble tick

// ── [AT8-3] Straight-Profile Lock (Welford Online Variance) ───────────
// Welford's algorithm computes running variance without numerical drift.
// After STRAIGHT_LOCK_WINDOW samples, if variance < STRAIGHT_LOCK_VAR_THR
// for STRAIGHT_LOCK_HITS consecutive windows, the straight profile is
// considered optimally converged and locked permanently.
#define STRAIGHT_LOCK_WINDOW 200       // samples per variance window
#define STRAIGHT_LOCK_VAR_THR 3200.0f  // error variance threshold (units²)
#define STRAIGHT_LOCK_HITS 3           // consecutive stable windows needed

// ── [AT8-4] Progressive Speed Escalation ─────────────────────────────
// No artificial maximum — robot escalates until it stalls / loses line.
// On speed step: Twiddle dp values are reset to initials, but the last
// best [Kp, Kd] is carried forward as the starting baseline.
#define AT8_SPD_START 200           // initial PWM (safe at 11.4V)
#define AT8_SPD_STEP 10            // PWM added per escalation
#define AT8_SPD_CONFIRM 2          // stable Twiddle rounds to step
#define AT8_TWIDDLE_MAX_ROUNDS 20  // max param-advance iterations before forced speed step

// ── [AT8-4] PD output clamp for AT drive ─────────────────────────────
// During tuning the robot uses PD-only (no I) to keep evaluation clean.
// Output is clamped to ±1.8× base to prevent rail saturation during test.
#define AT8_OUTPUT_CLAMP_FACTOR 1.8f

// ── [AT8-1] NVS Checkpoint Keys ──────────────────────────────────────
// Namespace "at8". All keys ≤ 15 chars (NVS limit).
// "at8v"=valid flag, "at8ph"=phase, "at8spd"=speed,
// "at8kpS"/"at8kdS"=straight profile, "at8kpT"/"at8kdT"=turn profile,
// "at8sLk"=straight locked, "at8cnf"=speed-confirm count
// Written on every speed escalation step and on clean completion.

// ── BUG-FIX: Mode-specific peak-amplitude gates ───────────────────────
// ZN crossings are accepted only if the PEAK |error| between two consecutive
// zero-crossings falls inside the target mode's operating range.
//
// WHY PEAK, NOT AT-CROSSING:  At a zero-crossing, |err|=0 by definition.
// The peak amplitude BETWEEN crossings tells you what kind of dynamics caused
// that oscillation:
//   Small peak  (< MODE1_ERR)  → straight-section hunting      → M0 data
//   Medium peak (< MODE2_ERR)  → corner-transition oscillation → M1 data
//   Large peak  (≥ MODE2_ERR)  → hard-corner oscillation       → M2 data
//
// Mode  | Track type         | Peak gate low  | Peak gate high
// M0    | Straight           | 0              | HYBRID_MODE1_ERR (800)
// M1    | 90° Corner loop    | HYBRID_MODE1_ERR (800) | HYBRID_MODE2_ERR (1800)*1.3
// M2    | S-Curve / fast     | 1200           | 3600
// Sweep | Corner loop (M1)   | same as M1     | same as M1
//
// Result: M0 AT on a corner loop correctly IGNORES corner crossings.
//         M1 AT on a corner loop correctly IGNORES straight-section crossings.
static const float AT_GATE_PEAK_LOW[4] = { 0.0f, 800.0f, 1200.0f, 800.0f };
static const float AT_GATE_PEAK_HIGH[4] = { 800.0f, 2340.0f, 3600.0f, 2340.0f };

// ── Period consistency check ──────────────────────────────────────────
// After collecting AT_CROSSINGS_REQUIRED accepted crossings, compute the
// coefficient of variation (std_dev / mean) of crossing intervals.
// High CoV = irregular spacing = mixed straight+corner dynamics.
// Reject the batch and start again.  Typical value for clean oscillation: <0.18
#define AT_PERIOD_COV_MAX 0.22f  // 22% variation allowed
// Track type mismatch: if the robot spends > this fraction of ticks in the
// WRONG error zone for the current AT mode, warn the user via webLog.
#define AT_ZONE_WARN_RATIO 0.40f  // >40% ticks in wrong zone → warn

// ── Speed-Sweep AutoTune (Corner-Speed Finder) ────────────────────────
// atTuneMode==3 activates this. Sweeps speed from AT_SWEEP_START_SPD up.
// Each successful ZN round steps speed by AT_SWEEP_STEP_PWM.
// Stops on first abort and reports last passing speed + PID.
#define AT_SWEEP_MODE_ID 3
#define AT_SWEEP_START_SPD 50
#define AT_SWEEP_STEP_PWM 10
#define AT_SWEEP_MAX_SPD 255
#define AT_SWEEP_CONFIRM_ROUNDS 2  // rounds at same speed before stepping up

// ══════════════════════════════════════════════════════════════════════
//  WI-FI
// ══════════════════════════════════════════════════════════════════════
const char* AP_SSID = "LineFollower";
const char* AP_PASS = "robot1234";

// ══════════════════════════════════════════════════════════════════════
//  WEB LOG  (ring buffer)
// ══════════════════════════════════════════════════════════════════════
#define LOG_LINES 32
#define LOG_WIDTH 120
#define LOG_CHUNK 8

struct LogBuf {
  char lines[LOG_LINES][LOG_WIDTH];
  uint8_t head = 0;
  uint8_t tail = 0;
  uint8_t count = 0;
  bool dirty = false;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
} gLog;

void webLog(const char* fmt, ...) {
  char tmp[LOG_WIDTH];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  portENTER_CRITICAL(&gLog.mux);
  strncpy(gLog.lines[gLog.head], tmp, LOG_WIDTH - 1);
  gLog.lines[gLog.head][LOG_WIDTH - 1] = '\0';
  gLog.head = (gLog.head + 1) % LOG_LINES;
  if (gLog.count < LOG_LINES) {
    gLog.count++;
  } else {
    gLog.tail = gLog.head;
  }
  gLog.dirty = true;
  portEXIT_CRITICAL(&gLog.mux);
}

// ══════════════════════════════════════════════════════════════════════
//  PID PARAMETERS — competition-tuned defaults for 11.2V VM
// ══════════════════════════════════════════════════════════════════════
struct PIDParam {
  float kP, kI, kD;
  int base;
  int top;
  int low;
};
PIDParam gPID[3] = {
  { 0.55f, 0.000f, 9.0f, 110, 140, 60 },  // Mode 0 Straight
  { 0.90f, 0.000f, 14.0f, 70, 90, 40 },   // Mode 1 Curve Near
  { 1.40f, 0.000f, 22.0f, 48, 68, 28 },   // Mode 2 Curve Far
};
portMUX_TYPE gPidMux = portMUX_INITIALIZER_UNLOCKED;

// ══════════════════════════════════════════════════════════════════════
//  ROBOT STATE ENUMS
// ══════════════════════════════════════════════════════════════════════
enum class State : uint8_t {
  IDLE = 0,
  CALIBRATING = 1,
  RUNNING = 2,
  STOPPED = 3,
  AUTO_TUNING = 4
};

enum class NavState : uint8_t {
  LINE_FOLLOW = 0,
  DASHED_FWD = 1,
  PIVOT_SEARCH = 2,  // [U3] now spins indefinitely toward lastKnownSide
  AT_INTERSECTION = 3,
  INTER_TURN_L = 4,
  INTER_TURN_R = 5,
  DEAD_END_UTURN = 6,
};

enum class LookAheadState : uint8_t {
  STRAIGHT = 0,
  BRAKE_6CM = 1,
  PREP_2CM = 2,
};

// ── [AT8-3] Terrain detected by the AT8 sensor state machine ─────────
enum class ATTerrain : uint8_t {
  UNKNOWN = 0,   // transitioning, neither confirmed
  STRAIGHT = 1,  // low error, no LA turn bits — suitable for STR eval
  TURN_90 = 2,   // high error or LA inner bits — suitable for TRN eval
};

// ── [AT8] Adaptive auto-tuner phase ──────────────────────────────────
enum class AT8Phase : uint8_t {
  INIT = 0,        // cold start or resume: read NVS, set initial state
  STR_TUNE = 1,    // Twiddle optimising straight profile (kpS, kdS)
  STR_LOCKED = 2,  // straight locked; transition bridge to TRN_TUNE
  TRN_TUNE = 3,    // Twiddle optimising turn profile    (kpT, kdT)
  SPD_STEP = 4,    // speed escalation checkpoint (NVS save, ramp up)
  COMPLETE = 5,    // mechanical ceiling reached — apply final result
  ABORTED = 6,     // E-brake or safety abort triggered
};

// ── [AT8-4] Twiddle coordinate descent helpers ────────────────────────
enum class TWParam : uint8_t { KP = 0,
                               KD = 1 };  // parameter under test
enum class TWDir : uint8_t { IDLE = 0,
                             TRY_PLUS = 1,
                             TRY_MINUS = 2 };
enum class TWEval : uint8_t { SETTLE = 0,
                              MEASURE = 1 };  // sub-phase

// ══════════════════════════════════════════════════════════════════════
//  [U5] NODE MAP STRUCT  (FIFO grid memory)
// ══════════════════════════════════════════════════════════════════════
struct NodeEntry {
  uint8_t dirsAvail;  // bitmask: DIR_LEFT | DIR_FWD | DIR_RIGHT
  uint8_t dirsTried;  // paths already taken from this node
  uint8_t entryDir;   // direction robot entered from (logical, pre-flip)
  bool valid;
};

// ══════════════════════════════════════════════════════════════════════
//  GLOBAL STATE FLAGS
// ══════════════════════════════════════════════════════════════════════
volatile State gState = State::IDLE;
volatile bool gCalibOK = false;
volatile bool gManualDrive = false;
volatile bool gObsEn = true;
volatile bool gQtrIrOn = false;
portMUX_TYPE gStateMux = portMUX_INITIALIZER_UNLOCKED;

inline State getState() {
  State s;
  portENTER_CRITICAL(&gStateMux);
  s = gState;
  portEXIT_CRITICAL(&gStateMux);
  return s;
}
inline void setState(State s) {
  portENTER_CRITICAL(&gStateMux);
  gState = s;
  portEXIT_CRITICAL(&gStateMux);
}

volatile uint8_t gLineWidthSensors = 2;
volatile uint8_t gCrossMinSensors = 5;
volatile bool gWsEverConnected = false;
volatile int gWsClients = 0;
volatile uint32_t gWsLastSeenMs = 0;

// ── [v5.6] Hybrid Braking System ─────────────────────────────────────
volatile bool gLookAheadHealthy = true;
String gActiveBrakingSystem = "HARDWARE (HLA)";
float gLastDError = 0.0f;

// ── [ABS] Manual brake trigger from Web UI ────────────────────────────
volatile bool gManualBrakeReq = false;
portMUX_TYPE gManualBrakeMux = portMUX_INITIALIZER_UNLOCKED;

// ── [AT] Auto-Tune Globals ────────────────────────────────────────────
volatile bool gAtStartReq = false;
volatile bool gAtStopReq = false;
volatile int gAtBaseSpeed = AT_BASE_SPEED_DEFAULT;
volatile int gAtModeSpeed[3] = { 240, 50, 50 };  // [FIX] per-mode AT starting speeds M0/M1/M2
// [FIX-RESET] Set by AT8_RESET_NVS to force cold-start from TWIDDLE_KP_INIT_STEP
// (not from gPID warm values) regardless of existing gPID contents.
volatile bool gAt8ForceReset = false;
portMUX_TYPE gAtMux = portMUX_INITIALIZER_UNLOCKED;

struct AutoTuneResult {
  float currentKp = 0.002f;
  float ku = 0.0f;
  float tu = 0.0f;
  float resultKp = 0.0f;
  float resultKd = 0.0f;
  int zeroCrossings = 0;
  int8_t phase = -1;
  int8_t tuneMode = 0;
} gAtResult;
portMUX_TYPE gAtResMux = portMUX_INITIALIZER_UNLOCKED;

volatile int gAtTuneMode = 0;
volatile bool gAtSuccessBroadcastPending = false;
portMUX_TYPE gAtBcastMux = portMUX_INITIALIZER_UNLOCKED;
volatile int gAtRampedBasePub = AT_SAFE_START_PWM;

// [U1] Bidirectional PID sync broadcast flag
volatile bool gPidFetchPending = false;
portMUX_TYPE gPidFetchMux = portMUX_INITIALIZER_UNLOCKED;

Preferences gPrefs;

// ══════════════════════════════════════════════════════════════════════
//  [AT8] GLOBAL STATE — Intelligent Dual-Profile Adaptive Auto-Tuner
// ══════════════════════════════════════════════════════════════════════

// ── [AT8-5] Emergency brake ISR flag ─────────────────────────────────
// Written by IRAM_ATTR ISR on GPIO 34 falling edge.
// Checked at the very top of the AT8 block before any other logic.
volatile bool gEBrakeFlag = false;
portMUX_TYPE gEBrakeMux = portMUX_INITIALIZER_UNLOCKED;

// ── [AT8] Published status snapshot (for telemetry / Web UI) ─────────
// Written by taskControl under gAt8Mux, read by taskWeb for broadcast.
struct AT8Status {
  float kpS = 0.0f;           // straight Kp (current candidate)
  float kdS = 0.0f;           // straight Kd (current candidate)
  float kpT = 0.0f;           // turn Kp (current candidate)
  float kdT = 0.0f;           // turn Kd (current candidate)
  float activeKp = 0.0f;      // Kp ACTUALLY driving motors RIGHT NOW
  float activeKd = 0.0f;      // Kd ACTUALLY driving motors RIGHT NOW
  bool sLocked = false;       // straight profile locked?
  int speed = AT8_SPD_START;  // current escalation speed
  float fitness = 0.0f;       // best IAE fitness found (lower = better)
  float curFit = 0.0f;        // IAE currently accumulating (live counter)
  uint8_t phase = (uint8_t)AT8Phase::INIT;
  uint8_t terrain = (uint8_t)ATTerrain::UNKNOWN;
  float dpSum = 0.0f;  // Twiddle convergence indicator
  int tuneMode = 0;    // M0/M1/M2 selected for this run
} gAt8Status;
portMUX_TYPE gAt8Mux = portMUX_INITIALIZER_UNLOCKED;

// ══════════════════════════════════════════════════════════════════════
//  TELEMETRY
// ══════════════════════════════════════════════════════════════════════
struct Telemetry {
  int32_t pos, err, lPWM, rPWM, pidMode;
  float pidOut;
  uint16_t qtr[8];
  uint8_t obs, obsRaw;
  bool lineLost;
  bool searchMode;
  uint8_t navState;
  uint8_t nodeCount;  // [U5] active nodes in FIFO
  bool atCross;
  uint8_t lineWidthSensors;
  uint8_t crossMinSensors;
  uint8_t laState;
  int8_t laTurnHint;
  bool laHealthy;
  bool dummy45Active;  // [U7] dummy filter active
} gTel = {};
portMUX_TYPE gTelMux = portMUX_INITIALIZER_UNLOCKED;

uint16_t qMin[8], qMax[8], qRaw[8], qCal[8];

// ══════════════════════════════════════════════════════════════════════
//  MOTOR CONTROL  (unchanged)
// ══════════════════════════════════════════════════════════════════════
static void motorDrive(uint8_t pwmPin,
                       uint8_t in1Pin, bool in1Fwd,
                       uint8_t in2Pin, bool in2Fwd,
                       int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1Pin, in1Fwd ? HIGH : LOW);
    digitalWrite(in2Pin, in2Fwd ? HIGH : LOW);
    ledcWrite(pwmPin, (uint32_t)speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, in1Fwd ? LOW : HIGH);
    digitalWrite(in2Pin, in2Fwd ? LOW : HIGH);
    ledcWrite(pwmPin, (uint32_t)(-speed));
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, 0U);
  }
}
void driveL(int s) {
  motorDrive(PIN_PWMA, PIN_AIN1, false, PIN_AIN2, true, s);
}
void driveR(int s) {
  motorDrive(PIN_PWMB, PIN_BIN1, true, PIN_BIN2, false, s);
}
void stopAll() {
  driveL(0);
  driveR(0);
}
// [ABS] Active electronic brake via TB6612FNG.
// IN1=IN2=HIGH + PWM=255 shorts motor terminals → shaft locks immediately.
// Always follow with stopAll() or driveL/R when releasing.
void motorBrake() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  ledcWrite(PIN_PWMA, 255);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMB, 255);
}
void motorInit() {
  ledcAttach(PIN_PWMA, LEDC_HZ, LEDC_BITS);
  ledcAttach(PIN_PWMB, LEDC_HZ, LEDC_BITS);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  stopAll();
}

// ══════════════════════════════════════════════════════════════════════
//  QTR-8RC  (unchanged)
// ══════════════════════════════════════════════════════════════════════
void qtrStartCharge() {
  for (int i = 0; i < 8; i++) {
    pinMode(QTR_PIN[i], OUTPUT);
    digitalWrite(QTR_PIN[i], HIGH);
  }
}
void qtrCompleteRead() {
  for (int i = 0; i < 8; i++) {
    pinMode(QTR_PIN[i], INPUT);
    qRaw[i] = QTR_TIMEOUT_US;
  }
  uint32_t t0 = micros();
  bool done[8] = {};
  int rem = 8;
  while (rem > 0) {
    uint32_t el = micros() - t0;
    if (el >= QTR_TIMEOUT_US) break;
    for (int i = 0; i < 8; i++)
      if (!done[i] && !digitalRead(QTR_PIN[i])) {
        qRaw[i] = (uint16_t)el;
        done[i] = true;
        rem--;
      }
  }
}
void qtrRead() {
  for (int i = 0; i < 8; i++) {
    pinMode(QTR_PIN[i], OUTPUT);
    digitalWrite(QTR_PIN[i], HIGH);
  }
  delayMicroseconds(10);
  uint32_t t0 = micros();
  bool done[8] = {};
  for (int i = 0; i < 8; i++) {
    pinMode(QTR_PIN[i], INPUT);
    qRaw[i] = QTR_TIMEOUT_US;
  }
  int rem = 8;
  while (rem > 0) {
    uint32_t el = micros() - t0;
    if (el >= QTR_TIMEOUT_US) break;
    for (int i = 0; i < 8; i++)
      if (!done[i] && !digitalRead(QTR_PIN[i])) {
        qRaw[i] = (uint16_t)el;
        done[i] = true;
        rem--;
      }
  }
}
void qtrCalibRun(uint32_t ms) {
  uint32_t end = millis() + ms;
  while (millis() < end) {
    qtrRead();
    for (int i = 0; i < 8; i++) {
      if (qRaw[i] < qMin[i]) qMin[i] = qRaw[i];
      if (qRaw[i] > qMax[i]) qMax[i] = qRaw[i];
    }
    delay(2);
  }
}
void qtrNormalize() {
  for (int i = 0; i < 8; i++) {
    int range = (int)qMax[i] - (int)qMin[i];
    if (!gCalibOK || range < 10) {
      qCal[i] = 0;
      continue;
    }
    long v = ((long)(qRaw[i] - (int)qMin[i]) * 1000L) / (long)range;
    qCal[i] = (uint16_t)constrain(v, 0L, 1000L);
  }
}

// ══════════════════════════════════════════════════════════════════════
//  74HC165 SHIFT REGISTER
// ══════════════════════════════════════════════════════════════════════
uint8_t sr165Read() {
  digitalWrite(PIN_SR_PL, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_SR_PL, HIGH);
  delayMicroseconds(5);
  uint8_t b = 0;
  for (int i = 7; i >= 0; i--) {
    b |= ((uint8_t)digitalRead(PIN_SR_Q7) << i);
    digitalWrite(PIN_SR_CP, HIGH);
    delayMicroseconds(4);
    digitalWrite(PIN_SR_CP, LOW);
    delayMicroseconds(4);
  }
  delayMicroseconds(2);
  return b;
}
void parseLookAhead(uint8_t raw, bool& s1, bool& s2, bool& s3, bool& s4) {
  s1 = (bool)((raw >> 4) & 1);
  s2 = (bool)((raw >> 5) & 1);
  s3 = (bool)((raw >> 6) & 1);
  s4 = (bool)((raw >> 7) & 1);
}

// ══════════════════════════════════════════════════════════════════════
//  [U10] VELOCITY RAMP HELPER
//  Applies asymmetric acceleration/deceleration to smooth PWM transitions.
//  Returns new current value stepped toward target by at most accel/decelRate.
// ══════════════════════════════════════════════════════════════════════
inline int velRamp(int current, int target, int accelRate, int decelRate) {
  if (current == target) return current;
  // [FIX] Compare absolute magnitudes so reverse speeds use correct physics:
  // |target| > |current| = accelerating (slow rate), else braking (fast rate)
  int activeRate = (abs(target) > abs(current)) ? accelRate : decelRate;
  if (target > current) return min(current + activeRate, target);
  else return max(current - activeRate, target);
}

// ══════════════════════════════════════════════════════════════════════
//  [AT8-5] EMERGENCY BRAKE ISR  —  IRAM_ATTR (runs from IRAM, safe in ISR)
//  Triggered by falling edge on PIN_EBRAKE (button pressed, active LOW).
//  Sets gEBrakeFlag; the AT8 block polls it every 5 ms control tick.
// ══════════════════════════════════════════════════════════════════════
void IRAM_ATTR isrEBrake() {
  portENTER_CRITICAL_ISR(&gEBrakeMux);
  gEBrakeFlag = true;
  portEXIT_CRITICAL_ISR(&gEBrakeMux);
}

// ══════════════════════════════════════════════════════════════════════
//  [AT8-1] NVS CHECKPOINT HELPERS
//  Save/load the full AT8 tuner state to Preferences ("at8" namespace).
//  Called on every speed escalation step, on clean completion, and on
//  E-brake / safety abort so that any crash leaves a valid resume point.
// ══════════════════════════════════════════════════════════════════════
// [FIX-PERMODE] at8NvsSave: stores tuneMode so per-mode resume/reset works correctly.
// [LOG] Every save emits a webLog entry for traceability.
static void at8NvsSave(AT8Phase phase, int spd,
                       float kpS, float kdS,
                       float kpT, float kdT,
                       bool sLocked, int cnf,
                       int tuneMode = 0) {
  gPrefs.putBool("at8v", true);
  gPrefs.putUChar("at8ph", (uint8_t)phase);
  gPrefs.putInt("at8spd", spd);
  gPrefs.putFloat("at8kpS", kpS);
  gPrefs.putFloat("at8kdS", kdS);
  gPrefs.putFloat("at8kpT", kpT);
  gPrefs.putFloat("at8kdT", kdT);
  gPrefs.putBool("at8sLk", sLocked);
  gPrefs.putInt("at8cnf", cnf);
  gPrefs.putUChar("at8mod", (uint8_t)constrain(tuneMode, 0, 2));  // [FIX-PERMODE]
  webLog("[AT8-NVS-SAVE] mode=M%d phase=%d spd=%d KpS=%.4f KdS=%.4f KpT=%.4f KdT=%.4f sLk=%d",
         tuneMode, (int)phase, spd, kpS, kdS, kpT, kdT, (int)sLocked);
}

// [FIX-PERMODE] at8NvsLoad: reqMode (-1=any, 0/1/2=specific mode).
// Returns false if mode mismatch so caller does a fresh start for that mode.
// [LOG] Emits webLog on every load attempt for traceability.
static bool at8NvsLoad(AT8Phase& phase, int& spd,
                       float& kpS, float& kdS,
                       float& kpT, float& kdT,
                       bool& sLocked, int& cnf,
                       int reqMode = -1) {
  if (!gPrefs.getBool("at8v", false)) {
    webLog("[AT8-NVS-LOAD] No checkpoint (at8v=false). Fresh start.");
    return false;
  }

  // [FIX-PERMODE] Mode-match guard: reject checkpoint if it belongs to a different mode
  int savedMode = (int)gPrefs.getUChar("at8mod", 255);
  if (reqMode >= 0 && savedMode != reqMode) {
    webLog("[AT8-NVS-LOAD] Mode mismatch: saved=M%d requested=M%d → fresh start.",
           savedMode, reqMode);
    return false;
  }

  phase    = (AT8Phase)gPrefs.getUChar("at8ph", (uint8_t)AT8Phase::STR_TUNE);
  spd      = gPrefs.getInt("at8spd", AT8_SPD_START);
  kpS      = gPrefs.getFloat("at8kpS", TWIDDLE_KP_INIT_STEP);
  kdS      = gPrefs.getFloat("at8kdS", TWIDDLE_KD_INIT_STEP);
  kpT      = gPrefs.getFloat("at8kpT", TWIDDLE_KP_INIT_STEP);
  kdT      = gPrefs.getFloat("at8kdT", TWIDDLE_KD_INIT_STEP);
  sLocked  = gPrefs.getBool("at8sLk", false);
  cnf      = gPrefs.getInt("at8cnf", 0);

  // Sanity: reject values that would cause unsafe behaviour on 11.4 V
  if (spd < AT8_SPD_START || spd > 255) spd = AT8_SPD_START;
  kpS = constrain(kpS, TWIDDLE_KP_MIN, TWIDDLE_KP_MAX);
  kdS = constrain(kdS, TWIDDLE_KD_MIN, TWIDDLE_KD_MAX);
  kpT = constrain(kpT, TWIDDLE_KP_MIN, TWIDDLE_KP_MAX);
  kdT = constrain(kdT, TWIDDLE_KD_MIN, TWIDDLE_KD_MAX);
  cnf = constrain(cnf, 0, AT8_SPD_CONFIRM);

  // If saved phase is COMPLETE/ABORTED/INIT, treat as fresh start
  if (phase == AT8Phase::COMPLETE || phase == AT8Phase::ABORTED
      || phase == AT8Phase::INIT) {
    phase = AT8Phase::STR_TUNE;
  }

  webLog("[AT8-NVS-LOAD] OK: mode=M%d phase=%d spd=%d KpS=%.4f KdS=%.4f KpT=%.4f KdT=%.4f sLk=%d",
         savedMode, (int)phase, spd, kpS, kdS, kpT, kdT, (int)sLocked);
  return true;
}
// [FIX-PERMODE] Clear AT8 NVS checkpoint.
// mode=-1: clear unconditionally (all modes).
// mode=0/1/2: only clear if the stored checkpoint matches that mode.
// [LOG] Always emits webLog for traceability.
static void at8NvsClear(int mode = -1) {
  if (mode >= 0) {
    int savedMode = (int)gPrefs.getUChar("at8mod", 255);
    if (savedMode != mode) {
      webLog("[AT8-NVS-CLR] Skip: saved=M%d != requested=M%d (no checkpoint for this mode).",
             savedMode, mode);
      return;
    }
  }
  gPrefs.putBool("at8v", false);
  const char* mLbl = (mode < 0) ? "ALL" : (mode == 0 ? "M0" : mode == 1 ? "M1" : "M2");
  webLog("[AT8-NVS-CLR] Checkpoint CLEARED (mode=%s). Next run: fresh start.", mLbl);
}
// ══════════════════════════════════════════════════════════════════════
void taskCalibrate(void*) {
  gManualDrive = false;
  gCalibOK = false;
  for (int i = 0; i < 8; i++) {
    qMin[i] = 0xFFFF;
    qMax[i] = 0;
  }

  digitalWrite(PIN_QTR_IR, HIGH);
  gQtrIrOn = true;
  delay(80);
  webLog("CAL: IR ON. Sweep: 3 passes...");
  driveL(-75);
  driveR(75);
  qtrCalibRun(700);
  driveL(75);
  driveR(-75);
  qtrCalibRun(1400);
  driveL(-75);
  driveR(75);
  qtrCalibRun(700);
  stopAll();
  delay(200);

  int ok = 0;
  for (int i = 0; i < 8; i++)
    if ((qMax[i] - qMin[i]) > 100) ok++;
  webLog("CAL: %d/8 sensors OK (range > 100us).", ok);
  if (ok < 4) webLog("CAL: WARN — few sensors respond. Check pot/height/wiring.");
  for (int i = 0; i < 8; i++)
    webLog("  S%d  min=%-4d max=%-4d D=%d", i, qMin[i], qMax[i], qMax[i] - qMin[i]);

  gCalibOK = true;
  webLog("CAL: [COMP-01] Probing line width — place robot ON line now.");
  webLog("CAL: [v5.6-01] Look-Ahead Self-Test in progress...");
  delay(500);

  uint8_t maxSimul = 0;
  const uint8_t probeThr = 600;
  bool laSawBlack[4] = { false, false, false, false };
  bool laSawWhite[4] = { false, false, false, false };

  auto trackLaSensors = [&]() {
    uint8_t s1 = sr165Read();
    delayMicroseconds(100);
    uint8_t s2 = sr165Read();
    delayMicroseconds(100);
    uint8_t s3 = sr165Read();
    uint8_t orV = s1 | s2 | s3;
    uint8_t andV = s1 & s2 & s3;
    for (int j = 0; j < 4; j++) {
      bool highInAny = (bool)((orV >> (4 + j)) & 1);
      bool lowInAny = !((bool)((andV >> (4 + j)) & 1));
      if (highInAny) laSawBlack[j] = true;
      if (lowInAny) laSawWhite[j] = true;
    }
  };

  driveL(-70);
  driveR(70);
  for (int i = 0; i < 150; i++) {
    qtrRead();
    qtrNormalize();
    uint8_t cnt = 0;
    for (int j = 0; j < 8; j++)
      if (qCal[j] >= probeThr) cnt++;
    if (cnt > maxSimul) maxSimul = cnt;
    trackLaSensors();
    delay(10);
  }
  driveL(70);
  driveR(-70);
  for (int i = 0; i < 300; i++) {
    qtrRead();
    qtrNormalize();
    uint8_t cnt = 0;
    for (int j = 0; j < 8; j++)
      if (qCal[j] >= probeThr) cnt++;
    if (cnt > maxSimul) maxSimul = cnt;
    trackLaSensors();
    delay(10);
  }
  stopAll();

  if (maxSimul == 0) {
    webLog("CAL: WARN — probe saw 0 sensors. Robot off line during probe?");
    webLog("CAL: Defaulting gLineWidthSensors=2, gCrossMinSensors=5.");
    maxSimul = 2;
  }
  uint8_t lwSens = constrain(maxSimul, 1, 5);
  uint8_t crMin = constrain(lwSens + 3, 4, 8);
  gLineWidthSensors = lwSens;
  gCrossMinSensors = crMin;
  webLog("CAL: [COMP-01] Line width probe: peak=%d sensors active.", maxSimul);
  webLog("CAL:   gLineWidthSensors=%d (~%.1f cm at 8mm pitch)", lwSens, lwSens * 0.8f);
  webLog("CAL:   gCrossMinSensors=%d (T-junction threshold)", crMin);

  const char* sNames[4] = { "S1(bit4/Left-End-6cm)", "S2(bit5/Left-Mid-2cm)",
                            "S3(bit6/Right-Mid-2cm)", "S4(bit7/Right-End-6cm)" };
  const bool isOuter[4] = { true, false, false, true };
  bool hardFail = false;
  bool outerAdvisory = false;
  for (int j = 0; j < 4; j++) {
    bool seenBlack = laSawBlack[j];
    bool seenWhite = laSawWhite[j];
    if (isOuter[j]) {
      if (seenBlack && seenWhite) webLog("CAL:   %s FULL PASS.", sNames[j]);
      else if (!seenBlack && seenWhite) {
        webLog("CAL:   %s ADVISORY - only WHITE.", sNames[j]);
        outerAdvisory = true;
      } else if (seenBlack && !seenWhite) {
        webLog("CAL:   %s HARD FAIL - stuck BLACK!", sNames[j]);
        hardFail = true;
      } else {
        webLog("CAL:   %s HARD FAIL - no data!", sNames[j]);
        hardFail = true;
      }
    } else {
      if (seenBlack && seenWhite) webLog("CAL:   %s PASS.", sNames[j]);
      else {
        webLog("CAL:   %s FAIL - stuck %s!", sNames[j], seenBlack ? "HIGH" : "LOW");
        hardFail = true;
      }
    }
  }

  if (!hardFail) {
    gLookAheadHealthy = true;
    gActiveBrakingSystem = "HARDWARE (HLA)";
    webLog("CAL: Look-Ahead %sPASSED. Braking: HARDWARE (HLA)", outerAdvisory ? "(advisory) " : "");
  } else {
    gLookAheadHealthy = false;
    gActiveBrakingSystem = "MATH-BRAKING (FALLBACK)";
    webLog("CAL: WARN - Look-Ahead HARD FAIL! Falling back to MATH-BRAKING.");
    webLog("CAL: WARN - Fallback threshold: %.0f units/tick.", (float)PRED_BRAKE_DRATE_THRESHOLD);
  }
  setState(State::IDLE);
  webLog("CAL: Done. Select preset -> Apply -> START. Or use AutoTune tab.");
  vTaskDelete(NULL);
}

// ══════════════════════════════════════════════════════════════════════
//  CONTROL TASK  —  Core 1, Priority 10, ~200 Hz (5 ms tick)
//
//  Upgrades implemented here:
//    [U3]  Line-loss directional memory + recovery spin
//    [U4]  Geometry-based look-ahead braking (smooth scale)
//    [U5]  Node map FIFO + visited-path routing
//    [U6]  Anti-backtrack dead-end recovery
//    [U7]  45-degree dummy line filter (de/dt sweep analysis)
//    [U8]  T-junction priority (STRAIGHT > RIGHT > LEFT)
//    [U9]  Hybrid braking with post-turn full-speed restore
//    [U10] Velocity ramp smoothing on all transitions
// ══════════════════════════════════════════════════════════════════════
void taskControl(void*) {

  PIDParam wp[3];
  auto syncParams = [&]() {
    portENTER_CRITICAL(&gPidMux);
    for (int i = 0; i < 3; i++) wp[i] = gPID[i];
    portEXIT_CRITICAL(&gPidMux);
  };
  syncParams();

  // ── PID state ────────────────────────────────────────────────────────
  float pidI = 0.0f;
  float pidPrev = 0.0f;
  int lastPos = QTR_SETPOINT;
  int lineSide = 0;  // sign of last non-zero error
  float lastAbsErr = 0.0f;
  int prevPm = 0;
  float pidDFiltered = 0.0f;
  int8_t laTurnHint = 0;

  unsigned long lastPMs = 0;
  unsigned long lastUs = micros();
  unsigned long lastTelMs = 0;
  TickType_t wake = xTaskGetTickCount();

  float effKp = wp[0].kP;
  float effKd = wp[0].kD;
  float effBase = (float)wp[0].base;

  State lastState = State::IDLE;

  // ── Navigation state machine ─────────────────────────────────────────
  NavState navState = NavState::LINE_FOLLOW;

  int lineLostTicks = 0;
  int dashedFwdTicks = 0;
  int uTurnTicks = 0;
  int crossDebTicks = 0;
  int crossPassTicks = 0;
  int interPassTicks = 0;
  int interTurnTicks = 0;
  int searchTicks = 0;
  int endDetectTicks = 0;
  int crossBlankTicks = 0;

  // ── Intersection arm sampling (STUB-FILTER) ───────────────────────────
  bool juncHasLeft = false;
  bool juncHasFwd = false;
  bool juncHasRight = false;
  int juncLeftWideTicks = 0;
  int juncRightWideTicks = 0;
  int juncFwdTicks = 0;
  int8_t juncEntryHint = 0;

  int lastLSpd = wp[0].base;
  int lastRSpd = wp[0].base;

  // ── [U10] Velocity ramp tracking ─────────────────────────────────────
  int rampedLSpd = wp[0].base;  // current actual L speed (ramped)
  int rampedRSpd = wp[0].base;  // current actual R speed (ramped)

  // ── [U3] Line-loss directional memory ────────────────────────────────
  int8_t lastKnownErrSign = 1;  // +1 = line was to the right, -1 = left
  bool recoverySpinActive = false;

  // ── [U5] Node Map FIFO ────────────────────────────────────────────────
  NodeEntry nodeMap[NODE_MAP_SIZE] = {};
  uint8_t nodeHead = 0;   // FIFO head (oldest, evicts here)
  uint8_t nodeCount = 0;  // active entries
  uint8_t nodeTail = 0;   // FIFO write pointer
  // Current junction being processed (index into FIFO ring)
  int8_t curNodeIdx = -1;         // -1 = not at a known node
  uint8_t lastTurnDir = DIR_FWD;  // direction taken at the last junction

  // ── [U6] Anti-backtrack state ─────────────────────────────────────────
  uint8_t lastJuncDirsTried = 0;  // dirs tried at junction we just U-turned from
  uint8_t lastJuncDirsAvail = 0;  // all available dirs at that junction
  bool postUTurnRouting = false;
  int postUTurnTicks = 0;

  // ── [U7] 45-degree dummy line filter ─────────────────────────────────
  float prevErrForSlope = 0.0f;
  int diagSweepTicks = 0;    // consecutive ticks of high de/dt
  int dummy45CoolTicks = 0;  // cooldown after filter fires
  bool dummy45Active = false;

  // ── [U9] Hybrid braking post-turn restore ────────────────────────────
  int hybridRecoverTicks = 0;        // consecutive ticks of low error after turn
  bool hybridBrakeOverride = false;  // HLA/math braking currently active

  // ── [ABS] Aggressive ABS junction braking state machine ──────────────
  // Priority 1: 6cm sensors (S1/S4) → 3 hard brake cycles → M1 speed.
  // Priority 2: 2cm sensors (S2/S3) → validate speed → 1 cycle if too fast.
  // Priority 3 (fallback): math-based (hybridBrakeOverride) for curved bends.
  // Rule: HW sensor ABS and math-based NEVER run simultaneously.
  bool absActive      = false;          // ABS state machine running
  int  absPhase       = 0;              // 0=LOCK phase, 1=RELEASE phase
  int  absPhaseTicks  = 0;              // ticks elapsed in current phase
  int  absCycleCount  = 0;              // completed lock+release cycles
  int  absCycleMax    = ABS_CYCLES_6CM; // target cycles for this trigger
  bool absDone        = false;          // ABS finished — guard against re-trigger
  bool absFrom6cm     = false;          // triggered by 6cm outer sensors
  bool absFrom2cm     = false;          // triggered by 2cm inner fail-safe
  bool absFromManual  = false;          // triggered by Web UI brake button

  // ── Telemetry snapshot locals ─────────────────────────────────────────
  int32_t snap_pos = QTR_SETPOINT, snap_err = 0, snap_lPWM = 0, snap_rPWM = 0;
  int32_t snap_pidMode = 0;
  uint16_t snap_qtr[8] = {};
  uint8_t snap_obs = 0, snap_obsRaw = 0;
  bool snap_lineLost = false, snap_searchMode = false, snap_atCross = false;
  uint8_t snap_navState = 0;
  uint8_t snap_laState = 0;
  int8_t snap_laTurnHint = 0;
  bool snap_laHealthy = true;
  bool snap_dummy45 = false;
  uint8_t snap_nodeCount = 0;

  // ── [v7.0 ZN AT] Legacy task-local state (kept for non-AT8 manual mode) ─
  float atCurrentKp = gAtKpStart[0];
  int atPhase = -1;
  int atPhaseTicks = 0;
  int atCrossCount = 0;
  float atLastErrSign = 0.0f;
  unsigned long atCrossTimes[AT_CROSSINGS_REQUIRED] = {};
  int atStallTicks = 0;
  int atViolentTicks = 0;
  int atLastLSpd = 0;
  int atLastRSpd = 0;
  int atRampedBase = AT_SAFE_START_PWM;
  int atTuneMode = 0;

  // ── Speed-Sweep state (atTuneMode == AT_SWEEP_MODE_ID) ───────────────
  int atSweepCurrentSpd = AT_SWEEP_START_SPD;
  int atSweepBestSpd = 0;
  float atSweepBestKp = 0.0f;
  float atSweepBestKd = 0.0f;
  int atSweepConfirmCount = 0;

  // ════════════════════════════════════════════════════════════════════
  //  [AT8] INTELLIGENT DUAL-PROFILE ADAPTIVE AUTO-TUNER — Task-Local State
  //  All variables live on the stack of taskControl (Core 1). No heap.
  // ════════════════════════════════════════════════════════════════════

  // ── Phase & speed ────────────────────────────────────────────────────
  AT8Phase at8Ph = AT8Phase::INIT;       // resolved on first AT start
  int at8Spd = AT8_SPD_START;            // current escalation PWM
  int at8RampedSpd = AT_SAFE_START_PWM;  // soft-start ramp tracker
  int at8StallTix = 0;                   // stall / line-loss counter
  int at8ViolTix = 0;                    // violent oscillation counter
  bool at8Initialized = false;           // false until first AT start

  // ── Dual PID profiles (candidate values being Twiddle-tested) ────────
  float at8KpS = TWIDDLE_KP_INIT_STEP;  // straight Kp candidate
  float at8KdS = TWIDDLE_KD_INIT_STEP;  // straight Kd candidate
  float at8KpT = TWIDDLE_KP_INIT_STEP;  // turn Kp candidate
  float at8KdT = TWIDDLE_KD_INIT_STEP;  // turn Kd candidate
  bool at8SLocked = false;              // straight profile locked?
  int at8SpdCnf = 0;                    // speed-escalation confirm count

  // ── Active PD output state ────────────────────────────────────────────
  float at8PrevErr = 0.0f;  // previous error for D-term
  float at8DFilt = 0.0f;    // EMA-filtered derivative
  int at8LastL = 0;         // soft-start prev L speed
  int at8LastR = 0;         // soft-start prev R speed

  // ── [M0-CORNER] Pre-corner braking state (M0 AutoTune ONLY) ──────────
  // During M0 tuning at high escalated speed, 90° corners require:
  //   Phase 1: TB6612FNG electronic hard-brake for ABS_LOCK_TICKS ticks.
  //   Phase 2: Corner follow at gAtModeSpeed[1] using saved M1 Kp/Kd.
  //   Exit:    Terrain SM confirms STRAIGHT → resume M0 speed ramp.
  // IAE isolation (Goal 4) is already enforced by terrainMatch gating
  // in the Twiddle engine — no additional suppression is needed here.
  bool at8M0CornerActive = false;  // true = currently in M0 corner-handling
  int  at8M0BrakeTicks   = 0;      // ticks of TB6612FNG hard-brake applied
  int  at8M0CornerTicks  = 0;      // total ticks in this corner episode
  bool at8M0HardBraking  = false;  // signals motor-output stage to call motorBrake()

  // ── [AT8-3] Terrain detection counters ───────────────────────────────
  ATTerrain at8Terrain = ATTerrain::UNKNOWN;
  int at8StrCnt = 0;  // consecutive straight ticks
  int at8TrnCnt = 0;  // consecutive turn ticks

  // ── [AT8-2] Wobble analysis — rolling circular buffer ─────────────────
  float at8ErrBuf[WOBBLE_WINDOW] = {};  // ring buffer of error samples
  int at8EBufIdx = 0;                   // write head
  int at8EBufFill = 0;                  // how many valid samples (0→WOBBLE_WINDOW)

  // ── [AT8-3] Welford online variance for straight-lock ─────────────────
  int at8WfN = 0;          // samples in current window
  float at8WfMean = 0.0f;  // Welford running mean
  float at8WfM2 = 0.0f;    // Welford M2 accumulator
  int at8LockHits = 0;     // consecutive stable windows

  // ── [AT8-4] Twiddle / Coordinate Descent ─────────────────────────────
  // We tune one scalar at a time (KP, then KD, cycling) per Twiddle step.
  // Each step: try param+dp → eval IAE → compare to best → keep or try -dp.
  TWParam at8TwPar = TWParam::KP;          // which param is under test
  TWDir at8TwDir = TWDir::IDLE;            // IDLE=baseline, PLUS, MINUS
  TWEval at8TwEval = TWEval::SETTLE;       // SETTLE or MEASURE sub-phase
  float at8TwDpKp = TWIDDLE_KP_INIT_STEP;  // Kp delta (shrinks/grows)
  float at8TwDpKd = TWIDDLE_KD_INIT_STEP;  // Kd delta
  float at8TwBestFit = 1e9f;               // best IAE seen (lower=better)
  float at8TwCurFit = 0.0f;                // IAE being accumulated now
  int at8TwTick = 0;                       // tick counter within sub-phase
  int at8TwTerrTick = 0;                   // terrain-filtered eval ticks
  bool at8TwBaseline = true;               // true = run baseline eval first
  int at8TwRounds = 0;                     // param-advance round counter per speed level

  qtrStartCharge();

  // ════════════════════════════════════════════════════════════════════
  //  MAIN CONTROL LOOP
  // ════════════════════════════════════════════════════════════════════
  while (true) {
    vTaskDelayUntil(&wake, 5 / portTICK_PERIOD_MS);

    bool calibOwnsQtr = (getState() == State::CALIBRATING);
    if (!calibOwnsQtr) {
      qtrCompleteRead();
      qtrNormalize();
    }

    if (millis() - lastPMs > 100) {
      syncParams();
      lastPMs = millis();
    }

    State localState = getState();
    bool isAutoTuning = (localState == State::AUTO_TUNING);

    // ── State guard ───────────────────────────────────────────────────
    if (localState != State::RUNNING && !isAutoTuning) {
      if (localState != State::CALIBRATING && !gManualDrive) stopAll();

      pidI = 0.0f;
      pidPrev = 0.0f;
      pidDFiltered = 0.0f;
      lineLostTicks = 0;
      navState = NavState::LINE_FOLLOW;
      crossBlankTicks = 0;
      endDetectTicks = 0;
      laTurnHint = 0;
      gLastDError = 0.0f;
      juncLeftWideTicks = 0;
      juncRightWideTicks = 0;
      juncFwdTicks = 0;
      juncEntryHint = 0;
      recoverySpinActive = false;
      dummy45Active = false;
      dummy45CoolTicks = 0;
      diagSweepTicks = 0;
      hybridRecoverTicks = 0;
      hybridBrakeOverride = false;
      postUTurnRouting = false;
      // [ABS] Reset ABS state on robot stop/idle
      absActive     = false;
      absPhase      = 0;
      absPhaseTicks = 0;
      absCycleCount = 0;
      absDone       = false;
      absFrom6cm    = false;
      absFrom2cm    = false;
      absFromManual = false;
      rampedLSpd = wp[0].base;
      rampedRSpd = wp[0].base;
      effBase = (float)wp[0].base;  // [FIX] prevent stale brake-reduced effBase on re-start
      // [FIX-EFFKP] Reset effective Kp/Kd to mode-0 baseline on every non-running tick.
      // Without this, stale high-mode gains from a previous run are still in effKp/effKd
      // when RUNNING restarts, causing 10-15 ticks of wrong-gain overshoot → wobble.
      effKp = wp[0].kP;
      effKd = wp[0].kD;
      prevPm = 0;
      // Reset AT state
      // BUG-FIX: phase==2 means AT just succeeded. gAtSuccessBroadcastPending
      // is still true and taskWeb (50 ms interval) hasn't read gAtResult yet.
      // Resetting here would zero-out all values before the broadcast, causing
      // the modal to show "0.0000" for Ku/Tu/Kp/Kd.
      // Solution: preserve gAtResult when leaving via a successful completion;
      // only clear it on abort (phase==3) or mid-run transitions.
      // Reset legacy ZN AT state (kept for non-AT8 compat; AT8 persists via NVS)
      if (atPhase != -1) {
        bool preserveResult = (atPhase == 2);
        atPhase = -1;
atCurrentKp = gAtKpStart[constrain(atTuneMode, 0, 2)];  // [FIX-PERMODE]
        atPhaseTicks = 0;
        atCrossCount = 0;
        atLastErrSign = 0.0f;
        atStallTicks = 0;
        atViolentTicks = 0;
        atLastLSpd = 0;
        atLastRSpd = 0;
        atRampedBase = AT_SAFE_START_PWM;
        if (!preserveResult) {
          portENTER_CRITICAL(&gAtResMux);
          gAtResult = AutoTuneResult{};
          portEXIT_CRITICAL(&gAtResMux);
        }
      }
      // [AT8] Reset soft state but NOT profiles — they persist via NVS
      at8StallTix = 0;
      at8ViolTix = 0;
      at8LastL = 0;
      at8LastR = 0;
      at8RampedSpd = AT_SAFE_START_PWM;
      at8PrevErr = 0.0f;
      at8DFilt = 0.0f;

      lastUs = micros();
      lastState = localState;
      if (!calibOwnsQtr) qtrStartCharge();
      continue;
    }

    // ── AT8: check start/stop requests ───────────────────────────────
    if (isAutoTuning) {
      bool stopReq = false;
      portENTER_CRITICAL(&gAtMux);
      stopReq = gAtStopReq;
      gAtStopReq = false;
      portEXIT_CRITICAL(&gAtMux);
      if (stopReq) {
        stopAll();
        setState(State::IDLE);
        atPhase = 3;
        at8Ph = AT8Phase::ABORTED;
        // [FIX-APPLY] Apply best AT8 values found so far to gPID RAM before saving.
        // Without this, aborting AT8 before a TRN_TUNE convergence discards ALL tuning
        // work — RUNNING mode falls back to original UI values as if AT8 never ran.
        {
          // Use last confirmed speed (current at8Spd may be mid-test at higher level)
          int appliedSpd = (at8Spd > AT8_SPD_START) ? (at8Spd - AT8_SPD_STEP) : at8Spd;
          appliedSpd = max(appliedSpd, AT8_SPD_START);
          portENTER_CRITICAL(&gPidMux);
      if (at8KpS > TWIDDLE_KP_MIN) {
            gPID[0].kP = at8KpS;
            // [FIX-KD] Only apply KdS if Twiddle actually tuned it (>min threshold)
            // KdS=0 means straight profile never had terrain-filtered ticks → skip
            if (at8KdS >= TWIDDLE_KD_MIN) {
              gPID[0].kD = at8KdS;
              webLog("AT8:[STOP] M0 Kp=%.4f Kd=%.4f applied.", at8KpS, at8KdS);
            } else {
              webLog("AT8:[STOP] M0 Kp=%.4f applied. Kd=%.4f NOT applied (not tuned) → kept=%.4f",
                     at8KpS, at8KdS, gPID[0].kD);
            }
gPID[0].base = appliedSpd;
            gPID[0].top  = appliedSpd + constrain(appliedSpd / 4, 20, 50);  // [FIX] auto-adjust
            gPID[0].low  = constrain(appliedSpd * 2 / 3, 15, appliedSpd - 10);
          }
          // [MODE-ISOLATION] Only push turn profile on abort for M1/M2 runs.
          // Mode 0 tunes ONLY gPID[0]; at8KpT for M0 is an uninitialised seed —
          // writing it to gPID[1]/[2] would corrupt previously-tuned turn values.
          if (at8KpT > TWIDDLE_KP_MIN && atTuneMode != 0) {
            int _st = appliedSpd + constrain(appliedSpd / 4, 20, 50);
            int _sl = constrain(appliedSpd * 2 / 3, 15, appliedSpd - 10);
            gPID[1].kP = at8KpT;  gPID[1].kD = at8KdT;
            gPID[1].base = appliedSpd; gPID[1].top = _st; gPID[1].low = _sl;
            gPID[2].kP = at8KpT * 1.4f; gPID[2].kD = at8KdT * 1.4f;
            gPID[2].base = appliedSpd; gPID[2].top = _st; gPID[2].low = _sl;
          }
          portEXIT_CRITICAL(&gPidMux);
          webLog("AT8: [STOP] Best values applied to gPID. spd=%d M0 Kp=%.4f Kd=%.4f",
                 appliedSpd, at8KpS, at8KdS);
          // Trigger UI refresh with applied values
          portENTER_CRITICAL(&gPidFetchMux);
          gPidFetchPending = true;
          portEXIT_CRITICAL(&gPidFetchMux);
        }
        // [AT8-1] Save checkpoint so resume lands in correct state
        at8NvsSave(at8Ph, at8Spd, at8KpS, at8KdS, at8KpT, at8KdT,
                   at8SLocked, at8SpdCnf, atTuneMode);
        webLog("[AT8-STOP] Stop req. Motors cut. mode=M%d phase=%d spd=%d State→IDLE.",
               atTuneMode, (int)at8Ph, at8Spd);
        portENTER_CRITICAL(&gAtResMux);
        gAtResult.phase = 3;
        portEXIT_CRITICAL(&gAtResMux);
        if (!calibOwnsQtr) qtrStartCharge();
        continue;
      }
      navState = NavState::LINE_FOLLOW;

      bool startReq = false;
      portENTER_CRITICAL(&gAtMux);
      startReq = gAtStartReq;
      gAtStartReq = false;
      portEXIT_CRITICAL(&gAtMux);
      if (startReq || !at8Initialized) {
        // ── [FIX-RESET] Read force-reset flag + mode atomically ──────────
        bool forceReset = false;
        portENTER_CRITICAL(&gAtMux);
        forceReset  = gAt8ForceReset;
        gAt8ForceReset = false;
        atTuneMode  = gAtTuneMode;  // read mode early for NVS mode-match
        portEXIT_CRITICAL(&gAtMux);
        webLog("[AT8-INIT] startReq=%d initialized=%d forceReset=%d mode=M%d",
               (int)startReq, (int)at8Initialized, (int)forceReset, atTuneMode);

        // ── [AT8-1] Attempt NVS resume ────────────────────────────────
        AT8Phase resumePh;
        int resumeSpd, resumeCnf;
        float resumeKpS, resumeKdS, resumeKpT, resumeKdT;
        bool resumeSLk;

        // [FIX-RESET] forceReset=true → skip NVS entirely (cold start).
        // [FIX-PERMODE] Pass atTuneMode so only a matching-mode checkpoint resumes.
        bool hasCheckpoint = !forceReset &&
                             at8NvsLoad(resumePh, resumeSpd,
                                        resumeKpS, resumeKdS,
                                        resumeKpT, resumeKdT,
                                        resumeSLk, resumeCnf,
                                        atTuneMode);
        webLog("[AT8-INIT] hasCheckpoint=%d forceReset=%d", (int)hasCheckpoint, (int)forceReset);

        if (hasCheckpoint && !startReq) {
          // RESUME path (power-cycle / crash recovery — NOT a user-pressed Start)
          at8Ph     = resumePh;
          at8Spd    = resumeSpd;
          at8KpS    = resumeKpS;
          at8KdS    = resumeKdS;
          at8KpT    = resumeKpT;
          at8KdT    = resumeKdT;
          at8SLocked = resumeSLk;
          at8SpdCnf = resumeCnf;
          webLog("[AT8-RESUME] mode=M%d spd=%d sLocked=%d phase=%d",
                 atTuneMode, at8Spd, (int)at8SLocked, (int)at8Ph);
          webLog("[AT8-RESUME] Str Kp=%.4f Kd=%.4f  Trn Kp=%.4f Kd=%.4f",
                 at8KpS, at8KdS, at8KpT, at8KdT);
        } else {
          // FRESH START (user-pressed Start, OR no checkpoint, OR force reset)
          portENTER_CRITICAL(&gAtMux);
          at8Spd = constrain(gAtModeSpeed[constrain(atTuneMode, 0, 2)], 50, 255);
          portEXIT_CRITICAL(&gAtMux);

          if (forceReset) {
            // [FIX-RESET] COLD start: use TWIDDLE_KP_INIT_STEP, ignoring gPID
            at8KpS = TWIDDLE_KP_INIT_STEP;
            at8KdS = TWIDDLE_KD_INIT_STEP;
            at8KpT = TWIDDLE_KP_INIT_STEP;
            at8KdT = TWIDDLE_KD_INIT_STEP;
            webLog("[AT8-COLD] Force reset → TWIDDLE_KP_INIT_STEP seed. mode=M%d spd=%d",
                   atTuneMode, at8Spd);
          } else {
            // WARM start: seed from per-mode gAtKpStart[] (user-configurable via UI)
            int _km = constrain(atTuneMode, 0, 2);
            at8KpS = gAtKpStart[0];         // straight always from M0 start
            at8KdS = TWIDDLE_KD_INIT_STEP;
            at8KpT = gAtKpStart[_km > 0 ? _km : 1];  // turn from M1 or M2
            at8KdT = TWIDDLE_KD_INIT_STEP;
            webLog("[AT8-WARM] mode=M%d seed KpS=%.4f KpT=%.4f spd=%d",
                   atTuneMode, at8KpS, at8KpT, at8Spd);
          }

          // Select starting phase based on user-selected mode
          if (atTuneMode == 1) {
            at8Ph = AT8Phase::TRN_TUNE;
            at8SLocked = true;
            webLog("[AT8-M1-TURN] TRN_TUNE only. spd=%d KpT=%.4f KdT=%.4f",
                   at8Spd, at8KpT, at8KdT);
          } else if (atTuneMode == 2) {
            at8Ph = AT8Phase::TRN_TUNE;
            at8SLocked = true;
            webLog("[AT8-M2-FAST] TRN_TUNE only. spd=%d KpT=%.4f KdT=%.4f",
                   at8Spd, at8KpT, at8KdT);
          } else {
            at8Ph = AT8Phase::STR_TUNE;
            at8SLocked = false;
            webLog("[AT8-M0-STR] STRICT STRAIGHT-ONLY (gPID[0] only). spd=%d KpS=%.4f KdS=%.4f",
                   at8Spd, at8KpS, at8KdS);
          }
          at8SpdCnf = 0;
          at8NvsClear(-1);  // clear stale checkpoint from any previous mode
        }
        // Common reset for both paths
         at8RampedSpd = AT_SAFE_START_PWM;
        at8StallTix = 0;
        at8ViolTix = 0;
        at8LastL = 0;
        at8LastR = 0;
        at8PrevErr = 0.0f;
        at8DFilt = 0.0f;
        at8M0CornerActive = false;  // [M0-CORNER] full reset on every new AT run
        at8M0BrakeTicks   = 0;
        at8M0CornerTicks  = 0;
        at8M0HardBraking  = false;
        at8Terrain = ATTerrain::UNKNOWN;
        at8StrCnt = 0;
        at8TrnCnt = 0;
        memset(at8ErrBuf, 0, sizeof(at8ErrBuf));
        at8EBufIdx = 0;
        at8EBufFill = 0;
        at8WfN = 0;
        at8WfMean = 0.0f;
        at8WfM2 = 0.0f;
        at8LockHits = 0;
        at8TwPar = TWParam::KP;
        at8TwDir = TWDir::IDLE;
        at8TwEval = TWEval::SETTLE;
        at8TwDpKp = gAtKpStep[constrain(atTuneMode, 0, 2)];  // [FIX-PERMODE] per-mode dpKp
        at8TwDpKd = TWIDDLE_KD_INIT_STEP;
        webLog("[AT8-TWIDDLE-INIT] dpKp=%.4f dpKd=%.4f mode=M%d",
               at8TwDpKp, at8TwDpKd, atTuneMode);
        at8TwBestFit = 1.0e9f;
        at8TwCurFit = 0.0f;
        at8TwTick = 0;
        at8TwTerrTick = 0;
        at8TwBaseline = true;
        at8TwRounds = 0;
        at8Initialized = true;

        portENTER_CRITICAL(&gAtResMux);
        gAtResult = AutoTuneResult{};
        gAtResult.phase = 0;
        portEXIT_CRITICAL(&gAtResMux);
        webLog("AT8: Init complete. Phase=%d Spd=%d", (int)at8Ph, at8Spd);
      }
    }

    if (!gQtrIrOn) {
      digitalWrite(PIN_QTR_IR, HIGH);
      gQtrIrOn = true;
    }

    // ── dt ────────────────────────────────────────────────────────────
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;
    lastUs = nowUs;

    // ──────────────────────────────────────────────────────────────────
    //  SENSOR DATA  — QTR position + look-ahead
    // ──────────────────────────────────────────────────────────────────
uint8_t srRaw = sr165Read();
    bool s1=false, s2=false, s3=false, s4=false;
    if (gObsEn) parseLookAhead(srRaw, s1, s2, s3, s4);

    // [FIX-LA-STUCK] If all 4 sensors trigger simultaneously for too long
    // during LINE_FOLLOW, it means sensors are reading the track line itself
    // (false positive). Auto-disable HW look-ahead → fallback to math braking.
    {
      static int _laStuckCnt = 0;
      static bool _laWasHealthy = true;
      bool allTriggered = s1 && s2 && s3 && s4;
      bool inLineFollow = (navState == NavState::LINE_FOLLOW);
      // [FIX] Do not count as "stuck" while a 90° turn is active or building.
      // At a junction the perpendicular line legitimately fires all 4 sensors
      // simultaneously for the entire brake-hold duration (>1.5 s), which is
      // indistinguishable from a hardware fault without this context guard.
      // at8TrnCnt > 0 catches early turn entry before terrain SM commits to TURN_90.
      bool turnActive   = (at8Terrain == ATTerrain::TURN_90 || at8TrnCnt > 0);
      if (allTriggered && inLineFollow && gLookAheadHealthy && !turnActive) {
        _laStuckCnt++;
        if (_laStuckCnt >= 300) { // 1.5 seconds stuck → mark unhealthy
          gLookAheadHealthy    = false;
          gActiveBrakingSystem = "MATH (LA-STUCK)";
          s1 = s2 = s3 = s4 = false; // suppress false triggers
          webLog("[LA-STUCK] All sensors stuck HIGH for 1.5s! HW braking DISABLED → Math fallback.");
          _laStuckCnt = 0;
        }
      } else if (!allTriggered || turnActive) {
        _laStuckCnt = 0; // reset if not all stuck OR actively in a turn
      }
      // Re-enable if sensor pattern becomes normal
      if (!gLookAheadHealthy && !allTriggered && _laWasHealthy == false) {
        static int _laRecoverCnt = 0;
        if (++_laRecoverCnt >= 100) {
          gLookAheadHealthy    = true;
          gActiveBrakingSystem = "HARDWARE (HLA)";
          webLog("[LA-STUCK] Sensors recovered — HW braking RE-ENABLED.");
          _laRecoverCnt = 0; _laWasHealthy = true;
        }
      }
      _laWasHealthy = !allTriggered;
    }
    {
      static int _laCnt = 0;
      if (++_laCnt >= 100) {
        _laCnt = 0;
        webLog("[LA-SENSOR] raw=0x%02X s1=%d s2=%d s3=%d s4=%d healthy=%d obsEn=%d",
               srRaw, (int)s1, (int)s2, (int)s3, (int)s4, (int)gLookAheadHealthy, (int)gObsEn);
      }
    }

    // Look-ahead state: outer sensors (S1/S4) at 6cm, inner (S2/S3) at 2cm
    LookAheadState laStateNow = LookAheadState::STRAIGHT;
    if (gLookAheadHealthy && gObsEn) {
      if (s2 || s3) laStateNow = LookAheadState::PREP_2CM;
      else if (s1 || s4) laStateNow = LookAheadState::BRAKE_6CM;
    }

    // LA turn hint from outer sensors
    if (gLookAheadHealthy && gObsEn) {
      if ((s1 || s2) && !(s3 || s4)) laTurnHint = -1;
      else if ((s3 || s4) && !(s1 || s2)) laTurnHint = 1;
    }

    // Compose obsLa bitmask (for telemetry)
    uint8_t obsLa = (uint8_t)((s1 ? 1 : 0) | (s2 ? 2 : 0) | (s3 ? 4 : 0) | (s4 ? 8 : 0));

    // ── QTR position calculation ──────────────────────────────────────
    uint8_t crossMinNow = gCrossMinSensors;
    uint8_t lw = gLineWidthSensors;

    long wSum = 0;
    long wTot = 0;
    int blackCount = 0;
    bool allBlack = true;
    bool allWhite = true;
    for (int i = 0; i < 8; i++) {
      if (qCal[i] >= QTR_LOST_THR) { allWhite = false; }
      if (qCal[i] < QTR_LOST_THR) { allBlack = false; }
      if (qCal[i] >= CROSS_BLACK_THR) blackCount++;
      if (qCal[i] >= SINGLE_SENSOR_MIN) {
        wSum += (long)qCal[i] * (long)(i * 1000);
        wTot += (long)qCal[i];
      }
    }
    int pos = (wTot > 0) ? (int)(wSum / wTot) : lastPos;
    bool lineVisible = (wTot > 0);

    if (lineVisible) lastPos = pos;
    if (lineVisible && (pos > QTR_SETPOINT)) lineSide = 1;
    if (lineVisible && (pos < QTR_SETPOINT)) lineSide = -1;

    // [U3] Update last known error sign whenever line is visible
    if (lineVisible) {
      float curErrSign = (float)(pos - QTR_SETPOINT);
      if (curErrSign > 50.0f) lastKnownErrSign = 1;
      else if (curErrSign < -50.0f) lastKnownErrSign = -1;
    }
    // [FIX] Removed !allBlack gate: solid T-junctions briefly make allBlack=true,
    // causing crossSeen=false. The cross debounce never starts, LINE_FOLLOW stays
    // active, and the end-zone timer falsely reaches END_DETECT_TICKS (30 ms) first.
    bool crossSeen = (blackCount >= (int)crossMinNow) && !allWhite;
    // ── [U7] 45-DEGREE DUMMY LINE FILTER ────────────────────────────
    //  Compute de/dt. A 45-degree line crossing produces a fast, monotonic
    //  sweep of the error while a true cross produces a sudden bilateral
    //  activation. If we detect |de/dt| > DUMMY45_RATE_THR for
    //  DUMMY45_CONFIRM_TICKS consecutive ticks, suppress the cross event
    //  and bias steering straight.
    {
      float errNow = (float)(pos - QTR_SETPOINT);
      float dedt = (errNow - prevErrForSlope);  // units per tick
      prevErrForSlope = errNow;

      if (dummy45CoolTicks > 0) dummy45CoolTicks--;

      if (crossSeen && dummy45CoolTicks == 0) {
        if (fabsf(dedt) > DUMMY45_RATE_THR) {
          diagSweepTicks++;
          if (diagSweepTicks >= DUMMY45_CONFIRM_TICKS) {
            // Filter: suppress this cross event as a 45-degree dummy
            crossSeen = false;
            dummy45Active = true;
            dummy45CoolTicks = DUMMY45_COOL_TICKS;
            diagSweepTicks = 0;
            webLog("[U7] 45° dummy line filtered (de/dt=%.0f). Maintaining course.", dedt);
          }
        } else {
          diagSweepTicks = 0;
        }
      } else {
        if (!crossSeen) diagSweepTicks = 0;
        if (dummy45Active && dummy45CoolTicks == 0) dummy45Active = false;
      }
    }

    // ── End-zone detection (unchanged) ───────────────────────────────
    bool endConfirmed = false;
    // [FIX-END-STUCK] Use blackCount>=8 (all sensors >= CROSS_BLACK_THR=700) instead of
    // allBlack (all sensors >= QTR_LOST_THR=400). allBlack fires falsely at thick junctions
    // when moving slowly (after ABS braking). A true finish box puts ALL 8 sensors above 700;
    // a 2cm junction line puts at most 4-6 sensors above 700 even at the dead centre.
    if (blackCount >= 8) {
      endDetectTicks++;
      if (endDetectTicks >= 40) endConfirmed = true;
    } else {
      endDetectTicks = 0;
    }

    // Cross debounce (unchanged)
    bool crossConfirmed = false;
    if (crossSeen && !allBlack) {
      crossDebTicks++;
      if (crossDebTicks >= CROSS_DEBOUNCE_TICKS && crossBlankTicks <= 0) crossConfirmed = true;
    } else {
      crossDebTicks = 0;
    }
    if (crossBlankTicks > 0) crossBlankTicks--;

    // ── PID mode selection ────────────────────────────────────────────
    int pm = 0;
    float absErrForMode = fabsf((float)(pos - QTR_SETPOINT));
    if (absErrForMode > HYBRID_MODE2_ERR) pm = 2;
    else if (absErrForMode > HYBRID_MODE1_ERR) pm = 1;

    // ═════════════════════════════════════════════════════════════════
    //  [ABS+HYBRID] INTELLIGENT DUAL BRAKING SYSTEM
    //  Priority 1 — Hardware ABS (6cm / 2cm sensors): aggressive instant stop.
    //  Priority 2 — Math fallback (de/dt): soft decel for wide curves.
    //  Rule: both systems NEVER activate simultaneously.
    //        HW sensor ABS takes full priority; math braking is the fallback
    //        only when look-ahead sensors are reported unhealthy.
    // ═════════════════════════════════════════════════════════════════
int laTargetBase = -1;  // -1 = no override (used by math-brake and 2cm soft path)

    // ── [ABS+HYBRID] Braking system — DISABLED during AutoTune ──────
    // Braking fires on look-ahead sensors at every junction, which
    // disrupts PID evaluation during Twiddle. Activate only in RUNNING
    // (control) mode after PID values are finalised via AutoTune.
    if (!isAutoTuning) {

    // ── [ABS] Clear done-flag when all sensors released ───────────────
    if (absDone && gLookAheadHealthy && !(s1 || s2 || s3 || s4)) {
      absDone       = false;
      absFrom6cm    = false;
      absFrom2cm    = false;
      absFromManual = false;
      webLog("[ABS] Sensors clear — guard reset. Ready for next junction.");
    }

    // ── [ABS] Manual brake trigger from Web UI ────────────────────────
    {
      bool manReq = false;
      portENTER_CRITICAL(&gManualBrakeMux);
      manReq = gManualBrakeReq;
      gManualBrakeReq = false;
      portEXIT_CRITICAL(&gManualBrakeMux);
      if (manReq && !absActive && !absDone) {
        absActive     = true;
        absFromManual = true;
        absPhase      = 0;
        absPhaseTicks = 0;
        absCycleCount = 0;
        absCycleMax   = ABS_CYCLES_6CM;
        webLog("[ABS-MANUAL] Manual brake TRIGGERED from Web UI. effBase=%.0f M1base=%d",
               effBase, wp[1].base);
      }
    }

    // ── [ABS] 6cm outer sensor trigger (S1 / S4) ─────────────────────
    // Fires if we see a junction 6cm ahead and speed is above M1 level.
    // This is the primary ABS trigger. Immediately starts 3-cycle ABS.
    if (gLookAheadHealthy && gObsEn && !absActive && !absDone) {
      if (s1 || s4) {
        if ((int)effBase > wp[1].base) {
          // Speed is above M1 — ABS required
          absActive     = true;
          absFrom6cm    = true;
          absPhase      = 0;
          absPhaseTicks = 0;
          absCycleCount = 0;
          absCycleMax   = ABS_CYCLES_6CM;
          webLog("[ABS-6CM] TRIGGERED! effBase=%.0f M1base=%d s1=%d s4=%d hint=%d",
                 effBase, wp[1].base, (int)s1, (int)s4, (int)laTurnHint);
        } else {
          // Already at/below M1 speed — smooth hold only
          laTargetBase = wp[1].base;
        }
      }
    }

    // ── [ABS] 2cm inner sensor fail-safe (S2 / S3) ───────────────────
    // Fires when robot is 2cm from the junction.
    // If ABS already handled speed → just validate and log.
    // If speed is STILL too high (ABS didn't work or was skipped) → 1-cycle hard brake.
    // If speed is OK → soft hold at M1 via laTargetBase.
    if (gLookAheadHealthy && gObsEn && (s2 || s3)) {
      if (!absActive) {
        if ((int)effBase > wp[1].base + ABS_2CM_SPD_MARGIN) {
          // Speed too high at 2cm — emergency single-cycle brake
          absActive     = true;
          absFrom2cm    = true;
          absPhase      = 0;
          absPhaseTicks = 0;
          absCycleCount = 0;
          absCycleMax   = ABS_CYCLES_2CM;
          webLog("[ABS-2CM] FAIL-SAFE! effBase=%.0f > M1+margin(%d) → 1-cycle brake.",
                 effBase, wp[1].base + ABS_2CM_SPD_MARGIN);
        } else {
          // Speed validated OK — soft cap at M1
          laTargetBase = wp[1].base;
          static bool _2cmLogged = false;
          if (!_2cmLogged) {
            webLog("[ABS-2CM] VALIDATE OK. effBase=%.0f <= M1base+margin(%d). Soft hold.",
                   effBase, wp[1].base + ABS_2CM_SPD_MARGIN);
            _2cmLogged = true;
          }
        }
      } else {
        // ABS already running — just log validation state
        static int _2cmActiveCnt = 0;
        if (++_2cmActiveCnt >= 20) {
          _2cmActiveCnt = 0;
          webLog("[ABS-2CM] ABS active at 2cm. effBase=%.0f cycle=%d/%d phase=%d",
                 effBase, absCycleCount, absCycleMax, absPhase);
        }
      }
    } else {
      // 2cm sensors clear — reset one-shot log flag
      static bool _2cmLogged2 = false;
      _2cmLogged2 = false;
      (void)_2cmLogged2;
    }

    // ── [ABS] After ABS completes → lock effBase to M1 for PID ──────
    if (absDone) {
      laTargetBase = wp[1].base;  // keep speed at M1 while passing junction
    }

    // ── When ABS is active, pre-set laTargetBase for post-ABS resume ─
    if (absActive) {
      laTargetBase = wp[1].base;
    }

    // ── [HYBRID-MATH] Fallback braking — only when HW sensors unavailable ──
    // Activates only when: look-ahead healthy=false AND ABS is NOT active.
    // Uses de/dt spike detection for wide curves sensors can't see.
    if (!gLookAheadHealthy && gObsEn && !absActive && !absDone) {
      if (fabsf(gLastDError) > PRED_BRAKE_DRATE_THRESHOLD) {
        if (!hybridBrakeOverride)
          webLog("[BRAKE-MATH] TRIGGERED! dErr=%.0f > thr=%.0f M1base=%d effBase=%.0f",
                 gLastDError, (float)PRED_BRAKE_DRATE_THRESHOLD, wp[1].base, effBase);
        hybridBrakeOverride = true;
        laTargetBase = wp[1].base;
      }
    }
    // Math-brake recovery: error settled → release
    if (hybridBrakeOverride) {
      if (absActive) {
        // HW ABS just triggered — release math brake immediately
        hybridBrakeOverride = false;
        hybridRecoverTicks = 0;
        webLog("[BRAKE-MATH] Overridden by HW ABS — math brake released.");
      } else {
        laTargetBase = wp[1].base;
        if (absErrForMode < HYBRID_RECOVER_ERR_THR) {
          hybridRecoverTicks++;
          if (hybridRecoverTicks >= HYBRID_RECOVER_TICKS) {
            hybridBrakeOverride = false;
            hybridRecoverTicks  = 0;
            laTargetBase = -1;
            webLog("[BRAKE-MATH] RELEASED. err=%.0f < %.0f effBase=%.0f",
                   absErrForMode, (float)HYBRID_RECOVER_ERR_THR, effBase);
          }
        } else {
          hybridRecoverTicks = 0;
        }
      }
    }

    // ── Log active braking system for diagnostics ─────────────────────
    {
      static int _brakeSysCnt = 0;
      if (++_brakeSysCnt >= 200) {
        _brakeSysCnt = 0;
        const char* sysName = absActive   ? (absFrom6cm ? "ABS-6CM" : absFrom2cm ? "ABS-2CM" : "ABS-MANUAL")
                            : absDone     ? "ABS-DONE(hold-M1)"
                            : hybridBrakeOverride ? "MATH-BRAKE"
                            : "NONE";
webLog("[BRAKE-SYS] active=%s effBase=%.0f laTarget=%d s1=%d s2=%d s3=%d s4=%d",
               sysName, effBase, laTargetBase, (int)s1, (int)s2, (int)s3, (int)s4);
      }
    }
    }  // end if (!isAutoTuning) — ABS+Hybrid braking system

    // ─────────────────────────────────────────────────────────────────
    //  END-ZONE / RACE FINISH  (stop only on true finish, not cross)
    // ─────────────────────────────────────────────────────────────────
    if (endConfirmed && navState == NavState::LINE_FOLLOW) {
      setState(State::STOPPED);
      stopAll();
      webLog("END ZONE: All sensors black — RACE COMPLETE. Motors cut.");
      portENTER_CRITICAL(&gTelMux);
      gTel.lineLost = true;
      portEXIT_CRITICAL(&gTelMux);
      if (!calibOwnsQtr) qtrStartCharge();
      continue;
    }

    // ─────────────────────────────────────────────────────────────────
    //  NAVIGATION STATE MACHINE
    // ─────────────────────────────────────────────────────────────────
    NavState prevNavState = navState;
    int lSpd = lastLSpd;
    int rSpd = lastRSpd;
    // [ABS] skipPID=true when ABS is overriding motors — no point computing PID output
    bool skipPID = absActive;

    switch (navState) {

      // ═══════════════════════════════════════════════════════════════
      //  STATE 0: LINE_FOLLOW  (primary PID follow mode)
      // ═══════════════════════════════════════════════════════════════
      case NavState::LINE_FOLLOW:
        if (crossConfirmed && !allBlack) {
          navState = NavState::AT_INTERSECTION;
          crossPassTicks = 0;
          crossDebTicks = 0;
          juncHasLeft = false;
          juncHasFwd = false;
          juncHasRight = false;
          juncLeftWideTicks = 0;
          juncRightWideTicks = 0;
          juncFwdTicks = 0;
          juncEntryHint = laTurnHint;
          lineLostTicks = 0;
          postUTurnRouting = false;  // new junction, reset anti-backtrack context
          webLog("JUNC: Cross! %d/%d sensors black.", blackCount, crossMinNow);
        } else if (!lineVisible) {
          lineLostTicks++;
          // [U3] Directional memory recovery — spin immediately toward last known side
          if (lineLostTicks > LINE_LOST_MAX_TICKS) {
            navState = NavState::PIVOT_SEARCH;
            searchTicks = 0;
            recoverySpinActive = true;
            webLog("[U3] Line lost. Recovery spin %s (lastKnownSign=%d).",
                   lastKnownErrSign > 0 ? "RIGHT" : "LEFT", lastKnownErrSign);
          }
        } else {
          lineLostTicks = 0;
          recoverySpinActive = false;
        }
        break;

      // ═══════════════════════════════════════════════════════════════
      //  STATE 2: PIVOT_SEARCH
      //  [U3] UPGRADED: infinite spin toward lastKnownErrSign until
      //  line is found. No timeout-stop. Pure directional memory recovery.
      // ═══════════════════════════════════════════════════════════════
      case NavState::PIVOT_SEARCH:
        skipPID = true;
        if (lineVisible) {
          navState = NavState::LINE_FOLLOW;
          lineLostTicks = 0;
          recoverySpinActive = false;
          webLog("[U3] Pivot-search: line re-acquired at tick %d.", searchTicks);
        } else {
          searchTicks++;
          // [U3] Spin continuously toward lastKnownErrSign
          int ss = wp[2].low + 5;
          if (recoverySpinActive) {
            // Spin toward the last known line position
            if (lastKnownErrSign > 0) {
              lSpd = ss;
              rSpd = -ss;
            }  // line was right → spin right
            else {
              lSpd = -ss;
              rSpd = ss;
            }  // line was left  → spin left
          } else {
            // Fallback alternating search (if recoverySpinActive somehow cleared)
            int phase = searchTicks / SEARCH_PHASE_TICKS;
            bool flipDir = (phase % 2 == 1);
            int eff = flipDir ? -lineSide : lineSide;
            if (eff == 0) eff = 1;
            if (eff > 0) {
              lSpd = ss;
              rSpd = -ss;
            } else {
              lSpd = -ss;
              rSpd = ss;
            }
          }
          // Safety: after very long search with no line, continue spinning
          // (remove the old STOPPED transition — per spec §3 robot must NOT stop)
         if (searchTicks > 120) { 
            webLog("WARN: Recovery spin timeout. Stopping to prevent going backwards.");
            setState(State::STOPPED);
            stopAll();
            navState = NavState::LINE_FOLLOW;
          }
        }
        break;

        // ═══════════════════════════════════════════════════════════════
        //  STATE 3: AT_INTERSECTION
        //  [U5] Node map lookup / [U6] Anti-backtrack / [U7] dummy filtered
        //  [U8] T-junction priority: STRAIGHT > RIGHT > LEFT
        // ═══════════════════════════════════════════════════════════════
      case NavState::AT_INTERSECTION:
        {
          skipPID = true;
          crossPassTicks++;
          // [FIX] Use M1 (curve) base speed for pass-through, not M0 full speed.
          // The old code undid all LA braking the instant a junction was confirmed,
          // making the robot lurch forward just as it needed to turn.
          int baseSpd = wp[1].base;

          // Pass-through phase: sample arm widths
          if (crossPassTicks <= CROSS_PASSTHRU_TICKS) {
            lSpd = baseSpd;
            rSpd = baseSpd;
            bool L_wide = (qCal[0] >= CROSS_BLACK_THR) && (qCal[1] >= CROSS_BLACK_THR);
            bool R_wide = (qCal[6] >= CROSS_BLACK_THR) && (qCal[7] >= CROSS_BLACK_THR);
            bool F_any = (qCal[3] >= CROSS_BLACK_THR) || (qCal[4] >= CROSS_BLACK_THR);
            if (L_wide) juncLeftWideTicks++;
            if (R_wide) juncRightWideTicks++;
            if (F_any) juncFwdTicks++;
          } else {
            // ── Finalise arm stub detection ──────────────────────────────
            int rightReqTicks = (gLookAheadHealthy && juncEntryHint > 0)
                                  ? STUB_MIN_ARM_WIDE_TICKS_HINTED
                                  : STUB_MIN_ARM_WIDE_TICKS;
            int leftReqTicks = (gLookAheadHealthy && juncEntryHint < 0)
                                 ? STUB_MIN_ARM_WIDE_TICKS_HINTED
                                 : STUB_MIN_ARM_WIDE_TICKS;
            juncHasLeft = (juncLeftWideTicks >= leftReqTicks);
            juncHasRight = (juncRightWideTicks >= rightReqTicks);
            juncHasFwd = (juncFwdTicks >= STUB_MIN_FWD_TICKS);

            uint8_t dirsAvail = (juncHasLeft ? DIR_LEFT : 0) | (juncHasFwd ? DIR_FWD : 0) | (juncHasRight ? DIR_RIGHT : 0);
            if (dirsAvail == 0) {
              dirsAvail = DIR_FWD;
              webLog("JUNC: WARN - 0 exits. Forcing FWD.");
            }

            webLog("STUB: Lw=%d Rw=%d Fw=%d hint=%+d -> L=%d F=%d R=%d",
                   juncLeftWideTicks, juncRightWideTicks, juncFwdTicks, (int)juncEntryHint,
                   (int)juncHasLeft, (int)juncHasFwd, (int)juncHasRight);

            // ── [U5] Node Map: look up or register this junction ─────────
            // Search existing nodes for a matching dirs signature
            int8_t foundIdx = -1;
            for (uint8_t i = 0; i < nodeCount; i++) {
              uint8_t idx = (nodeHead + i) % NODE_MAP_SIZE;
              if (nodeMap[idx].valid && nodeMap[idx].dirsAvail == dirsAvail) {
                foundIdx = (int8_t)idx;
                break;
              }
            }

            uint8_t dirsTried = 0;
            if (foundIdx >= 0) {
              // Revisiting a known node
              dirsTried = nodeMap[foundIdx].dirsTried;
              curNodeIdx = foundIdx;
              webLog("[U5] Revisit node idx=%d tried=0x%02X avail=0x%02X", foundIdx, dirsTried, dirsAvail);
            } else {
              // New node — register in FIFO
              if (nodeCount < NODE_MAP_SIZE) {
                nodeMap[nodeTail] = { dirsAvail, 0, 0, true };
                curNodeIdx = (int8_t)nodeTail;
                nodeTail = (nodeTail + 1) % NODE_MAP_SIZE;
                nodeCount++;
              } else {
                // FIFO full — evict oldest entry
                nodeMap[nodeHead] = { dirsAvail, 0, 0, true };
                curNodeIdx = (int8_t)nodeHead;
                nodeHead = (nodeHead + 1) % NODE_MAP_SIZE;
                webLog("[U5] FIFO full — oldest node evicted.");
              }
              dirsTried = 0;
              webLog("[U5] New node registered. avail=0x%02X nodes=%d", dirsAvail, nodeCount);
            }

            // ── [U6] Anti-backtrack: apply last junction tried mask ───────
            uint8_t dirsBlocked = 0;
            if (postUTurnRouting) {
              // Block paths that were already tried at this junction
              dirsBlocked = dirsTried;
              webLog("[U6] Anti-backtrack: blocking tried dirs=0x%02X", dirsBlocked);
              postUTurnRouting = false;
            }

            // Available but not yet tried (and not blocked)
            uint8_t dirsUntried = dirsAvail & ~dirsBlocked & ~dirsTried;
            if (dirsUntried == 0) {
              // All paths tried — fall back to any available
              dirsUntried = dirsAvail & ~dirsBlocked;
              if (dirsUntried == 0) dirsUntried = dirsAvail;  // last resort
              webLog("[U5/U6] All paths tried. Fallback to any avail dir.");
            }

           // ── PURE MAZE SOLVING (Right-Hand Rule) ──
            bool isFullCross = (dirsAvail & DIR_LEFT) && (dirsAvail & DIR_FWD) && (dirsAvail & DIR_RIGHT);

            uint8_t chosenDir = 0;
            
            // අපි dirsUntried වෙනුවට dirsAvail පාවිච්චි කරනවා.
            // U-Turn එකක් ගහලා ආපහු එද්දී robot එකට සාපේක්ෂව directions මාරු වෙන නිසා,
            // මේ විදිහට Right > Straight > Left හැරෙන්න දුන්නාම නිරායාසයෙන්ම 
            // කලින් නොගියපු (අනිත්) පාරට හැරෙනවා.
            if (dirsAvail & DIR_RIGHT) chosenDir = DIR_RIGHT;
            else if (dirsAvail & DIR_FWD) chosenDir = DIR_FWD;
            else if (dirsAvail & DIR_LEFT) chosenDir = DIR_LEFT;
            if (chosenDir == 0) {
              chosenDir = DIR_FWD;
              webLog("JUNC: WARN no dir — forcing FWD.");
            }

            // Mark chosen dir as tried in node map
            if (curNodeIdx >= 0) nodeMap[curNodeIdx].dirsTried |= chosenDir;
            lastTurnDir = chosenDir;

            webLog("JUNC[U5/U8]: exits=0x%02X tried=0x%02X chosen=%s %s",
                   dirsAvail, dirsTried,
                   (chosenDir == DIR_RIGHT) ? "RIGHT" : (chosenDir == DIR_FWD) ? "FWD"
                                                                               : "LEFT",
                   isFullCross ? "[CROSS]" : "[T-JNC]");

            // ── Execute chosen direction ──────────────────────────────────
            if (chosenDir == DIR_RIGHT) {
              navState = NavState::INTER_TURN_R;
              interPassTicks = 0;
              interTurnTicks = 0;
              lSpd = baseSpd;
              rSpd = baseSpd;
            } else if (chosenDir == DIR_FWD) {
              navState = NavState::LINE_FOLLOW;
              lineLostTicks = -CROSS_BLANK_TICKS;
              crossBlankTicks = CROSS_BLANK_TICKS;
              lSpd = baseSpd;
              rSpd = baseSpd;
            } else {  // DIR_LEFT
              navState = NavState::INTER_TURN_L;
              interPassTicks = 0;
              interTurnTicks = 0;
              lSpd = baseSpd;
              rSpd = baseSpd;
            }
          }
          break;
        }

        // ═══════════════════════════════════════════════════════════════
        //  STATE 4: INTER_TURN_L  [U5.1] Sensor-based left 90 pivot
        // ═══════════════════════════════════════════════════════════════
case NavState::INTER_TURN_L: {
      skipPID = true;
      int tSpd = max(wp[2].low + MOTOR_DEADBAND + 5, MOTOR_DEADBAND + 20);
      // [FIX-TURN] Use M1 base capped by LA-braked effBase — prevents fly-off
      int passSpd = min(wp[1].base, (int)effBase);
      if (interPassTicks < INTER_PASSTHRU_TICKS) {
        if (interPassTicks == 0)
          webLog("[TURN-L] ENTRY passSpd=%d (M1base=%d effBase=%.0f)",
                 passSpd, wp[1].base, effBase);
        interPassTicks++;
        lSpd = passSpd; rSpd = passSpd;
          } else {
            if (interTurnTicks == 0)
              webLog("[TURN-L] SPIN tSpd=%d effBase=%.0f", tSpd, effBase);
            interTurnTicks++;
            lSpd = -tSpd;
            rSpd = tSpd;
            bool lineOnRight = (qCal[6] >= TURN_SENSE_THR || qCal[7] >= TURN_SENSE_THR);
            bool sensorDone = (interTurnTicks >= INTER_TURN_MIN_TICKS) && lineOnRight;
            bool timeoutDone = (interTurnTicks >= INTER_TURN_TICKS);
            if (sensorDone || timeoutDone) {
              if (sensorDone) webLog("JUNC: Left turn sensor-complete %d ticks.", interTurnTicks);
              else webLog("JUNC: Left turn timeout %d ticks.", interTurnTicks);
              navState = NavState::LINE_FOLLOW;
              lineLostTicks = -CROSS_BLANK_TICKS;
              crossBlankTicks = CROSS_BLANK_TICKS;
              hybridBrakeOverride = false;  // [U9] release brake after turn
              hybridRecoverTicks = 0;
            }
          }
          break;
        }

        // ═══════════════════════════════════════════════════════════════
        //  STATE 5: INTER_TURN_R  [U5.1] Sensor-based right 90 pivot
        // ═══════════════════════════════════════════════════════════════
case NavState::INTER_TURN_R: {
      skipPID = true;
      int tSpd = max(wp[2].low + MOTOR_DEADBAND + 5, MOTOR_DEADBAND + 20);
      // [FIX-TURN] Use M1 base capped by LA-braked effBase — prevents fly-off
      int passSpd = min(wp[1].base, (int)effBase);
      if (interPassTicks < INTER_PASSTHRU_TICKS) {
        if (interPassTicks == 0)
          webLog("[TURN-R] ENTRY passSpd=%d (M1base=%d effBase=%.0f)",
                 passSpd, wp[1].base, effBase);
        interPassTicks++;
        lSpd = passSpd; rSpd = passSpd;
          } else {
            if (interTurnTicks == 0)
              webLog("[TURN-R] SPIN tSpd=%d effBase=%.0f", tSpd, effBase);
            interTurnTicks++;
            lSpd = tSpd;
            rSpd = -tSpd;
            bool lineOnLeft = (qCal[0] >= TURN_SENSE_THR || qCal[1] >= TURN_SENSE_THR);
            bool sensorDone = (interTurnTicks >= INTER_TURN_MIN_TICKS) && lineOnLeft;
            bool timeoutDone = (interTurnTicks >= INTER_TURN_TICKS);
            if (sensorDone || timeoutDone) {
              if (sensorDone) webLog("JUNC: Right turn sensor-complete %d ticks.", interTurnTicks);
              else webLog("JUNC: Right turn timeout %d ticks.", interTurnTicks);
              navState = NavState::LINE_FOLLOW;
              lineLostTicks = -CROSS_BLANK_TICKS;
              crossBlankTicks = CROSS_BLANK_TICKS;
              hybridBrakeOverride = false;  // [U9] release brake after turn
              hybridRecoverTicks = 0;
            }
          }
          break;
        }

      // ═══════════════════════════════════════════════════════════════
      //  STATE 1: DASHED_FWD
      // ═══════════════════════════════════════════════════════════════
      case NavState::DASHED_FWD:
        skipPID = true;
        if (lineVisible) {
          navState = NavState::LINE_FOLLOW;
          lineLostTicks = 0;
          webLog("JUNC: Dashed gap cleared.");
        } else if (crossConfirmed) {
          navState = NavState::AT_INTERSECTION;
          crossPassTicks = 0;
          crossDebTicks = 0;
          juncHasLeft = false;
          juncHasFwd = false;
          juncHasRight = false;
          juncLeftWideTicks = 0;
          juncRightWideTicks = 0;
          juncFwdTicks = 0;
          juncEntryHint = laTurnHint;
          lSpd = constrain(lastLSpd, -DASHED_COAST_MAX_SPD, DASHED_COAST_MAX_SPD);
          rSpd = constrain(lastRSpd, -DASHED_COAST_MAX_SPD, DASHED_COAST_MAX_SPD);
          webLog("JUNC: Cross found during dashed coast.");
        } else {
          dashedFwdTicks++;
    if (dashedFwdTicks > DASHED_FWD_TIMEOUT) {
            // Dead-end එකකදී U-Turn ගසා ආපසු හැරීම (Maze Solving සඳහා)
            navState = NavState::DEAD_END_UTURN;
            uTurnTicks = 0;
            webLog("DEAD-END. U-turn active to solve maze.");
          } else {
            lSpd = constrain(lastLSpd, -DASHED_COAST_MAX_SPD, DASHED_COAST_MAX_SPD);
            rSpd = constrain(lastRSpd, -DASHED_COAST_MAX_SPD, DASHED_COAST_MAX_SPD);
          }
        }
        break;

      // ═══════════════════════════════════════════════════════════════
      //  STATE 6: DEAD_END_UTURN  [U5.5 + U6]
      //  Reactive 180 U-turn + anti-backtrack via node map
      // ═══════════════════════════════════════════════════════════════
      case NavState::DEAD_END_UTURN:
        {
          skipPID = true;
          int uSpd = max(wp[2].low + MOTOR_DEADBAND + 10, MOTOR_DEADBAND + 30);
          uTurnTicks++;
          // [U8] heading-invariant spin: always use lineSide for relative direction
          if (lineSide >= 0) {
            lSpd = uSpd;
            rSpd = -uSpd;
          } else {
            lSpd = -uSpd;
            rSpd = uSpd;
          }

          if (uTurnTicks > UTURN_MIN_TICKS && lineVisible) {
            navState = NavState::LINE_FOLLOW;
            lineLostTicks = -CROSS_BLANK_TICKS;
            crossBlankTicks = CROSS_BLANK_TICKS;
            laTurnHint = 0;
            // [U6] postUTurnRouting stays true — AT_INTERSECTION will skip tried dirs
            webLog("[U6] U-turn complete tick=%d. Line found. Anti-backtrack active.", uTurnTicks);
          } else if (uTurnTicks > UTURN_MAX_TICKS) {
            // [U3] After max ticks, switch to directional spin recovery instead of stopping
            navState = NavState::PIVOT_SEARCH;
            recoverySpinActive = true;
            searchTicks = 0;
            webLog("[U3/U6] U-turn timed out (%d ms). Entering recovery spin.", UTURN_MAX_TICKS * 5);
          }
          break;
        }

      default:
        navState = NavState::LINE_FOLLOW;
        break;
    }  // end switch (navState)

    // ── Transition guard — reset PID/EMA on re-entry to LINE_FOLLOW ──
    if (prevNavState != NavState::LINE_FOLLOW && navState == NavState::LINE_FOLLOW) {
      pidI = 0.0f;
      pidPrev = (float)pos - QTR_SETPOINT;
      pidDFiltered = 0.0f;
      if (prevNavState != NavState::AT_INTERSECTION) laTurnHint = 0;
      // [FIX] Pre-seed ramp trackers and effBase to forward base speed.
      // Without this, rampedL/R carry the turn's differential spin values
      // (e.g. +tSpd / -tSpd), causing velRamp to fight a reverse-to-forward
      // transition for ~20 ms after every turn → jerk + momentary speed drop.
      rampedLSpd = wp[pm].base;
      rampedRSpd = wp[pm].base;
      effBase = (float)wp[pm].base;
    }

    // ══════════════════════════════════════════════════════════════════
    //  [AT8] INTELLIGENT DUAL-PROFILE ADAPTIVE AUTO-TUNE BLOCK
    //  Replaces v7.0 ZN block entirely. All 5 AT8 upgrades live here.
    //  Runs only when isAutoTuning == true (State::AUTO_TUNING).
    // ══════════════════════════════════════════════════════════════════
    if (isAutoTuning) {
      skipPID = true;

      // ── [AT8-5] EMERGENCY BRAKE — check FIRST, before everything ────
      bool eBrake = false;
      portENTER_CRITICAL(&gEBrakeMux);
      eBrake = gEBrakeFlag;
      gEBrakeFlag = false;
      portEXIT_CRITICAL(&gEBrakeMux);
      if (eBrake) {
        // TB6612FNG active electronic brake: both IN pins HIGH, PWM=full.
        // This back-drives the MOSFET bridge, locking both motor shafts.
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 255);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 255);
        vTaskDelay(200 / portTICK_PERIOD_MS);  // 200 ms hard lock
        stopAll();                             // then coast to rest
        setState(State::IDLE);
        at8Ph = AT8Phase::ABORTED;
        at8NvsSave(at8Ph, at8Spd, at8KpS, at8KdS, at8KpT, at8KdT,
                   at8SLocked, at8SpdCnf, atTuneMode);
        webLog("[AT8-EBRAKE] !! EMERGENCY BRAKE ACTIVATED !! Robot halted. mode=M%d", atTuneMode);
        webLog("[AT8-EBRAKE] Checkpoint saved → resume from spd=%d mode=M%d.", at8Spd, atTuneMode);
        portENTER_CRITICAL(&gAtResMux);
        gAtResult.phase = 3;
        portEXIT_CRITICAL(&gAtResMux);
        if (!calibOwnsQtr) qtrStartCharge();
        continue;
      }

      float atErr = (float)pos - QTR_SETPOINT;
      float absAtErr = fabsf(atErr);

      // ── Safety: stall / line-loss watchdog ───────────────────────────
// ── Safety: stall / line-loss watchdog ───────────────────────────
      float at8StallThr = AT_STALL_ERR_THR;
      // Removed !lineVisible so it can spin and search without aborting
      if (lineVisible && absAtErr > at8StallThr) {
        at8StallTix++;
        if (at8StallTix >= AT_SAFETY_STALL_TICKS) {
          stopAll();
          setState(State::IDLE);
          webLog("AT8: !! SAFETY ABORT !! Stall/line-loss %d ms. Saving checkpoint.",
                 AT_SAFETY_STALL_TICKS * 5);
          at8Ph = AT8Phase::ABORTED;
          at8NvsSave(at8Ph, at8Spd, at8KpS, at8KdS, at8KpT, at8KdT,
                     at8SLocked, at8SpdCnf, atTuneMode);
          webLog("[AT8-STALL] SAFETY ABORT saved. mode=M%d spd=%d phase=%d KpS=%.4f KpT=%.4f",
                 atTuneMode, at8Spd, (int)at8Ph, at8KpS, at8KpT);
          portENTER_CRITICAL(&gAtResMux);
          gAtResult.phase = 3;
          portEXIT_CRITICAL(&gAtResMux);
          if (!calibOwnsQtr) qtrStartCharge();
          continue;
        }
      } else {
        at8StallTix = 0;
      }

      // ── [AT8-3] TERRAIN DETECTION STATE MACHINE ──────────────────────
      // STRAIGHT: |err| < threshold AND no inner look-ahead, sustained.
      // TURN_90 : |err| > threshold OR inner LA active, sustained.
      // Only terrain-matched ticks count toward Twiddle IAE accumulation.
      {
        bool laActive = (s2 || s3);  // inner look-ahead = definitely turning
        if (absAtErr < TERRAIN_STR_ERR_THR && !laActive) {
          at8StrCnt = min(at8StrCnt + 1, TERRAIN_STR_HOLD + 1);
          at8TrnCnt = max(at8TrnCnt - 1, 0);
        } else if (absAtErr > TERRAIN_TRN_ERR_THR || laActive) {
          at8TrnCnt = min(at8TrnCnt + 1, TERRAIN_TRN_HOLD + 1);
          at8StrCnt = max(at8StrCnt - 1, 0);
        } else {
          // Transition zone — decay both counts slowly
          at8StrCnt = max(at8StrCnt - 1, 0);
          at8TrnCnt = max(at8TrnCnt - 1, 0);
        }
        ATTerrain _prevTerr = at8Terrain;
        if (at8StrCnt >= TERRAIN_STR_HOLD) at8Terrain = ATTerrain::STRAIGHT;
        else if (at8TrnCnt >= TERRAIN_TRN_HOLD) at8Terrain = ATTerrain::TURN_90;
        else at8Terrain = ATTerrain::UNKNOWN;
        if (at8Terrain != _prevTerr) {
          const char* _tN[] = { "UNKNOWN", "STRAIGHT", "TURN-90" };
          webLog("[AT8-TERRAIN] %s→%s err=%.0f sCnt=%d tCnt=%d",
                 _tN[(int)_prevTerr], _tN[(int)at8Terrain], absAtErr, at8StrCnt, at8TrnCnt);
        }
      }

      // ── [AT8-2] WOBBLE / OSCILLATION ANALYSIS ─────────────────────────
      // Maintain a rolling circular buffer of recent error values.
      // Compute RMS over the buffer each tick.
      // During SETTLE phase: if RMS > threshold → micro-reduce active Kp.
      float wobbleRMS = 0.0f;
      {
        // Write new sample into ring buffer
        at8ErrBuf[at8EBufIdx] = atErr;
        at8EBufIdx = (at8EBufIdx + 1) % WOBBLE_WINDOW;
        if (at8EBufFill < WOBBLE_WINDOW) at8EBufFill++;

        // Compute RMS
        if (at8EBufFill > 0) {
          float sumSq = 0.0f;
          for (int wi = 0; wi < at8EBufFill; wi++) sumSq += at8ErrBuf[wi] * at8ErrBuf[wi];
          wobbleRMS = sqrtf(sumSq / (float)at8EBufFill);
        }

        // During SETTLE: apply micro-correction to prevent heavy wobble buildup
        if (at8TwEval == TWEval::SETTLE && wobbleRMS > WOBBLE_HEAVY_RMS) {
          bool tuningStraight = (at8Ph == AT8Phase::STR_TUNE);
          {
            static int _wCnt = 0;
            if (++_wCnt >= 20) {
              _wCnt = 0;
              webLog("[AT8-WOBBLE] RMS=%.0f>%.0f Kp micro-step %s=%.4f",
                     wobbleRMS, (float)WOBBLE_HEAVY_RMS,
                     tuningStraight ? "KpS" : "KpT",
                     tuningStraight ? at8KpS : at8KpT);
            }
          }
          if (tuningStraight) {
            at8KpS = max(at8KpS - WOBBLE_MICRO_STEP, TWIDDLE_KP_MIN);
          } else {
            at8KpT = max(at8KpT - WOBBLE_MICRO_STEP, TWIDDLE_KP_MIN);
          }
        }
      }

      // ── [M0-CORNER] Pre-corner braking + speed-cap for M0 AutoTune ────
      // Fires when tuning M0 (atTuneMode==0) AND current speed > M1 corner
      // speed AND either:  (a) outer LA sensors s1/s4 see junction 6 cm
      // ahead, or (b) terrain SM has committed to TURN_90.
      //
      // Phase 1  (BrakeTicks < ABS_LOCK_TICKS):
      //   Sets at8M0HardBraking=true → motor-output stage calls motorBrake().
      //   Zeroes lSpd/rSpd for telemetry and freezes soft-start anchors.
      //   Snaps at8RampedSpd down to gAtModeSpeed[1] immediately.
      //
      // Phase 2  (corner-follow):
      //   at8RampedSpd held at gAtModeSpeed[1].
      //   activeKp/Kd selection (below) uses saved wp[1] M1 values.
      //
      // Exit:  terrain SM confirms STRAIGHT for TERRAIN_STR_HOLD ticks.
      //   at8SpdTarget resumes at at8Spd → ramp restores M0 speed.
      //
      // IAE isolation: terrainMatch in Twiddle engine already gates
      //   accumulation to STRAIGHT ticks only. No extra suppression needed.
      at8M0HardBraking = false;  // re-evaluated every tick; set below if active
      if (atTuneMode == 0) {
        bool outerSensorsNear = gLookAheadHealthy && gObsEn && (s1 || s4);
        bool cornerConfirmed  = (at8Terrain == ATTerrain::TURN_90);
        int  m1CornerSpd      = gAtModeSpeed[1];

        // ── Trigger corner entry ──────────────────────────────────────
        if (!at8M0CornerActive
            && (outerSensorsNear || cornerConfirmed)
            && (at8RampedSpd > m1CornerSpd)) {
          at8M0CornerActive = true;
          at8M0BrakeTicks   = 0;
          at8M0CornerTicks  = 0;
          webLog("[M0-CORNER] TRIGGERED: rampedSpd=%d -> M1spd=%d  s1=%d s4=%d terrain=%s",
                 at8RampedSpd, m1CornerSpd,
                 (int)s1, (int)s4,
                 cornerConfirmed ? "TURN_90" : "OUTER_SENS");
        }

        if (at8M0CornerActive) {
          at8M0CornerTicks++;

          if (at8M0BrakeTicks < ABS_LOCK_TICKS) {
            // ── Phase 1: Hard brake ───────────────────────────────────
            // motorBrake() is invoked by the motor-output stage (below)
            // when at8M0HardBraking==true, ensuring TB6612FNG stays in
            // electronic-brake mode for the full output path.
            at8M0HardBraking = true;
            at8M0BrakeTicks++;
            lSpd = 0; rSpd = 0;          // telemetry accuracy
            at8LastL = 0; at8LastR = 0;  // freeze soft-start anchors at rest
            at8RampedSpd = m1CornerSpd;  // snap speed to M1 immediately
          } else {
            // ── Phase 2: Corner-follow — hold at M1 speed ─────────────
            if (at8RampedSpd > m1CornerSpd) at8RampedSpd = m1CornerSpd;
          }

          // ── Corner exit: terrain SM confirms STRAIGHT ─────────────
          if (at8Terrain == ATTerrain::STRAIGHT
              && at8StrCnt >= TERRAIN_STR_HOLD) {
            at8M0CornerActive = false;
            at8M0BrakeTicks   = 0;
            at8M0CornerTicks  = 0;
            webLog("[M0-CORNER] EXIT: STRAIGHT confirmed. Resuming M0 ramp %d -> %d  StrCnt=%d",
                   at8RampedSpd, at8Spd, at8StrCnt);
          }
        }
      }

      // ── Soft-start speed ramp ─────────────────────────────────────────
      // [M0-CORNER] Target is capped to gAtModeSpeed[1] while the corner
      // phase is active; resumes ramping toward at8Spd once STRAIGHT
      // terrain is re-confirmed and at8M0CornerActive clears.
      int at8SpdTarget = (atTuneMode == 0 && at8M0CornerActive)
                         ? gAtModeSpeed[1] : at8Spd;
      if (at8RampedSpd < at8SpdTarget) {
        at8RampedSpd += AT_BASE_RAMP_RATE;
        if (at8RampedSpd > at8SpdTarget) at8RampedSpd = at8SpdTarget;
      } else {
        at8RampedSpd = at8SpdTarget;
      }

      // ── Select active PID profile based on terrain + phase ────────────
      // Straight terrain always uses straight profile (if locked or tuning).
      // Turn terrain uses turn profile. UNKNOWN blends based on phase.
      // [FIX] Terrain-first adaptive selection: WHERE the robot is takes priority
      // over WHAT it is tuning — prevents turn-profile instability on straights.
       float activeKp, activeKd;
      if (at8Terrain == ATTerrain::STRAIGHT) {
        // MODE 0: use live Twiddle straight candidate (being optimised now).
        // MODE 1/2 ISOLATION: on straight sections, use the already-stabilised
        //   gPID[0] values (wp[0]).  at8KpS for M1/M2 is only an uninitialised
        //   seed — applying it here would destabilise straight-line driving.
        //   IAE gate (terrainMatch) already freezes cost accumulation; this
        //   ensures the MOTOR OUTPUT also uses a known-good profile.
        if (atTuneMode == 0) {
          activeKp = at8KpS;
          activeKd = at8KdS;
        } else {
          activeKp = wp[0].kP;  // frozen, stabilised M0 straight profile
          activeKd = wp[0].kD;
        }
      } else if (at8Terrain == ATTerrain::TURN_90) {
        // [M0-CORNER] During M0 tuning, at8KpT/at8KdT are uninitialised
        // Twiddle seeds (0.020 / 0.400) — completely wrong for cornering.
        // Fall back to saved gPID[1] values (wp[1]) which the user has
        // configured or previously tuned via M1 AT.
        // For M1/M2 tuning the Twiddle turn candidate is correct.
        if (atTuneMode == 0) {
          activeKp = wp[1].kP;  // saved gPID[1].kP — real M1 value
          activeKd = wp[1].kD;  // saved gPID[1].kD
        } else {
          activeKp = at8KpT;
          activeKd = at8KdT;   // Twiddle turn candidate for M1/M2 tuning
        }
      } else {
        // UNKNOWN terrain (transitional) — use the profile matching the
        // current tune target.  COMPLETE+M0 must use at8KpS (tuned straight),
        // NOT at8KpT (the uninitialised turn seed, which is 0.020f).
        bool tuningStr = (at8Ph == AT8Phase::STR_TUNE || at8Ph == AT8Phase::STR_LOCKED)
                         || (at8Ph == AT8Phase::COMPLETE && atTuneMode == 0);
        activeKp = tuningStr ? at8KpS : at8KpT;
        activeKd = tuningStr ? at8KdS : at8KdT;
      }

      // ── PD computation for AT8 drive (no I-term: cleaner IAE eval) ────
 // ── PD computation for AT8 drive (no I-term: cleaner IAE eval) ────
      {
        int rawL, rawR;
        
        if (!lineVisible) {
          // --- LINE LOST RECOVERY SPIN ---
          int spinSpd = wp[2].low + 20; // Safe spinning speed
          if (lastKnownErrSign > 0) {
            rawL = spinSpd;
            rawR = -spinSpd;
          } else {
            rawL = -spinSpd;
            rawR = spinSpd;
          }
          at8PrevErr = 0.0f; // Reset PD memory
          at8DFilt = 0.0f;
        } else {
          // --- NORMAL LINE FOLLOWING ---
          float rawD = atErr - at8PrevErr;
          // raw derivative (per tick)
          at8DFilt = D_EMA_ALPHA * rawD + (1.0f - D_EMA_ALPHA) * at8DFilt;
          at8PrevErr = atErr;
          float atOut = activeKp * atErr + activeKd * at8DFilt;
          float clamp = (float)at8RampedSpd * AT8_OUTPUT_CLAMP_FACTOR;
          atOut = constrain(atOut, -clamp, clamp);
          rawL = (int)((float)at8RampedSpd + atOut);
          rawR = (int)((float)at8RampedSpd - atOut);
        }
        // Violent oscillation check
        int pwmDiff = abs(rawL - rawR);
        if (pwmDiff > AT_VIOLENT_OSC_THR) {
          at8ViolTix++;
          if (at8ViolTix >= AT_SAFETY_STALL_TICKS) {
            stopAll();
            setState(State::IDLE);
            webLog("AT8: !! ABORT !! Violent osc |L-R|=%d. Saving checkpoint.", pwmDiff);
            at8NvsSave(AT8Phase::ABORTED, at8Spd, at8KpS, at8KdS,
                       at8KpT, at8KdT, at8SLocked, at8SpdCnf, atTuneMode);
            webLog("[AT8-VIOL] Violent osc ABORT. mode=M%d spd=%d KpS=%.4f KpT=%.4f",
                   atTuneMode, at8Spd, at8KpS, at8KpT);
            portENTER_CRITICAL(&gAtResMux);
            gAtResult.phase = 3;
            portEXIT_CRITICAL(&gAtResMux);
            if (!calibOwnsQtr) qtrStartCharge();
            continue;
          }
        } else {
          at8ViolTix = 0;
        }

        // Soft-start ramp (AT8 uses same ramp as legacy, prevents jerk at 11.4V)
       rawL = constrain(rawL, at8LastL - AT_SOFT_START_RAMP, at8LastL + AT_SOFT_START_RAMP);
        rawR = constrain(rawR, at8LastR - AT_SOFT_START_RAMP, at8LastR + AT_SOFT_START_RAMP);
        // [M0-CORNER] During hard-brake phase: freeze soft-start anchors at 0
        // so the post-brake ramp starts cleanly from rest (not from PD output).
        // D-term (at8DFilt, at8PrevErr) has already been updated above, so
        // derivative continuity is maintained for the moment braking ends.
        if (!at8M0HardBraking) {
          at8LastL = rawL;
          at8LastR = rawR;
          lSpd = constrain(rawL, -255, 255);
          rSpd = constrain(rawR, -255, 255);
        }
        // When at8M0HardBraking==true: lSpd/rSpd remain 0 (set by M0-CORNER
        // block above); motor-output stage will call motorBrake() directly.
      }

      // ── [AT8-4] TWIDDLE / COORDINATE DESCENT ENGINE ─────────────────
      // Determines which PID profile is being evaluated based on current
      // AT8 phase. Terrain-gate ensures IAE only accumulates during the
      // relevant section of track (STRAIGHT or TURN_90).

      bool isTuningStr = (at8Ph == AT8Phase::STR_TUNE || at8Ph == AT8Phase::STR_LOCKED);
      bool isTuningTrn = (at8Ph == AT8Phase::TRN_TUNE);
      bool terrainMatch = isTuningStr ? (at8Terrain == ATTerrain::STRAIGHT)
                                      : (at8Terrain == ATTerrain::TURN_90);

      if (isTuningStr || isTuningTrn) {
        at8TwTick++;                        // wall-clock ticks (for settle phase)
        if (terrainMatch) at8TwTerrTick++;  // terrain-filtered ticks (for measure)

        // ── SETTLE sub-phase (robot stabilises; wobble mitigation active) ─
        if (at8TwEval == TWEval::SETTLE) {
          if (at8TwTick >= TWIDDLE_SETTLE_TICKS) {
            // Settle done → switch to MEASURE, reset IAE accumulator
            at8TwEval = TWEval::MEASURE;
            at8TwTick = 0;
            at8TwTerrTick = 0;
            at8TwCurFit = 0.0f;
          }
        }
        // ── MEASURE sub-phase (accumulate IAE only on matching terrain) ──
        else {  // TWEval::MEASURE
          if (terrainMatch) {
            at8TwCurFit += absAtErr;  // IAE: ∫|e|dt (dt=5ms, constant)
          }

         if (at8TwTerrTick >= TWIDDLE_EVAL_TICKS) {
            // Twiddle optimization DISABLED for continuous run
            webLog("[AT8-FIXED] Eval done. IAE=%.0f  Kp=%.4f  Kd=%.4f  spd=%d  (fixed-values mode)",
                   at8TwCurFit,
                   isTuningStr ? at8KpS : at8KpT,
                   isTuningStr ? at8KdS : at8KdT,
                   at8Spd);

            if (false) {  // Original Twiddle block disabled
              float fitness = at8TwCurFit; // lower = better line-following

              // ── Determine active param pointer for convenience ───────────

            // ── Determine active param pointer for convenience ───────────
            float* pKp = isTuningStr ? &at8KpS : &at8KpT;
            float* pKd = isTuningStr ? &at8KdS : &at8KdT;
            float* pDp = (at8TwPar == TWParam::KP) ? &at8TwDpKp : &at8TwDpKd;
            float* pPar = (at8TwPar == TWParam::KP) ? pKp : pKd;
            float pMin = (at8TwPar == TWParam::KP) ? TWIDDLE_KP_MIN : TWIDDLE_KD_MIN;
            float pMax = (at8TwPar == TWParam::KP) ? TWIDDLE_KP_MAX : TWIDDLE_KD_MAX;

            // ── [AT8-4] BIDIRECTIONAL TWIDDLE DECISION ──────────────────
            if (at8TwBaseline) {
              // First run ever — set baseline fitness, then try +dp on KP
              at8TwBestFit = fitness;
              at8TwBaseline = false;
              *pPar = constrain(*pPar + *pDp, pMin, pMax);
              at8TwDir = TWDir::TRY_PLUS;
              webLog("AT8-Twiddle: BASELINE IAE=%.0f  → trying %s+dp=%.4f",
                     fitness, (at8TwPar == TWParam::KP) ? "Kp" : "Kd", *pPar);
            } else if (at8TwDir == TWDir::TRY_PLUS) {
              if (fitness < at8TwBestFit) {
                // IMPROVEMENT with +dp → accept, grow dp, next param
                at8TwBestFit = fitness;
                *pDp *= TWIDDLE_DP_GROW;
                webLog("AT8-Twiddle: +dp IMPROVED IAE=%.0f best=%.0f. dp*=%.3f",
                       fitness, at8TwBestFit, *pDp);
                at8TwDir = TWDir::IDLE;  // will advance param below
              } else {
                // No improvement — try -dp instead (bidirectional)
                *pPar = constrain(*pPar - 2.0f * (*pDp), pMin, pMax);
                at8TwDir = TWDir::TRY_MINUS;
                webLog("AT8-Twiddle: +dp no gain. Trying %s-dp=%.4f",
                       (at8TwPar == TWParam::KP) ? "Kp" : "Kd", *pPar);
              }
            } else if (at8TwDir == TWDir::TRY_MINUS) {
              if (fitness < at8TwBestFit) {
                // IMPROVEMENT with -dp → accept, grow dp, next param
                at8TwBestFit = fitness;
                *pDp *= TWIDDLE_DP_GROW;
                webLog("AT8-Twiddle: -dp IMPROVED IAE=%.0f best=%.0f. dp*=%.3f",
                       fitness, at8TwBestFit, *pDp);
              } else {
                // Neither +dp nor -dp helped → restore, shrink dp
                *pPar = constrain(*pPar + (*pDp), pMin, pMax);  // restore
                *pDp *= TWIDDLE_DP_SHRINK;
                webLog("AT8-Twiddle: Neither dir helped. dp shrunk=%.4f %s=%.4f",
                       *pDp, (at8TwPar == TWParam::KP) ? "Kp" : "Kd", *pPar);
              }
              at8TwDir = TWDir::IDLE;  // advance param
            }

            // ── Advance to next parameter ─────────────────────────────────
            if (at8TwDir == TWDir::IDLE && !at8TwBaseline) {
              // Cycle: KP → KD → KP → ...
              at8TwPar = (at8TwPar == TWParam::KP) ? TWParam::KD : TWParam::KP;
              float* nPar = (at8TwPar == TWParam::KP) ? pKp : pKd;
              float* nDp = (at8TwPar == TWParam::KP) ? &at8TwDpKp : &at8TwDpKd;
              float nMin = (at8TwPar == TWParam::KP) ? TWIDDLE_KP_MIN : TWIDDLE_KD_MIN;
              float nMax = (at8TwPar == TWParam::KP) ? TWIDDLE_KP_MAX : TWIDDLE_KD_MAX;
              // Try +dp on new param
              *nPar = constrain(*nPar + *nDp, nMin, nMax);
              at8TwDir = TWDir::TRY_PLUS;
              webLog("AT8-Twiddle: Next param=%s new val=%.4f dpSum=%.4f",
                     (at8TwPar == TWParam::KP) ? "Kp" : "Kd", *nPar,
                     at8TwDpKp + at8TwDpKd);

              // ── Check convergence (sum of deltas) or max-rounds safety ─
              at8TwRounds++;
              float dpSum = at8TwDpKp + at8TwDpKd;
              bool converged = (dpSum < TWIDDLE_DONE_SUM);
              if (!converged && at8TwRounds >= AT8_TWIDDLE_MAX_ROUNDS) {
                // dp keeps growing (robot keeps improving) — force speed step
                at8SpdCnf = AT8_SPD_CONFIRM;
                at8TwRounds = 0;
                webLog("AT8-Twiddle: MAX_ROUNDS(%d) dpSum=%.4f -> forced step Kp=%.4f Kd=%.4f",
                       AT8_TWIDDLE_MAX_ROUNDS, dpSum,
                       isTuningStr ? at8KpS : at8KpT,
                       isTuningStr ? at8KdS : at8KdT);
              }
              if (converged || at8SpdCnf >= AT8_SPD_CONFIRM) {
                // Twiddle converged (or forced) for this speed level
                if (converged) {
                  at8SpdCnf++;
                  at8TwRounds = 0;
                }
                if (converged) webLog("AT8-Twiddle: CONVERGED (dpSum=%.5f cnt=%d/%d) Kp=%.4f Kd=%.4f",
                                      dpSum, at8SpdCnf, AT8_SPD_CONFIRM,
                                      isTuningStr ? at8KpS : at8KpT,
                                      isTuningStr ? at8KdS : at8KdT);

                if (at8Ph == AT8Phase::STR_TUNE) {
                  // ── [AT8-3] Check straight-profile lock ───────────────
                  // Lock if error variance < threshold for N windows
                  if (at8WfN >= STRAIGHT_LOCK_WINDOW) {
                    float variance = (at8WfN > 1) ? at8WfM2 / (float)(at8WfN - 1) : 1e9f;
                    if (variance < STRAIGHT_LOCK_VAR_THR) {
                      at8LockHits++;
                      webLog("AT8: Straight variance=%.0f < %.0f [%d/%d]",
                             variance, STRAIGHT_LOCK_VAR_THR, at8LockHits, STRAIGHT_LOCK_HITS);
                    } else {
                      at8LockHits = 0;
                    }
                    // Reset Welford window for next check
                    at8WfN = 0;
                    at8WfMean = 0.0f;
                    at8WfM2 = 0.0f;
                  }
                  if (at8LockHits >= STRAIGHT_LOCK_HITS) {
                    at8SLocked = true;
                    webLog("AT8: *** STRAIGHT PROFILE LOCKED *** Kp=%.4f Kd=%.4f",
                           at8KpS, at8KdS);
                    // Apply locked straight values to gPID M0
                    portENTER_CRITICAL(&gPidMux);
                    gPID[0].kP = at8KpS;
                    gPID[0].kD = at8KdS;
                    // [FIX-BASE] Sync tuned speed so RUNNING mode uses the same PWM as AT8.
                    gPID[0].base = at8Spd;
                    if (gPID[0].top < at8Spd + 10) gPID[0].top = at8Spd + 30;
                    portEXIT_CRITICAL(&gPidMux);
                    gPrefs.putFloat("m0kp", at8KpS);
                    gPrefs.putFloat("m0kd", at8KdS);
                    gPrefs.putInt("m0bs", at8Spd);  // [FIX-BASE] persist tuned speed to NVS
                    at8Ph = AT8Phase::STR_LOCKED;
                  }
                }

                if (at8SpdCnf >= AT8_SPD_CONFIRM || at8Ph == AT8Phase::STR_LOCKED) {
                  // ── [BUG4-FIX] UNIFIED SPEED ESCALATION — runs ONCE for both
                  //   STR_TUNE and TRN_TUNE.  The duplicate TRN_TUNE block below
                  //   has been removed; this is now the sole escalation site.
                  // [BUGFIX-1] Revert the probe applied in "advance param" BEFORE
                  // dp is reset.  At this point *nDp still holds the old converged
                  // delta.  Without this revert the new speed level's Twiddle
                  // baseline starts from best_param+old_dp (slightly off), so
                  // every speed step skews the starting point further from optimum.
                  *nPar = constrain(*nPar - *nDp, nMin, nMax);
                  int oldSpd = at8Spd;
                  at8Spd += AT8_SPD_STEP;  // escalated exactly once
                  at8SpdCnf = 0;
                  at8TwDpKp = gAtKpStep[constrain(atTuneMode, 0, 2)];  // [FIX-PERMODE]
                  at8TwDpKd = TWIDDLE_KD_INIT_STEP;
                  webLog("[AT8-SPD-STEP] dp reset: dpKp=%.4f dpKd=%.4f mode=M%d spd=%d→%d",
                         at8TwDpKp, at8TwDpKd, atTuneMode, oldSpd, at8Spd);
                  at8TwBestFit = 1.0e9f;
                  at8TwBaseline = true;
                  at8TwPar = TWParam::KP;
                  at8TwDir = TWDir::IDLE;
                  at8TwRounds = 0;
                  // [BUGFIX-2a] Reset Welford variance accumulators on every speed
                  // step.  Cross-speed data mixed into the window produces an
                  // artificially low variance → premature / incorrect straight lock.
                  at8WfN = 0;
                  at8WfMean = 0.0f;
                  at8WfM2 = 0.0f;
                  at8LockHits = 0;
                  // [BUGFIX-2b] Clear wobble ring-buffer on speed step so stale
                  // error samples from the previous (lower) speed do not falsely
                  // trigger the Kp micro-reduction during the new SETTLE phase.
                  memset(at8ErrBuf, 0, sizeof(at8ErrBuf));
                  at8EBufIdx = 0;
                  at8EBufFill = 0;

                  // ── [BUG4-FIX] Apply optimised values to gPID + NVS when
                  //   TRN_TUNE has converged (previously only done in the
                  //   now-deleted duplicate block, causing the second increment).
                  if (isTuningTrn) {
                    // [FIX] Compute intelligent top/low from tuned speed
                    int _top = oldSpd + constrain(oldSpd / 4, 20, 50);
                    int _low = constrain(oldSpd * 2 / 3, 15, oldSpd - 10);
                    portENTER_CRITICAL(&gPidMux);
                    if (atTuneMode == 1) {
                      // M1 mode: apply tuned turn values to gPID[1] only
                      gPID[1].kP = at8KpT;  gPID[1].kD = at8KdT;
                      gPID[1].base = oldSpd; gPID[1].top = _top; gPID[1].low = _low;  // [FIX] auto-adjust
                    } else if (atTuneMode == 2) {
                      // M2 mode: apply tuned values to gPID[2] only
                      gPID[2].kP = at8KpT;  gPID[2].kD = at8KdT;
                      gPID[2].base = oldSpd; gPID[2].top = _top; gPID[2].low = _low;  // [FIX] auto-adjust
                         } else {
                      // MODE 0 ISOLATION: write ONLY gPID[0] (straight profile).
                      // Never cross-write M1/M2 from a Mode 0 AT run.
                      // (This block is a defensive backstop; Mode 0 should reach
                      //  COMPLETE before TRN_TUNE via PATCH 1.)
                      gPID[0].kP   = at8KpS;
                      gPID[0].kD   = at8KdS;
                      gPID[0].base = oldSpd;
                      gPID[0].top  = _top;
                      gPID[0].low  = _low;
                    }
                    portEXIT_CRITICAL(&gPidMux);
                    // NVS persistence — only write the targeted profile(s), include top/low
                    if (atTuneMode == 1) {
                      gPrefs.putFloat("m1kp", at8KpT);
                      gPrefs.putFloat("m1kd", at8KdT);
                      gPrefs.putInt("m1bs", gPID[1].base);
                      gPrefs.putInt("m1tp", gPID[1].top);  // [FIX] persist auto-adjusted top
                      gPrefs.putInt("m1lw", gPID[1].low);  // [FIX] persist auto-adjusted low
                      webLog("AT8: [M1] TURN PROFILE APPLIED Kp=%.4f Kd=%.4f base=%d top=%d low=%d",
                             at8KpT, at8KdT, gPID[1].base, gPID[1].top, gPID[1].low);
                    } else if (atTuneMode == 2) {
                      gPrefs.putFloat("m2kp", at8KpT);
                      gPrefs.putFloat("m2kd", at8KdT);
                      gPrefs.putInt("m2bs", gPID[2].base);
                      gPrefs.putInt("m2tp", gPID[2].top);  // [FIX] persist auto-adjusted top
                      gPrefs.putInt("m2lw", gPID[2].low);  // [FIX] persist auto-adjusted low
                      webLog("AT8: [M2] FAST PROFILE APPLIED Kp=%.4f Kd=%.4f base=%d top=%d low=%d",
                             at8KpT, at8KdT, gPID[2].base, gPID[2].top, gPID[2].low);
                     } else {
                      // MODE 0 ISOLATION: persist ONLY the straight profile to NVS.
                      // Never overwrite m1kp/m2kp/m1kd/m2kd from a Mode 0 run.
                      gPrefs.putFloat("m0kp", at8KpS);
                      gPrefs.putFloat("m0kd", at8KdS);
                      gPrefs.putInt("m0bs", gPID[0].base);
                      gPrefs.putInt("m0tp", gPID[0].top);
                      gPrefs.putInt("m0lw", gPID[0].low);
                      webLog("AT8: [M0] STRAIGHT-ONLY PROFILE APPLIED Kp=%.4f Kd=%.4f base=%d top=%d low=%d",
                             at8KpS, at8KdS, gPID[0].base, gPID[0].top, gPID[0].low);
                    }
                    // Broadcast confirmed result to Web UI
                    portENTER_CRITICAL(&gAtResMux);
                    gAtResult.resultKp = at8KpT;
                    gAtResult.resultKd = at8KdT;
                    gAtResult.currentKp = at8KpS;
                    gAtResult.zeroCrossings = 8;  // sentinel: twiddle done
                    gAtResult.phase = 2;
                    gAtResult.tuneMode = atTuneMode;  // actual selected mode
                    portEXIT_CRITICAL(&gAtResMux);
                    portENTER_CRITICAL(&gAtBcastMux);
                    gAtSuccessBroadcastPending = true;
                    portEXIT_CRITICAL(&gAtBcastMux);
                  } else if (isTuningStr) {
                    // [FIX-STR] Apply STR_TUNE best values to gPID M0 at every speed step.
                    // Previously only the straight-lock Welford path updated gPID, meaning
                    // AT8 could run many speed levels without ever writing to gPID — RUNNING
                    // mode always saw the original UI values regardless of tuning progress.
                    portENTER_CRITICAL(&gPidMux);
                    gPID[0].kP = at8KpS;
                    gPID[0].kD = at8KdS;
                    gPID[0].base = oldSpd;
                    if (gPID[0].top < oldSpd + 10) gPID[0].top = oldSpd + 30;
                    portEXIT_CRITICAL(&gPidMux);
                    gPrefs.putFloat("m0kp", at8KpS);
                    gPrefs.putFloat("m0kd", at8KdS);
                    gPrefs.putInt("m0bs", oldSpd);
                    webLog("AT8: STR profile checkpoint @ spd=%d  Kp=%.4f Kd=%.4f",
                           oldSpd, at8KpS, at8KdS);
                    // Push updated M0 values to Web UI immediately
                    portENTER_CRITICAL(&gPidFetchMux);
                    gPidFetchPending = true;
                    portEXIT_CRITICAL(&gPidFetchMux);
                  }

                  webLog("AT8: [SPEED STEP] %d → %d PWM. Retuning...", oldSpd, at8Spd);
                  // [AT8-1] Save checkpoint (SPD_STEP phase during the brief transition)
                  at8NvsSave(AT8Phase::SPD_STEP, at8Spd, at8KpS, at8KdS,
                             at8KpT, at8KdT, at8SLocked, at8SpdCnf, atTuneMode);
                  webLog("[AT8-SPD-NVS] Speed step saved. mode=M%d spd=%d KpS=%.4f KpT=%.4f",
                         atTuneMode, at8Spd, at8KpS, at8KpT);
                   // ── Decide next tuning phase (MODE-ISOLATED) ──────────────
                  if (at8SLocked && atTuneMode == 0) {
                    // MODE 0 STRICT ISOLATION: straight locked → COMPLETE.
                    // Mode 0 tunes ONLY gPID[0]. It must NEVER enter TRN_TUNE.
                    // Roll back the speed increment — this is a completion, not an escalation.
                    at8Spd = oldSpd;
                    at8Ph = AT8Phase::COMPLETE;
                    // Overwrite the SPD_STEP NVS save with the final COMPLETE state.
                    at8NvsSave(AT8Phase::COMPLETE, oldSpd, at8KpS, at8KdS,
                               at8KpT, at8KdT, at8SLocked, at8SpdCnf, atTuneMode);
                    webLog("AT8:[M0-COMPLETE] STRAIGHT ONLY DONE spd=%d Kp=%.4f Kd=%.4f → gPID[0].",
                           oldSpd, at8KpS, at8KdS);
                    // Fire AT8 success broadcast so Web UI modal shows M0 result.
                    portENTER_CRITICAL(&gAtResMux);
                    gAtResult.resultKp   = at8KpS;
                    gAtResult.resultKd   = at8KdS;
                    gAtResult.currentKp  = at8KpS;
                    gAtResult.zeroCrossings = 8;  // sentinel: twiddle done
                    gAtResult.phase      = 2;
                    gAtResult.tuneMode   = 0;
                    portEXIT_CRITICAL(&gAtResMux);
                    portENTER_CRITICAL(&gAtBcastMux);
                    gAtSuccessBroadcastPending = true;
                    portEXIT_CRITICAL(&gAtBcastMux);
                  } else if (at8SLocked) {
                    // M1 / M2: straight locked → now tune the turn profile.
                    at8Ph = AT8Phase::TRN_TUNE;
                    webLog("AT8: Straight locked. Tuning TURN profile at spd=%d.", at8Spd);
                  } else {
                    at8Ph = AT8Phase::STR_TUNE;
                    webLog("AT8: Retuning STRAIGHT at new spd=%d.", at8Spd);
                  }
                  portENTER_CRITICAL(&gAtMux);
                  gAtBaseSpeed = at8Spd;
                  portEXIT_CRITICAL(&gAtMux);
                }
            }  // end convergence check
            }    // end param advance

            } // end of if (false) block - Twiddle optimization ends here

            // ── Restart settle/measure for next candidate ──────────────
            // This ALWAYS executes so the robot keeps running cleanly
            at8TwEval = TWEval::SETTLE;
            at8TwTick = 0;
            at8TwTerrTick = 0;
            at8TwCurFit = 0.0f;
          }  // end eval window complete
        }    // end MEASURE branch

        // ── [AT8-3] Welford online variance accumulation ───────────────
        // Runs in parallel with Twiddle. Used to decide straight lock.
        if (at8Ph == AT8Phase::STR_TUNE && at8Terrain == ATTerrain::STRAIGHT) {
          at8WfN++;
          float delta = atErr - at8WfMean;
          at8WfMean += delta / (float)at8WfN;
          float delta2 = atErr - at8WfMean;
          at8WfM2 += delta * delta2;
        }

      }  // end isTuningStr || isTuningTrn

      // [BUG4-FIX] Duplicate TRN_TUNE convergence + speed escalation block
      // has been removed. The unified escalation path inside the main Twiddle
      // convergence block above now handles both STR_TUNE and TRN_TUNE,
      // applies gPID values, writes NVS, and broadcasts — exactly once.

      // ── Publish AT8 status for telemetry ─────────────────────────────────
      portENTER_CRITICAL(&gAt8Mux);
      gAt8Status.kpS = at8KpS;
      gAt8Status.kdS = at8KdS;
      gAt8Status.kpT = at8KpT;
      gAt8Status.kdT = at8KdT;
      gAt8Status.activeKp = activeKp;  // terrain-selected Kp driving motors now
      gAt8Status.activeKd = activeKd;  // terrain-selected Kd driving motors now
      gAt8Status.sLocked = at8SLocked;
      gAt8Status.speed = at8Spd;
      gAt8Status.fitness = at8TwBestFit;
      gAt8Status.curFit = at8TwCurFit;  // live IAE accumulation counter
      gAt8Status.phase = (uint8_t)at8Ph;
      gAt8Status.terrain = (uint8_t)at8Terrain;
      gAt8Status.dpSum = at8TwDpKp + at8TwDpKd;
      gAt8Status.tuneMode = atTuneMode;  // M0/M1/M2 being tuned
      portEXIT_CRITICAL(&gAt8Mux);
// ── [FIX] Realtime write Twiddle candidates to gPID + Control tab (~2 Hz) ──
      // Writes live Kp/Kd candidates to gPID so PID-Tuner tab reflects AT8 in realtime.
      // Auto-adjusts top/low proportionally to the current tuning speed.
      static int at8PidRefreshCnt = 0;
      if (++at8PidRefreshCnt >= 100) {
        at8PidRefreshCnt = 0;
        int _spd = at8Spd;
        int _top = _spd + constrain(_spd / 4, 20, 50);
        int _low = constrain(_spd * 2 / 3, 15, _spd - 10);
        portENTER_CRITICAL(&gPidMux);
        if (atTuneMode == 1) {
          gPID[1].kP = at8KpT;  gPID[1].kD = at8KdT;
          gPID[1].base = _spd;  gPID[1].top = _top;  gPID[1].low = _low;
        } else if (atTuneMode == 2) {
          gPID[2].kP = at8KpT;  gPID[2].kD = at8KdT;
          gPID[2].base = _spd;  gPID[2].top = _top;  gPID[2].low = _low;
        } else {
           // MODE 0 ISOLATION: write ONLY gPID[0] during live refresh.
          // Mode 0 tunes only the straight profile — never touch gPID[1] here.
          gPID[0].kP = at8KpS;  gPID[0].kD = at8KdS;
          gPID[0].base = _spd;  gPID[0].top = _top;  gPID[0].low = _low;
        }
        portEXIT_CRITICAL(&gPidMux);
        portENTER_CRITICAL(&gPidFetchMux);
        gPidFetchPending = true;
        portEXIT_CRITICAL(&gPidFetchMux);
      }

      // Update legacy AT result for web UI phase indicator
// [FIX] Update legacy AT result — terrain-active Kp (not straight-only at8KpS)
  portENTER_CRITICAL(&gAtResMux);
  gAtResult.currentKp = activeKp;
      gAtResult.phase = (at8Ph == AT8Phase::ABORTED) ? 3 : (at8Ph == AT8Phase::COMPLETE) ? 2
                                                                                         : 1;
      portEXIT_CRITICAL(&gAtResMux);
      portENTER_CRITICAL(&gAtMux);
      gAtRampedBasePub = at8RampedSpd;
      portEXIT_CRITICAL(&gAtMux);

    }  // end isAutoTuning [AT8]

    // ──────────────────────────────────────────────────────────────────
    //  PID COMPUTATION  (with [U4] geometry-based base speed override
    //  and [U10] velocity smoothing)
    // ──────────────────────────────────────────────────────────────────
    if (!skipPID) {
      float err = (float)pos - QTR_SETPOINT;
      float absE = fabsf(err);

      if (absE > HYBRID_MODE2_ERR) pm = max(pm, 2);
      else if (absE > HYBRID_MODE1_ERR) pm = max(pm, 1);
      PIDParam* p = &wp[pm];
      if (pm != prevPm) {
        webLog("[PID-MODE] M%d→M%d err=%.0f Kp=%.4f Kd=%.4f base=%d top=%d",
               prevPm, pm, err, p->kP, p->kD, p->base, p->top);
        pidI = 0.0f;
        prevPm = pm;
      }

      effKp += (p->kP - effKp) * SMOOTH_ALPHA;
      effKd += (p->kD - effKd) * SMOOTH_ALPHA;

      // [FIX] Unified base-speed path: LA braking and normal recovery share a single
      // ramp. The old code smoothed effBase UP toward p->base then immediately velRamped
      // it DOWN to laTargetBase every 5 ms tick — a mathematical tug-of-war that
      // negated braking distance and caused PWM oscillations.
      float targetBase = (laTargetBase >= 0) ? (float)laTargetBase : (float)p->base;
      static float _prevEfB = -1.0f;
      if (targetBase < effBase) {
        effBase = (float)velRamp((int)effBase, (int)targetBase, VEL_ACCEL_RATE, VEL_DECEL_RATE);
      } else {
        effBase += (targetBase - effBase) * SMOOTH_ALPHA;
      }
      if (fabsf(effBase - _prevEfB) > 10.0f) {
        webLog("[SPEED] effBase %.0f→%.0f | target=%.0f laTgt=%d pm=%d",
               _prevEfB, effBase, targetBase, laTargetBase, pm);
        _prevEfB = effBase;
      }

      int smoothedBase = (int)roundf(effBase);

      // Reset integral on zero crossing
      if ((err > 0.0f) != (pidPrev > 0.0f)) pidI = 0.0f;
      pidI += err * dt;
      pidI = constrain(pidI, -400.0f, 400.0f);

      // [U5.2] D-term EMA
      float rawDer = (dt > 0.0f) ? ((err - pidPrev) / dt) : 0.0f;
      pidDFiltered = D_EMA_ALPHA * rawDer + (1.0f - D_EMA_ALPHA) * pidDFiltered;
      gLastDError = pidDFiltered;  // [U9] expose for math braking
      pidPrev = err;
      lastAbsErr = absE;

      float pidO = effKp * err + p->kI * pidI + effKd * pidDFiltered;

      int base;
      if (pm == 0) {
        float t = constrain(absE / 2000.0f, 0.0f, 1.0f);
        base = (int)((float)p->top - t * (float)(p->top - smoothedBase));
      } else {
        base = smoothedBase;
      }
      // [FIX] When t≈0 (robot centered on line approaching a corner), the Mode 0
      // formula evaluates to p->top regardless of LA-braked smoothedBase, silently
      // overriding all braking. Clamp base to the already-braked effBase.
      if (laTargetBase >= 0) {
        base = min(base, smoothedBase);
      }

      int targetL = (int)constrain((float)base + pidO, -(float)p->top, (float)p->top);
      int targetR = (int)constrain((float)base - pidO, -(float)p->top, (float)p->top);

      if (absE < ANTISTALL_ERR_THR) {
        if (targetL > 0 && targetL < p->low) targetL = p->low;
        if (targetR > 0 && targetR < p->low) targetR = p->low;
      }

      // [U10] Apply velocity ramp to smooth transitions
      lSpd = velRamp(rampedLSpd, targetL, VEL_ACCEL_RATE, VEL_DECEL_RATE);
      rSpd = velRamp(rampedRSpd, targetR, VEL_ACCEL_RATE, VEL_DECEL_RATE);

      lastLSpd = lSpd;
      lastRSpd = rSpd;
    }

    // [U10] Update ramp trackers
    rampedLSpd = lSpd;
    rampedRSpd = rSpd;

    // ── Motor deadband ────────────────────────────────────────────────
    auto applyDeadband = [](int spd) -> int {
      if (spd == 0) return 0;
      return (spd > 0) ? max(spd, MOTOR_DEADBAND) : min(spd, -MOTOR_DEADBAND);
    };
    lSpd = applyDeadband(lSpd);
    rSpd = applyDeadband(rSpd);

 // ── [ABS] State machine — overrides motor output if active ────────
    // Guard: ABS runs only in RUNNING mode. During AutoTune, AT8 block
    // already set lSpd/rSpd; falling through to else drives motors correctly.
    //
    // [M0-CORNER] AT8 pre-corner hard-brake path: motorBrake() MUST be
    // called here, not inside the AT8 block, because any subsequent
    // driveL/driveR call would exit the TB6612FNG short-circuit mode.
    // at8M0HardBraking is false in all non-AT8 ticks (cleared each tick
    // at the top of the M0-CORNER block, only set inside isAutoTuning).
    if (isAutoTuning && at8M0HardBraking) {
      motorBrake();  // TB6612FNG: IN1=IN2=HIGH, PWM=255 — electronic shaft lock
      // lSpd/rSpd already 0 (set in AT8 block) for telemetry accuracy
    } else if (!isAutoTuning && absActive) {
      absPhaseTicks++;

      if (absPhase == 0) {
        // ── LOCK phase: apply active brake ────────────────────────────
        motorBrake();
        lSpd = 0; rSpd = 0;  // for telemetry accuracy
        {
          static int _absLockCnt = 0;
          if (++_absLockCnt >= 5) {
            _absLockCnt = 0;
            webLog("[ABS-LOCK] cycle=%d/%d tick=%d/%d effBase=%.0f",
                   absCycleCount + 1, absCycleMax, absPhaseTicks, ABS_LOCK_TICKS, effBase);
          }
        }
        if (absPhaseTicks >= ABS_LOCK_TICKS) {
          absPhase = 1;
          absPhaseTicks = 0;
        }
      } else {
        // ── RELEASE phase: drive forward at slow speed ─────────────────
        driveL(ABS_RELEASE_PWM);
        driveR(ABS_RELEASE_PWM);
        lSpd = ABS_RELEASE_PWM; rSpd = ABS_RELEASE_PWM;  // telemetry
        {
          static int _absRelCnt = 0;
          if (++_absRelCnt >= 5) {
            _absRelCnt = 0;
            webLog("[ABS-REL] cycle=%d/%d tick=%d/%d",
                   absCycleCount + 1, absCycleMax, absPhaseTicks, ABS_RELEASE_TICKS);
          }
        }
        if (absPhaseTicks >= ABS_RELEASE_TICKS) {
          absCycleCount++;
          if (absCycleCount >= absCycleMax) {
            // ── ABS complete — restore speed state ──────────────────
            absActive     = false;
            absDone       = true;
            effBase       = (float)wp[1].base;
            rampedLSpd    = wp[1].base;
            rampedRSpd    = wp[1].base;
            at8LastL      = ABS_RELEASE_PWM;  // prevent AT8 soft-start jerk
            at8LastR      = ABS_RELEASE_PWM;
            webLog("[ABS-DONE] %d cycle(s) complete. Speed reset to M1base=%d. src=%s",
                   absCycleCount, wp[1].base,
                   absFrom6cm ? "6CM" : absFrom2cm ? "2CM" : "MANUAL");
          } else {
            // ── Next ABS cycle ───────────────────────────────────────
            absPhase = 0;
            absPhaseTicks = 0;
            webLog("[ABS-NEXT] Cycle %d/%d → LOCK phase.", absCycleCount + 1, absCycleMax);
          }
        }
      }
    } else {
      // Normal motor output
      driveL(lSpd);
      driveR(rSpd);
    }

    {
      static int _mCnt = 0;
      if (!isAutoTuning && ++_mCnt >= 200) {
        _mCnt = 0;
        webLog("[MOTOR] L=%d R=%d err=%d pm=%d effBase=%.0f laTgt=%d nav=%d abs=%d",
               lSpd, rSpd, (int)((float)pos - QTR_SETPOINT),
               pm, effBase, laTargetBase, (int)navState, (int)absActive);
      }
    }

    // ── Telemetry snapshot ────────────────────────────────────────────
    snap_pos = pos;
    snap_err = (int32_t)((float)pos - QTR_SETPOINT);
    snap_lPWM = lSpd;
    snap_rPWM = rSpd;
    snap_pidMode = pm;
    snap_obs = obsLa;
    snap_obsRaw = srRaw;
    snap_lineLost = !lineVisible;
    snap_searchMode = (navState == NavState::PIVOT_SEARCH);
    snap_navState = (uint8_t)navState;
    snap_atCross = crossSeen;
    snap_laState = (uint8_t)laStateNow;
    snap_laTurnHint = laTurnHint;
    snap_laHealthy = gLookAheadHealthy;
    snap_dummy45 = dummy45Active;
    snap_nodeCount = nodeCount;
    for (int i = 0; i < 8; i++) snap_qtr[i] = qCal[i];

    // Publish ramped base speed: AT8 publishes its own via gAtRampedBasePub
    // inside the AT8 block; this fallback handles RUNNING / IDLE states.
    if (!isAutoTuning) {
      portENTER_CRITICAL(&gAtMux);
      gAtRampedBasePub = atRampedBase;
      portEXIT_CRITICAL(&gAtMux);
    }

    unsigned long nowMs = millis();
    if (nowMs - lastTelMs >= 100) {
      lastTelMs = nowMs;
      portENTER_CRITICAL(&gTelMux);
      gTel.pos = snap_pos;
      gTel.err = snap_err;
      gTel.lPWM = snap_lPWM;
      gTel.rPWM = snap_rPWM;
      gTel.pidMode = snap_pidMode;
      gTel.obs = snap_obs;
      gTel.obsRaw = snap_obsRaw;
      gTel.lineLost = snap_lineLost;
      gTel.searchMode = snap_searchMode;
      gTel.navState = snap_navState;
      gTel.nodeCount = snap_nodeCount;
      gTel.atCross = snap_atCross;
      gTel.lineWidthSensors = gLineWidthSensors;
      gTel.crossMinSensors = gCrossMinSensors;
      gTel.laState = snap_laState;
      gTel.laTurnHint = snap_laTurnHint;
      gTel.laHealthy = snap_laHealthy;
      gTel.dummy45Active = snap_dummy45;
      for (int i = 0; i < 8; i++) gTel.qtr[i] = snap_qtr[i];
      portEXIT_CRITICAL(&gTelMux);
    }

    lastState = localState;
    qtrStartCharge();
  }  // end while(true)
}

// ══════════════════════════════════════════════════════════════════════
//  WEB INTERFACE  (HTML + CSS + JS — PROGMEM)
//  v7.0: All 10 upgrade UI additions integrated
//    [U1] "Load from Robot" button on PID Tuner tab
//    [U2] AT Base Speed slider now allows 50–255
//    [U7] Dummy-45 filter badge on Dashboard
//    [U5] Node count badge on Dashboard
// ══════════════════════════════════════════════════════════════════════
const char WEB_HTML[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>LineFollower v7.0</title>
<style>
:root{
  --bg:#070710;--bg2:#0d0d1c;--bg3:#111128;
  --border:#1e1e38;--border2:#252548;
  --text:#c8d0f0;--text2:#5a6490;
  --accent:#00a8ff;--green:#00e676;--red:#ff4444;
  --yellow:#ffca28;--orange:#ff7722;--purple:#b060ff;--cyan:#00e5ff;
  --radius:10px;--rsm:6px;--tab-h:60px;--hdr-h:50px;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%;overflow:hidden}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
     background:var(--bg);color:var(--text);font-size:14px;
     display:flex;flex-direction:column}
#hdr{height:var(--hdr-h);background:var(--bg2);border-bottom:1px solid var(--border);
     display:flex;align-items:center;padding:0 14px;gap:10px;flex-shrink:0;z-index:20}
#hdr-title{font-size:12px;font-weight:800;color:var(--accent);white-space:nowrap;letter-spacing:.4px}
#hdr-right{margin-left:auto;display:flex;align-items:center;gap:8px}
.ws-pill{display:flex;align-items:center;gap:5px;font-size:10px;padding:3px 8px;border-radius:20px;
         border:1px solid var(--border2);background:var(--bg3);font-weight:700;letter-spacing:.3px;color:var(--text2)}
.ws-dot{width:7px;height:7px;border-radius:50%;background:var(--red);flex-shrink:0}
.ws-dot.ok{background:var(--green)}
#state-pill{font-size:11px;font-weight:800;padding:3px 10px;border-radius:20px;
            border:1px solid var(--border2);background:var(--bg3);letter-spacing:.5px}
.pkt-cnt{font-size:10px;color:var(--text2)}
#content{flex:1;overflow-y:auto;overflow-x:hidden;
         padding:12px 12px calc(var(--tab-h) + 10px);-webkit-overflow-scrolling:touch}
#tabs{height:var(--tab-h);background:var(--bg2);border-top:1px solid var(--border);
      display:grid;grid-template-columns:repeat(6,1fr);flex-shrink:0;z-index:20}
.tab-btn{display:flex;flex-direction:column;align-items:center;justify-content:center;
         gap:2px;cursor:pointer;font-size:7px;font-weight:700;letter-spacing:.3px;
         color:var(--text2);border:none;background:none;padding:4px 2px;
         text-transform:uppercase;user-select:none;-webkit-user-select:none;
         transition:color .15s;position:relative}
.tab-btn .ti{font-size:17px;line-height:1;display:block}
.tab-btn.active{color:var(--accent)}
.tab-btn.active::after{content:'';position:absolute;bottom:0;left:20%;right:20%;
                        height:2px;background:var(--accent);border-radius:2px 2px 0 0}
.tab-notif{position:absolute;top:6px;right:18%;width:7px;height:7px;
           border-radius:50%;background:var(--red);display:none}
.panel{display:none}
.panel.active{display:block}
.card{background:var(--bg2);border:1px solid var(--border);
      border-radius:var(--radius);padding:14px;margin-bottom:10px}
.card-title{font-size:10px;font-weight:800;color:var(--text2);text-transform:uppercase;
            letter-spacing:.9px;margin-bottom:12px;display:flex;align-items:center;gap:7px}
.cdot{width:5px;height:5px;border-radius:50%;background:var(--accent);flex-shrink:0}
.metric-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px}
.metric{background:var(--bg3);border:1px solid var(--border);border-radius:var(--rsm);padding:11px 13px}
.mlabel{font-size:9px;color:var(--text2);text-transform:uppercase;letter-spacing:.6px;margin-bottom:5px}
.mval{font-size:30px;font-weight:900;line-height:1;font-variant-numeric:tabular-nums;letter-spacing:-1px;color:var(--accent)}
.ebar-wrap{width:100%;height:10px;background:var(--bg3);border-radius:5px;
           overflow:hidden;border:1px solid var(--border);position:relative;margin:8px 0}
.ebar-mid{position:absolute;left:50%;top:0;width:1px;height:100%;background:var(--border2)}
.ebar-fill{height:100%;position:absolute;border-radius:5px;transition:all .08s}
.badge-row{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0}
.badge{font-size:10px;font-weight:700;padding:3px 8px;border-radius:12px;
       border:1px solid var(--border2);background:var(--bg3);color:var(--text2);
       letter-spacing:.3px;white-space:nowrap;transition:all .15s}
.badge.g{color:var(--green);border-color:#1a4a2e;background:#081a0f}
.badge.r{color:var(--red);border-color:#4a1a1a;background:#1a0808;animation:blink 1.2s infinite}
.badge.y{color:var(--yellow);border-color:#4a3800;background:#1a1400}
.badge.b{color:var(--cyan);border-color:#004858;background:#001e28}
.badge.p{color:var(--purple);border-color:#3a1a58;background:#14082a}
.badge.o{color:var(--orange);border-color:#502800;background:#1a0e00}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.45}}
.info-row{display:flex;justify-content:space-between;align-items:center;
          padding:9px 0;border-bottom:1px solid var(--border);font-size:12px}
.info-row:last-child{border-bottom:none}
.ikey{color:var(--text2)}.ival{font-weight:700}
.btn-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px}
.btn{min-height:50px;min-width:80px;font-size:13px;font-weight:800;
     border-radius:var(--rsm);border:1px solid var(--border2);background:var(--bg3);
     color:var(--text);cursor:pointer;display:flex;align-items:center;
     justify-content:center;gap:6px;padding:0 18px;flex:1;
     user-select:none;-webkit-user-select:none;touch-action:manipulation;
     transition:all .12s;letter-spacing:.3px}
.btn:active{transform:scale(.94);filter:brightness(1.4)}
.btn.g{background:#081a0f;border-color:#1a5030;color:var(--green)}
.btn.r{background:#1a0808;border-color:#501818;color:var(--red)}
.btn.b{background:#08101e;border-color:#183050;color:var(--accent)}
.btn.y{background:#1a1400;border-color:#504000;color:var(--yellow)}
.btn.p{background:#14082a;border-color:#3a1a58;color:var(--purple)}
.btn.gr{background:var(--bg3);border-color:var(--border2);color:var(--text2)}
.btn:disabled{opacity:.35;pointer-events:none}
.toggle-row{display:flex;align-items:center;gap:12px;padding:11px 0;border-bottom:1px solid var(--border)}
.toggle-row:last-child{border-bottom:none}
.tlabel{flex:1;font-size:13px}
.tsub{font-size:10px;color:var(--text2);display:block;margin-top:2px}
.tgl{position:relative;width:46px;height:26px;flex-shrink:0}
.tgl input{opacity:0;width:0;height:0}
.tgl-sl{position:absolute;inset:0;background:#1a1a30;border-radius:13px;cursor:pointer;transition:.2s}
.tgl-sl:before{content:'';position:absolute;width:20px;height:20px;
               left:3px;top:3px;background:var(--text2);border-radius:50%;transition:.2s}
.tgl input:checked + .tgl-sl{background:#0a2818}
.tgl input:checked + .tgl-sl:before{transform:translateX(20px);background:var(--green)}
.qtr-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:3px;margin:8px 0}
.qbwrap{display:flex;flex-direction:column;align-items:center;gap:2px}
.qbouter{width:100%;height:64px;background:var(--bg3);border-radius:3px;
         overflow:hidden;display:flex;align-items:flex-end;border:1px solid var(--border)}
.qbinner{width:100%;min-height:2px;transition:height .08s,background .08s;border-radius:2px 2px 0 0}
.qbval{font-size:8px;color:var(--text2);text-align:center}
.qblbl{font-size:8px;color:var(--text2);text-align:center}
.la-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:8px 0}
.la-cell{border:1px solid var(--border2);border-radius:var(--rsm);padding:12px 4px;
         text-align:center;font-size:11px;font-weight:800;color:var(--text2);
         transition:all .12s;background:var(--bg3)}
.la-cell.hit{background:#1e1600;border-color:var(--yellow);color:var(--yellow);box-shadow:0 0 10px #ffca2838}
.lasub{font-size:9px;font-weight:400;color:var(--text2);margin-top:4px;display:block}
.la-cell.hit .lasub{color:#ffca2880}
.pos-bar{width:100%;height:22px;background:var(--bg3);position:relative;
         border-radius:4px;margin:8px 0;overflow:hidden;border:1px solid var(--border)}
.pos-mid{position:absolute;left:50%;top:0;width:1px;height:100%;background:var(--border2)}
.pos-cur{position:absolute;top:3px;width:6px;height:16px;background:var(--cyan);
         border-radius:3px;margin-left:-3px;transition:left .08s}
.pid-tabs{display:flex;gap:4px;margin-bottom:12px}
.pid-tab{flex:1;padding:9px 4px;text-align:center;cursor:pointer;font-size:11px;font-weight:800;
         letter-spacing:.3px;border-radius:var(--rsm);border:1px solid var(--border);
         background:var(--bg3);color:var(--text2);transition:.15s;user-select:none}
.pid-tab.m0{}.pid-tab.m1{}.pid-tab.m2{}
.pid-tab.active.m0{background:#08101e;border-color:var(--accent);color:var(--accent)}
.pid-tab.active.m1{background:#1a1400;border-color:var(--yellow);color:var(--yellow)}
.pid-tab.active.m2{background:#1a0808;border-color:var(--red);color:var(--red)}
.pid-field{margin-bottom:14px}
.pfl{font-size:11px;color:var(--text2);margin-bottom:5px;display:block}
.pid-field input{width:100%;background:var(--bg3);border:1px solid var(--border2);
                 border-radius:var(--rsm);padding:10px 12px;color:var(--text);
                 font-size:15px;font-weight:700;outline:none}
.pid-field input:focus{border-color:var(--accent)}
.apply-row{display:flex;gap:6px;margin-top:14px;flex-wrap:wrap;align-items:center}
.amsg{font-size:11px;flex:1;text-align:right;min-height:14px}
.pnote{font-size:10px;color:var(--text2);line-height:1.8;margin-top:4px}
.spd-row{display:flex;align-items:center;gap:10px;padding:12px 14px;
         background:var(--bg3);border-radius:var(--rsm);border:1px solid var(--border);margin-bottom:16px}
.spd-row label{font-size:12px;color:var(--text2);white-space:nowrap}
.spd-row input{flex:1}
.spd-val{font-size:18px;font-weight:900;color:var(--cyan);min-width:38px;text-align:right;font-variant-numeric:tabular-nums}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);gap:8px;max-width:300px;margin:0 auto;aspect-ratio:1}
.db{border-radius:var(--radius);border:1px solid var(--border2);background:var(--bg3);
    color:var(--accent);display:flex;flex-direction:column;align-items:center;
    justify-content:center;gap:4px;cursor:pointer;font-size:26px;
    user-select:none;-webkit-user-select:none;touch-action:none;transition:all .08s;min-height:72px}
.db span{font-size:9px;font-weight:700;opacity:.6;letter-spacing:.5px;text-transform:uppercase}
.db:active,.db.pr{background:#07121e;border-color:var(--accent);box-shadow:0 0 14px #00a8ff30;transform:scale(.93)}
.db-stp{background:#1a0808;border-color:#501818;color:var(--red)}
.db-stp.pr,.db-stp:active{border-color:var(--red);box-shadow:0 0 14px #ff444430}
.db-em{pointer-events:none;background:transparent;border-color:transparent}
.drive-locked{background:#1a0808;border:1px solid #501818;border-radius:var(--rsm);
              padding:18px;text-align:center;color:var(--red);font-size:13px;
              font-weight:800;letter-spacing:.3px;margin-bottom:14px}
.log-hdr{display:flex;gap:8px;margin-bottom:8px}
#log{background:#040410;border:1px solid var(--border);border-radius:var(--rsm);
     height:calc(100vh - var(--hdr-h) - var(--tab-h) - 90px);
     overflow-y:auto;padding:10px 12px;font:11px/1.8 'Courier New',monospace;
     scroll-behavior:smooth;-webkit-overflow-scrolling:touch}
.ll{white-space:pre-wrap;word-break:break-all;color:var(--accent)}
.ll.ok{color:var(--green)}.ll.warn{color:var(--yellow)}.ll.err{color:var(--red)}
.ll.maze{color:var(--purple)}.ll.comp{color:#ff80ab}.ll.hla{color:var(--cyan)}
.ll.at{color:var(--purple);font-weight:700}
.at-prog-wrap{width:100%;height:8px;background:var(--bg3);border-radius:4px;
              overflow:hidden;border:1px solid var(--border);margin:10px 0}
.at-prog-fill{height:100%;border-radius:4px;transition:width .3s,background .3s;width:0%}
.sep{border:none;border-top:1px solid var(--border);margin:12px 0}
@media(min-width:480px){.mval{font-size:36px}.dpad{max-width:340px}}
@media(min-width:720px){.metric-grid{grid-template-columns:repeat(4,1fr)}}
.at-mode-tabs{display:flex;gap:4px;margin-bottom:12px}
.at-mode-tab{flex:1;padding:10px 4px;text-align:center;cursor:pointer;font-size:11px;
             font-weight:800;letter-spacing:.3px;border-radius:var(--rsm);
             border:1px solid var(--border);background:var(--bg3);color:var(--text2);transition:.15s;user-select:none}
.at-mode-tab.active{background:#14082a;border-color:var(--purple);color:var(--purple)}
#atModal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.80);z-index:200;
         align-items:center;justify-content:center;padding:20px;overflow-y:auto}
#atModal.open{display:flex}
#atModalBox{background:var(--bg2);border:1px solid var(--border2);
            border-radius:var(--radius);width:100%;max-width:420px;padding:20px;max-height:90vh;overflow-y:auto}
.modal-hdr{display:flex;align-items:center;gap:14px;margin-bottom:18px}
.modal-icon{font-size:36px;line-height:1;flex-shrink:0}
.modal-title h3{font-size:16px;font-weight:800;color:var(--text)}
.modal-title p{font-size:11px;color:var(--text2);margin-top:3px}
.modal-row{display:flex;justify-content:space-between;align-items:center;
           padding:10px 0;border-bottom:1px solid var(--border);font-size:12px}
.modal-key{color:var(--text2)}
.modal-val{font-weight:800;color:var(--yellow)}
.modal-val.g{color:var(--green)}
.modal-btn-row{display:flex;gap:8px;margin-top:18px;flex-wrap:wrap}
.modal-btn{flex:1;min-height:46px;font-size:12px;font-weight:800;
           border-radius:var(--rsm);border:1px solid var(--border2);
           background:var(--bg3);color:var(--text);cursor:pointer;transition:.12s;user-select:none}
.modal-btn:active{transform:scale(.94);filter:brightness(1.4)}
.modal-btn.g{background:#081a0f;border-color:#1a5030;color:var(--green)}
.modal-btn.p{background:#14082a;border-color:#3a1a58;color:var(--purple)}
.modal-btn.gr{background:var(--bg3);border-color:var(--border2);color:var(--text2)}
</style>
</head>
<body>

<!-- HEADER -->
<div id="hdr">
  <div id="hdr-title">&#x1F3CE;&#xFE0F; LineFollower v7.0</div>
  <div id="hdr-right">
    <div class="ws-pill"><div class="ws-dot" id="wsDot"></div><span id="wsLbl">offline</span></div>
    <div id="state-pill" style="color:var(--text2)">IDLE</div>
    <span class="pkt-cnt" id="pktCnt" style="display:none"></span>
  </div>
</div>

<div id="content">

<!-- ═══════════ DASHBOARD ═══════════ -->
<div class="panel active" id="panel-dash">
  <div class="card">
    <div class="card-title"><span class="cdot"></span>Live Telemetry</div>
    <div class="metric-grid">
      <div class="metric"><div class="mlabel">Position</div><div class="mval" id="mv_pos" style="color:var(--cyan)">&#8212;</div></div>
      <div class="metric"><div class="mlabel">Error</div><div class="mval" id="mv_err">&#8212;</div></div>
      <div class="metric"><div class="mlabel">Left PWM</div><div class="mval" id="mv_lp" style="color:var(--green)">&#8212;</div></div>
      <div class="metric"><div class="mlabel">Right PWM</div><div class="mval" id="mv_rp" style="color:var(--green)">&#8212;</div></div>
    </div>
    <div class="ebar-wrap"><div class="ebar-mid"></div><div class="ebar-fill" id="egFill"></div></div>
    <div style="font-size:9px;color:var(--text2);text-align:center">Error bar &mdash; green=centred &middot; yellow=curve &middot; red=far-off</div>
  </div>
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--purple)"></span>Status Badges</div>
    <div class="badge-row">
      <span class="badge b" id="navBadge">FOLLOW</span>
      <span class="badge" id="pmBadge">PID: &#8212;</span>
      <span class="badge" id="laBadge">LA: STRAIGHT</span>
      <span class="badge b" id="brkBadge">&#9881; HW(HLA)</span>
      <span class="badge r" id="lostBadge" style="display:none">&#9888; LINE LOST</span>
      <span class="badge y" id="searchBadge" style="display:none">&#8635; RECOVERY SPIN</span>
      <span class="badge b" id="crossBadge" style="display:none">&#8862; CROSS</span>
      <span class="badge p" id="atRunBadge" style="display:none">&#9883; AUTO-TUNING</span>
      <span class="badge o" id="dummy45Badge" style="display:none">&#9678; 45&#176; FILTERED</span>
      <span class="badge g" id="nodesBadge">NODES: 0</span>
    </div>
    <div class="info-row"><span class="ikey">Calibrated</span><span class="ival" id="calVal" style="color:var(--red)">Not calibrated</span></div>
    <div class="info-row"><span class="ikey">IR Emitters</span><span class="ival" id="irVal">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Line Width Sensors</span><span class="ival" id="lwVal">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Cross Min Sensors</span><span class="ival" id="crVal">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Braking System</span><span class="ival" id="brkSysVal">&#8212;</span></div>
    <div class="info-row"><span class="ikey">LA Health</span><span class="badge g" id="laHealthBadge" style="margin:0">LA: HEALTHY</span></div>
    <div class="info-row"><span class="ikey">Active Nodes [U5]</span><span class="ival" id="nodeCountVal">0</span></div>
  </div>
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--green)"></span>QTR Sensor Array</div>
    <div class="qtr-grid">
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf0"></div></div><div class="qbval" id="qbv0">0</div><div class="qblbl">S0</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf1"></div></div><div class="qbval" id="qbv1">0</div><div class="qblbl">S1</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf2"></div></div><div class="qbval" id="qbv2">0</div><div class="qblbl">S2</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf3"></div></div><div class="qbval" id="qbv3">0</div><div class="qblbl">S3</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf4"></div></div><div class="qbval" id="qbv4">0</div><div class="qblbl">S4</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf5"></div></div><div class="qbval" id="qbv5">0</div><div class="qblbl">S5</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf6"></div></div><div class="qbval" id="qbv6">0</div><div class="qblbl">S6</div></div>
      <div class="qbwrap"><div class="qbouter"><div class="qbinner" id="qbf7"></div></div><div class="qbval" id="qbv7">0</div><div class="qblbl">S7</div></div>
    </div>
    <div class="pos-bar"><div class="pos-mid"></div><div class="pos-cur" id="posCur"></div></div>
  </div>
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--yellow)"></span>Look-Ahead Sensors</div>
    <div class="la-grid">
      <div class="la-cell" id="lac0">S1<span class="lasub">6cm L-End</span></div>
      <div class="la-cell" id="lac1">S2<span class="lasub">2cm L-Mid</span></div>
      <div class="la-cell" id="lac2">S3<span class="lasub">2cm R-Mid</span></div>
      <div class="la-cell" id="lac3">S4<span class="lasub">6cm R-End</span></div>
    </div>
    <div class="badge-row">
      <span class="badge b" id="laStateBadge">STRAIGHT</span>
      <span class="badge" id="laDirBadge">DIR: NONE</span>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-top:4px" id="obsRawTxt">raw=0x00</div>
  </div>
</div>

<!-- ═══════════ CONTROLS ═══════════ -->
<div class="panel" id="panel-ctrl">
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--green)"></span>Robot Control</div>
    <div class="btn-row">
      <button class="btn g" id="btnStart" onclick="sendCmd('start')">&#9654; START</button>
      <button class="btn r" id="btnStop"  onclick="sendCmd('stop')">&#9632; STOP</button>
    </div>
    <div class="btn-row">
      <button class="btn b" onclick="sendCmd('calibrate')">&#9881; CALIBRATE</button>
    </div>
  </div>
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--yellow)"></span>Feature Toggles</div>
    <div class="toggle-row">
      <div class="tlabel">Look-Ahead Sensors<span class="tsub">S1–S4 geometry braking [U4]</span></div>
      <label class="tgl"><input type="checkbox" id="obsEn" checked onchange="send({t:'obs',en:this.checked})"><span class="tgl-sl"></span></label>
    </div>
    <div class="toggle-row">
      <div class="tlabel">QTR IR Emitters<span class="tsub">Toggle IR LED array</span></div>
      <label class="tgl"><input type="checkbox" id="irEn" checked onchange="send({t:'ir',en:this.checked})"><span class="tgl-sl"></span></label>
    </div>
  </div>
</div>

<!-- ═══════════ PID TUNER ═══════════ -->
<div class="panel" id="panel-pid">
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--yellow)"></span>PID Tuner</div>
    <!-- [U1] Profile tabs -->
    <div class="pid-tabs">
      <div class="pid-tab m0 active" id="pidTab0" onclick="selPidMode(0)">M0 Slow</div>
      <div class="pid-tab m1"        id="pidTab1" onclick="selPidMode(1)">M1 Curve</div>
      <div class="pid-tab m2"        id="pidTab2" onclick="selPidMode(2)">M2 Fast</div>
    </div>
    <!-- [U1] Load from Robot button -->
    <div class="btn-row" style="margin-bottom:14px">
      <button class="btn b" onclick="fetchPidFromRobot()" title="Read current PID values from robot">&#8645; Load from Robot</button>
    </div>
    <div id="pidFields">
      <div class="pid-field"><label class="pfl">Kp (Proportional)</label><input type="number" step="0.001" id="pKp" value="0.550"></div>
      <div class="pid-field"><label class="pfl">Ki (Integral)</label><input type="number" step="0.0001" id="pKi" value="0.000"></div>
      <div class="pid-field"><label class="pfl">Kd (Derivative)</label><input type="number" step="0.1" id="pKd" value="9.0"></div>
      <div class="pid-field"><label class="pfl">Base Speed (PWM)</label><input type="number" step="1" id="pBase" value="110"></div>
      <div class="pid-field"><label class="pfl">Top Speed (PWM)</label><input type="number" step="1" id="pTop" value="140"></div>
      <div class="pid-field"><label class="pfl">Low Speed (PWM)</label><input type="number" step="1" id="pLow" value="60"></div>
    </div>
    <div class="apply-row">
      <button class="btn y" onclick="applyPID()">&#10003; Apply PID</button>
      <span class="amsg" id="pidMsg"></span>
    </div>
    <div class="pnote">Active PID badge: <span id="activePidBadge" class="badge">M0</span></div>
  </div>
</div>

<!-- ═══════════ AUTOTUNE ═══════════ -->
<div class="panel" id="panel-at">
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--purple)"></span>Ziegler-Nichols AutoTune
      <span class="badge p" id="atSafetyBadge" style="display:none;margin-left:auto">&#9888; ABORT</span>
    </div>
<div class="at-mode-tabs">
      <div class="at-mode-tab active" id="atMode0" onclick="selAtMode(0)">M0 Straight</div>
      <div class="at-mode-tab"        id="atMode1" onclick="selAtMode(1)">M1 Curve 90°</div>
      <div class="at-mode-tab"        id="atMode2" onclick="selAtMode(2)">M2 S-Curve</div>
      <div class="at-mode-tab"        id="atMode3" onclick="selAtMode(3)" style="color:var(--orange);border-color:#502800">&#9650; Corner Sweep</div>
    </div>
    <div id="atModeDesc" style="font-size:11px;color:var(--cyan);background:rgba(0,168,255,0.07);border:1px solid rgba(0,168,255,0.22);border-radius:6px;padding:8px 10px;margin-bottom:10px;line-height:1.6">
      <b>M0 Straight:</b> Tunes straight profile &rarr; saved to M0 (gPID[0]).<br>
      Then tunes turn profile &rarr; saved to M1+M2. Full dual-profile run.
    </div>
    <div id="atSweepInfo" style="display:none;background:#1a0e00;border:1px solid #502800;border-radius:6px;padding:10px;margin-bottom:12px;font-size:11px;color:var(--orange);line-height:1.8">
      <b>&#9650; Corner Speed Sweep Mode</b><br>
      Robot loops the track. AutoTune starts at <b>60 PWM</b> and steps up by <b>10 PWM</b> per confirmed round.<br>
      Stops automatically when a corner fails (stall/violent osc).<br>
      Reports <b>maximum corner speed</b> + optimal Kp/Kd → applied to <b>M1 (Curve)</b>.<br>
      &#9888; Place robot on a closed loop with 90° corners (like your track image).
    </div>
<div id="atSpdRow">
      <div class="spd-row" id="atSpdRow0" style="display:flex">
        <label>M0 Start Speed (PWM)</label>
        <input type="range" min="50" max="255" value="200" id="atBaseSpd0"
               oninput="el('atBaseSpdVal0').textContent=this.value;ws&&ws.send('SET_MODE_SPEED:0:'+this.value)">
        <span class="spd-val" id="atBaseSpdVal0">200</span>
      </div>
      <div class="spd-row" id="atSpdRow1" style="display:none">
        <label>M1 Start Speed (PWM)</label>
        <input type="range" min="50" max="255" value="150" id="atBaseSpd1"
               oninput="el('atBaseSpdVal1').textContent=this.value;ws&&ws.send('SET_MODE_SPEED:1:'+this.value)">
        <span class="spd-val" id="atBaseSpdVal1">150</span>
      </div>
      <div class="spd-row" id="atSpdRow2" style="display:none">
        <label>M2 Start Speed (PWM)</label>
        <input type="range" min="50" max="255" value="120" id="atBaseSpd2"
               oninput="el('atBaseSpdVal2').textContent=this.value;ws&&ws.send('SET_MODE_SPEED:2:'+this.value)">
        <span class="spd-val" id="atBaseSpdVal2">120</span>
      </div>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-bottom:12px">
      [FIX] Per-mode start speed 50–255 PWM. Firmware safe-starts at 55 PWM and ramps to selected speed.
    </div>

    <!-- [FIX-PERMODE] Per-mode ZN/Twiddle Kp Start + Step inputs -->
    <!-- Each row visible only when its mode tab is active (controlled by selAtMode JS) -->
    <div id="atKpRow0" style="display:flex;gap:8px;margin-bottom:6px;flex-wrap:wrap">
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M0 ZN Kp Start</label>
        <input type="number" id="atKpStart0" step="0.001" min="0.001" max="1.0" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(0)">
      </div>
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M0 ZN Kp Step</label>
        <input type="number" id="atKpStep0" step="0.001" min="0.001" max="0.5" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(0)">
      </div>
    </div>
    <div id="atKpRow1" style="display:none;gap:8px;margin-bottom:6px;flex-wrap:wrap">
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M1 ZN Kp Start</label>
        <input type="number" id="atKpStart1" step="0.001" min="0.001" max="1.0" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(1)">
      </div>
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M1 ZN Kp Step</label>
        <input type="number" id="atKpStep1" step="0.001" min="0.001" max="0.5" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(1)">
      </div>
    </div>
    <div id="atKpRow2" style="display:none;gap:8px;margin-bottom:6px;flex-wrap:wrap">
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M2 ZN Kp Start</label>
        <input type="number" id="atKpStart2" step="0.001" min="0.001" max="1.0" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(2)">
      </div>
      <div class="pid-field" style="flex:1;min-width:120px;margin-bottom:0">
        <label class="pfl">M2 ZN Kp Step</label>
        <input type="number" id="atKpStep2" step="0.001" min="0.001" max="0.5" value="0.020"
               style="width:100%;background:var(--bg3);border:1px solid var(--border2);border-radius:var(--rsm);padding:8px 10px;color:var(--text);font-size:14px;font-weight:700;outline:none"
               onchange="applyAtKpParam(2)">
      </div>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-bottom:12px">
      [FIX] Per-mode: Kp Start = initial Twiddle Kp seed (warm start); Kp Step = initial dpKp search granularity.
    </div>
    <div class="btn-row">
      <button class="btn p" id="atStartBtn" onclick="startAt()">&#9654; Start AutoTune</button>
      <button class="btn r" id="atStopBtn"  onclick="stopAt()" disabled>&#9632; Stop</button>
    </div>
    <div class="btn-row" style="margin-top:6px">
      <button class="btn" id="atBrakeBtn"
        style="background:linear-gradient(135deg,#7a1a00,#b03000);border-color:#ff4400;color:#fff;font-weight:700;letter-spacing:0.5px"
        onclick="manualBrake()"
        title="Manually trigger ABS braking (3 cycles). Works during Run or AutoTune.">
        &#9632;&#9632; BRAKE TEST (ABS)
      </button>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-bottom:8px">
      [ABS] Manual brake: triggers 3×150ms ABS cycles. Use to verify braking during tuning.
    </div>
    <hr class="sep">
    <div class="info-row"><span class="ikey">Status</span><span class="ival" id="atStatus" style="color:var(--text2)">IDLE</span></div>
    <div class="info-row"><span class="ikey">Active Mode</span><span class="ival" id="atActiveModeLabel">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Current Kp</span><span class="ival" id="atKpCur">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Zero Crossings (gated)</span><span class="ival" id="atCrossings">0 / 8</span></div>
    <div class="info-row"><span class="ikey">Ramp Progress</span><span class="ival" id="atRampLabel">&#8212;</span></div>
    <!-- Sweep-mode rows (hidden unless sweep active) -->
    <div class="info-row" id="sweepSpdRow" style="display:none">
      <span class="ikey">&#9650; Sweep Speed</span>
      <span class="ival" style="color:var(--orange)" id="sweepSpdVal">&#8212; PWM</span>
    </div>
    <div class="info-row" id="sweepBestRow" style="display:none">
      <span class="ikey">&#9654; Best Corner Speed</span>
      <span class="ival" style="color:var(--green)" id="sweepBestVal">&#8212; PWM</span>
    </div>
    <div class="info-row" id="sweepRoundRow" style="display:none">
      <span class="ikey">Confirm Rounds</span>
      <span class="ival" id="sweepRoundVal">&#8212;</span>
    </div>
    <div class="at-prog-wrap"><div class="at-prog-fill" id="atProgFill"></div></div>
    <span style="font-size:10px;color:var(--text2)" id="atPhaseLabel">&#8212;</span>
  </div>
  <div class="card" id="atResultCard" style="display:none">
    <div class="card-title"><span class="cdot" style="background:var(--green)"></span>Last AutoTune Result</div>
    <div class="info-row"><span class="ikey">Mode</span><span class="ival" id="atResMode">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Ku (Ultimate Gain)</span><span class="ival" id="atKu">&#8212;</span></div>
    <div class="info-row"><span class="ikey">Tu (Period)</span><span class="ival" id="atTu">&#8212;</span></div>
    <div class="info-row"><span class="ikey">ZN Kp</span><span class="ival" id="atResKp">&#8212;</span></div>
    <div class="info-row"><span class="ikey">ZN Kd</span><span class="ival" id="atResKd">&#8212;</span></div>
  </div>

  <!-- [AT8] Dual-Profile Adaptive Tuner Live Status Card -->
  <div class="card" style="border-color:#252548">
    <div class="card-title"><span class="cdot" style="background:var(--purple)"></span>[AT8] Dual-Profile Adaptive Tuner</div>
    <div class="info-row">
      <span class="ikey">Phase</span>
      <span class="ival" id="at8Phase" style="color:var(--purple)">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Terrain</span>
      <span class="ival" id="at8Terrain" style="color:var(--text2)">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Current Speed</span>
      <span class="ival" id="at8Spd">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Straight Kp</span>
      <span class="ival" id="at8KpS">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Straight Kd</span>
      <span class="ival" id="at8KdS">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Str Profile Lock</span>
      <span class="ival" id="at8SLock" style="color:var(--yellow)">unlocked</span>
    </div>
    <div class="info-row">
      <span class="ikey">Turn Kp</span>
      <span class="ival" id="at8KpT">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Turn Kd</span>
      <span class="ival" id="at8KdT">&#8212;</span>
    </div>
<div class="info-row" style="background:rgba(0,168,255,0.09);border:1px solid rgba(0,168,255,0.3);border-radius:6px;padding:7px 10px;margin-bottom:5px">
      <span class="ikey" style="color:var(--cyan);font-weight:700">&#9654; Active Kp NOW</span>
      <span class="ival" id="at8ActKp" style="color:var(--cyan);font-weight:700;font-size:16px">&#8212;</span>
    </div>
    <div class="info-row" style="background:rgba(0,168,255,0.09);border:1px solid rgba(0,168,255,0.3);border-radius:6px;padding:7px 10px;margin-bottom:8px">
      <span class="ikey" style="color:var(--cyan);font-weight:700">&#9654; Active Kd NOW</span>
      <span class="ival" id="at8ActKd" style="color:var(--cyan);font-weight:700;font-size:16px">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">Tuning Mode</span>
      <span class="ival" id="at8TuneModeLbl" style="color:var(--yellow)">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">IAE Best Fitness</span>
      <span class="ival" id="at8Fitness">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">IAE Live Counter</span>
      <span class="ival" id="at8CurFit" style="color:var(--orange)">&#8212;</span>
    </div>
    <div class="info-row">
      <span class="ikey">&#931;dp (convergence)</span>
      <span class="ival" id="at8DpSum">&#8212;</span>
    </div>
    <div style="margin-top:10px;margin-bottom:6px">
      <button class="btn p" id="at8PushBtn" style="width:100%;font-size:12px;min-height:44px"
        onclick="pushAt8ToPid()"
        title="Push live AT8 Kp/Kd to PID slots and save to NVS now">
        &#11014; Push AT8 &rarr; PID (Save to NVS)
      </button>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-bottom:10px">
      M0&nbsp;mode: Straight&rarr;M0, Turn&rarr;M1+M2 &nbsp;|&nbsp; M1: Turn&rarr;M1 &nbsp;|&nbsp; M2: Turn&rarr;M2
    </div>
    <div style="display:flex;gap:6px;flex-wrap:wrap">
      <button class="btn r" style="font-size:11px;padding:6px 12px"
        onclick="resetAt8Checkpoint(0)">&#128465; Reset M0</button>
      <button class="btn r" style="font-size:11px;padding:6px 12px"
        onclick="resetAt8Checkpoint(1)">&#128465; Reset M1</button>
      <button class="btn r" style="font-size:11px;padding:6px 12px"
        onclick="resetAt8Checkpoint(2)">&#128465; Reset M2</button>
      <button class="btn r" style="font-size:11px;padding:6px 12px;border-color:#7a1a1a"
        onclick="resetAt8Checkpoint(-1)">&#128465; Reset ALL</button>
    </div>
    <div style="font-size:10px;color:var(--text2);margin-top:4px">
      [FIX] Reset: clears NVS checkpoint + forces COLD start from TWIDDLE_KP_INIT_STEP.
      Per-mode reset only clears that mode's checkpoint.
    </div>
  </div>
</div>

<!-- ═══════════ DRIVE ═══════════ -->
<div class="panel" id="panel-drv">
  <div class="card">
    <div class="card-title"><span class="cdot" style="background:var(--cyan)"></span>Manual Drive</div>
    <div class="drive-locked" id="driveLocked" style="display:none">&#128274; Stop robot before manual drive</div>
    <div class="spd-row">
      <label>Speed</label>
      <input type="range" min="30" max="255" value="100" id="drvSpd" oninput="el('drvSpdVal').textContent=this.value">
      <span class="spd-val" id="drvSpdVal">100</span>
    </div>
    <div class="dpad">
      <div class="db-em"></div>
      <div class="db" id="db_fwd"   onpointerdown="dpad('fwd')"  onpointerup="dpad('stop')" onpointercancel="dpad('stop')">&#9650;<span>FWD</span></div>
      <div class="db-em"></div>
      <div class="db" id="db_left"  onpointerdown="dpad('left')" onpointerup="dpad('stop')" onpointercancel="dpad('stop')">&#9668;<span>LEFT</span></div>
      <div class="db db-stp"        onpointerdown="dpad('stop')">&#9632;<span>STOP</span></div>
      <div class="db" id="db_right" onpointerdown="dpad('right')"onpointerup="dpad('stop')" onpointercancel="dpad('stop')">&#9658;<span>RIGHT</span></div>
      <div class="db-em"></div>
      <div class="db" id="db_back"  onpointerdown="dpad('back')" onpointerup="dpad('stop')" onpointercancel="dpad('stop')">&#9660;<span>BACK</span></div>
      <div class="db-em"></div>
    </div>
  </div>
</div>

<!-- ═══════════ LOG ═══════════ -->
<div class="panel" id="panel-log">
  <div class="log-hdr">
    <button class="btn gr" style="min-height:34px;font-size:11px" onclick="el('log').innerHTML=''">Clear</button>
    <button class="btn b"  style="min-height:34px;font-size:11px" id="logScrollBtn" onclick="logAutoScroll=!logAutoScroll;this.textContent=logAutoScroll?'&#9660; Auto':'&#9660; Manual'">&#9660; Auto</button>
  </div>
  <div id="log"></div>
</div>

</div><!-- /content -->

<!-- TAB BAR -->
<div id="tabs">
  <button class="tab-btn active" onclick="selTab(0)"><span class="ti">&#128200;</span>Dash</button>
  <button class="tab-btn"        onclick="selTab(1)"><span class="ti">&#127918;</span>Control</button>
  <button class="tab-btn"        onclick="selTab(2)"><span class="ti">&#9881;</span>PID<span class="tab-notif" id="pidTabNotif"></span></button>
  <button class="tab-btn"        onclick="selTab(3)"><span class="ti">&#9883;</span>AutoTune<span class="tab-notif" id="atTabNotif"></span></button>
  <button class="tab-btn"        onclick="selTab(4)"><span class="ti">&#127918;</span>Drive</button>
  <button class="tab-btn"        onclick="selTab(5)"><span class="ti">&#128196;</span>Log</button>
</div>

<!-- ═══════════ AT RESULT MODAL ═══════════ -->
<div id="atModal">
  <div id="atModalBox">
    <div class="modal-hdr">
      <span class="modal-icon">&#9883;</span>
   <div class="modal-title"><h3>AutoTune Complete!</h3><p id="mResDesc">Results ready to apply.</p></div>
    </div>
    <div class="modal-row"><span class="modal-key">Mode</span><span class="modal-val" id="mResMode">&#8212;</span></div>
    <div class="modal-row" id="mRowKu"><span class="modal-key">Ku</span><span class="modal-val" id="mResKu">&#8212;</span></div>
    <div class="modal-row" id="mRowTu"><span class="modal-key">Tu</span><span class="modal-val" id="mResTu">&#8212;</span></div>
    <div class="modal-row"><span class="modal-key" id="mKpLabel">Kp</span><span class="modal-val g" id="mResKp">&#8212;</span></div>
    <div class="modal-row"><span class="modal-key" id="mKdLabel">Kd</span><span class="modal-val g" id="mResKd">&#8212;</span></div>
    <div class="modal-btn-row">
      <button class="modal-btn g" onclick="modalApplySave()">&#10003; Apply &amp; Save</button>
      <button class="modal-btn p" onclick="closeModal()">&#8635; Retune</button>
      <button class="modal-btn gr" onclick="closeModal()">Dismiss</button>
    </div>
  </div>
</div>

<script>
'use strict';
var ws=null, pktCount=0, logAutoScroll=true;
var lastAtSuccess=null, lastAtKp=0, lastAtKd=0;
var curPidMode=0, curTelPm=0;

// PID defaults per mode (firmware defaults)
var PID_DEFAULTS=[
  {kP:0.550,kI:0.000,kD:9.0,  base:110,top:140,low:60},
  {kP:0.900,kI:0.000,kD:14.0, base:70, top:90, low:40},
  {kP:1.400,kI:0.000,kD:22.0, base:48, top:68, low:28}
];

var MODES    =['Slow/M0','Curve/M1','Fast/M2'];
var MCOL     =['#00e5ff','#ffca28','#ff4444'];
var NAV_STATES=['FOLLOW','DASHED_FWD','PIVOT_SEARCH','AT_CROSS','TURN_L','TURN_R','UTURN'];
var NAV_COLS  =['#00e5ff','#ffca28','#ff4444','#b060ff','#00e5ff','#00e5ff','#ff7722'];
var NAV_BG    =['#001e28','#1a1400','#1a0808','#14082a','#001e28','#001e28','#1a0e00'];
var NAV_BC    =['#004858','#4a3800','#4a1a1a','#3a1a58','#004858','#004858','#502800'];
var LA_LABELS =['STRAIGHT','BRAKE 6CM','PREP 2CM'];
var LA_COLS   =['#00e676','#ffca28','#ff4444'];
var LA_BG     =['#081a0f','#1a1400','#1a0808'];
var LA_BC     =['#1a5030','#4a3800','#4a1a1a'];
var AT_PHASES =['SETTLING','DETECTING','COMPLETE','ABORTED'];
var AT_PCOLS  =['#ffca28','#00e5ff','#00e676','#ff4444'];
var AT_MODE_SHORT=['STRAIGHT','CURVE-90','S-CURVE','CORNER-SWEEP'];
var curAtMode =0;

function el(id){return document.getElementById(id);}
function txt(id,v){var e=el(id);if(e)e.textContent=v;}

// ── Tab & mode selectors ──────────────────────────────────────────────
var PANELS=['dash','ctrl','pid','at','drv','log'];
function selTab(i){
  document.querySelectorAll('.panel').forEach(function(p){p.classList.remove('active');});
  document.querySelectorAll('.tab-btn').forEach(function(b,j){b.classList.toggle('active',j===i);});
  var p=el('panel-'+PANELS[i]);if(p)p.classList.add('active');
  if(i===2){var n=el('pidTabNotif');if(n)n.style.display='none';}
  if(i===3){var n=el('atTabNotif'); if(n)n.style.display='none';}
}
function selPidMode(m){
  curPidMode=m;
  [0,1,2].forEach(function(i){
    var t=el('pidTab'+i);
    if(t){t.classList.toggle('active',i===m);}
  });
  var d=PID_DEFAULTS[m];
  el('pKp').value =d.kP;   el('pKi').value=d.kI;
  el('pKd').value =d.kD;   el('pBase').value=d.base;
  el('pTop').value=d.top;  el('pLow').value=d.low;
}
var AT_MODE_DESC_HTML=[
  '<b>M0 Straight:</b> Tunes straight profile &rarr; M0 (gPID[0]).<br>Then tunes turn profile &rarr; M1+M2. Full dual-profile.',
  '<b>M1 Curve 90&deg;:</b> Tunes turn/corner profile ONLY &rarr; M1 (gPID[1]).<br>Straight profile stays unchanged.',
  '<b>M2 S-Curve / Fast:</b> Tunes fast/S-curve profile ONLY &rarr; M2 (gPID[2]).<br>Seeded from current M2 values.',
  '<b>&#9650; Corner Speed Sweep:</b> Starts at 60 PWM, steps up until corner fails.<br>Reports max corner speed + Kp/Kd &rarr; M1.'
];
var AT_MODE_DESC_COL=['var(--cyan)','var(--green)','var(--yellow)','var(--orange)'];
function selAtMode(m){
  curAtMode=m;
  [0,1,2,3].forEach(function(i){
    var t=el('atMode'+i);if(t)t.classList.toggle('active',i===m);
  });
  var si=el('atSweepInfo');
  if(si) si.style.display=(m===3)?'block':'none';
  // Show per-mode speed row for active mode tab; hide during sweep
  [0,1,2].forEach(function(i){
    var sr=el('atSpdRow'+i);
    if(sr) sr.style.display=(m!==3&&m===i)?'flex':'none';
    // [FIX-PERMODE] Show per-mode ZN Kp Start/Step row for active mode tab
    var kr=el('atKpRow'+i);
    if(kr) kr.style.display=(m!==3&&m===i)?'flex':'none';
  });
  var md=el('atModeDesc');
  if(md){
    md.innerHTML=AT_MODE_DESC_HTML[m]||'';
    md.style.color=AT_MODE_DESC_COL[m]||'var(--cyan)';
    md.style.borderColor=(m===3)?'rgba(255,119,34,0.35)':'rgba(0,168,255,0.22)';
    md.style.background=(m===3)?'rgba(255,119,34,0.07)':'rgba(0,168,255,0.07)';
  }
}
function updateActiveBadge(){
  var b=el('activePidBadge');
  if(b){b.textContent='M'+curTelPm+' '+MODES[curTelPm];b.style.color=MCOL[curTelPm];}
}

// ── WebSocket ─────────────────────────────────────────────────────────
function connect(){
  if(ws){try{ws.close();}catch(e){}}
  ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=function(){
    var d=el('wsDot'),l=el('wsLbl');
    if(d){d.className='ws-dot ok';}if(l)l.textContent='online';
    // [FIX] Send keepalive ping every 2 s to prevent WS failsafe (threshold = 5 s)
    if(window._kaTimer)clearInterval(window._kaTimer);
    window._kaTimer=setInterval(function(){
      if(ws&&ws.readyState===1)ws.send(JSON.stringify({t:'ping'}));
    },2000);
  };
  ws.onclose=function(){
    var d=el('wsDot'),l=el('wsLbl');
    if(d){d.className='ws-dot';}if(l)l.textContent='offline';
    if(window._kaTimer){clearInterval(window._kaTimer);window._kaTimer=null;}
    setTimeout(connect,3000);
  };
  ws.onerror=function(){ws.close();};
  ws.onmessage=function(ev){
    pktCount++;
    var pc=el('pktCnt');
    if(pc){pc.style.display='inline';pc.textContent='#'+pktCount;}
    var d;try{d=JSON.parse(ev.data);}catch(e){return;}
    // [v6.6] Success modal
    if(d.status==='success'){openSuccessModal(d);return;}
    // [U1] GET_PID response
    if(d.type==='pid_values'){receivePidValues(d);return;}
    handleTelemetry(d);
  };
}
function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}
function sendCmd(v){send({t:'cmd',v:v});}

// ── [U1] BIDIRECTIONAL PID SYNC ──────────────────────────────────────
function fetchPidFromRobot(){
  if(!ws||ws.readyState!==1){alert('Not connected.');return;}
  ws.send('GET_PID:'+curPidMode);
  var m=el('pidMsg');if(m){m.textContent='Fetching from robot…';m.style.color='var(--cyan)';}
}
function receivePidValues(d){
  // d = {type:'pid_values', m:N, kP:x, kI:x, kD:x, base:x, top:x, low:x}
  var m=d.m||0;
  PID_DEFAULTS[m]={kP:d.kP,kI:d.kI,kD:d.kD,base:d.base,top:d.top,low:d.low};
  if(curPidMode===m) selPidMode(m);
  var msg=el('pidMsg');
  if(msg){msg.textContent='M'+m+' loaded: Kp='+d.kP.toFixed(3)+' Kd='+d.kD.toFixed(3);
          msg.style.color='var(--green)';}
  addLog(['[U1] PID M'+m+' fetched from robot: Kp='+d.kP.toFixed(3)+' Kd='+d.kD.toFixed(3)]);
}

// ── PID Apply ────────────────────────────────────────────────────────
function applyPID(){
  var m=curPidMode;
  var obj={t:'pid',m:m,
    kP:parseFloat(el('pKp').value)||0,
    kI:parseFloat(el('pKi').value)||0,
    kD:parseFloat(el('pKd').value)||0,
    base:parseInt(el('pBase').value)||0,
    top:parseInt(el('pTop').value)||0,
    low:parseInt(el('pLow').value)||0
  };
  send(obj);
  // Update local cache
  PID_DEFAULTS[m]={kP:obj.kP,kI:obj.kI,kD:obj.kD,base:obj.base,top:obj.top,low:obj.low};
  var msg=el('pidMsg');
  if(msg){msg.textContent='Applied M'+m+'!';msg.style.color='var(--green)';
          setTimeout(function(){if(msg)msg.textContent='';},3000);}
}

// ── AutoTune ─────────────────────────────────────────────────────────
function startAt(){
  if(!ws||ws.readyState!==1)return;
  ws.send('START_AUTOTUNE:'+curAtMode);
  var sb=el('atStartBtn'),stb=el('atStopBtn');
  if(sb)sb.disabled=true;if(stb)stb.disabled=false;
  el('atResultCard').style.display='none';
}
function stopAt(){
  if(!ws||ws.readyState!==1)return;
  ws.send('STOP_AUTOTUNE');
  var sb=el('atStartBtn'),stb=el('atStopBtn');
  if(sb)sb.disabled=false;if(stb)stb.disabled=true;
}
// [FIX-PERMODE] Per-mode ZN Kp Start/Step — sends mode-specific command to firmware
function applyAtKpParam(m){
  if(!ws||ws.readyState!==1)return;
  var ks =parseFloat(el('atKpStart'+m).value)||0.020;
  var kst=parseFloat(el('atKpStep'+m).value) ||0.020;
  ws.send('SET_AT_KP_START:'+m+':'+ks.toFixed(4));
  ws.send('SET_AT_KP_STEP:'+m+':'+kst.toFixed(4));
  addLog(['[FIX] AT8 M'+m+': KpStart='+ks.toFixed(4)+' KpStep='+kst.toFixed(4)+' sent to robot']);
}
// [FIX-RESET] Per-mode AT8 checkpoint reset with cold-start confirmation
// mode: 0/1/2 = specific mode, -1 = all modes
function manualBrake(){
  if(!ws||ws.readyState!==1){alert('Not connected');return;}
  ws.send('MANUAL_BRAKE');
  var b=el('atBrakeBtn');
  if(b){b.textContent='⏳ Braking…';b.disabled=true;setTimeout(function(){b.textContent='■■ BRAKE TEST (ABS)';b.disabled=false;},1800);}
}
function resetAt8Checkpoint(m){
  var lbl=(m<0)?'ALL MODES':'M'+m;
  if(!confirm('Reset AT8 checkpoint for '+lbl+'?\n\n'
    +'This will:\n'
    +'  • Clear the NVS checkpoint for '+lbl+'\n'
    +'  • Force COLD START from TWIDDLE_KP_INIT_STEP\n'
    +'  • Ignore any previous tuning values\n\n'
    +'Cannot be undone. Continue?')) return;
  if(!ws||ws.readyState!==1){alert('Not connected to robot.');return;}
  var cmd=(m<0)?'AT8_RESET_NVS':'AT8_RESET_NVS:'+m;
  ws.send(cmd);
  addLog(['[FIX-RESET] AT8 checkpoint reset sent: '+lbl+' → next run COLD START']);
}
function pushAt8ToPid(){
  if(!ws||ws.readyState!==1){alert('Not connected.');return;}
  var kpS=parseFloat((el('at8KpS')||{}).textContent)||0;
  var kdS=parseFloat((el('at8KdS')||{}).textContent)||0;
  var kpT=parseFloat((el('at8KpT')||{}).textContent)||0;
  var kdT=parseFloat((el('at8KdT')||{}).textContent)||0;
  var spd=parseInt((el('at8Spd')||{}).textContent)||0;
  if(kpS===0&&kpT===0){alert('No AT8 values yet. Start AutoTune first.');return;}
  var top=Math.round(spd+Math.min(Math.max(Math.round(spd/4),20),50));
  var low=Math.max(Math.round(spd*2/3),15);
  var m=curAtMode;
  if(m===0){
    ws.send(JSON.stringify({t:'pid',m:0,kP:kpS,kI:0,kD:kdS,base:spd,top:top,low:low}));
    ws.send(JSON.stringify({t:'pid',m:1,kP:kpT,kI:0,kD:kdT,base:spd,top:top,low:low}));
    ws.send(JSON.stringify({t:'pid',m:2,kP:kpT,kI:0,kD:kdT,base:spd,top:top,low:low}));
    addLog(['[PUSH-AT8] M0: Straight\u2192M0 Kp='+kpS.toFixed(4)+' Kd='+kdS.toFixed(4)+' | Turn\u2192M1+M2 Kp='+kpT.toFixed(4)+' Kd='+kdT.toFixed(4)+' Spd='+spd]);
  } else if(m===1){
    ws.send(JSON.stringify({t:'pid',m:1,kP:kpT,kI:0,kD:kdT,base:spd,top:top,low:low}));
    addLog(['[PUSH-AT8] M1: Turn\u2192M1 Kp='+kpT.toFixed(4)+' Kd='+kdT.toFixed(4)+' Spd='+spd]);
  } else if(m===2){
    ws.send(JSON.stringify({t:'pid',m:2,kP:kpT,kI:0,kD:kdT,base:spd,top:top,low:low}));
    addLog(['[PUSH-AT8] M2: Turn\u2192M2 Kp='+kpT.toFixed(4)+' Kd='+kdT.toFixed(4)+' Spd='+spd]);
  }
  var b=el('at8PushBtn');
  if(b){b.textContent='\u2713 Pushed & Saved!';setTimeout(function(){b.textContent='\u2b06 Push AT8\u2192PID (Save)';},2500);}
}
function openSuccessModal(d){
  // Guard: if all key values are zero/missing, data arrived before C++ set them — ignore
  var ku=parseFloat(d.ku)||0, tu=parseFloat(d.tu)||0;
  var kp=parseFloat(d.kp)||0, kd=parseFloat(d.kd)||0;
  if(kp===0 && kd===0 && ku===0){
    console.warn('AT modal: received zero values — ignoring stale broadcast.');
    return;
  }
  lastAtSuccess=d; lastAtKp=kp; lastAtKd=kd;
  // [BUGFIX-5] AT8 Twiddle uses crossings sentinel=8; ZN uses 0–8 real crossings.
  // Ku/Tu are 0 for AT8 (never set by firmware) — hide those rows for AT8.
  var isTwiddle=(d.crossings===8);
  var rowKu=el('mRowKu'),rowTu=el('mRowTu');
  var desc=el('mResDesc'),kpLbl=el('mKpLabel'),kdLbl=el('mKdLabel');
  if(isTwiddle){
    if(rowKu)rowKu.style.display='none';
    if(rowTu)rowTu.style.display='none';
    if(desc)desc.textContent='AT8 Twiddle results ready to apply.';
    if(kpLbl)kpLbl.textContent='Twiddle Kp';
    if(kdLbl)kdLbl.textContent='Twiddle Kd';
  } else {
    if(rowKu)rowKu.style.display='';
    if(rowTu)rowTu.style.display='';
    if(desc)desc.textContent='ZN results ready to apply.';
    if(kpLbl)kpLbl.textContent='ZN Kp';
    if(kdLbl)kdLbl.textContent='ZN Kd';
    txt('mResKu',ku.toFixed(4));
    txt('mResTu',tu.toFixed(4)+' s');
  }
  txt('mResMode','M'+d.mode+' '+(AT_MODE_SHORT[d.mode]||''));
  txt('mResKp',  kp.toFixed(4));
  txt('mResKd',  kd.toFixed(4));
  var m=el('atModal');if(m)m.classList.add('open');
  // [U1] Also push values into PID tuner fields
  var md=d.mode||0;
  PID_DEFAULTS[md].kP=kp;
  PID_DEFAULTS[md].kD=kd;
  if(curPidMode===md) selPidMode(md);
  var nt=el('atTabNotif');
  if(nt){nt.style.display='block';clearTimeout(nt._t);nt._t=setTimeout(function(){if(nt)nt.style.display='none';},15000);}
  var sb=el('atStartBtn'),stb=el('atStopBtn');
  if(sb)sb.disabled=false;if(stb)stb.disabled=true;
}
function closeModal(){var m=el('atModal');if(m)m.classList.remove('open');}
function modalApplySave(){
  if(!lastAtSuccess)return;
  var d=lastAtSuccess;
  if(ws&&ws.readyState===1) ws.send('SAVE_TUNED_PID:'+d.mode+':'+d.kp+':'+d.kd);
  closeModal();
  var tn=el('pidTabNotif');
  if(tn){tn.style.display='block';clearTimeout(tn._t);tn._t=setTimeout(function(){if(tn)tn.style.display='none';},10000);}
}

// ── D-pad drive ───────────────────────────────────────────────────────
function dpad(dir){
  var spd=parseInt(el('drvSpd').value)||100;
  send({t:'drive',dir:dir,spd:spd});
  ['fwd','left','right','back'].forEach(function(d){
    var b=el('db_'+d);if(b)b.classList.toggle('pr',d===dir);
  });
}

// ── Log ───────────────────────────────────────────────────────────────
function addLog(lines){
  var logEl=el('log');if(!logEl)return;
  lines.forEach(function(l){
    var div=document.createElement('div');
    div.className='ll'+(l.indexOf('WARN')>=0?' warn':l.indexOf('ERR')>=0||l.indexOf('FAIL')>=0?' err':
                       l.indexOf('OK')>=0||l.indexOf('PASS')>=0?' ok':
                       l.indexOf('JUNC')>=0||l.indexOf('NODE')>=0||l.indexOf('U5')>=0||l.indexOf('U6')>=0?' maze':
                       l.indexOf('CAL')>=0?' comp':
                       l.indexOf('HLA')>=0||l.indexOf('U4')>=0?' hla':
l.indexOf('AT:')>=0||l.indexOf('AT8')>=0?' at':'');
    div.textContent=l;
    logEl.appendChild(div);
    while(logEl.childNodes.length>500)logEl.removeChild(logEl.firstChild);
  });
  if(logAutoScroll)logEl.scrollTop=logEl.scrollHeight;
}

// ── Main telemetry handler ────────────────────────────────────────────
function handleTelemetry(d){
  // State pill
  var SP=['IDLE','CALIBRATING','RUNNING','STOPPED','AUTO-TUNING'];
  var SC=['#5a6490','#ffca28','#00e676','#ff4444','#b060ff'];
  if(d.state!=null){
    var sp=el('state-pill');
    if(sp){sp.textContent=SP[d.state]||('ST#'+d.state);sp.style.color=SC[d.state]||'#fff';}
    var arb=el('atRunBadge');if(arb)arb.style.display=(d.state===4)?'inline-block':'none';
    if(d.state!==4){
      var sb=el('atStartBtn'),stb=el('atStopBtn');
      if(sb&&stb&&!stb.disabled){sb.disabled=false;stb.disabled=true;}
    }
  }
  // Cal/IR
  if(d.cal!=null){var cv=el('calVal');if(cv){cv.textContent=d.cal?'OK':'Not calibrated';cv.style.color=d.cal?'#00e676':'#ff4444';}}
  if(d.ir!=null){var iv=el('irVal');if(iv){iv.textContent=d.ir?'ON':'OFF';iv.style.color=d.ir?'#00e676':'#5a6490';}var ie=el('irEn');if(ie)ie.checked=!!d.ir;}
  // Position / Error
  if(d.pos!=null){txt('mv_pos',d.pos);var pc2=el('posCur');if(pc2)pc2.style.left=(d.pos/7000*100).toFixed(2)+'%';}
  if(d.err!=null){
    var absE=Math.abs(+d.err);
    var mv=el('mv_err');if(mv){mv.textContent=d.err;mv.style.color=absE<200?'#00e676':absE<1200?'#ffca28':'#ff4444';}
    var pct=Math.min(Math.max((+d.err+3500)/7000*100,0),100);
    var fill=el('egFill');
    if(fill){var col=absE<200?'#00e676':absE<1200?'#ffca28':'#ff4444';fill.style.background=col;
      if(+d.err>=0){fill.style.left='50%';fill.style.width=(pct-50).toFixed(1)+'%';}
      else         {fill.style.left=pct.toFixed(1)+'%';fill.style.width=(50-pct).toFixed(1)+'%';}}
  }
  if(d.lpwm!=null){var lv=el('mv_lp');if(lv){lv.textContent=d.lpwm;lv.style.color=d.lpwm<0?'#ff4444':'#00e676';}}
  if(d.rpwm!=null){var rv=el('mv_rp');if(rv){rv.textContent=d.rpwm;rv.style.color=d.rpwm<0?'#ff4444':'#00e676';}}
  if(d.pm!=null){curTelPm=d.pm;var pb=el('pmBadge');if(pb){pb.textContent='PID: '+(MODES[d.pm]||d.pm);pb.style.color=MCOL[d.pm]||'#fff';}updateActiveBadge();}
  // QTR bars
  if(d.qtr&&Array.isArray(d.qtr)){
    for(var i=0;i<8;i++){
      var v=d.qtr[i];var f=el('qbf'+i);
      if(f){f.style.height=(v/10).toFixed(0)+'%';f.style.opacity=(v>80)?'1':'0.35';
            var p2=Math.min(v/1000,1);f.style.background=p2<0.5?'#008800':p2<0.85?'#00cc00':'#00e676';}
      txt('qbv'+i,v);
    }
  }
  // Look-ahead
  if(d.obs!=null){for(var i=0;i<4;i++){var oc=el('lac'+i);if(oc)oc.className='la-cell'+((d.obs&(1<<i))?' hit':'');}}
  if(d.obsRaw!=null) txt('obsRawTxt','raw=0x'+Number(d.obsRaw).toString(16).padStart(2,'0').toUpperCase());
  if(d.la!=null){
    var idx=d.la;
    function setB(e,lbl,ci,bi,bci){if(!e)return;e.textContent=lbl;e.style.color=ci;e.style.background=bi;e.style.borderColor=bci;}
    setB(el('laBadge'),      'LA: '+(LA_LABELS[idx]||idx), LA_COLS[idx]||'#5a6490',LA_BG[idx]||'#0d0d1c',LA_BC[idx]||'#1e1e38');
    setB(el('laStateBadge'), LA_LABELS[idx]||('LA:'+idx),  LA_COLS[idx]||'#5a6490',LA_BG[idx]||'#0d0d1c',LA_BC[idx]||'#1e1e38');
  }
  if(d.ladir!=null){var DMAP={'-1':'LEFT','0':'NONE','1':'RIGHT'},DCOL={'-1':'#00e5ff','0':'#5a6490','1':'#00e5ff'};var key=String(d.ladir);var db2=el('laDirBadge');if(db2){db2.textContent='DIR: '+(DMAP[key]||'?');db2.style.color=DCOL[key]||'#5a6490';}}
  if(d.laHealthy!=null){var healthy=!!d.laHealthy;var bb=el('brkBadge');if(bb){bb.textContent=healthy?'\u2699\uFE0F HW(HLA)':'\u26A0\uFE0F MATH-BRK';bb.className='badge '+(healthy?'b':'r');}var lhb=el('laHealthBadge');if(lhb){lhb.textContent=healthy?'LA: HEALTHY':'LA: FAULT';lhb.className='badge '+(healthy?'g':'r');}}
  if(d.brakeSys!=null){var sv=el('brkSysVal');if(sv){var hw=(d.brakeSys==='HARDWARE (HLA)');sv.textContent=d.brakeSys;sv.style.color=hw?'#00e5ff':'#ff4444';}}
  // State badges
  if(d.lost!=null){var li=el('lostBadge');if(li)li.style.display=d.lost?'inline-block':'none';}
  if(d.search!=null){var si=el('searchBadge');if(si)si.style.display=d.search?'inline-block':'none';}
  if(d.nav!=null){var label=NAV_STATES[d.nav]||('NAV#'+d.nav);var col=NAV_COLS[d.nav]||'#5a6490';var bg=NAV_BG[d.nav]||'#0d0d1c';var bc=NAV_BC[d.nav]||'#1e1e38';var nb=el('navBadge');if(nb){nb.textContent=label;nb.style.color=col;nb.style.background=bg;nb.style.borderColor=bc;}}
  if(d.cross!=null){var ci2=el('crossBadge');if(ci2)ci2.style.display=d.cross?'inline-block':'none';}
  if(d.lw!=null)txt('lwVal',d.lw);if(d.cr!=null)txt('crVal',d.cr);
  // [U7] Dummy-45 filter badge
  if(d.dummy45!=null){var d45=el('dummy45Badge');if(d45)d45.style.display=d.dummy45?'inline-block':'none';}
  // [U5] Node count
  if(d.nodes!=null){var nb2=el('nodesBadge');if(nb2)nb2.textContent='NODES: '+d.nodes;txt('nodeCountVal',d.nodes);}

  // AutoTune telemetry
  if(d.at_mode!=null){
    var mT2=d.at_mode;
    var lbl2=el('atActiveModeLabel');
    if(lbl2)lbl2.textContent='M'+mT2+' '+(AT_MODE_SHORT[mT2]||'M'+mT2);
    // Show/hide sweep rows
    var isSweep=(mT2===3);
    ['sweepSpdRow','sweepBestRow','sweepRoundRow'].forEach(function(id){
      var e=el(id);if(e)e.style.display=isSweep?'flex':'none';
    });
  }
  if(d.at_ramp!=null){var _atm=d.at8_tmod!=null?d.at8_tmod:curAtMode;var _sldr=el('atBaseSpd'+_atm);var targetPwm=_sldr?_sldr.value:'?';var rl=el('atRampLabel');if(rl)rl.textContent=d.at_ramp+' \u2192 '+targetPwm+' PWM';}
  if(d.at_phase!=null){
    var ph=d.at_phase;
    var atS=el('atStatus'),atPL=el('atPhaseLabel'),atPF=el('atProgFill'),atSB2=el('atSafetyBadge'),atRC2=el('atResultCard'),atTN=el('atTabNotif');
    if(ph===-1){if(atS){atS.textContent='IDLE';atS.style.color='#5a6490';}if(atPL)atPL.textContent='\u2014';if(atPF){atPF.style.width='0%';atPF.style.background='var(--purple)';}if(atSB2)atSB2.style.display='none';}
    else if(ph>=0&&ph<=3){
      var pLabel=AT_PHASES[ph]||'?',pCol=AT_PCOLS[ph]||'#5a6490';
      if(atS){atS.textContent=pLabel;atS.style.color=pCol;}if(atPL){atPL.textContent=pLabel;atPL.style.color=pCol;}
      var kpCur=d.at_kp_cur||0.05;var progPct=Math.min((kpCur-0.05)/(3.00-0.05)*100,100).toFixed(1);
      if(atPF){atPF.style.width=progPct+'%';atPF.style.background=pCol;}
      if(ph===2){if(atRC2)atRC2.style.display='block';if(atSB2)atSB2.style.display='none';
        var ku=d.at_ku||0,tu=d.at_tu||0,rkp=d.at_res_kp||0,rkd=d.at_res_kd||0,mT=d.at_mode||0;
        lastAtKp=rkp;lastAtKd=rkd;if(!lastAtSuccess)lastAtSuccess={mode:mT,ku:ku,tu:tu,kp:rkp,kd:rkd};
        txt('atResMode','M'+mT+' '+(AT_MODE_SHORT[mT]||''));txt('atKu',ku.toFixed(3));txt('atTu',tu.toFixed(4)+' s');txt('atResKp',rkp.toFixed(3));txt('atResKd',rkd.toFixed(3));
        var atTabEl=document.querySelectorAll('.tab-btn')[3];
        if(atTabEl&&!atTabEl.classList.contains('active')){if(atTN){atTN.style.display='block';clearTimeout(atTN._t);atTN._t=setTimeout(function(){if(atTN)atTN.style.display='none';},15000);}}
        var sb2=el('atStartBtn'),stb2=el('atStopBtn');if(sb2)sb2.disabled=false;if(stb2)stb2.disabled=true;
      } else if(ph===3){if(atSB2)atSB2.style.display='inline-block';var sb3=el('atStartBtn'),stb3=el('atStopBtn');if(sb3)sb3.disabled=false;if(stb3)stb3.disabled=true;if(atS){atS.textContent='ABORTED';atS.style.color='#ff4444';}}
    }
    if(d.at_kp_cur!=null)   txt('atKpCur',   (+d.at_kp_cur).toFixed(3));
    if(d.at_crossings!=null) txt('atCrossings',d.at_crossings+' / 8');
  }
  if(d.log&&Array.isArray(d.log)&&d.log.length)addLog(d.log);

  // ── [AT8] Dual-profile adaptive tuner status display ─────────────────
  var AT8_PH=['INIT','STR-TUNE','STR-LOCKED','TRN-TUNE','SPD-STEP','COMPLETE','ABORTED'];
  var AT8_PC=['#b060ff','#00a8ff','#00e676','#ffca28','#ff7722','#00e676','#ff4444'];
  var AT8_TER=['?','STRAIGHT','TURN-90'];
  var AT8_TC=['#5a6490','#00e5ff','#ffca28'];

  if(d.at8_ph!=null){
    var ph8=d.at8_ph;
    var a8Ph=el('at8Phase'); if(a8Ph){a8Ph.textContent=AT8_PH[ph8]||('PH'+ph8);a8Ph.style.color=AT8_PC[ph8]||'#fff';}
  }
  if(d.at8_ter!=null){
    var t8=el('at8Terrain');if(t8){t8.textContent=AT8_TER[d.at8_ter]||'?';t8.style.color=AT8_TC[d.at8_ter]||'#fff';}
  }
// [BUG3-FIX] Explicit DOM updates — direct .innerHTML guards against
  // silent failures in txt() when elements are temporarily detached.
  if(d.at8_spd!=null){var eSpd=el('at8Spd');  if(eSpd) eSpd.innerHTML=(+d.at8_spd)+' PWM';}
  if(d.at8_kpS!=null){var eKpS=el('at8KpS');  if(eKpS) eKpS.innerHTML=(+d.at8_kpS).toFixed(4);}
  if(d.at8_kdS!=null){var eKdS=el('at8KdS');  if(eKdS) eKdS.innerHTML=(+d.at8_kdS).toFixed(4);}
  if(d.at8_kpT!=null){var eKpT=el('at8KpT');  if(eKpT) eKpT.innerHTML=(+d.at8_kpT).toFixed(4);}
  if(d.at8_kdT!=null){var eKdT=el('at8KdT');  if(eKdT) eKdT.innerHTML=(+d.at8_kdT).toFixed(4);}
  if(d.at8_sLk!=null){var sl=el('at8SLock');if(sl){sl.textContent=d.at8_sLk?'LOCKED':'unlocked';sl.style.color=d.at8_sLk?'#00e676':'#ffca28';}}
  if(d.at8_fit!=null){
    var fv=d.at8_fit;var fs=el('at8Fitness');
    if(fs){fs.textContent=fv>1e8?'---':(+fv).toFixed(0);fs.style.color=fv<5000?'#00e676':fv<20000?'#ffca28':'#ff4444';}
  }
if(d.at8_dp!=null){
    var dp=el('at8DpSum');if(dp){dp.textContent=(+d.at8_dp).toFixed(5);dp.style.color=d.at8_dp<0.01?'#00e676':'#ffca28';}
  }
  // ── Realtime active Kp/Kd (terrain-switching, updates every 100 ms) ──
  if(d.at8_actKp!=null){var eAKp=el('at8ActKp');if(eAKp)eAKp.innerHTML=(+d.at8_actKp).toFixed(4);}
  if(d.at8_actKd!=null){var eAKd=el('at8ActKd');if(eAKd)eAKd.innerHTML=(+d.at8_actKd).toFixed(4);}
  // ── Live IAE accumulation counter ─────────────────────────────────────
  if(d.at8_cfit!=null){
    var eCF=el('at8CurFit');
    if(eCF){
      var cfv=+d.at8_cfit;
      eCF.textContent=cfv.toFixed(0);
      eCF.style.color=cfv<5000?'#00e676':cfv<20000?'#ffca28':'#ff4444';
    }
  }
  // ── Tuning mode label ─────────────────────────────────────────────────
  if(d.at8_tmod!=null){
    var eTM=el('at8TuneModeLbl');
    if(eTM) eTM.textContent='M'+d.at8_tmod+' '+(AT_MODE_SHORT[d.at8_tmod]||'');
  }
}
selTab(0); selAtMode(0); selPidMode(0); connect();
</script>
</body>
</html>
)HTMLEOF";

// ══════════════════════════════════════════════════════════════════════
//  WEBSOCKET + HTTP SERVER  (Core 0, non-blocking)
//  [U1] GET_PID command → bidirectional PID sync
//  [U2] SET_BASE_SPEED now accepts 50–255
// ══════════════════════════════════════════════════════════════════════
AsyncWebServer gServer(80);
AsyncWebSocket gWs("/ws");

// ── Build full telemetry JSON ─────────────────────────────────────────
static void buildTelJson(StaticJsonDocument<1536>& doc) {
  portENTER_CRITICAL(&gTelMux);
  auto t = gTel;
  portEXIT_CRITICAL(&gTelMux);

  portENTER_CRITICAL(&gAtResMux);
  auto ar = gAtResult;
  portEXIT_CRITICAL(&gAtResMux);

  portENTER_CRITICAL(&gAtMux);
  int rampedBase = gAtRampedBasePub;
  portEXIT_CRITICAL(&gAtMux);

  doc["state"] = (int)getState();
  doc["cal"] = (bool)gCalibOK;
  doc["ir"] = (bool)gQtrIrOn;
  doc["pos"] = t.pos;
  doc["err"] = t.err;
  doc["lpwm"] = t.lPWM;
  doc["rpwm"] = t.rPWM;
  doc["pm"] = t.pidMode;
  doc["lost"] = t.lineLost;
  doc["search"] = t.searchMode;
  doc["nav"] = t.navState;
  doc["cross"] = t.atCross;
  doc["obs"] = t.obs;
  doc["obsRaw"] = t.obsRaw;
  doc["lw"] = t.lineWidthSensors;
  doc["cr"] = t.crossMinSensors;
  doc["la"] = t.laState;
  doc["ladir"] = t.laTurnHint;
  doc["laHealthy"] = t.laHealthy;
  doc["brakeSys"] = gActiveBrakingSystem;
  doc["dummy45"] = t.dummy45Active;  // [U7]
  doc["nodes"] = t.nodeCount;        // [U5]

  JsonArray qa = doc.createNestedArray("qtr");
  for (int i = 0; i < 8; i++) qa.add(t.qtr[i]);

  // AT telemetry
  doc["at_phase"] = (int)ar.phase;
  doc["at_kp_cur"] = ar.currentKp;
  doc["at_crossings"] = ar.zeroCrossings;
  doc["at_ku"] = ar.ku;
  doc["at_tu"] = ar.tu;
  doc["at_res_kp"] = ar.resultKp;

  // [AT8] Extended AT8 dual-profile status
  portENTER_CRITICAL(&gAt8Mux);
  auto a8 = gAt8Status;
  portEXIT_CRITICAL(&gAt8Mux);
  doc["at8_kpS"] = a8.kpS;
  doc["at8_kdS"] = a8.kdS;
  doc["at8_kpT"] = a8.kpT;
  doc["at8_kdT"] = a8.kdT;
  doc["at8_actKp"] = a8.activeKp;  // Kp currently driving motors (terrain-selected)
  doc["at8_actKd"] = a8.activeKd;  // Kd currently driving motors (terrain-selected)
  doc["at8_sLk"] = a8.sLocked;
  doc["at8_spd"] = a8.speed;
  doc["at8_fit"] = a8.fitness;
  doc["at8_cfit"] = a8.curFit;  // live IAE counter (resets each eval window)
  doc["at8_ph"] = (int)a8.phase;
  doc["at8_ter"] = (int)a8.terrain;
  doc["at8_dp"] = a8.dpSum;
  doc["at8_tmod"] = a8.tuneMode;  // M0 / M1 / M2 selected for this run
  doc["at_res_kd"] = ar.resultKd;
  doc["at_mode"] = ar.tuneMode;
  doc["at_ramp"] = rampedBase;
}

// ── Build PID-only JSON  [U1] ─────────────────────────────────────────
static void buildPidJson(StaticJsonDocument<256>& doc, int m) {
  portENTER_CRITICAL(&gPidMux);
  PIDParam p = gPID[m];
  portEXIT_CRITICAL(&gPidMux);
  doc["type"] = "pid_values";
  doc["m"] = m;
  doc["kP"] = p.kP;
  doc["kI"] = p.kI;
  doc["kD"] = p.kD;
  doc["base"] = p.base;
  doc["top"] = p.top;
  doc["low"] = p.low;
}

// ── Build AT success JSON ─────────────────────────────────────────────
static void buildAtSuccessJson(StaticJsonDocument<256>& doc) {
  portENTER_CRITICAL(&gAtResMux);
  auto ar = gAtResult;
  portEXIT_CRITICAL(&gAtResMux);
  doc["status"] = "success";
  doc["mode"] = (int)ar.tuneMode;
  doc["ku"] = ar.ku;
  doc["tu"] = ar.tu;
  doc["kp"] = ar.resultKp;
  doc["kd"] = ar.resultKd;
  doc["crossings"] = ar.zeroCrossings;  // AT8 Twiddle sentinel = 8; ZN = 0-8 real
}

// ── WebSocket event handler ───────────────────────────────────────────
void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c,
               AwsEventType t, void* arg, uint8_t* data, size_t len) {
  if (t == WS_EVT_CONNECT) {
    gWsClients++;
    gWsEverConnected = true;
    gWsLastSeenMs = millis();
    webLog("WS: Client #%u connected (%d total)", c->id(), gWsClients);

    // Send full telemetry on connect
    StaticJsonDocument<1536> doc;
    buildTelJson(doc);
    String out;
    serializeJson(doc, out);
    c->text(out);

    // [FIX] Send ALL 3 PID profiles on connect so the UI reflects actual NVS values.
    // Previously only M0 was sent; clicking M1/M2 tab → Apply would broadcast stale
    // JS defaults back to the ESP32, silently overwriting NVS-tuned profiles.
    for (int m = 0; m < 3; m++) {
      StaticJsonDocument<256> pdoc;
      buildPidJson(pdoc, m);
      String pout;
      serializeJson(pdoc, pout);
      c->text(pout);
    }
  } else if (t == WS_EVT_DISCONNECT) {
    if (gWsClients > 0) gWsClients--;
    webLog("WS: Client #%u disconnected (%d remain)", c->id(), gWsClients);
  } else if (t == WS_EVT_DATA) {
    gWsLastSeenMs = millis();
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len) return;

    String msg = "";
    if (info->opcode == WS_TEXT) {
      msg.reserve(len + 1);
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
    } else return;

    // ── [U1] GET_PID bidirectional command ────────────────────────
    if (msg.startsWith("GET_PID:")) {
      int m = constrain(msg.substring(8).toInt(), 0, 2);
      StaticJsonDocument<256> pdoc;
      buildPidJson(pdoc, m);
      String pout;
      serializeJson(pdoc, pout);
      c->text(pout);
      webLog("[U1] GET_PID M%d -> Kp=%.3f Kd=%.3f sent to client", m, gPID[m].kP, gPID[m].kD);
      return;
    }

    // ── SET_BASE_SPEED  [U2] accepts 50–255 ──────────────────────
    // ── SET_BASE_SPEED  [U2] accepts 50–255 ──────────────────────
if (msg.startsWith("SET_BASE_SPEED:")) {
      int spd = constrain(msg.substring(15).toInt(), AT_BASE_SPEED_MIN, AT_BASE_SPEED_MAX);
      portENTER_CRITICAL(&gAtMux);
      gAtBaseSpeed = spd;
      portEXIT_CRITICAL(&gAtMux);
      webLog("AT: Base speed set to %d PWM.", spd);
      return;
    }

    // ── [FIX] SET_MODE_SPEED:<mode>:<pwm> — per-mode AT starting speed ─────
    if (msg.startsWith("SET_MODE_SPEED:")) {
      int col = msg.indexOf(':', 15);
      if (col > 0) {
        int m   = constrain(msg.substring(15, col).toInt(), 0, 2);
        int spd = constrain(msg.substring(col + 1).toInt(), 50, 255);
        portENTER_CRITICAL(&gAtMux);
        gAtModeSpeed[m] = spd;
        portEXIT_CRITICAL(&gAtMux);
        webLog("[FIX] AT: Mode M%d starting speed set to %d PWM.", m, spd);
      }
      return;
    }

    // ── [FIX-PERMODE] SET_AT_KP_START:<mode>:<val> — per-mode ZN/Twiddle Kp seed ──
    if (msg.startsWith("SET_AT_KP_START:")) {
      int c = msg.indexOf(':', 16);
      if (c > 0) {
        int m  = constrain(msg.substring(16, c).toInt(), 0, 2);
        float v = constrain(msg.substring(c + 1).toFloat(), 0.001f, 1.0f);
        gAtKpStart[m] = v;
        webLog("[FIX] AT M%d: KP_START=%.4f (warm-start seed for Twiddle Kp)", m, v);
      } else {
        // Legacy single-value fallback (all modes)
        float v = constrain(msg.substring(16).toFloat(), 0.001f, 1.0f);
        gAtKpStart[0] = gAtKpStart[1] = gAtKpStart[2] = v;
        webLog("[FIX] AT KP_START all modes=%.4f", v);
      }
      return;
    }

    // ── [FIX-PERMODE] SET_AT_KP_STEP:<mode>:<val> — per-mode Twiddle dpKp init ───
    if (msg.startsWith("SET_AT_KP_STEP:")) {
      int c = msg.indexOf(':', 15);
      if (c > 0) {
        int m  = constrain(msg.substring(15, c).toInt(), 0, 2);
        float v = constrain(msg.substring(c + 1).toFloat(), 0.001f, 0.5f);
        gAtKpStep[m] = v;
        webLog("[FIX] AT M%d: KP_STEP=%.4f (Twiddle dpKp init step)", m, v);
      } else {
        // Legacy single-value fallback (all modes)
        float v = constrain(msg.substring(15).toFloat(), 0.001f, 0.5f);
        gAtKpStep[0] = gAtKpStep[1] = gAtKpStep[2] = v;
        webLog("[FIX] AT KP_STEP all modes=%.4f", v);
      }
      return;
    }

    // ── START_AUTOTUNE ────────────────────────────────────────────
    if (msg.startsWith("START_AUTOTUNE:")) {
      int mode = constrain(msg.substring(15).toInt(), 0, 3);  // [BUG1-FIX] allow Mode 3 (Corner Sweep)
      portENTER_CRITICAL(&gAtMux);
      gAtTuneMode = mode;
      gAtStartReq = true;
      portEXIT_CRITICAL(&gAtMux);
      setState(State::AUTO_TUNING);
      webLog("AT: Start request: mode=%d base=%d.", mode, gAtBaseSpeed);
      return;
    }

    // ── STOP_AUTOTUNE ─────────────────────────────────────────────
    if (msg == "STOP_AUTOTUNE") {
      portENTER_CRITICAL(&gAtMux);
      gAtStopReq = true;
      portEXIT_CRITICAL(&gAtMux);
      webLog("AT: Stop request received.");
      return;
    }

    // ── [AT8-1] AT8_RESET_NVS — wipe checkpoint, force fresh start ──
    // Format: "AT8_RESET_NVS"       → clear ALL modes
    //         "AT8_RESET_NVS:<m>"   → clear only mode 0/1/2
    if (msg.startsWith("AT8_RESET_NVS")) {
      int modeIdx = -1;
      if (msg.length() > 13 && msg.charAt(13) == ':') {
        modeIdx = constrain(msg.substring(14).toInt(), 0, 2);
      }
      at8NvsClear(modeIdx);
      if (modeIdx < 0)
        webLog("AT8: NVS checkpoint CLEARED (ALL modes). Next start: fresh run.");
      else
        webLog("AT8: NVS checkpoint CLEARED for M%d. Next run: fresh start.", modeIdx);
      return;
    }

    // ── [ABS] MANUAL_BRAKE — trigger ABS braking from Web UI ─────────
    // Works in RUNNING or AUTO_TUNING state.
    // Useful for manual testing of the braking system during tuning.
    if (msg == "MANUAL_BRAKE") {
      State curSt = getState();
      if (curSt == State::RUNNING || curSt == State::AUTO_TUNING) {
        portENTER_CRITICAL(&gManualBrakeMux);
        gManualBrakeReq = true;
        portEXIT_CRITICAL(&gManualBrakeMux);
        webLog("[ABS-MANUAL] Brake request received from Web UI. state=%d",
               (int)curSt);
      } else {
        webLog("[ABS-MANUAL] Brake request ignored — robot not running (state=%d).",
               (int)curSt);
      }
      return;
    }

    // ── SAVE_TUNED_PID  (from modal Apply & Save) ─────────────────
    if (msg.startsWith("SAVE_TUNED_PID:")) {
      // Format: SAVE_TUNED_PID:<mode>:<kp>:<kd>
      int colon1 = msg.indexOf(':', 15);
      int colon2 = msg.indexOf(':', colon1 + 1);
      if (colon1 > 0 && colon2 > 0) {
        int modeS = constrain(msg.substring(15, colon1).toInt(), 0, 2);
        float kpS = msg.substring(colon1 + 1, colon2).toFloat();
        float kdS = msg.substring(colon2 + 1).toFloat();
        portENTER_CRITICAL(&gPidMux);
        gPID[modeS].kP = kpS;
        gPID[modeS].kD = kdS;
        portEXIT_CRITICAL(&gPidMux);
        static const char* kpKeys[3] = { "m0kp", "m1kp", "m2kp" };
        static const char* kdKeys[3] = { "m0kd", "m1kd", "m2kd" };
        gPrefs.putFloat(kpKeys[modeS], kpS);
        gPrefs.putFloat(kdKeys[modeS], kdS);
        webLog("WS: SAVE_TUNED_PID M%d Kp=%.4f Kd=%.4f saved to NVS.", modeS, kpS, kdS);
      }
      return;
    }

    // ── JSON message handling ─────────────────────────────────────
    StaticJsonDocument<256> jDoc;
    if (deserializeJson(jDoc, msg) != DeserializationError::Ok) return;

    const char* tp = jDoc["t"] | "";

    // cmd
    if (strcmp(tp, "cmd") == 0) {
      const char* v = jDoc["v"] | "";
      if (strcmp(v, "start") == 0) {
        if (!gCalibOK) {
          webLog("CMD: start blocked — not calibrated.");
          c->text("{\"err\":\"not calibrated\"}");
          return;
        }
        setState(State::RUNNING);
        webLog("CMD: START.");
      } else if (strcmp(v, "stop") == 0) {
        setState(State::STOPPED);
        stopAll();
        webLog("CMD: STOP.");
      } else if (strcmp(v, "calibrate") == 0) {
        if (getState() == State::RUNNING) {
          webLog("CMD: calibrate blocked — running.");
          return;
        }
        setState(State::CALIBRATING);
        xTaskCreatePinnedToCore(taskCalibrate, "calib", 4096, NULL, 5, NULL, 0);
        webLog("CMD: CALIBRATE started.");
      }
    }

    // pid  [U1] full bidirectional set
    else if (strcmp(tp, "pid") == 0) {
      int m = constrain((int)(jDoc["m"] | 0), 0, 2);
      portENTER_CRITICAL(&gPidMux);
      gPID[m].kP = jDoc["kP"] | gPID[m].kP;
      gPID[m].kI = jDoc["kI"] | gPID[m].kI;
      gPID[m].kD = jDoc["kD"] | gPID[m].kD;
      gPID[m].base = jDoc["base"] | gPID[m].base;
      gPID[m].top = jDoc["top"] | gPID[m].top;
      gPID[m].low = jDoc["low"] | gPID[m].low;
      portEXIT_CRITICAL(&gPidMux);
      // Persist all six params to NVS
      static const char* kpK[3] = { "m0kp", "m1kp", "m2kp" };
      static const char* kiK[3] = { "m0ki", "m1ki", "m2ki" };
      static const char* kdK[3] = { "m0kd", "m1kd", "m2kd" };
      static const char* bsK[3] = { "m0bs", "m1bs", "m2bs" };
      static const char* tpK[3] = { "m0tp", "m1tp", "m2tp" };
      static const char* lwK[3] = { "m0lw", "m1lw", "m2lw" };
      gPrefs.putFloat(kpK[m], gPID[m].kP);
      gPrefs.putFloat(kiK[m], gPID[m].kI);
      gPrefs.putFloat(kdK[m], gPID[m].kD);
      gPrefs.putInt(bsK[m], gPID[m].base);
      gPrefs.putInt(tpK[m], gPID[m].top);
      gPrefs.putInt(lwK[m], gPID[m].low);
      webLog("[U1] PID M%d set: Kp=%.3f Ki=%.4f Kd=%.3f B=%d T=%d L=%d",
             m, gPID[m].kP, gPID[m].kI, gPID[m].kD,
             gPID[m].base, gPID[m].top, gPID[m].low);
    }

    // drive
    else if (strcmp(tp, "drive") == 0) {
      const char* dir = jDoc["dir"] | "stop";
      int spd = constrain((int)(jDoc["spd"] | 100), 0, 255);
      gManualDrive = true;
      if (strcmp(dir, "fwd") == 0) {
        driveL(spd);
        driveR(spd);
      } else if (strcmp(dir, "back") == 0) {
        driveL(-spd);
        driveR(-spd);
      } else if (strcmp(dir, "left") == 0) {
        driveL(-spd);
        driveR(spd);
      } else if (strcmp(dir, "right") == 0) {
        driveL(spd);
        driveR(-spd);
      } else {
        stopAll();
        gManualDrive = false;
      }
    }

    // obs toggle
    else if (strcmp(tp, "obs") == 0) {
      gObsEn = (bool)(jDoc["en"] | true);
      webLog("OBS: Look-ahead %s.", gObsEn ? "ENABLED" : "DISABLED");
    }

    // ir toggle
    else if (strcmp(tp, "ir") == 0) {
      bool en = (bool)(jDoc["en"] | true);
      digitalWrite(PIN_QTR_IR, en ? HIGH : LOW);
      gQtrIrOn = en;
      webLog("IR: QTR emitters %s.", en ? "ON" : "OFF");
    }
    // [FIX] keepalive ping — gWsLastSeenMs already refreshed by WS_EVT_DATA, no-op here
    else if (strcmp(tp, "ping") == 0) { /* keepalive received */
    }
  } else if (t == WS_EVT_PONG || t == WS_EVT_PING) {
    gWsLastSeenMs = millis();
  }
}

// ══════════════════════════════════════════════════════════════════════
//  WEB TASK  —  Core 0, Priority 3  (~10 Hz broadcast)
// ══════════════════════════════════════════════════════════════════════
void taskWeb(void*) {
  unsigned long lastBroadMs = 0;
  unsigned long lastLogMs = 0;

  while (true) {
    gWs.cleanupClients();
    unsigned long nowMs = millis();

    // WS failsafe
    if (gWsEverConnected && gWsClients > 0 && (nowMs - gWsLastSeenMs) > WS_FAILSAFE_MS) {
      // [FIX] NEVER cut motors during an autonomous competition run.
      // Failsafe only fires during manual drive (phone screen off, Wi-Fi drop, etc.)
      if (gManualDrive && getState() != State::RUNNING) {
        setState(State::STOPPED);
        stopAll();
        webLog("WS: Failsafe triggered — client silent >%d ms. STOPPED.", WS_FAILSAFE_MS);
      }
    }

    // ── [U1/U2] AT success broadcast — also pushes PID values ────────
    bool doSuccessBcast = false;
    portENTER_CRITICAL(&gAtBcastMux);
    doSuccessBcast = gAtSuccessBroadcastPending;
    gAtSuccessBroadcastPending = false;
    portEXIT_CRITICAL(&gAtBcastMux);

    if (doSuccessBcast && gWsClients > 0) {
      // 1) Read gAtResult FIRST (before any possible reset from control task)
      StaticJsonDocument<256> sdoc;
      buildAtSuccessJson(sdoc);  // snapshot under mutex inside

      // 2) Verify values are non-zero before sending (race-condition guard)
      float sentKp = sdoc["kp"] | 0.0f;
      float sentKu = sdoc["ku"] | 0.0f;
      if (sentKp > 0.0f || sentKu > 0.0f) {
        String sout;
        serializeJson(sdoc, sout);
        gWs.textAll(sout);

        // 3) [U1] Push updated PID values for all three modes to UI
        for (int m = 0; m < 3; m++) {
          StaticJsonDocument<256> pdoc;
          buildPidJson(pdoc, m);
          String pout;
          serializeJson(pdoc, pout);
          gWs.textAll(pout);
        }
        webLog("[U1] AT success broadcast sent (Kp=%.4f Kd=%.4f).",
               (float)(sdoc["kp"] | 0.0f), (float)(sdoc["kd"] | 0.0f));

        // 4) NOW safe to clear gAtResult (broadcast already delivered)
        portENTER_CRITICAL(&gAtResMux);
        gAtResult.phase = -1;  // mark as consumed; keep values for NVS/log
        portEXIT_CRITICAL(&gAtResMux);
      } else {
        // Values arrived as zero — re-queue for next 50 ms tick
        webLog("AT: WARN broadcast values=0, re-queuing...");
        portENTER_CRITICAL(&gAtBcastMux);
        gAtSuccessBroadcastPending = true;
        portEXIT_CRITICAL(&gAtBcastMux);
      }
    } else if (doSuccessBcast && gWsClients == 0) {
      // No clients connected — re-queue so it fires when one connects
      portENTER_CRITICAL(&gAtBcastMux);
      gAtSuccessBroadcastPending = true;
      portEXIT_CRITICAL(&gAtBcastMux);
    }

    // ── [U1] Pending PID fetch broadcast ─────────────────────────────
    bool doPidFetch = false;
    portENTER_CRITICAL(&gPidFetchMux);
    doPidFetch = gPidFetchPending;
    gPidFetchPending = false;
    portEXIT_CRITICAL(&gPidFetchMux);

    if (doPidFetch && gWsClients > 0) {
      for (int m = 0; m < 3; m++) {
        StaticJsonDocument<256> pdoc;
        buildPidJson(pdoc, m);
        String pout;
        serializeJson(pdoc, pout);
        gWs.textAll(pout);
      }
    }

    // ── Regular telemetry broadcast ~10 Hz ───────────────────────────
    if (gWsClients > 0 && (nowMs - lastBroadMs >= 100)) {
      lastBroadMs = nowMs;
      StaticJsonDocument<1536> doc;
      buildTelJson(doc);

      // Attach any pending log lines
      portENTER_CRITICAL(&gLog.mux);
      bool logDirty = gLog.dirty;
      gLog.dirty = false;
      portEXIT_CRITICAL(&gLog.mux);

      if (logDirty) {
        JsonArray logArr = doc.createNestedArray("log");
        int sent = 0;
        portENTER_CRITICAL(&gLog.mux);
        while (gLog.count > 0 && sent < LOG_CHUNK) {
          logArr.add((const char*)gLog.lines[gLog.tail]);
          gLog.tail = (gLog.tail + 1) % LOG_LINES;
          gLog.count--;
          sent++;
        }
        portEXIT_CRITICAL(&gLog.mux);
      }

      String out;
      serializeJson(doc, out);
      gWs.textAll(out);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ══════════════════════════════════════════════════════════════════════
//  NVS LOAD/SAVE HELPERS
// ══════════════════════════════════════════════════════════════════════
void nvsLoad() {
  static const char* kpK[3] = { "m0kp", "m1kp", "m2kp" };
  static const char* kiK[3] = { "m0ki", "m1ki", "m2ki" };
  static const char* kdK[3] = { "m0kd", "m1kd", "m2kd" };
  static const char* bsK[3] = { "m0bs", "m1bs", "m2bs" };
  static const char* tpK[3] = { "m0tp", "m1tp", "m2tp" };
  static const char* lwK[3] = { "m0lw", "m1lw", "m2lw" };
  for (int m = 0; m < 3; m++) {
    gPID[m].kP = gPrefs.getFloat(kpK[m], gPID[m].kP);
    gPID[m].kI = gPrefs.getFloat(kiK[m], gPID[m].kI);
    gPID[m].kD = gPrefs.getFloat(kdK[m], gPID[m].kD);
    gPID[m].base = gPrefs.getInt(bsK[m], gPID[m].base);
    gPID[m].top = gPrefs.getInt(tpK[m], gPID[m].top);
    gPID[m].low = gPrefs.getInt(lwK[m], gPID[m].low);
    Serial.printf("NVS: M%d Kp=%.3f Ki=%.4f Kd=%.3f B=%d T=%d L=%d\n",
                  m, gPID[m].kP, gPID[m].kI, gPID[m].kD,
                  gPID[m].base, gPID[m].top, gPID[m].low);
  }
}

// ══════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin init
  motorInit();
  pinMode(PIN_QTR_IR, OUTPUT);
  digitalWrite(PIN_QTR_IR, LOW);
  gQtrIrOn = false;
  pinMode(PIN_SR_PL, OUTPUT);
  digitalWrite(PIN_SR_PL, HIGH);
  pinMode(PIN_SR_CP, OUTPUT);
  digitalWrite(PIN_SR_CP, LOW);
  pinMode(PIN_SR_Q7, INPUT);

  // [AT8-5] Emergency brake: GPIO 34 is input-only on ESP32.
  // External 10 kΩ pull-up to 3.3 V is required (no internal pull-up on GPIO 34).
  // ISR fires on FALLING edge (button pressed = GND).
  pinMode(PIN_EBRAKE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_EBRAKE), isrEBrake, FALLING);
  Serial.printf("AT8-EBRAKE: ISR attached on GPIO %d (FALLING, active-LOW)\n", PIN_EBRAKE);

  // NVS — open default PID namespace
  gPrefs.begin("lfr", false);
  nvsLoad();

  // Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // HTTP routes
  gServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", WEB_HTML);
  });
  gServer.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  // WebSocket
  gWs.onEvent(onWsEvent);
  gServer.addHandler(&gWs);
  gServer.begin();

  // Core 1 — control loop (highest priority, time-critical)
  xTaskCreatePinnedToCore(taskControl, "ctrl", 8192, NULL, 10, NULL, 1);

  // Core 0 — web broadcast task
  xTaskCreatePinnedToCore(taskWeb, "web", 4096, NULL, 3, NULL, 0);

  webLog("LFR v8.0 (INTELLIGENT AT EDITION) ready.  SSID=%s  IP=192.168.4.1", AP_SSID);
  webLog("[U1-U10] All v7.0 upgrades retained.");
  webLog("[AT8-1] State persistence & resume: NVS namespace 'at8'");
  webLog("[AT8-2] Wobble RMS analysis: window=%d heavy_thr=%.0f micro_step=%.4f",
         WOBBLE_WINDOW, WOBBLE_HEAVY_RMS, WOBBLE_MICRO_STEP);
  webLog("[AT8-3] Dual-profile terrain SM: STR<%.0f TRN>%.0f lock_var<%.0f hits=%d",
         TERRAIN_STR_ERR_THR, TERRAIN_TRN_ERR_THR,
         STRAIGHT_LOCK_VAR_THR, STRAIGHT_LOCK_HITS);
  webLog("[AT8-4] Twiddle coord descent: spd_start=%d step=%d  Kp0=%.3f Kd0=%.3f",
         AT8_SPD_START, AT8_SPD_STEP, TWIDDLE_KP_INIT_STEP, TWIDDLE_KD_INIT_STEP);
  webLog("[FIX] Per-mode AT speeds M0=%d M1=%d M2=%d",
         gAtModeSpeed[0], gAtModeSpeed[1], gAtModeSpeed[2]);
  webLog("[FIX] Per-mode ZN KpStart M0=%.4f M1=%.4f M2=%.4f",
         gAtKpStart[0], gAtKpStart[1], gAtKpStart[2]);
  webLog("[FIX] Per-mode ZN KpStep  M0=%.4f M1=%.4f M2=%.4f",
         gAtKpStep[0], gAtKpStep[1], gAtKpStep[2]);
  webLog("[AT8-5] Emergency brake ISR on GPIO %d (active-LOW, ext. pull-up required)", PIN_EBRAKE);
}

// ══════════════════════════════════════════════════════════════════════
//  LOOP  (Core 0 Arduino loop — deliberately minimal)
//  All real-time work is in taskControl (Core 1) + taskWeb (Core 0).
// ══════════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
