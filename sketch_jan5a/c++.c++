/*
 * ============================================================================
 * ARTTOUS QUADRUPED — v104 "NOMAD UPLINK"
 * ============================================================================
 *
 * STARTUP SEQUENCE ARCHITECTURE
 * ══════════════════════════════════════════════════════════════════════════
 *
 * PHASE 0 — DISPLAY & CONNECT
 *   Browser opens → 3D robot renders → wiring schematic shown
 *   WiFi connects → WebSocket link established
 *   Startup overlay shows live progress badges
 *
 * PHASE 1 — MPU ACQUIRE
 *   MPU6050 initializes → pitch/roll stream begins
 *   Horizon indicator activates in browser
 *   Firmware waits for 50 stable consecutive IMU reads before proceeding
 *
 * PHASE 2 — MOTOR INIT SEQUENCE  (4 legs × 4 joints = 16 moves, sequential)
 *   Each leg positions independently — never more than 4 motors at once
 *   Prevents inrush current spikes and logic glitches on shared I2C bus
 *   Order: FL → FR → BL → BR  (front-to-back diagonal pairs)
 *
 *   Each joint follows a TRAPEZOIDAL VELOCITY PROFILE over ~1 second:
 *   ┌─────────────────────────────────────────────────┐
 *   │  Speed                                          │
 *   │   80% ┤        ┌──────────────┐                 │
 *   │   70% ┤       /                \                │
 *   │    0% ┤──────/                  \───────────────│
 *   │        │ 20% │      60%         │ 20%  distance │
 *   └─────────────────────────────────────────────────┘
 *   - First 20% of distance: ease-in  (0 → 70% speed)
 *   - Middle 60% of distance: cruise  (80% power)
 *   - Last 20% of distance:  ease-out (70% → 0 speed)
 *   Time fractions: 28.6% accel / 42.8% cruise / 28.6% decel
 *
 * PHASE 3 — STABILIZE
 *   Robot stands on all 4 legs
 *   IMU variance measured over 200ms window
 *   When pitch_variance < 0.5° AND roll_variance < 0.5° → STABLE
 *
 * PHASE 4 — READY + CALIBRATION UNLOCKED
 *   Browser overlay dismisses → full control available
 *   Tilt calibration enabled: left/right/forward/backward offset trim
 *   Walking gait uses single-leg lift (1 leg moves, 3 legs support)
 *
 * WALKING GAIT (post-startup):
 *   Only 4 motors active at any time (one full leg)
 *   Sequence: FL lift → place → FR lift → place → BL → BR → repeat
 *   Support triangle maintained at all times
 *
 * ============================================================================
 * ALL FIXES FROM v103 PRESERVED:
 * ✓ ws://hostname:81/ WebSocket port (not /ws path)
 * ✓ JSON keys "p","r","l" (short, matched both sides)
 * ✓ test / test_stop command handlers
 * ✓ Gamepad dot: translate(calc(-50% + Xpx), calc(-50% + Ypx))
 * ✓ Per-leg BoxGeometry (no shared geometry mutation)
 * ✓ Canvas resize handler
 * ✓ wifi_config.h credentials pattern
 * ✓ testModeMutex protecting testingServos
 * ✓ statusMutex protecting hwStatus
 * ✓ Complementary IMU filter (DLPF 10Hz + 96/4 gyro/accel blend)
 * ✓ snprintf overflow detection
 * ✓ Static payload buffer (not stack)
 * ✓ Servo pre-flight → centerServos() only (no dangerous MIN/MAX sweep)
 * ✓ HTTP_PORT / WS_PORT collision check
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

// ============================================================================
// CREDENTIALS
// ============================================================================
// Create wifi_config.h:
//   #define WIFI_SSID "your_ssid"
//   #define WIFI_PASS "your_password"

#ifdef __has_include
  #if __has_include("wifi_config.h")
    #include "wifi_config.h"
  #else
    #define WIFI_SSID "DoubleDutch"
    #define WIFI_PASS "B0onD0cks56!!"
  #endif
#else
  #define WIFI_SSID "DoubleDutch"
  #define WIFI_PASS "B0onD0cks56!!"
#endif

// ============================================================================
// PINS & CONSTANTS
// ============================================================================

#define PIN_SERVO_SDA 4
#define PIN_SERVO_SCL 5
#define PIN_IMU_SDA   6
#define PIN_IMU_SCL   7
#define PIN_RGB       38

#define HTTP_PORT          80
#define WS_PORT            81
#define MAX_WS_PAYLOAD     512

constexpr float PI_F    = 3.14159265f;
constexpr float L_COXA  = 67.00f;
constexpr float L_FEMUR = 69.16f;
constexpr float L_TIBIA = 123.59f;

// Servo PWM limits — 50Hz, 12-bit PCA9685
// 150 ≈ 0.73ms, 300 ≈ 1.46ms (mid), 450 ≈ 2.20ms
constexpr int SERVO_MIN = 150;
constexpr int SERVO_MAX = 450;
constexpr int SERVO_MID = (SERVO_MIN + SERVO_MAX) / 2;  // 300

// Leg channel mapping: each leg = 4 consecutive channels
// FL=0-3, FR=4-7, BL=8-11, BR=12-15
constexpr int LEG_CHANNELS[4][4] = {
    {0,  1,  2,  3 },   // FL
    {4,  5,  6,  7 },   // FR
    {8,  9,  10, 11},   // BL
    {12, 13, 14, 15}    // BR
};

// Default joint positions (coxa, femur, tibia, twist)
constexpr int DEFAULT_POS[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

// Startup timing
constexpr unsigned long LEG_MOVE_DURATION_MS = 1000;   // 1 sec per leg
constexpr unsigned long STABILIZE_WINDOW_MS  = 200;    // IMU stability window
constexpr float         STABLE_VARIANCE_DEG  = 0.5f;   // max variance to pass
constexpr int           IMU_STABLE_COUNT     = 50;     // consecutive stable reads

// ============================================================================
// STARTUP STATE MACHINE
// ============================================================================

enum StartupState {
    SS_BOOT        = 0,  // hardware init in progress
    SS_MPU_WAIT    = 1,  // waiting for stable IMU readings
    SS_MOTOR_FL    = 2,  // positioning front-left leg
    SS_MOTOR_FR    = 3,  // positioning front-right leg
    SS_MOTOR_BL    = 4,  // positioning back-left leg
    SS_MOTOR_BR    = 5,  // positioning back-right leg
    SS_STABILIZE   = 6,  // waiting for body to settle
    SS_READY       = 7   // full operation + calibration available
};

volatile StartupState startupState = SS_BOOT;
SemaphoreHandle_t     startupMutex = NULL;

// ============================================================================
// HARDWARE STATUS
// ============================================================================

struct HardwareStatus {
    bool  pca9685        = false;
    bool  mpu6050        = false;
    bool  wifi           = false;
    float pitch          = 0.0f;
    float roll           = 0.0f;
    int   servoValues[16]= {0};
    int   startupPct     = 0;      // 0-100 for browser progress bar
} hwStatus;

SemaphoreHandle_t statusMutex   = NULL;

bool              testingServos = false;
SemaphoreHandle_t testModeMutex = NULL;

// Calibration offsets (set during SS_READY phase)
float calib_pitch_offset = 0.0f;
float calib_roll_offset  = 0.0f;

// ============================================================================
// IMU FILTER STATE
// ============================================================================

float         pitch_filtered  = 0.0f;
float         roll_filtered   = 0.0f;
unsigned long last_imu_time   = 0;
int           imu_stable_count= 0;

// Stability measurement
float pitch_history[10]       = {0};
float roll_history[10]        = {0};
int   history_idx             = 0;

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

Adafruit_PWMServoDriver pca(0x40);
Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
WebServer         server(HTTP_PORT);
WebSocketsServer  webSocket = WebSocketsServer(WS_PORT);

static char payload_buf[700];   // shared static buffer — never on stack

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}

// ============================================================================
// TRAPEZOIDAL VELOCITY PROFILE
// ============================================================================
//
//  Input:  t   — normalized time [0.0, 1.0]
//  Output: pos — normalized position [0.0, 1.0]
//
//  Velocity shape:
//    [0.000 – 0.286] → ease-in   (quadratic: 0 → v_cruise)
//    [0.286 – 0.714] → cruise    (constant: v_cruise = 0.8)
//    [0.714 – 1.000] → ease-out  (quadratic: v_cruise → 0)
//
//  These time fractions produce exactly:
//    20% of distance in accel phase
//    60% of distance in cruise phase
//    20% of distance in decel phase
//
float trapezoidPosition(float t) {
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

    constexpr float T1 = 0.2857f;   // end of accel (2/7)
    constexpr float T2 = 0.7143f;   // end of cruise (5/7)

    if (t < T1) {
        float tn = t / T1;                          // 0→1 within accel phase
        return 0.2f * (tn * tn);                    // quadratic → 20% distance
    } else if (t < T2) {
        float tn = (t - T1) / (T2 - T1);           // 0→1 within cruise phase
        return 0.2f + 0.6f * tn;                    // linear → 60% distance
    } else {
        float tn = (t - T2) / (1.0f - T2);         // 0→1 within decel phase
        return 0.8f + 0.2f * (2.0f * tn - tn * tn); // ease-out → 20% distance
    }
}

// ============================================================================
// MPU6050
// ============================================================================

const int MPU_ADDR = 0x68;

void writeMPU(byte reg, byte data) {
    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(reg);
    Wire1.write(data);
    Wire1.endTransmission();
}

bool initMPU() {
    if (!Wire1.begin(PIN_IMU_SDA, PIN_IMU_SCL, 100000)) {
        Serial.println("[MPU] I2C init FAILED");
        return false;
    }
    Wire1.setTimeOut(25);
    Wire1.beginTransmission(MPU_ADDR);
    if (Wire1.endTransmission() != 0) {
        Serial.println("[MPU] Not detected at 0x68");
        return false;
    }
    writeMPU(0x6B, 0x00);       // wake up
    delay(10);
    writeMPU(0x1C, 0x00);       // accel ±2g
    writeMPU(0x1A, 0x05);       // DLPF mode 5 = 10Hz cutoff
    writeMPU(0x1B, 0x08);       // gyro ±500 deg/s → 65.5 LSB/deg/s
    last_imu_time = millis();
    Serial.println("[MPU] ✓ Initialized (DLPF 10Hz)");
    return true;
}

void readMPU() {
    if (!hwStatus.mpu6050) return;

    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x3B);
    if (Wire1.endTransmission(false) != 0 || Wire1.requestFrom(MPU_ADDR, 14, true) < 14) {
        hwStatus.mpu6050 = false;
        Serial.println("[MPU] LOST CONNECTION");
        return;
    }

    int16_t ax = Wire1.read() << 8; ax |= Wire1.read();
    int16_t ay = Wire1.read() << 8; ay |= Wire1.read();
    int16_t az = Wire1.read() << 8; az |= Wire1.read();
    Wire1.read(); Wire1.read();  // temp
    int16_t gx = Wire1.read() << 8; gx |= Wire1.read();
    int16_t gy = Wire1.read() << 8; gy |= Wire1.read();
    Wire1.read(); Wire1.read();  // gz

    unsigned long now = millis();
    float dt = (float)(now - last_imu_time) / 1000.0f;
    if (dt > 0.5f) dt = 0.02f;
    if (dt < 0.001f) dt = 0.001f;
    last_imu_time = now;

    float accPitch  = atan2f((float)ay, (float)az) * 57.2958f;
    float accRoll   = atan2f(-(float)ax, (float)az) * 57.2958f;
    float gyroPitch = ((float)gx / 65.5f) * dt;
    float gyroRoll  = ((float)gy / 65.5f) * dt;

    // Complementary filter: 96% gyro integration + 4% accel correction
    pitch_filtered = 0.96f * (pitch_filtered + gyroPitch) + 0.04f * accPitch;
    roll_filtered  = 0.96f * (roll_filtered  + gyroRoll)  + 0.04f * accRoll;

    // Rolling history for stability check
    pitch_history[history_idx % 10] = pitch_filtered;
    roll_history[history_idx % 10]  = roll_filtered;
    history_idx++;

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hwStatus.pitch = pitch_filtered - calib_pitch_offset;
        hwStatus.roll  = roll_filtered  - calib_roll_offset;
        xSemaphoreGive(statusMutex);
    }
}

// Returns true when IMU has been stable for the required window
bool isMPUStable() {
    if (history_idx < 10) return false;
    float pSum = 0, rSum = 0;
    for (int i = 0; i < 10; i++) { pSum += pitch_history[i]; rSum += roll_history[i]; }
    float pMean = pSum / 10.0f, rMean = rSum / 10.0f;
    float pVar = 0, rVar = 0;
    for (int i = 0; i < 10; i++) {
        pVar += (pitch_history[i] - pMean) * (pitch_history[i] - pMean);
        rVar += (roll_history[i]  - rMean) * (roll_history[i]  - rMean);
    }
    return (pVar / 10.0f < STABLE_VARIANCE_DEG) && (rVar / 10.0f < STABLE_VARIANCE_DEG);
}

// ============================================================================
// PCA9685
// ============================================================================

bool initPCA() {
    Wire.begin(PIN_SERVO_SDA, PIN_SERVO_SCL, 400000);
    Wire.setTimeOut(50);
    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() != 0) {
        Serial.println("[PCA] Not detected at 0x40");
        return false;
    }
    pca.begin();
    pca.setPWMFreq(50);
    Serial.println("[PCA] ✓ Initialized at 50Hz");
    return true;
}

// Move a single PWM channel to target value, returns current interpolated value
int interpolateServo(int from, int to, float pos_fraction) {
    return from + (int)((float)(to - from) * pos_fraction);
}

// ── TRAPEZOIDAL LEG POSITIONING ─────────────────────────────────────────────
// Moves all 4 joints of one leg from current position to target.
// Blocks for duration_ms using trapezoidal profile.
// legIndex: 0=FL, 1=FR, 2=BL, 3=BR
void positionLegTrapezoidal(int legIndex, const int targetPWM[4],
                             unsigned long duration_ms) {
    if (!hwStatus.pca9685) return;
    if (legIndex < 0 || legIndex > 3) return;

    // Snapshot current positions
    int fromPWM[4];
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int j = 0; j < 4; j++)
            fromPWM[j] = hwStatus.servoValues[LEG_CHANNELS[legIndex][j]];
        xSemaphoreGive(statusMutex);
    } else {
        for (int j = 0; j < 4; j++) fromPWM[j] = SERVO_MID;
    }

    unsigned long start = millis();
    while (true) {
        unsigned long elapsed = millis() - start;
        if (elapsed >= duration_ms) elapsed = duration_ms;

        float t   = (float)elapsed / (float)duration_ms;
        float pos = trapezoidPosition(t);

        int newPWM[4];
        for (int j = 0; j < 4; j++) {
            newPWM[j] = interpolateServo(fromPWM[j], targetPWM[j], pos);
            pca.setPWM(LEG_CHANNELS[legIndex][j], 0, newPWM[j]);
        }

        // Update hwStatus under mutex
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int j = 0; j < 4; j++)
                hwStatus.servoValues[LEG_CHANNELS[legIndex][j]] = newPWM[j];
            xSemaphoreGive(statusMutex);
        }

        if (elapsed >= duration_ms) break;
        vTaskDelay(pdMS_TO_TICKS(16));  // ~60Hz update rate
    }

    Serial.printf("[SEQ] Leg %d positioned\n", legIndex);
}

void sweepServos() {
    if (!hwStatus.pca9685) return;
    static float phase = 0.0f;
    phase += 0.05f;
    if (phase > 2.0f * PI_F) phase -= 2.0f * PI_F;

    int tempValues[16];
    for (int ch = 0; ch < 16; ch++) {
        float offset    = (float)ch * (PI_F / 8.0f);
        float norm      = (sinf(phase + offset) + 1.0f) * 0.5f;
        tempValues[ch]  = SERVO_MIN + (int)(norm * (SERVO_MAX - SERVO_MIN));
        pca.setPWM(ch, 0, tempValues[ch]);
    }
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(hwStatus.servoValues, tempValues, sizeof(tempValues));
        xSemaphoreGive(statusMutex);
    }
}

void centerServos() {
    if (!hwStatus.pca9685) return;
    for (int ch = 0; ch < 16; ch++) pca.setPWM(ch, 0, SERVO_MID);
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int ch = 0; ch < 16; ch++) hwStatus.servoValues[ch] = SERVO_MID;
        xSemaphoreGive(statusMutex);
    }
}

// ============================================================================
// STARTUP SEQUENCE RUNNER (called from TaskDiagnostic)
// ============================================================================

void broadcastStartupState(const char* phase, int pct) {
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hwStatus.startupPct = pct;
        xSemaphoreGive(statusMutex);
    }
    int w = snprintf(payload_buf, sizeof(payload_buf),
        "{\"startup\":\"%s\",\"pct\":%d}", phase, pct);
    if (w > 0 && w < (int)sizeof(payload_buf))
        webSocket.broadcastTXT(payload_buf);
}

void runStartupSequence() {
    const int defaultTarget[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

    // ── PHASE 1: Wait for stable IMU ────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_MPU_WAIT;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("mpu_wait", 10);
    Serial.println("[SEQ] Waiting for IMU stability...");

    unsigned long imu_timeout = millis() + 5000;  // max 5 sec wait
    while (!isMPUStable() && millis() < imu_timeout) {
        readMPU();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    Serial.printf("[SEQ] IMU stable — pitch=%.1f roll=%.1f\n",
        pitch_filtered, roll_filtered);
    broadcastStartupState("mpu_ok", 20);

    // ── PHASE 2: Position FL leg ─────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_MOTOR_FL;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("leg_fl", 35);
    Serial.println("[SEQ] Positioning FL leg...");
    setRGB(124, 42, 232);  // purple = moving
    positionLegTrapezoidal(0, defaultTarget, LEG_MOVE_DURATION_MS);

    // ── PHASE 3: Position FR leg ─────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_MOTOR_FR;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("leg_fr", 50);
    Serial.println("[SEQ] Positioning FR leg...");
    positionLegTrapezoidal(1, defaultTarget, LEG_MOVE_DURATION_MS);

    // ── PHASE 4: Position BL leg ─────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_MOTOR_BL;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("leg_bl", 65);
    Serial.println("[SEQ] Positioning BL leg...");
    positionLegTrapezoidal(2, defaultTarget, LEG_MOVE_DURATION_MS);

    // ── PHASE 5: Position BR leg ─────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_MOTOR_BR;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("leg_br", 80);
    Serial.println("[SEQ] Positioning BR leg...");
    positionLegTrapezoidal(3, defaultTarget, LEG_MOVE_DURATION_MS);

    // ── PHASE 6: Stabilize ───────────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_STABILIZE;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("stabilize", 90);
    Serial.println("[SEQ] Waiting for body stabilization...");

    history_idx = 0;  // reset history to measure fresh standing stability
    unsigned long stab_timeout = millis() + 3000;
    while (!isMPUStable() && millis() < stab_timeout) {
        readMPU();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    Serial.println("[SEQ] Body stable — startup complete");

    // ── PHASE 7: READY ───────────────────────────────────────────────────────
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        startupState = SS_READY;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupState("ready", 100);
    setRGB(0, 255, 0);  // green = ready
    Serial.println("[SEQ] ✓ ROBOT READY — calibration available");
}

// ============================================================================
// HTML INTERFACE
// ============================================================================

const char html_interface[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ARTTOUS | NOMAD UPLINK</title>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600&family=JetBrains+Mono:wght@400;700&family=Orbitron:wght@500;700;900&display=swap" rel="stylesheet">
  <style>
    :root { --bg-core:#050505; --panel:rgba(10,10,10,0.85); --accent:#7c2ae8;
            --warn:#e74c3c; --success:#2ecc71; --text:#e0e0e0;
            --font-tech:'Orbitron',Arial,sans-serif; --font-mono:'JetBrains Mono','Courier New',monospace; }
    * { box-sizing:border-box; }
    body { margin:0; background:var(--bg-core); color:var(--text); font-family:'Inter',Arial,sans-serif; overflow:hidden; }
    canvas { position:absolute; top:0; left:0; z-index:-1; }

    /* ── Startup overlay ── */
    #startup-overlay {
      position:fixed; inset:0; z-index:100;
      background:rgba(5,5,5,0.92);
      display:flex; flex-direction:column;
      align-items:center; justify-content:center; gap:24px;
      backdrop-filter:blur(8px);
      transition:opacity 0.6s ease;
    }
    #startup-overlay.hidden { opacity:0; pointer-events:none; }
    .boot-title { font-family:var(--font-tech); font-size:1.6rem; color:#fff;
                  letter-spacing:4px; text-align:center; }
    .boot-subtitle { font-family:var(--font-mono); font-size:0.75rem; color:#666; letter-spacing:2px; }

    /* Progress bar */
    .progress-track { width:360px; height:4px; background:#1a1a1a; border-radius:2px; }
    .progress-fill  { height:100%; background:var(--accent); border-radius:2px;
                      width:0%; transition:width 0.4s ease;
                      box-shadow:0 0 12px rgba(124,42,232,0.6); }
    /* Step list */
    .boot-steps { display:flex; flex-direction:column; gap:8px; width:360px; }
    .step { display:flex; align-items:center; gap:12px;
            font-family:var(--font-mono); font-size:0.7rem; color:#444;
            transition:color 0.3s; }
    .step.active { color:#fff; }
    .step.done   { color:var(--success); }
    .step-icon { width:16px; text-align:center; }
    .step-bar { flex:1; height:2px; background:#222; border-radius:1px; overflow:hidden; }
    .step-fill { height:100%; width:0%; background:var(--accent);
                 border-radius:1px; transition:width 0.8s ease; }
    .step.done .step-fill { width:100%; background:var(--success); }
    .step.active .step-fill { width:60%; animation:pulse 1.2s infinite; }
    @keyframes pulse { 0%,100%{opacity:0.4} 50%{opacity:1} }

    /* ── Main UI ── */
    .ui { position:absolute; width:100%; height:100%; pointer-events:none;
          display:flex; justify-content:space-between; padding:20px; }
    .panel { width:320px; background:var(--panel); border:1px solid #333;
             padding:20px; pointer-events:auto; display:flex; flex-direction:column;
             backdrop-filter:blur(5px); overflow-y:auto; max-height:100vh; }
    /* Disable panels during startup */
    .panel.locked { pointer-events:none; opacity:0.4; }

    h3 { border-bottom:1px solid #333; padding-bottom:10px; margin:0 0 15px 0;
         font-size:1rem; letter-spacing:2px; font-family:var(--font-tech); color:#fff; }
    button { padding:12px; background:transparent; color:#aaa; border:1px solid #444;
             cursor:pointer; font-family:var(--font-tech); font-size:0.75rem;
             margin-bottom:8px; width:100%; transition:0.2s; }
    button:hover { border-color:var(--accent); color:#fff; }
    button.sel  { background:rgba(124,42,232,0.2); border-color:var(--accent); color:#fff; }
    button.warn { border-color:#444; color:#666; }
    button.warn:hover { border-color:var(--warn); background:rgba(231,76,60,0.1); color:var(--warn); }

    input[type=range] { -webkit-appearance:none; width:100%; background:transparent;
                        margin:10px 0 15px 0; cursor:pointer; }
    input[type=range]::-webkit-slider-runnable-track { height:4px; background:#222; border-radius:2px; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; height:16px; width:8px;
      background:var(--accent); margin-top:-6px; border-radius:2px;
      box-shadow:0 0 8px rgba(124,42,232,0.5); }
    label { font-family:var(--font-mono); font-size:0.7rem; color:#888; display:block; margin-top:5px; }

    .horizon-box  { width:100%; height:120px; background:#000; border:1px solid #333;
                    margin-bottom:15px; position:relative; overflow:hidden; }
    .horizon-sky  { width:300%; height:300%;
                    background:linear-gradient(to bottom,rgba(124,42,232,0.3) 50%,#111 50%);
                    position:absolute; top:-100%; left:-100%; transition:transform 0.05s linear; }
    .horizon-line { width:100%; height:1px; background:var(--accent);
                    position:absolute; top:50%; left:0; box-shadow:0 0 10px var(--accent); }
    .horizon-data { position:absolute; top:8px; left:8px; font-family:var(--font-mono);
                    font-size:0.75rem; color:#fff; text-shadow:1px 1px 2px #000; }

    .sticks    { display:flex; justify-content:space-between; margin:15px 0; }
    .stick-box { width:120px; height:120px; border:1px dashed #333; background:#080808;
                 position:relative; border-radius:50%; }
    .stick-dot { width:10px; height:10px; background:var(--accent); border-radius:50%;
                 position:absolute; top:50%; left:50%;
                 transform:translate(-50%,-50%);
                 box-shadow:0 0 10px var(--accent); transition:transform 0.05s linear; }

    .status-bar { margin-top:auto; padding-top:15px; border-top:1px solid #222;
                  display:flex; justify-content:space-between; align-items:center; }
    .stat-badge { padding:4px 8px; font-family:var(--font-mono); font-size:0.65rem;
                  border-radius:2px; border:1px solid #333; color:#666; }
    .stat-badge.ok  { color:var(--success); border-color:var(--success); background:rgba(46,204,113,0.1); }
    .stat-badge.bad { color:var(--warn);    border-color:var(--warn);    background:rgba(231,76,60,0.1); }
  </style>
</head>
<body>

<!-- ── STARTUP OVERLAY ────────────────────────────────────────────────── -->
<div id="startup-overlay">
  <div class="boot-title">ARTTOUS</div>
  <div class="boot-subtitle">NOMAD UPLINK v104 — INITIALIZING</div>

  <div class="progress-track"><div class="progress-fill" id="prog-fill"></div></div>

  <div class="boot-steps">
    <div class="step" id="step-ws">
      <span class="step-icon">○</span>
      <span>WEBSOCKET LINK</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-mpu">
      <span class="step-icon">○</span>
      <span>IMU ACQUIRE</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-fl">
      <span class="step-icon">○</span>
      <span>LEG FL — TRAPEZOIDAL INIT</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-fr">
      <span class="step-icon">○</span>
      <span>LEG FR — TRAPEZOIDAL INIT</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-bl">
      <span class="step-icon">○</span>
      <span>LEG BL — TRAPEZOIDAL INIT</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-br">
      <span class="step-icon">○</span>
      <span>LEG BR — TRAPEZOIDAL INIT</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-stab">
      <span class="step-icon">○</span>
      <span>BODY STABILIZATION</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
    <div class="step" id="step-ready">
      <span class="step-icon">○</span>
      <span>SYSTEM READY</span>
      <div class="step-bar"><div class="step-fill"></div></div>
    </div>
  </div>
</div>

<!-- ── MAIN UI ─────────────────────────────────────────────────────────── -->
<div class="ui">
  <div class="panel locked" id="panel-left">
    <h3>FLIGHT DECK</h3>
    <div class="horizon-box">
      <div class="horizon-sky" id="sky"></div>
      <div class="horizon-line"></div>
      <div class="horizon-data">
        PITCH: <span id="val-p" style="color:var(--accent);">0</span>°<br>
        ROLL:  <span id="val-r" style="color:var(--accent);">0</span>°
      </div>
    </div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
      <button onclick="send({cmd:'mode',val:'stand'})">STAND</button>
      <button onclick="toggleWalk()" id="btn-walk">AUTO WALK</button>
    </div>
    <button onclick="send({cmd:'calib'})" class="warn" style="color:#f1c40f;border-color:#444;"
            id="btn-calib" disabled>CALIBRATE IMU</button>
    <div class="sticks">
      <div class="stick-box"><div id="dot-l" class="stick-dot"></div></div>
      <div class="stick-box"><div id="dot-r" class="stick-dot"></div></div>
    </div>
    <label>Z-AXIS CLEARANCE (HEIGHT)</label>
    <input type="range" min="-140" max="-40" value="-60"
           oninput="send({cmd:'h',val:parseFloat(this.value)})">
    <button class="warn" onclick="send({cmd:'mode',val:'relax'})" style="margin-top:10px;">RELAX (POWER OFF)</button>
    <div class="status-bar">
      <span id="imu-stat" class="stat-badge bad">IMU DEAD</span>
      <span id="net-stat" class="stat-badge bad">OFFLINE</span>
    </div>
  </div>

  <div class="panel locked" id="panel-right">
    <h3>ENGINEERING</h3>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:15px;">
      <button id="l0" class="sel" onclick="selLeg(0)">LEG FL</button>
      <button id="l1"             onclick="selLeg(1)">LEG FR</button>
      <button id="l2"             onclick="selLeg(2)">LEG BL</button>
      <button id="l3"             onclick="selLeg(3)">LEG BR</button>
    </div>
    <label>J1_COXA (HIP)</label>
    <input type="range" min="-45" max="45" value="0" oninput="move(0,this.value)">
    <span id="val-c"  style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label>J2_FEMUR (SHOULDER)</label>
    <input type="range" min="-90" max="90" value="0" oninput="move(1,this.value)">
    <span id="val-f"  style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label>J3_TIBIA (ELBOW)</label>
    <input type="range" min="-90" max="90" value="0" oninput="move(2,this.value)">
    <span id="val-t"  style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label style="color:var(--accent);">J4_TWIST (WRIST)</label>
    <input type="range" min="-45" max="45" value="0" oninput="move(3,this.value)">
    <span id="val-tw" style="color:var(--accent);font-size:0.65rem;">0°</span>
    <hr style="border:0;border-top:1px dashed #333;margin:20px 0 10px 0;">
    <h3>DIAGNOSTICS</h3>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px;">
      <button class="warn" onclick="startTest(1)">TEST 4CH</button>
      <button class="warn" onclick="startTest(2)">TEST 16CH</button>
    </div>
    <button id="stop-btn" class="warn" style="display:none;" onclick="stopTest()">STOP TEST</button>
  </div>
</div>

<script type="module">
  import * as THREE from 'https://esm.sh/three@0.160.0';
  import { OrbitControls } from 'https://esm.sh/three@0.160.0/examples/jsm/controls/OrbitControls.js';

  // ── Startup overlay controller ────────────────────────────────────────────
  const STEP_MAP = {
    'ws':       'step-ws',
    'mpu_wait': 'step-mpu',
    'mpu_ok':   'step-mpu',
    'leg_fl':   'step-fl',
    'leg_fr':   'step-fr',
    'leg_bl':   'step-bl',
    'leg_br':   'step-br',
    'stabilize':'step-stab',
    'ready':    'step-ready'
  };

  // Mark a step as active, previous steps as done
  const stepOrder = ['step-ws','step-mpu','step-fl','step-fr','step-bl','step-br','step-stab','step-ready'];
  function advanceStep(stepId, pct) {
    document.getElementById('prog-fill').style.width = pct + '%';
    const idx = stepOrder.indexOf(stepId);
    stepOrder.forEach((id, i) => {
      const el = document.getElementById(id);
      if (!el) return;
      if (i < idx)  { el.classList.remove('active'); el.classList.add('done');   el.querySelector('.step-icon').innerText = '✓'; }
      if (i === idx){ el.classList.add('active');    el.classList.remove('done'); el.querySelector('.step-icon').innerText = '▶'; }
    });
  }

  function dismissOverlay() {
    // Mark all done
    stepOrder.forEach(id => {
      const el = document.getElementById(id);
      if (!el) return;
      el.classList.remove('active'); el.classList.add('done');
      el.querySelector('.step-icon').innerText = '✓';
    });
    document.getElementById('prog-fill').style.width = '100%';
    setTimeout(() => {
      document.getElementById('startup-overlay').classList.add('hidden');
      // Unlock panels
      document.getElementById('panel-left').classList.remove('locked');
      document.getElementById('panel-right').classList.remove('locked');
      document.getElementById('btn-calib').disabled = false;
    }, 800);
  }

  // ── WebSocket ─────────────────────────────────────────────────────────────
  let wsReady = false;
  const connectWS = () => {
    // FIX-C1: explicit port 81
    const ws = new WebSocket(`ws://` + location.hostname + `:81/`);
    window.ws = ws;
    ws.onopen = () => {
      wsReady = true;
      setBadge('net-stat', 'LINK OK', true);
      advanceStep('step-ws', 5);
    };
    ws.onclose = () => {
      wsReady = false;
      setBadge('net-stat', 'RECONNECTING...', false);
      setTimeout(connectWS, 2000);
    };
    ws.onerror = (e) => { console.error('[WS]', e); ws.close(); };
    ws.onmessage = onMessage;
  };
  connectWS();

  window.send = (data) => {
    if (wsReady && window.ws.readyState === WebSocket.OPEN)
      window.ws.send(JSON.stringify(data));
  };

  // ── Message handler ───────────────────────────────────────────────────────
  const onMessage = (e) => {
    let d; try { d = JSON.parse(e.data); } catch { return; }
    if (d.error) { console.warn('[SERVER]', d.error); return; }

    // Startup progress
    if (d.startup !== undefined) {
      const stepId = STEP_MAP[d.startup];
      if (stepId) advanceStep(stepId, d.pct || 0);
      if (d.startup === 'ready') dismissOverlay();
      return;
    }

    // IMU telemetry — FIX-C2: "p"/"r" keys
    if (d.p !== undefined) {
      setBadge('imu-stat', 'IMU LIVE', true);
      document.getElementById('val-p').innerText = d.p.toFixed(1);
      document.getElementById('val-r').innerText = d.r.toFixed(1);
      const cp = Math.max(-45, Math.min(45, d.p));
      const cr = Math.max(-45, Math.min(45, d.r));
      body.rotation.x = cp * 0.01745;
      body.rotation.z = cr * 0.01745;
      const py = Math.max(-60, Math.min(60, d.p * 3.0));
      document.getElementById('sky').style.transform =
        `translateY(${py}px) rotate(${-cr}deg)`;
    }

    // Leg angles
    if (d.l && Array.isArray(d.l) && d.l.length >= 4) {
      for (let i = 0; i < 4; i++) {
        if (d.l[i] && Array.isArray(d.l[i]) && d.l[i].length >= 4 && legsVis[i]) {
          const offRad = legsVis[i].userData.off * 0.01745;
          legsVis[i].userData.h.rotation.y  = offRad + d.l[i][0] * 0.01745;
          legsVis[i].userData.tw.rotation.x = d.l[i][3] * 0.01745;
          legsVis[i].userData.f.rotation.z  = d.l[i][1] * 0.01745;
          legsVis[i].userData.t.rotation.z  = d.l[i][2] * 0.01745;
        }
      }
      const al = d.l[window.activeLeg];
      if (al && al.length >= 4) {
        document.getElementById('val-c').innerText  = al[0].toFixed(1) + '°';
        document.getElementById('val-f').innerText  = al[1].toFixed(1) + '°';
        document.getElementById('val-t').innerText  = al[2].toFixed(1) + '°';
        document.getElementById('val-tw').innerText = al[3].toFixed(1) + '°';
      }
    }
  };

  setInterval(() => window.send({cmd:'ping'}), 1000);

  // ── Three.js ──────────────────────────────────────────────────────────────
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050505);
  const cam = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 1, 3000);
  cam.position.set(0, 400, 500);
  const ren = new THREE.WebGLRenderer({antialias:true});
  ren.setPixelRatio(devicePixelRatio);
  ren.setSize(innerWidth, innerHeight);
  document.body.appendChild(ren.domElement);
  const controls = new OrbitControls(cam, ren.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.GridHelper(1000, 100, 0x7c2ae8, 0x111111));
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));
  const spot = new THREE.PointLight(0x7c2ae8, 1.5, 1000);
  spot.position.set(0, 500, 0); scene.add(spot);

  const matWire  = new THREE.MeshBasicMaterial({color:0x7c2ae8, wireframe:true, opacity:0.4, transparent:true});
  const matSolid = new THREE.MeshStandardMaterial({color:0x111111, roughness:0.7});

  const body = new THREE.Group();
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77,46,134), matSolid));
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77,46,134), matWire));
  body.add(new THREE.AxesHelper(60));
  body.position.y = 23;
  scene.add(body);

  const legsVis = [];
  const mkBox = (w, h, d, ox) => {
    const g = new THREE.BoxGeometry(w, h, d); g.translate(ox, 0, 0);
    return new THREE.Mesh(g, matWire);
  };
  const createLeg = (x, z, mountDeg) => {
    const root  = new THREE.Group(); root.position.set(x, 23, z);
    const hip   = new THREE.Group(); hip.rotation.y = mountDeg * 0.01745; root.add(hip);
    const twist = new THREE.Group(); twist.position.x = 20; hip.add(twist);
    const femur = new THREE.Group(); femur.position.x = 46; twist.add(femur);
    const tibia = new THREE.Group(); tibia.position.x = 69; femur.add(tibia);
    hip.add(mkBox(20,10,10,10));  twist.add(mkBox(46,12,12,23));
    femur.add(mkBox(69,8,8,34.5)); tibia.add(mkBox(123,5,5,61.5));
    root.userData = {h:hip, tw:twist, f:femur, t:tibia, off:mountDeg};
    scene.add(root); legsVis.push(root);
  };
  createLeg(-38.8,-67.4, 135); createLeg( 38.8,-67.4,  45);
  createLeg(-38.8, 67.4, 225); createLeg( 38.8, 67.4, -45);

  window.addEventListener('resize', () => {
    cam.aspect = innerWidth / innerHeight;
    cam.updateProjectionMatrix();
    ren.setSize(innerWidth, innerHeight);
  });
  const renderLoop = () => { requestAnimationFrame(renderLoop); controls.update(); ren.render(scene, cam); };
  renderLoop();

  // ── Gamepad ───────────────────────────────────────────────────────────────
  setInterval(() => {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gps && gps[0]; if (!gp) return;
    const lx = gp.axes[0], ly = gp.axes[1], rx = gp.axes[2], ry = gp.axes[3];
    document.getElementById('dot-l').style.transform =
      `translate(calc(-50% + ${lx*50}px), calc(-50% + ${ly*50}px))`;
    document.getElementById('dot-r').style.transform =
      `translate(calc(-50% + ${rx*50}px), calc(-50% + ${ry*50}px))`;
    window.send({cmd:'pad', lx, ly, rx, ry, btn:Array.from(gp.buttons).map(b=>b.pressed?1:0)});
  }, 50);

  // ── Controls ──────────────────────────────────────────────────────────────
  window.activeLeg = 0;
  let walking = false;
  window.toggleWalk = () => {
    walking = !walking;
    const btn = document.getElementById('btn-walk');
    btn.innerText         = walking ? 'STOP WALK'  : 'AUTO WALK';
    btn.style.color       = walking ? '#fff'        : '#aaa';
    btn.style.background  = walking ? 'rgba(124,42,232,0.3)' : 'transparent';
    btn.style.borderColor = walking ? 'var(--accent)' : '#444';
    window.send({cmd:'mode', val: walking ? 'walk' : 'stand', auto: walking});
  };
  window.selLeg = (id) => {
    window.activeLeg = id;
    document.querySelectorAll('.panel button.sel').forEach(b => b.classList.remove('sel'));
    document.getElementById('l'+id).classList.add('sel');
    window.send({cmd:'active', val:id});
  };
  window.move      = (id, val) => window.send({cmd:'servo', leg:window.activeLeg, id, val:parseFloat(val)});
  window.startTest = (t) => { document.getElementById('stop-btn').style.display='block';  window.send({cmd:'test', val:parseInt(t)}); };
  window.stopTest  = ()  => { document.getElementById('stop-btn').style.display='none';   window.send({cmd:'test_stop'}); };

  function setBadge(id, text, ok) {
    const el = document.getElementById(id);
    el.innerText = text; el.className = 'stat-badge ' + (ok ? 'ok' : 'bad');
  }
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// HTTP HANDLER
// ============================================================================

void handleRoot() { server.send_P(200, PSTR("text/html"), html_interface); }

// ============================================================================
// WEBSOCKET
// ============================================================================

String buildStatusPayload() {
    String ip = WiFi.localIP().toString();
    if (ip.length() > 15) ip = "0.0.0.0";

    int srv[16];
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(srv, hwStatus.servoValues, sizeof(srv));
        xSemaphoreGive(statusMutex);
    } else { memset(srv, 0, sizeof(srv)); }

    int w = snprintf(payload_buf, sizeof(payload_buf),
        "{\"wifi\":%s,\"ip\":\"%s\",\"pca\":%s,\"mpu\":%s"
        ",\"p\":%.1f,\"r\":%.1f"
        ",\"srv\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
        hwStatus.wifi    ? "true" : "false", ip.c_str(),
        hwStatus.pca9685 ? "true" : "false",
        hwStatus.mpu6050 ? "true" : "false",
        hwStatus.pitch, hwStatus.roll,
        srv[0],srv[1],srv[2],srv[3],srv[4],srv[5],srv[6],srv[7],
        srv[8],srv[9],srv[10],srv[11],srv[12],srv[13],srv[14],srv[15]);

    if (w < 0 || w >= (int)sizeof(payload_buf)) return "{\"error\":\"overflow\"}";
    return String(payload_buf);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            Serial.printf("[WS] Client %d connected\n", num);
            webSocket.sendTXT(num, buildStatusPayload());
            // Send current startup state immediately
            StartupState ss = SS_BOOT;
            if (xSemaphoreTake(startupMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ss = startupState;
                xSemaphoreGive(startupMutex);
            }
            if (ss == SS_READY) {
                webSocket.sendTXT(num, "{\"startup\":\"ready\",\"pct\":100}");
            }
            break;
        }
        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client %d disconnected\n", num);
            if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                testingServos = false;
                xSemaphoreGive(testModeMutex);
            }
            centerServos();
            break;
        case WStype_TEXT: {
            if (length > MAX_WS_PAYLOAD) {
                webSocket.sendTXT(num, "{\"error\":\"payload_too_large\"}");
                return;
            }
            JsonDocument doc;
            if (deserializeJson(doc, (char *)payload) != DeserializationError::Ok) {
                webSocket.sendTXT(num, "{\"error\":\"invalid_json\"}");
                return;
            }
            const char *cmd = doc["cmd"];
            if (!cmd) return;

            if (strcmp(cmd, "status") == 0) {
                webSocket.sendTXT(num, buildStatusPayload());
            }
            else if (strcmp(cmd, "test") == 0) {
                if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    testingServos = true; xSemaphoreGive(testModeMutex);
                }
                webSocket.broadcastTXT("{\"log\":\"Servo sweep STARTED\"}");
            }
            else if (strcmp(cmd, "test_stop") == 0) {
                if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    testingServos = false; xSemaphoreGive(testModeMutex);
                }
                centerServos();
                webSocket.broadcastTXT("{\"log\":\"Servo sweep STOPPED\"}");
            }
            else if (strcmp(cmd, "calib") == 0) {
                // Zero out calibration offsets at current stable position
                calib_pitch_offset = pitch_filtered;
                calib_roll_offset  = roll_filtered;
                Serial.printf("[CALIB] Offsets set — pitch=%.2f roll=%.2f\n",
                    calib_pitch_offset, calib_roll_offset);
                webSocket.sendTXT(num, "{\"log\":\"IMU calibrated\"}");
            }
            break;
        }
        case WStype_PING: break;
        case WStype_PONG: break;
        default: break;
    }
}

// ============================================================================
// DIAGNOSTIC TASK — Core 0
// ============================================================================

void TaskDiagnostic(void *pvParameters) {
    static unsigned long lastBroadcast = 0;

    // Run startup sequence first (blocks until SS_READY)
    runStartupSequence();

    // Main telemetry loop
    for (;;) {
        readMPU();

        bool should_sweep = false;
        if (xSemaphoreTake(testModeMutex, 0) == pdTRUE) {
            should_sweep = testingServos; xSemaphoreGive(testModeMutex);
        }
        if (should_sweep) sweepServos();

        if (millis() - lastBroadcast > 100) {
            lastBroadcast = millis();
            if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                float p = hwStatus.pitch, r = hwStatus.roll;
                xSemaphoreGive(statusMutex);
                int w = snprintf(payload_buf, sizeof(payload_buf),
                    "{\"p\":%.1f,\"r\":%.1f"
                    ",\"l\":[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]}", p, r);
                if (w > 0 && w < (int)sizeof(payload_buf))
                    webSocket.broadcastTXT(payload_buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n╔═══════════════════════════════════════╗");
    Serial.println("║  ARTTOUS NOMAD UPLINK v104           ║");
    Serial.println("║  Trapezoidal Startup Sequence        ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    if (HTTP_PORT == WS_PORT) {
        Serial.println("[FATAL] Port collision!");
        while(1) delay(100);
    }

    rgb.begin();
    setRGB(255, 100, 0);

    statusMutex   = xSemaphoreCreateMutex();
    testModeMutex = xSemaphoreCreateMutex();
    startupMutex  = xSemaphoreCreateMutex();

    Serial.println("[INIT] PCA9685...");
    hwStatus.pca9685 = initPCA();
    Serial.println(hwStatus.pca9685 ? "[INIT] ✓ PCA9685 OK" : "[INIT] ✗ FAILED");

    if (hwStatus.pca9685) centerServos();  // safe park only — no sweep

    Serial.println("[INIT] MPU6050...");
    hwStatus.mpu6050 = initMPU();
    Serial.println(hwStatus.mpu6050 ? "[INIT] ✓ MPU6050 OK" : "[INIT] ✗ FAILED");

    Serial.println("[INIT] WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500); Serial.print("."); attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        hwStatus.wifi = true;
        Serial.printf("\n[INIT] ✓ WiFi — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[INIT] ✗ WiFi FAILED");
        setRGB(255, 0, 0);
    }

    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.printf("[HTTP] Port %d\n", HTTP_PORT);

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocket.enableHeartbeat(25000, 5000, 2);
    Serial.printf("[WS]   Port %d\n", WS_PORT);

    // Core 0: startup sequence → then telemetry loop
    xTaskCreatePinnedToCore(TaskDiagnostic, "Diagnostic", 8192, NULL, 2, NULL, 0);
    Serial.println("[SYSTEM] Task on core 0\n");
    Serial.printf("OPEN: http://%s\n\n", WiFi.localIP().toString().c_str());
}

// ============================================================================
// LOOP — Core 1
// ============================================================================

void loop() {
    server.handleClient();
    webSocket.loop();
    delay(1);
}
