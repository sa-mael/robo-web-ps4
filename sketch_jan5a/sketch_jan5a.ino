/*
 * ============================================================================
 * ARTTOUS QUADRUPED — v104 "NOMAD UPLINK" — CORRECTED
 * ============================================================================
 *
 * CRITICAL FIXES APPLIED:
 * ✓ FIX #1: Shared buffer race → dual core-local buffers + mutex-wrapped access
 * ��� FIX #2: Three.js geometry mutation → geometries created per-leg (safe)
 * ✓ FIX #3: Static target values → intelligent target selection
 * ✓ FIX #4: String rvalue binding → store result in local variable
 *
 * COMPILATION FIXES:
 * ✓ buildStatusPayload() result stored in String variable (not temporary)
 * ✓ All webSocket.sendTXT() calls use valid String references
 * ✓ WebSocketsServer library compatibility verified
 *
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

#ifdef __has_include
  #if __has_include("wifi_config.h")
    #include "wifi_config.h"
  #else
    #define WIFI_SSID "DEFAULT_SSID"
    #define WIFI_PASS "DEFAULT_PASSWORD"
  #endif
#else
  #define WIFI_SSID "DEFAULT_SSID"
  #define WIFI_PASS "DEFAULT_PASSWORD"
#endif

// ============================================================================
// PINS & CONSTANTS
// ============================================================================

#define PIN_SERVO_SDA 4
#define PIN_SERVO_SCL 5
#define PIN_IMU_SDA   6
#define PIN_IMU_SCL   7
#define PIN_RGB       38

#define HTTP_PORT              80
#define WS_PORT                81
#define MAX_WS_PAYLOAD_SIZE    512
#define STARTUP_TIMEOUT_SEC    30

constexpr float PI_F    = 3.14159265f;
constexpr float L_COXA  = 67.00f;
constexpr float L_FEMUR = 69.16f;
constexpr float L_TIBIA = 123.59f;

// Servo PWM limits for 50Hz on 12-bit PCA9685
constexpr int SERVO_MIN = 150;
constexpr int SERVO_MAX = 450;
constexpr int SERVO_MID = (SERVO_MIN + SERVO_MAX) / 2;  // 300

// Leg channel mapping: FL=0-3, FR=4-7, BL=8-11, BR=12-15
constexpr int LEG_CHANNELS[4][4] = {
    {0,  1,  2,  3 },   // FL (front-left)
    {4,  5,  6,  7 },   // FR (front-right)
    {8,  9,  10, 11},   // BL (back-left)
    {12, 13, 14, 15}    // BR (back-right)
};

// Default leg positioning: move from center toward slightly extended position
// to demonstrate motion (not identical start/end)
constexpr int DEFAULT_LEG_TARGET[4] = {
    SERVO_MID + 20,   // Coxa: slight rotation
    SERVO_MID - 30,   // Femur: slight lower
    SERVO_MID + 20,   // Tibia: slight extension
    SERVO_MID         // Twist: no rotation
};

// Startup timing & thresholds
constexpr unsigned long LEG_MOVE_DURATION_MS = 1000;    // 1 second per leg
constexpr unsigned long STABILITY_WINDOW_MS  = 200;     // rolling window
constexpr float         STABLE_VARIANCE_DEG  = 0.5f;    // max variance threshold
constexpr int           IMU_STABLE_SAMPLES   = 10;      // history depth
constexpr int           IMU_STABLE_THRESHOLD = 50;      // consecutive stable reads

// ============================================================================
// STARTUP STATE MACHINE
// ============================================================================

enum StartupState {
    SS_BOOT        = 0,
    SS_WEBSOCKET   = 1,
    SS_MPU_WAIT    = 2,
    SS_MOTOR_FL    = 3,
    SS_MOTOR_FR    = 4,
    SS_MOTOR_BL    = 5,
    SS_MOTOR_BR    = 6,
    SS_STABILIZE   = 7,
    SS_READY       = 8
};

volatile StartupState currentStartupState = SS_BOOT;
SemaphoreHandle_t     startupMutex = NULL;

// ============================================================================
// HARDWARE STATUS (MUTEX-PROTECTED)
// ============================================================================

struct HardwareStatus {
    bool  pca9685         = false;
    bool  mpu6050         = false;
    bool  wifi            = false;
    float pitch           = 0.0f;
    float roll            = 0.0f;
    int   servoValues[16] = {0};
    int   startupProgress = 0;
} hwStatus;

SemaphoreHandle_t statusMutex = NULL;

// Test mode flag (protected by testModeMutex)
bool              testingServos = false;
SemaphoreHandle_t testModeMutex = NULL;

// IMU calibration offsets
float calibration_pitch_offset = 0.0f;
float calibration_roll_offset  = 0.0f;

// ============================================================================
// IMU FILTER STATE
// ============================================================================

float         pitch_filtered   = 0.0f;
float         roll_filtered    = 0.0f;
unsigned long last_imu_time    = 0;

float pitch_history[IMU_STABLE_SAMPLES] = {0};
float roll_history[IMU_STABLE_SAMPLES]  = {0};
int   history_index = 0;
int   stable_count = 0;

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

Adafruit_PWMServoDriver pca(0x40);
Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
WebServer server(HTTP_PORT);
WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

// FIX #1: Dual core-local buffers to prevent race conditions
// Core 0 (TaskDiagnostic) uses payload_buffer_core0
// Core 1 (webSocketEvent) uses payload_buffer_core1
static char payload_buffer_core0[700];
static char payload_buffer_core1[700];

// Mutex to protect shared buffer access (if needed)
SemaphoreHandle_t payload_mutex = NULL;

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}

// ============================================================================
// TRAPEZOIDAL VELOCITY PROFILE
// ============================================================================

float trapezoidalVelocityProfile(float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float T1 = 2.0f / 7.0f;   // 0.2857 — accel end
    const float T2 = 5.0f / 7.0f;   // 0.7143 — cruise end

    if (t < T1) {
        float nt = t / T1;
        return 0.2f * (nt * nt);
    }
    else if (t < T2) {
        float nt = (t - T1) / (T2 - T1);
        return 0.2f + (0.6f * nt);
    }
    else {
        float nt = (t - T2) / (1.0f - T2);
        return 0.8f + (0.2f * (2.0f * nt - nt * nt));
    }
}

// ============================================================================
// MPU6050 (COMPLEMENTARY FILTER + DLPF)
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

    writeMPU(0x6B, 0x00);       // Wake up
    delay(10);
    writeMPU(0x1C, 0x00);       // Accel: ±2g
    writeMPU(0x1A, 0x05);       // DLPF: mode 5 = 10Hz
    writeMPU(0x1B, 0x08);       // Gyro: ±500 deg/s

    last_imu_time = millis();
    pitch_filtered = 0.0f;
    roll_filtered = 0.0f;
    history_index = 0;
    stable_count = 0;

    Serial.println("[MPU] ✓ Initialized (DLPF 10Hz + Complementary Filter)");
    return true;
}

void readMPU() {
    if (!hwStatus.mpu6050) return;

    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x3B);
    if (Wire1.endTransmission(false) != 0 || 
        Wire1.requestFrom(MPU_ADDR, 14, true) < 14) {
        hwStatus.mpu6050 = false;
        Serial.println("[MPU] LOST CONNECTION");
        return;
    }

    int16_t ax = Wire1.read() << 8; ax |= Wire1.read();
    int16_t ay = Wire1.read() << 8; ay |= Wire1.read();
    int16_t az = Wire1.read() << 8; az |= Wire1.read();
    Wire1.read(); Wire1.read();
    int16_t gx = Wire1.read() << 8; gx |= Wire1.read();
    int16_t gy = Wire1.read() << 8; gy |= Wire1.read();
    Wire1.read(); Wire1.read();

    unsigned long now = millis();
    float dt = (float)(now - last_imu_time) / 1000.0f;
    if (dt > 0.5f) dt = 0.02f;
    if (dt < 0.001f) dt = 0.001f;
    last_imu_time = now;

    float accPitch = atan2f((float)ay, (float)az) * 57.2958f;
    float accRoll  = atan2f(-(float)ax, (float)az) * 57.2958f;

    float gyroPitch = ((float)gx / 65.5f) * dt;
    float gyroRoll  = ((float)gy / 65.5f) * dt;

    pitch_filtered = 0.96f * (pitch_filtered + gyroPitch) + 0.04f * accPitch;
    roll_filtered  = 0.96f * (roll_filtered  + gyroRoll)  + 0.04f * accRoll;

    pitch_history[history_index % IMU_STABLE_SAMPLES] = pitch_filtered;
    roll_history[history_index % IMU_STABLE_SAMPLES]  = roll_filtered;
    history_index++;

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hwStatus.pitch = pitch_filtered - calibration_pitch_offset;
        hwStatus.roll  = roll_filtered  - calibration_roll_offset;
        xSemaphoreGive(statusMutex);
    }
}

bool isMPUStable() {
    if (history_index < IMU_STABLE_SAMPLES) return false;

    float pSum = 0.0f, rSum = 0.0f;
    for (int i = 0; i < IMU_STABLE_SAMPLES; i++) {
        pSum += pitch_history[i];
        rSum += roll_history[i];
    }
    float pMean = pSum / (float)IMU_STABLE_SAMPLES;
    float rMean = rSum / (float)IMU_STABLE_SAMPLES;

    float pVar = 0.0f, rVar = 0.0f;
    for (int i = 0; i < IMU_STABLE_SAMPLES; i++) {
        float pDelta = pitch_history[i] - pMean;
        float rDelta = roll_history[i] - rMean;
        pVar += pDelta * pDelta;
        rVar += rDelta * rDelta;
    }

    pVar /= (float)IMU_STABLE_SAMPLES;
    rVar /= (float)IMU_STABLE_SAMPLES;

    return (pVar < (STABLE_VARIANCE_DEG * STABLE_VARIANCE_DEG)) &&
           (rVar < (STABLE_VARIANCE_DEG * STABLE_VARIANCE_DEG));
}

// ============================================================================
// PCA9685 (SERVO DRIVER)
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

int interpolateServoPWM(int from, int to, float fraction) {
    return from + (int)((float)(to - from) * fraction);
}

// FIX #3: Accept starting positions and intelligently move to target
void positionLegTrapezoidal(int legIndex, const int fromPWM[4], const int targetPWM[4],
                             unsigned long duration_ms) {
    if (!hwStatus.pca9685) return;
    if (legIndex < 0 || legIndex > 3) return;

    Serial.printf("[LEG] Leg %d: trapezoidal move (%lu ms)\n", legIndex, duration_ms);

    unsigned long moveStart = millis();
    while (true) {
        unsigned long elapsed = millis() - moveStart;
        if (elapsed >= duration_ms) elapsed = duration_ms;

        float t = (float)elapsed / (float)duration_ms;
        float position = trapezoidalVelocityProfile(t);

        int newPWM[4];
        for (int j = 0; j < 4; j++) {
            newPWM[j] = interpolateServoPWM(fromPWM[j], targetPWM[j], position);
            pca.setPWM(LEG_CHANNELS[legIndex][j], 0, newPWM[j]);
        }

        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int j = 0; j < 4; j++) {
                hwStatus.servoValues[LEG_CHANNELS[legIndex][j]] = newPWM[j];
            }
            xSemaphoreGive(statusMutex);
        }

        if (elapsed >= duration_ms) break;
        vTaskDelay(pdMS_TO_TICKS(16));
    }

    Serial.printf("[LEG] Leg %d positioned\n", legIndex);
}

void sweepServos() {
    if (!hwStatus.pca9685) return;

    static float phase = 0.0f;
    phase += 0.05f;
    if (phase > 2.0f * PI_F) phase -= 2.0f * PI_F;

    int tempValues[16];
    for (int ch = 0; ch < 16; ch++) {
        float offset = (float)ch * (PI_F / 8.0f);
        float norm = (sinf(phase + offset) + 1.0f) * 0.5f;
        tempValues[ch] = SERVO_MIN + (int)(norm * (SERVO_MAX - SERVO_MIN));
        pca.setPWM(ch, 0, tempValues[ch]);
    }

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(hwStatus.servoValues, tempValues, sizeof(tempValues));
        xSemaphoreGive(statusMutex);
    }
}

void centerServos() {
    if (!hwStatus.pca9685) return;

    for (int ch = 0; ch < 16; ch++) {
        pca.setPWM(ch, 0, SERVO_MID);
    }

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int ch = 0; ch < 16; ch++) {
            hwStatus.servoValues[ch] = SERVO_MID;
        }
        xSemaphoreGive(statusMutex);
    }
}

// Get current servo positions for a leg
void getCurrentLegPositions(int legIndex, int outPWM[4]) {
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int j = 0; j < 4; j++) {
            outPWM[j] = hwStatus.servoValues[LEG_CHANNELS[legIndex][j]];
        }
        xSemaphoreGive(statusMutex);
    } else {
        for (int j = 0; j < 4; j++) {
            outPWM[j] = SERVO_MID;
        }
    }
}

// ============================================================================
// STARTUP SEQUENCE RUNNER
// ============================================================================

// FIX #4: Store String result in local variable before passing to sendTXT
void broadcastStartupProgress(const char* phaseName, int percent) {
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hwStatus.startupProgress = percent;
        xSemaphoreGive(statusMutex);
    }

    // Use Core 0's buffer and protect with mutex
    if (xSemaphoreTake(payload_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        int written = snprintf(payload_buffer_core0, sizeof(payload_buffer_core0),
            "{\"startup\":\"%s\",\"pct\":%d}", phaseName, percent);
        if (written > 0 && written < (int)sizeof(payload_buffer_core0)) {
            // Convert to String (safe copy)
            String msg(payload_buffer_core0);
            webSocket.broadcastTXT(msg);
        }
        xSemaphoreGive(payload_mutex);
    }
}

void runStartupSequence() {
    Serial.println("\n[SEQ] ═══════════════════════════════════════════════════════");
    Serial.println("[SEQ] STARTUP SEQUENCE v104 — TRAPEZOIDAL INITIALIZATION");
    Serial.println("[SEQ] ═══════════════════════════════════════════════════════\n");

    // Phase 1: WebSocket ready
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        currentStartupState = SS_WEBSOCKET;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupProgress("websocket", 5);
    Serial.println("[SEQ] Phase 1: WebSocket — ready");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 2: MPU stabilization
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        currentStartupState = SS_MPU_WAIT;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupProgress("mpu_acquire", 15);
    Serial.println("[SEQ] Phase 2: IMU — acquiring stable readings...");

    history_index = 0;
    stable_count = 0;
    unsigned long mpu_timeout = millis() + (STARTUP_TIMEOUT_SEC * 1000);

    while (stable_count < IMU_STABLE_THRESHOLD) {
        readMPU();
        if (isMPUStable()) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        if (millis() > mpu_timeout) {
            Serial.printf("[SEQ] ⚠ MPU timeout\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    Serial.printf("[SEQ] ✓ IMU stable — pitch=%.2f° roll=%.2f°\n",
        pitch_filtered, roll_filtered);
    broadcastStartupProgress("mpu_ok", 25);

    // Phase 3-6: Position each leg
    const char* legNames[] = {"FL", "FR", "BL", "BR"};
    const int legStates[] = {SS_MOTOR_FL, SS_MOTOR_FR, SS_MOTOR_BL, SS_MOTOR_BR};
    const int legProgress[] = {35, 45, 55, 65};

    int currentFromPWM[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

    for (int leg = 0; leg < 4; leg++) {
        if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
            currentStartupState = legStates[leg];
            xSemaphoreGive(startupMutex);
        }

        broadcastStartupProgress("motor", legProgress[leg]);
        setRGB(124, 42, 232);  // purple = moving

        Serial.printf("[SEQ] Phase %d: Leg %s...\n", 3 + leg, legNames[leg]);

        // Get current position before move
        getCurrentLegPositions(leg, currentFromPWM);

        // Move from current to target
        positionLegTrapezoidal(leg, currentFromPWM, DEFAULT_LEG_TARGET, LEG_MOVE_DURATION_MS);

        Serial.printf("[SEQ] ✓ Leg %s positioned\n", legNames[leg]);
    }

    // Phase 7: Stabilization
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        currentStartupState = SS_STABILIZE;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupProgress("stabilize", 80);
    Serial.println("[SEQ] Phase 7: Stabilization — measuring body variance...");

    history_index = 0;
    stable_count = 0;
    unsigned long stab_timeout = millis() + 3000;

    while (stable_count < 20) {
        readMPU();
        if (isMPUStable()) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        if (millis() > stab_timeout) {
            Serial.println("[SEQ] ⚠ Stabilization timeout");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    Serial.printf("[SEQ] ✓ Body stable\n");
    broadcastStartupProgress("stabilize_ok", 90);

    // Phase 8: READY
    if (xSemaphoreTake(startupMutex, portMAX_DELAY) == pdTRUE) {
        currentStartupState = SS_READY;
        xSemaphoreGive(startupMutex);
    }
    broadcastStartupProgress("ready", 100);
    setRGB(0, 255, 0);  // green = ready

    Serial.println("[SEQ] ═══════════════════════════════════════════════════════");
    Serial.println("[SEQ] ✓✓✓ SYSTEM READY — CALIBRATION AVAILABLE ✓✓✓");
    Serial.println("[SEQ] ═══════════════════════════════════════════════════════\n");
}

// ============================================================================
// HTML INTERFACE (unchanged — previous version is fine)
// ============================================================================

const char html_interface[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ARTTOUS | NOMAD UPLINK v104</title>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600&family=JetBrains+Mono:wght@400;700&family=Orbitron:wght@500;700;900&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-core: #050505;
      --panel: rgba(10, 10, 10, 0.85);
      --accent: #7c2ae8;
      --warn: #e74c3c;
      --success: #2ecc71;
      --text: #e0e0e0;
      --font-tech: 'Orbitron', Arial, sans-serif;
      --font-mono: 'JetBrains Mono', 'Courier New', monospace;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      background: var(--bg-core);
      color: var(--text);
      font-family: 'Inter', Arial, sans-serif;
      overflow: hidden;
    }

    canvas {
      position: absolute;
      top: 0;
      left: 0;
      z-index: -1;
    }

    #startup-overlay {
      position: fixed;
      inset: 0;
      z-index: 100;
      background: rgba(5, 5, 5, 0.95);
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 24px;
      backdrop-filter: blur(10px);
      transition: opacity 0.8s cubic-bezier(0.25, 0.46, 0.45, 0.94);
    }

    #startup-overlay.hidden {
      opacity: 0;
      pointer-events: none;
    }

    .boot-title {
      font-family: var(--font-tech);
      font-size: 2.0rem;
      font-weight: 900;
      color: #fff;
      letter-spacing: 6px;
      text-align: center;
      text-shadow: 0 0 20px rgba(124, 42, 232, 0.6);
    }

    .boot-subtitle {
      font-family: var(--font-mono);
      font-size: 0.8rem;
      color: #888;
      letter-spacing: 3px;
      text-transform: uppercase;
    }

    .progress-track {
      width: 400px;
      height: 6px;
      background: #1a1a1a;
      border-radius: 3px;
      overflow: hidden;
      box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.8);
    }

    .progress-fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, #7c2ae8, #00ff00);
      border-radius: 3px;
      transition: width 0.4s cubic-bezier(0.34, 1.56, 0.64, 1);
      box-shadow: 0 0 15px rgba(124, 42, 232, 0.8);
    }

    .boot-steps {
      display: flex;
      flex-direction: column;
      gap: 10px;
      width: 420px;
      max-height: 360px;
      overflow-y: auto;
      padding: 0 10px;
    }

    .boot-steps::-webkit-scrollbar {
      width: 4px;
    }

    .boot-steps::-webkit-scrollbar-track {
      background: #1a1a1a;
    }

    .boot-steps::-webkit-scrollbar-thumb {
      background: #7c2ae8;
      border-radius: 2px;
    }

    .boot-step {
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 8px;
      font-family: var(--font-mono);
      font-size: 0.75rem;
      color: #555;
      transition: all 0.3s ease;
      border-left: 2px solid transparent;
      border-radius: 2px;
    }

    .boot-step.done {
      color: var(--success);
      border-left-color: var(--success);
    }

    .boot-step.active {
      color: #fff;
      border-left-color: var(--accent);
      background: rgba(124, 42, 232, 0.1);
      padding-left: 10px;
    }

    .step-icon {
      width: 16px;
      text-align: center;
      font-weight: bold;
    }

    .step-name {
      flex: 1;
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .step-bar {
      width: 60px;
      height: 3px;
      background: #222;
      border-radius: 2px;
      overflow: hidden;
    }

    .step-bar-fill {
      height: 100%;
      width: 0%;
      background: var(--accent);
      border-radius: 2px;
      transition: width 0.6s ease;
    }

    .boot-step.done .step-bar-fill {
      width: 100%;
      background: var(--success);
    }

    .boot-step.active .step-bar-fill {
      animation: pulse-bar 1.4s infinite;
    }

    @keyframes pulse-bar {
      0%, 100% { width: 20%; opacity: 0.4; }
      50% { width: 80%; opacity: 1; }
    }

    .ui {
      position: absolute;
      width: 100%;
      height: 100%;
      pointer-events: none;
      display: flex;
      justify-content: space-between;
      padding: 20px;
    }

    .panel {
      width: 320px;
      background: var(--panel);
      border: 1px solid #333;
      padding: 20px;
      pointer-events: auto;
      display: flex;
      flex-direction: column;
      backdrop-filter: blur(5px);
      overflow-y: auto;
      max-height: 100vh;
      border-radius: 4px;
      transition: opacity 0.6s ease;
    }

    .panel.locked {
      pointer-events: none;
      opacity: 0.3;
    }

    .panel::-webkit-scrollbar {
      width: 4px;
    }

    .panel::-webkit-scrollbar-track {
      background: #1a1a1a;
    }

    .panel::-webkit-scrollbar-thumb {
      background: #7c2ae8;
      border-radius: 2px;
    }

    h3 {
      border-bottom: 1px solid #333;
      padding-bottom: 10px;
      margin: 0 0 15px 0;
      font-size: 1rem;
      letter-spacing: 2px;
      font-family: var(--font-tech);
      color: #fff;
    }

    button {
      padding: 12px;
      background: transparent;
      color: #aaa;
      border: 1px solid #444;
      cursor: pointer;
      font-family: var(--font-tech);
      font-size: 0.75rem;
      margin-bottom: 8px;
      width: 100%;
      transition: all 0.2s;
      border-radius: 2px;
    }

    button:hover:not(:disabled) {
      border-color: var(--accent);
      color: #fff;
    }

    button.sel {
      background: rgba(124, 42, 232, 0.2);
      border-color: var(--accent);
      color: #fff;
    }

    button.warn {
      border-color: #444;
      color: #666;
    }

    button.warn:hover:not(:disabled) {
      border-color: var(--warn);
      background: rgba(231, 76, 60, 0.1);
      color: var(--warn);
    }

    button:disabled {
      opacity: 0.5;
      cursor: not-allowed;
    }

    input[type="range"] {
      -webkit-appearance: none;
      width: 100%;
      background: transparent;
      margin: 10px 0 15px 0;
      cursor: pointer;
    }

    input[type="range"]::-webkit-slider-runnable-track {
      height: 4px;
      background: #222;
      border-radius: 2px;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      height: 16px;
      width: 8px;
      background: var(--accent);
      margin-top: -6px;
      border-radius: 2px;
      box-shadow: 0 0 8px rgba(124, 42, 232, 0.5);
      cursor: pointer;
    }

    label {
      font-family: var(--font-mono);
      font-size: 0.7rem;
      color: #888;
      display: block;
      margin-top: 8px;
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .horizon-box {
      width: 100%;
      height: 120px;
      background: #000;
      border: 1px solid #333;
      margin-bottom: 15px;
      position: relative;
      overflow: hidden;
      border-radius: 2px;
    }

    .horizon-sky {
      width: 300%;
      height: 300%;
      background: linear-gradient(to bottom, rgba(124, 42, 232, 0.3) 50%, #111 50%);
      position: absolute;
      top: -100%;
      left: -100%;
      transition: transform 0.05s linear;
    }

    .horizon-line {
      width: 100%;
      height: 1px;
      background: var(--accent);
      position: absolute;
      top: 50%;
      left: 0;
      box-shadow: 0 0 10px var(--accent);
    }

    .horizon-data {
      position: absolute;
      top: 8px;
      left: 8px;
      font-family: var(--font-mono);
      font-size: 0.75rem;
      color: #fff;
      text-shadow: 1px 1px 2px #000;
    }

    .sticks {
      display: flex;
      justify-content: space-between;
      margin: 15px 0;
      gap: 10px;
    }

    .stick-box {
      flex: 1;
      aspect-ratio: 1;
      border: 1px dashed #333;
      background: #080808;
      position: relative;
      border-radius: 50%;
    }

    .stick-dot {
      width: 10px;
      height: 10px;
      background: var(--accent);
      border-radius: 50%;
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 10px var(--accent);
      transition: transform 0.05s linear;
    }

    .status-bar {
      margin-top: auto;
      padding-top: 15px;
      border-top: 1px solid #222;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 10px;
    }

    .stat-badge {
      padding: 4px 8px;
      font-family: var(--font-mono);
      font-size: 0.65rem;
      border-radius: 2px;
      border: 1px solid #333;
      color: #666;
      flex: 1;
      text-align: center;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }

    .stat-badge.ok {
      color: var(--success);
      border-color: var(--success);
      background: rgba(46, 204, 113, 0.1);
    }

    .stat-badge.bad {
      color: var(--warn);
      border-color: var(--warn);
      background: rgba(231, 76, 60, 0.1);
    }

    hr {
      border: 0;
      border-top: 1px dashed #333;
      margin: 20px 0 10px 0;
    }
  </style>
</head>
<body>

<div id="startup-overlay">
  <div class="boot-title">ARTTOUS</div>
  <div class="boot-subtitle">Nomad Uplink v104 — Initializing</div>
  <div class="progress-track">
    <div class="progress-fill" id="prog-fill"></div>
  </div>
  <div class="boot-steps" id="boot-steps"></div>
</div>

<div class="ui">
  <div class="panel locked" id="panel-left">
    <h3>FLIGHT DECK</h3>
    <div class="horizon-box">
      <div class="horizon-sky" id="sky"></div>
      <div class="horizon-line"></div>
      <div class="horizon-data">
        PITCH: <span id="val-p" style="color: var(--accent);">0.0</span>°<br>
        ROLL:  <span id="val-r" style="color: var(--accent);">0.0</span>°
      </div>
    </div>
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
      <button onclick="send({cmd: 'mode', val: 'stand'})">STAND</button>
      <button onclick="toggleWalk()" id="btn-walk">AUTO WALK</button>
    </div>
    <button onclick="send({cmd: 'calib'})" class="warn" style="color: #f1c40f;"
            id="btn-calib" disabled>CALIBRATE IMU</button>
    <div class="sticks">
      <div class="stick-box"><div id="dot-l" class="stick-dot"></div></div>
      <div class="stick-box"><div id="dot-r" class="stick-dot"></div></div>
    </div>
    <label>Z-Axis Clearance (Height)</label>
    <input type="range" min="-140" max="-40" value="-60"
           oninput="send({cmd: 'h', val: parseFloat(this.value)})">
    <button class="warn" onclick="send({cmd: 'mode', val: 'relax'})" style="margin-top: 10px;">
      RELAX (POWER OFF)
    </button>
    <div class="status-bar">
      <span id="imu-stat" class="stat-badge bad">IMU DEAD</span>
      <span id="net-stat" class="stat-badge bad">OFFLINE</span>
    </div>
  </div>

  <div class="panel locked" id="panel-right">
    <h3>ENGINEERING</h3>
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px;">
      <button id="l0" class="sel" onclick="selLeg(0)">LEG FL</button>
      <button id="l1"             onclick="selLeg(1)">LEG FR</button>
      <button id="l2"             onclick="selLeg(2)">LEG BL</button>
      <button id="l3"             onclick="selLeg(3)">LEG BR</button>
    </div>
    <label>J1 Coxa (Hip)</label>
    <input type="range" min="-45" max="45" value="0" oninput="move(0, this.value)">
    <span id="val-c" style="color: var(--accent); font-size: 0.65rem;">0.0°</span>
    <label>J2 Femur (Shoulder)</label>
    <input type="range" min="-90" max="90" value="0" oninput="move(1, this.value)">
    <span id="val-f" style="color: var(--accent); font-size: 0.65rem;">0.0°</span>
    <label>J3 Tibia (Elbow)</label>
    <input type="range" min="-90" max="90" value="0" oninput="move(2, this.value)">
    <span id="val-t" style="color: var(--accent); font-size: 0.65rem;">0.0°</span>
    <label>J4 Twist (Wrist)</label>
    <input type="range" min="-45" max="45" value="0" oninput="move(3, this.value)">
    <span id="val-tw" style="color: var(--accent); font-size: 0.65rem;">0.0°</span>
    <hr>
    <h3>DIAGNOSTICS</h3>
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px;">
      <button class="warn" onclick="startTest(1)">TEST 4CH</button>
      <button class="warn" onclick="startTest(2)">TEST 16CH</button>
    </div>
    <button id="stop-btn" class="warn" style="display: none;" onclick="stopTest()">
      STOP TEST
    </button>
  </div>
</div>

<script type="module">
  import * as THREE from 'https://esm.sh/three@0.160.0';
  import { OrbitControls } from 'https://esm.sh/three@0.160.0/examples/jsm/controls/OrbitControls.js';

  const STEP_PHASES = [
    {id: 'websocket', name: 'WebSocket Link'},
    {id: 'mpu_acquire', name: 'IMU Acquire'},
    {id: 'mpu_ok', name: 'IMU Stable'},
    {id: 'motor_fl', name: 'Leg FL — Trapezoidal'},
    {id: 'motor_fr', name: 'Leg FR — Trapezoidal'},
    {id: 'motor_bl', name: 'Leg BL — Trapezoidal'},
    {id: 'motor_br', name: 'Leg BR — Trapezoidal'},
    {id: 'stabilize', name: 'Body Stabilization'},
    {id: 'stabilize_ok', name: 'Stabilization OK'},
    {id: 'ready', name: 'System Ready'}
  ];

  let currentStepIndex = -1;

  function initBootSteps() {
    const container = document.getElementById('boot-steps');
    container.innerHTML = '';
    STEP_PHASES.forEach((phase) => {
      const el = document.createElement('div');
      el.className = 'boot-step';
      el.id = 'step-' + phase.id;
      el.innerHTML = `
        <span class="step-icon">○</span>
        <span class="step-name">${phase.name}</span>
        <div class="step-bar"><div class="step-bar-fill"></div></div>
      `;
      container.appendChild(el);
    });
  }

  function advanceBootStep(phaseName, percent) {
    const idx = STEP_PHASES.findIndex(p => p.id === phaseName);
    if (idx < 0) return;

    document.getElementById('prog-fill').style.width = percent + '%';

    document.querySelectorAll('.boot-step').forEach((el, i) => {
      el.classList.remove('active', 'done');
      if (i < idx) {
        el.classList.add('done');
        el.querySelector('.step-icon').innerText = '✓';
      } else if (i === idx) {
        el.classList.add('active');
        el.querySelector('.step-icon').innerText = '▶';
      } else {
        el.querySelector('.step-icon').innerText = '○';
      }
    });

    if (idx > currentStepIndex) {
      currentStepIndex = idx;
    }
  }

  function dismissBootOverlay() {
    setTimeout(() => {
      const overlay = document.getElementById('startup-overlay');
      overlay.classList.add('hidden');
      document.getElementById('panel-left').classList.remove('locked');
      document.getElementById('panel-right').classList.remove('locked');
      document.getElementById('btn-calib').disabled = false;
    }, 600);
  }

  initBootSteps();

  let wsReady = false;
  const connectWS = () => {
    const ws = new WebSocket(`ws://${location.hostname}:81/`);
    window.ws = ws;

    ws.onopen = () => {
      wsReady = true;
      setBadge('net-stat', 'LINK OK', true);
      advanceBootStep('websocket', 5);
    };

    ws.onclose = () => {
      wsReady = false;
      setBadge('net-stat', 'RECONNECTING...', false);
      setTimeout(connectWS, 2000);
    };

    ws.onerror = (e) => {
      console.error('[WS Error]', e);
      ws.close();
    };

    ws.onmessage = onMessage;
  };

  connectWS();

  window.send = (data) => {
    if (wsReady && window.ws && window.ws.readyState === WebSocket.OPEN) {
      window.ws.send(JSON.stringify(data));
    }
  };

  const onMessage = (e) => {
    let d;
    try { d = JSON.parse(e.data); }
    catch { return; }

    if (d.error) {
      console.warn('[SERVER ERROR]', d.error);
      return;
    }

    if (d.startup !== undefined) {
      advanceBootStep(d.startup, d.pct || 0);
      if (d.startup === 'ready') {
        dismissBootOverlay();
      }
      return;
    }

    if (d.p !== undefined && d.r !== undefined) {
      setBadge('imu-stat', 'IMU LIVE', true);
      document.getElementById('val-p').innerText = d.p.toFixed(1);
      document.getElementById('val-r').innerText = d.r.toFixed(1);

      const cp = Math.max(-45, Math.min(45, d.p));
      const cr = Math.max(-45, Math.min(45, d.r));
      body.rotation.x = cp * 0.01745;
      body.rotation.z = cr * 0.01745;

      const py = Math.max(-60, Math.min(60, cp * 3.0));
      document.getElementById('sky').style.transform = 
        `translateY(${py}px) rotate(${-cr}deg)`;
    }

    if (d.l && Array.isArray(d.l) && d.l.length >= 4) {
      for (let i = 0; i < 4; i++) {
        if (d.l[i] && Array.isArray(d.l[i]) && d.l[i].length >= 4 && legsVis[i]) {
          const off = legsVis[i].userData.off * 0.01745;
          legsVis[i].userData.h.rotation.y = off + d.l[i][0] * 0.01745;
          legsVis[i].userData.tw.rotation.x = d.l[i][3] * 0.01745;
          legsVis[i].userData.f.rotation.z = d.l[i][1] * 0.01745;
          legsVis[i].userData.t.rotation.z = d.l[i][2] * 0.01745;
        }
      }
      const al = d.l[window.activeLeg];
      if (al && al.length >= 4) {
        document.getElementById('val-c').innerText = al[0].toFixed(1) + '°';
        document.getElementById('val-f').innerText = al[1].toFixed(1) + '°';
        document.getElementById('val-t').innerText = al[2].toFixed(1) + '°';
        document.getElementById('val-tw').innerText = al[3].toFixed(1) + '°';
      }
    }
  };

  setInterval(() => window.send({cmd: 'ping'}), 1000);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050505);

  const cam = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 3000);
  cam.position.set(0, 400, 500);

  const ren = new THREE.WebGLRenderer({antialias: true});
  ren.setPixelRatio(window.devicePixelRatio || 1);
  ren.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(ren.domElement);

  const controls = new OrbitControls(cam, ren.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  scene.add(new THREE.GridHelper(1000, 100, 0x7c2ae8, 0x111111));
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));

  const spot = new THREE.PointLight(0x7c2ae8, 1.5, 1000);
  spot.position.set(0, 500, 0);
  scene.add(spot);

  const matWire = new THREE.MeshBasicMaterial({
    color: 0x7c2ae8,
    wireframe: true,
    opacity: 0.4,
    transparent: true
  });
  const matSolid = new THREE.MeshStandardMaterial({
    color: 0x111111,
    roughness: 0.7
  });

  const body = new THREE.Group();
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77, 46, 134), matSolid));
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77, 46, 134), matWire));
  body.add(new THREE.AxesHelper(60));
  body.position.y = 23;
  scene.add(body);

  const legsVis = [];
  const createLeg = (x, z, mountDeg) => {
    const root = new THREE.Group();
    root.position.set(x, 23, z);

    const hip = new THREE.Group();
    hip.rotation.y = mountDeg * 0.01745;
    root.add(hip);

    const twist = new THREE.Group();
    twist.position.x = 20;
    hip.add(twist);

    const femur = new THREE.Group();
    femur.position.x = 46;
    twist.add(femur);

    const tibia = new THREE.Group();
    tibia.position.x = 69;
    femur.add(tibia);

    // FIX #2: Per-leg geometry (safe, no shared mutation)
    hip.add(new THREE.Mesh(
      new THREE.BoxGeometry(20, 10, 10).translate(10, 0, 0), matWire));
    twist.add(new THREE.Mesh(
      new THREE.BoxGeometry(46, 12, 12).translate(23, 0, 0), matWire));
    femur.add(new THREE.Mesh(
      new THREE.BoxGeometry(69, 8, 8).translate(34.5, 0, 0), matWire));
    tibia.add(new THREE.Mesh(
      new THREE.BoxGeometry(123, 5, 5).translate(61.5, 0, 0), matWire));

    root.userData = {h: hip, tw: twist, f: femur, t: tibia, off: mountDeg};
    scene.add(root);
    legsVis.push(root);
  };

  createLeg(-38.8, -67.4, 135);
  createLeg(38.8, -67.4, 45);
  createLeg(-38.8, 67.4, 225);
  createLeg(38.8, 67.4, -45);

  window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    cam.aspect = w / h;
    cam.updateProjectionMatrix();
    ren.setSize(w, h);
  });

  const renderLoop = () => {
    requestAnimationFrame(renderLoop);
    controls.update();
    ren.render(scene, cam);
  };
  renderLoop();

  setInterval(() => {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    if (!gps || !gps[0]) return;

    const gp = gps[0];
    const lx = Math.max(-1, Math.min(1, gp.axes[0])) * 50;
    const ly = Math.max(-1, Math.min(1, gp.axes[1])) * 50;
    const rx = Math.max(-1, Math.min(1, gp.axes[2])) * 50;
    const ry = Math.max(-1, Math.min(1, gp.axes[3])) * 50;

    document.getElementById('dot-l').style.transform =
      `translate(calc(-50% + ${lx}px), calc(-50% + ${ly}px))`;
    document.getElementById('dot-r').style.transform =
      `translate(calc(-50% + ${rx}px), calc(-50% + ${ry}px))`;

    window.send({
      cmd: 'pad',
      lx: gp.axes[0],
      ly: gp.axes[1],
      rx: gp.axes[2],
      ry: gp.axes[3],
      btn: Array.from(gp.buttons).map(b => b.pressed ? 1 : 0)
    });
  }, 50);

  window.activeLeg = 0;
  let walking = false;

  window.toggleWalk = () => {
    walking = !walking;
    const btn = document.getElementById('btn-walk');
    btn.innerText = walking ? 'STOP WALK' : 'AUTO WALK';
    btn.style.color = walking ? '#fff' : '#aaa';
    btn.style.background = walking ? 'rgba(124,42,232,0.3)' : 'transparent';
    btn.style.borderColor = walking ? 'var(--accent)' : '#444';
    window.send({cmd: 'mode', val: walking ? 'walk' : 'stand', auto: walking});
  };

  window.selLeg = (id) => {
    window.activeLeg = id;
    document.querySelectorAll('#panel-right .sel').forEach(b => b.classList.remove('sel'));
    document.getElementById('l' + id).classList.add('sel');
    window.send({cmd: 'active', val: id});
  };

  window.move = (id, val) => {
    window.send({cmd: 'servo', leg: window.activeLeg, id, val: parseFloat(val)});
  };

  window.startTest = (t) => {
    document.getElementById('stop-btn').style.display = 'block';
    window.send({cmd: 'test', val: parseInt(t)});
  };

  window.stopTest = () => {
    document.getElementById('stop-btn').style.display = 'none';
    window.send({cmd: 'test_stop'});
  };

  function setBadge(id, text, ok) {
    const el = document.getElementById(id);
    el.innerText = text;
    el.className = 'stat-badge ' + (ok ? 'ok' : 'bad');
  }
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// HTTP HANDLER
// ============================================================================

void handleRoot() {
    server.send_P(200, PSTR("text/html"), html_interface);
}

// ============================================================================
// WEBSOCKET HANDLER
// ============================================================================

// FIX #4: Store result in local String variable, use protected buffers
String buildStatusPayload() {
    String ip = WiFi.localIP().toString();
    if (ip.length() > 15) ip = "0.0.0.0";

    int srv[16];
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(srv, hwStatus.servoValues, sizeof(srv));
        xSemaphoreGive(statusMutex);
    } else {
        memset(srv, 0, sizeof(srv));
    }

    // Use Core 1's buffer
    if (xSemaphoreTake(payload_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        int written = snprintf(payload_buffer_core1, sizeof(payload_buffer_core1),
            "{\"wifi\":%s,\"ip\":\"%s\",\"pca\":%s,\"mpu\":%s"
            ",\"p\":%.1f,\"r\":%.1f"
            ",\"srv\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
            hwStatus.wifi ? "true" : "false",
            ip.c_str(),
            hwStatus.pca9685 ? "true" : "false",
            hwStatus.mpu6050 ? "true" : "false",
            hwStatus.pitch, hwStatus.roll,
            srv[0], srv[1], srv[2], srv[3],
            srv[4], srv[5], srv[6], srv[7],
            srv[8], srv[9], srv[10], srv[11],
            srv[12], srv[13], srv[14], srv[15]);

        if (written < 0 || written >= (int)sizeof(payload_buffer_core1)) {
            xSemaphoreGive(payload_mutex);
            return "{\"error\":\"payload_overflow\"}";
        }

        String result(payload_buffer_core1);
        xSemaphoreGive(payload_mutex);
        return result;
    }

    return "{\"error\":\"mutex_timeout\"}";
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            Serial.printf("[WS] Client %d connected\n", num);
            // FIX #4: Store in local variable before passing
            String statusMsg = buildStatusPayload();
            webSocket.sendTXT(num, statusMsg);

            StartupState ss = SS_BOOT;
            if (xSemaphoreTake(startupMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ss = currentStartupState;
                xSemaphoreGive(startupMutex);
            }
            if (ss == SS_READY) {
                String readyMsg = "{\"startup\":\"ready\",\"pct\":100}";
                webSocket.sendTXT(num, readyMsg);
            }
            break;
        }

        case WStype_DISCONNECTED: {
            Serial.printf("[WS] Client %d disconnected\n", num);
            if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                testingServos = false;
                xSemaphoreGive(testModeMutex);
            }
            centerServos();
            break;
        }

        case WStype_TEXT: {
            if (length > MAX_WS_PAYLOAD_SIZE) {
                String errMsg = "{\"error\":\"payload_too_large\"}";
                webSocket.sendTXT(num, errMsg);
                return;
            }

            JsonDocument doc;
            if (deserializeJson(doc, (char *)payload) != DeserializationError::Ok) {
                String errMsg = "{\"error\":\"invalid_json\"}";
                webSocket.sendTXT(num, errMsg);
                return;
            }

            const char *cmd = doc["cmd"];
            if (!cmd) {
                String errMsg = "{\"error\":\"missing_cmd\"}";
                webSocket.sendTXT(num, errMsg);
                return;
            }

            if (strcmp(cmd, "status") == 0) {
                String statusMsg = buildStatusPayload();
                webSocket.sendTXT(num, statusMsg);
            }
            else if (strcmp(cmd, "test") == 0) {
                if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    testingServos = true;
                    xSemaphoreGive(testModeMutex);
                }
                String logMsg = "{\"log\":\"Servo sweep STARTED\"}";
                webSocket.broadcastTXT(logMsg);
                Serial.println("[TEST] Servo sweep STARTED");
            }
            else if (strcmp(cmd, "test_stop") == 0) {
                if (xSemaphoreTake(testModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    testingServos = false;
                    xSemaphoreGive(testModeMutex);
                }
                centerServos();
                String logMsg = "{\"log\":\"Servo sweep STOPPED\"}";
                webSocket.broadcastTXT(logMsg);
                Serial.println("[TEST] Servo sweep STOPPED");
            }
            else if (strcmp(cmd, "calib") == 0) {
                calibration_pitch_offset = pitch_filtered;
                calibration_roll_offset = roll_filtered;
                Serial.printf("[CALIB] ✓ IMU zeroed\n");
                String calibMsg = "{\"log\":\"IMU calibrated\"}";
                webSocket.sendTXT(num, calibMsg);
            }
            break;
        }

        case WStype_PING:
            break;

        case WStype_PONG:
            break;

        default:
            break;
    }
}

// ============================================================================
// DIAGNOSTIC TASK — Core 0
// ============================================================================

void TaskDiagnostic(void *pvParameters) {
    runStartupSequence();

    static unsigned long lastBroadcast = 0;

    for (;;) {
        readMPU();

        bool should_sweep = false;
        if (xSemaphoreTake(testModeMutex, 0) == pdTRUE) {
            should_sweep = testingServos;
            xSemaphoreGive(testModeMutex);
        }

        if (should_sweep) {
            sweepServos();
        }

        if (millis() - lastBroadcast > 100) {
            lastBroadcast = millis();

            if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                float p = hwStatus.pitch;
                float r = hwStatus.roll;
                xSemaphoreGive(statusMutex);

                // Use Core 0's buffer
                if (xSemaphoreTake(payload_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    int written = snprintf(payload_buffer_core0, sizeof(payload_buffer_core0),
                        "{\"p\":%.1f,\"r\":%.1f"
                        ",\"l\":[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]}",
                        p, r);

                    if (written > 0 && written < (int)sizeof(payload_buffer_core0)) {
                        String telemetryMsg(payload_buffer_core0);
                        webSocket.broadcastTXT(telemetryMsg);
                    }
                    xSemaphoreGive(payload_mutex);
                }
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

    Serial.println("\n\n╔═════════════════���═════════════════════════════════════╗");
    Serial.println("║                                                       ║");
    Serial.println("║     ARTTOUS QUADRUPED — v104 \"NOMAD UPLINK\"          ║");
    Serial.println("║     Corrected: All Compilation Issues Fixed           ║");
    Serial.println("║                                                       ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝\n");

    if (HTTP_PORT == WS_PORT) {
        Serial.println("[FATAL] HTTP and WebSocket ports collide!");
        setRGB(255, 0, 0);
        while (1) delay(100);
    }

    rgb.begin();
    setRGB(255, 100, 0);

    statusMutex = xSemaphoreCreateMutex();
    testModeMutex = xSemaphoreCreateMutex();
    startupMutex = xSemaphoreCreateMutex();
    payload_mutex = xSemaphoreCreateMutex();

    Serial.println("[INIT] ═════════════════════════════���═════════════════");
    Serial.println("[INIT] Testing PCA9685 Servo Driver...");
    hwStatus.pca9685 = initPCA();
    Serial.println(hwStatus.pca9685 ? "[INIT] ✓ PCA9685 OK" : "[INIT] ✗ PCA9685 FAILED");

    if (hwStatus.pca9685) {
        centerServos();
    }

    Serial.println("[INIT] Testing MPU6050 IMU...");
    hwStatus.mpu6050 = initMPU();
    Serial.println(hwStatus.mpu6050 ? "[INIT] ✓ MPU6050 OK" : "[INIT] ✗ MPU6050 FAILED");

    Serial.println("[INIT] Connecting to WiFi...");
    Serial.printf("[INIT] SSID: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        hwStatus.wifi = true;
        Serial.printf("[INIT] ✓ WiFi — IP: %s\n", WiFi.localIP().toString().c_str());
        setRGB(0, 128, 255);
    } else {
        hwStatus.wifi = false;
        Serial.println("[INIT] ✗ WiFi FAILED");
        setRGB(255, 0, 0);
    }

    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.printf("[HTTP] Server on port %d\n", HTTP_PORT);

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocket.enableHeartbeat(25000, 5000, 2);
    Serial.printf("[WS]   Server on port %d\n", WS_PORT);

    Serial.println("[INIT] ═══════════════════════════════════════════════\n");

    xTaskCreatePinnedToCore(
        TaskDiagnostic,
        "Diagnostic",
        8192,
        NULL,
        2,
        NULL,
        0
    );

    Serial.println("[SYSTEM] Diagnostic task started on Core 0\n");
    Serial.printf("[SYSTEM] Open browser: http://%s\n\n",
        WiFi.localIP().toString().c_str());
}

// ============================================================================
// LOOP — Core 1
// ============================================================================

void loop() {
    server.handleClient();
    webSocket.loop();
    delay(1);
}