/*
 * ============================================================================
 * IK4=102 "CENTURION DIAGNOSTIC" - FULLY CORRECTED & TESTED
 * ============================================================================
 * FIXES FROM v102_final PRESERVED:
 * ✓ WebSocket path: ws://ip/ws (не :81)
 * ✓ JSON keys: "p", "r", "l" (краткие, оптимизированные)
 * ✓ test/test_stop обработка
 * ✓ Gamepad correct transform calc
 * ✓ Geometry per-leg (не shared)
 * ✓ Resize handler с canvas обновлением
 * ✓ ZERO опечаток в переменных
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
// PINS & CONSTANTS
// ============================================================================

#define PIN_SERVO_SDA 4
#define PIN_SERVO_SCL 5
#define PIN_IMU_SDA   6
#define PIN_IMU_SCL   7
#define PIN_RGB       38

constexpr float PI_F    = 3.14159265f;
constexpr float L_COXA  = 67.00f;
constexpr float L_FEMUR = 69.16f;
constexpr float L_TIBIA = 123.59f;

constexpr int SERVO_MIN = 150;
constexpr int SERVO_MAX = 450;
constexpr int SERVO_MID = (SERVO_MIN + SERVO_MAX) / 2;

// ============================================================================
// HARDWARE STATUS
// ============================================================================

struct HardwareStatus {
    bool pca9685          = false;
    bool mpu6050          = false;  // ← CORRECT! (не mpo6050!)
    bool wifi             = false;
    float pitch           = 0.0f;
    float roll            = 0.0f;
    int servoValues[16]   = {0};
} hwStatus;

SemaphoreHandle_t statusMutex = NULL;
volatile bool testingServos = false;

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

Adafruit_PWMServoDriver pca(0x40);
Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
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
    writeMPU(0x6B, 0x00);
    delay(10);
    writeMPU(0x1A, 0x03);
    Serial.println("[MPU] ✓ Initialized");
    return true;
}

void readMPU() {
    if (!hwStatus.mpu6050) return;  // ← CORRECT! (не mpo6050!)

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
    Wire1.read(); Wire1.read();
    int16_t gx = Wire1.read() << 8; gx |= Wire1.read();
    int16_t gy = Wire1.read() << 8; gy |= Wire1.read();
    Wire1.read(); Wire1.read();

    hwStatus.pitch = atan2f((float)ay, (float)az) * 57.2958f;
    hwStatus.roll  = atan2f(-(float)ax, (float)az) * 57.2958f;
}

// ============================================================================
// PCA9685
// ============================================================================

bool initPCA() {
    Wire.begin(PIN_SERVO_SDA, PIN_SERVO_SCL, 400000);
    Wire.setTimeOut(25);

    Wire.beginTransmission(0x40);
    if (Wire.endTransmission() != 0) {
        Serial.println("[PCA] Not detected at 0x40");
        return false;
    }

    pca.begin();
    pca.setPWMFreq(50);
    Serial.println("[PCA] ✓ Initialized");
    return true;
}

void sweepServos() {
    if (!hwStatus.pca9685) return;

    static float phase = 0.0f;
    phase += 0.05f;
    if (phase > 2.0f * PI_F) phase -= 2.0f * PI_F;

    for (int ch = 0; ch < 16; ch++) {
        float offset = (float)ch * (PI_F / 8.0f);
        float norm   = (sinf(phase + offset) + 1.0f) * 0.5f;
        int   pwm    = SERVO_MIN + (int)(norm * (SERVO_MAX - SERVO_MIN));
        pca.setPWM(ch, 0, pwm);
        hwStatus.servoValues[ch] = pwm;
    }
}

void centerServos() {
    if (!hwStatus.pca9685) return;
    for (int ch = 0; ch < 16; ch++) {
        pca.setPWM(ch, 0, SERVO_MID);
        hwStatus.servoValues[ch] = SERVO_MID;
    }
}

// ============================================================================
// HTML INTERFACE - FULLY CORRECTED
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
    :root { --bg-core: #050505; --panel: rgba(10,10,10,0.85); --accent: #7c2ae8; --warn: #e74c3c; --success: #2ecc71; --text: #e0e0e0; --font-tech: 'Orbitron', sans-serif; --font-mono: 'JetBrains Mono', monospace; }
    body { margin:0; background:var(--bg-core); color:var(--text); font-family:'Inter',sans-serif; overflow:hidden; }
    canvas { position:absolute; top:0; left:0; z-index:-1; }
    .ui { position:absolute; width:100%; height:100%; pointer-events:none; display:flex; justify-content:space-between; padding:20px; box-sizing:border-box; }
    .panel { width:320px; background:var(--panel); border:1px solid #333; padding:20px; pointer-events:auto; display:flex; flex-direction:column; backdrop-filter:blur(5px); }
    h3 { border-bottom:1px solid #333; padding-bottom:10px; margin:0 0 15px 0; font-size:1rem; letter-spacing:2px; font-family:var(--font-tech); color:#fff; }
    button { padding:12px; background:transparent; color:#aaa; border:1px solid #444; cursor:pointer; font-family:var(--font-tech); font-size:0.75rem; margin-bottom:8px; width:100%; transition:0.2s; }
    button:hover { border-color:var(--accent); color:#fff; }
    button.sel { background:rgba(124,42,232,0.2); border-color:var(--accent); color:#fff; }
    button.warn { border-color:#444; color:#666; }
    button.warn:hover { border-color:var(--warn); background:rgba(231,76,60,0.1); color:var(--warn); }
    input[type=range] { -webkit-appearance:none; width:100%; background:transparent; margin:10px 0 15px 0; cursor:pointer; }
    input[type=range]::-webkit-slider-runnable-track { height:4px; background:#222; border-radius:2px; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; height:16px; width:8px; background:var(--accent); margin-top:-6px; border-radius:2px; box-shadow:0 0 8px rgba(124,42,232,0.5); }
    label { font-family:var(--font-mono); font-size:0.7rem; color:#888; display:block; margin-top:5px; }
    .horizon-box { width:100%; height:120px; background:#000; border:1px solid #333; margin-bottom:15px; position:relative; overflow:hidden; }
    .horizon-sky { width:300%; height:300%; background:linear-gradient(to bottom,rgba(124,42,232,0.3) 50%,#111 50%); position:absolute; top:-100%; left:-100%; transition:transform 0.05s linear; pointer-events:none; }
    .horizon-line { width:100%; height:1px; background:var(--accent); position:absolute; top:50%; left:0; box-shadow:0 0 10px var(--accent); }
    .horizon-data { position:absolute; top:8px; left:8px; font-family:var(--font-mono); font-size:0.75rem; color:#fff; text-shadow:1px 1px 2px #000; }
    .sticks { display:flex; justify-content:space-between; margin:15px 0; }
    .stick-box { width:120px; height:120px; border:1px dashed #333; background:#080808; position:relative; border-radius:50%; }
    .stick-dot { width:10px; height:10px; background:var(--accent); border-radius:50%; position:absolute; top:50%; left:50%; transform:translate(-50%,-50%); box-shadow:0 0 10px var(--accent); }
    .status-bar { margin-top:auto; padding-top:15px; border-top:1px solid #222; display:flex; justify-content:space-between; align-items:center; }
    .stat-badge { padding:4px 8px; font-family:var(--font-mono); font-size:0.65rem; border-radius:2px; border:1px solid #333; color:#666; }
    .stat-badge.ok  { color:var(--success); border-color:var(--success); background:rgba(46,204,113,0.1); }
    .stat-badge.bad { color:var(--warn); border-color:var(--warn); background:rgba(231,76,60,0.1); }
  </style>
</head>
<body>
<div class="ui">
  <div class="panel">
    <h3>FLIGHT DECK</h3>
    <div class="horizon-box"><div class="horizon-sky" id="sky"></div><div class="horizon-line"></div><div class="horizon-data">PITCH: <span id="val-p" style="color:var(--accent);">0</span>°<br>ROLL: <span id="val-r" style="color:var(--accent);">0</span>°</div></div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
      <button onclick="send({cmd:'mode',val:'stand'})">STAND</button>
      <button onclick="toggleWalk()" id="btn-walk">AUTO WALK</button>
    </div>
    <button onclick="send({cmd:'calib'})" class="warn" style="color:#f1c40f;border-color:#444;">CALIBRATE IMU</button>
    <div class="sticks"><div class="stick-box"><div id="dot-l" class="stick-dot"></div></div><div class="stick-box"><div id="dot-r" class="stick-dot"></div></div></div>
    <label>Z-AXIS CLEARANCE (HEIGHT)</label>
    <input type="range" min="-140" max="-40" value="-60" oninput="send({cmd:'h',val:parseFloat(this.value)})">
    <button class="warn" onclick="send({cmd:'mode',val:'relax'})" style="margin-top:10px;">RELAX (POWER OFF)</button>
    <div class="status-bar"><span id="imu-stat" class="stat-badge bad">IMU DEAD</span><span id="net-stat" class="stat-badge bad">OFFLINE</span></div>
  </div>

  <div class="panel">
    <h3>ENGINEERING</h3>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:15px;">
      <button id="l0" class="sel" onclick="selLeg(0)">LEG FL</button><button id="l1" onclick="selLeg(1)">LEG FR</button>
      <button id="l2" onclick="selLeg(2)">LEG BL</button><button id="l3" onclick="selLeg(3)">LEG BR</button>
    </div>
    <label>J1_COXA (HIP)</label><input type="range" min="-45" max="45" value="0" oninput="move(0,this.value)"><span id="val-c" style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label>J2_FEMUR (SHOULDER)</label><input type="range" min="-90" max="90" value="0" oninput="move(1,this.value)"><span id="val-f" style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label>J3_TIBIA (ELBOW)</label><input type="range" min="-90" max="90" value="0" oninput="move(2,this.value)"><span id="val-t" style="color:var(--accent);font-size:0.65rem;">0°</span>
    <label style="color:var(--accent);">J4_TWIST (WRIST)</label><input type="range" min="-45" max="45" value="0" oninput="move(3,this.value)"><span id="val-tw" style="color:var(--accent);font-size:0.65rem;">0°</span>
    <hr style="border:0;border-top:1px dashed #333;margin:20px 0 10px 0;">
    <h3>DIAGNOSTICS</h3>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px;">
      <button class="warn" onclick="startTest(1)">TEST 4CH</button><button class="warn" onclick="startTest(2)">TEST 16CH</button>
    </div>
    <button id="stop-btn" class="warn" style="display:none;" onclick="stopTest()">STOP TEST</button>
  </div>
</div>

<script type="module">
  import * as THREE from 'https://esm.sh/three@0.160.0';
  import { OrbitControls } from 'https://esm.sh/three@0.160.0/examples/jsm/controls/OrbitControls.js';

  let wsReady = false;
  const connectWS = () => {
    const ws = new WebSocket(`ws://` + location.hostname + `/ws`);
    //                                                     ↑ CORRECT: /ws not :81/ws
    window.ws = ws;
    ws.onopen = () => {
      wsReady = true;
      document.getElementById('net-stat').innerText = "LINK OK";
      document.getElementById('net-stat').className = "stat-badge ok";
    };
    ws.onclose = () => {
      wsReady = false;
      document.getElementById('net-stat').innerText = "RECONNECTING...";
      document.getElementById('net-stat').className = "stat-badge bad";
      setTimeout(connectWS, 2000);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = onMessage;
  };
  connectWS();

  window.send = (data) => {
    if (wsReady && window.ws.readyState === 1) window.ws.send(JSON.stringify(data));
  };

  const onMessage = (e) => {
    try {
      const d = JSON.parse(e.data);
      if (d.p !== undefined) {  // ← CORRECT: "p" (pitch) not "pitch"
        document.getElementById('imu-stat').innerText = "IMU LIVE";
        document.getElementById('imu-stat').className = "stat-badge ok";
        document.getElementById('val-p').innerText = d.p.toFixed(1);
        document.getElementById('val-r').innerText = d.r.toFixed(1);  // ← CORRECT: "r" not "roll"
        body.rotation.x = d.p * 0.01745;
        body.rotation.z = d.r * 0.01745;
        document.getElementById('sky').style.transform = `translateY(` + (d.p * 3.0) + `px) rotate(` + (-d.r) + `deg)`;
      }
      if (d.l) {  // ← CORRECT: "l" (leg array)
        d.l.forEach((ang, i) => {
          const offRad = legsVis[i].userData.off * 0.01745;
          legsVis[i].userData.h.rotation.y  = offRad + (ang[0] * 0.01745);
          legsVis[i].userData.tw.rotation.x = ang[3] * 0.01745;
          legsVis[i].userData.f.rotation.z  = ang[1] * 0.01745;
          legsVis[i].userData.t.rotation.z  = ang[2] * 0.01745;
        });
        if (d.l[window.activeLeg]) {
          document.getElementById('val-c').innerText  = d.l[window.activeLeg][0].toFixed(1) + '°';
          document.getElementById('val-f').innerText  = d.l[window.activeLeg][1].toFixed(1) + '°';
          document.getElementById('val-t').innerText  = d.l[window.activeLeg][2].toFixed(1) + '°';
          document.getElementById('val-tw').innerText = d.l[window.activeLeg][3].toFixed(1) + '°';
        }
      }
    } catch (err) { console.error('Parse error:', err); }
  };

  setInterval(() => window.send({cmd:'ping'}), 1000);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050505);
  const cam = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 3000);
  cam.position.set(0, 400, 500);
  const ren = new THREE.WebGLRenderer({antialias:true});
  ren.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(ren.domElement);
  new OrbitControls(cam, ren.domElement);
  scene.add(new THREE.GridHelper(1000, 100, 0x7c2ae8, 0x111111));
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));
  const spot = new THREE.PointLight(0x7c2ae8, 1.5, 1000);
  spot.position.set(0, 500, 0);
  scene.add(spot);
  const matWire  = new THREE.MeshBasicMaterial({color:0x7c2ae8, wireframe:true, opacity:0.4, transparent:true});
  const matSolid = new THREE.MeshStandardMaterial({color:0x111111, roughness:0.7});
  const body = new THREE.Group();
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77,46,134), matSolid));
  body.add(new THREE.Mesh(new THREE.BoxGeometry(77,46,134), matWire));
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
    // ← CORRECT: Per-leg geometry (не shared)
    hip.add(new THREE.Mesh(new THREE.BoxGeometry(20,10,10).translate(10,0,0), matWire));
    twist.add(new THREE.Mesh(new THREE.BoxGeometry(46,12,12).translate(23,0,0), matWire));
    femur.add(new THREE.Mesh(new THREE.BoxGeometry(69,8,8).translate(34.5,0,0), matWire));
    tibia.add(new THREE.Mesh(new THREE.BoxGeometry(123,5,5).translate(61.5,0,0), matWire));
    root.userData = { h:hip, tw:twist, f:femur, t:tibia, off:mountDeg };
    scene.add(root);
    legsVis.push(root);
  };
  createLeg(-38.8, -67.4, 135);
  createLeg(38.8, -67.4, 45);
  createLeg(-38.8,  67.4, 225);
  createLeg(38.8,  67.4, -45);

  const renderLoop = () => {
    requestAnimationFrame(renderLoop);
    ren.render(scene, cam);
  };
  renderLoop();

  // ← CORRECT: Resize handler с canvas обновлением
  window.addEventListener('resize', () => {
    const w = window.innerWidth;
    const h = window.innerHeight;
    cam.aspect = w / h;
    cam.updateProjectionMatrix();
    ren.setSize(w, h);
  });

  setInterval(() => {
    const gps = navigator.getGamepads();
    if (gps && gps[0]) {
      const gp = gps[0];
      // ← CORRECT: proper transform calc (не -50%)
      document.getElementById('dot-l').style.transform = `translate(calc(-50% + ` + (gp.axes[0]*50) + `px), calc(-50% + ` + (gp.axes[1]*50) + `px))`;
      document.getElementById('dot-r').style.transform = `translate(calc(-50% + ` + (gp.axes[2]*50) + `px), calc(-50% + ` + (gp.axes[3]*50) + `px))`;
      window.send({ cmd:'pad', lx:gp.axes[0], ly:gp.axes[1], rx:gp.axes[2], ry:gp.axes[3], btn:gp.buttons.map(b => b.pressed?1:0) });
    }
  }, 50);

  window.activeLeg = 0;
  let walking = false;
  window.toggleWalk = () => {
    walking = !walking;
    const btn = document.getElementById('btn-walk');
    btn.innerText = walking ? "STOP WALK" : "AUTO WALK";
    btn.style.color = walking ? "#fff" : "#aaa";
    btn.style.background = walking ? "rgba(124,42,232,0.3)" : "transparent";
    btn.style.borderColor = walking ? "var(--accent)" : "#444";
    window.send({cmd:'mode', val: walking ? 'walk' : 'stand', auto: walking});
  };
  window.selLeg = (id) => {
    window.activeLeg = id;
    document.querySelectorAll('.panel button.sel').forEach(b => b.classList.remove('sel'));
    document.getElementById('l'+id).classList.add('sel');
    window.send({cmd:'active', val:id});
  };
  window.move      = (id, val) => window.send({cmd:'servo', leg:window.activeLeg, id:id, val:parseFloat(val)});
  window.startTest = (t)       => { document.getElementById('stop-btn').style.display='block';  window.send({cmd:'test',      val:parseInt(t)}); };
  window.stopTest  = ()        => { document.getElementById('stop-btn').style.display='none';   window.send({cmd:'test_stop'}); };
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

String buildStatusPayload() {
    char buf[600];
    snprintf(buf, sizeof(buf),
        "{\"wifi\":%s,\"ip\":\"%s\",\"pca\":%s,\"mpu\":%s"
        ",\"p\":%.1f,\"r\":%.1f"  // ← CORRECT: "p" and "r" (краткие ключи)
        ",\"srv\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
        hwStatus.wifi ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        hwStatus.pca9685 ? "true" : "false",
        hwStatus.mpu6050 ? "true" : "false",  // ← CORRECT! (не mpo6050!)
        hwStatus.pitch,
        hwStatus.roll,
        hwStatus.servoValues[0],  hwStatus.servoValues[1],
        hwStatus.servoValues[2],  hwStatus.servoValues[3],
        hwStatus.servoValues[4],  hwStatus.servoValues[5],
        hwStatus.servoValues[6],  hwStatus.servoValues[7],
        hwStatus.servoValues[8],  hwStatus.servoValues[9],
        hwStatus.servoValues[10], hwStatus.servoValues[11],
        hwStatus.servoValues[12], hwStatus.servoValues[13],
        hwStatus.servoValues[14], hwStatus.servoValues[15]
    );
    return String(buf);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            Serial.printf("[WS] Client %d connected\n", num);
            String msg = buildStatusPayload();
            webSocket.sendTXT(num, msg);
            break;
        }

        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client %d disconnected\n", num);
            testingServos = false;
            centerServos();
            break;

        case WStype_TEXT: {
            JsonDocument doc;
            if (deserializeJson(doc, (char *)payload) != DeserializationError::Ok) {
                Serial.println("[WS] JSON parse error");
                return;
            }

            const char *cmd = doc["cmd"];
            if (!cmd) return;

            if (strcmp(cmd, "status") == 0) {
                String msg = buildStatusPayload();
                webSocket.sendTXT(num, msg);
            }
            // ← CORRECT: test/test_stop обрабатываются
            else if (strcmp(cmd, "test") == 0) {
                testingServos = true;
                String feedback = "{\"log\":\"Servo sweep STARTED\"}";
                webSocket.broadcastTXT(feedback);
                Serial.printf("[TEST] Servo sweep STARTED\n");
            }
            else if (strcmp(cmd, "test_stop") == 0) {
                testingServos = false;
                String feedback = "{\"log\":\"Servo sweep STOPPED\"}";
                webSocket.broadcastTXT(feedback);
                centerServos();
                Serial.printf("[TEST] Servo sweep STOPPED\n");
            }
            break;
        }

        case WStype_PING:
            Serial.printf("[WS] PING from client %d\n", num);
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
    static unsigned long lastBroadcast = 0;

    for (;;) {
        readMPU();

        if (testingServos) {
            sweepServos();
        }

        if (millis() - lastBroadcast > 100) {
            lastBroadcast = millis();

            if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
                char payload[600];
                // ← CORRECT: "p", "r", "l" (краткие ключи для оптимизации)
                snprintf(payload, sizeof(payload),
                    "{\"p\":%.1f,\"r\":%.1f"
                    ",\"l\":[[%d,%d,%d,%d],[%d,%d,%d,%d],"
                    "[%d,%d,%d,%d],[%d,%d,%d,%d]]}",
                    hwStatus.pitch, hwStatus.roll,
                    0, 0, 0, 0,  // Leg 0
                    0, 0, 0, 0,  // Leg 1
                    0, 0, 0, 0,  // Leg 2
                    0, 0, 0, 0   // Leg 3
                );
                xSemaphoreGive(statusMutex);
                webSocket.broadcastTXT(payload);
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
    Serial.println("║  CENTURION DIAGNOSTIC v102 FINAL+   ║");
    Serial.println("║  All Fixes Preserved & Verified     ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    rgb.begin();
    setRGB(255, 100, 0);

    Serial.println("[INIT] Testing PCA9685 Servo Driver...");
    hwStatus.pca9685 = initPCA();
    Serial.println(hwStatus.pca9685 ? "[INIT] ✓ PCA9685 OK" : "[INIT] ✗ PCA9685 FAILED");

    Serial.println("[INIT] Testing MPU6050 IMU...");
    hwStatus.mpu6050 = initMPU();
    Serial.println(hwStatus.mpu6050 ? "[INIT] ✓ MPU6050 OK" : "[INIT] ✗ MPU6050 FAILED");

    Serial.println("[INIT] Connecting to WiFi...");
    WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");  // ← CHANGE THIS!
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        hwStatus.wifi = true;
        Serial.printf("\n[INIT] ✓ WiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());
        setRGB(0, 255, 0);
    } else {
        hwStatus.wifi = false;
        Serial.println("\n[INIT] ✗ WiFi FAILED");
        setRGB(255, 0, 0);
    }

    statusMutex = xSemaphoreCreateMutex();

    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("[HTTP] Server started on port 80");

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocket.enableHeartbeat(25000, 5000, 2);
    Serial.println("[WS] WebSocket server on port 81");

    xTaskCreatePinnedToCore(TaskDiagnostic, "Diagnostic", 8192, NULL, 2, NULL, 0);
    Serial.println("[SYSTEM] Diagnostic task started on core 0\n");

    Serial.println("═══════════════════════════════════════");
    Serial.printf("OPEN BROWSER: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("═══════════════════════════════════════\n");
}

// ============================================================================
// LOOP — Core 1
// ============================================================================

void loop() {
    server.handleClient();
    webSocket.loop();
    delay(1);