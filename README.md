
# ARTTOUS | Autonomous Mesh Reality Platform

> **"Where wheels fail, Arttous walks."**

## /// MISSION PROFILE

**Arttous** is an open-source, 16-DOF (Degree of Freedom) quadrupedal robotics platform designed for navigation in GNSS-denied environments. Unlike traditional rovers that rely on satellite GPS, Arttous generates its own local reality using a **T3S3 LoRa Mesh** network to triangulate position via Time-of-Flight (ToF) ranging.

This repository contains the **Web Telemetry Interface (Mission Control)**, documentation, and access portals for the advanced RTOS Firmware series.

---

## /// SYSTEM ARCHITECTURE (v75.0 RTOS EDITION)

With the transition to v75.0, the platform utilizes a **Dual-Core FreeRTOS** architecture to prevent I2C starvation and ensure strict 50Hz deadlines for the Inverse Kinematics (IK) engine.

* **CORE 0 (TaskNetwork):** Handles WiFi, HTTP Web Server, and 100ms WebSockets JSON telemetry broadcasts.
* **CORE 1 (TaskControl):** Dedicated strictly to the IK Math Engine, IMU sensor fusion, and Gait Scheduling.
* **Memory Protection:** Cross-core data transfer is secured via `SemaphoreHandle_t stateMutex`.

---

## /// HARDWARE DEEP DIVE: BUDGET vs. PREMIUM

To make Arttous accessible for distributed manufacturing, we heavily optimized the Bill of Materials (BOM). Below is a detailed breakdown of how each component works, why we chose it, and what the "military-grade" expensive alternative would be.

### 1. Central Processing Unit (The Brain)

* **Currently Using:** **ESP32-S3 (N8R8) Microcontroller (~$8)**
* **How it works:** A 32-bit dual-core processor with WiFi/Bluetooth capabilities. We use its two cores to separate network traffic from real-time physics calculations. It directly controls the dual I2C buses.
* **The Premium Alternative:** *Nvidia Jetson Nano or Raspberry Pi Compute Module 4 ($150 - $200)*
* **Why the alternative is better:** A Jetson would allow full ROS2 (Robot Operating System) integration, onboard AI vision (running YOLO for object detection), and SLAM mapping.
* **Why we didn't use it:** Massive power draw requiring huge batteries, extreme heat output, and overkill for pure kinematic locomotion.



### 2. Actuators (The Muscles)

* **Currently Using:** **16x MG90S Micro Servos (~$3 each / $48 total)**
* **How it works:** These are simple DC motors with a built-in potentiometer and a tiny control board. They receive a PWM (Pulse Width Modulation) signal and rotate to a specific angle. The metal gears provide decent torque for a lightweight chassis.
* **The Premium Alternative:** *Quasi-Direct Drive (QDD) Brushless Motors (e.g., Moteus or MIT Mini Cheetah actuators) ($100 - $250 EACH / $1,600+ total)*
* **Why the alternative is better:** QDD Brushless motors provide **Proprioception** (they can "feel" the ground by sensing torque/current). They have zero backlash, infinite rotation, and can jump or backflip.
* **Why we didn't use it:** Prohibitively expensive and requires highly complex FOC (Field Oriented Control) drivers for every single joint.



### 3. Servo Controller (The Nervous System)

* **Currently Using:** **PCA9685 16-Channel PWM Driver (~$5)**
* **How it works:** Instead of the ESP32 generating 16 separate PWM signals (which causes CPU jitter and leg twitching), the ESP32 sends a single I2C command to this board. The PCA9685 then generates perfectly stable hardware PWM for all 16 servos simultaneously.
* **The Premium Alternative:** *RS485 / CAN Bus Daisy-Chained Smart Servos (e.g., Dynamixel X-Series) ($50+ per servo)*
* **Why the alternative is better:** Eliminates the "spaghetti wiring" problem. You run a single cable through the entire leg, and each servo reads its specific packet. They also report their temperature and exact physical position back to the MCU.
* **Why we didn't use it:** Cost. Dynamixels would push the project budget well over $800.



### 4. Inertial Measurement Unit (The Inner Ear)

* **Currently Using:** **MPU6050 6-DOF Sensor (~$3)**
* **How it works:** Uses microscopic MEMS (Micro-Electromechanical Systems) to detect acceleration (gravity) and rotational velocity. The firmware uses this data (Pitch and Roll) to actively compensate the Z-axis of each leg, keeping the body level on uneven terrain.
* **The Premium Alternative:** *BNO085 or Industrial RTK IMU ($30 - $150)*
* **Why the alternative is better:** The MPU6050 suffers from "drift" over time and requires constant manual calibration. The BNO085 has an internal ARM Cortex processor that fuses data onboard, providing absolute orientation (quaternions) with zero drift and immunity to magnetic interference.



### 5. Localization (The Senses)

* **Currently Using:** **LilyGo T3S3 SX1280 2.4GHz LoRa (~$20)**
* **How it works:** Uses Time-of-Flight (ToF). It measures exactly how many nanoseconds a radio wave takes to bounce between nodes, calculating distance without needing satellites.
* **The Premium Alternative:** *Ultra-Wideband (UWB) modules (e.g., DWM1000) or RTK GPS ($50 - $300)*
* **Why the alternative is better:** UWB provides centimeter-level accuracy (<10cm) indoors, compared to LoRa's ~1-2 meter accuracy. RTK GPS provides millimeter accuracy outdoors.
* **Why we didn't use it:** UWB requires a dense array of anchors placed around the room. LoRa provides a massive range (kilometers) and acceptable accuracy for a mesh network outdoors.



---

## /// PROJECT STRUCTURE

```bash
ARTTOUS_ROOT/
│
├── index.html          # MISSION CONTROL (Landing Page & 3D Hero)
├── tracking.html       # LIVE TELEMETRY (Radar & Graphs)
├── firmware.html       # DRIVER HUB (Download & Changelog)
├── docs.html           # DOCUMENTATION (API & Pinouts)
├── account.html        # OPERATOR UPLINK (Profile, Missions, FPV)
│
└── assets/
    ├── css/            # Core stylesheets (Sci-Fi / Cyberpunk UI)
    ├── js/             # Three.js Digital Twin logic
    ├── models/         # STL Files & GLB Renders
    └── images/         # Texture maps & mechanical photos

```

---

## /// GETTING STARTED

### Prerequisites

To run the Mission Control interface locally and render the Three.js models, you must run a local HTTP server to bypass browser CORS restrictions.

### Installation

1. **Clone the Repository:**

```bash
git clone https://github.com/sa-mael/robo-web-ps4.git
cd robo-web-ps4

```

2. **Launch Local Server:**

* **VS Code:** Install the "Live Server" extension and click "Go Live".
* **Python:**

```bash
python3 -m http.server 8000

```

* **Node.js:**

```bash
npx serve .

```

3. **Access:**
Open your browser and navigate to `http://localhost:8000`.

---

## /// FABRICATION & ASSEMBLY

The platform is designed for distributed manufacturing using standard FDM 3D printers.

* **Chassis Core:** Print in **PETG-CF** (Carbon Fiber) for maximum rigidity against the torque of 16 servos.
* **Leg Links (Femur/Tibia):** Print in **ASA** or **ABS** for high impact resistance.
* **Foot Pads:** Print in **TPU 95A** for terrain traction and shock absorption.

> **WARNING:** Ensure printer bed leveling is calibrated to < 0.1mm. Dimensional accuracy is critical for servo horn fitment. See `docs.html` for specific orientation blueprints.

---

https://github.com/user-attachments/assets/150d3ed4-cd5a-4805-b0cd-df4852e4e3df





https://github.com/user-attachments/assets/111de0a9-df7d-4ebe-b7d9-911710fa93a3



## /// CONTRIBUTING

We welcome operators of all ranks. Check the **Mission Board** in your `account.html` dashboard for Open Contracts.

1. Fork the repo and create a branch (`git checkout -b feature/AdvancedGait`).
2. Commit your changes (`git commit -m 'Add new Trot Gait algorithm'`).
3. Push to the branch (`git push origin feature/AdvancedGait`).
4. Open a Pull Request.

---

## /// DISCLAIMER & LEGAL

**ARTTOUS** is an experimental hardware/software platform.

* **Safety:** High-torque actuators can pinch or crush fingers. Lithium Polymer (LiPo) batteries pose a fire risk if over-discharged or punctured. Always use a low-voltage alarm.
* **Liability:** By using this software, you agree to the terms outlined in the platform. The creators assume no responsibility for hardware damage or physical injury.
* **Usage:** Weaponization of this platform is strictly prohibited.

---

### END OF LINE.

© 2026 Arttous Robotics Inc. | *Ontario, Canada*

*You are ready to deploy. Good luck, Operator. Where wheels fail, you will walk.*
