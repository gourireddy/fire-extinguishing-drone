# Semi-Autonomous Indoor Fire-Extinguishing Drone

[![ESP32-S3](https://img.shields.io/badge/Platform-ESP32--S3-blue.svg)](https://www.espressif.com/)
[![Arduino Core](https://img.shields.io/badge/Arduino--ESP32-v3.3.12-green.svg)](https://github.com/espressif/arduino-esp32)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.6%2B-orange.svg)](https://pytorch.org/)
[![YOLOv5](https://img.shields.io/badge/Model-YOLOv5s_DFire-red.svg)](https://github.com/ultralytics/yolov5)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

An intelligent fire-suppression payload architecture designed for indoor quadcopters. This repository contains the unified **ESP32-S3 (N16R8)** embedded firmware, the **GC2145** camera streaming pipeline, an **8-point safety interlock state machine**, a **3D projectile ballistics aiming solver**, and a real-time **laptop ground station HUD** powered by YOLOv5.

---

## 🎯 Architecture Overview

- **Manual Flight Constraint:** 100% manual RC pilot flight control. The quadcopter flight controller (FC) operates independently without software modification or autonomous flight authority.
- **Payload Automation:** The payload micro-controller (ESP32-S3) autonomously controls the 2-axis pan/tilt targeting gimbal and water pump MOSFET under strict safety interlocks.
- **Physical Actuator Safety:** Supports a software-only simulation mode (`STAGE5_SIMULATE_PUMP_ONLY = true`) where GPIO 21 is physically clamped to $0.0\text{ V}$ to prevent accidental liquid discharge during development.

```
                  +----------------------------------------------+
                  |              RC Pilot (Manual)               |
                  +----------------------+-----------------------+
                                         | 2.4 GHz RC Link
                                         v
                      +--------------------------------------+
                      |      Quadcopter Flight Controller    |
                      +--------------------------------------+

           ================= PAYLOAD SUBSYSTEM =================
  +--------------------------------------------------------------------+
  |                                                                    |
  |  +------------------------+             +-----------------------+  |
  |  |  ESP32-S3-WROOM-1      |  Wi-Fi AP   | Laptop Ground Station |  |
  |  |  - GC2145 (LCD_CAM)    | <=========> | - Native MJPEG Stream |  |
  |  |  - LEDC 14-bit Servos  |  WebSocket  | - YOLOv5s Flame Base  |  |
  |  |  - GPIO 21 Pump FSM    |             | - 3D Ballistics Math  |  |
  |  +------------------------+             +-----------------------+  |
  |               |                                                    |
  |       +-------+-------+                                            |
  |       |               |                                            |
  |       v               v                                            |
  |   MG90S Servos    MOSFET Driver                                    |
  |   (Pan & Tilt)    (Water Pump)                                     |
  |                                                                    |
  +--------------------------------------------------------------------+
```

---

## 📌 Carrier Board Hardware Pinout (ESP32-S3-WROOM-1 N16R8)

### Actuators & Safety (Right Header J2)
| Function | GPIO | Pin on Carrier | Electrical Characteristics |
|---|---|---|---|
| **MG90S Pan PWM** | `GPIO 1` | Pin 3 | 50 Hz LEDC Timer 0, 14-bit resolution |
| **MG90S Tilt PWM** | `GPIO 2` | Pin 4 | 50 Hz LEDC Timer 0, 14-bit resolution |
| **Pump MOSFET Gate** | `GPIO 21` | Pin 17 | Active-HIGH, $10\text{ k}\Omega$ pull-down resistor |
| **Auxiliary Disarm Input**| `GPIO 47` | Pin 16 | Internal pull-up (Software-read arming gate) |
| **Reserved ToF I2C SDA** | `GPIO 41` | Pin 6 | Reserved for future distance sensor |
| **Reserved ToF I2C SCL** | `GPIO 42` | Pin 5 | Reserved for future distance sensor |

### GC2145 DVP Camera Bus (Left Header J1 & 24-Pin FPC)
*Driven natively by the ESP32-S3 `LCD_CAM` hardware peripheral (Zero LEDC timer conflict).*
| Signal | GPIO | Signal | GPIO |
|---|---|---|---|
| **XCLK** | `GPIO 15` | **D3** | `GPIO 10` |
| **SIOD (SDA)** | `GPIO 4` | **D4** | `GPIO 12` |
| **SIOC (SCL)** | `GPIO 5` | **D5** | `GPIO 18` |
| **VSYNC** | `GPIO 6` | **D6** | `GPIO 17` |
| **HREF** | `GPIO 7` | **D7** | `GPIO 16` |
| **PCLK** | `GPIO 13` | **D0** | `GPIO 11` |
| **D1** | `GPIO 9` | **D2** | `GPIO 8` |

---

## 🧮 3D Projectile Ballistics Model

Instead of a naive horizontal time-of-flight approximation, the aiming engine solves the **exact parametric projectile equations**:

$$\begin{cases} x(t) = v_0 \cos(\theta) t \implies t = \frac{R}{v_0 \cos\theta} \\ y(t) = v_0 \sin(\theta) t - \frac{1}{2}g t^2 \end{cases}$$

Substituting $t$ into $y(t) = H$ and expressing in terms of $u = \tan\theta$:

$$k u^2 - R u + (H + k) = 0 \quad\text{where}\quad k = \frac{g R^2}{2 v_0^2}$$

Discriminant:
$$D = R^2 \left[ 1 - \frac{2 g H}{v_0^2} - \frac{g^2 R^2}{v_0^4} \right]$$

Low-trajectory elevation angle:
$$\theta_{\text{tilt}} = \arctan\left(\frac{R - \sqrt{D}}{2k}\right), \quad \theta_{\text{pan}} = \text{atan2}(X_n, Z_n)$$

### Key Features:
- **Flame Base Extraction:** Targets the physical combustion base $(u_{\text{base}} = \frac{x_1 + x_2}{2}, v_{\text{base}} = y_{\max})$ rather than flickering flame tips or centroids.
- **Empirical Calibration Anchor Grid:** Supports bilinear interpolation from physical spray test measurements in [`scripts/calibration_config.json`](file:///c:/Users/saksh/Desktop/FED%20Project/scripts/calibration_config.json).
- **Camera-to-Nozzle Offset:** Incorporates 3D CAD translation vector $(0\text{ mm}, -40\text{ mm}, +15\text{ mm})$.

---

## 🛡️ 8-Point Autonomous Discharge Interlock

For any pump discharge (live or simulated), **ALL 8 interlock conditions** must evaluate to `TRUE`:

1. **System Arm State:** System must be explicitly armed by the operator.
2. **Fresh Target Data:** Fire detection timestamp must be $\le 200\text{ ms}$ old.
3. **Detection Confidence:** YOLO confidence must exceed $\ge 70\%$.
4. **Combustion Persistence:** Fire target must be continuously tracked for $\ge 300\text{ ms}$.
5. **Angular Alignment:** Gimbal pan/tilt pointing error must be $\le \pm 3.0^\circ$.
6. **Operating Distance Envelope:** Target must be within $1.0\text{ m} \pm 0.25\text{ m}$ ($0.75\text{ m} - 1.25\text{ m}$).
7. **Pump Cooldown Timer:** Minimum mandatory lockout cooldown of $3000\text{ ms}$ between bursts.
8. **Communication Watchdog:** WebSocket connection packet received within $\le 400\text{ ms}$.

---

## 📂 Repository Structure

```
.
├── firmware/
│   ├── stage1_gpio_diag/          # Diagnostic GPIO boot & glitch verification
│   ├── stage2_servo_pwm/          # 14-bit 50 Hz MG90S pan/tilt LEDC control
│   ├── stage3_pump_control/       # Non-blocking pump state machine with cooldown
│   ├── stage4_wifi_integration/   # Wi-Fi video streaming & WebSocket interlock
│   └── stage5_full_integration/   # Full production integrated firmware
│       ├── camera_pins.h
│       ├── hardware_config.h
│       ├── safety_interlock.h
│       ├── wifi_credentials.h
│       └── stage5_full_integration.ino
├── scripts/
│   ├── calibration_config.json    # Camera intrinsics, offsets, & empirical grid
│   ├── drone_ground_station_final.py # Live OpenCV HUD & PyTorch YOLOv5 client
│   ├── test_stage2_logic.py       # Stage 2 software test suite
│   ├── test_stage3_logic.py       # Stage 3 FSM unit tests
│   ├── test_stage4_logic.py       # Stage 4 WebSocket & safety tests
│   └── test_stage5_system.py      # Stage 5 integration test suite (21/21 passed)
├── ref docs/                      # Engineering project plans and schematics
├── view_camera.html               # Lightweight in-browser stream viewer
└── .gitignore
```

---

## 🚀 Quick Start Guide

### 1. Flash the ESP32-S3 Firmware
Using Arduino CLI:
```powershell
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,FlashMode=qio firmware/stage5_full_integration -u -p COM9
```

### 2. Run Automated Regression Tests
```powershell
python scripts/test_stage5_system.py
```

### 3. Launch the Ground Station HUD
Connect to the ESP32-S3 Wi-Fi network and execute:
```powershell
python scripts/drone_ground_station_final.py --ip 192.168.0.113
```

### Keyboard Shortcuts in HUD:
- `[A]` — **ARM** system
- `[D]` — **DISARM** system
- `[SPACE]` — Trigger simulated discharge burst
- `[E]` — Emergency Stop (**E-STOP**)
- `[F]` — Toggle Fullscreen mode
- `[Q]` / `[ESC]` — Exit application
