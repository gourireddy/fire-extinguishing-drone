#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// =============================================================================
// CENTRAL HARDWARE CONFIGURATION - STAGE 5: FULL-SYSTEM INTEGRATION & SIMULATION
// Semi-Autonomous Indoor Fire-Extinguishing Drone (Vikas R&D Quadcopter)
// Target Platform: ESP32-S3-WROOM-1 (N16R8)
// Verified Environment: Arduino-ESP32 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// --- Actuator GPIO Assignments ---
// Physically located on Right Header (J2) of carrier board
#define PIN_SERVO_PAN           1    // Right Header Pin 3 (MG90S Pan PWM)
#define PIN_SERVO_TILT          2    // Right Header Pin 4 (MG90S Tilt PWM)
#define PIN_PUMP_GATE          21    // Right Header Pin 17 (MOSFET Gate, active-HIGH, 10k pull-down)

// --- Reserved Expansion & Auxiliary Inputs (Right Header J2) ---
#define PIN_AUX_ARMING         47    // Right Header Pin 16 (Software Read RC Arming Input)
#define PIN_RESERVED_SDA       41    // Right Header Pin 6 (Reserved: Future ToF I2C SDA)
#define PIN_RESERVED_SCL       42    // Right Header Pin 5 (Reserved: Future ToF I2C SCL)

// --- Camera DVP Bus Assignments (Left Header J1 & 24-Pin Socket) ---
// Driven by ESP32-S3 LCD_CAM hardware peripheral (Zero LEDC timer usage)
#define PIN_CAM_XCLK           15    // Camera Master Clock (LCD_CAM clock generator)
#define PIN_CAM_SIOD            4    // SCCB Data (I2C SDA)
#define PIN_CAM_SIOC            5    // SCCB Clock (I2C SCL)
#define PIN_CAM_VSYNC           6    // Vertical Sync
#define PIN_CAM_HREF            7    // Horizontal Reference
#define PIN_CAM_PCLK           13    // Pixel Clock
#define PIN_CAM_D0             11    // Data Bit 0
#define PIN_CAM_D1              9    // Data Bit 1
#define PIN_CAM_D2              8    // Data Bit 2
#define PIN_CAM_D3             10    // Data Bit 3
#define PIN_CAM_D4             12    // Data Bit 4
#define PIN_CAM_D5             18    // Data Bit 5
#define PIN_CAM_D6             17    // Data Bit 6
#define PIN_CAM_D7             16    // Data Bit 7

// --- LEDC PWM Allocation for Servos ---
#define SERVO_LEDC_TIMER       LEDC_TIMER_0   // Dedicated 50 Hz Servo Timer
#define SERVO_PAN_CHANNEL      LEDC_CHANNEL_0 // Pan Channel (GPIO 1)
#define SERVO_TILT_CHANNEL     LEDC_CHANNEL_1 // Tilt Channel (GPIO 2)
#define SERVO_PWM_FREQ_HZ      50             // 50 Hz (20 ms period)
#define SERVO_PWM_RES_BITS     14             // 14-bit resolution (16384 ticks / 20ms = 0.8192 ticks/µs)

// --- Servo Physical vs. Permitted Software Windows ---
// Theoretical electrical scale (unverified mechanical baseline, configurable):
#define SERVO_PHYSICAL_MIN_US     500   // 0.0° theoretical reference
#define SERVO_PHYSICAL_MAX_US    2500   // 180.0° theoretical reference
#define SERVO_PULSE_NEUTRAL_US   1500   // 90.0° mechanical center / Home

// Initial Conservative Software Movement Limits (Narrow Window):
// Clamped to ±15° around neutral to protect uncalibrated physical linkages.
#define ACTIVE_PAN_MIN_US        1333   // ~75.0° initial lower bound
#define ACTIVE_PAN_MAX_US        1667   // ~105.0° initial upper bound
#define ACTIVE_TILT_MIN_US       1333   // ~75.0° initial lower bound
#define ACTIVE_TILT_MAX_US       1667   // ~105.0° initial upper bound

// Slew Rate Limit (Smooth Motion Engine):
#define SERVO_DEFAULT_SLEW_US_PER_TICK 10.0f  // 10 µs per 20 ms cycle = 500 µs/sec slew rate

// Estimated Servo Settling Time (Open-loop mechanical wait buffer):
#define SERVO_ESTIMATED_SETTLING_MS   150     // 150 ms mechanical settling buffer

// --- Water Pump Control & Safety Policy Parameters ---
#define PUMP_ACTIVE_LEVEL        HIGH
#define PUMP_INACTIVE_LEVEL      LOW

// STAGE 5 SAFETY CONSTRAINT: Physical Pump Drive Disabled in Simulation Mode
// GPIO 21 is physically locked LOW (0.0V) to prevent accidental discharge
// while software state machine, telemetry, and interlocks operate realistically.
#define STAGE5_SIMULATE_PUMP_ONLY true

// Timing Constraints
#define PUMP_BURST_MIN_MS          50   // Minimum burst duration (ms)
#define PUMP_BURST_DEFAULT_MS     200   // Default burst duration (ms)
#define PUMP_BURST_MAX_MS         500   // Hard cap on single operational burst (ms)
#define PUMP_COOLDOWN_MS         3000   // Mandatory lockout cooldown period (ms)

// --- Dual-Tier Timeout Policy ---
// Tier 1: Target Freshness Timeout (fire detection data must be refreshed every 200 ms)
#define TARGET_FRESHNESS_TIMEOUT_MS  200  // 200 ms target freshness window

// Tier 2: Link Watchdog Timeout (tightened to 400 ms)
#define COMM_WATCHDOG_TIMEOUT_MS     400  // 400 ms link watchdog window

// --- Autonomous Discharge Safety Interlock Thresholds ---
#define FIRE_CONFIDENCE_THRESHOLD    0.70f // 70% minimum YOLO detection confidence
#define FIRE_PERSISTENCE_MIN_MS      300   // Fire must be detected continuously for >= 300 ms
#define TARGET_ALIGN_TOLERANCE_DEG   3.0f  // Pan & Tilt angular alignment error <= ±3.0°

// --- Distance Envelope & Calibrated Fixed-Distance Mode ---
// Fixed calibrated operating distance (provisional until physical ToF sensor is integrated)
#define CALIBRATED_TARGET_DIST_M     1.0f  // 1.0 meter nominal calibrated test distance
#define CALIBRATED_DIST_TOLERANCE_M  0.25f // Valid distance band: 0.75m to 1.25m
#define TARGET_DISTANCE_MIN_M        0.5f  // Absolute hard floor (meters)
#define TARGET_DISTANCE_MAX_M        2.0f  // Absolute hard ceiling (meters)

// --- Camera-to-Nozzle Physical Offset Calibration ---
// Physical vector from camera optical center to nozzle exit orifice (in millimeters)
#define NOZZLE_OFFSET_X_MM           0.0f  // Lateral offset (mm)
#define NOZZLE_OFFSET_Y_MM         -40.0f  // Nozzle mounted 40 mm below camera lens (-Y in camera frame)
#define NOZZLE_OFFSET_Z_MM          15.0f  // Nozzle tip extends 15 mm forward of lens (+Z)

// Ballistic Water Jet Characteristics (Configurable)
#define WATER_JET_VELOCITY_MPS       4.0f  // Estimated exit velocity (meters/sec)
#define GRAVITY_ACCEL_MPS2           9.81f // Gravity (m/s^2)

// --- Configurable Network Authentication ---
#define AUTH_CONFIG_TOKEN            "DRONE_SECURE_AUTH_STAGE5_2026"

#endif // HARDWARE_CONFIG_H
