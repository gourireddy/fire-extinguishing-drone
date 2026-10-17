#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// =============================================================================
// CENTRAL HARDWARE CONFIGURATION - STAGE 3: INTEGRATED SERVO & PUMP CONTROL
// Semi-Autonomous Indoor Fire-Extinguishing Drone (Vikas R&D Quadcopter)
// Target Platform: ESP32-S3-WROOM-1 (N16R8)
// Verified Environment: Arduino-ESP32 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// --- Actuator GPIO Assignments ---
// Physically located on Right Header (J2) of carrier board
#define PIN_SERVO_PAN           1    // Right Header Pin 3 (MG90S Pan PWM)
#define PIN_SERVO_TILT          2    // Right Header Pin 4 (MG90S Tilt PWM)
#define PIN_PUMP_GATE          21    // Right Header Pin 17 (MOSFET Gate, active-HIGH, 10k pull-down)

// --- Reserved Expansion & Safety Inputs (Right Header J2) ---
#define PIN_AUX_ARMING         47    // Right Header Pin 16 (Reserved: Hardware Disarm / RC Arming)
#define PIN_RESERVED_SDA       41    // Right Header Pin 6 (Reserved: Future ToF I2C SDA)
#define PIN_RESERVED_SCL       42    // Right Header Pin 5 (Reserved: Future ToF I2C SCL)

// --- Verified Camera DVP Bus Assignments (Left Header J1 & 24-Pin Socket) ---
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
// ESP32-S3 camera driver uses LCD_CAM, so LEDC Timers 0-3 are completely available.
#define SERVO_LEDC_TIMER       LEDC_TIMER_0   // Dedicated 50 Hz Servo Timer
#define SERVO_PAN_CHANNEL      LEDC_CHANNEL_0 // Pan Channel (GPIO 1)
#define SERVO_TILT_CHANNEL     LEDC_CHANNEL_1 // Tilt Channel (GPIO 2)
#define SERVO_PWM_FREQ_HZ      50             // 50 Hz (20 ms period)
#define SERVO_PWM_RES_BITS     14             // 14-bit resolution (16384 ticks / 20ms = 0.8192 ticks/µs)

// --- Servo Physical vs. Permitted Software Windows ---
// 1. Theoretical Physical Scale (Unverified Mechanical Baseline):
// MG90S theoretical full scale assumes 500 us (0 deg) to 2500 us (180 deg) -> 2000 us span.
// Theoretical slope: 180 deg / 2000 us = 0.090 deg/us.
// With 14-bit PWM: 1.2207 us/tick * 0.090 deg/us ≈ 0.11 deg/tick.
// NOTE: Actual mechanical travel is unverified and depends on linkage geometry.
#define SERVO_PHYSICAL_MIN_US     500   // 0.0° theoretical reference
#define SERVO_PHYSICAL_MAX_US    2500   // 180.0° theoretical reference
#define SERVO_PULSE_NEUTRAL_US   1500   // 90.0° mechanical center / Home

// 2. Initial Conservative Software Movement Limits (Narrow Window):
// Clamped to ±15° around neutral to protect uncalibrated physical linkages.
#define ACTIVE_PAN_MIN_US        1333   // ~75.0° initial lower bound
#define ACTIVE_PAN_MAX_US        1667   // ~105.0° initial upper bound
#define ACTIVE_TILT_MIN_US       1333   // ~75.0° initial lower bound
#define ACTIVE_TILT_MAX_US       1667   // ~105.0° initial upper bound

// 3. Slew Rate Limit (Smooth Motion Engine):
// 10 µs per 20 ms cycle = 500 µs/sec slew rate.
#define SERVO_DEFAULT_SLEW_US_PER_TICK 10.0f

// --- Water Pump Control & Safety State Machine Parameters ---
// Control Logic: Active-HIGH N-Channel MOSFET Gate
#define PUMP_ACTIVE_LEVEL        HIGH
#define PUMP_INACTIVE_LEVEL      LOW

// Timing Constraints (Configurable & Enforced by State Machine)
#define PUMP_BURST_MIN_MS          50   // Minimum detectable burst duration (ms)
#define PUMP_BURST_DEFAULT_MS     200   // Default burst duration if unspecified (ms)
#define PUMP_BURST_MAX_MS         500   // Hard non-blocking cap on single operational burst (ms)
#define PUMP_COOLDOWN_MS         3000   // Mandatory lockout cooldown period between bursts (ms)

// Communication Watchdog Timeout
// If the system is ARMED and no valid command or heartbeat is received within this window,
// the state machine immediately transitions to FAULT and cuts the pump output.
#define COMM_WATCHDOG_TIMEOUT_MS 1000   // 1.0 second failsafe window

#endif // HARDWARE_CONFIG_H
