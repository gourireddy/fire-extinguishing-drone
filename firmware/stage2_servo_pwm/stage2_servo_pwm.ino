#include <Arduino.h>
#include "hardware_config.h"
#include "driver/ledc.h"

// =============================================================================
// STAGE 2: MG90S SERVO PWM FIRMWARE & CONSERVATIVE CALIBRATION CLI
// Semi-Autonomous Indoor Fire-Extinguishing Drone Project
// Target: ESP32-S3-WROOM-1 (N16R8) | Arduino Core 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// Active Permitted Movement Window (Microseconds)
// Initial values are strictly clamped to a conservative window (±15° around neutral)
// to prevent mechanical binding on uncalibrated brackets.
uint32_t active_pan_min_us  = ACTIVE_PAN_MIN_US;   // 1333 us (~75.0° nominal)
uint32_t active_pan_max_us  = ACTIVE_PAN_MAX_US;   // 1667 us (~105.0° nominal)
uint32_t active_tilt_min_us = ACTIVE_TILT_MIN_US;  // 1333 us (~75.0° nominal)
uint32_t active_tilt_max_us = ACTIVE_TILT_MAX_US;  // 1667 us (~105.0° nominal)

// Current and Target Pulse Widths (Microseconds)
float cur_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float cur_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us

// Slew rate limiting: Maximum pulse change in microseconds per 20 ms tick
// 10.0 us per 20 ms tick = 500.0 us/sec linear pulse slew rate
float slew_us_per_tick = SERVO_DEFAULT_SLEW_US_PER_TICK;

unsigned long last_servo_update = 0;

// Non-blocking serial input buffer
String serial_input_buffer = "";

// Convert microseconds to 14-bit LEDC duty ticks (50 Hz / 20 ms period)
// 14-bit = 16384 ticks per 20000 us -> 16384 / 20000 = 0.8192 ticks/us
inline uint32_t usToDuty(float us) {
  return (uint32_t)(us * 0.8192f + 0.5f);
}

// Convert pulse width (us) to approximate nominal angle based on standard full scale (500-2500 us = 0-180 deg)
// NOTE: Actual mechanical angle depends on linkage geometry and individual servo batch calibration.
float usToNominalAngle(float us) {
  us = constrain(us, (float)SERVO_PHYSICAL_MIN_US, (float)SERVO_PHYSICAL_MAX_US);
  return ((us - SERVO_PHYSICAL_MIN_US) / (float)(SERVO_PHYSICAL_MAX_US - SERVO_PHYSICAL_MIN_US)) * 180.0f;
}

// Helper: Check if string is a valid positive integer
bool isValidPositiveInt(const String &str, int &out_val) {
  String s = str;
  s.trim();
  if (s.length() == 0 || s.length() > 5) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) return false;
  }
  long parsed = s.toInt();
  if (parsed < 0 || parsed > 65535) return false;
  out_val = (int)parsed;
  return true;
}

// Initialize LEDC Hardware Timer for Servos
void initServos() {
  Serial.println("[INFO] Initializing LEDC Timer 0 for MG90S Servos (50 Hz, 14-bit)...");

  ledc_timer_config_t servo_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)SERVO_PWM_RES_BITS,
    .timer_num        = SERVO_LEDC_TIMER,
    .freq_hz          = SERVO_PWM_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

  // Channel 0: Pan Servo (GPIO 1)
  ledc_channel_config_t pan_ch = {
    .gpio_num   = PIN_SERVO_PAN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = SERVO_PAN_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = SERVO_LEDC_TIMER,
    .duty       = usToDuty(SERVO_PULSE_NEUTRAL_US),
    .hpoint     = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&pan_ch));

  // Channel 1: Tilt Servo (GPIO 2)
  ledc_channel_config_t tilt_ch = {
    .gpio_num   = PIN_SERVO_TILT,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = SERVO_TILT_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = SERVO_LEDC_TIMER,
    .duty       = usToDuty(SERVO_PULSE_NEUTRAL_US),
    .hpoint     = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&tilt_ch));

  cur_pan_us  = SERVO_PULSE_NEUTRAL_US;
  tgt_pan_us  = SERVO_PULSE_NEUTRAL_US;
  cur_tilt_us = SERVO_PULSE_NEUTRAL_US;
  tgt_tilt_us = SERVO_PULSE_NEUTRAL_US;

  Serial.println("[PASS] LEDC Timer 0 initialized. Initial pulse set to neutral (1500 us).");
}

// Periodic non-blocking servo position updater (Executed exactly every 20 ms)
void updateServos() {
  unsigned long now = millis();
  if (now - last_servo_update < 20) return;
  last_servo_update = now;

  // Slew Pan pulse towards target
  if (cur_pan_us < tgt_pan_us) {
    cur_pan_us = min(cur_pan_us + slew_us_per_tick, tgt_pan_us);
  } else if (cur_pan_us > tgt_pan_us) {
    cur_pan_us = max(cur_pan_us - slew_us_per_tick, tgt_pan_us);
  }

  // Slew Tilt pulse towards target
  if (cur_tilt_us < tgt_tilt_us) {
    cur_tilt_us = min(cur_tilt_us + slew_us_per_tick, tgt_tilt_us);
  } else if (cur_tilt_us > tgt_tilt_us) {
    cur_tilt_us = max(cur_tilt_us - slew_us_per_tick, tgt_tilt_us);
  }

  // Update hardware PWM duty registers
  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_PAN_CHANNEL, usToDuty(cur_pan_us));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_PAN_CHANNEL);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_TILT_CHANNEL, usToDuty(cur_tilt_us));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_TILT_CHANNEL);
}

void setTargetPanUs(float us) {
  tgt_pan_us = constrain(us, (float)active_pan_min_us, (float)active_pan_max_us);
  Serial.printf("[SERVO] Pan Target Pulse: %.0f us (Nominal: %.1f deg) [Permitted Window: %d - %d us]\n",
                tgt_pan_us, usToNominalAngle(tgt_pan_us), active_pan_min_us, active_pan_max_us);
}

void setTargetTiltUs(float us) {
  tgt_tilt_us = constrain(us, (float)active_tilt_min_us, (float)active_tilt_max_us);
  Serial.printf("[SERVO] Tilt Target Pulse: %.0f us (Nominal: %.1f deg) [Permitted Window: %d - %d us]\n",
                tgt_tilt_us, usToNominalAngle(tgt_tilt_us), active_tilt_min_us, active_tilt_max_us);
}

void setNeutral() {
  tgt_pan_us = SERVO_PULSE_NEUTRAL_US;
  tgt_tilt_us = SERVO_PULSE_NEUTRAL_US;
  Serial.println("[ACTION] Servos commanded to neutral (1500 us).");
}

void printStatus() {
  Serial.println("\n==============================================================");
  Serial.println("           CURRENT SERVO CALIBRATION & WINDOW STATUS          ");
  Serial.println("==============================================================");
  Serial.println("THEORETICAL PHYSICAL SCALE (Unverified Electrical Reference):");
  Serial.printf("  Reference Range: %d us to %d us (Center: %d us)\n",
                SERVO_PHYSICAL_MIN_US, SERVO_PHYSICAL_MAX_US, SERVO_PULSE_NEUTRAL_US);
  Serial.println("  *WARNING: Full 500-2500 us range is UNVERIFIED on physical hardware.");
  Serial.println("   Budget clone MG90S units frequently hit internal stops outside 700-2300 us.");
  Serial.println("--------------------------------------------------------------");
  Serial.println("ACTIVE PERMITTED MOVEMENT WINDOW (Safe Clamping Active):");
  Serial.printf("  PAN SERVO  (GPIO 1): Min = %d us, Max = %d us | Target = %.0f us\n",
                active_pan_min_us, active_pan_max_us, tgt_pan_us);
  Serial.printf("  TILT SERVO (GPIO 2): Min = %d us, Max = %d us | Target = %.0f us\n",
                active_tilt_min_us, active_tilt_max_us, tgt_tilt_us);
  Serial.println("--------------------------------------------------------------");
  Serial.printf("  SLEW RATE: %.1f us / 20ms tick (%.0f us/sec pulse rate)\n",
                slew_us_per_tick, slew_us_per_tick * 50.0f);
  Serial.println("  PUMP PIN   (GPIO 21): LOW (0.0V) | STATUS: PERMANENTLY LOCKED INACTIVE");
  Serial.println("==============================================================\n");
}

void printHelp() {
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("       STAGE 2: CONSERVATIVE SERVO CALIBRATION CLI            ");
  Serial.println("--------------------------------------------------------------");
  Serial.println("Incremental Step Commands (Check physical clearance step-by-step):");
  Serial.println("  p+5 / p-5      - Step Pan target by +55 us or -55 us");
  Serial.println("  t+5 / t-5      - Step Tilt target by +55 us or -55 us");
  Serial.println("  pus <us>       - Set Pan pulse in microseconds within active window");
  Serial.println("  tus <us>       - Set Tilt pulse in microseconds within active window");
  Serial.println("  neutral        - Move both servos back to 1500 us");
  Serial.println("\nActive Window Adjustments (Validated before acceptance):");
  Serial.println("  min_p <us>     - Expand/contract Pan lower limit (>= 500 us, < max_p - 100)");
  Serial.println("  max_p <us>     - Expand/contract Pan upper limit (<= 2500 us, > min_p + 100)");
  Serial.println("  min_t <us>     - Expand/contract Tilt lower limit (>= 500 us, < max_t - 100)");
  Serial.println("  max_t <us>     - Expand/contract Tilt upper limit (<= 2500 us, > min_t + 100)");
  Serial.println("  status         - Display active window and current pulse values");
  Serial.println("  help           - Show this command reference");
  Serial.println("--------------------------------------------------------------");
  Serial.println("HARDWARE ADVISORY: Servos typically exhibit a brief mechanical");
  Serial.println("jerk/twitch when power is first applied before decoding PWM.");
  Serial.println("Keep fingers and physical linkages clear during power-up.");
  Serial.println("--------------------------------------------------------------\n");
}

// Validate and process Serial commands
void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
    printHelp();
  } else if (cmd.equalsIgnoreCase("status") || cmd.equalsIgnoreCase("cal")) {
    printStatus();
  } else if (cmd.equalsIgnoreCase("neutral")) {
    setNeutral();
  } else if (cmd.equalsIgnoreCase("p+5")) {
    setTargetPanUs(tgt_pan_us + 55.0f);
  } else if (cmd.equalsIgnoreCase("p-5")) {
    setTargetPanUs(tgt_pan_us - 55.0f);
  } else if (cmd.equalsIgnoreCase("t+5")) {
    setTargetTiltUs(tgt_tilt_us + 55.0f);
  } else if (cmd.equalsIgnoreCase("t-5")) {
    setTargetTiltUs(tgt_tilt_us - 55.0f);
  } else if (cmd.startsWith("pus ") || cmd.startsWith("PUS ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(4), val)) {
      Serial.println("[REJECTED] Malformed input. Value must be a valid positive integer in microseconds.");
    } else if (val < (int)active_pan_min_us || val > (int)active_pan_max_us) {
      Serial.printf("[REJECTED] Pulse %d us is outside current active window [%d - %d us].\n",
                    val, active_pan_min_us, active_pan_max_us);
    } else {
      setTargetPanUs((float)val);
    }
  } else if (cmd.startsWith("tus ") || cmd.startsWith("TUS ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(4), val)) {
      Serial.println("[REJECTED] Malformed input. Value must be a valid positive integer in microseconds.");
    } else if (val < (int)active_tilt_min_us || val > (int)active_tilt_max_us) {
      Serial.printf("[REJECTED] Pulse %d us is outside current active window [%d - %d us].\n",
                    val, active_tilt_min_us, active_tilt_max_us);
    } else {
      setTargetTiltUs((float)val);
    }
  } else if (cmd.startsWith("min_p ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(6), val)) {
      Serial.println("[REJECTED] Malformed input. Min value must be a positive integer.");
    } else if (val < SERVO_PHYSICAL_MIN_US) {
      Serial.printf("[REJECTED] Value %d us is below absolute physical floor (%d us).\n", val, SERVO_PHYSICAL_MIN_US);
    } else if (val >= (int)active_pan_max_us - 100) {
      Serial.printf("[REJECTED] Reversed or narrow limit: %d us must be at least 100 us below max (%d us).\n",
                    val, active_pan_max_us);
    } else {
      active_pan_min_us = val;
      Serial.printf("[UPDATED] Active Pan Min Limit set to: %d us\n", active_pan_min_us);
    }
  } else if (cmd.startsWith("max_p ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(6), val)) {
      Serial.println("[REJECTED] Malformed input. Max value must be a positive integer.");
    } else if (val > SERVO_PHYSICAL_MAX_US) {
      Serial.printf("[REJECTED] Value %d us exceeds absolute physical ceiling (%d us).\n", val, SERVO_PHYSICAL_MAX_US);
    } else if (val <= (int)active_pan_min_us + 100) {
      Serial.printf("[REJECTED] Reversed or narrow limit: %d us must be at least 100 us above min (%d us).\n",
                    val, active_pan_min_us);
    } else {
      active_pan_max_us = val;
      Serial.printf("[UPDATED] Active Pan Max Limit set to: %d us\n", active_pan_max_us);
    }
  } else if (cmd.startsWith("min_t ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(6), val)) {
      Serial.println("[REJECTED] Malformed input. Min value must be a positive integer.");
    } else if (val < SERVO_PHYSICAL_MIN_US) {
      Serial.printf("[REJECTED] Value %d us is below absolute physical floor (%d us).\n", val, SERVO_PHYSICAL_MIN_US);
    } else if (val >= (int)active_tilt_max_us - 100) {
      Serial.printf("[REJECTED] Reversed or narrow limit: %d us must be at least 100 us below max (%d us).\n",
                    val, active_tilt_max_us);
    } else {
      active_tilt_min_us = val;
      Serial.printf("[UPDATED] Active Tilt Min Limit set to: %d us\n", active_tilt_min_us);
    }
  } else if (cmd.startsWith("max_t ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(6), val)) {
      Serial.println("[REJECTED] Malformed input. Max value must be a positive integer.");
    } else if (val > SERVO_PHYSICAL_MAX_US) {
      Serial.printf("[REJECTED] Value %d us exceeds absolute physical ceiling (%d us).\n", val, SERVO_PHYSICAL_MAX_US);
    } else if (val <= (int)active_tilt_min_us + 100) {
      Serial.printf("[REJECTED] Reversed or narrow limit: %d us must be at least 100 us above min (%d us).\n",
                    val, active_tilt_min_us);
    } else {
      active_tilt_max_us = val;
      Serial.printf("[UPDATED] Active Tilt Max Limit set to: %d us\n", active_tilt_max_us);
    }
  } else {
    Serial.printf("[ERROR] Unrecognized command: '%s'. Type 'help' for command reference.\n", cmd.c_str());
  }
}

void setup() {
  // ===========================================================================
  // STEP 0: IMMEDIATE SAFETY CLAMPING (EXECUTED BEFORE SERIAL/DELAYS)
  // Preload and lock Pump output LOW to ensure it stays permanently off.
  // ===========================================================================
  digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
  pinMode(PIN_PUMP_GATE, OUTPUT);
  digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);

  // Initialize USB-UART Serial interface
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n==============================================================");
  Serial.println("  ESP32-S3 Stage 2: MG90S Servo PWM & Conservative Cal CLI    ");
  Serial.println("  Core: Arduino-ESP32 3.3.12 (ESP-IDF 5.5.5)                 ");
  Serial.println("==============================================================");
  Serial.println("[SAFETY] GPIO 21 (Pump Gate) preloaded and locked LOW (0.0V).");
  Serial.println("[SAFETY] Pump remains permanently locked inactive throughout Stage 2.");

  // Initialize Servos on LEDC Timer 0
  initServos();

  printStatus();
  printHelp();
}

void loop() {
  // Non-blocking serial character collector (never blocks servo update task)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_input_buffer.length() > 0) {
        parseCommand(serial_input_buffer);
        serial_input_buffer = "";
      }
    } else {
      if (serial_input_buffer.length() < 64) {
        serial_input_buffer += c;
      }
    }
  }

  // Update servo PWM with smooth slew rate limiting every 20 ms
  updateServos();
}
