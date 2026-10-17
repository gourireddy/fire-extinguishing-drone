#include <Arduino.h>
#include "hardware_config.h"
#include "pump_fsm.h"
#include "driver/ledc.h"

// =============================================================================
// STAGE 3: INTEGRATED PAN/TILT SERVO & WATER PUMP SAFETY CONTROL FIRMWARE
// Semi-Autonomous Indoor Fire-Extinguishing Drone Project
// Target: ESP32-S3-WROOM-1 (N16R8) | Arduino Core 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// Pump Safety State Machine Instance
PumpController pump;

// --- Servo Subsystem Parameters (Inherited from Stage 2) ---
uint32_t active_pan_min_us  = ACTIVE_PAN_MIN_US;   // 1333 us
uint32_t active_pan_max_us  = ACTIVE_PAN_MAX_US;   // 1667 us
uint32_t active_tilt_min_us = ACTIVE_TILT_MIN_US;  // 1333 us
uint32_t active_tilt_max_us = ACTIVE_TILT_MAX_US;  // 1667 us

float cur_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float cur_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_pan_us  = SERVO_PULSE_NEUTRAL_US; // 1500 us
float tgt_tilt_us = SERVO_PULSE_NEUTRAL_US; // 1500 us

float slew_us_per_tick = SERVO_DEFAULT_SLEW_US_PER_TICK; // 10 us / 20ms tick
unsigned long last_servo_update = 0;

// Non-blocking serial input buffer
String serial_input_buffer = "";

// Convert microseconds to 14-bit LEDC duty ticks (50 Hz / 20 ms period)
// 14-bit = 16384 ticks per 20000 us -> 16384 / 20000 = 0.8192 ticks/us
inline uint32_t usToDuty(float us) {
  return (uint32_t)(us * 0.8192f + 0.5f);
}

// Convert pulse width (us) to approximate nominal angle based on theoretical full scale (500-2500 us = 0-180 deg)
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
  Serial.printf("[SERVO] Pan Target: %.0f us (Nominal: %.1f deg) [Window: %d - %d us]\n",
                tgt_pan_us, usToNominalAngle(tgt_pan_us), active_pan_min_us, active_pan_max_us);
}

void setTargetTiltUs(float us) {
  tgt_tilt_us = constrain(us, (float)active_tilt_min_us, (float)active_tilt_max_us);
  Serial.printf("[SERVO] Tilt Target: %.0f us (Nominal: %.1f deg) [Window: %d - %d us]\n",
                tgt_tilt_us, usToNominalAngle(tgt_tilt_us), active_tilt_min_us, active_tilt_max_us);
}

void setNeutral() {
  tgt_pan_us = SERVO_PULSE_NEUTRAL_US;
  tgt_tilt_us = SERVO_PULSE_NEUTRAL_US;
  Serial.println("[ACTION] Servos commanded to neutral (1500 us).");
}

void printStatus() {
  Serial.println("\n==============================================================");
  Serial.println("           INTEGRATED SERVO & PUMP SUBSYSTEM STATUS           ");
  Serial.println("==============================================================");
  Serial.printf("  PAN SERVO  (GPIO 1): %.0f us (Current) -> %.0f us (Target) [Window: %d-%d us]\n",
                cur_pan_us, tgt_pan_us, active_pan_min_us, active_pan_max_us);
  Serial.printf("  TILT SERVO (GPIO 2): %.0f us (Current) -> %.0f us (Target) [Window: %d-%d us]\n",
                cur_tilt_us, tgt_tilt_us, active_tilt_min_us, active_tilt_max_us);
  Serial.println("--------------------------------------------------------------");
  Serial.printf("  PUMP STATE (GPIO 21): %s\n", pump.getStateStr());
  Serial.printf("  PUMP FAULT REASON  : %s\n", pump.getFaultReasonStr());
  Serial.printf("  WATCHDOG STATUS    : %s (Timeout: %d ms)\n",
                pump.isWatchdogEnabled() ? "ENABLED" : "DISABLED", COMM_WATCHDOG_TIMEOUT_MS);
  if (pump.getState() == PUMP_STATE_BURSTING) {
    Serial.printf("  BURST IN PROGRESS  : %u ms remaining\n", pump.getRemainingBurstMs());
  } else if (pump.getState() == PUMP_STATE_COOLDOWN) {
    Serial.printf("  COOLDOWN LOCKOUT   : %u ms remaining\n", pump.getRemainingCooldownMs());
  }
  Serial.println("--------------------------------------------------------------");
  Serial.println("SAFETY STATUS: Physical actuators & MOSFET driver must remain");
  Serial.println("DISCONNECTED until external supply and GPIO validation are complete.");
  Serial.println("==============================================================\n");
}

void printHelp() {
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("   STAGE 3: INTEGRATED SERVO & PUMP CONTROL COMMAND MENU      ");
  Serial.println("--------------------------------------------------------------");
  Serial.println("PUMP CONTROL COMMANDS:");
  Serial.println("  arm            - Arm pump safety FSM (enables firing & watchdog)");
  Serial.println("  disarm         - Disarm pump (locks output LOW, clears faults)");
  Serial.println("  estop          - Emergency Stop (immediate shutdown into FAULT)");
  Serial.println("  fire [ms]      - Request single-shot burst (50-500 ms, default 200 ms)");
  Serial.println("  hb / ping      - Refresh communication watchdog heartbeat");
  Serial.println("\nSERVO COMMANDS:");
  Serial.println("  p+5 / p-5      - Step Pan by +55 us or -55 us");
  Serial.println("  t+5 / t-5      - Step Tilt by +55 us or -55 us");
  Serial.println("  pus <us>       - Set Pan pulse in microseconds");
  Serial.println("  tus <us>       - Set Tilt pulse in microseconds");
  Serial.println("  neutral        - Move both servos back to 1500 us");
  Serial.println("  min_p / max_p  - Reconfigure Pan active limits (500-2500 us)");
  Serial.println("  min_t / max_t  - Reconfigure Tilt active limits (500-2500 us)");
  Serial.println("\nSYSTEM COMMANDS:");
  Serial.println("  status         - Display full system status");
  Serial.println("  help           - Display this reference menu");
  Serial.println("--------------------------------------------------------------\n");
}

// Process single validated serial command string
void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Always refresh watchdog on valid serial activity
  pump.feedWatchdog();

  if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
    printHelp();
  } else if (cmd.equalsIgnoreCase("status")) {
    printStatus();
  } else if (cmd.equalsIgnoreCase("arm")) {
    PumpCommandResult res = pump.arm();
    if (res == RESULT_OK) {
      Serial.println("[PUMP] System ARMED. Watchdog active. Ready for single-shot discharge.");
    } else if (res == ERR_IN_FAULT) {
      Serial.printf("[REJECTED] Cannot arm: System in FAULT (%s). Call 'disarm' to reset.\n", pump.getFaultReasonStr());
    } else {
      Serial.println("[REJECTED] Cannot arm in current state.");
    }
  } else if (cmd.equalsIgnoreCase("disarm")) {
    pump.disarm();
    Serial.println("[PUMP] System DISARMED. Output clamped LOW (0.0V). All faults cleared.");
  } else if (cmd.equalsIgnoreCase("estop")) {
    pump.emergencyStop();
    Serial.println("[EMERGENCY] PUMP EMERGENCY STOP TRIGGERED! Output clamped LOW. State: FAULT.");
  } else if (cmd.equalsIgnoreCase("hb") || cmd.equalsIgnoreCase("ping")) {
    pump.feedWatchdog();
    Serial.println("[WATCHDOG] Heartbeat acknowledged.");
  } else if (cmd.startsWith("fire") || cmd.startsWith("FIRE")) {
    uint32_t burst_ms = PUMP_BURST_DEFAULT_MS;
    if (cmd.length() > 4) {
      int parsed = 0;
      if (!isValidPositiveInt(cmd.substring(4), parsed)) {
        Serial.println("[REJECTED] Malformed duration. Syntax: 'fire' or 'fire <ms>'.");
        return;
      }
      burst_ms = (uint32_t)parsed;
    }

    PumpCommandResult res = pump.requestBurst(burst_ms);
    switch (res) {
      case RESULT_OK:
        Serial.printf("[PUMP] Burst INITIATED: %u ms (GPIO 21 -> HIGH)\n", burst_ms);
        break;
      case ERR_NOT_ARMED:
        Serial.println("[REJECTED] Fire command ignored: Pump is DISARMED. Send 'arm' first.");
        break;
      case ERR_IN_COOLDOWN:
        Serial.printf("[REJECTED] Fire command ignored: In COOLDOWN lockout (%u ms remaining).\n", pump.getRemainingCooldownMs());
        break;
      case ERR_ALREADY_BURSTING:
        Serial.println("[REJECTED] Fire command ignored: Burst already active.");
        break;
      case ERR_IN_FAULT:
        Serial.printf("[REJECTED] Fire command ignored: In FAULT state (%s). Send 'disarm' to clear.\n", pump.getFaultReasonStr());
        break;
      case ERR_INVALID_DURATION:
        Serial.printf("[REJECTED] Invalid duration %u ms. Allowed range: %d to %d ms.\n",
                      burst_ms, PUMP_BURST_MIN_MS, PUMP_BURST_MAX_MS);
        break;
    }
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
      Serial.println("[REJECTED] Malformed input. Pulse must be a positive integer in microseconds.");
    } else if (val < (int)active_pan_min_us || val > (int)active_pan_max_us) {
      Serial.printf("[REJECTED] Pulse %d us is outside active window [%d - %d us].\n",
                    val, active_pan_min_us, active_pan_max_us);
    } else {
      setTargetPanUs((float)val);
    }
  } else if (cmd.startsWith("tus ") || cmd.startsWith("TUS ")) {
    int val = 0;
    if (!isValidPositiveInt(cmd.substring(4), val)) {
      Serial.println("[REJECTED] Malformed input. Pulse must be a positive integer in microseconds.");
    } else if (val < (int)active_tilt_min_us || val > (int)active_tilt_max_us) {
      Serial.printf("[REJECTED] Pulse %d us is outside active window [%d - %d us].\n",
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
  // STEP 0: IMMEDIATE HARDWARE SAFETY CLAMPING (BEFORE SERIAL OR TIMERS)
  // Lock pump gate output LOW and initialize state machine in DISARMED state.
  // ===========================================================================
  pump.init();

  // Initialize USB-UART Serial interface
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n==============================================================");
  Serial.println("   STAGE 3: INTEGRATED SERVO & PUMP SAFETY FIRMWARE          ");
  Serial.println("   Platform: ESP32-S3-WROOM-1 | Arduino Core 3.3.12 (ESP-IDF 5.5.5)");
  Serial.println("==============================================================");
  Serial.println("[SAFETY] GPIO 21 preloaded LOW (0.0V). Pump State: DISARMED.");
  Serial.println("[SAFETY] Hard burst limit: 500 ms | Mandatory Cooldown: 3000 ms.");
  Serial.println("[SAFETY] Communication Watchdog: 1000 ms failsafe trip.");

  // Initialize Servos on LEDC Timer 0
  initServos();
  Serial.println("[PASS] Servos initialized to safe neutral (1500 us).");

  printStatus();
  printHelp();
}

void loop() {
  // 1. Non-blocking serial command processing
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

  // 2. Non-blocking evaluation of pump safety state machine & watchdog
  pump.update();

  // 3. Periodic non-blocking servo position updates (50 Hz / 20 ms)
  updateServos();
}
