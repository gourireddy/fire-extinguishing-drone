#ifndef PUMP_FSM_H
#define PUMP_FSM_H

#include <Arduino.h>
#include "hardware_config.h"

// =============================================================================
// STAGE 3: WATER PUMP NON-BLOCKING SAFETY FINITE STATE MACHINE (FSM)
// Semi-Autonomous Indoor Fire-Extinguishing Drone Project
// =============================================================================

enum PumpState {
  PUMP_STATE_DISARMED = 0, // Default boot state. Output locked LOW. Firing rejected.
  PUMP_STATE_ARMED_IDLE,   // Armed and safe. Ready for timed burst or heartbeat.
  PUMP_STATE_BURSTING,     // Active-HIGH gate pulse. Hard-capped non-blocking timer.
  PUMP_STATE_COOLDOWN,     // Output forced LOW. 3000 ms lockout window active.
  PUMP_STATE_FAULT         // Safety trip (watchdog expiry, e-stop). Output locked LOW.
};

enum PumpFaultReason {
  FAULT_NONE = 0,
  FAULT_WATCHDOG_TIMEOUT,
  FAULT_EMERGENCY_DISARM,
  FAULT_INVALID_COMMAND
};

enum PumpCommandResult {
  RESULT_OK = 0,
  ERR_NOT_ARMED,
  ERR_IN_COOLDOWN,
  ERR_ALREADY_BURSTING,
  ERR_IN_FAULT,
  ERR_INVALID_DURATION
};

class PumpController {
public:
  PumpController()
    : state(PUMP_STATE_DISARMED),
      fault_reason(FAULT_NONE),
      burst_start_ms(0),
      burst_duration_ms(0),
      cooldown_start_ms(0),
      last_heartbeat_ms(0),
      watchdog_enabled(true) {}

  // STEP 0: Immediate hardware clamp prior to any delays or subsystem init
  void init() {
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
    pinMode(PIN_PUMP_GATE, OUTPUT);
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);

    state = PUMP_STATE_DISARMED;
    fault_reason = FAULT_NONE;
    burst_duration_ms = 0;
    feedWatchdog();
  }

  // Refresh communication heartbeat
  void feedWatchdog() {
    last_heartbeat_ms = millis();
  }

  void setWatchdogEnabled(bool enabled) {
    watchdog_enabled = enabled;
    if (enabled) {
      feedWatchdog();
    }
  }

  bool isWatchdogEnabled() const {
    return watchdog_enabled;
  }

  // Arm the pump system (Requires explicit operator action)
  PumpCommandResult arm() {
    if (state == PUMP_STATE_FAULT) {
      return ERR_IN_FAULT; // Must call disarm() to clear fault first
    }
    if (state == PUMP_STATE_BURSTING) {
      return ERR_ALREADY_BURSTING;
    }
    state = PUMP_STATE_ARMED_IDLE;
    feedWatchdog();
    return RESULT_OK;
  }

  // Disarm the pump system (Safely returns to default safe state)
  void disarm() {
    // Immediately clamp pin LOW
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
    state = PUMP_STATE_DISARMED;
    fault_reason = FAULT_NONE;
    burst_duration_ms = 0;
  }

  // Emergency Disarm / Stop (Highest priority action)
  void emergencyStop() {
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
    state = PUMP_STATE_FAULT;
    fault_reason = FAULT_EMERGENCY_DISARM;
    burst_duration_ms = 0;
  }

  // Request a single-shot timed discharge
  PumpCommandResult requestBurst(uint32_t duration_ms) {
    if (state == PUMP_STATE_FAULT) {
      return ERR_IN_FAULT;
    }
    if (state == PUMP_STATE_DISARMED) {
      return ERR_NOT_ARMED;
    }
    if (state == PUMP_STATE_COOLDOWN) {
      return ERR_IN_COOLDOWN;
    }
    if (state == PUMP_STATE_BURSTING) {
      return ERR_ALREADY_BURSTING;
    }

    if (duration_ms < PUMP_BURST_MIN_MS || duration_ms > PUMP_BURST_MAX_MS) {
      return ERR_INVALID_DURATION;
    }

    // Refresh watchdog and initiate discharge
    feedWatchdog();
    burst_duration_ms = duration_ms;
    burst_start_ms = millis();
    state = PUMP_STATE_BURSTING;
    digitalWrite(PIN_PUMP_GATE, PUMP_ACTIVE_LEVEL);

    return RESULT_OK;
  }

  // Periodic non-blocking state machine evaluation
  void update() {
    unsigned long now = millis();

    // 1. Safety Watchdog Verification (Active when Armed or Bursting)
    if (watchdog_enabled && (state == PUMP_STATE_ARMED_IDLE || state == PUMP_STATE_BURSTING)) {
      if (now - last_heartbeat_ms > COMM_WATCHDOG_TIMEOUT_MS) {
        // Communication lost! Immediate emergency clamp
        digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
        state = PUMP_STATE_FAULT;
        fault_reason = FAULT_WATCHDOG_TIMEOUT;
        burst_duration_ms = 0;
        return;
      }
    }

    // 2. State-specific non-blocking timers
    switch (state) {
      case PUMP_STATE_BURSTING:
        if (now - burst_start_ms >= burst_duration_ms) {
          // Burst period completed: Force gate LOW unconditionally
          digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
          state = PUMP_STATE_COOLDOWN;
          cooldown_start_ms = now;
        }
        break;

      case PUMP_STATE_COOLDOWN:
        if (now - cooldown_start_ms >= PUMP_COOLDOWN_MS) {
          // Cooldown completed: Transition back to Armed Idle
          state = PUMP_STATE_ARMED_IDLE;
          feedWatchdog();
        }
        break;

      case PUMP_STATE_DISARMED:
      case PUMP_STATE_ARMED_IDLE:
      case PUMP_STATE_FAULT:
      default:
        // Ensure pin is strictly held LOW in all quiescent states
        digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
        break;
    }
  }

  // State inspection getters
  PumpState getState() const { return state; }
  PumpFaultReason getFaultReason() const { return fault_reason; }
  bool isArmed() const { return (state == PUMP_STATE_ARMED_IDLE || state == PUMP_STATE_BURSTING || state == PUMP_STATE_COOLDOWN); }
  bool isBursting() const { return state == PUMP_STATE_BURSTING; }

  uint32_t getRemainingBurstMs() const {
    if (state != PUMP_STATE_BURSTING) return 0;
    unsigned long elapsed = millis() - burst_start_ms;
    return (elapsed >= burst_duration_ms) ? 0 : (burst_duration_ms - elapsed);
  }

  uint32_t getRemainingCooldownMs() const {
    if (state != PUMP_STATE_COOLDOWN) return 0;
    unsigned long elapsed = millis() - cooldown_start_ms;
    return (elapsed >= PUMP_COOLDOWN_MS) ? 0 : (PUMP_COOLDOWN_MS - elapsed);
  }

  const char* getStateStr() const {
    switch (state) {
      case PUMP_STATE_DISARMED:   return "DISARMED";
      case PUMP_STATE_ARMED_IDLE: return "ARMED_IDLE";
      case PUMP_STATE_BURSTING:   return "BURSTING";
      case PUMP_STATE_COOLDOWN:   return "COOLDOWN";
      case PUMP_STATE_FAULT:      return "FAULT";
      default:                    return "UNKNOWN";
    }
  }

  const char* getFaultReasonStr() const {
    switch (fault_reason) {
      case FAULT_NONE:             return "NONE";
      case FAULT_WATCHDOG_TIMEOUT: return "WATCHDOG_TIMEOUT";
      case FAULT_EMERGENCY_DISARM: return "EMERGENCY_DISARM";
      case FAULT_INVALID_COMMAND:  return "INVALID_COMMAND";
      default:                     return "UNKNOWN";
    }
  }

private:
  PumpState state;
  PumpFaultReason fault_reason;
  unsigned long burst_start_ms;
  uint32_t burst_duration_ms;
  unsigned long cooldown_start_ms;
  unsigned long last_heartbeat_ms;
  bool watchdog_enabled;
};

#endif // PUMP_FSM_H
