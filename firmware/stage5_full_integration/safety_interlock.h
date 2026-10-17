#ifndef SAFETY_INTERLOCK_H
#define SAFETY_INTERLOCK_H

#include <Arduino.h>
#include "hardware_config.h"

// =============================================================================
// STAGE 5: SAFETY INTERLOCK & CALIBRATED AUTONOMOUS DISCHARGE ENGINE
// Semi-Autonomous Indoor Fire-Extinguishing Drone Project
// =============================================================================

enum PumpState {
  PUMP_STATE_DISARMED = 0,
  PUMP_STATE_ARMED_IDLE,
  PUMP_STATE_BURSTING,
  PUMP_STATE_COOLDOWN,
  PUMP_STATE_FAULT
};

enum PumpFaultReason {
  FAULT_NONE = 0,
  FAULT_WATCHDOG_TIMEOUT,
  FAULT_TARGET_TIMEOUT,
  FAULT_EMERGENCY_DISARM,
  FAULT_INVALID_COMMAND,
  FAULT_ALIGNMENT_LOST
};

// Target tracking data updated by laptop YOLO vision pipeline
struct TargetTelemetry {
  bool fire_detected;
  float confidence;
  float pan_error_deg;
  float tilt_error_deg;
  float distance_m;
  unsigned long frame_capture_ts; // Hardware frame timestamp from camera
  unsigned long timestamp_ms;      // Local arrival timestamp
  unsigned long first_detected_ms; // First continuous detection timestamp
};

// 8-Point Interlock Status Codes
enum InterlockStatus {
  INTERLOCK_OK = 0,
  FAIL_DISARMED,
  FAIL_IN_FAULT,
  FAIL_IN_COOLDOWN,
  FAIL_ALREADY_BURSTING,
  FAIL_NO_FIRE_DETECTED,
  FAIL_CONFIDENCE_LOW,
  FAIL_PERSISTENCE_NOT_MET,
  FAIL_TARGET_DATA_STALE,
  FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS,
  FAIL_ALIGNMENT_ERROR_EXCEEDED,
  FAIL_SERVO_NOT_SETTLED,
  FAIL_LINK_WATCHDOG_EXPIRED
};

class SafetyInterlockController {
public:
  SafetyInterlockController()
    : state(PUMP_STATE_DISARMED),
      fault_reason(FAULT_NONE),
      burst_start_ms(0),
      burst_duration_ms(0),
      cooldown_start_ms(0),
      last_heartbeat_ms(0),
      last_servo_move_ms(0),
      is_servo_slewing(false),
      last_seq_processed(0),
      simulated_burst_active(false),
      active_session_id("") {
    memset(&target, 0, sizeof(TargetTelemetry));
  }

  void init() {
    // Physical hardware clamping: Always lock gate LOW at boot
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
    pinMode(PIN_PUMP_GATE, OUTPUT);
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);

    state = PUMP_STATE_DISARMED;
    fault_reason = FAULT_NONE;
    burst_duration_ms = 0;
    simulated_burst_active = false;
    feedHeartbeat();
  }

  // Refreshes link watchdog only (Does NOT refresh target freshness)
  void feedHeartbeat() {
    last_heartbeat_ms = millis();
  }

  // Handshake and sequence-number reset for new client sessions
  void handleHandshake(const String &session_id) {
    active_session_id = session_id;
    last_seq_processed = 0;
    feedHeartbeat();
    disarm(); // Return to safe baseline disarmed state
    Serial.printf("[SESSION] Handshake established. Session ID: %s | Seq reset to 0\n", session_id.c_str());
  }

  // Update target tracking telemetry from laptop YOLO packet
  void updateTargetData(bool detected, float conf, float pan_err, float tilt_err, float dist_m, unsigned long frame_ts) {
    unsigned long now = millis();
    feedHeartbeat(); // A target packet counts as link activity

    if (detected) {
      if (!target.fire_detected) {
        target.first_detected_ms = now; // Mark start of continuous detection
      }
      target.fire_detected = true;
      target.confidence = conf;
      target.pan_error_deg = pan_err;
      target.tilt_error_deg = tilt_err;
      target.distance_m = dist_m;
      target.frame_capture_ts = frame_ts;
      target.timestamp_ms = now; // Specific target freshness timestamp
    } else {
      target.fire_detected = false;
      target.confidence = 0.0f;
      target.first_detected_ms = 0;
      target.timestamp_ms = now;
    }
  }

  void notifyServoMotion(bool slewing) {
    is_servo_slewing = slewing;
    last_servo_move_ms = millis();
  }

  // Explicit operator arming
  bool arm() {
    if (state == PUMP_STATE_FAULT) return false;
    if (state == PUMP_STATE_BURSTING) return false;
    state = PUMP_STATE_ARMED_IDLE;
    feedHeartbeat();
    return true;
  }

  // Explicit operator disarm
  void disarm() {
    clampHardwareLow();
    state = PUMP_STATE_DISARMED;
    fault_reason = FAULT_NONE;
    burst_duration_ms = 0;
    simulated_burst_active = false;
  }

  // Immediate emergency stop
  void emergencyStop() {
    clampHardwareLow();
    state = PUMP_STATE_FAULT;
    fault_reason = FAULT_EMERGENCY_DISARM;
    burst_duration_ms = 0;
    simulated_burst_active = false;
  }

  // 8-Point Safety Interlock Verification
  // The pump NEVER activates on command arrival alone.
  InterlockStatus evaluateDischargeConditions(uint32_t req_burst_ms) {
    unsigned long now = millis();

    // 1. Operator Arming Check
    if (state == PUMP_STATE_FAULT) return FAIL_IN_FAULT;
    if (state == PUMP_STATE_DISARMED) return FAIL_DISARMED;
    if (state == PUMP_STATE_COOLDOWN) return FAIL_IN_COOLDOWN;
    if (state == PUMP_STATE_BURSTING) return FAIL_ALREADY_BURSTING;

    // 2. Link Watchdog Check (Tier 2: 400 ms)
    if (now - last_heartbeat_ms > COMM_WATCHDOG_TIMEOUT_MS) {
      return FAIL_LINK_WATCHDOG_EXPIRED;
    }

    // 3. Target Data Freshness Check (Tier 1: 200 ms)
    // NOTE: Heartbeat alone does NOT satisfy this! Target must be fresh.
    if (!target.fire_detected || (now - target.timestamp_ms > TARGET_FRESHNESS_TIMEOUT_MS)) {
      return FAIL_TARGET_DATA_STALE;
    }

    // 4. Fire Detection Confidence Check (>= 70%)
    if (target.confidence < FIRE_CONFIDENCE_THRESHOLD) {
      return FAIL_CONFIDENCE_LOW;
    }

    // 5. Persistent Fire Detection Check (>= 300 ms)
    if (now - target.first_detected_ms < FIRE_PERSISTENCE_MIN_MS) {
      return FAIL_PERSISTENCE_NOT_MET;
    }

    // 6. Calibrated Distance Envelope Check
    // Calibrated band: 1.0m ± 0.25m, bounded by [0.5m, 2.0m]
    if (fabs(target.distance_m - CALIBRATED_TARGET_DIST_M) > CALIBRATED_DIST_TOLERANCE_M ||
        target.distance_m < TARGET_DISTANCE_MIN_M || target.distance_m > TARGET_DISTANCE_MAX_M) {
      return FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS;
    }

    // 7. Predicted Nozzle Alignment Check (within ±3.0°)
    if (fabs(target.pan_error_deg) > TARGET_ALIGN_TOLERANCE_DEG ||
        fabs(target.tilt_error_deg) > TARGET_ALIGN_TOLERANCE_DEG) {
      return FAIL_ALIGNMENT_ERROR_EXCEEDED;
    }

    // 8. Estimated Servo Settling Time Check (>= 150 ms without slew)
    if (is_servo_slewing || (now - last_servo_move_ms < SERVO_ESTIMATED_SETTLING_MS)) {
      return FAIL_SERVO_NOT_SETTLED;
    }

    return INTERLOCK_OK;
  }

  // Request discharge (Protected by 8-point interlock)
  InterlockStatus requestDischarge(uint32_t duration_ms) {
    if (duration_ms < PUMP_BURST_MIN_MS) duration_ms = PUMP_BURST_MIN_MS;
    if (duration_ms > PUMP_BURST_MAX_MS) duration_ms = PUMP_BURST_MAX_MS;

    InterlockStatus check = evaluateDischargeConditions(duration_ms);
    if (check != INTERLOCK_OK) {
      return check;
    }

    // All 8 safety conditions verified: Initiate time-limited burst
    burst_duration_ms = duration_ms;
    burst_start_ms = millis();
    state = PUMP_STATE_BURSTING;
    simulated_burst_active = true;

    // Safety constraint: In Stage 5 simulation mode, physical gate remains 0.0V LOW!
    if (!STAGE5_SIMULATE_PUMP_ONLY) {
      digitalWrite(PIN_PUMP_GATE, PUMP_ACTIVE_LEVEL);
    }

    return INTERLOCK_OK;
  }

  // Periodic non-blocking safety evaluation
  void update() {
    unsigned long now = millis();

    // 1. Link Watchdog Enforcement (Active when armed or bursting)
    if (state == PUMP_STATE_ARMED_IDLE || state == PUMP_STATE_BURSTING) {
      if (now - last_heartbeat_ms > COMM_WATCHDOG_TIMEOUT_MS) {
        clampHardwareLow();
        state = PUMP_STATE_FAULT;
        fault_reason = FAULT_WATCHDOG_TIMEOUT;
        simulated_burst_active = false;
        return;
      }
    }

    // 2. Active Burst Supervision (Abort if alignment or target lost mid-burst)
    if (state == PUMP_STATE_BURSTING) {
      if (now - target.timestamp_ms > TARGET_FRESHNESS_TIMEOUT_MS || !target.fire_detected) {
        clampHardwareLow();
        state = PUMP_STATE_FAULT;
        fault_reason = FAULT_TARGET_TIMEOUT;
        simulated_burst_active = false;
        return;
      }

      if (fabs(target.pan_error_deg) > (TARGET_ALIGN_TOLERANCE_DEG + 2.0f) ||
          fabs(target.tilt_error_deg) > (TARGET_ALIGN_TOLERANCE_DEG + 2.0f)) {
        clampHardwareLow();
        state = PUMP_STATE_FAULT;
        fault_reason = FAULT_ALIGNMENT_LOST;
        simulated_burst_active = false;
        return;
      }

      if (now - burst_start_ms >= burst_duration_ms) {
        clampHardwareLow();
        state = PUMP_STATE_COOLDOWN;
        cooldown_start_ms = now;
        simulated_burst_active = false;
      }
    } else if (state == PUMP_STATE_COOLDOWN) {
      if (now - cooldown_start_ms >= PUMP_COOLDOWN_MS) {
        state = PUMP_STATE_ARMED_IDLE;
        feedHeartbeat();
      }
    } else {
      clampHardwareLow();
    }
  }

  void clampHardwareLow() {
    digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
  }

  // Getters
  PumpState getState() const { return state; }
  PumpFaultReason getFaultReason() const { return fault_reason; }
  bool isArmed() const { return state == PUMP_STATE_ARMED_IDLE || state == PUMP_STATE_BURSTING || state == PUMP_STATE_COOLDOWN; }
  bool isBursting() const { return state == PUMP_STATE_BURSTING; }
  bool isSimulatedBurstActive() const { return simulated_burst_active; }

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

  const TargetTelemetry& getTarget() const { return target; }

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
      case FAULT_TARGET_TIMEOUT:   return "TARGET_TIMEOUT";
      case FAULT_EMERGENCY_DISARM: return "EMERGENCY_DISARM";
      case FAULT_INVALID_COMMAND:  return "INVALID_COMMAND";
      case FAULT_ALIGNMENT_LOST:   return "ALIGNMENT_LOST";
      default:                     return "UNKNOWN";
    }
  }

  static const char* getInterlockFailStr(InterlockStatus status) {
    switch (status) {
      case INTERLOCK_OK:                              return "ALL_CONDITIONS_MET";
      case FAIL_DISARMED:                             return "SYSTEM_DISARMED";
      case FAIL_IN_FAULT:                             return "SYSTEM_IN_FAULT";
      case FAIL_IN_COOLDOWN:                          return "COOLDOWN_LOCKOUT_ACTIVE";
      case FAIL_ALREADY_BURSTING:                     return "ALREADY_BURSTING";
      case FAIL_NO_FIRE_DETECTED:                     return "NO_FIRE_DETECTED";
      case FAIL_CONFIDENCE_LOW:                       return "CONFIDENCE_BELOW_70_PERCENT";
      case FAIL_PERSISTENCE_NOT_MET:                  return "PERSISTENCE_BELOW_300_MS";
      case FAIL_TARGET_DATA_STALE:                    return "TARGET_DATA_STALE_OVER_200_MS";
      case FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS: return "DISTANCE_OUT_OF_CALIBRATED_BAND_0.75_TO_1.25M";
      case FAIL_ALIGNMENT_ERROR_EXCEEDED:             return "ALIGNMENT_ERROR_EXCEEDS_3.0_DEG";
      case FAIL_SERVO_NOT_SETTLED:                    return "SERVO_NOT_SETTLED_WAIT_150_MS";
      case FAIL_LINK_WATCHDOG_EXPIRED:                return "COMM_WATCHDOG_EXPIRED";
      default:                                        return "UNKNOWN_INTERLOCK_ERROR";
    }
  }

  uint32_t getLastSeqProcessed() const { return last_seq_processed; }
  void setLastSeqProcessed(uint32_t seq) { last_seq_processed = seq; }
  const String& getActiveSessionId() const { return active_session_id; }

private:
  PumpState state;
  PumpFaultReason fault_reason;
  unsigned long burst_start_ms;
  uint32_t burst_duration_ms;
  unsigned long cooldown_start_ms;
  unsigned long last_heartbeat_ms;
  unsigned long last_servo_move_ms;
  bool is_servo_slewing;
  uint32_t last_seq_processed;
  bool simulated_burst_active;
  String active_session_id;
  TargetTelemetry target;
};

#endif // SAFETY_INTERLOCK_H
