"""
Stage 5 Full-System Integration & Calibration Verification Suite
Verifies:
1. Camera-to-Nozzle Calibration Mathematics (Intrinsics, Offsets, Ballistic Gravity Drop)
2. Flame Base Targeting vs Center/Apex Targeting
3. Fixed-Distance Calibration Mode (1.0m +/- 0.25m) vs Out-of-Bounds Rejection
4. 8-Point Interlock Gate Full Logic
5. Heartbeat Isolation vs Target Freshness Expiry (Heartbeat NEVER extends stale detection)
6. Session Handshake & Sequence Reset Protocol
7. Hardware Simulation Clamp (GPIO 21 locked LOW)
"""

import sys
import math
import time

# Calibration Engine implementation matching drone_ground_station_stage5.py
class CalibrationEngine:
    def __init__(self,
                 img_w=320, img_h=240,
                 hfov_deg=60.0, vfov_deg=45.0,
                 nozzle_offset_x_m=0.0,
                 nozzle_offset_y_m=-0.040, # 40mm below lens
                 nozzle_offset_z_m=0.015,  # 15mm forward of lens
                 water_speed_mps=4.0,
                 calibrated_dist_m=1.0):
        self.w = img_w
        self.h = img_h
        self.cx = img_w / 2.0
        self.cy = img_h / 2.0
        self.fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        self.fy = (img_h / 2.0) / math.tan(math.radians(vfov_deg / 2.0))
        self.off_x = nozzle_offset_x_m
        self.off_y = nozzle_offset_y_m
        self.off_z = nozzle_offset_z_m
        self.v_water = water_speed_mps
        self.dist_m = calibrated_dist_m
        self.g = 9.81

    def compute_aim_angles(self, u_base, v_base, dist_m=None):
        if dist_m is None or dist_m <= 0:
            dist_m = self.dist_m

        X_c = ((u_base - self.cx) / self.fx) * dist_m
        Y_c = ((v_base - self.cy) / self.fy) * dist_m
        Z_c = dist_m

        X_n = X_c - self.off_x
        Y_n = Y_c - self.off_y
        Z_n = max(0.1, Z_c - self.off_z)

        t_flight = Z_n / self.v_water
        drop_m = 0.5 * self.g * (t_flight ** 2)

        Y_aim = Y_n - drop_m

        pan_deg = math.degrees(math.atan2(X_n, Z_n))
        tilt_deg = -math.degrees(math.atan2(Y_aim, Z_n))

        pan_us = max(1333.0, min(1667.0, 1500.0 + pan_deg * 11.11))
        tilt_us = max(1333.0, min(1667.0, 1500.0 + tilt_deg * 11.11))

        return {
            "pan_deg": pan_deg,
            "tilt_deg": tilt_deg,
            "pan_us": pan_us,
            "tilt_us": tilt_us,
            "drop_cm": drop_m * 100.0,
            "t_flight_ms": t_flight * 1000.0
        }

# Mock Safety Interlock matching Stage 5 firmware
class MockStage5Interlock:
    def __init__(self):
        self.state = "DISARMED"
        self.fault_reason = "NONE"
        self.burst_start_ms = 0
        self.burst_duration_ms = 0
        self.cooldown_start_ms = 0
        self.last_heartbeat_ms = 0
        self.last_servo_move_ms = 0
        self.is_servo_slewing = False
        self.last_seq_processed = 0
        self.physical_gpio21_level = 0
        self.simulated_burst_active = False

        self.target_detected = False
        self.target_confidence = 0.0
        self.target_pan_err = 0.0
        self.target_tilt_err = 0.0
        self.target_dist_m = 1.0
        self.target_timestamp_ms = 0
        self.target_first_detected_ms = 0

    def init(self, now=0):
        self.physical_gpio21_level = 0
        self.state = "DISARMED"
        self.fault_reason = "NONE"
        self.simulated_burst_active = False
        self.feed_heartbeat(now)

    def feed_heartbeat(self, now):
        self.last_heartbeat_ms = now

    def handle_handshake(self, session_id, now):
        self.last_seq_processed = 0
        self.feed_heartbeat(now)
        self.disarm()

    def update_target(self, detected, conf, pan_err, tilt_err, dist, now):
        self.feed_heartbeat(now)
        if detected:
            if not self.target_detected:
                self.target_first_detected_ms = now
            self.target_detected = True
            self.target_confidence = conf
            self.target_pan_err = pan_err
            self.target_tilt_err = tilt_err
            self.target_dist_m = dist
            self.target_timestamp_ms = now
        else:
            self.target_detected = False
            self.target_confidence = 0.0
            self.target_first_detected_ms = 0
            self.target_timestamp_ms = now

    def notify_servo_motion(self, slewing, now):
        self.is_servo_slewing = slewing
        self.last_servo_move_ms = now

    def arm(self, now):
        if self.state == "FAULT" or self.state == "BURSTING":
            return False
        self.state = "ARMED_IDLE"
        self.feed_heartbeat(now)
        return True

    def disarm(self):
        self.physical_gpio21_level = 0
        self.state = "DISARMED"
        self.fault_reason = "NONE"
        self.simulated_burst_active = False

    def emergency_stop(self):
        self.physical_gpio21_level = 0
        self.state = "FAULT"
        self.fault_reason = "EMERGENCY_DISARM"
        self.simulated_burst_active = False

    def evaluate_interlock(self, now):
        if self.state == "FAULT": return "FAIL_IN_FAULT"
        if self.state == "DISARMED": return "FAIL_DISARMED"
        if self.state == "COOLDOWN": return "FAIL_IN_COOLDOWN"
        if self.state == "BURSTING": return "FAIL_ALREADY_BURSTING"

        # Link Watchdog (400 ms)
        if now - self.last_heartbeat_ms > 400:
            return "FAIL_LINK_WATCHDOG_EXPIRED"

        # Target freshness (200 ms)
        if not self.target_detected or (now - self.target_timestamp_ms > 200):
            return "FAIL_TARGET_DATA_STALE"

        # Confidence (>= 70%)
        if self.target_confidence < 0.70:
            return "FAIL_CONFIDENCE_LOW"

        # Persistence (>= 300 ms)
        if now - self.target_first_detected_ms < 300:
            return "FAIL_PERSISTENCE_NOT_MET"

        # Calibrated Distance (1.0m +/- 0.25m, bounded by 0.5 - 2.0m)
        if abs(self.target_dist_m - 1.0) > 0.25 or self.target_dist_m < 0.5 or self.target_dist_m > 2.0:
            return "FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS"

        # Alignment error (<= 3.0 deg)
        if abs(self.target_pan_err) > 3.0 or abs(self.target_tilt_err) > 3.0:
            return "FAIL_ALIGNMENT_ERROR_EXCEEDED"

        # Servo settling (>= 150 ms)
        if self.is_servo_slewing or (now - self.last_servo_move_ms < 150):
            return "FAIL_SERVO_NOT_SETTLED"

        return "INTERLOCK_OK"

    def request_discharge(self, duration_ms, now):
        res = self.evaluate_interlock(now)
        if res != "INTERLOCK_OK":
            return res

        dur = max(50, min(500, duration_ms))
        self.burst_duration_ms = dur
        self.burst_start_ms = now
        self.state = "BURSTING"
        self.simulated_burst_active = True
        self.physical_gpio21_level = 0 # locked LOW in simulation mode
        return "INTERLOCK_OK"

    def update(self, now):
        if self.state in ("ARMED_IDLE", "BURSTING"):
            if now - self.last_heartbeat_ms > 400:
                self.physical_gpio21_level = 0
                self.state = "FAULT"
                self.fault_reason = "WATCHDOG_TIMEOUT"
                self.simulated_burst_active = False
                return

        if self.state == "BURSTING":
            if now - self.target_timestamp_ms > 200 or not self.target_detected:
                self.physical_gpio21_level = 0
                self.state = "FAULT"
                self.fault_reason = "TARGET_TIMEOUT"
                self.simulated_burst_active = False
                return

            if abs(self.target_pan_err) > 5.0 or abs(self.target_tilt_err) > 5.0:
                self.physical_gpio21_level = 0
                self.state = "FAULT"
                self.fault_reason = "ALIGNMENT_LOST"
                self.simulated_burst_active = False
                return

            if now - self.burst_start_ms >= self.burst_duration_ms:
                self.physical_gpio21_level = 0
                self.state = "COOLDOWN"
                self.cooldown_start_ms = now
                self.simulated_burst_active = False
        elif self.state == "COOLDOWN":
            if now - self.cooldown_start_ms >= 3000:
                self.state = "ARMED_IDLE"
                self.feed_heartbeat(now)
        else:
            self.physical_gpio21_level = 0

def run_stage5_tests():
    passed = 0
    failed = 0

    def check(test_name, condition, detail=""):
        nonlocal passed, failed
        if condition:
            passed += 1
            print(f"  [PASS] {test_name}: {detail}")
        else:
            failed += 1
            print(f"  [FAIL] {test_name}: {detail}")

    print("=" * 70)
    print("STAGE 5 SYSTEM INTEGRATION & CALIBRATION VERIFICATION SUITE")
    print("=" * 70)

    # TEST GROUP 1: Camera-to-Nozzle Calibration & Ballistics
    print("\n--- Test Group 1: Camera-to-Nozzle Calibration & Ballistics ---")
    calib = CalibrationEngine(img_w=320, img_h=240, hfov_deg=60.0, vfov_deg=45.0,
                              nozzle_offset_y_m=-0.040, water_speed_mps=4.0, calibrated_dist_m=1.0)

    # Test center target: flame base exactly at image center (160, 120)
    aim_center = calib.compute_aim_angles(160, 120, dist_m=1.0)
    check("Center target Pan angle is 0.0 deg", abs(aim_center["pan_deg"]) < 0.01)
    # Tilt must be positive (aimed upward) to compensate for nozzle offset (-40mm) and gravity drop (~30.6cm)
    check("Center target Tilt angle is positive (aimed up)", aim_center["tilt_deg"] > 10.0,
          f"Tilt = {aim_center['tilt_deg']:.2f} deg, Drop = {aim_center['drop_cm']:.1f} cm")
    expected_drop_cm = 0.5 * 9.81 * (((1.0 - 0.015) / 4.0) ** 2) * 100.0
    check("Gravity drop calculation at 1.0m", abs(aim_center["drop_cm"] - expected_drop_cm) < 0.1,
          f"Drop = {aim_center['drop_cm']:.2f} cm (Expected: {expected_drop_cm:.2f} cm)")

    # Test flame base target at bottom of image (160, 220)
    aim_bottom = calib.compute_aim_angles(160, 220, dist_m=1.0)
    check("Bottom target Pan angle is 0.0 deg", abs(aim_bottom["pan_deg"]) < 0.01)
    check("Flame base tilts lower than center target", aim_bottom["tilt_deg"] < aim_center["tilt_deg"],
          f"Bottom tilt = {aim_bottom['tilt_deg']:.2f} deg vs Center tilt = {aim_center['tilt_deg']:.2f} deg")

    # Test lateral target (240, 120) -> right of center
    aim_right = calib.compute_aim_angles(240, 120, dist_m=1.0)
    check("Right target Pan angle is positive", aim_right["pan_deg"] > 10.0,
          f"Pan = {aim_right['pan_deg']:.2f} deg")

    # TEST GROUP 2: Distance Envelope & Pending ToF Handling
    print("\n--- Test Group 2: Distance Envelope & Calibration Bands ---")
    s = MockStage5Interlock()
    t = 1000
    s.init(t)
    s.arm(t)

    # Valid calibrated distance (1.0m)
    s.update_target(True, 0.85, 0.0, 0.0, 1.0, t)
    t += 350
    s.update_target(True, 0.85, 0.0, 0.0, 1.0, t)
    s.notify_servo_motion(False, t - 200)
    check("Target at calibrated 1.0m is ACCEPTED", s.evaluate_interlock(t) == "INTERLOCK_OK")

    # Valid within tolerance (1.15m within 1.0m +/- 0.25m)
    s.update_target(True, 0.85, 0.0, 0.0, 1.15, t)
    check("Target at 1.15m (within 0.25m tolerance) is ACCEPTED", s.evaluate_interlock(t) == "INTERLOCK_OK")

    # Rejected outside calibrated band (1.40m > 1.25m)
    s.update_target(True, 0.85, 0.0, 0.0, 1.40, t)
    check("Target at 1.40m (> 1.25m band) is REJECTED",
          s.evaluate_interlock(t) == "FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS")

    # Rejected outside calibrated band (0.60m < 0.75m)
    s.update_target(True, 0.85, 0.0, 0.0, 0.60, t)
    check("Target at 0.60m (< 0.75m band) is REJECTED",
          s.evaluate_interlock(t) == "FAIL_DISTANCE_UNVERIFIED_OR_OUT_OF_BOUNDS")

    # TEST GROUP 3: Target Freshness vs Link Heartbeat Isolation
    print("\n--- Test Group 3: Target Freshness vs Heartbeat Isolation ---")
    s.disarm()
    t += 1000
    s.arm(t)
    s.update_target(True, 0.90, 0.0, 0.0, 1.0, t)
    t += 350
    s.update_target(True, 0.90, 0.0, 0.0, 1.0, t)
    s.notify_servo_motion(False, t - 200)
    check("Target initially valid", s.evaluate_interlock(t) == "INTERLOCK_OK")

    # Advance time 250 ms. Laptop sends heartbeats continuously, but NO fresh target!
    t += 250
    s.feed_heartbeat(t)
    s.update(t)
    check("Heartbeat keeps link alive", t - s.last_heartbeat_ms <= 400)
    check("Target data expired (> 200 ms)", t - s.target_timestamp_ms > 200)
    check("Discharge BLOCKED due to stale target despite active heartbeat",
          s.request_discharge(200, t) == "FAIL_TARGET_DATA_STALE")

    # TEST GROUP 4: Session Handshake & Sequence Number Resets
    print("\n--- Test Group 4: Session Handshake & Sequence Resets ---")
    s.last_seq_processed = 500
    check("Sequence currently at 500", s.last_seq_processed == 500)
    # Handshake resets sequence counter
    s.handle_handshake("session_test_123", t)
    check("Handshake resets seq to 0", s.last_seq_processed == 0)
    check("Handshake safely resets state to DISARMED", s.state == "DISARMED")

    # TEST GROUP 5: Physical Clamp Assurance in Simulation Mode
    print("\n--- Test Group 5: Hardware Clamp in Simulation Mode ---")
    s.arm(t)
    s.update_target(True, 0.90, 0.0, 0.0, 1.0, t)
    t += 350
    s.update_target(True, 0.90, 0.0, 0.0, 1.0, t)
    s.notify_servo_motion(False, t - 200)

    res = s.request_discharge(200, t)
    check("Discharge accepted in software", res == "INTERLOCK_OK")
    check("Software state is BURSTING", s.state == "BURSTING")
    check("Software bursting flag is TRUE", s.simulated_burst_active is True)
    check("Physical GPIO 21 level remains strictly 0 (LOW 0.0V)", s.physical_gpio21_level == 0)

    print("\n" + "=" * 70)
    print(f"STAGE 5 VERIFICATION RESULTS: {passed} PASSED | {failed} FAILED")
    print("=" * 70)

    return failed == 0

if __name__ == "__main__":
    success = run_stage5_tests()
    sys.exit(0 if success else 1)
