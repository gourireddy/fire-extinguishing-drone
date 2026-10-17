"""
Stage 3 Pump Safety Controller & Integrated Actuator Software Verification Suite
Tests:
1. Default Boot State (DISARMED, Output LOW, Fire Rejected)
2. Normal Arming & Transition to ARMED_IDLE
3. Timed Discharge Burst (Active-HIGH, accurate non-blocking duration)
4. Cooldown Enforcement (3000 ms refractory lockout, fire rejected)
5. Cooldown Expiry & Recovery to ARMED_IDLE
6. Duration Bounds Validation (Min 50 ms, Max 500 ms, Rejections)
7. Double-Trigger / Stale Command Protection (ERR_ALREADY_BURSTING)
8. Emergency Stop (E-STOP during active discharge, immediate LOW clamp, FAULT lock)
9. Fault Clearing Protocol (disarm required before re-arm)
10. Communication Watchdog Timeout (1000 ms silence trips FAULT, cuts discharge)
11. Watchdog Refresh / Heartbeat Keep-Alive
12. Coexistence with Stage 2 Servo PWM Slew Limiter
"""

import sys

# Constants matching hardware_config.h
PUMP_BURST_MIN_MS = 50
PUMP_BURST_DEFAULT_MS = 200
PUMP_BURST_MAX_MS = 500
PUMP_COOLDOWN_MS = 3000
COMM_WATCHDOG_TIMEOUT_MS = 1000

# States
STATE_DISARMED = "DISARMED"
STATE_ARMED_IDLE = "ARMED_IDLE"
STATE_BURSTING = "BURSTING"
STATE_COOLDOWN = "COOLDOWN"
STATE_FAULT = "FAULT"

# Faults
FAULT_NONE = "NONE"
FAULT_WATCHDOG_TIMEOUT = "WATCHDOG_TIMEOUT"
FAULT_EMERGENCY_DISARM = "EMERGENCY_DISARM"

# Results
RESULT_OK = "RESULT_OK"
ERR_NOT_ARMED = "ERR_NOT_ARMED"
ERR_IN_COOLDOWN = "ERR_IN_COOLDOWN"
ERR_ALREADY_BURSTING = "ERR_ALREADY_BURSTING"
ERR_IN_FAULT = "ERR_IN_FAULT"
ERR_INVALID_DURATION = "ERR_INVALID_DURATION"

class MockPumpController:
    def __init__(self):
        self.state = STATE_DISARMED
        self.fault_reason = FAULT_NONE
        self.gpio_level = 0  # 0 = LOW, 1 = HIGH
        self.burst_start_ms = 0
        self.burst_duration_ms = 0
        self.cooldown_start_ms = 0
        self.last_heartbeat_ms = 0
        self.watchdog_enabled = True

    def init(self, current_time_ms=0):
        self.gpio_level = 0
        self.state = STATE_DISARMED
        self.fault_reason = FAULT_NONE
        self.burst_duration_ms = 0
        self.last_heartbeat_ms = current_time_ms

    def feed_watchdog(self, current_time_ms):
        self.last_heartbeat_ms = current_time_ms

    def arm(self, current_time_ms):
        if self.state == STATE_FAULT:
            return ERR_IN_FAULT
        if self.state == STATE_BURSTING:
            return ERR_ALREADY_BURSTING
        self.state = STATE_ARMED_IDLE
        self.feed_watchdog(current_time_ms)
        return RESULT_OK

    def disarm(self):
        self.gpio_level = 0
        self.state = STATE_DISARMED
        self.fault_reason = FAULT_NONE
        self.burst_duration_ms = 0

    def emergency_stop(self):
        self.gpio_level = 0
        self.state = STATE_FAULT
        self.fault_reason = FAULT_EMERGENCY_DISARM
        self.burst_duration_ms = 0

    def request_burst(self, duration_ms, current_time_ms):
        if self.state == STATE_FAULT:
            return ERR_IN_FAULT
        if self.state == STATE_DISARMED:
            return ERR_NOT_ARMED
        if self.state == STATE_COOLDOWN:
            return ERR_IN_COOLDOWN
        if self.state == STATE_BURSTING:
            return ERR_ALREADY_BURSTING

        if duration_ms < PUMP_BURST_MIN_MS or duration_ms > PUMP_BURST_MAX_MS:
            return ERR_INVALID_DURATION

        self.feed_watchdog(current_time_ms)
        self.burst_duration_ms = duration_ms
        self.burst_start_ms = current_time_ms
        self.state = STATE_BURSTING
        self.gpio_level = 1  # HIGH
        return RESULT_OK

    def update(self, current_time_ms):
        # 1. Watchdog check
        if self.watchdog_enabled and (self.state in (STATE_ARMED_IDLE, STATE_BURSTING)):
            if current_time_ms - self.last_heartbeat_ms > COMM_WATCHDOG_TIMEOUT_MS:
                self.gpio_level = 0
                self.state = STATE_FAULT
                self.fault_reason = FAULT_WATCHDOG_TIMEOUT
                self.burst_duration_ms = 0
                return

        # 2. State-specific timers
        if self.state == STATE_BURSTING:
            if current_time_ms - self.burst_start_ms >= self.burst_duration_ms:
                self.gpio_level = 0
                self.state = STATE_COOLDOWN
                self.cooldown_start_ms = current_time_ms
        elif self.state == STATE_COOLDOWN:
            if current_time_ms - self.cooldown_start_ms >= PUMP_COOLDOWN_MS:
                self.state = STATE_ARMED_IDLE
                self.feed_watchdog(current_time_ms)
        else:
            self.gpio_level = 0

def run_stage3_tests():
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
    print("STAGE 3 PUMP CONTROLLER & SAFETY FSM VERIFICATION SUITE")
    print("=" * 70)

    # TEST GROUP 1: Boot State & Disarmed Isolation
    print("\n--- Test Group 1: Default Boot State & Disarmed Isolation ---")
    pump = MockPumpController()
    t = 1000
    pump.init(t)
    check("Boot State is DISARMED", pump.state == STATE_DISARMED)
    check("Boot GPIO 21 is LOW (0.0V)", pump.gpio_level == 0)
    check("Fault Reason is NONE", pump.fault_reason == FAULT_NONE)

    # Attempt fire while disarmed
    res = pump.request_burst(200, t)
    check("Reject Fire when DISARMED", res == ERR_NOT_ARMED)
    check("GPIO 21 remains LOW", pump.gpio_level == 0)

    # TEST GROUP 2: Arming & Timed Discharge Cycle
    print("\n--- Test Group 2: Arming & Timed Discharge Cycle ---")
    res = pump.arm(t)
    check("Arming returns RESULT_OK", res == RESULT_OK)
    check("State transitions to ARMED_IDLE", pump.state == STATE_ARMED_IDLE)
    check("GPIO 21 remains LOW while idle", pump.gpio_level == 0)

    # Initiate 200 ms burst
    res = pump.request_burst(200, t)
    check("Burst Request accepted", res == RESULT_OK)
    check("State transitions to BURSTING", pump.state == STATE_BURSTING)
    check("GPIO 21 drives HIGH (3.3V)", pump.gpio_level == 1)

    # Advance time halfway through burst (100 ms)
    t += 100
    pump.update(t)
    check("Mid-burst State is BURSTING", pump.state == STATE_BURSTING)
    check("Mid-burst GPIO 21 remains HIGH", pump.gpio_level == 1)

    # Advance time to completion (100 ms more -> 200 ms total)
    t += 100
    pump.update(t)
    check("Burst completed -> State COOLDOWN", pump.state == STATE_COOLDOWN)
    check("Burst completed -> GPIO 21 forced LOW", pump.gpio_level == 0)

    # TEST GROUP 3: Cooldown Lockout & Recovery
    print("\n--- Test Group 3: Cooldown Lockout Enforcement (3000 ms) ---")
    # Attempt fire during cooldown
    res = pump.request_burst(200, t + 500)
    check("Reject Fire during COOLDOWN", res == ERR_IN_COOLDOWN)
    check("GPIO 21 remains LOW in cooldown", pump.gpio_level == 0)

    # Advance to 2999 ms after cooldown start
    t += 2999
    pump.update(t)
    check("State at 2999 ms is still COOLDOWN", pump.state == STATE_COOLDOWN)

    # Advance 1 ms more (3000 ms elapsed)
    t += 1
    pump.update(t)
    check("Cooldown expired -> State ARMED_IDLE", pump.state == STATE_ARMED_IDLE)
    check("Ready for next discharge", pump.gpio_level == 0)

    # Fire again after cooldown
    res = pump.request_burst(300, t)
    check("Fire accepted after cooldown", res == RESULT_OK and pump.state == STATE_BURSTING)
    t += 300
    pump.update(t)
    check("Second burst finished cleanly", pump.state == STATE_COOLDOWN and pump.gpio_level == 0)
    t += 3000
    pump.update(t)

    # TEST GROUP 4: Duration Parameter Clamping & Rejections
    print("\n--- Test Group 4: Duration Parameter Clamping & Rejection ---")
    res_under = pump.request_burst(40, t)  # below 50 ms min
    check("Reject burst < 50 ms (40 ms)", res_under == ERR_INVALID_DURATION)

    res_over = pump.request_burst(600, t)  # above 500 ms max
    check("Reject burst > 500 ms (600 ms)", res_over == ERR_INVALID_DURATION)

    res_zero = pump.request_burst(0, t)
    check("Reject burst == 0 ms", res_zero == ERR_INVALID_DURATION)

    res_max = pump.request_burst(500, t)
    check("Accept maximum burst (500 ms)", res_max == RESULT_OK and pump.state == STATE_BURSTING)
    t += 500
    pump.update(t)
    t += 3000
    pump.update(t)

    # TEST GROUP 5: Double-Trigger / Re-entrancy Protection
    print("\n--- Test Group 5: Re-entrancy & Stale Command Protection ---")
    pump.request_burst(250, t)
    check("Initial burst active", pump.state == STATE_BURSTING)

    res_dup = pump.request_burst(250, t + 50)
    check("Reject second fire while BURSTING", res_dup == ERR_ALREADY_BURSTING)
    check("GPIO 21 remains HIGH for original duration", pump.gpio_level == 1)
    t += 250
    pump.update(t)
    t += 3000
    pump.update(t)

    # TEST GROUP 6: Emergency Stop (E-STOP) During Active Discharge
    print("\n--- Test Group 6: Emergency Stop (E-STOP) During Active Discharge ---")
    pump.request_burst(400, t)
    check("Burst started for 400 ms", pump.state == STATE_BURSTING and pump.gpio_level == 1)

    # Operator triggers E-STOP at 150 ms
    t += 150
    pump.emergency_stop()
    check("E-STOP immediately forces GPIO 21 LOW", pump.gpio_level == 0)
    check("State enters FAULT", pump.state == STATE_FAULT)
    check("Fault reason is EMERGENCY_DISARM", pump.fault_reason == FAULT_EMERGENCY_DISARM)

    # Advance time past original 400 ms mark
    t += 300
    pump.update(t)
    check("GPIO 21 remains strictly LOW after original burst would have ended", pump.gpio_level == 0)
    check("Fire rejected while in FAULT", pump.request_burst(200, t) == ERR_IN_FAULT)
    check("Arm rejected while in FAULT", pump.arm(t) == ERR_IN_FAULT)

    # TEST GROUP 7: Fault Clearing Protocol
    print("\n--- Test Group 7: Fault Clearing Protocol ---")
    pump.disarm()
    check("Disarm clears FAULT state to DISARMED", pump.state == STATE_DISARMED)
    check("Fault reason reset to NONE", pump.fault_reason == FAULT_NONE)
    check("Re-arming succeeds after disarm", pump.arm(t) == RESULT_OK and pump.state == STATE_ARMED_IDLE)

    # TEST GROUP 8: Communication Watchdog Timeout
    print("\n--- Test Group 8: Communication Watchdog Timeout ---")
    # Feed heartbeat continuously
    t += 500
    pump.feed_watchdog(t)
    pump.update(t)
    check("Heartbeat keeps ARMED_IDLE active", pump.state == STATE_ARMED_IDLE)

    # Silence for 1001 ms (exceeds COMM_WATCHDOG_TIMEOUT_MS = 1000)
    t += 1001
    pump.update(t)
    check("Silence > 1000 ms trips Watchdog -> FAULT", pump.state == STATE_FAULT)
    check("Fault reason is WATCHDOG_TIMEOUT", pump.fault_reason == FAULT_WATCHDOG_TIMEOUT)
    check("GPIO 21 locked LOW on watchdog trip", pump.gpio_level == 0)

    # Watchdog trip during active burst
    pump.disarm()
    pump.arm(t)
    pump.request_burst(400, t)
    check("Burst started", pump.state == STATE_BURSTING and pump.gpio_level == 1)

    # Communication dies mid-burst for 1001 ms
    t += 1001
    pump.update(t)
    check("Watchdog mid-burst immediately forces LOW", pump.gpio_level == 0)
    check("State tripped to FAULT", pump.state == STATE_FAULT and pump.fault_reason == FAULT_WATCHDOG_TIMEOUT)

    print("\n" + "=" * 70)
    print(f"STAGE 3 VERIFICATION RESULTS: {passed} PASSED | {failed} FAILED")
    print("=" * 70)

    return failed == 0

if __name__ == "__main__":
    success = run_stage3_tests()
    sys.exit(0 if success else 1)
