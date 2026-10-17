"""
Stage 2 Firmware Logic & Algorithmic Verification Suite
Tests:
1. PWM Duty Cycle calculations (14-bit, 50 Hz, 0.8192 ticks/us)
2. Active Window clamping & constrain mechanics
3. Command string validation (integers, malformed, overflows)
4. Window adjustment boundaries (min < max - 100, physical floor/ceil)
5. Slew rate limiter dynamics (discrete 20 ms ticks)
"""

import sys

SERVO_PWM_FREQ_HZ = 50
SERVO_PWM_RES_BITS = 14
TICKS_PER_PERIOD = 1 << SERVO_PWM_RES_BITS  # 16384
PERIOD_US = 1000000.0 / SERVO_PWM_FREQ_HZ   # 20000.0 us
TICKS_PER_US = TICKS_PER_PERIOD / PERIOD_US # 0.8192

SERVO_PHYSICAL_MIN_US = 500
SERVO_PHYSICAL_MAX_US = 2500
SERVO_PULSE_NEUTRAL_US = 1500

DEFAULT_ACTIVE_PAN_MIN_US = 1333
DEFAULT_ACTIVE_PAN_MAX_US = 1667
DEFAULT_ACTIVE_TILT_MIN_US = 1333
DEFAULT_ACTIVE_TILT_MAX_US = 1667
SERVO_DEFAULT_SLEW_US_PER_TICK = 10.0

def us_to_duty(us: float) -> int:
    return int(us * 0.8192 + 0.5)

def is_valid_positive_int(s: str):
    s = s.strip()
    if len(s) == 0 or len(s) > 5:
        return False, None
    if not s.isdigit():
        return False, None
    val = int(s)
    if val < 0 or val > 65535:
        return False, None
    return True, val

class ServoSimulator:
    def __init__(self):
        self.active_pan_min_us = DEFAULT_ACTIVE_PAN_MIN_US
        self.active_pan_max_us = DEFAULT_ACTIVE_PAN_MAX_US
        self.active_tilt_min_us = DEFAULT_ACTIVE_TILT_MIN_US
        self.active_tilt_max_us = DEFAULT_ACTIVE_TILT_MAX_US

        self.cur_pan_us = float(SERVO_PULSE_NEUTRAL_US)
        self.cur_tilt_us = float(SERVO_PULSE_NEUTRAL_US)
        self.tgt_pan_us = float(SERVO_PULSE_NEUTRAL_US)
        self.tgt_tilt_us = float(SERVO_PULSE_NEUTRAL_US)

        self.slew_us_per_tick = SERVO_DEFAULT_SLEW_US_PER_TICK
        self.logs = []

    def log(self, msg):
        self.logs.append(msg)

    def set_target_pan_us(self, us: float):
        clamped = max(float(self.active_pan_min_us), min(float(self.active_pan_max_us), us))
        self.tgt_pan_us = clamped

    def set_target_tilt_us(self, us: float):
        clamped = max(float(self.active_tilt_min_us), min(float(self.active_tilt_max_us), us))
        self.tgt_tilt_us = clamped

    def set_neutral(self):
        self.tgt_pan_us = float(SERVO_PULSE_NEUTRAL_US)
        self.tgt_tilt_us = float(SERVO_PULSE_NEUTRAL_US)

    def update_servos_tick(self):
        # 1 tick = 20 ms
        if self.cur_pan_us < self.tgt_pan_us:
            self.cur_pan_us = min(self.cur_pan_us + self.slew_us_per_tick, self.tgt_pan_us)
        elif self.cur_pan_us > self.tgt_pan_us:
            self.cur_pan_us = max(self.cur_pan_us - self.slew_us_per_tick, self.tgt_pan_us)

        if self.cur_tilt_us < self.tgt_tilt_us:
            self.cur_tilt_us = min(self.cur_tilt_us + self.slew_us_per_tick, self.tgt_tilt_us)
        elif self.cur_tilt_us > self.tgt_tilt_us:
            self.cur_tilt_us = max(self.cur_tilt_us - self.slew_us_per_tick, self.tgt_tilt_us)

    def parse_command(self, cmd: str) -> str:
        cmd = cmd.strip()
        if not cmd:
            return "EMPTY"
        
        lower = cmd.lower()
        if lower in ("help", "?"):
            return "HELP"
        elif lower in ("status", "cal"):
            return "STATUS"
        elif lower == "neutral":
            self.set_neutral()
            return "NEUTRAL_OK"
        elif lower == "p+5":
            self.set_target_pan_us(self.tgt_pan_us + 55.0)
            return "P+5_OK"
        elif lower == "p-5":
            self.set_target_pan_us(self.tgt_pan_us - 55.0)
            return "P-5_OK"
        elif lower == "t+5":
            self.set_target_tilt_us(self.tgt_tilt_us + 55.0)
            return "T+5_OK"
        elif lower == "t-5":
            self.set_target_tilt_us(self.tgt_tilt_us - 55.0)
            return "T-5_OK"
        elif lower.startswith("pus "):
            ok, val = is_valid_positive_int(cmd[4:])
            if not ok:
                return "ERR_MALFORMED"
            if val < self.active_pan_min_us or val > self.active_pan_max_us:
                return "ERR_OUT_OF_WINDOW"
            self.set_target_pan_us(float(val))
            return "PUS_OK"
        elif lower.startswith("tus "):
            ok, val = is_valid_positive_int(cmd[4:])
            if not ok:
                return "ERR_MALFORMED"
            if val < self.active_tilt_min_us or val > self.active_tilt_max_us:
                return "ERR_OUT_OF_WINDOW"
            self.set_target_tilt_us(float(val))
            return "TUS_OK"
        elif lower.startswith("min_p "):
            ok, val = is_valid_positive_int(cmd[6:])
            if not ok:
                return "ERR_MALFORMED"
            if val < SERVO_PHYSICAL_MIN_US:
                return "ERR_BELOW_PHYSICAL_MIN"
            if val >= self.active_pan_max_us - 100:
                return "ERR_REVERSED_OR_NARROW"
            self.active_pan_min_us = val
            return "MIN_P_OK"
        elif lower.startswith("max_p "):
            ok, val = is_valid_positive_int(cmd[6:])
            if not ok:
                return "ERR_MALFORMED"
            if val > SERVO_PHYSICAL_MAX_US:
                return "ERR_ABOVE_PHYSICAL_MAX"
            if val <= self.active_pan_min_us + 100:
                return "ERR_REVERSED_OR_NARROW"
            self.active_pan_max_us = val
            return "MAX_P_OK"
        elif lower.startswith("min_t "):
            ok, val = is_valid_positive_int(cmd[6:])
            if not ok:
                return "ERR_MALFORMED"
            if val < SERVO_PHYSICAL_MIN_US:
                return "ERR_BELOW_PHYSICAL_MIN"
            if val >= self.active_tilt_max_us - 100:
                return "ERR_REVERSED_OR_NARROW"
            self.active_tilt_min_us = val
            return "MIN_T_OK"
        elif lower.startswith("max_t "):
            ok, val = is_valid_positive_int(cmd[6:])
            if not ok:
                return "ERR_MALFORMED"
            if val > SERVO_PHYSICAL_MAX_US:
                return "ERR_ABOVE_PHYSICAL_MAX"
            if val <= self.active_tilt_min_us + 100:
                return "ERR_REVERSED_OR_NARROW"
            self.active_tilt_max_us = val
            return "MAX_T_OK"
        else:
            return "ERR_UNKNOWN"

def run_tests():
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
    print("STAGE 2 FIRMWARE LOGIC & PWM VERIFICATION SUITE")
    print("=" * 70)

    # TEST GROUP 1: PWM Duty Calculation Accuracy
    print("\n--- Test Group 1: LEDC 14-bit PWM Duty Calculations (50 Hz) ---")
    duty_500 = us_to_duty(500)
    check("Duty @ 500 us", duty_500 == 410, f"Expected 410 ticks, got {duty_500} ({duty_500/16384*100:.3f}%)")

    duty_1333 = us_to_duty(1333)
    check("Duty @ 1333 us (Min Active)", duty_1333 == 1092, f"Expected 1092 ticks, got {duty_1333} ({duty_1333/16384*100:.3f}%)")

    duty_1500 = us_to_duty(1500)
    check("Duty @ 1500 us (Neutral)", duty_1500 == 1229, f"Expected 1229 ticks, got {duty_1500} ({duty_1500/16384*100:.3f}%)")

    duty_1667 = us_to_duty(1667)
    check("Duty @ 1667 us (Max Active)", duty_1667 == 1366, f"Expected 1366 ticks, got {duty_1667} ({duty_1667/16384*100:.3f}%)")

    duty_2500 = us_to_duty(2500)
    check("Duty @ 2500 us", duty_2500 == 2048, f"Expected 2048 ticks, got {duty_2500} ({duty_2500/16384*100:.3f}%)")

    # Quantization resolution test
    tick_step_us = 1.0 / 0.8192
    check("Tick Resolution", abs(tick_step_us - 1.2207) < 0.001, f"Step = {tick_step_us:.4f} us/tick (~0.055 deg/tick)")

    # TEST GROUP 2: String & Integer Input Validation
    print("\n--- Test Group 2: String & Command Input Validation ---")
    check("Valid int '1500'", is_valid_positive_int("1500") == (True, 1500))
    check("Valid int with spaces '  1400  '", is_valid_positive_int("  1400  ") == (True, 1400))
    check("Reject negative '-500'", is_valid_positive_int("-500") == (False, None))
    check("Reject alphabetic 'abc'", is_valid_positive_int("abc") == (False, None))
    check("Reject alphanumeric '1500a'", is_valid_positive_int("1500a") == (False, None))
    check("Reject empty ''", is_valid_positive_int("") == (False, None))
    check("Reject whitespace only '   '", is_valid_positive_int("   ") == (False, None))
    check("Reject 6-digit overflow '100000'", is_valid_positive_int("100000") == (False, None))

    # TEST GROUP 3: Command Parser & Active Window Enforcement
    print("\n--- Test Group 3: Command Parser & Window Enforcement ---")
    sim = ServoSimulator()
    check("Initial Neutral Pan", sim.tgt_pan_us == 1500.0)
    check("Initial Neutral Tilt", sim.tgt_tilt_us == 1500.0)

    # Valid in-window pulses
    res = sim.parse_command("pus 1550")
    check("Command pus 1550", res == "PUS_OK" and sim.tgt_pan_us == 1550.0)

    res = sim.parse_command("tus 1450")
    check("Command tus 1450", res == "TUS_OK" and sim.tgt_tilt_us == 1450.0)

    # Out of active window rejection
    res = sim.parse_command("pus 1200")
    check("Reject pus 1200 (Below active min 1333)", res == "ERR_OUT_OF_WINDOW" and sim.tgt_pan_us == 1550.0)

    res = sim.parse_command("pus 1800")
    check("Reject pus 1800 (Above active max 1667)", res == "ERR_OUT_OF_WINDOW" and sim.tgt_pan_us == 1550.0)

    # Incremental nudges
    sim.set_neutral()
    res = sim.parse_command("p+5")
    check("Command p+5 (+55 us)", res == "P+5_OK" and sim.tgt_pan_us == 1555.0)

    res = sim.parse_command("p-5")
    check("Command p-5 (-55 us)", res == "P-5_OK" and sim.tgt_pan_us == 1500.0)

    # Clamping at window bounds under incremental steps
    for _ in range(10):
        sim.parse_command("p+5")
    check("Nudge clamped to active max", sim.tgt_pan_us == 1667.0, f"Target = {sim.tgt_pan_us} us")

    # TEST GROUP 4: Dynamic Window Reconfiguration & Safety Boundaries
    print("\n--- Test Group 4: Window Reconfiguration & Safety Boundaries ---")
    # Reject expansion below physical floor
    res = sim.parse_command("min_p 400")
    check("Reject min_p 400 (Below physical min 500)", res == "ERR_BELOW_PHYSICAL_MIN")

    # Reject expansion above physical ceiling
    res = sim.parse_command("max_p 2600")
    check("Reject max_p 2600 (Above physical max 2500)", res == "ERR_ABOVE_PHYSICAL_MAX")

    # Reject reversed bounds (min >= max - 100)
    res = sim.parse_command("min_p 1600") # max is 1667, difference is only 67 us
    check("Reject min_p 1600 (Reversed/narrow bounds)", res == "ERR_REVERSED_OR_NARROW")

    # Valid expansion
    res = sim.parse_command("min_p 1200")
    check("Accept min_p 1200", res == "MIN_P_OK" and sim.active_pan_min_us == 1200)

    res = sim.parse_command("max_p 1800")
    check("Accept max_p 1800", res == "MAX_P_OK" and sim.active_pan_max_us == 1800)

    # Now pus 1250 should succeed
    res = sim.parse_command("pus 1250")
    check("Accept pus 1250 in expanded window", res == "PUS_OK" and sim.tgt_pan_us == 1250.0)

    # TEST GROUP 5: Slew Rate Limiter Dynamics
    print("\n--- Test Group 5: Slew Rate Limiter (20 ms ticks) ---")
    sim.cur_pan_us = 1500.0
    sim.tgt_pan_us = 1600.0  # +100 us delta
    
    ticks = 0
    while sim.cur_pan_us < sim.tgt_pan_us:
        sim.update_servos_tick()
        ticks += 1

    check("Slew Rate Limiter Tick Count", ticks == 10, f"Took {ticks} ticks (200 ms) for 100 us change at 10 us/tick")
    check("Final Pulse Reached", sim.cur_pan_us == 1600.0)

    # Negative slew
    sim.tgt_pan_us = 1550.0  # -50 us delta
    ticks = 0
    while sim.cur_pan_us > sim.tgt_pan_us:
        sim.update_servos_tick()
        ticks += 1

    check("Slew Rate Limiter Negative Step", ticks == 5, f"Took {ticks} ticks (100 ms) for -50 us change")

    print("\n" + "=" * 70)
    print(f"VERIFICATION RESULTS: {passed} PASSED | {failed} FAILED")
    print("=" * 70)

    return failed == 0

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
