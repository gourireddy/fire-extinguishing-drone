#include <Arduino.h>
#include "hardware_config.h"

// =============================================================================
// STAGE 1: GPIO STARTUP BEHAVIOR & HARDWARE DIAGNOSTIC FIRMWARE
// Semi-Autonomous Indoor Fire-Extinguishing Drone Project
// Target: ESP32-S3-WROOM-1 (N16R8) | Arduino Core 3.3.12 (ESP-IDF 5.5.5)
// =============================================================================

// Commanded logic states (Tracking internal software registers)
bool commanded_gpio1  = false;
bool commanded_gpio2  = false;

// Timed diagnostic pulse state for GPIO 21 (Non-blocking)
bool pump_pulse_active = false;
unsigned long pump_pulse_start_ms = 0;

void printMenu() {
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("  STAGE 1: GPIO DIAGNOSTIC & VOLTAGE VALIDATION INTERFACE    ");
  Serial.println("--------------------------------------------------------------");
  Serial.println("Commands:");
  Serial.println("  [1] - Toggle Commanded State on GPIO 1 (Pan Servo Signal)");
  Serial.println("  [2] - Toggle Commanded State on GPIO 2 (Tilt Servo Signal)");
  Serial.println("  [p] - Trigger 100 ms Auto-Timed Diagnostic Pulse on GPIO 21");
  Serial.println("  [s] - Display Commanded Register States & Electrical Guide");
  Serial.println("  [h] - Display this command menu");
  Serial.println("--------------------------------------------------------------");
  Serial.println("SAFETY NOTICE: All tests must be conducted with actuators");
  Serial.println("and MOSFET driver PHYSICALLY DISCONNECTED from pin headers.");
  Serial.println("--------------------------------------------------------------\n");
}

void printStatus() {
  Serial.println("\n==============================================================");
  Serial.println("             COMMANDED GPIO STATES & ELECTRICAL TARGETS       ");
  Serial.println("==============================================================");
  Serial.printf("  GPIO 1  (Pan Servo Pin)   : Commanded %-4s | Target: %s\n",
                commanded_gpio1 ? "HIGH" : "LOW",
                commanded_gpio1 ? "Nominal 3.3V" : "0.00V");
  Serial.printf("  GPIO 2  (Tilt Servo Pin)  : Commanded %-4s | Target: %s\n",
                commanded_gpio2 ? "HIGH" : "LOW",
                commanded_gpio2 ? "Nominal 3.3V" : "0.00V");
  Serial.printf("  GPIO 21 (Pump Gate Pin)   : Commanded %-4s | Target: 0.00V (LOCKED)\n",
                pump_pulse_active ? "HIGH" : "LOW");
  Serial.println("--------------------------------------------------------------");
  Serial.println("CRITICAL MEASUREMENT PRINCIPLES:");
  Serial.println("  1. Commanded states shown above reflect internal software latches.");
  Serial.println("     Physical voltages MUST be measured with a DMM or oscilloscope.");
  Serial.println("  2. A DMM measures steady-state DC and CANNOT detect boot glitches.");
  Serial.println("     Sub-millisecond transient capture requires a digital storage scope.");
  Serial.println("  3. An external 10k pull-down to GND on GPIO 21 is mandatory to clamp");
  Serial.println("     the gate during unpowered and high-Z bootloader states.");
  Serial.println("==============================================================\n");
}

void setup() {
  // ===========================================================================
  // STEP 0: IMMEDIATE HARDWARE SAFETY CLAMPING (EXECUTED BEFORE SERIAL/DELAYS)
  // Preload output latches to LOW before enabling driver circuitry to prevent
  // intermediate floating-to-output edge glitches.
  // ===========================================================================
  digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
  pinMode(PIN_PUMP_GATE, OUTPUT);
  digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);

  digitalWrite(PIN_SERVO_PAN, LOW);
  pinMode(PIN_SERVO_PAN, OUTPUT);
  digitalWrite(PIN_SERVO_PAN, LOW);

  digitalWrite(PIN_SERVO_TILT, LOW);
  pinMode(PIN_SERVO_TILT, OUTPUT);
  digitalWrite(PIN_SERVO_TILT, LOW);

  // Initialize USB-UART Serial interface
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n==============================================================");
  Serial.println("   ESP32-S3 Stage 1: Actuator GPIO & Boot Validation Test     ");
  Serial.println("   Core: Arduino-ESP32 3.3.12 (ESP-IDF 5.5.5)                 ");
  Serial.println("==============================================================");
  Serial.println("[SAFE] Hardware outputs clamped LOW prior to Serial startup.");
  Serial.println("[SAFE] GPIO 21 preloaded and locked LOW (0.0V).");
  Serial.println("[SAFE] GPIO 1 & GPIO 2 preloaded and locked LOW (0.0V).");

  printStatus();
  printMenu();
}

void loop() {
  // Non-blocking timer enforcement for GPIO 21 diagnostic pulse
  if (pump_pulse_active) {
    if (millis() - pump_pulse_start_ms >= PUMP_DIAG_PULSE_MAX_MS) {
      digitalWrite(PIN_PUMP_GATE, PUMP_INACTIVE_LEVEL);
      pump_pulse_active = false;
      Serial.println("[SAFETY] GPIO 21 diagnostic pulse expired (100 ms limit reached).");
      Serial.println("[SAFETY] GPIO 21 unconditionally returned to LOW (0.0V).");
    }
  }

  // Non-blocking serial command processing
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') return;

    switch (c) {
      case '1':
        commanded_gpio1 = !commanded_gpio1;
        digitalWrite(PIN_SERVO_PAN, commanded_gpio1 ? HIGH : LOW);
        Serial.printf("[COMMAND] GPIO 1 (Pan) set to: %s -> Measure Pin 3 on Right Header\n",
                      commanded_gpio1 ? "HIGH (Target 3.3V)" : "LOW (Target 0.0V)");
        break;

      case '2':
        commanded_gpio2 = !commanded_gpio2;
        digitalWrite(PIN_SERVO_TILT, commanded_gpio2 ? HIGH : LOW);
        Serial.printf("[COMMAND] GPIO 2 (Tilt) set to: %s -> Measure Pin 4 on Right Header\n",
                      commanded_gpio2 ? "HIGH (Target 3.3V)" : "LOW (Target 0.0V)");
        break;

      case 'p':
      case 'P':
        if (!pump_pulse_active) {
          pump_pulse_active = true;
          pump_pulse_start_ms = millis();
          digitalWrite(PIN_PUMP_GATE, PUMP_ACTIVE_LEVEL);
          Serial.println("[COMMAND] Triggered auto-timed 100 ms pulse on GPIO 21 (Pin 17).");
          Serial.println("          Monitoring auto-shutdown timer...");
        } else {
          Serial.println("[BUSY] Diagnostic pulse already active.");
        }
        break;

      case 's':
      case 'S':
        printStatus();
        break;

      case 'h':
      case 'H':
      default:
        printMenu();
        break;
    }
  }
}
