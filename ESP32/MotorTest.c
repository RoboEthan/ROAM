// -----------------------------------------------------------------------------
// Forward-only BTS7960 test (software PWM, Serial duty input 0–250)
// IN1 = GPIO12 → PWM signal
// IN2 = GPIO11 → held LOW
// INH pins tied HIGH, logic VCC = 5 V, all GNDs common.
//
// Type 0..250 in Serial Monitor (250 = 100 % duty, 0 = OFF)
// Serial Monitor: 115200 baud, Line ending = Newline
// -----------------------------------------------------------------------------

#include <Arduino.h>

// === Pin Definitions ===
#define IN1 12
#define IN2 13

// === PWM Settings ===
const uint32_t PWM_FREQ_HZ = 1000;                // ~1 kHz
const uint32_t PERIOD_US   = 1000000UL / PWM_FREQ_HZ;

int duty_0_250 = 0;                                // 0–250 (maps to 0–100%)
uint32_t period_start = 0;

// === Helper: check if input string is an unsigned integer ===
bool isUInt(const String &s) {
  if (s.length() == 0) return false;
  for (int i = 0; i < (int)s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void setup() {
  // --- Pin setup ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN2, LOW);                          // Other leg OFF

  // --- Serial setup ---
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBTS7960 Forward-Only PWM Test (no ledc)");
  Serial.println("Type 0..250 (250 = 100 %).  Set Line ending = Newline.\n");

  period_start = micros();
}

void loop() {
  // ---------- Read Serial input ----------
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;                      // Ignore CR
    if (c == '\n') {                              // NL = end of entry
      line.trim();
      if (isUInt(line)) {
        int v = line.toInt();
        if (v < 0) v = 0;
        if (v > 250) v = 250;
        duty_0_250 = v;
        Serial.printf("Duty set to %d/250 (%.1f %%)\n",
                      duty_0_250, duty_0_250 * 100.0 / 250.0);
      } else if (line.length()) {
        Serial.println("Enter 0–250 (e.g., 125 for ~50 %)");
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 32) line = "";          // Prevent overflow
    }
  }

  // ---------- Software PWM generation ----------
  uint32_t now = micros();
  uint32_t elapsed = now - period_start;

  if (elapsed >= PERIOD_US) {
    period_start = now;
    elapsed = 0;
  }

  uint32_t on_us = (uint64_t)PERIOD_US * duty_0_250 / 250;
  digitalWrite(IN1, (elapsed < on_us) ? HIGH : LOW);
}
