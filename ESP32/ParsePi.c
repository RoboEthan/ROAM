/*
  Pi -> ESP32 JSON -> Signed Motor/LED Brightness (Gamma Matched)
  - UART1 @ 115200 (RX=17, TX=18), newline-terminated JSON
  - EXACT control field (required):
      {"L": <signed int>}
        L > 0  : update LEFT  only   (GPIO10)
        L < 0  : update RIGHT only   (GPIO21)
        L == 0 : stop BOTH
  - Magnitude handling:
      * Accepts -127..127 or -255..255 (auto-normalizes to 0..255)
      * Gamma-corrected so +N and -N have same perceived brightness
  - Optional category lines: {"C":"T"} / {"C":"G"} -> just logs
*/

#include <Arduino.h>
#include <math.h>

// ========= Pi <-> ESP32 UART =========
#define RX_PIN 17      // Pi TX -> ESP32 RX
#define TX_PIN 18      // Pi RX <- ESP32 TX
#define BAUD   115200

// ========= Motor/LED Pins =========
#define LEFT_PWM_PIN    10
#define LEFT_EN_PIN      9
#define RIGHT_PWM_PIN   21
#define RIGHT_EN_PIN     8

// If the LED/device is wired active-low (pin sinks current), set true to invert PWM
const bool LEFT_ACTIVE_LOW  = false;   // GPIO10 likely active-low with LED to 3V3
const bool RIGHT_ACTIVE_LOW = false;   // GPIO21 active-high (default)

// Perception tuning
const float   GAMMA        = 2.2f;    // 2.0–2.4 typical for LEDs
const uint8_t MIN_NONZERO  = 2;       // ensure tiny values still visible (optional)

const int MAX_DUTY = 255;

// ========= Line buffer for JSON from Pi =========
static char   lineBuf[256];
static size_t lineLen = 0;

// Track last applied values
static int currentLeftDuty  = 0;      // linear 0..255 before gamma/inversion
static int currentRightDuty = 0;

// ========= Decls =========
void handleJsonLine(const char* s);
bool parseCategory(const char* s, char &outCat);
bool parseLValue(const char* s, long &outVal);

// ---- Enables ----
static inline void leftEnable (bool on)  { digitalWrite(LEFT_EN_PIN,  on ? HIGH : LOW); }
static inline void rightEnable(bool on)  { digitalWrite(RIGHT_EN_PIN, on ? HIGH : LOW); }

// ---- Gamma + PWM writer (inversion AFTER gamma) ----
static inline uint8_t gammaCorrect(uint8_t linear) {
  if (linear == 0) return 0;
  float x = linear / 255.0f;
  int out = (int)roundf(powf(x, GAMMA) * 255.0f);
  if (out > 0 && out < MIN_NONZERO) out = MIN_NONZERO;
  if (out > 255) out = 255;
  return (uint8_t)out;
}

static inline void pwmWritePin(int pin, int duty_linear_0_255, bool activeLow) {
  duty_linear_0_255 = constrain(duty_linear_0_255, 0, 255);
  uint8_t duty = gammaCorrect((uint8_t)duty_linear_0_255);
  if (activeLow) duty = 255 - duty;
  analogWrite(pin, duty);
}

// ---- Helpers to set side duties (linear 0..255) ----
void leftSetDuty (int d) {
  d = constrain(d, 0, MAX_DUTY);
  pwmWritePin(LEFT_PWM_PIN, d, LEFT_ACTIVE_LOW);
  leftEnable(d > 0);
  currentLeftDuty = d;
  Serial.printf("[LEFT] duty=%d (%.0f%%)%s\n", d, (d / 255.0f) * 100.0f, LEFT_ACTIVE_LOW ? " [INV]" : "");
}

void rightSetDuty(int d) {
  d = constrain(d, 0, MAX_DUTY);
  pwmWritePin(RIGHT_PWM_PIN, d, RIGHT_ACTIVE_LOW);
  rightEnable(d > 0);
  currentRightDuty = d;
  Serial.printf("[RIGHT] duty=%d (%.0f%%)%s\n", d, (d / 255.0f) * 100.0f, RIGHT_ACTIVE_LOW ? " [INV]" : "");
}

void stopBoth() {
  leftSetDuty(0);
  rightSetDuty(0);
  leftEnable(false);
  rightEnable(false);
  Serial.println("[MOTOR] Stop BOTH");
}

// ========= Magnitude normalization =========
// Normalize |v| so that both -127..127 and -255..255 map to 0..255
static inline int normalizeMagnitude(long v) {
  long mag = labs(v);
  if (mag <= 127) {
    // scale 0..127 -> 0..255 with rounding
    mag = (mag * 255 + 63) / 127;
  }
  if (mag > 255) mag = 255;
  return (int)mag;
}

// ========= Setup =========
void setup() {
  Serial.begin(BAUD);
  delay(50);

  Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(LEFT_EN_PIN,   OUTPUT);
  pinMode(RIGHT_EN_PIN,  OUTPUT);
  pinMode(LEFT_PWM_PIN,  OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  // Optional: quieter PWM on ESP32 (many cores support this)
  #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    analogWriteFrequency(LEFT_PWM_PIN,  20000);
    analogWriteFrequency(RIGHT_PWM_PIN, 20000);
  #endif

  stopBoth();

  Serial.println("\n[ESP32] Ready. Use JSON: {\"L\": <signed int>}  (+ -> LEFT GPIO10, - -> RIGHT GPIO21, 0 -> stop)");
  Serial.println("[ESP32] Magnitude normalized + gamma-corrected for equal perceived brightness.");
}

// ========= Loop =========
void loop() {
  // Accumulate UART1 bytes until newline
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    if (c == '\r') continue; // ignore CR

    if (c == '\n') {
      lineBuf[min(lineLen, sizeof(lineBuf) - 1)] = '\0';
      if (lineLen > 0) handleJsonLine(lineBuf);
      lineLen = 0;
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        // Overflow: reset to avoid partial garbage
        lineLen = 0;
      }
    }
  }
}

/* ================= Parsing & Handling ================= */

void handleJsonLine(const char* s) {
  // Trim leading spaces
  while (*s == ' ' || *s == '\t') s++;

  // Basic JSON gate
  if (*s != '{' || strchr(s, '}') == nullptr) {
    Serial.print("[WARN] Not JSON: ");
    Serial.println(s);
    return;
  }

  // Optional category logging
  char cat;
  if (parseCategory(s, cat)) {
    if (cat == 'T')      Serial.println("[RX] CATEGORY: Treasure (T)");
    else if (cat == 'G') Serial.println("[RX] CATEGORY: Garbage (G)");
    else                 Serial.printf("[RX] CATEGORY: Unknown (%c)\n", cat);
    // Fall-through; same line can include L too
  }

  long L;
  if (!parseLValue(s, L)) {
    Serial.print("[INFO] No \"L\" field found (expected {\"L\": <signed int>}): ");
    Serial.println(s);
    return;
  }

  Serial.printf("[RX] L = %ld\n", L);

  int mag = normalizeMagnitude(L); // unified 0..255 magnitude

  if (L > 0) {
    // Positive -> LEFT only
    leftSetDuty(mag);
  } else if (L < 0) {
    // Negative -> RIGHT only
    rightSetDuty(mag);
  } else {
    // Zero -> stop both
    stopBoth();
  }
}

bool parseCategory(const char* s, char &outCat) {
  const char* key = strstr(s, "\"C\"");
  if (!key) return false;

  const char* colon = strchr(key, ':');
  if (!colon) return false;

  const char* p = colon + 1;
  while (*p == ' ' || *p == '\t') p++;

  if (*p != '\"') return false;
  p++; // inside first quote
  if (*p == '\0') return false;

  outCat = *p; // first character inside quotes
  return true;
}

// STRICT: only accept {"L": <signed int>}
bool parseLValue(const char* s, long &outVal) {
  const char* k = strstr(s, "\"L\"");
  if (!k) return false;

  const char* colon = strchr(k, ':');
  if (!colon) return false;

  const char* p = colon + 1;
  while (*p == ' ' || *p == '\t') p++;

  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  else if (*p == '+') { p++; }

  if (*p < '0' || *p > '9') return false;

  long val = 0;
  while (*p >= '0' && *p <= '9') {
    val = val * 10 + (*p - '0');
    p++;
  }
  outVal = neg ? -val : val;
  return true;
}