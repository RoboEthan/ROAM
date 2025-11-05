#include <Arduino.h>
#include <HardwareSerial.h>

// ===== Wiring (ESP32-S3) =====
// MODEM_TX (ESP32->SIM7600 RX) on GPIO17
// MODEM_RX (ESP32<-SIM7600 TX) on GPIO16
#define MODEM_TX 17
#define MODEM_RX 16

// ===== Network / Server =====
#define APN  "wholesale"                       // Mint Mobile APN (change if needed)
#define URL  "https://roambot.dev/data"        // Your endpoint

// ===== Serial Port to Modem =====
HardwareSerial modem(2); // UART2

// ===== GPS Fix Struct =====
struct GpsFix {
  bool valid = false;
  float latitude = 0.0f;
  float longitude = 0.0f;
  String date; // DDMMYY
  String time; // HHMMSS.s
};

// ===== Utility Functions =====
void flushModem(uint32_t ms = 50) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    while (modem.available()) modem.read();
    delay(5);
  }
}

String readAll(uint32_t timeout = 1000) {
  String resp;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (modem.available()) {
      resp += (char)modem.read();
    }
    delay(2);
  }
  return resp;
}

bool sendWait(const String &cmd, const String &expect, uint32_t timeout = 8000, bool echoLog = true) {
  if (echoLog) Serial.println(">> " + cmd);
  modem.println(cmd);

  String buf;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (modem.available()) {
      char c = modem.read();
      buf += c;
      if (buf.indexOf(expect) != -1) {
        if (echoLog) Serial.print(buf);
        return true;
      }
      if (buf.indexOf("ERROR") != -1) {
        if (echoLog) Serial.print(buf);
        return false;
      }
    }
    delay(4);
  }
  if (echoLog) Serial.print(buf);
  return (expect.length() == 0);
}

bool sendOK(const String &cmd, uint32_t timeout = 8000) {
  return sendWait(cmd, "OK", timeout);
}

bool waitForIP(uint32_t timeout_ms = 30000) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    modem.println("AT+IPADDR");
    String r = readAll(800);
    Serial.print(r);
    if (r.indexOf(".") != -1 && r.indexOf("+IP ERROR") == -1) return true;
    delay(500);
  }
  return false;
}

// ===== GPS Conversion =====
// Convert NMEA ddmm.mmmm (lat) / dddmm.mmmm (lon) to decimal degrees
bool nmeaToDecimal(const String &degMin, char hemi, float &out) {
  if (degMin.length() < 4) return false;
  int dot = degMin.indexOf('.');
  bool isLon = (hemi == 'E' || hemi == 'W');
  int degDigits = isLon ? 3 : 2;
  if (dot < 0 || dot < degDigits) return false;

  int d = degMin.substring(0, degDigits).toInt();
  float m = degMin.substring(degDigits).toFloat();
  float dec = d + (m / 60.0f);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  out = dec;
  return true;
}

// ===== GPS Parsing =====
GpsFix parseCGPSINFO(const String &resp) {
  GpsFix fix;
  int idx = resp.lastIndexOf("+CGPSINFO:");
  if (idx == -1) return fix;

  String line = resp.substring(idx);
  int nl = line.indexOf('\n');
  if (nl != -1) line = line.substring(0, nl);

  int colon = line.indexOf(':');
  if (colon == -1) return fix;
  String csv = line.substring(colon + 1);
  csv.trim();

  String fields[10];
  int f = 0, start = 0;
  for (int i = 0; i <= (int)csv.length() && f < 10; ++i) {
    if (i == (int)csv.length() || csv[i] == ',') {
      fields[f++] = csv.substring(start, i);
      start = i + 1;
    }
  }

  if (f < 6) return fix;

  String latStr = fields[0];
  String NS     = fields[1];
  String lonStr = fields[2];
  String EW     = fields[3];
  String date   = fields[4];
  String time   = fields[5];

  if (latStr.isEmpty() || lonStr.isEmpty() || NS.isEmpty() || EW.isEmpty()) return fix;

  float lat, lon;
  if (!nmeaToDecimal(latStr, NS[0], lat)) return fix;
  if (!nmeaToDecimal(lonStr, EW[0], lon)) return fix;

  fix.valid = true;
  fix.latitude = lat;
  fix.longitude = lon;
  fix.date = date;
  fix.time = time;
  return fix;
}

// ===== HTTP Functions =====
bool httpInitAndConfig() {
  sendOK("AT+HTTPTERM", 1500); // ignore error
  if (!sendOK("AT+HTTPINIT", 5000)) return false;
  if (!sendOK("AT+HTTPPARA=\"CID\",1", 2000)) return false;
  if (!sendOK("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2000)) return false;
  if (!sendOK(String("AT+HTTPPARA=\"URL\",\"") + URL + "\"", 3000)) return false;
  return true;
}

bool httpPostJson(const String &body) {
  Serial.println("\n--- HTTP POST BODY ---");
  Serial.println(body);

  if (!sendWait("AT+HTTPDATA=" + String(body.length()) + ",15000", "DOWNLOAD", 8000)) {
    Serial.println("[HTTP] Didn't get DOWNLOAD prompt.");
    return false;
  }
  modem.print(body);
  delay(50);

  if (!sendWait("AT+HTTPACTION=1", "+HTTPACTION:", 20000)) {
    Serial.println("[HTTP] No +HTTPACTION.");
    return false;
  }

  sendWait("AT+HTTPREAD", "OK", 8000);
  return true;
}

// ===== Global State =====
String lastStamp;
bool httpReady = true;

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(300);
  modem.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1200);

  Serial.println("\n=== SIM7600 GPS → HTTPS POST (Latitude / Longitude) ===");

  sendOK("AT", 2000);
  sendOK("ATE0", 1500);
  sendOK("AT+CPIN?", 4000);
  sendOK("AT+CFUN=1", 4000);
  sendOK("AT+CSQ", 2000);
  sendOK("AT+COPS?", 4000);

  sendOK("AT+CGATT=1", 15000);
  sendOK(String("AT+CGDCONT=1,\"IP\",\"") + APN + "\"", 4000);

  if (!sendOK("AT+NETOPEN", 15000)) {
    Serial.println("[NET] NETOPEN failed; trying to proceed anyway.");
  }
  if (!waitForIP(20000)) {
    Serial.println("[NET] No IP address; HTTP may fail.");
  }

  Serial.println("[GPS] Powering GNSS...");
  sendOK("AT+CGPS=0", 1500);
  if (!sendOK("AT+CGPS=1", 5000)) {
    Serial.println("[GPS] Failed to start GNSS.");
  }

  if (!httpInitAndConfig()) {
    Serial.println("[HTTP] HTTPINIT failed. Will retry later.");
    httpReady = false;
  }

  Serial.println("[RUN] Starting 1 Hz GPS loop.");
}

// ===== Main Loop =====
void loop() {
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < 1000) return; // every 1 second
  lastPoll = millis();

  flushModem(10);
  modem.println("AT+CGPSINFO");
  String r = readAll(400);
  if (r.length() == 0) r = readAll(600);
  Serial.print(r);

  GpsFix fix = parseCGPSINFO(r);
  if (!fix.valid) {
    Serial.println("[GPS] No valid fix yet.");
    return;
  }

  // Display decimal coordinates
  Serial.printf("[GPS] Latitude: %.6f, Longitude: %.6f\n", fix.latitude, fix.longitude);

  String stamp = fix.date + "," + fix.time;
  if (stamp == lastStamp) {
    Serial.println("[GPS] Same timestamp as last post (no new fix).");
    return;
  }

  String body = String("{\"latitude\":") + String(fix.latitude, 6) +
                ",\"longitude\":" + String(fix.longitude, 6) +
                ",\"utc_date\":\"" + fix.date + "\"" +
                ",\"utc_time\":\"" + fix.time + "\"}";

  if (!httpReady) {
    httpReady = httpInitAndConfig();
    if (!httpReady) {
      Serial.println("[HTTP] Re-init failed; skipping this tick.");
      return;
    }
  }

  if (!httpPostJson(body)) {
    Serial.println("[HTTP] POST failed. Will retry after re-init.");
    sendOK("AT+HTTPTERM", 1500);
    httpReady = false;
    return;
  }

  lastStamp = stamp;
  Serial.printf("[POST] Sent lat=%.6f lon=%.6f @ %s\n",
                fix.latitude, fix.longitude, stamp.c_str());
}