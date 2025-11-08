// ESP32-S3 — Dual-core diagnostics (USB Serial + Pi UART + SIM7600)
// Core 0: Pi UART + motors + HEARTBEAT prints every 1s
// Core 1: SIM7600 (GPS->HTTP)

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

SemaphoreHandle_t gSerialMtx = nullptr;

// ===== UARTs (safe pins on S3) =====
#define PI_RX_PIN   44      // Pi TX -> ESP32 RX (Serial1)
#define PI_TX_PIN   43      // Pi RX <- ESP32 TX
#define PI_BAUD     115200

#define MODEM_TX    17      // ESP32 -> SIM7600 RX (Serial2)
#define MODEM_RX    18      // ESP32 <- SIM7600 TX
#define MODEM_BAUD  115200
#define APN "wholesale"
#define URL "https://roambot.dev/data"

HardwareSerial modem(2);

// ===== Motor pins (RIGHT PWM moved off USB pins) =====
#define LEFT_PWM_PIN      10
#define RIGHT_PWM_PIN     21 //do not assign to 19, it will break usb connection I think
#define LEFT_EN_PIN        9
#define RIGHT_EN_PIN       8

const bool LEFT_ACTIVE_LOW  = false;
const bool RIGHT_ACTIVE_LOW = false;

const float   GAMMA        = 2.2f;
const uint8_t MIN_NONZERO  = 2;
const int     MAX_DUTY     = 255;

static char   lineBuf[256];
static size_t lineLen = 0;

static int  currentLeftDuty  = 0;
static int  currentRightDuty = 0;

// ---------- safe prints (tolerate null mutex) ----------
void sp(const char* s){ if(gSerialMtx) xSemaphoreTake(gSerialMtx, portMAX_DELAY); Serial.print(s);  if(gSerialMtx) xSemaphoreGive(gSerialMtx); }
void spln(const char* s){ if(gSerialMtx) xSemaphoreTake(gSerialMtx, portMAX_DELAY); Serial.println(s); if(gSerialMtx) xSemaphoreGive(gSerialMtx); }
void spf(const char* fmt, ...){
  if(gSerialMtx) xSemaphoreTake(gSerialMtx, portMAX_DELAY);
  va_list ap; va_start(ap, fmt); Serial.vprintf(fmt, ap); va_end(ap);
  if(gSerialMtx) xSemaphoreGive(gSerialMtx);
}

// ---------- PWM helpers ----------
static inline uint8_t gammaCorrect(uint8_t linear){
  if(!linear) return 0;
  float x = linear / 255.0f;
  int out = (int)roundf(powf(x, GAMMA) * 255.0f);
  if(out > 0 && out < MIN_NONZERO) out = MIN_NONZERO;
  return (uint8_t)constrain(out, 0, 255);
}
static inline void pwmWritePin(int pin, int duty_linear_0_255, bool activeLow){
  duty_linear_0_255 = constrain(duty_linear_0_255, 0, 255);
  uint8_t duty = gammaCorrect((uint8_t)duty_linear_0_255);
  if(activeLow) duty = 255 - duty;
  analogWrite(pin, duty);
}
static inline void leftEnable (bool on){  digitalWrite(LEFT_EN_PIN,  on?HIGH:LOW); }
static inline void rightEnable(bool on){  digitalWrite(RIGHT_EN_PIN, on?HIGH:LOW); }

void leftSetDuty (int d){
  d = constrain(d, 0, MAX_DUTY);
  pwmWritePin(LEFT_PWM_PIN, d, LEFT_ACTIVE_LOW);
  leftEnable(d > 0);
  currentLeftDuty = d;
  spf("[LEFT] duty=%d (RIGHT=%d)\n", currentLeftDuty, currentRightDuty);
}
void rightSetDuty(int d){
  d = constrain(d, 0, MAX_DUTY);
  pwmWritePin(RIGHT_PWM_PIN, d, RIGHT_ACTIVE_LOW);
  rightEnable(d > 0);
  currentRightDuty = d;
  spf("[RIGHT] duty=%d (LEFT=%d)\n", currentRightDuty, currentLeftDuty);
}
void stopBoth(){
  leftSetDuty(0); rightSetDuty(0);
  leftEnable(false); rightEnable(false);
  spln("[MOTOR] Stop BOTH");
}
static inline int normalizeMagnitude(long v){
  long mag = labs(v);
  if (mag > 256) mag = 256;
  int scaled = (int)roundf((mag / 256.0f) * 255.0f);
  return constrain(scaled, 0, 255);
}

// ---------- JSON parsing ----------
void handleJsonLine(const char* s);
bool parseCategory(const char* s, char &outCat);
bool parseLValue(const char* s, long &outVal);

// ---------- SIM helpers ----------
String sendAT(const String &cmd, uint32_t timeout=10000, const char *waitFor=nullptr){
  modem.println(cmd);
  sp(">> "); spln(cmd.c_str());
  String resp; uint32_t start=millis();
  while(millis()-start < timeout){
    while(modem.available()) resp += (char)modem.read();
    if(waitFor && resp.indexOf(waitFor)!=-1) break;
    if(resp.indexOf("+HTTPACTION:")!=-1) break;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if(resp.length()) spln(resp.c_str());
  return resp;
}

// ===================== TASKS =====================

// Core 0: Pi UART + motors + heartbeat
void piMotorTask(void*){
  // First, prove this task runs:
  for(int i=0;i<3;i++){ spln("[CORE0] booting piMotorTask..."); vTaskDelay(pdMS_TO_TICKS(200)); }

  Serial1.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  while (Serial1.available()) Serial1.read();

  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_ESP32)
  analogWriteResolution(LEFT_PWM_PIN,  8);
  analogWriteResolution(RIGHT_PWM_PIN, 8);
  analogWriteFrequency(LEFT_PWM_PIN,   20000);
  analogWriteFrequency(RIGHT_PWM_PIN,  20000);
#endif

  stopBoth();
  spln("[CORE0] READY: JSON at 115200 on RX=44/TX=43");

  TickType_t lastBeat = xTaskGetTickCount();
  for(;;){
    // Heartbeat every 1s regardless of UART traffic
    if(xTaskGetTickCount() - lastBeat >= pdMS_TO_TICKS(1000)){
      spln("[CORE0] heartbeat");
      lastBeat = xTaskGetTickCount();
    }

    // Read until newline and echo raw
    while (Serial1.available()){
      char c = (char)Serial1.read();
      // show raw bytes occasionally
      spf("[CORE0] RX byte: 0x%02X '%c'\n", (uint8_t)c, (c>=32&&c<127)?c:'.');
      if (c=='\r') continue;
      if (c=='\n'){
        lineBuf[min(lineLen, sizeof(lineBuf)-1)] = '\0';
        if(lineLen>0){
          sp("[PI->ESP32] "); spln(lineBuf);
          handleJsonLine(lineBuf);
        }
        lineLen = 0;
      }else{
        if (lineLen < sizeof(lineBuf)-1) lineBuf[lineLen++] = c;
        else lineLen = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Core 1: SIM7600 (unchanged behavior)
float parseCoord(const String &coord, const String &dir){
  if(coord.length()<4) return 0.0;
  float val = coord.substring(0, coord.length()-7).toFloat();
  int   deg = (int)(val / 100.0f);           // 28 (lat) or 81 (lon)
  float min = val - (deg * 100.0f);          // 35.665756 or 13.977191
  float dd = deg + (min/60.0f);
  if (dir=="S" || dir=="W") dd = -dd;
  return dd;
}
bool getGPS(float &lat, float &lon){
  String resp = sendAT("AT+CGPSINFO", 5000);
  int idx = resp.indexOf("+CGPSINFO:"); if(idx==-1) return false;
  String data = resp.substring(idx+10); data.trim();
  if (data.startsWith(",")) return false;
  String t[8]; int last=0,c=0;
  for(int i=0;i<data.length() && c<8;i++){
    if(data[i]==','||data[i]=='\n'||data[i]=='\r'){ t[c++]=data.substring(last,i); last=i+1; }
  }
  if(c<4) return false;
  lat = parseCoord(t[0], t[1]); lon = parseCoord(t[2], t[3]);
  return (lat!=0 && lon!=0);
}
void modemTask(void*){
  modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  vTaskDelay(pdMS_TO_TICKS(1500));
  spln("[CORE1] modemTask start");
  sendAT("AT"); sendAT("AT+CPIN?"); sendAT("AT+CSQ");
  sendAT("AT+CGATT=1", 10000);
  sendAT("AT+CGDCONT=1,\"IP\",\"" APN "\"");
  sendAT("AT+CGPADDR=1");
  sendAT("AT+CGPS=0", 1500);
  sendAT("AT+CGPS=1", 5000);

  for(;;){
    float lat=0, lon=0;
    spln("[CORE1] waiting GPS...");
    for(int i=0;i<5 && !getGPS(lat,lon); i++){ sp("."); vTaskDelay(pdMS_TO_TICKS(1500)); }
    sp("\n");
    String body = "{\"lat\":"+String(lat,6)+",\"lon\":"+String(lon,6)+"}";
    spln(("[CORE1] POST "+body).c_str());
    sendAT("AT+HTTPTERM");
    sendAT("AT+HTTPINIT");
    sendAT("AT+HTTPPARA=\"CID\",1");
    sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    sendAT("AT+HTTPPARA=\"URL\",\"" URL "\"");
    sendAT("AT+HTTPSSL=1");
    int len = body.length();
    sendAT("AT+HTTPDATA="+String(len)+",10000", 5000, "DOWNLOAD");
    modem.print(body);
    vTaskDelay(pdMS_TO_TICKS(300));
    sendAT("AT+HTTPACTION=1", 15000, "+HTTPACTION");
    sendAT("AT+HTTPREAD", 5000);
    sendAT("AT+HTTPTERM");
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// ---------- JSON handlers ----------
void handleJsonLine(const char* s){
  while(*s==' '||*s=='\t') s++;
  if(*s!='{' || strchr(s,'}')==nullptr){ spln("[WARN] Not JSON"); return; }
  long L;
  if(!parseLValue(s,L)){ spln("[INFO] No \"L\" field"); return; }
  spf("[RX] L=%ld\n", L);
  int mag = normalizeMagnitude(L);
  if (L>0)      leftSetDuty(mag);
  else if (L<0) rightSetDuty(mag);
  else          stopBoth();
}
bool parseLValue(const char* s, long &outVal){
  const char* k=strstr(s,"\"L\""); if(!k) return false;
  const char* c=strchr(k,':');     if(!c) return false;
  const char* p=c+1; while(*p==' '||*p=='\t') p++;
  bool neg=false; if(*p=='-'){neg=true;p++;} else if(*p=='+'){p++;}
  if(*p<'0'||*p>'9') return false;
  long v=0; while(*p>='0'&&*p<='9'){ v=v*10+(*p-'0'); p++; }
  outVal = neg? -v : v; return true;
}
bool parseCategory(const char* s, char &outCat){ const char* k=strstr(s,"\"C\""); if(!k) return false; const char* c=strchr(k,':'); if(!c) return false; const char* p=c+1; while(*p==' '||*p=='\t') p++; if(*p!='\"') return false; p++; if(!*p) return false; outCat=*p; return true; }

// ===================== Arduino entry =====================
void setup(){
  Serial.begin(115200);
  // Help the host re-open the COM port after reset
  uint32_t t0=millis(); while((millis()-t0)<1500 && !Serial){ delay(50); }
  Serial.println("\n[BOOT] setup() starting");

  gSerialMtx = xSemaphoreCreateMutex();
  if(!gSerialMtx) Serial.println("[BOOT] WARN: mutex create failed; continuing without lock");

  xTaskCreatePinnedToCore(piMotorTask, "piMotorTask", 4096, NULL, 2, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(modemTask,   "modemTask",   8192, NULL, 2, NULL, 1); // Core 1
  Serial.println("[BOOT] tasks created");
}
void loop(){}
