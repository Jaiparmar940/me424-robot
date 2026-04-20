#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME "me424-sensor"
#endif

// UART to main board
// Sensor TX (D27) -> Main RX (D18)
// Sensor RX (D26) <- Main TX (D19)
HardwareSerial MainSerial(1);

const int UART_RX_PIN = 26;
const int UART_TX_PIN = 27;
const long UART_BAUD = 115200;

// --- Tool: MAG/VAC/SAW + tool ID + probe (same protocol as old MOSFET board) ---
// UART to main stays on GPIO26/27 — do not use those for tool.
// MAG=18, VAC=19. SAW: GPIO21 enables saw power MOSFET; GPIO22 is BLDC PWM to driver.
// Tool ID + pot: GPIO34/35 are ADC1 input-only (divider + wiper; never drive them).
const int MAG_PIN = 18;
const int VAC_PIN = 19;
/** Saw load / power-path MOSFET gate (digital). */
const int SAW_MOSFET_PIN = 21;
const int SAW_BLDC_PWM_PIN = 22;
const int SAW_BLDC_PWM_CHANNEL = 0;
const int SAW_BLDC_PWM_FREQ_HZ = 5000;
const int SAW_BLDC_PWM_BITS = 8;

// Tool ID: 3.3V — 10k — node (ADC pin) — tool resistor (2.2k / 5k / 10k) — GND
const int TOOL_ID_ADC_PIN = 34;
const float TOOL_R_SERIES_OHMS = 10000.0f;
const float TOOL_VCC = 3.3f;

// Pot / probe stroke (mm); raw 4095 = free (no compression)
const int POT_ADC_PIN = 35;
const float POT_STROKE_MM = 26.9875f;
const int POT_RAW_FREE = 4095;

static const int TOOL_NOMINAL_OHMS[3] = {2200, 5000, 10000};

/** Magnet auto-off after continuous ON (safety after manual / UI mag on). */
#ifndef MAG_AUTO_OFF_MS
#define MAG_AUTO_OFF_MS 30000UL
#endif

bool magState = false;
/** millis() when MAG last turned on; 0 if MAG off. */
static unsigned long magOnSinceMs = 0;
bool vacState = false;
bool sawState = false;
uint8_t sawSpeedPercent = 0;
bool sawBldcPwmAttached = false;

static void sawBldcPwmInit() {
  ledcSetup(SAW_BLDC_PWM_CHANNEL, SAW_BLDC_PWM_FREQ_HZ, SAW_BLDC_PWM_BITS);
  pinMode(SAW_BLDC_PWM_PIN, OUTPUT);
  digitalWrite(SAW_BLDC_PWM_PIN, LOW);
}

static void sawBldcEnsureAttached() {
  if (sawBldcPwmAttached) return;
  ledcAttachPin(SAW_BLDC_PWM_PIN, SAW_BLDC_PWM_CHANNEL);
  sawBldcPwmAttached = true;
}

static void sawBldcForceLow() {
  if (sawBldcPwmAttached) {
    ledcWrite(SAW_BLDC_PWM_CHANNEL, 0);
    ledcDetachPin(SAW_BLDC_PWM_PIN);
    sawBldcPwmAttached = false;
  }
  pinMode(SAW_BLDC_PWM_PIN, OUTPUT);
  digitalWrite(SAW_BLDC_PWM_PIN, LOW);
}

static void sawBldcApplyDuty(uint32_t duty) {
  sawBldcEnsureAttached();
  ledcWrite(SAW_BLDC_PWM_CHANNEL, duty);
}

static uint32_t sawSpeedDuty() {
  uint32_t duty = (uint32_t)sawSpeedPercent * 255U / 100U;
  if (duty > 255U) duty = 255U;
  return duty;
}

static void sawBldcPwmApply() {
  if (!sawState) {
    digitalWrite(SAW_MOSFET_PIN, LOW);
    sawBldcForceLow();
    return;
  }
  digitalWrite(SAW_MOSFET_PIN, HIGH);
  const uint32_t duty = sawSpeedDuty();
  if (duty == 0U) {
    sawBldcForceLow();
    return;
  }
  sawBldcApplyDuty(duty);
}

static void sawBldcPwmSetpointOnly() {
  if (!sawState) return;
  digitalWrite(SAW_MOSFET_PIN, HIGH);
  const uint32_t duty = sawSpeedDuty();
  if (duty == 0U) {
    sawBldcForceLow();
    return;
  }
  sawBldcApplyDuty(duty);
}

void setMag(bool on) {
  magState = on;
  digitalWrite(MAG_PIN, on ? HIGH : LOW);
  magOnSinceMs = on ? millis() : 0;
}

void setVac(bool on) {
  vacState = on;
  digitalWrite(VAC_PIN, on ? HIGH : LOW);
}

void setSaw(bool on) {
  sawState = on;
  sawBldcPwmApply();
}

int readAdcAvg(int pin, int samples = 16) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(acc / samples);
}

float adcVoltageFromRaw(int raw) {
  return (float)raw / 4095.0f * TOOL_VCC;
}

float computeToolRFromVoltage(float vTap) {
  if (vTap <= 0.01f || vTap >= TOOL_VCC - 0.01f) return 0.0f;
  return vTap * TOOL_R_SERIES_OHMS / (TOOL_VCC - vTap);
}

/** 1=2.2k, 2=5k, 3=10k, 0=unknown */
int classifyToolOhms(float r) {
  if (r < 500.0f) return 0;
  int best = 0;
  float bestErr = 1e9f;
  for (int i = 0; i < 3; i++) {
    float err = fabsf(r - (float)TOOL_NOMINAL_OHMS[i]);
    if (err < bestErr) {
      bestErr = err;
      best = i + 1;
    }
  }
  if (bestErr > 1500.0f) return 0;
  return best;
}

float readCompressionMm() {
  int raw = readAdcAvg(POT_ADC_PIN);
  float compression = (float)(POT_RAW_FREE - raw) / 4095.0f * POT_STROKE_MM;
  if (compression < 0.0f) compression = 0.0f;
  return compression;
}

void readToolId(float &rOut, int &idOut) {
  int raw = readAdcAvg(TOOL_ID_ADC_PIN);
  float v = adcVoltageFromRaw(raw);
  rOut = computeToolRFromVoltage(v);
  idOut = classifyToolOhms(rOut);
}

void sendAck(const String &msg) {
  MainSerial.println(msg);
  Serial.println(msg);
}

/** Full tool line for main UI / mstatus (same shape as MOSFET board). */
void sendMosfetStyleStatus() {
  float rTool = 0;
  int toolId = 0;
  readToolId(rTool, toolId);
  float compMm = readCompressionMm();

  String msg = "STATUS MAG=" + String(magState ? "ON" : "OFF") +
               " VAC=" + String(vacState ? "ON" : "OFF") +
               " SAW=" + String(sawState ? "ON" : "OFF") +
               " SAW_SPD=" + String((int)sawSpeedPercent) +
               " TOOL_R=" + String(rTool, 0) +
               " TOOL_ID=" + String(toolId) +
               " COMP_MM=" + String(compMm, 2);
  MainSerial.println(msg);
  Serial.println(msg);
}

// Limit switches (wired to GND)
// Stage 1 turntable: CW-only limit (sensor D13). S1=1 when active (blocks s1cw on main).
const int STAGE1_CW_LIMIT_PIN = 13;

const int STAGE3_LIMIT_PIN = 32;
const int STAGE2_LIMIT_PIN = 33;
const int STAGE4_LIMIT_PIN = 25;

// Stage 5 zero: linear Hall (e.g. 3144) + HW-477 comparator on GPIO14 (D14).
// Not a motion limit — used only for automatic homing on main board.
// On the UART line, S5H=0 means at zero, S5H=1 means away (see sendLimitStatus).
// Tune HW-477 pot; flip S5_HALL_AT_ZERO_ACTIVE_HIGH if the pin sense is wrong.
const int STAGE5_HALL_PIN = 14;
// false: pin LOW = at Hall zero (typical HW-477 wiring); set true if your module is the opposite.
const bool S5_HALL_AT_ZERO_ACTIVE_HIGH = false;

// S1–S4 mechanical switches: pin must read stable this long before we change UART (ms).
// Override from PlatformIO: -D LIMIT_MECH_DEBOUNCE_MS=80
#ifndef LIMIT_MECH_DEBOUNCE_MS
#define LIMIT_MECH_DEBOUNCE_MS 60
#endif

String uartBuffer = "";

bool lastStage1 = false;
bool lastStage2 = false;
bool lastStage3 = false;
bool lastStage4 = false;
bool lastStage5Hall = false;
bool stage5HallStable = false;
bool stage5HallRaw = false;
unsigned long stage5HallRawChangeMs = 0;
const unsigned long S5_HALL_DEBOUNCE_MS = 25;

struct MechLimitDebouncer {
  bool pendingRaw = false;
  unsigned long changeMs = 0;
  bool stable = false;
};

static MechLimitDebouncer debS1, debS2, debS3, debS4;

/** rawHit: true when switch active (pin LOW with INPUT_PULLUP). Returns debounced hit. */
static bool debounceMech(MechLimitDebouncer &d, bool rawHit) {
  if (rawHit != d.pendingRaw) {
    d.pendingRaw = rawHit;
    d.changeMs = millis();
  }
  if ((millis() - d.changeMs) >= (unsigned long)LIMIT_MECH_DEBOUNCE_MS && d.stable != d.pendingRaw) {
    d.stable = d.pendingRaw;
  }
  return d.stable;
}

bool readStage1CWHitRaw() {
  return digitalRead(STAGE1_CW_LIMIT_PIN) == LOW;
}

bool readStage1CWHit() {
  return debounceMech(debS1, readStage1CWHitRaw());
}

bool readStage2HitRaw() {
  return digitalRead(STAGE2_LIMIT_PIN) == LOW;
}

bool readStage3HitRaw() {
  return digitalRead(STAGE3_LIMIT_PIN) == LOW;
}

bool readStage4HitRaw() {
  return digitalRead(STAGE4_LIMIT_PIN) == LOW;
}

bool readStage2Hit() {
  return debounceMech(debS2, readStage2HitRaw());
}

bool readStage3Hit() {
  return debounceMech(debS3, readStage3HitRaw());
}

bool readStage4Hit() {
  return debounceMech(debS4, readStage4HitRaw());
}

bool readStage5HallAtZeroRaw() {
  int v = digitalRead(STAGE5_HALL_PIN);
  return S5_HALL_AT_ZERO_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

bool readStage5HallAtZero() {
  bool raw = readStage5HallAtZeroRaw();
  if (raw != stage5HallRaw) {
    stage5HallRaw = raw;
    stage5HallRawChangeMs = millis();
  }

  if ((millis() - stage5HallRawChangeMs) >= S5_HALL_DEBOUNCE_MS && stage5HallStable != stage5HallRaw) {
    stage5HallStable = stage5HallRaw;
  }

  return stage5HallStable;
}

void sendLimitStatus() {
  bool s1 = readStage1CWHit();
  bool s2 = readStage2Hit();
  bool s3 = readStage3Hit();
  bool s4 = readStage4Hit();
  bool s5h = readStage5HallAtZero();

  // S5H: 0 = at Hall zero position, 1 = away (opposite sense from S2–S4 limits).
  String msg = "LIM S1=" + String(s1 ? "1" : "0") +
               " S2=" + String(s2 ? "1" : "0") +
               " S3=" + String(s3 ? "1" : "0") +
               " S4=" + String(s4 ? "1" : "0") +
               " S5H=" + String(s5h ? "0" : "1");

  MainSerial.println(msg);
  Serial.println(msg);

  lastStage1 = s1;
  lastStage2 = s2;
  lastStage3 = s3;
  lastStage4 = s4;
  lastStage5Hall = s5h;
}

static void printIpToSerialAndMain() {
  String line;
  if (WiFi.status() == WL_CONNECTED) {
    line = String("IP ") + WiFi.localIP().toString() + " hostname " + WIFI_HOSTNAME;
  } else {
    line = "IP (WiFi not connected)";
  }
  Serial.println(line);
  MainSerial.println(line);
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  // Tool status (main sends MSTATUS when MOSFET commands are routed over this UART).
  if (cmd == "MSTATUS") {
    sendMosfetStyleStatus();
    return;
  }

  if (cmd == "STATUS") {
    sendLimitStatus();
    return;
  }

  if (cmd == "MAG ON") {
    setMag(true);
    sendAck("ACK MAG ON");
    return;
  }
  if (cmd == "MAG OFF") {
    setMag(false);
    sendAck("ACK MAG OFF");
    return;
  }
  if (cmd == "VAC ON") {
    setVac(true);
    sendAck("ACK VAC ON");
    return;
  }
  if (cmd == "VAC OFF") {
    setVac(false);
    sendAck("ACK VAC OFF");
    return;
  }
  if (cmd == "SAW ON") {
    setSaw(true);
    sendAck("ACK SAW ON");
    return;
  }
  if (cmd == "SAW OFF") {
    setSaw(false);
    sendAck("ACK SAW OFF");
    return;
  }
  if (cmd.startsWith("SAW SPEED ")) {
    int v = cmd.substring(10).toInt();
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    sawSpeedPercent = (uint8_t)v;
    sawBldcPwmSetpointOnly();
    sendAck("ACK SAW SPEED " + String((int)sawSpeedPercent));
    return;
  }
  if (cmd == "ALL OFF") {
    setMag(false);
    setVac(false);
    setSaw(false);
    sendAck("ACK ALL OFF");
    return;
  }

  if (cmd == "IP") {
    printIpToSerialAndMain();
    return;
  }

  MainSerial.println("ERR UNKNOWN_CMD");
  Serial.println("ERR UNKNOWN_CMD");
}

void setup() {
  Serial.begin(115200);
  MainSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (String(WIFI_SSID).length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
    if (WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.setHostname(WIFI_HOSTNAME);
      ArduinoOTA.begin();
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
    } else {
      Serial.println("WiFi connect failed (OTA disabled)");
    }
  } else {
    Serial.println("WiFi disabled (set WIFI_SSID/WIFI_PASSWORD build flags)");
  }

  pinMode(MAG_PIN, OUTPUT);
  pinMode(VAC_PIN, OUTPUT);
  pinMode(SAW_MOSFET_PIN, OUTPUT);
  digitalWrite(MAG_PIN, LOW);
  digitalWrite(VAC_PIN, LOW);
  digitalWrite(SAW_MOSFET_PIN, LOW);
  (void)ledcDetachPin(MAG_PIN);
  (void)ledcDetachPin(VAC_PIN);
  (void)ledcDetachPin(SAW_MOSFET_PIN);
  (void)ledcDetachPin(SAW_BLDC_PWM_PIN);
  sawBldcPwmInit();

  pinMode(STAGE1_CW_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE2_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE3_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE4_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE5_HALL_PIN, INPUT);

  analogSetPinAttenuation(TOOL_ID_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(POT_ADC_PIN, ADC_11db);

  delay(200);

  bool ir1 = readStage1CWHitRaw();
  lastStage1 = ir1;
  debS1.pendingRaw = ir1;
  debS1.stable = ir1;
  debS1.changeMs = millis();

  bool ir2 = readStage2HitRaw();
  lastStage2 = ir2;
  debS2.pendingRaw = ir2;
  debS2.stable = ir2;
  debS2.changeMs = millis();

  bool ir3 = readStage3HitRaw();
  lastStage3 = ir3;
  debS3.pendingRaw = ir3;
  debS3.stable = ir3;
  debS3.changeMs = millis();

  bool ir4 = readStage4HitRaw();
  lastStage4 = ir4;
  debS4.pendingRaw = ir4;
  debS4.stable = ir4;
  debS4.changeMs = millis();
  bool initS5h = readStage5HallAtZeroRaw();
  lastStage5Hall = initS5h;
  stage5HallStable = initS5h;
  stage5HallRaw = initS5h;
  stage5HallRawChangeMs = millis();

  Serial.println(String("Sensor board ready (S1–S4 mech debounce ") + LIMIT_MECH_DEBOUNCE_MS +
                 " ms, S5 hall GPIO14; MAG=18 VAC=19 SAW_MOS=21 SAW_PWM=22 TOOL_ID_ADC=34 POT_ADC=35; MAG auto-off " +
                 String((unsigned long)(MAG_AUTO_OFF_MS / 1000UL)) + "s)");
  MainSerial.println("SENSOR READY");
  sendLimitStatus();
}

void loop() {
  while (MainSerial.available()) {
    char c = (char)MainSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (uartBuffer.length() > 0) {
        handleCommand(uartBuffer);
        uartBuffer = "";
      }
    } else {
      uartBuffer += c;
    }
  }

  if (magState && magOnSinceMs != 0UL && (millis() - magOnSinceMs) >= MAG_AUTO_OFF_MS) {
    setMag(false);
    Serial.println("MAG: auto-off (timeout)");
    MainSerial.println("MAG AUTO OFF");
  }

  bool s1 = readStage1CWHit();
  bool s2 = readStage2Hit();
  bool s3 = readStage3Hit();
  bool s4 = readStage4Hit();
  bool prevS5h = lastStage5Hall;
  bool s5h = readStage5HallAtZero();
  if (s5h != prevS5h) {
    if (s5h) {
      Serial.println("S5 HALL: at zero (GPIO14)");
    } else {
      Serial.println("S5 HALL: left zero zone");
    }
  }

  if (s1 != lastStage1 || s2 != lastStage2 || s3 != lastStage3 || s4 != lastStage4 ||
      s5h != lastStage5Hall) {
    sendLimitStatus();
  }
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
}
