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
#define WIFI_HOSTNAME "me424-mosfet"
#endif

// Use UART1 on remapped pins
HardwareSerial MainSerial(1);

const int UART_RX_PIN = 25;   // Slave RX from Main TX2
const int UART_TX_PIN = 26;   // Slave TX to Main RX2
const long UART_BAUD = 115200;

// MAG/VAC: digital outputs into your MOSFET switching stages (magnet, vacuum).
// SAW_POWER (D23): enable / power path (e.g. MOSFET or relay) for the saw supply.
// SAW_BLDC_PWM (D22): logic/PWM into an external **BLDC driver module** (ESC / driver IC input) —
//                      not a bare high‑current MOSFET leg like 18/19; the driver handles the motor phases.
const int MAG_PIN = 18;
const int VAC_PIN = 19;
const int SAW_POWER_PIN = 23;
const int SAW_BLDC_PWM_PIN = 22;
const int SAW_BLDC_PWM_CHANNEL = 0;
const int SAW_BLDC_PWM_FREQ_HZ = 5000;
const int SAW_BLDC_PWM_BITS = 8;

// Tool ID: 3.3V — 10k — node (D32) — tool resistor (2.2k / 5k / 10k) — GND
const int TOOL_ID_ADC_PIN = 32;
const float TOOL_R_SERIES_OHMS = 10000.0f;
const float TOOL_VCC = 3.3f;

// Potentiometer stroke (mm); raw 4095 = free (no compression)
const int POT_ADC_PIN = 33;
const float POT_STROKE_MM = 26.9875f;
const int POT_RAW_FREE = 4095;

static const int TOOL_NOMINAL_OHMS[3] = {2200, 5000, 10000};

String uartBuffer = "";

bool magState = false;
bool vacState = false;
bool sawState = false;
uint8_t sawSpeedPercent = 0;

static void sawBldcPwmInit() {
  ledcSetup(SAW_BLDC_PWM_CHANNEL, SAW_BLDC_PWM_FREQ_HZ, SAW_BLDC_PWM_BITS);
  ledcAttachPin(SAW_BLDC_PWM_PIN, SAW_BLDC_PWM_CHANNEL);
  ledcWrite(SAW_BLDC_PWM_CHANNEL, 0);
}

static void sawBldcPwmApply() {
  if (!sawState) {
    ledcWrite(SAW_BLDC_PWM_CHANNEL, 0);
    return;
  }
  uint32_t duty = (uint32_t)sawSpeedPercent * 255U / 100U;
  if (duty > 255U) duty = 255U;
  ledcWrite(SAW_BLDC_PWM_CHANNEL, duty);
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

/** Compute R_tool (V divider to GND) from measured tap voltage. */
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

void setMag(bool on) {
  magState = on;
  digitalWrite(MAG_PIN, on ? HIGH : LOW);
}

void setVac(bool on) {
  vacState = on;
  digitalWrite(VAC_PIN, on ? HIGH : LOW);
}

void setSaw(bool on) {
  sawState = on;
  digitalWrite(SAW_POWER_PIN, on ? HIGH : LOW);
  sawBldcPwmApply();
}

void sendStatus() {
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

void sendAck(const String &msg) {
  MainSerial.println(msg);
  Serial.println(msg);
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

  if (cmd == "IP") {
    printIpToSerialAndMain();
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
    sawBldcPwmApply();
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

  if (cmd == "STATUS") {
    sendStatus();
    return;
  }

  sendAck("ERR UNKNOWN_CMD");
}

void setup() {
  pinMode(MAG_PIN, OUTPUT);
  pinMode(VAC_PIN, OUTPUT);
  pinMode(SAW_POWER_PIN, OUTPUT);

  digitalWrite(MAG_PIN, LOW);
  digitalWrite(VAC_PIN, LOW);
  digitalWrite(SAW_POWER_PIN, LOW);

  (void)ledcDetachPin(MAG_PIN);
  (void)ledcDetachPin(VAC_PIN);
  (void)ledcDetachPin(SAW_POWER_PIN);
  (void)ledcDetachPin(SAW_BLDC_PWM_PIN);
  sawBldcPwmInit();

  analogSetPinAttenuation(TOOL_ID_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(POT_ADC_PIN, ADC_11db);

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

  delay(500);
  Serial.println("MOSFET slave ready");
  sendAck("SLAVE READY");
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
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
}
