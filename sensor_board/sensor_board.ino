#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

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
HardwareSerial MainSerial(1);

const int UART_RX_PIN = 26;
const int UART_TX_PIN = 27;
const long UART_BAUD = 115200;

const int STAGE3_LIMIT_PIN = 32;
const int STAGE2_LIMIT_PIN = 33;
const int STAGE4_LIMIT_PIN = 25;

// Stage 5: Hall + HW-477 on GPIO 14 (D14) — homing only. UART sends S5H=0 at zero, S5H=1 away.
const int STAGE5_HALL_PIN = 14;
// false: pin LOW = at zero (typical HW-477); true if inverted.
const bool S5_HALL_AT_ZERO_ACTIVE_HIGH = false;

String uartBuffer = "";

bool lastStage2 = false;
bool lastStage3 = false;
bool lastStage4 = false;
bool lastStage5Hall = false;
bool stage5HallRaw = false;
unsigned long stage5HallRawChangeMs = 0;
const unsigned long S5_HALL_DEBOUNCE_MS = 25;

bool readStage2Hit() { return digitalRead(STAGE2_LIMIT_PIN) == LOW; }
bool readStage3Hit() { return digitalRead(STAGE3_LIMIT_PIN) == LOW; }
bool readStage4Hit() { return digitalRead(STAGE4_LIMIT_PIN) == LOW; }

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
  if ((millis() - stage5HallRawChangeMs) >= S5_HALL_DEBOUNCE_MS && lastStage5Hall != stage5HallRaw) {
    lastStage5Hall = stage5HallRaw;
  }
  return lastStage5Hall;
}

void sendLimitStatus() {
  bool s2 = readStage2Hit();
  bool s3 = readStage3Hit();
  bool s4 = readStage4Hit();
  bool s5h = readStage5HallAtZero();

  // S5H: 0 = at Hall zero position, 1 = away (opposite sense from S2–S4 limits).
  String msg = "LIM S2=" + String(s2 ? "1" : "0") +
               " S3=" + String(s3 ? "1" : "0") +
               " S4=" + String(s4 ? "1" : "0") +
               " S5H=" + String(s5h ? "0" : "1");

  MainSerial.println(msg);
  Serial.println(msg);

  lastStage2 = s2;
  lastStage3 = s3;
  lastStage4 = s4;
  lastStage5Hall = s5h;
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;
  if (cmd == "STATUS") {
    sendLimitStatus();
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

  pinMode(STAGE2_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE3_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE4_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE5_HALL_PIN, INPUT);

  delay(200);

  lastStage2 = readStage2Hit();
  lastStage3 = readStage3Hit();
  lastStage4 = readStage4Hit();
  lastStage5Hall = readStage5HallAtZeroRaw();
  stage5HallRaw = lastStage5Hall;
  stage5HallRawChangeMs = millis();

  Serial.println("Sensor board ready (S5 hall on GPIO14)");
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

  if (s2 != lastStage2 || s3 != lastStage3 || s4 != lastStage4 || s5h != lastStage5Hall) {
    sendLimitStatus();
  }
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
}
