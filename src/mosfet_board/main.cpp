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
#define WIFI_HOSTNAME "me424-mosfet"
#endif

// Use UART1 on remapped pins
HardwareSerial MainSerial(1);

const int UART_RX_PIN = 25;   // Slave RX from Main TX2
const int UART_TX_PIN = 26;   // Slave TX to Main RX2
const long UART_BAUD = 115200;

// MOSFET outputs
const int MAG_PIN = 18;
const int VAC_PIN = 19;
const int SAW_PIN = 21;

String uartBuffer = "";

bool magState = false;
bool vacState = false;
bool sawState = false;

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
  digitalWrite(SAW_PIN, on ? HIGH : LOW);
}

void sendStatus() {
  String msg = "STATUS MAG=" + String(magState ? "ON" : "OFF") +
               " VAC=" + String(vacState ? "ON" : "OFF") +
               " SAW=" + String(sawState ? "ON" : "OFF");
  MainSerial.println(msg);
  Serial.println(msg);
}

void sendAck(const String &msg) {
  MainSerial.println(msg);
  Serial.println(msg);
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

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
  pinMode(SAW_PIN, OUTPUT);

  digitalWrite(MAG_PIN, LOW);
  digitalWrite(VAC_PIN, LOW);
  digitalWrite(SAW_PIN, LOW);

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
