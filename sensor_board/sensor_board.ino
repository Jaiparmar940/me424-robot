#include <Arduino.h>

// UART to main board
// Sensor TX (D27) -> Main RX (D18)
// Sensor RX (D26) <- Main TX (D19)
HardwareSerial MainSerial(1);

const int UART_RX_PIN = 26;
const int UART_TX_PIN = 27;
const long UART_BAUD = 115200;

// Limit switches (wired to GND)
const int STAGE3_LIMIT_PIN = 32;
const int STAGE2_LIMIT_PIN = 33;
const int STAGE4_LIMIT_PIN = 25;

String uartBuffer = "";

// Last reported stable states
bool lastStage2 = false;
bool lastStage3 = false;
bool lastStage4 = false;

bool readStage2Hit() {
  return digitalRead(STAGE2_LIMIT_PIN) == LOW;
}

bool readStage3Hit() {
  return digitalRead(STAGE3_LIMIT_PIN) == LOW;
}

bool readStage4Hit() {
  return digitalRead(STAGE4_LIMIT_PIN) == LOW;
}

void sendLimitStatus() {
  bool s2 = readStage2Hit();
  bool s3 = readStage3Hit();
  bool s4 = readStage4Hit();

  String msg = "LIM S2=" + String(s2 ? "1" : "0") +
               " S3=" + String(s3 ? "1" : "0") +
               " S4=" + String(s4 ? "1" : "0");

  MainSerial.println(msg);
  Serial.println(msg);

  lastStage2 = s2;
  lastStage3 = s3;
  lastStage4 = s4;
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

  pinMode(STAGE2_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE3_LIMIT_PIN, INPUT_PULLUP);
  pinMode(STAGE4_LIMIT_PIN, INPUT_PULLUP);

  delay(200);

  lastStage2 = readStage2Hit();
  lastStage3 = readStage3Hit();
  lastStage4 = readStage4Hit();

  Serial.println("Sensor board ready");
  MainSerial.println("SENSOR READY");
  sendLimitStatus();
}

void loop() {
  // Handle incoming UART commands from main board
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

  // Only send when a limit state changes
  bool s2 = readStage2Hit();
  bool s3 = readStage3Hit();
  bool s4 = readStage4Hit();

  if (s2 != lastStage2 || s3 != lastStage3 || s4 != lastStage4) {
    sendLimitStatus();
  }
}