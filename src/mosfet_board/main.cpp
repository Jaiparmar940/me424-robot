#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// =========================
// UART to MOSFET board
// Change these if your board's RX2/TX2 are not GPIO16/17
// =========================
HardwareSerial MosfetSerial(2);
const int MOSFET_RX_PIN = 16;   // main RX2
const int MOSFET_TX_PIN = 17;   // main TX2
const long MOSFET_BAUD = 115200;

// =========================
// UART to sensor board
// Sensor TX(27) -> Main RX(18)
// Sensor RX(26) <- Main TX(19)
// =========================
HardwareSerial SensorSerial(1);
const int SENSOR_RX_PIN = 18;   // main RX from sensor TX
const int SENSOR_TX_PIN = 19;   // main TX to sensor RX
const long SENSOR_BAUD = 115200;

// =========================
// Stepper config
// =========================
const int NUM_DRIVERS = 5;

const int STEP_PINS[NUM_DRIVERS] = {25, 33, 32, 27, 26};
const int DIR_PINS[NUM_DRIVERS]  = {13, 14, 22, 21, 23};

int pulseDelayUs = 1000;

// Updated stage notation:
// Stage 1 = turntable (not assigned here yet)
// Stage 2 = base lift pair
// Stage 3 = next stage
// Stage 4 = next stage

const int STAGE2_RIGHT = 0;  // controller 1
const int STAGE3       = 1;  // controller 2
const int STAGE2_LEFT  = 3;  // controller 4

bool invertMotor[NUM_DRIVERS] = {
  false, // controller 1 / stage 2 right
  false, // controller 2 / stage 3
  false, // controller 3 / unassigned
  true,  // controller 4 / stage 2 left
  false  // controller 5 / unassigned
};

// =========================
// Limit states from sensor board
// true = switch hit
// =========================
volatile bool stage2LimitHit = false;
volatile bool stage3LimitHit = false;
volatile bool stage4LimitHit = false;

// =========================
// Buffers
// =========================
String serialBuffer = "";
String btBuffer = "";
String mosfetBuffer = "";
String sensorBuffer = "";

// =========================
// Helpers
// =========================
void printBoth(const String &msg) {
  Serial.print(msg);
  SerialBT.print(msg);
}

void printlnBoth(const String &msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}

void sendToMosfet(const String &msg) {
  MosfetSerial.println(msg);
  printlnBoth("[TO MOSFET] " + msg);
}

void sendToSensor(const String &msg) {
  SensorSerial.println(msg);
  printlnBoth("[TO SENSOR] " + msg);
}

bool isLimitHitForStage(int stageNum) {
  switch (stageNum) {
    case 2: return stage2LimitHit;
    case 3: return stage3LimitHit;
    case 4: return stage4LimitHit;
    default: return false;
  }
}

bool isLimitHitForMotor(int motorIndex) {
  if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) return stage2LimitHit;
  if (motorIndex == STAGE3) return stage3LimitHit;
  return false;
}

void printLimitStatus() {
  printlnBoth("LIMITS S2=" + String(stage2LimitHit ? "1" : "0") +
              " S3=" + String(stage3LimitHit ? "1" : "0") +
              " S4=" + String(stage4LimitHit ? "1" : "0"));
}

void setDirectionRaw(int motorIndex, bool forward) {
  digitalWrite(DIR_PINS[motorIndex], forward ? HIGH : LOW);
}

void setDirection(int motorIndex, bool forward) {
  bool actualForward = invertMotor[motorIndex] ? !forward : forward;
  setDirectionRaw(motorIndex, actualForward);
  delayMicroseconds(50);
}

bool stepMotor(int motorIndex, bool forward, int steps) {
  if (isLimitHitForMotor(motorIndex)) {
    printlnBoth("ABORT: limit already active for controller " + String(motorIndex + 1));
    return false;
  }

  setDirection(motorIndex, forward);

  for (int i = 0; i < steps; i++) {
    if (isLimitHitForMotor(motorIndex)) {
      printlnBoth("STOP: limit hit on controller " + String(motorIndex + 1));
      return false;
    }

    digitalWrite(STEP_PINS[motorIndex], HIGH);
    delayMicroseconds(pulseDelayUs);
    digitalWrite(STEP_PINS[motorIndex], LOW);
    delayMicroseconds(pulseDelayUs);
  }

  return true;
}

bool stepStage2(bool forward, int steps) {
  if (stage2LimitHit) {
    printlnBoth("ABORT: Stage 2 limit already active");
    return false;
  }

  setDirection(STAGE2_RIGHT, forward);
  setDirection(STAGE2_LEFT, forward);

  for (int i = 0; i < steps; i++) {
    if (stage2LimitHit) {
      printlnBoth("STOP: Stage 2 limit hit");
      return false;
    }

    digitalWrite(STEP_PINS[STAGE2_RIGHT], HIGH);
    digitalWrite(STEP_PINS[STAGE2_LEFT], HIGH);
    delayMicroseconds(pulseDelayUs);

    digitalWrite(STEP_PINS[STAGE2_RIGHT], LOW);
    digitalWrite(STEP_PINS[STAGE2_LEFT], LOW);
    delayMicroseconds(pulseDelayUs);
  }

  return true;
}

void runControllerCommand(int controllerNum, bool forward, int steps) {
  int motorIndex = controllerNum - 1;

  if (motorIndex < 0 || motorIndex >= NUM_DRIVERS) {
    printlnBoth("Invalid controller number");
    return;
  }

  bool ok = stepMotor(motorIndex, forward, steps);

  if (ok) {
    printlnBoth("Controller " + String(controllerNum) +
                (forward ? " forward " : " reverse ") +
                String(steps) + " steps");
  } else {
    printlnBoth("Controller " + String(controllerNum) + " aborted");
  }
}

void printHelp() {
  printlnBoth("");
  printlnBoth("Stage / motor commands:");
  printlnBoth("  s2f 400     -> stage 2 forward 400 steps");
  printlnBoth("  s2r 400     -> stage 2 reverse 400 steps");
  printlnBoth("  s3f 400     -> stage 3 forward 400 steps");
  printlnBoth("  s3r 400     -> stage 3 reverse 400 steps");
  printlnBoth("  c1f 400     -> controller 1 forward 400 steps");
  printlnBoth("  c1r 400     -> controller 1 reverse 400 steps");
  printlnBoth("  c2f 400     -> controller 2 forward 400 steps");
  printlnBoth("  c2r 400     -> controller 2 reverse 400 steps");
  printlnBoth("  c4f 400     -> controller 4 forward 400 steps");
  printlnBoth("  c4r 400     -> controller 4 reverse 400 steps");
  printlnBoth("  speed 800   -> set pulse delay in microseconds");
  printlnBoth("");

  printlnBoth("MOSFET commands:");
  printlnBoth("  mag on      -> electromagnet on");
  printlnBoth("  mag off     -> electromagnet off");
  printlnBoth("  vac on      -> vacuum on");
  printlnBoth("  vac off     -> vacuum off");
  printlnBoth("  saw on      -> saw on");
  printlnBoth("  saw off     -> saw off");
  printlnBoth("  alloff      -> all MOSFET outputs off");
  printlnBoth("  mstatus     -> request MOSFET status");
  printlnBoth("");

  printlnBoth("Sensor commands:");
  printlnBoth("  limits      -> print latest limit states");
  printlnBoth("  lstatus     -> request fresh sensor status");
  printlnBoth("");

  printlnBoth("  help        -> print this menu");
  printlnBoth("");
  printlnBoth("Current pulse delay: " + String(pulseDelayUs) + " us");
  printlnBoth("");
}

// =========================
// Parse sensor message
// Expected format: LIM S2=1 S3=0 S4=1
// =========================
void parseSensorMessage(String msg) {
  msg.trim();
  msg.toUpperCase();

  if (!msg.startsWith("LIM ")) return;

  int s2pos = msg.indexOf("S2=");
  int s3pos = msg.indexOf("S3=");
  int s4pos = msg.indexOf("S4=");

  if (s2pos >= 0 && s2pos + 3 < (int)msg.length()) {
    stage2LimitHit = (msg.charAt(s2pos + 3) == '1');
  }
  if (s3pos >= 0 && s3pos + 3 < (int)msg.length()) {
    stage3LimitHit = (msg.charAt(s3pos + 3) == '1');
  }
  if (s4pos >= 0 && s4pos + 3 < (int)msg.length()) {
    stage4LimitHit = (msg.charAt(s4pos + 3) == '1');
  }

  printlnBoth("[SENSOR] " + msg);
}

// =========================
// Command handling
// =========================
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "help") {
    printHelp();
    return;
  }

  if (cmd.startsWith("speed ")) {
    int newDelay = cmd.substring(6).toInt();
    if (newDelay >= 100) {
      pulseDelayUs = newDelay;
      printlnBoth("Pulse delay set to " + String(pulseDelayUs) + " us");
    } else {
      printlnBoth("Use a value >= 100");
    }
    return;
  }

  // Stage 2 pair
  if (cmd.startsWith("s2f ")) {
    int steps = cmd.substring(4).toInt();
    if (steps > 0) {
      bool ok = stepStage2(true, steps);
      printlnBoth(ok ? "Stage 2 forward done" : "Stage 2 forward aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s2r ")) {
    int steps = cmd.substring(4).toInt();
    if (steps > 0) {
      bool ok = stepStage2(false, steps);
      printlnBoth(ok ? "Stage 2 reverse done" : "Stage 2 reverse aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Stage 3 single motor
  if (cmd.startsWith("s3f ")) {
    int steps = cmd.substring(4).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, true, steps);
      printlnBoth(ok ? "Stage 3 forward done" : "Stage 3 forward aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s3r ")) {
    int steps = cmd.substring(4).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, false, steps);
      printlnBoth(ok ? "Stage 3 reverse done" : "Stage 3 reverse aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Controller commands: c1f 400
  if (cmd.length() >= 5 && cmd.charAt(0) == 'c') {
    int controllerNum = cmd.charAt(1) - '0';
    char dirChar = cmd.charAt(2);
    int spacePos = cmd.indexOf(' ');

    if (controllerNum >= 1 && controllerNum <= 5 &&
        (dirChar == 'f' || dirChar == 'r') &&
        spacePos > 0) {

      int steps = cmd.substring(spacePos + 1).toInt();

      if (steps > 0) {
        bool forward = (dirChar == 'f');
        runControllerCommand(controllerNum, forward, steps);
      } else {
        printlnBoth("Invalid step count");
      }
      return;
    }
  }

  // MOSFET / slave commands
  if (cmd == "mag on")  { sendToMosfet("MAG ON");  return; }
  if (cmd == "mag off") { sendToMosfet("MAG OFF"); return; }

  if (cmd == "vac on")  { sendToMosfet("VAC ON");  return; }
  if (cmd == "vac off") { sendToMosfet("VAC OFF"); return; }

  if (cmd == "saw on")  { sendToMosfet("SAW ON");  return; }
  if (cmd == "saw off") { sendToMosfet("SAW OFF"); return; }

  if (cmd == "alloff")  { sendToMosfet("ALL OFF"); return; }
  if (cmd == "mstatus") { sendToMosfet("STATUS");  return; }

  // Sensor commands
  if (cmd == "limits")  { printLimitStatus(); return; }
  if (cmd == "lstatus") { sendToSensor("STATUS"); return; }

  printlnBoth("Unknown command. Type 'help'");
}

// =========================
// Readers
// =========================
void readFromSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void readFromBluetooth() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (btBuffer.length() > 0) {
        handleCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

void readFromMosfet() {
  while (MosfetSerial.available()) {
    char c = (char)MosfetSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (mosfetBuffer.length() > 0) {
        printlnBoth("[MOSFET] " + mosfetBuffer);
        mosfetBuffer = "";
      }
    } else {
      mosfetBuffer += c;
    }
  }
}

void readFromSensor() {
  while (SensorSerial.available()) {
    char c = (char)SensorSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (sensorBuffer.length() > 0) {
        parseSensorMessage(sensorBuffer);
        sensorBuffer = "";
      }
    } else {
      sensorBuffer += c;
    }
  }
}

void setup() {
  for (int i = 0; i < NUM_DRIVERS; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }

  Serial.begin(115200);
  SerialBT.begin("ESP32_STAGE_CTRL");

  MosfetSerial.begin(MOSFET_BAUD, SERIAL_8N1, MOSFET_RX_PIN, MOSFET_TX_PIN);
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  delay(1000);

  printlnBoth("Main board ready");
  printlnBoth("Bluetooth name: ESP32_STAGE_CTRL");
  sendToSensor("STATUS");
  printHelp();
}

void loop() {
  readFromSerial();
  readFromBluetooth();
  readFromMosfet();
  readFromSensor();
}
