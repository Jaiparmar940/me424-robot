#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// =========================
// Stepper config
// =========================
const int NUM_DRIVERS = 6;

const int STEP_PINS[NUM_DRIVERS] = {25, 33, 32, 27, 26, 4};
const int DIR_PINS[NUM_DRIVERS]  = {13, 14, 22, 21, 23, 15};

int pulseDelayUs = 1000;
long currentPos[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
volatile bool estopLatched = false;
String estopSerialBuffer = "";
String estopBtBuffer = "";

enum QueueItemType {
  Q_POSE = 0,
  Q_DELAY = 1,
  Q_CMD = 2
};

struct QueueItem {
  QueueItemType type;
  long targets[NUM_DRIVERS];
  long maxSps;
  int rampSteps;
  unsigned long delayMs;
  String cmd;
};

const int MAX_QUEUE_ITEMS = 64;
QueueItem runQueueItems[MAX_QUEUE_ITEMS];
int runQueueCount = 0;
bool queueStopRequested = false;

// Stage-to-controller mapping:
// Stage 1 = turntable          -> controller 6 (index 5)
// Stage 2 = base lift pair     -> controllers 1+4 (index 0+3)
// Stage 3 = arm segment        -> controller 2 (index 1)
// Stage 4 = arm segment        -> controller 3 (index 2)
// Stage 5 = wrist              -> controller 5 (index 4)

const int STAGE1       = 5;   // controller 6
const int STAGE2_RIGHT = 0;   // controller 1
const int STAGE3       = 1;   // controller 2
const int STAGE4       = 2;   // controller 3
const int STAGE2_LEFT  = 3;   // controller 4
const int STAGE5       = 4;   // controller 5

void enforceStage2PairTarget(long targets[NUM_DRIVERS]) {
  // Stage 2 must always remain paired (controllers 1 and 4).
  // Use C1 as the authoritative value when both are provided.
  targets[STAGE2_LEFT] = targets[STAGE2_RIGHT];
}

bool invertMotor[NUM_DRIVERS] = {
  false, // controller 1 / stage 2 right
  false, // controller 2 / stage 3
  true,  // controller 3 / stage 4 (reversed)
  true,  // controller 4 / stage 2 left
  false, // controller 5 / stage 5
  false  // controller 6 / stage 1 (turntable)
};

// =========================
// UART to MOSFET slave
// Assumes RX2/TX2 map to GPIO16/GPIO17 on your board.
// If not, change these.
// =========================
HardwareSerial SlaveSerial(2);
const int SLAVE_RX_PIN = 16;   // main RX2
const int SLAVE_TX_PIN = 17;   // main TX2
const long SLAVE_BAUD = 115200;

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
String slaveBuffer = "";
String sensorBuffer = "";

// Forward declarations used by queue/emergency helpers
void handleCommand(String cmd);
void readFromSlave();

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

void sendToSlave(const String &msg) {
  SlaveSerial.println(msg);
  printlnBoth("[TO SLAVE] " + msg);
}

void parseEStopLine(String line) {
  line.trim();
  line.toLowerCase();
  if (line == "estop") {
    estopLatched = true;
    sendToSlave("ALL OFF");
    printlnBoth("ESTOP LATCHED");
  } else if (line == "estop clear") {
    estopLatched = false;
    printlnBoth("ESTOP CLEARED");
  }
}

void pollEmergencyInputs() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (estopSerialBuffer.length() > 0) {
        parseEStopLine(estopSerialBuffer);
        estopSerialBuffer = "";
      }
    } else {
      estopSerialBuffer += c;
    }
  }

  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (estopBtBuffer.length() > 0) {
        parseEStopLine(estopBtBuffer);
        estopBtBuffer = "";
      }
    } else {
      estopBtBuffer += c;
    }
  }
}

void printLimitStatus() {
  printlnBoth("LIMITS S2=" + String(stage2LimitHit ? "1" : "0") +
              " S3=" + String(stage3LimitHit ? "1" : "0") +
              " S4=" + String(stage4LimitHit ? "1" : "0"));
}

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

void serviceSensorUART() {
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

// =========================
// Direction-aware limit logic
//
// Stage 2:  s2down -> forward=false (blocked by limit)
// Stage 3:  s3down -> forward=true  (blocked by limit)
// Stage 4:  s4down -> forward=true  (blocked by limit)
// Stage 1, 5: no limit switches
// =========================
bool shouldStopMotor(int motorIndex, bool forward) {
  if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) {
    bool movingDown = !forward;
    return stage2LimitHit && movingDown;
  }

  if (motorIndex == STAGE3) {
    bool movingDown = forward;
    return stage3LimitHit && movingDown;
  }

  if (motorIndex == STAGE4) {
    bool movingDown = forward;
    return stage4LimitHit && movingDown;
  }

  return false;
}

void setDirectionRaw(int motorIndex, bool forward) {
  digitalWrite(DIR_PINS[motorIndex], forward ? HIGH : LOW);
}

void setDirection(int motorIndex, bool forward) {
  bool actualForward = invertMotor[motorIndex] ? !forward : forward;
  setDirectionRaw(motorIndex, actualForward);
  delayMicroseconds(50);
}

// =========================
// Motion functions
// =========================
bool stepMotor(int motorIndex, bool forward, int steps) {
  if (estopLatched) {
    printlnBoth("ESTOP: motion blocked");
    return false;
  }
  serviceSensorUART();
  int movedSteps = 0;

  if (shouldStopMotor(motorIndex, forward)) {
    printlnBoth("ABORT: limit active for controller " + String(motorIndex + 1) +
                " in requested direction");
    return false;
  }

  setDirection(motorIndex, forward);

  for (int i = 0; i < steps; i++) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnBoth("ESTOP: controller " + String(motorIndex + 1) + " stopped");
      currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
      return false;
    }
    serviceSensorUART();

    if (shouldStopMotor(motorIndex, forward)) {
      printlnBoth("STOP: limit hit on controller " + String(motorIndex + 1));
      currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
      return false;
    }

    digitalWrite(STEP_PINS[motorIndex], HIGH);
    delayMicroseconds(pulseDelayUs);
    digitalWrite(STEP_PINS[motorIndex], LOW);
    delayMicroseconds(pulseDelayUs);
    movedSteps++;
  }

  currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
  return true;
}

bool stepStage2(bool forward, int steps) {
  if (estopLatched) {
    printlnBoth("ESTOP: motion blocked");
    return false;
  }
  serviceSensorUART();
  int movedSteps = 0;

  bool movingDown = !forward;
  if (stage2LimitHit && movingDown) {
    printlnBoth("ABORT: Stage 2 limit already active for downward motion");
    return false;
  }

  setDirection(STAGE2_RIGHT, forward);
  setDirection(STAGE2_LEFT, forward);

  for (int i = 0; i < steps; i++) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnBoth("ESTOP: Stage 2 stopped");
      long d = forward ? movedSteps : -movedSteps;
      currentPos[STAGE2_RIGHT] += d;
      currentPos[STAGE2_LEFT]  += d;
      return false;
    }
    serviceSensorUART();

    if (stage2LimitHit && movingDown) {
      printlnBoth("STOP: Stage 2 limit hit");
      long delta = forward ? movedSteps : -movedSteps;
      currentPos[STAGE2_RIGHT] += delta;
      currentPos[STAGE2_LEFT]  += delta;
      return false;
    }

    digitalWrite(STEP_PINS[STAGE2_RIGHT], HIGH);
    digitalWrite(STEP_PINS[STAGE2_LEFT], HIGH);
    delayMicroseconds(pulseDelayUs);

    digitalWrite(STEP_PINS[STAGE2_RIGHT], LOW);
    digitalWrite(STEP_PINS[STAGE2_LEFT], LOW);
    delayMicroseconds(pulseDelayUs);
    movedSteps++;
  }

  long delta = forward ? movedSteps : -movedSteps;
  currentPos[STAGE2_RIGHT] += delta;
  currentPos[STAGE2_LEFT]  += delta;
  return true;
}

void runControllerCommand(int controllerNum, bool forward, int steps) {
  int motorIndex = controllerNum - 1;

  if (motorIndex < 0 || motorIndex >= NUM_DRIVERS) {
    printlnBoth("Invalid controller number");
    return;
  }

  bool ok = false;
  if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) {
    // Always keep stage 2 pair mechanically locked.
    ok = stepStage2(forward, steps);
  } else {
    ok = stepMotor(motorIndex, forward, steps);
  }

  if (ok) {
    printlnBoth("Controller " + String(controllerNum) +
                (forward ? " forward " : " reverse ") +
                String(steps) + " steps");
  } else {
    printlnBoth("Controller " + String(controllerNum) + " aborted");
  }
}

// =========================
// Sequential / parallel execution
//
// MotorMove holds a single motor's move parameters.
// Stage 2 expands to two MotorMove entries (RIGHT + LEFT).
// =========================
struct MotorMove {
  int  motorIndex;
  bool forward;
  int  stepsRemaining;
  int  originalSteps;
};

// Translate one stage/controller command string into MotorMove entries.
// Returns number of entries written (0 = unrecognised or bad step count).
int resolveToMoves(String cmd, MotorMove *moves, int maxMoves) {
  cmd.trim();
  cmd.toLowerCase();

  int steps = 0;
  int spPos = cmd.lastIndexOf(' ');
  if (spPos > 0) steps = cmd.substring(spPos + 1).toInt();
  if (steps <= 0) return 0;

  // Stage 1 — turntable
  if (cmd.startsWith("s1cw "))  { moves[0] = {STAGE1, true,  steps, steps}; return 1; }
  if (cmd.startsWith("s1ccw ")) { moves[0] = {STAGE1, false, steps, steps}; return 1; }

  // Stage 2 — paired motors
  if (cmd.startsWith("s2up ")) {
    if (maxMoves < 2) return 0;
    moves[0] = {STAGE2_RIGHT, true, steps, steps};
    moves[1] = {STAGE2_LEFT,  true, steps, steps};
    return 2;
  }
  if (cmd.startsWith("s2down ")) {
    if (maxMoves < 2) return 0;
    moves[0] = {STAGE2_RIGHT, false, steps, steps};
    moves[1] = {STAGE2_LEFT,  false, steps, steps};
    return 2;
  }

  // Stage 3
  if (cmd.startsWith("s3up "))   { moves[0] = {STAGE3, false, steps, steps}; return 1; }
  if (cmd.startsWith("s3down ")) { moves[0] = {STAGE3, true,  steps, steps}; return 1; }

  // Stage 4
  if (cmd.startsWith("s4up "))   { moves[0] = {STAGE4, false, steps, steps}; return 1; }
  if (cmd.startsWith("s4down ")) { moves[0] = {STAGE4, true,  steps, steps}; return 1; }

  // Stage 5
  if (cmd.startsWith("s5cw "))   { moves[0] = {STAGE5, false, steps, steps}; return 1; }
  if (cmd.startsWith("s5ccw "))  { moves[0] = {STAGE5, true,  steps, steps}; return 1; }

  // Raw controller: cXf / cXr
  if (cmd.length() >= 5 && cmd.charAt(0) == 'c') {
    int num     = cmd.charAt(1) - '0';
    char dirCh  = cmd.charAt(2);
    if (num >= 1 && num <= 6 && (dirCh == 'f' || dirCh == 'r')) {
      if (num == 1 || num == 4) {
        if (maxMoves < 2) return 0;
        bool fwd = (dirCh == 'f');
        moves[0] = {STAGE2_RIGHT, fwd, steps, steps};
        moves[1] = {STAGE2_LEFT,  fwd, steps, steps};
        return 2;
      }
      moves[0] = {num - 1, dirCh == 'f', steps, steps};
      return 1;
    }
  }

  return 0;
}

// Step multiple motors in parallel.
// Each motor is pulsed on the same clock tick; motors with fewer steps
// finish early while the others keep running. Limits are checked per-motor
// every iteration.
bool stepMultiple(MotorMove *moves, int count) {
  if (estopLatched) {
    printlnBoth("ESTOP: motion blocked");
    return false;
  }
  // Pre-flight limit check
  for (int i = 0; i < count; i++) {
    if (shouldStopMotor(moves[i].motorIndex, moves[i].forward)) {
      printlnBoth("PAR ABORT: limit active for controller " +
                  String(moves[i].motorIndex + 1));
      return false;
    }
  }

  // Set all directions before any stepping starts
  for (int i = 0; i < count; i++) {
    setDirection(moves[i].motorIndex, moves[i].forward);
  }

  while (true) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnBoth("ESTOP: parallel motion stopped");
      for (int i = 0; i < count; i++) {
        int moved = moves[i].originalSteps - moves[i].stepsRemaining;
        currentPos[moves[i].motorIndex] += moves[i].forward ? moved : -moved;
      }
      return false;
    }
    serviceSensorUART();

    // Check limits and count still-active motors
    int active = 0;
    for (int i = 0; i < count; i++) {
      if (moves[i].stepsRemaining <= 0) continue;
      if (shouldStopMotor(moves[i].motorIndex, moves[i].forward)) {
        printlnBoth("PAR STOP: limit hit on controller " +
                    String(moves[i].motorIndex + 1));
        moves[i].stepsRemaining = 0;
        continue;
      }
      active++;
    }
    if (active == 0) break;

    // Pulse HIGH for all active motors
    for (int i = 0; i < count; i++) {
      if (moves[i].stepsRemaining > 0)
        digitalWrite(STEP_PINS[moves[i].motorIndex], HIGH);
    }
    delayMicroseconds(pulseDelayUs);

    // Pulse LOW and decrement
    for (int i = 0; i < count; i++) {
      if (moves[i].stepsRemaining > 0) {
        digitalWrite(STEP_PINS[moves[i].motorIndex], LOW);
        moves[i].stepsRemaining--;
      }
    }
    delayMicroseconds(pulseDelayUs);
  }

  for (int i = 0; i < count; i++) {
    int moved = moves[i].originalSteps - moves[i].stepsRemaining;
    currentPos[moves[i].motorIndex] += moves[i].forward ? moved : -moved;
  }
  return true;
}

int parseSyncAbsArgs(const String &cmd, long targets[NUM_DRIVERS], long &maxSps, int &rampSteps) {
  // Format:
  // syncabs t1 t2 t3 t4 t5 t6 max_sps ramp_steps
  return sscanf(cmd.c_str(),
                "syncabs %ld %ld %ld %ld %ld %ld %ld %d",
                &targets[0], &targets[1], &targets[2],
                &targets[3], &targets[4], &targets[5],
                &maxSps, &rampSteps);
}

bool runSyncAbs(long targets[NUM_DRIVERS], long maxSps, int rampSteps) {
  if (estopLatched) {
    printlnBoth("ESTOP: motion blocked");
    return false;
  }
  if (maxSps < 1) {
    printlnBoth("SYNCABS: max_sps must be >= 1");
    return false;
  }

  enforceStage2PairTarget(targets);

  long delta[NUM_DRIVERS];
  long absDelta[NUM_DRIVERS];
  bool forward[NUM_DRIVERS];
  long accum[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
  long totalSteps = 0;

  for (int i = 0; i < NUM_DRIVERS; i++) {
    delta[i] = targets[i] - currentPos[i];
    absDelta[i] = labs(delta[i]);
    forward[i] = (delta[i] >= 0);
    if (absDelta[i] > totalSteps) totalSteps = absDelta[i];
  }

  if (totalSteps == 0) {
    printlnBoth("SYNCABS: already at target");
    return true;
  }

  for (int i = 0; i < NUM_DRIVERS; i++) {
    if (absDelta[i] == 0) continue;
    if (shouldStopMotor(i, forward[i])) {
      printlnBoth("SYNCABS ABORT: limit active on controller " + String(i + 1));
      return false;
    }
    setDirection(i, forward[i]);
  }

  long minDelayUs = 500000L / maxSps; // half-cycle delay
  if (minDelayUs < 100) minDelayUs = 100;
  long slowDelayUs = max((long)pulseDelayUs, minDelayUs * 3L);
  if (rampSteps < 1) rampSteps = 1;
  if ((long)rampSteps * 2L > totalSteps) rampSteps = (int)(totalSteps / 2L);
  if (rampSteps < 1) rampSteps = 1;

  for (long m = 0; m < totalSteps; m++) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnBoth("ESTOP: sync motion stopped");
      return false;
    }
    serviceSensorUART();

    // Ramp profile on the shared timeline
    long dUs = minDelayUs;
    if (m < rampSteps) {
      long num = (long)(rampSteps - m) * (slowDelayUs - minDelayUs);
      dUs = minDelayUs + num / rampSteps;
    } else if (m >= totalSteps - rampSteps) {
      long phase = m - (totalSteps - rampSteps);
      long num = phase * (slowDelayUs - minDelayUs);
      dUs = minDelayUs + num / rampSteps;
    }

    bool pulseThisTick[NUM_DRIVERS] = {false, false, false, false, false, false};

    // Determine which axes step this master tick (Bresenham style)
    for (int i = 0; i < NUM_DRIVERS; i++) {
      if (absDelta[i] == 0) continue;
      if (shouldStopMotor(i, forward[i])) {
        printlnBoth("SYNCABS STOP: limit hit on controller " + String(i + 1));
        // Snap done axes only; interrupted axis keeps current position
        for (int j = 0; j < NUM_DRIVERS; j++) {
          if (absDelta[j] > 0 && accum[j] >= totalSteps) {
            currentPos[j] = targets[j];
          }
        }
        return false;
      }
      accum[i] += absDelta[i];
      if (accum[i] >= totalSteps) {
        accum[i] -= totalSteps;
        pulseThisTick[i] = true;
      }
    }

    for (int i = 0; i < NUM_DRIVERS; i++) {
      if (pulseThisTick[i]) digitalWrite(STEP_PINS[i], HIGH);
    }
    delayMicroseconds((int)dUs);
    for (int i = 0; i < NUM_DRIVERS; i++) {
      if (pulseThisTick[i]) digitalWrite(STEP_PINS[i], LOW);
    }
    delayMicroseconds((int)dUs);
  }

  for (int i = 0; i < NUM_DRIVERS; i++) {
    currentPos[i] = targets[i];
  }
  return true;
}

bool queueAddPose(long targets[NUM_DRIVERS], long maxSps, int rampSteps) {
  if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
  enforceStage2PairTarget(targets);
  QueueItem &it = runQueueItems[runQueueCount++];
  it.type = Q_POSE;
  for (int i = 0; i < NUM_DRIVERS; i++) it.targets[i] = targets[i];
  it.maxSps = maxSps;
  it.rampSteps = rampSteps;
  it.delayMs = 0;
  it.cmd = "";
  return true;
}

bool queueAddDelay(unsigned long ms) {
  if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
  QueueItem &it = runQueueItems[runQueueCount++];
  it.type = Q_DELAY;
  it.maxSps = 0;
  it.rampSteps = 0;
  it.delayMs = ms;
  it.cmd = "";
  return true;
}

bool queueAddCmd(const String &cmd) {
  if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
  QueueItem &it = runQueueItems[runQueueCount++];
  it.type = Q_CMD;
  it.maxSps = 0;
  it.rampSteps = 0;
  it.delayMs = 0;
  it.cmd = cmd;
  return true;
}

bool queueDelayWait(unsigned long ms) {
  unsigned long t0 = millis();
  while ((millis() - t0) < ms) {
    pollEmergencyInputs();
    serviceSensorUART();
    readFromSlave();
    if (estopLatched || queueStopRequested) return false;
    delay(2);
  }
  return true;
}

bool runQueueExecution() {
  if (runQueueCount == 0) {
    printlnBoth("QRUN: queue empty");
    return false;
  }

  if (estopLatched) {
    printlnBoth("QRUN: blocked by ESTOP");
    return false;
  }

  queueStopRequested = false;
  printlnBoth("QRUN: start");

  for (int i = 0; i < runQueueCount; i++) {
    pollEmergencyInputs();
    if (estopLatched || queueStopRequested) {
      printlnBoth("QRUN: stopped");
      return false;
    }

    QueueItem &it = runQueueItems[i];
    if (it.type == Q_POSE) {
      bool ok = runSyncAbs(it.targets, it.maxSps, it.rampSteps);
      if (!ok) {
        printlnBoth("QRUN: pose aborted at item " + String(i));
        return false;
      }
    } else if (it.type == Q_DELAY) {
      if (!queueDelayWait(it.delayMs)) {
        printlnBoth("QRUN: delay interrupted at item " + String(i));
        return false;
      }
    } else if (it.type == Q_CMD) {
      handleCommand(it.cmd);
    }
  }

  printlnBoth("QRUN: done");
  return true;
}

// Forward declaration so runSequential can call handleCommand
void handleCommand(String cmd);

// Split cmdList by ',' and execute each sub-command in order.
// Any command type (motor, MOSFET, utility) is supported.
void runSequential(const String &cmdList) {
  int start = 0;
  int idx   = 0;
  while (start < (int)cmdList.length()) {
    int comma = cmdList.indexOf(',', start);
    String sub = (comma < 0)
                   ? cmdList.substring(start)
                   : cmdList.substring(start, comma);
    sub.trim();
    if (sub.length() > 0) {
      printlnBoth("SEQ[" + String(idx++) + "]: " + sub);
      handleCommand(sub);
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  printlnBoth("SEQ: done");
}

// Split cmdList by ',', resolve each to motor moves, run all simultaneously.
// Only motor/stage commands are supported in parallel mode.
void runParallel(const String &cmdList) {
  // Max 12 moves: up to 6 commands, stage2 counts as 2
  MotorMove moves[12];
  int totalMoves = 0;

  int start = 0;
  while (start < (int)cmdList.length()) {
    int comma = cmdList.indexOf(',', start);
    String sub = (comma < 0)
                   ? cmdList.substring(start)
                   : cmdList.substring(start, comma);
    sub.trim();

    if (sub.length() > 0) {
      int added = resolveToMoves(sub, moves + totalMoves, 12 - totalMoves);
      if (added == 0) {
        printlnBoth("PAR: unrecognised motor command: " + sub);
        return;
      }
      totalMoves += added;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  if (totalMoves == 0) {
    printlnBoth("PAR: no valid motor commands");
    return;
  }

  bool ok = stepMultiple(moves, totalMoves);
  printlnBoth(ok ? "PAR: done" : "PAR: aborted");
}

void printHelp() {
  printlnBoth("");
  printlnBoth("Stage commands:");
  printlnBoth("  s1cw 400      -> stage 1 (turntable) clockwise");
  printlnBoth("  s1ccw 400     -> stage 1 (turntable) counter-clockwise");
  printlnBoth("  s2up 400      -> stage 2 up 400 steps");
  printlnBoth("  s2down 400    -> stage 2 down 400 steps");
  printlnBoth("  s3up 400      -> stage 3 up 400 steps");
  printlnBoth("  s3down 400    -> stage 3 down 400 steps");
  printlnBoth("  s4up 400      -> stage 4 up 400 steps");
  printlnBoth("  s4down 400    -> stage 4 down 400 steps");
  printlnBoth("  s5cw 400      -> stage 5 clockwise 400 steps");
  printlnBoth("  s5ccw 400     -> stage 5 counter-clockwise 400 steps");
  printlnBoth("");

  printlnBoth("Controller commands (raw):");
  printlnBoth("  c1f 400       -> controller 1 forward 400 steps");
  printlnBoth("  c1r 400       -> controller 1 reverse 400 steps");
  printlnBoth("  c2f 400       -> controller 2 forward 400 steps");
  printlnBoth("  c2r 400       -> controller 2 reverse 400 steps");
  printlnBoth("  c3f 400       -> controller 3 forward 400 steps");
  printlnBoth("  c3r 400       -> controller 3 reverse 400 steps");
  printlnBoth("  c4f 400       -> controller 4 forward 400 steps");
  printlnBoth("  c4r 400       -> controller 4 reverse 400 steps");
  printlnBoth("  c5f 400       -> controller 5 forward 400 steps");
  printlnBoth("  c5r 400       -> controller 5 reverse 400 steps");
  printlnBoth("  c6f 400       -> controller 6 forward 400 steps");
  printlnBoth("  c6r 400       -> controller 6 reverse 400 steps");
  printlnBoth("");

  printlnBoth("Sequence commands:");
  printlnBoth("  seq s2up 400, s3down 200, mag on   -> run in order");
  printlnBoth("  par s2up 400, s5cw 200             -> run simultaneously");
  printlnBoth("  (par supports motor/stage commands only)");
  printlnBoth("  syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps");
  printlnBoth("     -> synchronized absolute move with accel/decel ramp");
  printlnBoth("  qclear");
  printlnBoth("  qadd pose t1 t2 t3 t4 t5 t6 maxSps rampSteps");
  printlnBoth("  qadd delay ms");
  printlnBoth("  qadd cmd <firmware command>");
  printlnBoth("  qlist");
  printlnBoth("  qrun");
  printlnBoth("  qstop");
  printlnBoth("  qstatus");
  printlnBoth("");

  printlnBoth("Other:");
  printlnBoth("  speed 800     -> set pulse delay in microseconds");
  printlnBoth("  where         -> print current tracked positions");
  printlnBoth("  setpos p1 p2 p3 p4 p5 p6 -> overwrite tracked positions");
  printlnBoth("  estop         -> latch emergency stop (blocks all motion)");
  printlnBoth("  estop clear   -> clear e-stop latch");
  printlnBoth("  estop status  -> print e-stop state");
  printlnBoth("  limits        -> print latest limit states");
  printlnBoth("");

  printlnBoth("MOSFET / attachment commands:");
  printlnBoth("  mag on");
  printlnBoth("  mag off");
  printlnBoth("  vac on");
  printlnBoth("  vac off");
  printlnBoth("  saw on");
  printlnBoth("  saw off");
  printlnBoth("  alloff");
  printlnBoth("  mstatus");
  printlnBoth("");

  printlnBoth("  help");
  printlnBoth("");
  printlnBoth("Current pulse delay: " + String(pulseDelayUs) + " us");
  printlnBoth("");
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

  if (cmd == "limits") {
    printLimitStatus();
    return;
  }

  if (cmd == "estop") {
    estopLatched = true;
    sendToSlave("ALL OFF");
    printlnBoth("ESTOP LATCHED");
    return;
  }

  if (cmd == "estop clear") {
    estopLatched = false;
    printlnBoth("ESTOP CLEARED");
    return;
  }

  if (cmd == "estop status") {
    printlnBoth(String("ESTOP=") + (estopLatched ? "1" : "0"));
    return;
  }

  if (cmd == "qclear") {
    runQueueCount = 0;
    queueStopRequested = false;
    printlnBoth("Q: cleared");
    return;
  }

  if (cmd == "qstop") {
    queueStopRequested = true;
    printlnBoth("Q: stop requested");
    return;
  }

  if (cmd == "qstatus") {
    printlnBoth("Q: count=" + String(runQueueCount) +
                " stop=" + String(queueStopRequested ? "1" : "0"));
    return;
  }

  if (cmd == "qlist") {
    printlnBoth("Q: items=" + String(runQueueCount));
    for (int i = 0; i < runQueueCount; i++) {
      QueueItem &it = runQueueItems[i];
      if (it.type == Q_POSE) {
        printlnBoth("Q[" + String(i) + "] POSE " +
                    String(it.targets[0]) + " " + String(it.targets[1]) + " " +
                    String(it.targets[2]) + " " + String(it.targets[3]) + " " +
                    String(it.targets[4]) + " " + String(it.targets[5]) +
                    " maxSps=" + String(it.maxSps) +
                    " ramp=" + String(it.rampSteps));
      } else if (it.type == Q_DELAY) {
        printlnBoth("Q[" + String(i) + "] DELAY " + String(it.delayMs) + "ms");
      } else {
        printlnBoth("Q[" + String(i) + "] CMD " + it.cmd);
      }
    }
    return;
  }

  if (cmd == "qrun") {
    runQueueExecution();
    return;
  }

  if (cmd.startsWith("qadd pose ")) {
    long t[NUM_DRIVERS];
    long maxSps;
    int rampSteps;
    if (sscanf(cmd.c_str(),
               "qadd pose %ld %ld %ld %ld %ld %ld %ld %d",
               &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &maxSps, &rampSteps) == 8) {
      if (queueAddPose(t, maxSps, rampSteps)) {
        printlnBoth("Q: added pose");
      } else {
        printlnBoth("Q: full");
      }
    } else {
      printlnBoth("Usage: qadd pose t1 t2 t3 t4 t5 t6 maxSps rampSteps");
    }
    return;
  }

  if (cmd.startsWith("qadd delay ")) {
    unsigned long ms = (unsigned long)cmd.substring(11).toInt();
    if (ms > 0 || cmd.substring(11) == "0") {
      if (queueAddDelay(ms)) {
        printlnBoth("Q: added delay");
      } else {
        printlnBoth("Q: full");
      }
    } else {
      printlnBoth("Usage: qadd delay ms");
    }
    return;
  }

  if (cmd.startsWith("qadd cmd ")) {
    String qcmd = cmd.substring(9);
    qcmd.trim();
    if (qcmd.length() == 0) {
      printlnBoth("Usage: qadd cmd <firmware command>");
      return;
    }
    if (queueAddCmd(qcmd)) {
      printlnBoth("Q: added cmd");
    } else {
      printlnBoth("Q: full");
    }
    return;
  }

  if (cmd == "where") {
    // Keep pair visible as unified value in position reporting.
    currentPos[STAGE2_LEFT] = currentPos[STAGE2_RIGHT];
    printlnBoth("POS C1=" + String(currentPos[0]) +
                " C2=" + String(currentPos[1]) +
                " C3=" + String(currentPos[2]) +
                " C4=" + String(currentPos[3]) +
                " C5=" + String(currentPos[4]) +
                " C6=" + String(currentPos[5]));
    return;
  }

  if (cmd.startsWith("setpos ")) {
    long p[NUM_DRIVERS];
    if (sscanf(cmd.c_str(), "setpos %ld %ld %ld %ld %ld %ld",
               &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]) == 6) {
      // Stage 2 pair is always unified.
      p[STAGE2_LEFT] = p[STAGE2_RIGHT];
      for (int i = 0; i < NUM_DRIVERS; i++) currentPos[i] = p[i];
      printlnBoth("Position state updated");
    } else {
      printlnBoth("Usage: setpos p1 p2 p3 p4 p5 p6");
    }
    return;
  }

  if (cmd.startsWith("syncabs ")) {
    long targets[NUM_DRIVERS];
    long maxSps = 0;
    int rampSteps = 0;
    if (parseSyncAbsArgs(cmd, targets, maxSps, rampSteps) == 8) {
      bool ok = runSyncAbs(targets, maxSps, rampSteps);
      printlnBoth(ok ? "SYNCABS: done" : "SYNCABS: aborted");
    } else {
      printlnBoth("Usage: syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps");
    }
    return;
  }

  if (cmd.startsWith("seq ")) {
    runSequential(cmd.substring(4));
    return;
  }

  if (cmd.startsWith("par ")) {
    runParallel(cmd.substring(4));
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

  // Stage 1 (turntable)
  if (cmd.startsWith("s1cw ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE1, true, steps);
      printlnBoth(ok ? "Stage 1 CW done" : "Stage 1 CW aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s1ccw ")) {
    int steps = cmd.substring(6).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE1, false, steps);
      printlnBoth(ok ? "Stage 1 CCW done" : "Stage 1 CCW aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Stage 2
  if (cmd.startsWith("s2up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepStage2(true, steps);
      printlnBoth(ok ? "Stage 2 up done" : "Stage 2 up aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s2down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepStage2(false, steps);
      printlnBoth(ok ? "Stage 2 down done" : "Stage 2 down aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Stage 3
  if (cmd.startsWith("s3down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, true, steps);
      printlnBoth(ok ? "Stage 3 down done" : "Stage 3 down aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s3up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, false, steps);
      printlnBoth(ok ? "Stage 3 up done" : "Stage 3 up aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Stage 4
  if (cmd.startsWith("s4up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE4, false, steps);
      printlnBoth(ok ? "Stage 4 up done" : "Stage 4 up aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s4down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE4, true, steps);
      printlnBoth(ok ? "Stage 4 down done" : "Stage 4 down aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Stage 5
  if (cmd.startsWith("s5ccw ")) {
    int steps = cmd.substring(6).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE5, true, steps);
      printlnBoth(ok ? "Stage 5 CCW done" : "Stage 5 CCW aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  if (cmd.startsWith("s5cw ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE5, false, steps);
      printlnBoth(ok ? "Stage 5 CW done" : "Stage 5 CW aborted");
    } else {
      printlnBoth("Invalid step count");
    }
    return;
  }

  // Raw controller commands
  if (cmd.length() >= 5 && cmd.charAt(0) == 'c') {
    int controllerNum = cmd.charAt(1) - '0';
    char dirChar = cmd.charAt(2);
    int spacePos = cmd.indexOf(' ');

    if (controllerNum >= 1 && controllerNum <= 6 &&
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
  if (cmd == "mag on")  { sendToSlave("MAG ON");  return; }
  if (cmd == "mag off") { sendToSlave("MAG OFF"); return; }

  if (cmd == "vac on")  { sendToSlave("VAC ON");  return; }
  if (cmd == "vac off") { sendToSlave("VAC OFF"); return; }

  if (cmd == "saw on")  { sendToSlave("SAW ON");  return; }
  if (cmd == "saw off") { sendToSlave("SAW OFF"); return; }

  if (cmd == "alloff")  { sendToSlave("ALL OFF"); return; }
  if (cmd == "mstatus") { sendToSlave("STATUS");  return; }

  printlnBoth("Unknown command. Type 'help'");
}

// =========================
// Input readers
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

void readFromSlave() {
  while (SlaveSerial.available()) {
    char c = (char)SlaveSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (slaveBuffer.length() > 0) {
        printlnBoth("[SLAVE] " + slaveBuffer);
        slaveBuffer = "";
      }
    } else {
      slaveBuffer += c;
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

  if (!SerialBT.begin("ESP32_STAGE_CTRL")) {
    Serial.println("Bluetooth init failed");
    while (true) delay(1000);
  }

  SlaveSerial.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  delay(1000);

  printlnBoth("Main board ready");
  printlnBoth("Bluetooth name: ESP32_STAGE_CTRL");
  printlnBoth("UART2 to MOSFET slave ready");
  printlnBoth("UART1 to sensor board ready");
  printHelp();
}

void loop() {
  readFromSerial();
  readFromBluetooth();
  readFromSlave();
  serviceSensorUART();
}
