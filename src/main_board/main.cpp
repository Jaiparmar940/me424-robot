#include <Arduino.h>
#include <cstring>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Version 12.2

#ifndef WIFI_SSID
#define WIFI_SSID "surgical_clanker2.4"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "bone_saw"
#endif
#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME "me424-main"
#endif

WiFiServer CommandServer(3333);
WiFiClient commandClient;

WebServer httpServer(80);
WebSocketsServer webSocket(81);
bool webSocketLive = false;

// Never call broadcastTXT from inside webSocketEvent (e.g. during handleCommand) — it drops clients.
// Queue lines here; wsTxFlush() runs after webSocket.loop().
// printHelp() alone is ~70 lines; keep headroom so WebSocket clients see the full list (e.g. "ip" under Other:).
#define WS_TX_QUEUE_DEPTH 160
static String wsTxQueue[WS_TX_QUEUE_DEPTH];
static uint8_t wsTxQueueLen = 0;

static void wsTxEnqueue(const String &line) {
  if (wsTxQueueLen >= WS_TX_QUEUE_DEPTH) {
    Serial.println(F("WS TX queue overflow (line dropped)"));
    return;
  }
  wsTxQueue[wsTxQueueLen++] = line;
}

static void wsTxFlush() {
  if (!webSocketLive || wsTxQueueLen == 0) return;
  for (uint8_t i = 0; i < wsTxQueueLen; i++) {
    webSocket.broadcastTXT(wsTxQueue[i]);
  }
  wsTxQueueLen = 0;
}
bool httpServerLive = false;

bool DEBUG = false;

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
String estopWifiBuffer = "";

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
volatile bool stage1CWLimitHit = false;
volatile bool stage2LimitHit = false;
volatile bool stage3LimitHit = false;
volatile bool stage4LimitHit = false;
// Stage 5: Hall + HW-477 on sensor board GPIO14 — homing/zero only, not a motion limit
volatile bool stage5HallAtZero = false;
String lastSensorLimMsg = "";

// =========================
// Buffers
// =========================
String serialBuffer = "";
String wifiBuffer = "";
String slaveBuffer = "";
String sensorBuffer = "";

// Forward declarations used by queue/emergency helpers
void handleCommand(String cmd);
void readFromSlave();

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    String s;
    s.reserve(length);
    for (size_t i = 0; i < length; i++) s += (char)payload[i];
    s.trim();
    if (s.length()) handleCommand(s);
  } else if (type == WStype_CONNECTED) {
    webSocket.sendTXT(num, "WS: connected to ME424 main");
  }
}

static void httpSendLittleFSFile(const char *path, const char *mime) {
  if (!LittleFS.exists(path)) {
    httpServer.send(404, "text/plain",
                    String("LittleFS missing: ") + path +
                        " — put index.html, app.js, styles.css in data/ then: pio run -e main_board -t uploadfs");
    return;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    httpServer.send(500, "text/plain", "LittleFS open failed");
    return;
  }
  httpServer.streamFile(f, mime);
  f.close();
}

// =========================
// Output helpers
//
// printlnControl() — machine-readable protocol lines (always emitted on Serial).
//   Used for ACK / DONE / ERR / state-report lines that the ROS bridge parses.
//
// printlnDebug() — verbose human-readable noise, gated by the DEBUG flag.
//   Send  "debug on"  /  "debug off"  at runtime to toggle without reflashing.
//
// printlnBoth() — legacy broadcast to Serial + WiFi TCP + WebSocket.
//   Kept for WiFi-facing status messages (WebSocket UI, TCP client).
//   In serial-only mode it behaves identically to printlnControl().
// =========================

void printlnControl(const String &msg) {
  Serial.println(msg);
  // Also mirror to WiFi clients so the web UI stays in sync when WiFi is active.
  if (commandClient && commandClient.connected()) commandClient.println(msg);
  if (webSocketLive) {
    String wscopy = msg;
    webSocket.broadcastTXT(wscopy);
  }
}

void printlnDebug(const String &msg) {
  if (DEBUG) Serial.println("[DBG] " + msg);
}

// printlnBoth is retained for call-sites that genuinely want WiFi broadcast
// (sensor events, slave relay lines, startup banners).  It does NOT go through
// printlnControl so debug noise never leaks into the control stream.
void printBoth(const String &msg) {
  if (DEBUG) Serial.print(msg);
  if (commandClient && commandClient.connected()) commandClient.print(msg);
  if (webSocketLive) {
    wsTxEnqueue(msg);
  }
}

void printlnBoth(const String &msg) {
  if (DEBUG) Serial.println(msg);
  if (commandClient && commandClient.connected()) commandClient.println(msg);
  if (webSocketLive) {
    wsTxEnqueue(msg);
  }
}

// --- Protocol helpers ---
// Always call these (not printlnBoth) for ROS-parseable responses.

void sendACK(const String &tag) {
  printlnControl("ACK " + tag);
}

void sendDONE(const String &tag) {
  printlnControl("DONE " + tag);
}

void sendERR(const String &tag, const String &reason) {
  printlnControl("ERR " + tag + " " + reason);
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
    printlnControl("ESTOP LATCHED");
  } else if (line == "estop clear") {
    estopLatched = false;
    printlnControl("ESTOP CLEARED");
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

  if (commandClient && commandClient.connected()) {
    while (commandClient.available()) {
      char c = (char)commandClient.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (estopWifiBuffer.length() > 0) {
          parseEStopLine(estopWifiBuffer);
          estopWifiBuffer = "";
        }
      } else {
        estopWifiBuffer += c;
      }
    }
  }
}

void printLimitStatus() {
  // S5H: 0 = at Hall zero, 1 = away (unlike S2–S4 where 1 = limit hit).
  printlnControl("LIMITS S1=" + String(stage1CWLimitHit ? "1" : "0") +
              " S2=" + String(stage2LimitHit ? "1" : "0") +
              " S3=" + String(stage3LimitHit ? "1" : "0") +
              " S4=" + String(stage4LimitHit ? "1" : "0") +
              " S5H=" + String(stage5HallAtZero ? "0" : "1"));
}

void parseSensorMessage(String msg) {
  msg.trim();
  msg.toUpperCase();

  if (!msg.startsWith("LIM ")) return;

  const bool prevS1 = stage1CWLimitHit;
  const bool prevS2 = stage2LimitHit;
  const bool prevS3 = stage3LimitHit;
  const bool prevS4 = stage4LimitHit;
  const bool prevS5Hall = stage5HallAtZero;

  // Space-prefixed tokens avoid false matches (e.g. stray "S1=" in other fields).
  // If S1 is missing from the line, clear turntable CW limit (stale S1=1 was blocking s1cw / c6f).
  int s1p = msg.indexOf(" S1=");
  if (s1p >= 0 && s1p + 4 < (int)msg.length()) {
    char c1 = msg.charAt(s1p + 4);
    if (c1 == '1') stage1CWLimitHit = true;
    else if (c1 == '0') stage1CWLimitHit = false;
  } else {
    stage1CWLimitHit = false;
  }

  int s2pos = msg.indexOf(" S2=");
  int s3pos = msg.indexOf(" S3=");
  int s4pos = msg.indexOf(" S4=");
  int s5hpos = msg.indexOf(" S5H=");

  if (s2pos >= 0 && s2pos + 4 < (int)msg.length()) {
    char c2 = msg.charAt(s2pos + 4);
    if (c2 == '1') stage2LimitHit = true;
    else if (c2 == '0') stage2LimitHit = false;
  }
  if (s3pos >= 0 && s3pos + 4 < (int)msg.length()) {
    char c3 = msg.charAt(s3pos + 4);
    if (c3 == '1') stage3LimitHit = true;
    else if (c3 == '0') stage3LimitHit = false;
  }
  if (s4pos >= 0 && s4pos + 4 < (int)msg.length()) {
    char c4 = msg.charAt(s4pos + 4);
    if (c4 == '1') stage4LimitHit = true;
    else if (c4 == '0') stage4LimitHit = false;
  }
  if (s5hpos >= 0 && s5hpos + 5 < (int)msg.length()) {
    bool s5 = (msg.charAt(s5hpos + 5) == '0');
    stage5HallAtZero = s5;
    if (s5 && !prevS5Hall) {
      printlnDebug("S5 HALL: at zero (sensor)");
    } else if (!s5 && prevS5Hall) {
      printlnDebug("S5 HALL: left zero zone");
    }
  }

  bool stateChanged = (prevS1 != stage1CWLimitHit) ||
                      (prevS2 != stage2LimitHit) ||
                      (prevS3 != stage3LimitHit) ||
                      (prevS4 != stage4LimitHit) ||
                      (prevS5Hall != stage5HallAtZero);
  if (stateChanged || msg != lastSensorLimMsg) {
    // printlnBoth("[SENSOR] " + msg);
    printlnDebug("[SENSOR] " + msg);
    lastSensorLimMsg = msg;
  }
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
// Stage 1: turntable CW limit on sensor D13 — blocks s1cw only (CCW still allowed).
// Stage 5: no limit switch blocking (Hall S5H is for homing only, not hard stop)
// =========================
bool shouldStopMotor(int motorIndex, bool forward) {
  if (motorIndex == STAGE1) {
    bool movingCW = forward;
    return stage1CWLimitHit && movingCW;
  }

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
    printlnDebug("ESTOP: motion blocked");
    return false;
  }
  serviceSensorUART();
  int movedSteps = 0;

  if (shouldStopMotor(motorIndex, forward)) {
    printlnDebug("ABORT: limit active for controller " + String(motorIndex + 1) +
                " in requested direction");
    return false;
  }

  setDirection(motorIndex, forward);

  for (int i = 0; i < steps; i++) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnDebug("ESTOP: controller " + String(motorIndex + 1) + " stopped");
      currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
      return false;
    }
    serviceSensorUART();

    if (shouldStopMotor(motorIndex, forward)) {
      printlnDebug("STOP: limit hit on controller " + String(motorIndex + 1));
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
    printlnDebug("ESTOP: motion blocked");
    return false;
  }
  serviceSensorUART();
  int movedSteps = 0;

  bool movingDown = !forward;
  if (stage2LimitHit && movingDown) {
    printlnDebug("ABORT: Stage 2 limit already active for downward motion");
    return false;
  }

  setDirection(STAGE2_RIGHT, forward);
  setDirection(STAGE2_LEFT, forward);

  for (int i = 0; i < steps; i++) {
    pollEmergencyInputs();
    if (estopLatched) {
      printlnDebug("ESTOP: Stage 2 stopped");
      long d = forward ? movedSteps : -movedSteps;
      currentPos[STAGE2_RIGHT] += d;
      currentPos[STAGE2_LEFT]  += d;
      return false;
    }
    serviceSensorUART();

    if (stage2LimitHit && movingDown) {
      printlnDebug("STOP: Stage 2 limit hit");
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
    sendERR("c" + String(controllerNum), "invalid controller");
    return;
  }

  bool ok = false;
  if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) {
    ok = stepStage2(forward, steps);
  } else {
    ok = stepMotor(motorIndex, forward, steps);
  }

  printlnDebug("Controller " + String(controllerNum) +
               (forward ? " forward " : " reverse ") + String(steps) + " steps " +
               (ok ? "done" : "aborted"));
  // DONE/ERR is sent by the handleCommand caller after this returns
  // (raw controller path sends them inline in handleCommand)
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
    printlnDebug("ESTOP: motion blocked");
    return false;
  }
  // Pre-flight limit check
  for (int i = 0; i < count; i++) {
    if (shouldStopMotor(moves[i].motorIndex, moves[i].forward)) {
      printlnDebug("PAR ABORT: limit active for controller " +
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
      printlnDebug("ESTOP: parallel motion stopped");
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
        printlnDebug("PAR STOP: limit hit on controller " +
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
    printlnDebug("ESTOP: motion blocked");
    return false;
  }
  if (maxSps < 1) {
    printlnDebug("SYNCABS: max_sps must be >= 1");
    return false;
  }

  enforceStage2PairTarget(targets);

  long delta[NUM_DRIVERS];
  long absDelta[NUM_DRIVERS];
  bool forward[NUM_DRIVERS];
  bool axisActive[NUM_DRIVERS];
  long accum[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
  long stepped[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
  long totalSteps = 0;
  bool anyAxisBlocked = false;

  for (int i = 0; i < NUM_DRIVERS; i++) {
    delta[i] = targets[i] - currentPos[i];
    absDelta[i] = labs(delta[i]);
    forward[i] = (delta[i] >= 0);
    axisActive[i] = (absDelta[i] > 0);
    if (absDelta[i] > totalSteps) totalSteps = absDelta[i];
  }

  if (totalSteps == 0) {
    printlnDebug("SYNCABS: already at target");
    return true;
  }

  for (int i = 0; i < NUM_DRIVERS; i++) {
    if (!axisActive[i]) continue;
    if (shouldStopMotor(i, forward[i])) {
      printlnDebug("SYNCABS: controller " + String(i + 1) +
                  " blocked by active limit; skipping this axis");
      axisActive[i] = false;
      anyAxisBlocked = true;
      continue;
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
      printlnDebug("ESTOP: sync motion stopped");
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
      if (!axisActive[i]) continue;
      if (shouldStopMotor(i, forward[i])) {
        printlnDebug("SYNCABS: limit hit on controller " + String(i + 1) +
                    "; stopping only this axis");
        axisActive[i] = false;
        anyAxisBlocked = true;
        continue;
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
      if (pulseThisTick[i]) {
        digitalWrite(STEP_PINS[i], LOW);
        stepped[i]++;
      }
    }
    delayMicroseconds((int)dUs);
  }

  for (int i = 0; i < NUM_DRIVERS; i++) {
    currentPos[i] += forward[i] ? stepped[i] : -stepped[i];
  }
  if (anyAxisBlocked) {
    printlnDebug("SYNCABS: completed with one or more axes blocked by limits");
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
    printlnDebug("QRUN: queue empty");
    return false;
  }

  if (estopLatched) {
    printlnDebug("QRUN: blocked by ESTOP");
    return false;
  }

  queueStopRequested = false;
  printlnDebug("QRUN: start");

  for (int i = 0; i < runQueueCount; i++) {
    pollEmergencyInputs();
    if (estopLatched || queueStopRequested) {
      printlnDebug("QRUN: stopped");
      return false;
    }

    QueueItem &it = runQueueItems[i];
    if (it.type == Q_POSE) {
      bool ok = runSyncAbs(it.targets, it.maxSps, it.rampSteps);
      if (!ok) {
        printlnDebug("QRUN: pose aborted at item " + String(i));
        return false;
      }
    } else if (it.type == Q_DELAY) {
      if (!queueDelayWait(it.delayMs)) {
        printlnDebug("QRUN: delay interrupted at item " + String(i));
        return false;
      }
    } else if (it.type == Q_CMD) {
      handleCommand(it.cmd);
    }
  }

  printlnDebug("QRUN: done");
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
      printlnDebug("SEQ[" + String(idx++) + "]: " + sub);
      handleCommand(sub);
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  printlnDebug("SEQ: done");
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
        printlnDebug("PAR: unrecognised motor command: " + sub);
        return;
      }
      totalMoves += added;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  if (totalMoves == 0) {
    printlnDebug("PAR: no valid motor commands");
    return;
  }

  bool ok = stepMultiple(moves, totalMoves);
  printlnDebug(ok ? "PAR: done" : "PAR: aborted");
}

void applyPositionState(long p[NUM_DRIVERS]) {
  enforceStage2PairTarget(p);
  for (int i = 0; i < NUM_DRIVERS; i++) currentPos[i] = p[i];
}

static const int HOME_CHUNK_STEPS = 40;
static const long HOME_MAX_SEEK_STEPS = 250000L;
// Bounce homing (`home sN bounce`): fast approach, back off, slow final creep, then zero.
static const int HOME_BOUNCE_BACKOFF_STEPS = 260;
static const int HOME_BOUNCE_SLOW_CHUNK = 8;

static int homeBounceSlowPulseUs(int saved) {
  return min(max(saved * 5, saved + 800), 14000);
}

// Turntable (S1) homing uses gentler pulse timing than other axes.
static int homeS1StraightHomingPulseUs(int saved) {
  return min(max(saved * 2, saved + 900), 12000);
}
static int homeS1BounceFastPulseUs(int saved) {
  return max(400, (saved * 3) / 4);
}
static int homeS1BounceSlowPulseUs(int saved) {
  return min(max(saved * 6, saved + 1500), 20000);
}
// Stage 5 Hall seek: slower pulse + smaller chunks (does not change global `speed` / manual jog).
static const int HOME_S5_CHUNK_STEPS = 24;
static const int HOME_S5_PULSE_DELAY_US = 2400;

// Seek toward limit only (no zero). Caller sets `pulseDelayUs`; restored by bounce callers.
static bool homeStage1SeekCWNoZero(int chunkSteps) {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage1CWLimitHit) return true;
    long chunk = chunkSteps;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = (int)(HOME_MAX_SEEK_STEPS - moved);
    if (chunk <= 0) break;
    if (!stepMotor(STAGE1, true, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage1CWLimitHit) return true;
      return false;
    }
    moved += chunk;
  }
  return stage1CWLimitHit;
}

static bool homeStage2SeekDownNoZero(int chunkSteps) {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage2LimitHit) return true;
    long chunk = chunkSteps;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = (int)(HOME_MAX_SEEK_STEPS - moved);
    if (chunk <= 0) break;
    if (!stepStage2(false, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage2LimitHit) return true;
      return false;
    }
    moved += chunk;
  }
  return stage2LimitHit;
}

static bool homeStage3SeekDownNoZero(int chunkSteps) {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage3LimitHit) return true;
    long chunk = chunkSteps;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = (int)(HOME_MAX_SEEK_STEPS - moved);
    if (chunk <= 0) break;
    if (!stepMotor(STAGE3, true, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage3LimitHit) return true;
      return false;
    }
    moved += chunk;
  }
  return stage3LimitHit;
}

static bool homeStage4SeekDownNoZero(int chunkSteps) {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage4LimitHit) return true;
    long chunk = chunkSteps;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = (int)(HOME_MAX_SEEK_STEPS - moved);
    if (chunk <= 0) break;
    if (!stepMotor(STAGE4, true, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage4LimitHit) return true;
      return false;
    }
    moved += chunk;
  }
  return stage4LimitHit;
}

bool homeStage1ToLimit() {
  if (estopLatched) return false;
  const int savedPulse = pulseDelayUs;
  pulseDelayUs = homeS1StraightHomingPulseUs(savedPulse);
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) {
      pulseDelayUs = savedPulse;
      return false;
    }
    if (stage1CWLimitHit) break;
    long chunk = HOME_CHUNK_STEPS;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
    if (chunk <= 0) break;
    if (!stepMotor(STAGE1, true, (int)chunk)) {
      serviceSensorUART();
      if (stage1CWLimitHit) break;
      pulseDelayUs = savedPulse;
      return false;
    }
    moved += chunk;
  }
  pulseDelayUs = savedPulse;
  if (!stage1CWLimitHit) {
    printlnDebug("HOME S1: CW limit not reached (check sensor LIM S1=1, wiring, debounce)");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE1] = 0;
  applyPositionState(p);
  printlnDebug("HOME S1: zero at CW limit (same + direction as c6f / s1cw)");
  return true;
}

bool homeStage2ToLimit() {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage2LimitHit) break;
    long chunk = HOME_CHUNK_STEPS;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
    if (chunk <= 0) break;
    if (!stepStage2(false, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage2LimitHit) break;
      return false;
    }
    moved += chunk;
  }
  if (!stage2LimitHit) {
    printlnDebug("HOME S2: limit not reached");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE2_RIGHT] = 0;
  p[STAGE2_LEFT] = 0;
  applyPositionState(p);
  printlnDebug("HOME S2: zero at limit");
  return true;
}

bool homeStage3ToLimit() {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage3LimitHit) break;
    long chunk = HOME_CHUNK_STEPS;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
    if (chunk <= 0) break;
    if (!stepMotor(STAGE3, true, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage3LimitHit) break;
      return false;
    }
    moved += chunk;
  }
  if (!stage3LimitHit) {
    printlnDebug("HOME S3: limit not reached");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE3] = 0;
  applyPositionState(p);
  printlnDebug("HOME S3: zero at limit");
  return true;
}

bool homeStage4ToLimit() {
  long moved = 0;
  while (moved < HOME_MAX_SEEK_STEPS) {
    pollEmergencyInputs();
    serviceSensorUART();
    if (estopLatched) return false;
    if (stage4LimitHit) break;
    long chunk = HOME_CHUNK_STEPS;
    if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
    if (chunk <= 0) break;
    if (!stepMotor(STAGE4, true, (int)chunk)) {
      if (estopLatched) return false;
      serviceSensorUART();
      if (stage4LimitHit) break;
      return false;
    }
    moved += chunk;
  }
  if (!stage4LimitHit) {
    printlnDebug("HOME S4: limit not reached");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE4] = 0;
  applyPositionState(p);
  printlnDebug("HOME S4: zero at limit");
  return true;
}

bool homeStage1ToLimitBounce() {
  if (estopLatched) return false;
  const int saved = pulseDelayUs;
  pulseDelayUs = homeS1BounceFastPulseUs(saved);
  if (!homeStage1SeekCWNoZero(HOME_CHUNK_STEPS)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (estopLatched) return false;
  if (!stepMotor(STAGE1, false, HOME_BOUNCE_BACKOFF_STEPS)) return false;
  serviceSensorUART();
  pulseDelayUs = homeS1BounceSlowPulseUs(saved);
  if (!homeStage1SeekCWNoZero(HOME_BOUNCE_SLOW_CHUNK)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (!stage1CWLimitHit) {
    printlnDebug("HOME S1 bounce: CW limit not confirmed");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE1] = 0;
  applyPositionState(p);
  printlnDebug("HOME S1: zero at CW limit (bounce)");
  return true;
}

bool homeStage2ToLimitBounce() {
  if (estopLatched) return false;
  const int saved = pulseDelayUs;
  pulseDelayUs = max(100, saved / 2);
  if (!homeStage2SeekDownNoZero(HOME_CHUNK_STEPS)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (estopLatched) return false;
  if (!stepStage2(true, HOME_BOUNCE_BACKOFF_STEPS)) return false;
  serviceSensorUART();
  pulseDelayUs = homeBounceSlowPulseUs(saved);
  if (!homeStage2SeekDownNoZero(HOME_BOUNCE_SLOW_CHUNK)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (!stage2LimitHit) {
    printlnDebug("HOME S2 bounce: limit not confirmed");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE2_RIGHT] = 0;
  p[STAGE2_LEFT] = 0;
  applyPositionState(p);
  printlnDebug("HOME S2: zero at limit (bounce)");
  return true;
}

bool homeStage3ToLimitBounce() {
  if (estopLatched) return false;
  const int saved = pulseDelayUs;
  pulseDelayUs = max(100, saved / 2);
  if (!homeStage3SeekDownNoZero(HOME_CHUNK_STEPS)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (estopLatched) return false;
  if (!stepMotor(STAGE3, false, HOME_BOUNCE_BACKOFF_STEPS)) return false;
  serviceSensorUART();
  pulseDelayUs = homeBounceSlowPulseUs(saved);
  if (!homeStage3SeekDownNoZero(HOME_BOUNCE_SLOW_CHUNK)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (!stage3LimitHit) {
    printlnDebug("HOME S3 bounce: limit not confirmed");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE3] = 0;
  applyPositionState(p);
  printlnDebug("HOME S3: zero at limit (bounce)");
  return true;
}

bool homeStage4ToLimitBounce() {
  if (estopLatched) return false;
  const int saved = pulseDelayUs;
  pulseDelayUs = max(100, saved / 2);
  if (!homeStage4SeekDownNoZero(HOME_CHUNK_STEPS)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (estopLatched) return false;
  if (!stepMotor(STAGE4, false, HOME_BOUNCE_BACKOFF_STEPS)) return false;
  serviceSensorUART();
  pulseDelayUs = homeBounceSlowPulseUs(saved);
  if (!homeStage4SeekDownNoZero(HOME_BOUNCE_SLOW_CHUNK)) {
    pulseDelayUs = saved;
    return false;
  }
  pulseDelayUs = saved;
  if (!stage4LimitHit) {
    printlnDebug("HOME S4 bounce: limit not confirmed");
    return false;
  }
  long p[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
  p[STAGE4] = 0;
  applyPositionState(p);
  printlnDebug("HOME S4: zero at limit (bounce)");
  return true;
}

bool homeStage5ToHall() {
  serviceSensorUART();
  if (estopLatched) return false;

  if (stage5HallAtZero) {
    long p[NUM_DRIVERS];
    for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
    p[STAGE5] = 0;
    applyPositionState(p);
    printlnDebug("HOME S5: already at hall, C5=0");
    return true;
  }

  const int savedPulse = pulseDelayUs;
  pulseDelayUs = HOME_S5_PULSE_DELAY_US;

  const bool dirs[2] = { true, false };
  for (int d = 0; d < 2; d++) {
    long moved = 0;
    while (moved < HOME_MAX_SEEK_STEPS) {
      pollEmergencyInputs();
      serviceSensorUART();
      if (estopLatched) {
        pulseDelayUs = savedPulse;
        return false;
      }
      if (stage5HallAtZero) {
        long p[NUM_DRIVERS];
        for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
        p[STAGE5] = 0;
        applyPositionState(p);
        printlnDebug("HOME S5: zero at hall");
        pulseDelayUs = savedPulse;
        return true;
      }
      long chunk = HOME_S5_CHUNK_STEPS;
      if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
      if (chunk <= 0) break;
      if (!stepMotor(STAGE5, dirs[d], (int)chunk)) {
        pulseDelayUs = savedPulse;
        return false;
      }
      moved += chunk;
    }
  }
  pulseDelayUs = savedPulse;
  printlnDebug("HOME S5: hall not found (check S5H / HW-477 / magnet; try home s5 from other side)");
  return false;
}

// --- App parity: VERTICAL_POSE / LIFT_ROUTINE in app.js (keep numbers in sync). ---
static const long kAppVerticalPose[NUM_DRIVERS] =
    {7833L, -5827L, -3972L, 7833L, 0L, -2967L};
static const long kLiftRoutinePreS2 = 4500L;
static const long kLiftRoutinePostC2 = -6000L;
static const long kLiftRoutinePostC3 = -5000L;
static const long kLiftRoutineBounceLift = 4500L;
static const long kMacroDefaultMaxSps = 1200L;
static const int kMacroDefaultRamp = 150;

static void parseMacroMaxSpsRamp(const String &cmd, const char *keyword,
                                 long &maxSps, int &rampSteps) {
  maxSps = kMacroDefaultMaxSps;
  rampSteps = kMacroDefaultRamp;
  const int kwLen = (int)strlen(keyword);
  if ((int)cmd.length() <= kwLen) return;
  if (!cmd.startsWith(keyword)) return;
  String tail = cmd.substring(kwLen);
  tail.trim();
  if (tail.length() == 0) return;
  long m = 0;
  int r = 0;
  int got = sscanf(tail.c_str(), "%ld %d", &m, &r);
  if (got >= 1 && m >= 10) maxSps = m;
  if (got >= 2 && r >= 1) rampSteps = r;
}

static bool runVerticalMacro(long maxSps, int rampSteps) {
  long t[NUM_DRIVERS];
  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = kAppVerticalPose[i];
  return runSyncAbs(t, maxSps, rampSteps);
}

static bool runLiftAutohomeMacro(long maxSps, int rampSteps) {
  if (estopLatched) return false;
  long t[NUM_DRIVERS];

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = currentPos[i];
  t[STAGE2_RIGHT] = kLiftRoutinePreS2;
  t[STAGE2_LEFT] = kLiftRoutinePreS2;
  if (!runSyncAbs(t, maxSps, rampSteps)) return false;

  if (!homeStage1ToLimit()) return false;
  if (!homeStage3ToLimit()) return false;
  if (!homeStage4ToLimit()) return false;

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = currentPos[i];
  t[STAGE3] = kLiftRoutinePostC2;
  t[STAGE4] = kLiftRoutinePostC3;
  if (!runSyncAbs(t, maxSps, rampSteps)) return false;

  if (!homeStage2ToLimit()) return false;

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = kAppVerticalPose[i];
  if (!runSyncAbs(t, maxSps, rampSteps)) return false;

  if (!homeStage2ToLimitBounce()) return false;

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = currentPos[i];
  t[STAGE2_RIGHT] = kLiftRoutineBounceLift;
  t[STAGE2_LEFT] = kLiftRoutineBounceLift;
  if (!runSyncAbs(t, maxSps, rampSteps)) return false;

  if (!homeStage3ToLimitBounce()) return false;
  if (!homeStage4ToLimitBounce()) return false;
  if (!homeStage1ToLimitBounce()) return false;

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = kAppVerticalPose[i];
  if (!runSyncAbs(t, maxSps, rampSteps)) return false;

  for (int i = 0; i < NUM_DRIVERS; i++) t[i] = 0;
  applyPositionState(t);
  return true;
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
  printlnBoth("  vertical [maxSps] [ramp]  -> same as app Go to vertical (default 1200 150)");
  printlnBoth("  autohome [maxSps] [ramp]  -> same as app Lift autohome (default 1200 150)");
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
  printlnBoth("  ip            -> print this board WiFi IP and hostname");
  printlnBoth("  speed 800     -> set pulse delay in microseconds");
  printlnBoth("  where         -> print current tracked positions");
  printlnBoth("  setpos p1 p2 p3 p4 p5 p6 -> overwrite tracked positions");
  printlnBoth("  home s1       -> jog C6 + (c6f/s1cw) to S1 limit, then C6=0");
  printlnBoth("  home s2..s4   -> drive to limit switches, then zero those axes");
  printlnBoth("  home sN bounce -> s1-s4 only: fast seek, back off, slow creep, then zero");
  printlnBoth("  home s5       -> find Hall zero (S5H), then zero C5");
  printlnBoth("  estop         -> latch emergency stop (blocks all motion)");
  printlnBoth("  estop clear   -> clear e-stop latch");
  printlnBoth("  estop status  -> print e-stop state");
  printlnBoth("  limits        -> request fresh LIM (+S5H hall) from sensor, print state");
  printlnBoth("");

  printlnBoth("MOSFET / attachment commands:");
  printlnBoth("  mag on");
  printlnBoth("  mag off");
  printlnBoth("  vac on");
  printlnBoth("  vac off");
  printlnBoth("  saw on");
  printlnBoth("  saw off");
  printlnBoth("  saw speed <0-100>  -> BLDC PWM on MOSFET board GPIO 22 (forwarded to slave)");
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
static void printWifiIpLine() {
  if (WiFi.status() == WL_CONNECTED) {
    printlnBoth(String("IP ") + WiFi.localIP().toString() + " hostname " + WIFI_HOSTNAME);
  } else {
    printlnBoth("IP (WiFi not connected)");
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  // Emit ACK immediately so the ROS bridge knows the command was received.
  sendACK(cmd);

  // ---- runtime debug toggle ----
  if (cmd == "debug on")  { DEBUG = true;  sendDONE("debug on");  return; }
  if (cmd == "debug off") { DEBUG = false; sendDONE("debug off"); return; }

  if (cmd == "help") {
    printHelp();
    sendDONE("help");
    return;
  }

  if (cmd == "ip") {
    printWifiIpLine();
    return;
  }

  if (cmd == "limits") {
    SensorSerial.println("STATUS");
    unsigned long t0 = millis();
    while (millis() - t0 < 80) {
      serviceSensorUART();
      pollEmergencyInputs();
    }
    printLimitStatus();
    sendDONE("limits");
    return;
  }

  if (cmd == "estop") {
    estopLatched = true;
    sendToSlave("ALL OFF");
    printlnControl("ESTOP LATCHED");
    sendDONE("estop");
    return;
  }

  if (cmd == "estop clear") {
    estopLatched = false;
    printlnControl("ESTOP CLEARED");
    sendDONE("estop clear");
    return;
  }

  if (cmd == "estop status") {
    printlnControl(String("ESTOP=") + (estopLatched ? "1" : "0"));
    sendDONE("estop status");
    return;
  }

  if (cmd.startsWith("home ")) {
    String h = cmd.substring(5);
    h.trim();
    bool bounce = false;
    if (h.endsWith(" bounce")) {
      bounce = true;
      h = h.substring(0, h.length() - 7);
      h.trim();
    }
    bool ok = false;
    if (h == "s1") {
      ok = bounce ? homeStage1ToLimitBounce() : homeStage1ToLimit();
    } else if (h == "s2") {
      ok = bounce ? homeStage2ToLimitBounce() : homeStage2ToLimit();
    } else if (h == "s3") {
      ok = bounce ? homeStage3ToLimitBounce() : homeStage3ToLimit();
    } else if (h == "s4") {
      ok = bounce ? homeStage4ToLimitBounce() : homeStage4ToLimit();
    } else if (h == "s5") {
      if (bounce) {
        sendERR(cmd, "s5 bounce not supported (use home s5)");
        return;
      }
      ok = homeStage5ToHall();
    } else {
      sendERR(cmd, "unknown stage");
      return;
    }
    ok ? sendDONE(cmd) : sendERR(cmd, "aborted");
    return;
  }

  if (cmd == "qclear") {
    runQueueCount = 0;
    queueStopRequested = false;
    sendDONE("qclear");
    return;
  }

  if (cmd == "qstop") {
    queueStopRequested = true;
    sendDONE("qstop");
    return;
  }

  if (cmd == "qstatus") {
    printlnControl("Q count=" + String(runQueueCount) +
                " stop=" + String(queueStopRequested ? "1" : "0"));
    sendDONE("qstatus");
    return;
  }

  if (cmd == "qlist") {
    printlnControl("Q items=" + String(runQueueCount));
    for (int i = 0; i < runQueueCount; i++) {
      QueueItem &it = runQueueItems[i];
      if (it.type == Q_POSE) {
        printlnControl("Q[" + String(i) + "] POSE " +
                    String(it.targets[0]) + " " + String(it.targets[1]) + " " +
                    String(it.targets[2]) + " " + String(it.targets[3]) + " " +
                    String(it.targets[4]) + " " + String(it.targets[5]) +
                    " maxSps=" + String(it.maxSps) +
                    " ramp=" + String(it.rampSteps));
      } else if (it.type == Q_DELAY) {
        printlnControl("Q[" + String(i) + "] DELAY " + String(it.delayMs) + "ms");
      } else {
        printlnControl("Q[" + String(i) + "] CMD " + it.cmd);
      }
    }
    sendDONE("qlist");
    return;
  }

  if (cmd == "qrun") {
    bool ok = runQueueExecution();
    ok ? sendDONE("qrun") : sendERR("qrun", "aborted");
    return;
  }

  if (cmd.startsWith("qadd pose ")) {
    long t[NUM_DRIVERS];
    long maxSps;
    int rampSteps;
    if (sscanf(cmd.c_str(),
               "qadd pose %ld %ld %ld %ld %ld %ld %ld %d",
               &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &maxSps, &rampSteps) == 8) {
      queueAddPose(t, maxSps, rampSteps) ? sendDONE("qadd pose") : sendERR("qadd pose", "full");
    } else {
      sendERR("qadd pose", "bad args");
    }
    return;
  }

  if (cmd.startsWith("qadd delay ")) {
    unsigned long ms = (unsigned long)cmd.substring(11).toInt();
    if (ms > 0 || cmd.substring(11) == "0") {
      queueAddDelay(ms) ? sendDONE("qadd delay") : sendERR("qadd delay", "full");
    } else {
      sendERR("qadd delay", "bad args");
    }
    return;
  }

  if (cmd.startsWith("qadd cmd ")) {
    String qcmd = cmd.substring(9);
    qcmd.trim();
    if (qcmd.length() == 0) { sendERR("qadd cmd", "empty"); return; }
    queueAddCmd(qcmd) ? sendDONE("qadd cmd") : sendERR("qadd cmd", "full");
    return;
  }

  if (cmd == "where") {
    currentPos[STAGE2_LEFT] = currentPos[STAGE2_RIGHT];
    printlnControl("POS C1=" + String(currentPos[0]) +
                " C2=" + String(currentPos[1]) +
                " C3=" + String(currentPos[2]) +
                " C4=" + String(currentPos[3]) +
                " C5=" + String(currentPos[4]) +
                " C6=" + String(currentPos[5]));
    sendDONE("where");
    return;
  }

  if (cmd.startsWith("setpos ")) {
    long p[NUM_DRIVERS];
    if (sscanf(cmd.c_str(), "setpos %ld %ld %ld %ld %ld %ld",
               &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]) == 6) {
      p[STAGE2_LEFT] = p[STAGE2_RIGHT];
      for (int i = 0; i < NUM_DRIVERS; i++) currentPos[i] = p[i];
      sendDONE("setpos");
    } else {
      sendERR("setpos", "bad args");
    }
    return;
  }

  if (cmd.startsWith("syncabs ")) {
    long targets[NUM_DRIVERS];
    long maxSps = 0;
    int rampSteps = 0;
    if (parseSyncAbsArgs(cmd, targets, maxSps, rampSteps) == 8) {
      bool ok = runSyncAbs(targets, maxSps, rampSteps);
      ok ? sendDONE("syncabs") : sendERR("syncabs", "aborted");
    } else {
      sendERR("syncabs", "bad args");
    }
    return;
  }

  if (cmd == "vertical" || cmd.startsWith("vertical ")) {
    long maxSps;
    int rampSteps;
    parseMacroMaxSpsRamp(cmd, "vertical", maxSps, rampSteps);
    bool ok = runVerticalMacro(maxSps, rampSteps);
    ok ? sendDONE("vertical") : sendERR("vertical", "aborted");
    return;
  }

  if (cmd == "autohome" || cmd.startsWith("autohome ")) {
    long maxSps;
    int rampSteps;
    parseMacroMaxSpsRamp(cmd, "autohome", maxSps, rampSteps);
    bool ok = runLiftAutohomeMacro(maxSps, rampSteps);
    ok ? sendDONE("autohome") : sendERR("autohome", "aborted");
    return;
  }

  if (cmd.startsWith("seq ")) {
    runSequential(cmd.substring(4));
    sendDONE("seq");
    return;
  }

  if (cmd.startsWith("par ")) {
    runParallel(cmd.substring(4));
    sendDONE("par");
    return;
  }

  if (cmd.startsWith("speed ")) {
    int newDelay = cmd.substring(6).toInt();
    if (newDelay >= 100) {
      pulseDelayUs = newDelay;
      printlnDebug("Pulse delay set to " + String(pulseDelayUs) + " us");
      sendDONE("speed");
    } else {
      sendERR("speed", "value must be >= 100");
    }
    return;
  }

  // Stage 1 (turntable)
  if (cmd.startsWith("s1cw ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE1, true, steps);
      ok ? sendDONE("s1cw") : sendERR("s1cw", "aborted");
    } else { sendERR("s1cw", "bad step count"); }
    return;
  }

  if (cmd.startsWith("s1ccw ")) {
    int steps = cmd.substring(6).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE1, false, steps);
      ok ? sendDONE("s1ccw") : sendERR("s1ccw", "aborted");
    } else { sendERR("s1ccw", "bad step count"); }
    return;
  }

  // Stage 2
  if (cmd.startsWith("s2up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepStage2(true, steps);
      ok ? sendDONE("s2up") : sendERR("s2up", "aborted");
    } else { sendERR("s2up", "bad step count"); }
    return;
  }

  if (cmd.startsWith("s2down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepStage2(false, steps);
      ok ? sendDONE("s2down") : sendERR("s2down", "aborted");
    } else { sendERR("s2down", "bad step count"); }
    return;
  }

  // Stage 3
  if (cmd.startsWith("s3down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, true, steps);
      ok ? sendDONE("s3down") : sendERR("s3down", "aborted");
    } else { sendERR("s3down", "bad step count"); }
    return;
  }

  if (cmd.startsWith("s3up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE3, false, steps);
      ok ? sendDONE("s3up") : sendERR("s3up", "aborted");
    } else { sendERR("s3up", "bad step count"); }
    return;
  }

  // Stage 4
  if (cmd.startsWith("s4up ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE4, false, steps);
      ok ? sendDONE("s4up") : sendERR("s4up", "aborted");
    } else { sendERR("s4up", "bad step count"); }
    return;
  }

  if (cmd.startsWith("s4down ")) {
    int steps = cmd.substring(7).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE4, true, steps);
      ok ? sendDONE("s4down") : sendERR("s4down", "aborted");
    } else { sendERR("s4down", "bad step count"); }
    return;
  }

  // Stage 5
  if (cmd.startsWith("s5ccw ")) {
    int steps = cmd.substring(6).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE5, true, steps);
      ok ? sendDONE("s5ccw") : sendERR("s5ccw", "aborted");
    } else { sendERR("s5ccw", "bad step count"); }
    return;
  }

  if (cmd.startsWith("s5cw ")) {
    int steps = cmd.substring(5).toInt();
    if (steps > 0) {
      bool ok = stepMotor(STAGE5, false, steps);
      ok ? sendDONE("s5cw") : sendERR("s5cw", "aborted");
    } else { sendERR("s5cw", "bad step count"); }
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
        // runControllerCommand emits its own debug; DONE/ERR already sent inside it
        // (see updated runControllerCommand below)
      } else {
        sendERR(cmd, "bad step count");
      }
      return;
    }
  }

  // MOSFET / slave commands
  if (cmd == "mag on")  { sendToSlave("MAG ON");  sendDONE("mag on");  return; }
  if (cmd == "mag off") { sendToSlave("MAG OFF"); sendDONE("mag off"); return; }
  if (cmd == "vac on")  { sendToSlave("VAC ON");  sendDONE("vac on");  return; }
  if (cmd == "vac off") { sendToSlave("VAC OFF"); sendDONE("vac off"); return; }
  if (cmd == "saw on")  { sendToSlave("SAW ON");  sendDONE("saw on");  return; }
  if (cmd == "saw off") { sendToSlave("SAW OFF"); sendDONE("saw off"); return; }
  if (cmd == "alloff")  { sendToSlave("ALL OFF"); sendDONE("alloff");  return; }
  if (cmd == "mstatus") { sendToSlave("STATUS");  sendDONE("mstatus"); return; }

  if (cmd.startsWith("saw speed ")) {
    int v = cmd.substring(10).toInt();
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    sendToSlave("SAW SPEED " + String(v));
    sendDONE("saw speed");
    return;
  }

  sendERR(cmd, "unknown command");
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

void readFromWiFi() {
  if (!commandClient || !commandClient.connected()) {
    WiFiClient incoming = CommandServer.available();
    if (incoming) {
      if (commandClient && commandClient.connected()) commandClient.stop();
      commandClient = incoming;
      commandClient.println("Connected to ME424 main board WiFi command socket.");
    }
  }

  if (commandClient && commandClient.connected()) {
    while (commandClient.available()) {
      char c = (char)commandClient.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (wifiBuffer.length() > 0) {
          handleCommand(wifiBuffer);
          wifiBuffer = "";
        }
      } else {
        wifiBuffer += c;
      }
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

  if (String(WIFI_SSID).length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi connecting");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.setHostname(WIFI_HOSTNAME);
      ArduinoOTA.begin();
      CommandServer.begin();
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
      Serial.println(String("WiFi command port: ") + 3333);

      if (LittleFS.begin(true)) {
        // Explicit routes — ESP32 serveStatic often does not map "/" -> index.html.
        httpServer.on("/", HTTP_GET, []() { httpSendLittleFSFile("/index.html", "text/html"); });
        httpServer.on("/index.html", HTTP_GET, []() { httpSendLittleFSFile("/index.html", "text/html"); });
        httpServer.on("/app.js", HTTP_GET, []() { httpSendLittleFSFile("/app.js", "application/javascript"); });
        httpServer.on("/styles.css", HTTP_GET, []() { httpSendLittleFSFile("/styles.css", "text/css"); });
        httpServer.onNotFound([]() {
          httpServer.send(404, "text/plain", "Not found (expected /, /app.js, /styles.css)");
        });
        httpServer.begin();
        httpServerLive = true;
        Serial.println("Web UI: http://" + WiFi.localIP().toString() + "/");
        Serial.println(String("LittleFS index.html: ") + (LittleFS.exists("/index.html") ? "yes" : "NO"));
        Serial.println(String("LittleFS app.js: ") + (LittleFS.exists("/app.js") ? "yes" : "NO"));
        Serial.println(String("LittleFS styles.css: ") + (LittleFS.exists("/styles.css") ? "yes" : "NO"));
      } else {
        Serial.println("LittleFS mount failed (run: pio run -e main_board -t uploadfs)");
      }

      webSocket.onEvent(webSocketEvent);
      webSocket.begin();
      webSocketLive = true;
      Serial.println(String("WebSocket commands: ws://") + WiFi.localIP().toString() + ":81/");
    } else {
      Serial.println("WiFi connect failed, continuing without WiFi/OTA");
    }
  } else {
    Serial.println("WiFi disabled (set WIFI_SSID/WIFI_PASSWORD build flags)");
  }

  SlaveSerial.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);
  SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  delay(1000);

  printlnBoth("Main board ready");
  printlnBoth("USB serial + WiFi (TCP 3333 / WebSocket 81) + HTTP UI on /");
  printlnBoth("UART2 to MOSFET slave ready");
  printlnBoth("UART1 to sensor board ready");
  if (WiFi.status() == WL_CONNECTED) {
    printlnBoth("OTA host: " + String(WIFI_HOSTNAME) + ".local");
  }
  printHelp();
}

void loop() {
  readFromSerial();
  readFromWiFi();
  readFromSlave();
  serviceSensorUART();
  if (httpServerLive) httpServer.handleClient();
  if (webSocketLive) {
    webSocket.loop();
    wsTxFlush();
  }
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
}

// #include <Arduino.h>
// #include <WiFi.h>
// #include <ArduinoOTA.h>
// #include <FS.h>
// #include <LittleFS.h>
// #include <WebServer.h>
// #include <WebSocketsServer.h>

// #ifndef WIFI_SSID
// #define WIFI_SSID ""
// #endif
// #ifndef WIFI_PASSWORD
// #define WIFI_PASSWORD ""
// #endif
// #ifndef WIFI_HOSTNAME
// #define WIFI_HOSTNAME "me424-main"
// #endif

// WiFiServer CommandServer(3333);
// WiFiClient commandClient;

// WebServer httpServer(80);
// WebSocketsServer webSocket(81);
// bool webSocketLive = false;
// bool httpServerLive = false;

// bool DEBUG = false;

// // =========================
// // Stepper config
// // =========================
// const int NUM_DRIVERS = 6;

// const int STEP_PINS[NUM_DRIVERS] = {25, 33, 32, 27, 26, 4};
// const int DIR_PINS[NUM_DRIVERS]  = {13, 14, 22, 21, 23, 15};

// int pulseDelayUs = 1000;
// long currentPos[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
// volatile bool estopLatched = false;
// String estopSerialBuffer = "";
// String estopWifiBuffer = "";

// enum QueueItemType {
//   Q_POSE = 0,
//   Q_DELAY = 1,
//   Q_CMD = 2
// };

// struct QueueItem {
//   QueueItemType type;
//   long targets[NUM_DRIVERS];
//   long maxSps;
//   int rampSteps;
//   unsigned long delayMs;
//   String cmd;
// };

// const int MAX_QUEUE_ITEMS = 64;
// QueueItem runQueueItems[MAX_QUEUE_ITEMS];
// int runQueueCount = 0;
// bool queueStopRequested = false;

// // Stage-to-controller mapping:
// // Stage 1 = turntable          -> controller 6 (index 5)
// // Stage 2 = base lift pair     -> controllers 1+4 (index 0+3)
// // Stage 3 = arm segment        -> controller 2 (index 1)
// // Stage 4 = arm segment        -> controller 3 (index 2)
// // Stage 5 = wrist              -> controller 5 (index 4)

// const int STAGE1       = 5;   // controller 6
// const int STAGE2_RIGHT = 0;   // controller 1
// const int STAGE3       = 1;   // controller 2
// const int STAGE4       = 2;   // controller 3
// const int STAGE2_LEFT  = 3;   // controller 4
// const int STAGE5       = 4;   // controller 5

// void enforceStage2PairTarget(long targets[NUM_DRIVERS]) {
//   // Stage 2 must always remain paired (controllers 1 and 4).
//   // Use C1 as the authoritative value when both are provided.
//   targets[STAGE2_LEFT] = targets[STAGE2_RIGHT];
// }

// bool invertMotor[NUM_DRIVERS] = {
//   false, // controller 1 / stage 2 right
//   false, // controller 2 / stage 3
//   true,  // controller 3 / stage 4 (reversed)
//   true,  // controller 4 / stage 2 left
//   false, // controller 5 / stage 5
//   false  // controller 6 / stage 1 (turntable)
// };

// // =========================
// // UART to MOSFET slave
// // Assumes RX2/TX2 map to GPIO16/GPIO17 on your board.
// // If not, change these.
// // =========================
// HardwareSerial SlaveSerial(2);
// const int SLAVE_RX_PIN = 16;   // main RX2
// const int SLAVE_TX_PIN = 17;   // main TX2
// const long SLAVE_BAUD = 115200;

// // =========================
// // UART to sensor board
// // Sensor TX(27) -> Main RX(18)
// // Sensor RX(26) <- Main TX(19)
// // =========================
// HardwareSerial SensorSerial(1);
// const int SENSOR_RX_PIN = 18;   // main RX from sensor TX
// const int SENSOR_TX_PIN = 19;   // main TX to sensor RX
// const long SENSOR_BAUD = 115200;

// // =========================
// // Limit states from sensor board
// // true = switch hit
// // =========================
// volatile bool stage2LimitHit = false;
// volatile bool stage3LimitHit = false;
// volatile bool stage4LimitHit = false;
// // Stage 5: Hall + HW-477 on sensor board GPIO14 — homing/zero only, not a motion limit
// volatile bool stage5HallAtZero = false;
// String lastSensorLimMsg = "";

// // =========================
// // Buffers
// // =========================
// String serialBuffer = "";
// String wifiBuffer = "";
// String slaveBuffer = "";
// String sensorBuffer = "";

// // Forward declarations used by queue/emergency helpers
// void handleCommand(String cmd);
// void readFromSlave();

// void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
//   if (type == WStype_TEXT) {
//     String s;
//     s.reserve(length);
//     for (size_t i = 0; i < length; i++) s += (char)payload[i];
//     s.trim();
//     if (s.length()) handleCommand(s);
//   } else if (type == WStype_CONNECTED) {
//     webSocket.sendTXT(num, "WS: connected to ME424 main");
//   }
// }

// static void httpSendLittleFSFile(const char *path, const char *mime) {
//   if (!LittleFS.exists(path)) {
//     httpServer.send(404, "text/plain",
//                     String("LittleFS missing: ") + path +
//                         " — put index.html, app.js, styles.css in data/ then: pio run -e main_board -t uploadfs");
//     return;
//   }
//   File f = LittleFS.open(path, "r");
//   if (!f) {
//     httpServer.send(500, "text/plain", "LittleFS open failed");
//     return;
//   }
//   httpServer.streamFile(f, mime);
//   f.close();
// }

// // =========================
// // Helpers
// // =========================
// void printBoth(const String &msg) {
//   Serial.print(msg);
//   if (commandClient && commandClient.connected()) commandClient.print(msg);
//   if (webSocketLive) {
//     String wscopy = msg;
//     webSocket.broadcastTXT(wscopy);
//   }
// }

// void printlnBoth(const String &msg) {
//   Serial.println(msg);
//   if (commandClient && commandClient.connected()) commandClient.println(msg);
//   if (webSocketLive) {
//     String wscopy = msg;
//     webSocket.broadcastTXT(wscopy);
//   }
// }

// void printlnControl(const String &msg) {
//   Serial.println(msg);
// }

// void printlnDebug(const String &msg) {
//   if (DEBUG) Serial.println("[DBG] " + msg);
// }

// void sendToSlave(const String &msg) {
//   SlaveSerial.println(msg);
//   printlnBoth("[TO SLAVE] " + msg);
// }

// void parseEStopLine(String line) {
//   line.trim();
//   line.toLowerCase();
//   if (line == "estop") {
//     estopLatched = true;
//     sendToSlave("ALL OFF");
//     printlnBoth("ESTOP LATCHED");
//   } else if (line == "estop clear") {
//     estopLatched = false;
//     printlnBoth("ESTOP CLEARED");
//   }
// }

// void pollEmergencyInputs() {
//   while (Serial.available()) {
//     char c = (char)Serial.read();
//     if (c == '\r') continue;
//     if (c == '\n') {
//       if (estopSerialBuffer.length() > 0) {
//         parseEStopLine(estopSerialBuffer);
//         estopSerialBuffer = "";
//       }
//     } else {
//       estopSerialBuffer += c;
//     }
//   }

//   if (commandClient && commandClient.connected()) {
//     while (commandClient.available()) {
//       char c = (char)commandClient.read();
//       if (c == '\r') continue;
//       if (c == '\n') {
//         if (estopWifiBuffer.length() > 0) {
//           parseEStopLine(estopWifiBuffer);
//           estopWifiBuffer = "";
//         }
//       } else {
//         estopWifiBuffer += c;
//       }
//     }
//   }
// }

// void printLimitStatus() {
//   // S5H: 0 = at Hall zero, 1 = away (unlike S2–S4 where 1 = limit hit).
//   printlnBoth("LIMITS S2=" + String(stage2LimitHit ? "1" : "0") +
//               " S3=" + String(stage3LimitHit ? "1" : "0") +
//               " S4=" + String(stage4LimitHit ? "1" : "0") +
//               " S5H=" + String(stage5HallAtZero ? "0" : "1"));
// }

// void parseSensorMessage(String msg) {
//   msg.trim();
//   msg.toUpperCase();

//   if (!msg.startsWith("LIM ")) return;

//   const bool prevS2 = stage2LimitHit;
//   const bool prevS3 = stage3LimitHit;
//   const bool prevS4 = stage4LimitHit;
//   const bool prevS5Hall = stage5HallAtZero;

//   int s2pos = msg.indexOf("S2=");
//   int s3pos = msg.indexOf("S3=");
//   int s4pos = msg.indexOf("S4=");
//   int s5hpos = msg.indexOf("S5H=");

//   if (s2pos >= 0 && s2pos + 3 < (int)msg.length()) {
//     stage2LimitHit = (msg.charAt(s2pos + 3) == '1');
//   }
//   if (s3pos >= 0 && s3pos + 3 < (int)msg.length()) {
//     stage3LimitHit = (msg.charAt(s3pos + 3) == '1');
//   }
//   if (s4pos >= 0 && s4pos + 3 < (int)msg.length()) {
//     stage4LimitHit = (msg.charAt(s4pos + 3) == '1');
//   }
//   if (s5hpos >= 0 && s5hpos + 4 < (int)msg.length()) {
//     bool s5 = (msg.charAt(s5hpos + 4) == '0');
//     stage5HallAtZero = s5;
//     if (s5 && !prevS5Hall) {
//       printlnBoth("S5 HALL: at zero (sensor)");
//     } else if (!s5 && prevS5Hall) {
//       printlnBoth("S5 HALL: left zero zone");
//     }
//   }

//   bool stateChanged = (prevS2 != stage2LimitHit) ||
//                       (prevS3 != stage3LimitHit) ||
//                       (prevS4 != stage4LimitHit) ||
//                       (prevS5Hall != stage5HallAtZero);
//   if (stateChanged || msg != lastSensorLimMsg) {
//     // printlnBoth("[SENSOR] " + msg);
//     printlnDebug("[SENSOR] " + msg);
//     lastSensorLimMsg = msg;
//   }
// }

// void serviceSensorUART() {
//   while (SensorSerial.available()) {
//     char c = (char)SensorSerial.read();

//     if (c == '\r') continue;
//     if (c == '\n') {
//       if (sensorBuffer.length() > 0) {
//         parseSensorMessage(sensorBuffer);
//         sensorBuffer = "";
//       }
//     } else {
//       sensorBuffer += c;
//     }
//   }
// }

// // =========================
// // Direction-aware limit logic
// //
// // Stage 2:  s2down -> forward=false (blocked by limit)
// // Stage 3:  s3down -> forward=true  (blocked by limit)
// // Stage 4:  s4down -> forward=true  (blocked by limit)
// // Stage 5: no limit switch blocking (Hall S5H is for homing only, not hard stop)
// // =========================
// bool shouldStopMotor(int motorIndex, bool forward) {
//   if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) {
//     bool movingDown = !forward;
//     return stage2LimitHit && movingDown;
//   }

//   if (motorIndex == STAGE3) {
//     bool movingDown = forward;
//     return stage3LimitHit && movingDown;
//   }

//   if (motorIndex == STAGE4) {
//     bool movingDown = forward;
//     return stage4LimitHit && movingDown;
//   }

//   return false;
// }

// void setDirectionRaw(int motorIndex, bool forward) {
//   digitalWrite(DIR_PINS[motorIndex], forward ? HIGH : LOW);
// }

// void setDirection(int motorIndex, bool forward) {
//   bool actualForward = invertMotor[motorIndex] ? !forward : forward;
//   setDirectionRaw(motorIndex, actualForward);
//   delayMicroseconds(50);
// }

// // =========================
// // Motion functions
// // =========================
// bool stepMotor(int motorIndex, bool forward, int steps) {
//   if (estopLatched) {
//     printlnBoth("ESTOP: motion blocked");
//     return false;
//   }
//   serviceSensorUART();
//   int movedSteps = 0;

//   if (shouldStopMotor(motorIndex, forward)) {
//     printlnBoth("ABORT: limit active for controller " + String(motorIndex + 1) +
//                 " in requested direction");
//     return false;
//   }

//   setDirection(motorIndex, forward);

//   for (int i = 0; i < steps; i++) {
//     pollEmergencyInputs();
//     if (estopLatched) {
//       printlnBoth("ESTOP: controller " + String(motorIndex + 1) + " stopped");
//       currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
//       return false;
//     }
//     serviceSensorUART();

//     if (shouldStopMotor(motorIndex, forward)) {
//       printlnBoth("STOP: limit hit on controller " + String(motorIndex + 1));
//       currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
//       return false;
//     }

//     digitalWrite(STEP_PINS[motorIndex], HIGH);
//     delayMicroseconds(pulseDelayUs);
//     digitalWrite(STEP_PINS[motorIndex], LOW);
//     delayMicroseconds(pulseDelayUs);
//     movedSteps++;
//   }

//   currentPos[motorIndex] += forward ? movedSteps : -movedSteps;
//   return true;
// }

// bool stepStage2(bool forward, int steps) {
//   if (estopLatched) {
//     printlnBoth("ESTOP: motion blocked");
//     return false;
//   }
//   serviceSensorUART();
//   int movedSteps = 0;

//   bool movingDown = !forward;
//   if (stage2LimitHit && movingDown) {
//     printlnBoth("ABORT: Stage 2 limit already active for downward motion");
//     return false;
//   }

//   setDirection(STAGE2_RIGHT, forward);
//   setDirection(STAGE2_LEFT, forward);

//   for (int i = 0; i < steps; i++) {
//     pollEmergencyInputs();
//     if (estopLatched) {
//       printlnBoth("ESTOP: Stage 2 stopped");
//       long d = forward ? movedSteps : -movedSteps;
//       currentPos[STAGE2_RIGHT] += d;
//       currentPos[STAGE2_LEFT]  += d;
//       return false;
//     }
//     serviceSensorUART();

//     if (stage2LimitHit && movingDown) {
//       printlnBoth("STOP: Stage 2 limit hit");
//       long delta = forward ? movedSteps : -movedSteps;
//       currentPos[STAGE2_RIGHT] += delta;
//       currentPos[STAGE2_LEFT]  += delta;
//       return false;
//     }

//     digitalWrite(STEP_PINS[STAGE2_RIGHT], HIGH);
//     digitalWrite(STEP_PINS[STAGE2_LEFT], HIGH);
//     delayMicroseconds(pulseDelayUs);

//     digitalWrite(STEP_PINS[STAGE2_RIGHT], LOW);
//     digitalWrite(STEP_PINS[STAGE2_LEFT], LOW);
//     delayMicroseconds(pulseDelayUs);
//     movedSteps++;
//   }

//   long delta = forward ? movedSteps : -movedSteps;
//   currentPos[STAGE2_RIGHT] += delta;
//   currentPos[STAGE2_LEFT]  += delta;
//   return true;
// }

// void runControllerCommand(int controllerNum, bool forward, int steps) {
//   int motorIndex = controllerNum - 1;

//   if (motorIndex < 0 || motorIndex >= NUM_DRIVERS) {
//     printlnBoth("Invalid controller number");
//     return;
//   }

//   bool ok = false;
//   if (motorIndex == STAGE2_RIGHT || motorIndex == STAGE2_LEFT) {
//     // Always keep stage 2 pair mechanically locked.
//     ok = stepStage2(forward, steps);
//   } else {
//     ok = stepMotor(motorIndex, forward, steps);
//   }

//   if (ok) {
//     printlnBoth("Controller " + String(controllerNum) +
//                 (forward ? " forward " : " reverse ") +
//                 String(steps) + " steps");
//   } else {
//     printlnBoth("Controller " + String(controllerNum) + " aborted");
//   }
// }

// // =========================
// // Sequential / parallel execution
// //
// // MotorMove holds a single motor's move parameters.
// // Stage 2 expands to two MotorMove entries (RIGHT + LEFT).
// // =========================
// struct MotorMove {
//   int  motorIndex;
//   bool forward;
//   int  stepsRemaining;
//   int  originalSteps;
// };

// // Translate one stage/controller command string into MotorMove entries.
// // Returns number of entries written (0 = unrecognised or bad step count).
// int resolveToMoves(String cmd, MotorMove *moves, int maxMoves) {
//   cmd.trim();
//   cmd.toLowerCase();

//   int steps = 0;
//   int spPos = cmd.lastIndexOf(' ');
//   if (spPos > 0) steps = cmd.substring(spPos + 1).toInt();
//   if (steps <= 0) return 0;

//   // Stage 1 — turntable
//   if (cmd.startsWith("s1cw "))  { moves[0] = {STAGE1, true,  steps, steps}; return 1; }
//   if (cmd.startsWith("s1ccw ")) { moves[0] = {STAGE1, false, steps, steps}; return 1; }

//   // Stage 2 — paired motors
//   if (cmd.startsWith("s2up ")) {
//     if (maxMoves < 2) return 0;
//     moves[0] = {STAGE2_RIGHT, true, steps, steps};
//     moves[1] = {STAGE2_LEFT,  true, steps, steps};
//     return 2;
//   }
//   if (cmd.startsWith("s2down ")) {
//     if (maxMoves < 2) return 0;
//     moves[0] = {STAGE2_RIGHT, false, steps, steps};
//     moves[1] = {STAGE2_LEFT,  false, steps, steps};
//     return 2;
//   }

//   // Stage 3
//   if (cmd.startsWith("s3up "))   { moves[0] = {STAGE3, false, steps, steps}; return 1; }
//   if (cmd.startsWith("s3down ")) { moves[0] = {STAGE3, true,  steps, steps}; return 1; }

//   // Stage 4
//   if (cmd.startsWith("s4up "))   { moves[0] = {STAGE4, false, steps, steps}; return 1; }
//   if (cmd.startsWith("s4down ")) { moves[0] = {STAGE4, true,  steps, steps}; return 1; }

//   // Stage 5
//   if (cmd.startsWith("s5cw "))   { moves[0] = {STAGE5, false, steps, steps}; return 1; }
//   if (cmd.startsWith("s5ccw "))  { moves[0] = {STAGE5, true,  steps, steps}; return 1; }

//   // Raw controller: cXf / cXr
//   if (cmd.length() >= 5 && cmd.charAt(0) == 'c') {
//     int num     = cmd.charAt(1) - '0';
//     char dirCh  = cmd.charAt(2);
//     if (num >= 1 && num <= 6 && (dirCh == 'f' || dirCh == 'r')) {
//       if (num == 1 || num == 4) {
//         if (maxMoves < 2) return 0;
//         bool fwd = (dirCh == 'f');
//         moves[0] = {STAGE2_RIGHT, fwd, steps, steps};
//         moves[1] = {STAGE2_LEFT,  fwd, steps, steps};
//         return 2;
//       }
//       moves[0] = {num - 1, dirCh == 'f', steps, steps};
//       return 1;
//     }
//   }

//   return 0;
// }

// // Step multiple motors in parallel.
// // Each motor is pulsed on the same clock tick; motors with fewer steps
// // finish early while the others keep running. Limits are checked per-motor
// // every iteration.
// bool stepMultiple(MotorMove *moves, int count) {
//   if (estopLatched) {
//     printlnBoth("ESTOP: motion blocked");
//     return false;
//   }
//   // Pre-flight limit check
//   for (int i = 0; i < count; i++) {
//     if (shouldStopMotor(moves[i].motorIndex, moves[i].forward)) {
//       printlnBoth("PAR ABORT: limit active for controller " +
//                   String(moves[i].motorIndex + 1));
//       return false;
//     }
//   }

//   // Set all directions before any stepping starts
//   for (int i = 0; i < count; i++) {
//     setDirection(moves[i].motorIndex, moves[i].forward);
//   }

//   while (true) {
//     pollEmergencyInputs();
//     if (estopLatched) {
//       printlnBoth("ESTOP: parallel motion stopped");
//       for (int i = 0; i < count; i++) {
//         int moved = moves[i].originalSteps - moves[i].stepsRemaining;
//         currentPos[moves[i].motorIndex] += moves[i].forward ? moved : -moved;
//       }
//       return false;
//     }
//     serviceSensorUART();

//     // Check limits and count still-active motors
//     int active = 0;
//     for (int i = 0; i < count; i++) {
//       if (moves[i].stepsRemaining <= 0) continue;
//       if (shouldStopMotor(moves[i].motorIndex, moves[i].forward)) {
//         printlnBoth("PAR STOP: limit hit on controller " +
//                     String(moves[i].motorIndex + 1));
//         moves[i].stepsRemaining = 0;
//         continue;
//       }
//       active++;
//     }
//     if (active == 0) break;

//     // Pulse HIGH for all active motors
//     for (int i = 0; i < count; i++) {
//       if (moves[i].stepsRemaining > 0)
//         digitalWrite(STEP_PINS[moves[i].motorIndex], HIGH);
//     }
//     delayMicroseconds(pulseDelayUs);

//     // Pulse LOW and decrement
//     for (int i = 0; i < count; i++) {
//       if (moves[i].stepsRemaining > 0) {
//         digitalWrite(STEP_PINS[moves[i].motorIndex], LOW);
//         moves[i].stepsRemaining--;
//       }
//     }
//     delayMicroseconds(pulseDelayUs);
//   }

//   for (int i = 0; i < count; i++) {
//     int moved = moves[i].originalSteps - moves[i].stepsRemaining;
//     currentPos[moves[i].motorIndex] += moves[i].forward ? moved : -moved;
//   }
//   return true;
// }

// int parseSyncAbsArgs(const String &cmd, long targets[NUM_DRIVERS], long &maxSps, int &rampSteps) {
//   // Format:
//   // syncabs t1 t2 t3 t4 t5 t6 max_sps ramp_steps
//   return sscanf(cmd.c_str(),
//                 "syncabs %ld %ld %ld %ld %ld %ld %ld %d",
//                 &targets[0], &targets[1], &targets[2],
//                 &targets[3], &targets[4], &targets[5],
//                 &maxSps, &rampSteps);
// }

// bool runSyncAbs(long targets[NUM_DRIVERS], long maxSps, int rampSteps) {
//   if (estopLatched) {
//     printlnBoth("ESTOP: motion blocked");
//     return false;
//   }
//   if (maxSps < 1) {
//     printlnBoth("SYNCABS: max_sps must be >= 1");
//     return false;
//   }

//   enforceStage2PairTarget(targets);

//   long delta[NUM_DRIVERS];
//   long absDelta[NUM_DRIVERS];
//   bool forward[NUM_DRIVERS];
//   bool axisActive[NUM_DRIVERS];
//   long accum[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
//   long stepped[NUM_DRIVERS] = {0, 0, 0, 0, 0, 0};
//   long totalSteps = 0;
//   bool anyAxisBlocked = false;

//   for (int i = 0; i < NUM_DRIVERS; i++) {
//     delta[i] = targets[i] - currentPos[i];
//     absDelta[i] = labs(delta[i]);
//     forward[i] = (delta[i] >= 0);
//     axisActive[i] = (absDelta[i] > 0);
//     if (absDelta[i] > totalSteps) totalSteps = absDelta[i];
//   }

//   if (totalSteps == 0) {
//     printlnBoth("SYNCABS: already at target");
//     return true;
//   }

//   for (int i = 0; i < NUM_DRIVERS; i++) {
//     if (!axisActive[i]) continue;
//     if (shouldStopMotor(i, forward[i])) {
//       printlnBoth("SYNCABS: controller " + String(i + 1) +
//                   " blocked by active limit; skipping this axis");
//       axisActive[i] = false;
//       anyAxisBlocked = true;
//       continue;
//     }
//     setDirection(i, forward[i]);
//   }

//   long minDelayUs = 500000L / maxSps; // half-cycle delay
//   if (minDelayUs < 100) minDelayUs = 100;
//   long slowDelayUs = max((long)pulseDelayUs, minDelayUs * 3L);
//   if (rampSteps < 1) rampSteps = 1;
//   if ((long)rampSteps * 2L > totalSteps) rampSteps = (int)(totalSteps / 2L);
//   if (rampSteps < 1) rampSteps = 1;

//   for (long m = 0; m < totalSteps; m++) {
//     pollEmergencyInputs();
//     if (estopLatched) {
//       printlnBoth("ESTOP: sync motion stopped");
//       return false;
//     }
//     serviceSensorUART();

//     // Ramp profile on the shared timeline
//     long dUs = minDelayUs;
//     if (m < rampSteps) {
//       long num = (long)(rampSteps - m) * (slowDelayUs - minDelayUs);
//       dUs = minDelayUs + num / rampSteps;
//     } else if (m >= totalSteps - rampSteps) {
//       long phase = m - (totalSteps - rampSteps);
//       long num = phase * (slowDelayUs - minDelayUs);
//       dUs = minDelayUs + num / rampSteps;
//     }

//     bool pulseThisTick[NUM_DRIVERS] = {false, false, false, false, false, false};

//     // Determine which axes step this master tick (Bresenham style)
//     for (int i = 0; i < NUM_DRIVERS; i++) {
//       if (!axisActive[i]) continue;
//       if (shouldStopMotor(i, forward[i])) {
//         printlnBoth("SYNCABS: limit hit on controller " + String(i + 1) +
//                     "; stopping only this axis");
//         axisActive[i] = false;
//         anyAxisBlocked = true;
//         continue;
//       }
//       accum[i] += absDelta[i];
//       if (accum[i] >= totalSteps) {
//         accum[i] -= totalSteps;
//         pulseThisTick[i] = true;
//       }
//     }

//     for (int i = 0; i < NUM_DRIVERS; i++) {
//       if (pulseThisTick[i]) digitalWrite(STEP_PINS[i], HIGH);
//     }
//     delayMicroseconds((int)dUs);
//     for (int i = 0; i < NUM_DRIVERS; i++) {
//       if (pulseThisTick[i]) {
//         digitalWrite(STEP_PINS[i], LOW);
//         stepped[i]++;
//       }
//     }
//     delayMicroseconds((int)dUs);
//   }

//   for (int i = 0; i < NUM_DRIVERS; i++) {
//     currentPos[i] += forward[i] ? stepped[i] : -stepped[i];
//   }
//   if (anyAxisBlocked) {
//     printlnBoth("SYNCABS: completed with one or more axes blocked by limits");
//   }
//   return true;
// }

// bool queueAddPose(long targets[NUM_DRIVERS], long maxSps, int rampSteps) {
//   if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
//   enforceStage2PairTarget(targets);
//   QueueItem &it = runQueueItems[runQueueCount++];
//   it.type = Q_POSE;
//   for (int i = 0; i < NUM_DRIVERS; i++) it.targets[i] = targets[i];
//   it.maxSps = maxSps;
//   it.rampSteps = rampSteps;
//   it.delayMs = 0;
//   it.cmd = "";
//   return true;
// }

// bool queueAddDelay(unsigned long ms) {
//   if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
//   QueueItem &it = runQueueItems[runQueueCount++];
//   it.type = Q_DELAY;
//   it.maxSps = 0;
//   it.rampSteps = 0;
//   it.delayMs = ms;
//   it.cmd = "";
//   return true;
// }

// bool queueAddCmd(const String &cmd) {
//   if (runQueueCount >= MAX_QUEUE_ITEMS) return false;
//   QueueItem &it = runQueueItems[runQueueCount++];
//   it.type = Q_CMD;
//   it.maxSps = 0;
//   it.rampSteps = 0;
//   it.delayMs = 0;
//   it.cmd = cmd;
//   return true;
// }

// bool queueDelayWait(unsigned long ms) {
//   unsigned long t0 = millis();
//   while ((millis() - t0) < ms) {
//     pollEmergencyInputs();
//     serviceSensorUART();
//     readFromSlave();
//     if (estopLatched || queueStopRequested) return false;
//     delay(2);
//   }
//   return true;
// }

// bool runQueueExecution() {
//   if (runQueueCount == 0) {
//     printlnBoth("QRUN: queue empty");
//     return false;
//   }

//   if (estopLatched) {
//     printlnBoth("QRUN: blocked by ESTOP");
//     return false;
//   }

//   queueStopRequested = false;
//   printlnBoth("QRUN: start");

//   for (int i = 0; i < runQueueCount; i++) {
//     pollEmergencyInputs();
//     if (estopLatched || queueStopRequested) {
//       printlnBoth("QRUN: stopped");
//       return false;
//     }

//     QueueItem &it = runQueueItems[i];
//     if (it.type == Q_POSE) {
//       bool ok = runSyncAbs(it.targets, it.maxSps, it.rampSteps);
//       if (!ok) {
//         printlnBoth("QRUN: pose aborted at item " + String(i));
//         return false;
//       }
//     } else if (it.type == Q_DELAY) {
//       if (!queueDelayWait(it.delayMs)) {
//         printlnBoth("QRUN: delay interrupted at item " + String(i));
//         return false;
//       }
//     } else if (it.type == Q_CMD) {
//       handleCommand(it.cmd);
//     }
//   }

//   printlnBoth("QRUN: done");
//   return true;
// }

// // Forward declaration so runSequential can call handleCommand
// void handleCommand(String cmd);

// // Split cmdList by ',' and execute each sub-command in order.
// // Any command type (motor, MOSFET, utility) is supported.
// void runSequential(const String &cmdList) {
//   int start = 0;
//   int idx   = 0;
//   while (start < (int)cmdList.length()) {
//     int comma = cmdList.indexOf(',', start);
//     String sub = (comma < 0)
//                    ? cmdList.substring(start)
//                    : cmdList.substring(start, comma);
//     sub.trim();
//     if (sub.length() > 0) {
//       printlnBoth("SEQ[" + String(idx++) + "]: " + sub);
//       handleCommand(sub);
//     }
//     if (comma < 0) break;
//     start = comma + 1;
//   }
//   printlnBoth("SEQ: done");
// }

// // Split cmdList by ',', resolve each to motor moves, run all simultaneously.
// // Only motor/stage commands are supported in parallel mode.
// void runParallel(const String &cmdList) {
//   // Max 12 moves: up to 6 commands, stage2 counts as 2
//   MotorMove moves[12];
//   int totalMoves = 0;

//   int start = 0;
//   while (start < (int)cmdList.length()) {
//     int comma = cmdList.indexOf(',', start);
//     String sub = (comma < 0)
//                    ? cmdList.substring(start)
//                    : cmdList.substring(start, comma);
//     sub.trim();

//     if (sub.length() > 0) {
//       int added = resolveToMoves(sub, moves + totalMoves, 12 - totalMoves);
//       if (added == 0) {
//         printlnBoth("PAR: unrecognised motor command: " + sub);
//         return;
//       }
//       totalMoves += added;
//     }

//     if (comma < 0) break;
//     start = comma + 1;
//   }

//   if (totalMoves == 0) {
//     printlnBoth("PAR: no valid motor commands");
//     return;
//   }

//   bool ok = stepMultiple(moves, totalMoves);
//   printlnBoth(ok ? "PAR: done" : "PAR: aborted");
// }

// void applyPositionState(long p[NUM_DRIVERS]) {
//   enforceStage2PairTarget(p);
//   for (int i = 0; i < NUM_DRIVERS; i++) currentPos[i] = p[i];
// }

// static const int HOME_CHUNK_STEPS = 40;
// static const long HOME_MAX_SEEK_STEPS = 250000L;
// // Stage 5 Hall seek: slower pulse + smaller chunks (does not change global `speed` / manual jog).
// static const int HOME_S5_CHUNK_STEPS = 24;
// static const int HOME_S5_PULSE_DELAY_US = 2400;

// bool homeStage1SoftZero() {
//   if (estopLatched) return false;
//   long p[NUM_DRIVERS];
//   for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//   p[STAGE1] = 0;
//   applyPositionState(p);
//   printlnBoth("HOME S1: soft zero (C6=0 at current position)");
//   return true;
// }

// bool homeStage2ToLimit() {
//   long moved = 0;
//   while (moved < HOME_MAX_SEEK_STEPS) {
//     pollEmergencyInputs();
//     serviceSensorUART();
//     if (estopLatched) return false;
//     if (stage2LimitHit) break;
//     long chunk = HOME_CHUNK_STEPS;
//     if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
//     if (chunk <= 0) break;
//     if (!stepStage2(false, (int)chunk)) return false;
//     moved += chunk;
//   }
//   if (!stage2LimitHit) {
//     printlnBoth("HOME S2: limit not reached");
//     return false;
//   }
//   long p[NUM_DRIVERS];
//   for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//   p[STAGE2_RIGHT] = 0;
//   p[STAGE2_LEFT] = 0;
//   applyPositionState(p);
//   printlnBoth("HOME S2: zero at limit");
//   return true;
// }

// bool homeStage3ToLimit() {
//   long moved = 0;
//   while (moved < HOME_MAX_SEEK_STEPS) {
//     pollEmergencyInputs();
//     serviceSensorUART();
//     if (estopLatched) return false;
//     if (stage3LimitHit) break;
//     long chunk = HOME_CHUNK_STEPS;
//     if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
//     if (chunk <= 0) break;
//     if (!stepMotor(STAGE3, true, (int)chunk)) return false;
//     moved += chunk;
//   }
//   if (!stage3LimitHit) {
//     printlnBoth("HOME S3: limit not reached");
//     return false;
//   }
//   long p[NUM_DRIVERS];
//   for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//   p[STAGE3] = 0;
//   applyPositionState(p);
//   printlnBoth("HOME S3: zero at limit");
//   return true;
// }

// bool homeStage4ToLimit() {
//   long moved = 0;
//   while (moved < HOME_MAX_SEEK_STEPS) {
//     pollEmergencyInputs();
//     serviceSensorUART();
//     if (estopLatched) return false;
//     if (stage4LimitHit) break;
//     long chunk = HOME_CHUNK_STEPS;
//     if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
//     if (chunk <= 0) break;
//     if (!stepMotor(STAGE4, true, (int)chunk)) return false;
//     moved += chunk;
//   }
//   if (!stage4LimitHit) {
//     printlnBoth("HOME S4: limit not reached");
//     return false;
//   }
//   long p[NUM_DRIVERS];
//   for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//   p[STAGE4] = 0;
//   applyPositionState(p);
//   printlnBoth("HOME S4: zero at limit");
//   return true;
// }

// bool homeStage5ToHall() {
//   serviceSensorUART();
//   if (estopLatched) return false;

//   if (stage5HallAtZero) {
//     long p[NUM_DRIVERS];
//     for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//     p[STAGE5] = 0;
//     applyPositionState(p);
//     printlnBoth("HOME S5: already at hall, C5=0");
//     return true;
//   }

//   const int savedPulse = pulseDelayUs;
//   pulseDelayUs = HOME_S5_PULSE_DELAY_US;

//   const bool dirs[2] = { true, false };
//   for (int d = 0; d < 2; d++) {
//     long moved = 0;
//     while (moved < HOME_MAX_SEEK_STEPS) {
//       pollEmergencyInputs();
//       serviceSensorUART();
//       if (estopLatched) {
//         pulseDelayUs = savedPulse;
//         return false;
//       }
//       if (stage5HallAtZero) {
//         long p[NUM_DRIVERS];
//         for (int i = 0; i < NUM_DRIVERS; i++) p[i] = currentPos[i];
//         p[STAGE5] = 0;
//         applyPositionState(p);
//         printlnBoth("HOME S5: zero at hall");
//         pulseDelayUs = savedPulse;
//         return true;
//       }
//       long chunk = HOME_S5_CHUNK_STEPS;
//       if (moved + chunk > HOME_MAX_SEEK_STEPS) chunk = HOME_MAX_SEEK_STEPS - moved;
//       if (chunk <= 0) break;
//       if (!stepMotor(STAGE5, dirs[d], (int)chunk)) {
//         pulseDelayUs = savedPulse;
//         return false;
//       }
//       moved += chunk;
//     }
//   }
//   pulseDelayUs = savedPulse;
//   printlnBoth("HOME S5: hall not found (check S5H / HW-477 / magnet; try home s5 from other side)");
//   return false;
// }

// void printHelp() {
//   printlnBoth("");
//   printlnBoth("Stage commands:");
//   printlnBoth("  s1cw 400      -> stage 1 (turntable) clockwise");
//   printlnBoth("  s1ccw 400     -> stage 1 (turntable) counter-clockwise");
//   printlnBoth("  s2up 400      -> stage 2 up 400 steps");
//   printlnBoth("  s2down 400    -> stage 2 down 400 steps");
//   printlnBoth("  s3up 400      -> stage 3 up 400 steps");
//   printlnBoth("  s3down 400    -> stage 3 down 400 steps");
//   printlnBoth("  s4up 400      -> stage 4 up 400 steps");
//   printlnBoth("  s4down 400    -> stage 4 down 400 steps");
//   printlnBoth("  s5cw 400      -> stage 5 clockwise 400 steps");
//   printlnBoth("  s5ccw 400     -> stage 5 counter-clockwise 400 steps");
//   printlnBoth("");

//   printlnBoth("Controller commands (raw):");
//   printlnBoth("  c1f 400       -> controller 1 forward 400 steps");
//   printlnBoth("  c1r 400       -> controller 1 reverse 400 steps");
//   printlnBoth("  c2f 400       -> controller 2 forward 400 steps");
//   printlnBoth("  c2r 400       -> controller 2 reverse 400 steps");
//   printlnBoth("  c3f 400       -> controller 3 forward 400 steps");
//   printlnBoth("  c3r 400       -> controller 3 reverse 400 steps");
//   printlnBoth("  c4f 400       -> controller 4 forward 400 steps");
//   printlnBoth("  c4r 400       -> controller 4 reverse 400 steps");
//   printlnBoth("  c5f 400       -> controller 5 forward 400 steps");
//   printlnBoth("  c5r 400       -> controller 5 reverse 400 steps");
//   printlnBoth("  c6f 400       -> controller 6 forward 400 steps");
//   printlnBoth("  c6r 400       -> controller 6 reverse 400 steps");
//   printlnBoth("");

//   printlnBoth("Sequence commands:");
//   printlnBoth("  seq s2up 400, s3down 200, mag on   -> run in order");
//   printlnBoth("  par s2up 400, s5cw 200             -> run simultaneously");
//   printlnBoth("  (par supports motor/stage commands only)");
//   printlnBoth("  syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps");
//   printlnBoth("     -> synchronized absolute move with accel/decel ramp");
//   printlnBoth("  qclear");
//   printlnBoth("  qadd pose t1 t2 t3 t4 t5 t6 maxSps rampSteps");
//   printlnBoth("  qadd delay ms");
//   printlnBoth("  qadd cmd <firmware command>");
//   printlnBoth("  qlist");
//   printlnBoth("  qrun");
//   printlnBoth("  qstop");
//   printlnBoth("  qstatus");
//   printlnBoth("");

//   printlnBoth("Other:");
//   printlnBoth("  speed 800     -> set pulse delay in microseconds");
//   printlnBoth("  where         -> print current tracked positions");
//   printlnBoth("  setpos p1 p2 p3 p4 p5 p6 -> overwrite tracked positions");
//   printlnBoth("  home s1       -> soft zero turntable (C6=0 here)");
//   printlnBoth("  home s2..s4   -> drive to limit switches, then zero those axes");
//   printlnBoth("  home s5       -> find Hall zero (S5H), then zero C5");
//   printlnBoth("  estop         -> latch emergency stop (blocks all motion)");
//   printlnBoth("  estop clear   -> clear e-stop latch");
//   printlnBoth("  estop status  -> print e-stop state");
//   printlnBoth("  limits        -> request fresh LIM (+S5H hall) from sensor, print state");
//   printlnBoth("");

//   printlnBoth("MOSFET / attachment commands:");
//   printlnBoth("  mag on");
//   printlnBoth("  mag off");
//   printlnBoth("  vac on");
//   printlnBoth("  vac off");
//   printlnBoth("  saw on");
//   printlnBoth("  saw off");
//   printlnBoth("  alloff");
//   printlnBoth("  mstatus");
//   printlnBoth("");

//   printlnBoth("  help");
//   printlnBoth("");
//   printlnBoth("Current pulse delay: " + String(pulseDelayUs) + " us");
//   printlnBoth("");
// }

// // =========================
// // Command handling
// // =========================
// void handleCommand(String cmd) {
//   cmd.trim();
//   cmd.toLowerCase();

//   if (cmd.length() == 0) return;

//   if (cmd == "help") {
//     printHelp();
//     return;
//   }

//   if (cmd == "limits") {
//     // Ask sensor board for a fresh LIM line (includes S5H Hall-at-zero).
//     SensorSerial.println("STATUS");
//     unsigned long t0 = millis();
//     while (millis() - t0 < 80) {
//       serviceSensorUART();
//       pollEmergencyInputs();
//     }
//     printLimitStatus();
//     return;
//   }

//   if (cmd == "estop") {
//     estopLatched = true;
//     sendToSlave("ALL OFF");
//     printlnBoth("ESTOP LATCHED");
//     return;
//   }

//   if (cmd == "estop clear") {
//     estopLatched = false;
//     printlnBoth("ESTOP CLEARED");
//     return;
//   }

//   if (cmd == "estop status") {
//     printlnBoth(String("ESTOP=") + (estopLatched ? "1" : "0"));
//     return;
//   }

//   if (cmd.startsWith("home ")) {
//     String h = cmd.substring(5);
//     h.trim();
//     if (h == "s1") {
//       homeStage1SoftZero();
//     } else if (h == "s2") {
//       homeStage2ToLimit();
//     } else if (h == "s3") {
//       homeStage3ToLimit();
//     } else if (h == "s4") {
//       homeStage4ToLimit();
//     } else if (h == "s5") {
//       homeStage5ToHall();
//     } else {
//       printlnBoth("Usage: home s1 | home s2 | home s3 | home s4 | home s5");
//     }
//     return;
//   }

//   if (cmd == "qclear") {
//     runQueueCount = 0;
//     queueStopRequested = false;
//     printlnBoth("Q: cleared");
//     return;
//   }

//   if (cmd == "qstop") {
//     queueStopRequested = true;
//     printlnBoth("Q: stop requested");
//     return;
//   }

//   if (cmd == "qstatus") {
//     printlnBoth("Q: count=" + String(runQueueCount) +
//                 " stop=" + String(queueStopRequested ? "1" : "0"));
//     return;
//   }

//   if (cmd == "qlist") {
//     printlnBoth("Q: items=" + String(runQueueCount));
//     for (int i = 0; i < runQueueCount; i++) {
//       QueueItem &it = runQueueItems[i];
//       if (it.type == Q_POSE) {
//         printlnBoth("Q[" + String(i) + "] POSE " +
//                     String(it.targets[0]) + " " + String(it.targets[1]) + " " +
//                     String(it.targets[2]) + " " + String(it.targets[3]) + " " +
//                     String(it.targets[4]) + " " + String(it.targets[5]) +
//                     " maxSps=" + String(it.maxSps) +
//                     " ramp=" + String(it.rampSteps));
//       } else if (it.type == Q_DELAY) {
//         printlnBoth("Q[" + String(i) + "] DELAY " + String(it.delayMs) + "ms");
//       } else {
//         printlnBoth("Q[" + String(i) + "] CMD " + it.cmd);
//       }
//     }
//     return;
//   }

//   if (cmd == "qrun") {
//     runQueueExecution();
//     return;
//   }

//   if (cmd.startsWith("qadd pose ")) {
//     long t[NUM_DRIVERS];
//     long maxSps;
//     int rampSteps;
//     if (sscanf(cmd.c_str(),
//                "qadd pose %ld %ld %ld %ld %ld %ld %ld %d",
//                &t[0], &t[1], &t[2], &t[3], &t[4], &t[5], &maxSps, &rampSteps) == 8) {
//       if (queueAddPose(t, maxSps, rampSteps)) {
//         printlnBoth("Q: added pose");
//       } else {
//         printlnBoth("Q: full");
//       }
//     } else {
//       printlnBoth("Usage: qadd pose t1 t2 t3 t4 t5 t6 maxSps rampSteps");
//     }
//     return;
//   }

//   if (cmd.startsWith("qadd delay ")) {
//     unsigned long ms = (unsigned long)cmd.substring(11).toInt();
//     if (ms > 0 || cmd.substring(11) == "0") {
//       if (queueAddDelay(ms)) {
//         printlnBoth("Q: added delay");
//       } else {
//         printlnBoth("Q: full");
//       }
//     } else {
//       printlnBoth("Usage: qadd delay ms");
//     }
//     return;
//   }

//   if (cmd.startsWith("qadd cmd ")) {
//     String qcmd = cmd.substring(9);
//     qcmd.trim();
//     if (qcmd.length() == 0) {
//       printlnBoth("Usage: qadd cmd <firmware command>");
//       return;
//     }
//     if (queueAddCmd(qcmd)) {
//       printlnBoth("Q: added cmd");
//     } else {
//       printlnBoth("Q: full");
//     }
//     return;
//   }

//   if (cmd == "where") {
//     // Keep pair visible as unified value in position reporting.
//     currentPos[STAGE2_LEFT] = currentPos[STAGE2_RIGHT];
//     printlnBoth("POS C1=" + String(currentPos[0]) +
//                 " C2=" + String(currentPos[1]) +
//                 " C3=" + String(currentPos[2]) +
//                 " C4=" + String(currentPos[3]) +
//                 " C5=" + String(currentPos[4]) +
//                 " C6=" + String(currentPos[5]));
//     return;
//   }

//   if (cmd.startsWith("setpos ")) {
//     long p[NUM_DRIVERS];
//     if (sscanf(cmd.c_str(), "setpos %ld %ld %ld %ld %ld %ld",
//                &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]) == 6) {
//       // Stage 2 pair is always unified.
//       p[STAGE2_LEFT] = p[STAGE2_RIGHT];
//       for (int i = 0; i < NUM_DRIVERS; i++) currentPos[i] = p[i];
//       printlnBoth("Position state updated");
//     } else {
//       printlnBoth("Usage: setpos p1 p2 p3 p4 p5 p6");
//     }
//     return;
//   }

//   if (cmd.startsWith("syncabs ")) {
//     long targets[NUM_DRIVERS];
//     long maxSps = 0;
//     int rampSteps = 0;
//     if (parseSyncAbsArgs(cmd, targets, maxSps, rampSteps) == 8) {
//       bool ok = runSyncAbs(targets, maxSps, rampSteps);
//       printlnBoth(ok ? "SYNCABS: done" : "SYNCABS: aborted");
//     } else {
//       printlnBoth("Usage: syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps");
//     }
//     return;
//   }

//   if (cmd.startsWith("seq ")) {
//     runSequential(cmd.substring(4));
//     return;
//   }

//   if (cmd.startsWith("par ")) {
//     runParallel(cmd.substring(4));
//     return;
//   }

//   if (cmd.startsWith("speed ")) {
//     int newDelay = cmd.substring(6).toInt();
//     if (newDelay >= 100) {
//       pulseDelayUs = newDelay;
//       printlnBoth("Pulse delay set to " + String(pulseDelayUs) + " us");
//     } else {
//       printlnBoth("Use a value >= 100");
//     }
//     return;
//   }

//   // Stage 1 (turntable)
//   if (cmd.startsWith("s1cw ")) {
//     int steps = cmd.substring(5).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE1, true, steps);
//       printlnBoth(ok ? "Stage 1 CW done" : "Stage 1 CW aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   if (cmd.startsWith("s1ccw ")) {
//     int steps = cmd.substring(6).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE1, false, steps);
//       printlnBoth(ok ? "Stage 1 CCW done" : "Stage 1 CCW aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   // Stage 2
//   if (cmd.startsWith("s2up ")) {
//     int steps = cmd.substring(5).toInt();
//     if (steps > 0) {
//       bool ok = stepStage2(true, steps);
//       printlnBoth(ok ? "Stage 2 up done" : "Stage 2 up aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   if (cmd.startsWith("s2down ")) {
//     int steps = cmd.substring(7).toInt();
//     if (steps > 0) {
//       bool ok = stepStage2(false, steps);
//       printlnBoth(ok ? "Stage 2 down done" : "Stage 2 down aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   // Stage 3
//   if (cmd.startsWith("s3down ")) {
//     int steps = cmd.substring(7).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE3, true, steps);
//       printlnBoth(ok ? "Stage 3 down done" : "Stage 3 down aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   if (cmd.startsWith("s3up ")) {
//     int steps = cmd.substring(5).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE3, false, steps);
//       printlnBoth(ok ? "Stage 3 up done" : "Stage 3 up aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   // Stage 4
//   if (cmd.startsWith("s4up ")) {
//     int steps = cmd.substring(5).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE4, false, steps);
//       printlnBoth(ok ? "Stage 4 up done" : "Stage 4 up aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   if (cmd.startsWith("s4down ")) {
//     int steps = cmd.substring(7).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE4, true, steps);
//       printlnBoth(ok ? "Stage 4 down done" : "Stage 4 down aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   // Stage 5
//   if (cmd.startsWith("s5ccw ")) {
//     int steps = cmd.substring(6).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE5, true, steps);
//       printlnBoth(ok ? "Stage 5 CCW done" : "Stage 5 CCW aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   if (cmd.startsWith("s5cw ")) {
//     int steps = cmd.substring(5).toInt();
//     if (steps > 0) {
//       bool ok = stepMotor(STAGE5, false, steps);
//       printlnBoth(ok ? "Stage 5 CW done" : "Stage 5 CW aborted");
//     } else {
//       printlnBoth("Invalid step count");
//     }
//     return;
//   }

//   // Raw controller commands
//   if (cmd.length() >= 5 && cmd.charAt(0) == 'c') {
//     int controllerNum = cmd.charAt(1) - '0';
//     char dirChar = cmd.charAt(2);
//     int spacePos = cmd.indexOf(' ');

//     if (controllerNum >= 1 && controllerNum <= 6 &&
//         (dirChar == 'f' || dirChar == 'r') &&
//         spacePos > 0) {

//       int steps = cmd.substring(spacePos + 1).toInt();

//       if (steps > 0) {
//         bool forward = (dirChar == 'f');
//         runControllerCommand(controllerNum, forward, steps);
//       } else {
//         printlnBoth("Invalid step count");
//       }
//       return;
//     }
//   }

//   // MOSFET / slave commands
//   if (cmd == "mag on")  { sendToSlave("MAG ON");  return; }
//   if (cmd == "mag off") { sendToSlave("MAG OFF"); return; }

//   if (cmd == "vac on")  { sendToSlave("VAC ON");  return; }
//   if (cmd == "vac off") { sendToSlave("VAC OFF"); return; }

//   if (cmd == "saw on")  { sendToSlave("SAW ON");  return; }
//   if (cmd == "saw off") { sendToSlave("SAW OFF"); return; }

//   if (cmd == "alloff")  { sendToSlave("ALL OFF"); return; }
//   if (cmd == "mstatus") { sendToSlave("STATUS");  return; }

//   printlnBoth("Unknown command. Type 'help'");
// }

// // =========================
// // Input readers
// // =========================
// void readFromSerial() {
//   while (Serial.available()) {
//     char c = (char)Serial.read();

//     if (c == '\r') continue;
//     if (c == '\n') {
//       if (serialBuffer.length() > 0) {
//         handleCommand(serialBuffer);
//         serialBuffer = "";
//       }
//     } else {
//       serialBuffer += c;
//     }
//   }
// }

// void readFromWiFi() {
//   if (!commandClient || !commandClient.connected()) {
//     WiFiClient incoming = CommandServer.available();
//     if (incoming) {
//       if (commandClient && commandClient.connected()) commandClient.stop();
//       commandClient = incoming;
//       commandClient.println("Connected to ME424 main board WiFi command socket.");
//     }
//   }

//   if (commandClient && commandClient.connected()) {
//     while (commandClient.available()) {
//       char c = (char)commandClient.read();
//       if (c == '\r') continue;
//       if (c == '\n') {
//         if (wifiBuffer.length() > 0) {
//           handleCommand(wifiBuffer);
//           wifiBuffer = "";
//         }
//       } else {
//         wifiBuffer += c;
//       }
//     }
//   }
// }

// void readFromSlave() {
//   while (SlaveSerial.available()) {
//     char c = (char)SlaveSerial.read();

//     if (c == '\r') continue;
//     if (c == '\n') {
//       if (slaveBuffer.length() > 0) {
//         printlnBoth("[SLAVE] " + slaveBuffer);
//         slaveBuffer = "";
//       }
//     } else {
//       slaveBuffer += c;
//     }
//   }
// }

// void setup() {
//   for (int i = 0; i < NUM_DRIVERS; i++) {
//     pinMode(STEP_PINS[i], OUTPUT);
//     pinMode(DIR_PINS[i], OUTPUT);
//     digitalWrite(STEP_PINS[i], LOW);
//     digitalWrite(DIR_PINS[i], LOW);
//   }

//   Serial.begin(115200);

//   if (String(WIFI_SSID).length() > 0) {
//     WiFi.mode(WIFI_STA);
//     WiFi.setHostname(WIFI_HOSTNAME);
//     WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//     Serial.print("WiFi connecting");
//     unsigned long t0 = millis();
//     while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
//       delay(250);
//       Serial.print(".");
//     }
//     Serial.println();

//     if (WiFi.status() == WL_CONNECTED) {
//       ArduinoOTA.setHostname(WIFI_HOSTNAME);
//       ArduinoOTA.begin();
//       CommandServer.begin();
//       Serial.println("WiFi connected: " + WiFi.localIP().toString());
//       Serial.println(String("WiFi command port: ") + 3333);

//       if (LittleFS.begin(true)) {
//         // Explicit routes — ESP32 serveStatic often does not map "/" -> index.html.
//         httpServer.on("/", HTTP_GET, []() { httpSendLittleFSFile("/index.html", "text/html"); });
//         httpServer.on("/index.html", HTTP_GET, []() { httpSendLittleFSFile("/index.html", "text/html"); });
//         httpServer.on("/app.js", HTTP_GET, []() { httpSendLittleFSFile("/app.js", "application/javascript"); });
//         httpServer.on("/styles.css", HTTP_GET, []() { httpSendLittleFSFile("/styles.css", "text/css"); });
//         httpServer.onNotFound([]() {
//           httpServer.send(404, "text/plain", "Not found (expected /, /app.js, /styles.css)");
//         });
//         httpServer.begin();
//         httpServerLive = true;
//         Serial.println("Web UI: http://" + WiFi.localIP().toString() + "/");
//         Serial.println(String("LittleFS index.html: ") + (LittleFS.exists("/index.html") ? "yes" : "NO"));
//         Serial.println(String("LittleFS app.js: ") + (LittleFS.exists("/app.js") ? "yes" : "NO"));
//         Serial.println(String("LittleFS styles.css: ") + (LittleFS.exists("/styles.css") ? "yes" : "NO"));
//       } else {
//         Serial.println("LittleFS mount failed (run: pio run -e main_board -t uploadfs)");
//       }

//       webSocket.onEvent(webSocketEvent);
//       webSocket.begin();
//       webSocketLive = true;
//       Serial.println(String("WebSocket commands: ws://") + WiFi.localIP().toString() + ":81/");
//     } else {
//       Serial.println("WiFi connect failed, continuing without WiFi/OTA");
//     }
//   } else {
//     Serial.println("WiFi disabled (set WIFI_SSID/WIFI_PASSWORD build flags)");
//   }

//   SlaveSerial.begin(SLAVE_BAUD, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);
//   SensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

//   delay(1000);

//   printlnBoth("Main board ready");
//   printlnBoth("USB serial + WiFi (TCP 3333 / WebSocket 81) + HTTP UI on /");
//   printlnBoth("UART2 to MOSFET slave ready");
//   printlnBoth("UART1 to sensor board ready");
//   if (WiFi.status() == WL_CONNECTED) {
//     printlnBoth("OTA host: " + String(WIFI_HOSTNAME) + ".local");
//   }
//   printHelp();
// }

// void loop() {
//   readFromSerial();
//   readFromWiFi();
//   readFromSlave();
//   serviceSensorUART();
//   if (httpServerLive) httpServer.handleClient();
//   if (webSocketLive) webSocket.loop();
//   if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
// }
