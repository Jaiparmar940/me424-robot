const AXES = [
  // Ordered by stage number, not controller number.
  { idx: 5, name: 'Stage 1 / Turntable (C6)', plus: 'c6f', minus: 'c6r', homeCmd: 'home s1 bounce' },
  // Stage 2 is physically paired (C1 + C4). Jogging either raw controller now
  // moves both in firmware, so show one unified control here.
  { idx: 0, name: 'Stage 2 / Lift Pair (C1 + C4)', plus: 'c1f', minus: 'c1r', homeCmd: 'home s2 bounce' },
  // Match firmware: s3up/s4up = reverse (c2r/c3r), s3down/s4down = forward (c2f/c3f).
  { idx: 1, name: 'Stage 3 (C2)', plus: 'c2r', minus: 'c2f', homeCmd: 'home s3 bounce' },
  { idx: 2, name: 'Stage 4 (C3)', plus: 'c3r', minus: 'c3f', homeCmd: 'home s4 bounce' },
  { idx: 4, name: 'Stage 5 (C5)', plus: 'c5f', minus: 'c5r', homeCmd: 'home s5 bounce' },
];

const WS_PORT = 81;
/** When the page is opened from python/http.server, WebSocket must target the ESP32, not localhost. */
const DEFAULT_ROBOT_WS_HOST = 'me424-main.local';

let port = null;
let reader = null;
let writer = null;
let readLoopActive = false;
let readBuffer = '';

/** True when either USB serial or Wi‑Fi WebSocket is ready for commands. */
let linkReady = false;
let useWebSocket = false;
let ws = null;
let estopLatchedUI = false;

let currentPos = [0, 0, 0, 0, 0, 0];
let sequence = [];
let running = false;
let stopRequested = false;
let lastManualPulseDelaySent = null;
let nextPosResolvers = [];

/** @type {{ expectedTag: string, timer: ReturnType<typeof setTimeout>, resolve: (v: { ok: boolean, line: string }) => void }[]} */
let fwCompletionWaiters = [];

let liftRoutineRunning = false;

/** Vertical / “go to vertical” pose: C1..C6 tracked counts after homing. */
const VERTICAL_POSE = [7533, -6027, -3874, 7533, 0, -2847];

/** Tracked pose targets for the one-click lift calibration routine (C1/C4 pair, then C2/C3). */
const LIFT_ROUTINE = {
  // Keep in sync with firmware `kAppVerticalPose` / `kLiftRoutine*` in `src/main_board/main.cpp`.
  preHomeS2Steps: 4500,
  postHomeC2: -6000,
  postHomeC3: -5000,
  /** Same array as VERTICAL_POSE; used before bounce block (lift routine also ends there). */
  preBouncePose: VERTICAL_POSE,
  /** After `home s2 bounce`, lift Stage 2 to this height (C1=C4) before s3/s4/s1 bounce. */
  bounceLiftHeightSteps: 4500,
};

const LIFT_ROUTINE_SYNC_TIMEOUT_MS = 300000;

/** Last-known MOSFET outputs (synced from slave STATUS/ACK, optimistic clicks, localStorage). */
const MOSFET_LS_KEY = 'me424_mosfet_outputs_v1';
/** Matches MOSFET slave TOOL_ID: 1=2.2k saw, 2=5k probe, 3=10k vacuum. */
const TOOL_ID_LABELS = [
  { name: 'Unknown', nominal: null },
  { name: 'Saw', nominal: '2.2 kΩ' },
  { name: 'Probe', nominal: '5 kΩ' },
  { name: 'Vacuum', nominal: '10 kΩ' },
];

let mosfetState = {
  mag: false,
  vac: false,
  saw: false,
  /** 0–100; MOSFET board GPIO 22 PWM (applied only while saw on). */
  sawBldcSpeed: 0,
  toolId: 0,
  toolROhms: null,
  compMm: null,
};

let sawBldcSliderSilent = false;
let sawBldcSpeedSendTimer = null;

function loadMosfetState() {
  try {
    const raw = localStorage.getItem(MOSFET_LS_KEY);
    if (!raw) return;
    const o = JSON.parse(raw);
    if (typeof o.mag === 'boolean') mosfetState.mag = o.mag;
    if (typeof o.vac === 'boolean') mosfetState.vac = o.vac;
    if (typeof o.saw === 'boolean') mosfetState.saw = o.saw;
    if (typeof o.sawBldcSpeed === 'number' && o.sawBldcSpeed >= 0 && o.sawBldcSpeed <= 100) {
      mosfetState.sawBldcSpeed = Math.round(o.sawBldcSpeed);
    }
    if (typeof o.toolId === 'number' && o.toolId >= 0 && o.toolId <= 3) mosfetState.toolId = o.toolId;
    if (typeof o.toolROhms === 'number' && o.toolROhms >= 0) mosfetState.toolROhms = o.toolROhms;
    if (typeof o.compMm === 'number' && o.compMm >= 0) mosfetState.compMm = o.compMm;
  } catch (_) {}
}

function saveMosfetState() {
  try {
    localStorage.setItem(
      MOSFET_LS_KEY,
      JSON.stringify({
        mag: mosfetState.mag,
        vac: mosfetState.vac,
        saw: mosfetState.saw,
        sawBldcSpeed: mosfetState.sawBldcSpeed,
        toolId: mosfetState.toolId,
        toolROhms: mosfetState.toolROhms,
        compMm: mosfetState.compMm,
      }),
    );
  } catch (_) {}
}

function updateSawBldcSliderUI() {
  const sl = document.getElementById('sawBldcSpeedSlider');
  const lab = document.getElementById('sawBldcSpeedPct');
  if (!sl || !lab) return;
  sawBldcSliderSilent = true;
  sl.value = String(mosfetState.sawBldcSpeed);
  lab.textContent = String(mosfetState.sawBldcSpeed);
  sawBldcSliderSilent = false;
}

function wireSawBldcSpeedSlider() {
  const sl = document.getElementById('sawBldcSpeedSlider');
  if (!sl || sl.dataset.wired === '1') return;
  sl.dataset.wired = '1';
  sl.addEventListener('input', () => {
    if (sawBldcSliderSilent) return;
    let v = parseInt(sl.value, 10);
    if (Number.isNaN(v)) v = mosfetState.sawBldcSpeed;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mosfetState.sawBldcSpeed = v;
    const lab = document.getElementById('sawBldcSpeedPct');
    if (lab) lab.textContent = String(v);
    saveMosfetState();
    if (!linkReady) return;
    clearTimeout(sawBldcSpeedSendTimer);
    sawBldcSpeedSendTimer = setTimeout(() => {
      sawBldcSpeedSendTimer = null;
      sendLine(`saw speed ${v}`).catch(() => {});
    }, 120);
  });
}

function applySawBldcFromIncomingLine(line) {
  const s = line.replace(/^\[SLAVE\]\s*/i, '').trim();
  const m = s.match(/^ACK\s+SAW\s+SPEED\s+(\d+)$/i);
  if (!m) return;
  let v = parseInt(m[1], 10);
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  mosfetState.sawBldcSpeed = v;
  updateSawBldcSliderUI();
  saveMosfetState();
}

function updateToolReadoutUI() {
  const main = document.getElementById('toolReadoutMain');
  const sub = document.getElementById('toolReadoutSub');
  if (!main) return;
  const id = mosfetState.toolId >= 0 && mosfetState.toolId <= 3 ? mosfetState.toolId : 0;
  const t = TOOL_ID_LABELS[id];
  main.textContent = t.name;
  if (!sub) return;
  const parts = [];
  if (t.nominal) parts.push(`${t.nominal} ID`);
  if (mosfetState.toolROhms != null && mosfetState.toolROhms > 0) {
    parts.push(`measured ~${Math.round(mosfetState.toolROhms)} Ω`);
  }
  if (mosfetState.compMm != null && !Number.isNaN(mosfetState.compMm)) {
    parts.push(`compression ${mosfetState.compMm.toFixed(2)} mm`);
  }
  sub.textContent = parts.join(' · ');
}

function updateMosfetUI() {
  document.querySelectorAll('.mosfet-send[data-channel]').forEach((btn) => {
    const ch = btn.getAttribute('data-channel');
    const wantOn = btn.getAttribute('data-want-on') === 'true';
    if (!ch || !(ch in mosfetState)) return;
    const isOn = mosfetState[ch];
    const active = (wantOn && isOn) || (!wantOn && !isOn);
    btn.classList.toggle('mosfet-active', active);
  });
  const alloff = document.querySelector('.mosfet-send.mosfet-alloff');
  if (alloff) {
    const allOff = !mosfetState.mag && !mosfetState.vac && !mosfetState.saw;
    alloff.classList.toggle('mosfet-active', allOff);
  }
}

/** Update UI immediately; ACK/STATUS lines will confirm. */
function applyMosfetOptimistic(cmd) {
  const c = cmd.trim().toLowerCase();
  const spd = c.match(/^saw speed (\d+)$/);
  if (spd) {
    let v = parseInt(spd[1], 10);
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mosfetState.sawBldcSpeed = v;
    updateSawBldcSliderUI();
    saveMosfetState();
    return;
  }
  if (c === 'mag on') mosfetState.mag = true;
  else if (c === 'mag off') mosfetState.mag = false;
  else if (c === 'vac on') mosfetState.vac = true;
  else if (c === 'vac off') mosfetState.vac = false;
  else if (c === 'saw on') mosfetState.saw = true;
  else if (c === 'saw off') mosfetState.saw = false;
  else if (c === 'alloff') {
    mosfetState.mag = false;
    mosfetState.vac = false;
    mosfetState.saw = false;
  } else return;
  updateMosfetUI();
  saveMosfetState();
}

/**
 * Parse lines from the MOSFET slave relayed as `[SLAVE] ...` on the main board.
 * See `src/mosfet_board/main.cpp` (STATUS / ACK …).
 */
function applyMosfetFromIncomingLine(line) {
  const s = line.replace(/^\[SLAVE\]\s*/i, '').trim();
  let changed = false;

  const st = s.match(/^STATUS\s+MAG=(ON|OFF)\s+VAC=(ON|OFF)\s+SAW=(ON|OFF)/i);
  if (st) {
    mosfetState.mag = st[1].toUpperCase() === 'ON';
    mosfetState.vac = st[2].toUpperCase() === 'ON';
    mosfetState.saw = st[3].toUpperCase() === 'ON';
    changed = true;
    const mSpd = /\bSAW_SPD=(\d+)\b/i.exec(s);
    if (mSpd) {
      let sp = parseInt(mSpd[1], 10);
      if (sp < 0) sp = 0;
      if (sp > 100) sp = 100;
      mosfetState.sawBldcSpeed = sp;
      updateSawBldcSliderUI();
    }
    const mR = /\bTOOL_R=([\d.]+)\b/i.exec(s);
    if (mR) {
      const r = parseFloat(mR[1]);
      if (!Number.isNaN(r) && r >= 0) mosfetState.toolROhms = r;
    }
    const mId = /\bTOOL_ID=(\d+)\b/i.exec(s);
    if (mId) {
      let tid = parseInt(mId[1], 10);
      if (tid < 0) tid = 0;
      if (tid > 3) tid = 3;
      mosfetState.toolId = tid;
    }
    const mC = /\bCOMP_MM=([\d.]+)\b/i.exec(s);
    if (mC) {
      const cm = parseFloat(mC[1]);
      if (!Number.isNaN(cm) && cm >= 0) mosfetState.compMm = cm;
    }
    updateToolReadoutUI();
  }

  const ackMag = s.match(/^ACK\s+MAG\s+(ON|OFF)$/i);
  if (ackMag) {
    mosfetState.mag = ackMag[1].toUpperCase() === 'ON';
    changed = true;
  }
  const ackVac = s.match(/^ACK\s+VAC\s+(ON|OFF)$/i);
  if (ackVac) {
    mosfetState.vac = ackVac[1].toUpperCase() === 'ON';
    changed = true;
  }
  const ackSaw = s.match(/^ACK\s+SAW\s+(ON|OFF)$/i);
  if (ackSaw) {
    mosfetState.saw = ackSaw[1].toUpperCase() === 'ON';
    changed = true;
  }
  if (/^ACK\s+ALL\s+OFF$/i.test(s)) {
    mosfetState.mag = false;
    mosfetState.vac = false;
    mosfetState.saw = false;
    changed = true;
  }

  if (changed) {
    updateMosfetUI();
    saveMosfetState();
  }
}

const els = {
  connectBtn: document.getElementById('connectBtn'),
  disconnectBtn: document.getElementById('disconnectBtn'),
  refreshPosBtn: document.getElementById('refreshPosBtn'),
  connStatus: document.getElementById('connStatus'),
  estopBtn: document.getElementById('estopBtn'),
  estopClearBtn: document.getElementById('estopClearBtn'),
  estopStatus: document.getElementById('estopStatus'),
  stepSize: document.getElementById('stepSize'),
  manualPulseDelay: document.getElementById('manualPulseDelay'),
  jogMaxSps: document.getElementById('jogMaxSps'),
  jogRamp: document.getElementById('jogRamp'),
  axisGrid: document.getElementById('axisGrid'),
  zeroAllBtn: document.getElementById('zeroAllBtn'),
  limitAutohomeBtn: document.getElementById('limitAutohomeBtn'),
  liftAutohomeBtn: document.getElementById('liftAutohomeBtn'),
  goToVerticalBtn: document.getElementById('goToVerticalBtn'),
  recordPoseBtn: document.getElementById('recordPoseBtn'),
  delayMs: document.getElementById('delayMs'),
  addDelayBtn: document.getElementById('addDelayBtn'),
  mosfetAction: document.getElementById('mosfetAction'),
  addMosfetBtn: document.getElementById('addMosfetBtn'),
  homeBounceAction: document.getElementById('homeBounceAction'),
  addHomeBounceBtn: document.getElementById('addHomeBounceBtn'),
  playbackMaxSps: document.getElementById('playbackMaxSps'),
  playbackRamp: document.getElementById('playbackRamp'),
  runSequenceBtn: document.getElementById('runSequenceBtn'),
  stopSequenceBtn: document.getElementById('stopSequenceBtn'),
  clearSeqBtn: document.getElementById('clearSeqBtn'),
  saveLocalBtn: document.getElementById('saveLocalBtn'),
  loadLocalBtn: document.getElementById('loadLocalBtn'),
  exportBtn: document.getElementById('exportBtn'),
  importInput: document.getElementById('importInput'),
  sequenceList: document.getElementById('sequenceList'),
  manualCmd: document.getElementById('manualCmd'),
  sendCmdBtn: document.getElementById('sendCmdBtn'),
  logBox: document.getElementById('logBox'),
};

function log(msg) {
  const ts = new Date().toLocaleTimeString();
  els.logBox.textContent += `[${ts}] ${msg}\n`;
  els.logBox.scrollTop = els.logBox.scrollHeight;
}

function updateEStopUI() {
  if (els.estopStatus) {
    els.estopStatus.textContent = estopLatchedUI ? 'ESTOP: LATCHED' : 'ESTOP: clear';
    els.estopStatus.classList.toggle('is-latched', estopLatchedUI);
  }
  if (els.estopBtn) els.estopBtn.disabled = !linkReady;
  if (els.estopClearBtn) els.estopClearBtn.disabled = !linkReady || !estopLatchedUI;

  const disableMotion = !linkReady || estopLatchedUI;
  const motionIds = ['goToVerticalBtn', 'runSequenceBtn', 'stopSequenceBtn'];
  motionIds.forEach((id) => {
    if (els[id]) els[id].disabled = disableMotion;
  });
  if (els.limitAutohomeBtn) els.limitAutohomeBtn.disabled = disableMotion || liftRoutineRunning;
  if (els.liftAutohomeBtn) els.liftAutohomeBtn.disabled = disableMotion || liftRoutineRunning;
  document.querySelectorAll('[id^="minus_"], [id^="plus_"], [id^="autohome_"]').forEach((b) => {
    b.disabled = disableMotion;
  });
}

function syncEStopFromIncomingLine(line) {
  if (/ESTOP\s+LATCHED/i.test(line) || /ESTOP=1(?:\D|$)/i.test(line)) {
    estopLatchedUI = true;
    stopRequested = true;
    updateEStopUI();
    return;
  }
  if (/ESTOP\s+CLEARED/i.test(line) || /ESTOP=0(?:\D|$)/i.test(line)) {
    estopLatchedUI = false;
    updateEStopUI();
  }
}

function shouldAbortAutomation() {
  return stopRequested || estopLatchedUI;
}

function isMotionCommand(cmd) {
  const c = cmd.trim().toLowerCase();
  if (!c) return false;
  if (/^syncabs\b/.test(c)) return true;
  if (/^(s[1-5](up|down|cw|ccw)\s+\d+)$/.test(c)) return true;
  if (/^(c[1-6][fr]\s+\d+)$/.test(c)) return true;
  if (/^home\s+s[1-5]\b/.test(c)) return true;
  if (/^qrun\b/.test(c)) return true;
  if (/^seq\b/.test(c)) return true;
  if (/^par\b/.test(c)) return true;
  if (/^vertical\b/.test(c)) return true;
  if (/^autohome\b/.test(c)) return true;
  return false;
}

function setConnectedUI(connected) {
  els.connectBtn.disabled = connected;
  els.disconnectBtn.disabled = !connected;
  els.refreshPosBtn.disabled = !connected;
  els.zeroAllBtn.disabled = !connected;
  if (els.limitAutohomeBtn) els.limitAutohomeBtn.disabled = !connected || liftRoutineRunning;
  if (els.liftAutohomeBtn) els.liftAutohomeBtn.disabled = !connected || liftRoutineRunning;
  if (els.goToVerticalBtn) els.goToVerticalBtn.disabled = !connected;
  els.recordPoseBtn.disabled = !connected;
  els.addDelayBtn.disabled = !connected;
  if (els.addHomeBounceBtn) els.addHomeBounceBtn.disabled = !connected;
  els.addMosfetBtn.disabled = !connected;
  els.runSequenceBtn.disabled = !connected;
  els.sendCmdBtn.disabled = !connected;
  els.connStatus.textContent = connected ? 'Connected' : 'Disconnected';
  document.querySelectorAll('.mosfet-send').forEach((b) => {
    b.disabled = !connected;
  });
  const bldcSl = document.getElementById('sawBldcSpeedSlider');
  if (bldcSl) bldcSl.disabled = !connected;
  updateEStopUI();
}

function preferWebSocket() {
  const q = new URLSearchParams(window.location.search);
  if (q.get('usb') === '1') return false;
  if (window.location.protocol === 'file:') return false;
  return true;
}

/** Hostname or IP for ws://…:81 (not the HTTP page origin). */
function robotWebSocketHost() {
  const q = new URLSearchParams(window.location.search);
  const fromQuery = q.get('host');
  if (fromQuery) return fromQuery.trim();
  const pageHost = window.location.hostname;
  // If the UI was opened from the robot (or any non-localhost page), WebSocket must target
  // that same host — not a stale me424_robot_host from an old python://localhost session.
  if (pageHost && pageHost !== 'localhost' && pageHost !== '127.0.0.1') {
    return pageHost;
  }
  const inp = document.getElementById('robotHost');
  if (inp && inp.value.trim()) return inp.value.trim();
  const stored = localStorage.getItem('me424_robot_host');
  if (stored) return stored.trim();
  return DEFAULT_ROBOT_WS_HOST;
}

function renderAxes() {
  els.axisGrid.innerHTML = '';
  AXES.forEach((a) => {
    const wrap = document.createElement('div');
    wrap.className = 'axis';
    wrap.innerHTML = `
      <h3>${a.name}</h3>
      <div class="pos" id="pos_${a.idx}">pos=${currentPos[a.idx]}</div>
      <div class="row">
        <button id="minus_${a.idx}" ${!linkReady ? 'disabled' : ''}>- Jog</button>
        <button id="plus_${a.idx}" ${!linkReady ? 'disabled' : ''}>+ Jog</button>
        <button id="zero_${a.idx}" ${!linkReady ? 'disabled' : ''}>Zero This Axis</button>
      </div>
      <div class="row">
        <button id="autohome_${a.idx}" class="btn-home" ${!linkReady ? 'disabled' : ''}>Auto home (${a.homeCmd})</button>
      </div>
    `;
    els.axisGrid.appendChild(wrap);

    document.getElementById(`minus_${a.idx}`).onclick = () => jogAxis(a, false);
    document.getElementById(`plus_${a.idx}`).onclick = () => jogAxis(a, true);
    document.getElementById(`zero_${a.idx}`).onclick = () => zeroAxis(a.idx);
    document.getElementById(`autohome_${a.idx}`).onclick = () => autoHomeStage(a.homeCmd);
  });
}

function updatePosUI() {
  AXES.forEach((a) => {
    const el = document.getElementById(`pos_${a.idx}`);
    if (el) el.textContent = `pos=${currentPos[a.idx]}`;
  });
}

async function connectWebSocket() {
  if (linkReady && useWebSocket) return;
  if (port) await disconnectSerial();

  const host = robotWebSocketHost();
  const url = `ws://${host}:${WS_PORT}/`;
  return new Promise((resolve, reject) => {
    try {
      ws = new WebSocket(url);
    } catch (e) {
      reject(e);
      return;
    }
    ws.onopen = () => {
      useWebSocket = true;
      linkReady = true;
      setConnectedUI(true);
      renderAxes();
      log(`WebSocket connected ${url}`);
      requestWhere()
        .then(() => sendLine('mstatus').catch(() => {}))
        .then(() => sendLine('estop status').catch(() => {}))
        .then(resolve)
        .catch(resolve);
    };
    ws.onmessage = (ev) => {
      const t = typeof ev.data === 'string' ? ev.data : '';
      const parts = t.split(/\r?\n/);
      for (const line of parts) {
        const trimmed = line.trim();
        if (trimmed) handleIncoming(trimmed);
      }
    };
    ws.onerror = () => log(`WebSocket error (check host "${host}" and port ${WS_PORT})`);
    ws.onclose = () => {
      linkReady = false;
      useWebSocket = false;
      ws = null;
      setConnectedUI(false);
      renderAxes();
      log('WebSocket disconnected');
    };
  });
}

async function connectSerial() {
  if (!('serial' in navigator)) {
    alert('Web Serial is not supported in this browser. Use Chrome/Edge desktop.');
    return;
  }
  if (ws) {
    ws.close();
    ws = null;
    useWebSocket = false;
    linkReady = false;
  }
  port = await navigator.serial.requestPort();
  await port.open({ baudRate: 115200 });

  const encoder = new TextEncoderStream();
  encoder.readable.pipeTo(port.writable);
  writer = encoder.writable.getWriter();

  const decoder = new TextDecoderStream();
  port.readable.pipeTo(decoder.writable);
  reader = decoder.readable.getReader();

  readLoopActive = true;
  readLoop();

  linkReady = true;
  useWebSocket = false;
  setConnectedUI(true);
  renderAxes();
  log('Connected serial at 115200');
  await requestWhere();
  await sendLine('mstatus').catch(() => {});
  await sendLine('estop status').catch(() => {});
}

async function disconnectSerial() {
  try {
    readLoopActive = false;
    if (reader) {
      await reader.cancel();
      reader.releaseLock();
      reader = null;
    }
    if (writer) {
      writer.releaseLock();
      writer = null;
    }
    if (port) {
      await port.close();
      port = null;
    }
    linkReady = false;
    setConnectedUI(false);
    renderAxes();
    log('Disconnected');
  } catch (e) {
    log(`Disconnect error: ${e.message}`);
  }
}

async function connectTransport() {
  if (preferWebSocket()) {
    await connectWebSocket();
  } else {
    await connectSerial();
  }
}

async function disconnectTransport() {
  if (useWebSocket && ws) {
    ws.close();
    ws = null;
    useWebSocket = false;
    linkReady = false;
    setConnectedUI(false);
    renderAxes();
    log('Disconnected');
    return;
  }
  await disconnectSerial();
}

async function readLoop() {
  while (readLoopActive && reader) {
    try {
      const { value, done } = await reader.read();
      if (done) break;
      if (!value) continue;

      readBuffer += value;
      let idx;
      while ((idx = readBuffer.indexOf('\n')) >= 0) {
        const line = readBuffer.slice(0, idx).replace(/\r/g, '').trim();
        readBuffer = readBuffer.slice(idx + 1);
        if (line) handleIncoming(line);
      }
    } catch (e) {
      log(`Read error: ${e.message}`);
      break;
    }
  }
}

function notifyFwCompletionWaiters(line) {
  const u = line.trim();
  for (let i = 0; i < fwCompletionWaiters.length; i++) {
    const w = fwCompletionWaiters[i];
    if (u === `DONE ${w.expectedTag}`) {
      clearTimeout(w.timer);
      fwCompletionWaiters.splice(i, 1);
      w.resolve({ ok: true, line: u });
      return;
    }
    if (u.startsWith(`ERR ${w.expectedTag} `)) {
      clearTimeout(w.timer);
      fwCompletionWaiters.splice(i, 1);
      w.resolve({ ok: false, line: u });
      return;
    }
  }
}

function handleIncoming(line) {
  log(`< ${line}`);

  applyMosfetFromIncomingLine(line);
  applySawBldcFromIncomingLine(line);
  syncEStopFromIncomingLine(line);
  notifyFwCompletionWaiters(line);

  // Example: POS C1=0 C2=0 C3=0 C4=0 C5=0 C6=0
  const m = line.match(/^POS\s+C1=(-?\d+)\s+C2=(-?\d+)\s+C3=(-?\d+)\s+C4=(-?\d+)\s+C5=(-?\d+)\s+C6=(-?\d+)$/i);
  if (m) {
    currentPos = [1,2,3,4,5,6].map((i) => parseInt(m[i], 10));
    updatePosUI();
    if (nextPosResolvers.length > 0) {
      const resolvers = [...nextPosResolvers];
      nextPosResolvers = [];
      resolvers.forEach((r) => r([...currentPos]));
    }
  }
}

async function sendLine(cmd) {
  if (!linkReady) throw new Error('Not connected');
  const clean = cmd.trim();
  if (!clean) return;
  if (estopLatchedUI && isMotionCommand(clean)) {
    log(`! blocked by ESTOP: ${clean}`);
    throw new Error('ESTOP latched: clear ESTOP to allow motion');
  }
  log(`> ${clean}`);
  if (useWebSocket && ws && ws.readyState === WebSocket.OPEN) {
    ws.send(`${clean}\n`);
    return;
  }
  if (!writer) throw new Error('Not connected');
  await writer.write(`${clean}\n`);
}

async function requestWhere() {
  await sendLine('where');
}

function waitForFirmwareCompletion(expectedTag, timeoutMs) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      fwCompletionWaiters = fwCompletionWaiters.filter((w) => w !== entry);
      reject(new Error(`Timeout waiting for firmware (${expectedTag})`));
    }, timeoutMs);
    const entry = { expectedTag, timer, resolve };
    fwCompletionWaiters.push(entry);
  });
}

async function refreshPositionFromRobot(timeoutMs = 1500) {
  await requestWhere();
  currentPos = await waitForNextPositionUpdate(timeoutMs);
  updatePosUI();
}

function waitForNextPositionUpdate(timeoutMs = 1500) {
  return new Promise((resolve, reject) => {
    const resolver = (pos) => {
      clearTimeout(timer);
      resolve(pos);
    };
    const timer = setTimeout(() => {
      nextPosResolvers = nextPosResolvers.filter((r) => r !== resolver);
      reject(new Error('Timed out waiting for position update'));
    }, timeoutMs);
    nextPosResolvers.push(resolver);
  });
}

async function jogAxis(axis, positive) {
  if (!linkReady) return;
  const steps = Math.max(1, parseInt(els.stepSize.value || '200', 10));
  const manualPulseDelay = Math.max(100, parseInt(els.manualPulseDelay.value || '1200', 10));

  // Keep manual jogging smooth by applying a manual pulse delay.
  // Only resend when changed to reduce serial chatter.
  if (lastManualPulseDelaySent !== manualPulseDelay) {
    await sendLine(`speed ${manualPulseDelay}`);
    lastManualPulseDelaySent = manualPulseDelay;
  }

  const cmd = `${positive ? axis.plus : axis.minus} ${steps}`;
  await sendLine(cmd);
  await sleep(50);
  await requestWhere();
}

async function zeroAxis(idx) {
  const p = [...currentPos];
  p[idx] = 0;
  await sendLine(`setpos ${p.join(' ')}`);
  currentPos = p;
  updatePosUI();
}

async function zeroAll() {
  await sendLine('setpos 0 0 0 0 0 0');
  currentPos = [0, 0, 0, 0, 0, 0];
  updatePosUI();
}

/** Firmware homing: limits for S2–S4, Hall S5H for S5, soft zero for S1. */
const HOME_CMD_TIMEOUT_MS = 30000;

async function autoHomeStage(homeCmd) {
  if (shouldAbortAutomation()) return;
  await sendLine(homeCmd);
  if (shouldAbortAutomation()) return;
  await requestWhere();
  try {
    currentPos = await waitForNextPositionUpdate(HOME_CMD_TIMEOUT_MS);
    updatePosUI();
  } catch (e) {
    log(`Auto home: ${e.message}`);
  }
}


async function syncAbsAndWait(targetsSix, maxSps, ramp, timeoutMs) {
  await sendLine(`syncabs ${targetsSix.join(' ')} ${maxSps} ${ramp}`);
  const r = await waitForFirmwareCompletion('syncabs', timeoutMs);
  if (!r.ok) throw new Error(r.line);
  await refreshPositionFromRobot(timeoutMs);
}

/** Firmware `setpos 0…0` + wait for DONE; sync UI (vertical / “here” becomes origin). */
async function zeroTrackedAtCurrentPose(timeoutMs = 10000) {
  await sendLine('setpos 0 0 0 0 0 0');
  const r = await waitForFirmwareCompletion('setpos', timeoutMs);
  if (!r.ok) throw new Error(r.line);
  currentPos = [0, 0, 0, 0, 0, 0];
  updatePosUI();
}

async function homeStageAndWait(cmd, timeoutMs) {
  await sendLine(cmd);
  const r = await waitForFirmwareCompletion(cmd, timeoutMs);
  if (!r.ok) throw new Error(r.line);
  await refreshPositionFromRobot(timeoutMs);
}

/** `syncabs` to VERTICAL_POSE using Jog Speed / Ramp from the manual section. */
async function goToVerticalPose() {
  if (!linkReady) return;
  const maxSps = Math.max(10, parseInt(els.jogMaxSps.value || '1200', 10));
  const ramp = Math.max(1, parseInt(els.jogRamp.value || '150', 10));
  log(`Go to vertical: syncabs [${VERTICAL_POSE.join(', ')}]…`);
  await syncAbsAndWait([...VERTICAL_POSE], maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);
  log('Go to vertical: done.');
}

/**
 * One-click lift calibration: pre-position Stage 2, home S1+S3+S4, park S3/S4, re-home S2,
 * park to preBouncePose, bounce-home S2 → lift 4500 → bounce-home S3, S4, S1, S5 → vertical pose → setpos 0.
 */
async function runLiftAutohomeRoutine() {
  if (!linkReady || liftRoutineRunning) return;
  if (estopLatchedUI) {
    alert('Clear ESTOP before running the lift routine.');
    return;
  }
  liftRoutineRunning = true;
  stopRequested = false;
  updateEStopUI();

  const maxSps = Math.max(10, parseInt(els.jogMaxSps.value || '1200', 10));
  const ramp = Math.max(1, parseInt(els.jogRamp.value || '150', 10));
  const pb = LIFT_ROUTINE.preBouncePose;

  try {
    log('Lift routine: (1/14) refresh position…');
    await refreshPositionFromRobot(3000);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(`Lift routine: (2/14) Stage 2 → C1=C4=${LIFT_ROUTINE.preHomeS2Steps} (syncabs)…`);
    const pre = [...currentPos];
    pre[0] = LIFT_ROUTINE.preHomeS2Steps;
    pre[3] = LIFT_ROUTINE.preHomeS2Steps;
    await syncAbsAndWait(pre, maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (3/14) home s1, home s3, home s4 (sequential, all zeros updated)…');
    for (const h of ['home s1', 'home s3', 'home s4']) {
      if (shouldAbortAutomation()) throw new Error('Cancelled');
      await homeStageAndWait(h, HOME_CMD_TIMEOUT_MS);
    }

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(
      `Lift routine: (4/14) syncabs C2=${LIFT_ROUTINE.postHomeC2} C3=${LIFT_ROUTINE.postHomeC3} (simultaneous vs new zero)…`,
    );
    const park = [...currentPos];
    park[1] = LIFT_ROUTINE.postHomeC2;
    park[2] = LIFT_ROUTINE.postHomeC3;
    await syncAbsAndWait(park, maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (5/14) home s2 (re-zero lift)…');
    await homeStageAndWait('home s2', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(`Lift routine: (6/14) syncabs pre-bounce pose [${pb.join(', ')}]…`);
    await syncAbsAndWait([...pb], maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (7/14) home s2 bounce…');
    await homeStageAndWait('home s2 bounce', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(
      `Lift routine: (8/14) Stage 2 up → C1=C4=${LIFT_ROUTINE.bounceLiftHeightSteps} (syncabs)…`,
    );
    const liftUp = [...currentPos];
    liftUp[0] = LIFT_ROUTINE.bounceLiftHeightSteps;
    liftUp[3] = LIFT_ROUTINE.bounceLiftHeightSteps;
    await syncAbsAndWait(liftUp, maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (9/14) home s3 bounce…');
    await homeStageAndWait('home s3 bounce', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (10/14) home s4 bounce…');
    await homeStageAndWait('home s4 bounce', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (11/14) home s1 bounce…');
    await homeStageAndWait('home s1 bounce', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (12/14) home s5 bounce…');
    await homeStageAndWait('home s5 bounce', HOME_CMD_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(`Lift routine: (13/14) syncabs vertical [${pb.join(', ')}]…`);
    await syncAbsAndWait([...VERTICAL_POSE], maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Lift routine: (14/14) set tracked origin at vertical (setpos 0…0)…');
    await zeroTrackedAtCurrentPose(10000);

    log('Lift routine: complete.');
  } catch (e) {
    log(`Lift routine error: ${e.message}`);
    alert(`Lift autohome routine failed:\n${e.message}`);
  } finally {
    liftRoutineRunning = false;
    updateEStopUI();
  }
}

/**
 * Limit autohome: single bounce pass for all stages, no vertical pose, no re-zero.
 * Pre-position S2 → bounce-home S1, S3, S4 → park S3/S4 → bounce-home S2 → bounce-home S5.
 */
async function runLimitAutohomeRoutine() {
  if (!linkReady || liftRoutineRunning) return;
  if (estopLatchedUI) {
    alert('Clear ESTOP before running the limit autohome.');
    return;
  }
  liftRoutineRunning = true;
  stopRequested = false;
  updateEStopUI();

  const maxSps = Math.max(10, parseInt(els.jogMaxSps.value || '1200', 10));
  const ramp = Math.max(1, parseInt(els.jogRamp.value || '150', 10));

  try {
    log('Limit autohome: (1/9) refresh position…');
    await refreshPositionFromRobot(3000);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(`Limit autohome: (2/9) Stage 2 → C1=C4=${LIFT_ROUTINE.preHomeS2Steps} (syncabs)…`);
    const pre = [...currentPos];
    pre[0] = LIFT_ROUTINE.preHomeS2Steps;
    pre[3] = LIFT_ROUTINE.preHomeS2Steps;
    await syncAbsAndWait(pre, maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Limit autohome: (3/9) home s1 bounce…');
    await homeStageAndWait('home s1 bounce', HOME_CMD_TIMEOUT_MS);
    log(`  → pos after S1: [${currentPos.join(', ')}]`);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Limit autohome: (4/9) home s3 bounce…');
    await homeStageAndWait('home s3 bounce', HOME_CMD_TIMEOUT_MS);
    log(`  → pos after S3: [${currentPos.join(', ')}]`);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Limit autohome: (5/9) home s4 bounce…');
    await homeStageAndWait('home s4 bounce', HOME_CMD_TIMEOUT_MS);
    log(`  → pos after S4: [${currentPos.join(', ')}]`);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log(
      `Limit autohome: (6/9) park S3/S4 → C2=${LIFT_ROUTINE.postHomeC2} C3=${LIFT_ROUTINE.postHomeC3} (syncabs)…`,
    );
    const park = [...currentPos];
    park[1] = LIFT_ROUTINE.postHomeC2;
    park[2] = LIFT_ROUTINE.postHomeC3;
    await syncAbsAndWait(park, maxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Limit autohome: (7/9) home s2 bounce…');
    await homeStageAndWait('home s2 bounce', HOME_CMD_TIMEOUT_MS);
    log(`  → pos after S2: [${currentPos.join(', ')}]`);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    log('Limit autohome: (8/9) home s5 bounce…');
    await homeStageAndWait('home s5 bounce', HOME_CMD_TIMEOUT_MS);
    log(`  → pos after S5: [${currentPos.join(', ')}]`);

    if (shouldAbortAutomation()) throw new Error('Cancelled');

    const limitEndPose = [3400, -6000, -3200, 3400, 0, -3150];
    const endMaxSps = Math.max(10, Math.floor(maxSps / 2));
    log(
      `Limit autohome: (9/9) syncabs to [${limitEndPose.join(', ')}] (half speed: maxSps=${endMaxSps})…`,
    );
    await syncAbsAndWait(limitEndPose, endMaxSps, ramp, LIFT_ROUTINE_SYNC_TIMEOUT_MS);

    log('Limit autohome: complete.');
  } catch (e) {
    log(`Limit autohome error: ${e.message}`);
    alert(`Limit autohome routine failed:\n${e.message}`);
  } finally {
    liftRoutineRunning = false;
    updateEStopUI();
  }
}

function addSequenceItem(item) {
  sequence.push(item);
  renderSequence();
}

/** Effective syncabs speed/ramp for a pose step (per-step or playback panel). */
function resolvePoseMotion(item) {
  const gMax = Math.max(10, parseInt(els.playbackMaxSps.value || '1000', 10));
  const gRamp = Math.max(1, parseInt(els.playbackRamp.value || '200', 10));
  return {
    maxSps:
      typeof item.maxSps === 'number' && !Number.isNaN(item.maxSps) && item.maxSps >= 10
        ? item.maxSps
        : gMax,
    ramp:
      typeof item.ramp === 'number' && !Number.isNaN(item.ramp) && item.ramp >= 1
        ? item.ramp
        : gRamp,
  };
}

function poseMotionSourceLabel(item) {
  const hasSps = typeof item.maxSps === 'number' && !Number.isNaN(item.maxSps) && item.maxSps >= 10;
  const hasRamp = typeof item.ramp === 'number' && !Number.isNaN(item.ramp) && item.ramp >= 1;
  if (hasSps && hasRamp) return '';
  if (!hasSps && !hasRamp) return ' (playback)';
  return ' (mixed: unset uses playback)';
}

function renderSequence() {
  els.sequenceList.innerHTML = '';
  sequence.forEach((item, i) => {
    const li = document.createElement('li');
    const row = document.createElement('div');
    row.className = 'seq-item-row';
    const txt = document.createElement('span');
    txt.className = 'seq-text';

    if (item.type === 'pose') {
      const m = resolvePoseMotion(item);
      txt.textContent = `#${i} POSE [${item.pos.join(', ')}] @ ${m.maxSps} sps, ramp ${m.ramp}${poseMotionSourceLabel(item)}`;
    } else if (item.type === 'delay') {
      txt.textContent = `#${i} DELAY ${item.ms}ms`;
    } else if (item.type === 'mosfet') {
      txt.textContent = `#${i} MOSFET ${item.cmd}`;
    } else if (item.type === 'home') {
      txt.textContent = `#${i} HOME ${item.cmd}`;
    } else {
      txt.textContent = `#${i} ${JSON.stringify(item)}`;
    }

    const actions = document.createElement('span');
    actions.className = 'seq-actions';

    const upBtn = document.createElement('button');
    upBtn.textContent = 'Up';
    upBtn.disabled = i === 0;
    upBtn.onclick = () => moveSequenceItem(i, -1);

    const downBtn = document.createElement('button');
    downBtn.textContent = 'Down';
    downBtn.disabled = i === sequence.length - 1;
    downBtn.onclick = () => moveSequenceItem(i, 1);

    const editBtn = document.createElement('button');
    editBtn.textContent = 'Edit';
    editBtn.onclick = () => editSequenceItem(i);

    const goBtn = document.createElement('button');
    goBtn.textContent = 'Go';
    goBtn.disabled = (item.type !== 'pose' && item.type !== 'home') || !linkReady;
    goBtn.onclick = () => runSingleSequenceItem(i);

    const delBtn = document.createElement('button');
    delBtn.textContent = 'Delete';
    delBtn.onclick = () => deleteSequenceItem(i);

    actions.appendChild(upBtn);
    actions.appendChild(downBtn);
    actions.appendChild(editBtn);
    actions.appendChild(goBtn);
    actions.appendChild(delBtn);

    row.appendChild(txt);
    row.appendChild(actions);
    li.appendChild(row);
    els.sequenceList.appendChild(li);
  });
}

function moveSequenceItem(index, delta) {
  const ni = index + delta;
  if (ni < 0 || ni >= sequence.length) return;
  const temp = sequence[index];
  sequence[index] = sequence[ni];
  sequence[ni] = temp;
  renderSequence();
}

function deleteSequenceItem(index) {
  sequence.splice(index, 1);
  renderSequence();
}

function editSequenceItem(index) {
  const item = sequence[index];
  if (!item) return;

  if (item.type === 'delay') {
    const val = prompt('Delay ms:', String(item.ms));
    if (val === null) return;
    const ms = parseInt(val, 10);
    if (Number.isNaN(ms) || ms < 0) return alert('Invalid delay value.');
    item.ms = ms;
    renderSequence();
    return;
  }

  if (item.type === 'mosfet') {
    const val = prompt('MOSFET command:', item.cmd);
    if (val === null) return;
    const cmd = val.trim().toLowerCase();
    if (!cmd) return alert('Command cannot be empty.');
    item.cmd = cmd;
    renderSequence();
    return;
  }

  if (item.type === 'pose') {
    const ms = typeof item.maxSps === 'number' && item.maxSps >= 10 ? String(item.maxSps) : '';
    const rm = typeof item.ramp === 'number' && item.ramp >= 1 ? String(item.ramp) : '';
    const defaultLine = `${item.pos.join(',')} | ${ms} | ${rm}`;
    const hint =
      '6 positions, then optional " | maxSps | ramp" (each optional; empty = use playback for that field).';
    const val = prompt(`${hint}\nExample: 0,0,0,0,0,0 | 800 | 150`, defaultLine);
    if (val === null) return;
    const segs = val.split('|').map((s) => s.trim());
    const poseParts = segs[0].split(',').map((s) => parseInt(s.trim(), 10));
    if (poseParts.length !== 6 || poseParts.some((n) => Number.isNaN(n))) {
      return alert('Invalid pose. First segment must be exactly 6 integers.');
    }
    item.pos = poseParts;
    if (segs.length >= 2 && segs[1] !== '') {
      const ms = parseInt(segs[1], 10);
      if (Number.isNaN(ms) || ms < 10) return alert('maxSps must be an integer ≥ 10.');
      item.maxSps = ms;
    } else {
      delete item.maxSps;
    }
    if (segs.length >= 3 && segs[2] !== '') {
      const rm = parseInt(segs[2], 10);
      if (Number.isNaN(rm) || rm < 1) return alert('ramp must be an integer ≥ 1.');
      item.ramp = rm;
    } else {
      delete item.ramp;
    }
    renderSequence();
  }

  if (item.type === 'home') {
    const val = prompt('Bounce home command (e.g. home s3 bounce):', item.cmd);
    if (val === null) return;
    const cmd = val.trim().toLowerCase();
    if (!/^home s[1-5] bounce$/.test(cmd)) {
      return alert('Use: home s1 bounce … home s5 bounce');
    }
    item.cmd = cmd;
    renderSequence();
  }
}

async function runSingleSequenceItem(index) {
  const item = sequence[index];
  if (!item || !linkReady) {
    alert('Connect to the controller first.');
    return;
  }
  if (running) {
    alert('A sequence is currently running.');
    return;
  }

  try {
    if (item.type === 'pose') {
      const { maxSps, ramp } = resolvePoseMotion(item);
      await sendLine(`syncabs ${item.pos.join(' ')} ${maxSps} ${ramp}`);
      const r = await waitForFirmwareCompletion('syncabs', 300000);
      if (!r.ok) throw new Error(r.line);
      await refreshPositionFromRobot(300000);
    } else if (item.type === 'home') {
      log(`Sequence step: ${item.cmd}`);
      await homeStageAndWait(item.cmd, HOME_CMD_TIMEOUT_MS);
    }
  } catch (e) {
    log(`Sequence step error: ${e.message}`);
  }
}

async function recordPose() {
  await requestWhere();
  const maxSps = Math.max(10, parseInt(els.playbackMaxSps.value || '1000', 10));
  const ramp = Math.max(1, parseInt(els.playbackRamp.value || '200', 10));
  try {
    const fresh = await waitForNextPositionUpdate();
    addSequenceItem({ type: 'pose', pos: [...fresh], maxSps, ramp });
  } catch (e) {
    log(`Record pose fallback: ${e.message}`);
    addSequenceItem({ type: 'pose', pos: [...currentPos], maxSps, ramp });
  }
}

function addDelay() {
  const ms = Math.max(0, parseInt(els.delayMs.value || '0', 10));
  addSequenceItem({ type: 'delay', ms });
}

function addMosfet() {
  addSequenceItem({ type: 'mosfet', cmd: els.mosfetAction.value });
}

function addHomeBounceToSequence() {
  const sel = els.homeBounceAction;
  const cmd = (sel && sel.value ? sel.value : '').trim().toLowerCase();
  if (!/^home s[1-5] bounce$/.test(cmd)) return;
  addSequenceItem({ type: 'home', cmd });
}

async function runSequence() {
  if (running) return;
  if (estopLatchedUI) {
    alert('ESTOP is latched. Clear ESTOP before running a sequence.');
    return;
  }
  if (!sequence.length) {
    alert('Sequence is empty.');
    return;
  }

  running = true;
  stopRequested = false;
  els.stopSequenceBtn.disabled = false;
  els.runSequenceBtn.disabled = true;

  try {
    for (const item of sequence) {
      if (shouldAbortAutomation()) break;

      if (item.type === 'pose') {
        const { maxSps, ramp } = resolvePoseMotion(item);
        await sendLine(`syncabs ${item.pos.join(' ')} ${maxSps} ${ramp}`);
        const r = await waitForFirmwareCompletion('syncabs', 300000);
        if (!r.ok) throw new Error(r.line);
        await refreshPositionFromRobot(300000);
      } else if (item.type === 'delay') {
        await sleep(item.ms);
      } else if (item.type === 'mosfet') {
        applyMosfetOptimistic(item.cmd);
        await sendLine(item.cmd);
      } else if (item.type === 'home') {
        log(`Run sequence: ${item.cmd}`);
        await homeStageAndWait(item.cmd, HOME_CMD_TIMEOUT_MS);
      }
    }
  } catch (e) {
    log(`Run error: ${e.message}`);
  } finally {
    running = false;
    els.stopSequenceBtn.disabled = true;
    els.runSequenceBtn.disabled = !linkReady;
    log(stopRequested ? 'Sequence stopped.' : 'Sequence complete.');
  }
}

function stopSequence() {
  stopRequested = true;
  estopLatchedUI = true;
  updateEStopUI();
  sendLine('estop').catch((e) => log(`ESTOP send failed: ${e.message}`));
}

function saveLocal() {
  localStorage.setItem('me424_sequence_v1', JSON.stringify(sequence));
  log('Sequence saved to localStorage');
}

function loadLocal() {
  const raw = localStorage.getItem('me424_sequence_v1');
  if (!raw) {
    alert('No saved sequence found.');
    return;
  }
  try {
    sequence = JSON.parse(raw);
    renderSequence();
    log('Sequence loaded from localStorage');
  } catch (e) {
    alert(`Invalid saved data: ${e.message}`);
  }
}

function exportJson() {
  const blob = new Blob([JSON.stringify(sequence, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'me424-sequence.json';
  a.click();
  URL.revokeObjectURL(url);
}

function importJson(file) {
  const r = new FileReader();
  r.onload = () => {
    try {
      sequence = JSON.parse(String(r.result));
      renderSequence();
      log('Sequence imported');
    } catch (e) {
      alert(`Invalid JSON: ${e.message}`);
    }
  };
  r.readAsText(file);
}

function sleep(ms) {
  return new Promise((res) => setTimeout(res, ms));
}

function wireEvents() {
  wireSawBldcSpeedSlider();
  els.connectBtn.onclick = async () => {
    try { await connectTransport(); } catch (e) { log(`Connect error: ${e.message}`); }
  };
  els.disconnectBtn.onclick = disconnectTransport;
  els.refreshPosBtn.onclick = requestWhere;
  els.zeroAllBtn.onclick = zeroAll;
  if (els.limitAutohomeBtn) els.limitAutohomeBtn.onclick = () => runLimitAutohomeRoutine();
  if (els.liftAutohomeBtn) els.liftAutohomeBtn.onclick = () => runLiftAutohomeRoutine();
  if (els.goToVerticalBtn) {
    els.goToVerticalBtn.onclick = async () => {
      try {
        await goToVerticalPose();
      } catch (e) {
        log(`Go to vertical: ${e.message}`);
        alert(`Go to vertical failed:\n${e.message}`);
      }
    };
  }
  els.recordPoseBtn.onclick = recordPose;
  els.addDelayBtn.onclick = addDelay;
  els.addMosfetBtn.onclick = addMosfet;
  if (els.addHomeBounceBtn) els.addHomeBounceBtn.onclick = addHomeBounceToSequence;
  els.runSequenceBtn.onclick = runSequence;
  els.stopSequenceBtn.onclick = stopSequence;
  if (els.estopBtn) {
    els.estopBtn.onclick = async () => {
      stopRequested = true;
      estopLatchedUI = true;
      updateEStopUI();
      try {
        await sendLine('estop');
      } catch (e) {
        log(`ESTOP send failed: ${e.message}`);
      }
    };
  }
  if (els.estopClearBtn) {
    els.estopClearBtn.onclick = async () => {
      try {
        await sendLine('estop clear');
        stopRequested = false;
      } catch (e) {
        log(`ESTOP clear failed: ${e.message}`);
      }
    };
  }
  els.clearSeqBtn.onclick = () => { sequence = []; renderSequence(); };
  els.saveLocalBtn.onclick = saveLocal;
  els.loadLocalBtn.onclick = loadLocal;
  els.exportBtn.onclick = exportJson;
  els.importInput.onchange = (e) => {
    const f = e.target.files?.[0];
    if (f) importJson(f);
  };
  els.sendCmdBtn.onclick = async () => {
    const cmd = els.manualCmd.value.trim();
    els.manualCmd.value = '';
    if (/^(mag|vac|saw)\s+(on|off)$/i.test(cmd) || /^alloff$/i.test(cmd)) {
      applyMosfetOptimistic(cmd.toLowerCase());
    } else if (/^saw speed \d+$/i.test(cmd)) {
      applyMosfetOptimistic(cmd.toLowerCase());
    }
    await sendLine(cmd);
  };

  document.querySelectorAll('.mosfet-send').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const cmd = btn.getAttribute('data-cmd');
      if (!cmd || !linkReady) return;
      if (cmd !== 'mstatus') applyMosfetOptimistic(cmd.toLowerCase());
      try {
        await sendLine(cmd);
      } catch (e) {
        log(`MOSFET: ${e.message}`);
      }
    });
  });
}

function init() {
  loadMosfetState();
  updateMosfetUI();
  updateToolReadoutUI();
  updateSawBldcSliderUI();
  updateEStopUI();

  const rh = document.getElementById('robotHost');
  if (rh) {
    const pageHost = window.location.hostname;
    if (pageHost && pageHost !== 'localhost' && pageHost !== '127.0.0.1') {
      rh.value = pageHost;
    } else {
      const s = localStorage.getItem('me424_robot_host');
      if (s) rh.value = s;
    }
    rh.addEventListener('change', () => {
      localStorage.setItem('me424_robot_host', rh.value.trim());
    });
  }
  wireEvents();
  if (preferWebSocket()) {
    els.connectBtn.textContent = 'Connect (Wi‑Fi)';
    renderAxes();
    renderSequence();
    connectWebSocket().catch(() => {});
  } else {
    els.connectBtn.textContent = 'Connect (USB Serial)';
    renderAxes();
    renderSequence();
    setConnectedUI(false);
  }
}

init();
