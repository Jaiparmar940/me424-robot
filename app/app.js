const AXES = [
  { idx: 0, name: 'C1 / Stage2 Right', plus: 'c1f', minus: 'c1r' },
  { idx: 1, name: 'C2 / Stage3', plus: 'c2f', minus: 'c2r' },
  { idx: 2, name: 'C3 / Stage4', plus: 'c3f', minus: 'c3r' },
  { idx: 3, name: 'C4 / Stage2 Left', plus: 'c4f', minus: 'c4r' },
  { idx: 4, name: 'C5 / Stage5', plus: 'c5f', minus: 'c5r' },
  { idx: 5, name: 'C6 / Stage1 Turntable', plus: 'c6f', minus: 'c6r' },
];

let port = null;
let reader = null;
let writer = null;
let readLoopActive = false;
let readBuffer = '';

let currentPos = [0, 0, 0, 0, 0, 0];
let sequence = [];
let running = false;
let stopRequested = false;

const els = {
  connectBtn: document.getElementById('connectBtn'),
  disconnectBtn: document.getElementById('disconnectBtn'),
  refreshPosBtn: document.getElementById('refreshPosBtn'),
  connStatus: document.getElementById('connStatus'),
  stepSize: document.getElementById('stepSize'),
  jogMaxSps: document.getElementById('jogMaxSps'),
  jogRamp: document.getElementById('jogRamp'),
  axisGrid: document.getElementById('axisGrid'),
  zeroAllBtn: document.getElementById('zeroAllBtn'),
  home234Btn: document.getElementById('home234Btn'),
  recordPoseBtn: document.getElementById('recordPoseBtn'),
  delayMs: document.getElementById('delayMs'),
  addDelayBtn: document.getElementById('addDelayBtn'),
  mosfetAction: document.getElementById('mosfetAction'),
  addMosfetBtn: document.getElementById('addMosfetBtn'),
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

function setConnectedUI(connected) {
  els.connectBtn.disabled = connected;
  els.disconnectBtn.disabled = !connected;
  els.refreshPosBtn.disabled = !connected;
  els.zeroAllBtn.disabled = !connected;
  els.home234Btn.disabled = !connected;
  els.recordPoseBtn.disabled = !connected;
  els.addDelayBtn.disabled = !connected;
  els.addMosfetBtn.disabled = !connected;
  els.runSequenceBtn.disabled = !connected;
  els.sendCmdBtn.disabled = !connected;
  els.connStatus.textContent = connected ? 'Connected' : 'Disconnected';
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
        <button id="minus_${a.idx}" ${!port ? 'disabled' : ''}>- Jog</button>
        <button id="plus_${a.idx}" ${!port ? 'disabled' : ''}>+ Jog</button>
        <button id="zero_${a.idx}" ${!port ? 'disabled' : ''}>Zero This Axis</button>
      </div>
    `;
    els.axisGrid.appendChild(wrap);

    document.getElementById(`minus_${a.idx}`).onclick = () => jogAxis(a, false);
    document.getElementById(`plus_${a.idx}`).onclick = () => jogAxis(a, true);
    document.getElementById(`zero_${a.idx}`).onclick = () => zeroAxis(a.idx);
  });
}

function updatePosUI() {
  AXES.forEach((a) => {
    const el = document.getElementById(`pos_${a.idx}`);
    if (el) el.textContent = `pos=${currentPos[a.idx]}`;
  });
}

async function connectSerial() {
  if (!('serial' in navigator)) {
    alert('Web Serial is not supported in this browser. Use Chrome/Edge desktop.');
    return;
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

  setConnectedUI(true);
  renderAxes();
  log('Connected serial at 115200');
  await requestWhere();
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
    setConnectedUI(false);
    renderAxes();
    log('Disconnected');
  } catch (e) {
    log(`Disconnect error: ${e.message}`);
  }
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

function handleIncoming(line) {
  log(`< ${line}`);

  // Example: POS C1=0 C2=0 C3=0 C4=0 C5=0 C6=0
  const m = line.match(/^POS\s+C1=(-?\d+)\s+C2=(-?\d+)\s+C3=(-?\d+)\s+C4=(-?\d+)\s+C5=(-?\d+)\s+C6=(-?\d+)$/i);
  if (m) {
    currentPos = [1,2,3,4,5,6].map((i) => parseInt(m[i], 10));
    updatePosUI();
  }
}

async function sendLine(cmd) {
  if (!writer) throw new Error('Not connected');
  const clean = cmd.trim();
  if (!clean) return;
  log(`> ${clean}`);
  await writer.write(`${clean}\n`);
}

async function requestWhere() {
  await sendLine('where');
}

async function jogAxis(axis, positive) {
  if (!port) return;
  const steps = Math.max(1, parseInt(els.stepSize.value || '200', 10));
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

async function home234() {
  // Downward directions based on firmware mapping
  await sendLine('s2down 200000');
  await sendLine('s3down 200000');
  await sendLine('s4down 200000');
  // set zero for homed axes while preserving others
  await requestWhere();
  const p = [...currentPos];
  p[0] = 0; // s2 right
  p[3] = 0; // s2 left
  p[1] = 0; // s3
  p[2] = 0; // s4
  await sendLine(`setpos ${p.join(' ')}`);
  currentPos = p;
  updatePosUI();
}

function addSequenceItem(item) {
  sequence.push(item);
  renderSequence();
}

function renderSequence() {
  els.sequenceList.innerHTML = '';
  sequence.forEach((item, i) => {
    const li = document.createElement('li');
    if (item.type === 'pose') {
      li.textContent = `#${i} POSE [${item.pos.join(', ')}]`; 
    } else if (item.type === 'delay') {
      li.textContent = `#${i} DELAY ${item.ms}ms`;
    } else if (item.type === 'mosfet') {
      li.textContent = `#${i} MOSFET ${item.cmd}`;
    }
    els.sequenceList.appendChild(li);
  });
}

async function recordPose() {
  await requestWhere();
  addSequenceItem({ type: 'pose', pos: [...currentPos] });
}

function addDelay() {
  const ms = Math.max(0, parseInt(els.delayMs.value || '0', 10));
  addSequenceItem({ type: 'delay', ms });
}

function addMosfet() {
  addSequenceItem({ type: 'mosfet', cmd: els.mosfetAction.value });
}

async function runSequence() {
  if (running) return;
  if (!sequence.length) {
    alert('Sequence is empty.');
    return;
  }

  running = true;
  stopRequested = false;
  els.stopSequenceBtn.disabled = false;
  els.runSequenceBtn.disabled = true;

  const maxSps = Math.max(10, parseInt(els.playbackMaxSps.value || '1000', 10));
  const ramp = Math.max(1, parseInt(els.playbackRamp.value || '200', 10));

  try {
    for (const item of sequence) {
      if (stopRequested) break;

      if (item.type === 'pose') {
        await sendLine(`syncabs ${item.pos.join(' ')} ${maxSps} ${ramp}`);
        await waitForSyncAbsResult();
        await requestWhere();
      } else if (item.type === 'delay') {
        await sleep(item.ms);
      } else if (item.type === 'mosfet') {
        await sendLine(item.cmd);
      }
    }
  } catch (e) {
    log(`Run error: ${e.message}`);
  } finally {
    running = false;
    els.stopSequenceBtn.disabled = true;
    els.runSequenceBtn.disabled = !port;
    log(stopRequested ? 'Sequence stopped.' : 'Sequence complete.');
  }
}

function stopSequence() {
  stopRequested = true;
  sendLine('estop').catch((e) => log(`ESTOP send failed: ${e.message}`));
}

function waitForSyncAbsResult(timeoutMs = 300000) {
  return new Promise((resolve, reject) => {
    const started = Date.now();
    const startLen = els.logBox.textContent.length;

    const check = () => {
      if (!running) return resolve();
      const txt = els.logBox.textContent.slice(startLen);
      if (txt.includes('SYNCABS: done') || txt.includes('SYNCABS: aborted')) {
        return resolve();
      }
      if (Date.now() - started > timeoutMs) {
        return reject(new Error('Timed out waiting for SYNCABS completion'));
      }
      setTimeout(check, 100);
    };

    check();
  });
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
  els.connectBtn.onclick = async () => {
    try { await connectSerial(); } catch (e) { log(`Connect error: ${e.message}`); }
  };
  els.disconnectBtn.onclick = disconnectSerial;
  els.refreshPosBtn.onclick = requestWhere;
  els.zeroAllBtn.onclick = zeroAll;
  els.home234Btn.onclick = home234;
  els.recordPoseBtn.onclick = recordPose;
  els.addDelayBtn.onclick = addDelay;
  els.addMosfetBtn.onclick = addMosfet;
  els.runSequenceBtn.onclick = runSequence;
  els.stopSequenceBtn.onclick = stopSequence;
  els.clearSeqBtn.onclick = () => { sequence = []; renderSequence(); };
  els.saveLocalBtn.onclick = saveLocal;
  els.loadLocalBtn.onclick = loadLocal;
  els.exportBtn.onclick = exportJson;
  els.importInput.onchange = (e) => {
    const f = e.target.files?.[0];
    if (f) importJson(f);
  };
  els.sendCmdBtn.onclick = async () => {
    const cmd = els.manualCmd.value;
    els.manualCmd.value = '';
    await sendLine(cmd);
  };
}

function init() {
  wireEvents();
  renderAxes();
  renderSequence();
  setConnectedUI(false);
}

init();
