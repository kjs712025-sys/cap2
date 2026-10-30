const $ = (id) => document.getElementById(id);

let ws = null;
let wsRetryDelay = 1000;

function setConnection(online) {
  const connection = $('connection');
  connection.textContent = online ? 'Online' : 'Offline';
  connection.classList.toggle('online', online);
  connection.classList.toggle('offline', !online);
}

function setBadge(id, online) {
  const badge = $(id);
  badge.textContent = online ? 'online' : 'offline';
  badge.classList.toggle('online', online);
  badge.classList.toggle('offline', !online);
}

function formatUptime(seconds) {
  const s = Math.floor(seconds % 60);
  const m = Math.floor((seconds / 60) % 60);
  const h = Math.floor(seconds / 3600);
  return `uptime ${h}h ${m}m ${s}s`;
}

function updateStatus(status) {
  if (!status) return;
  $('mode').textContent = status.mode || '--';
  $('mission').textContent = status.mission || '--';
  $('battery').textContent = status.battery != null ? `${status.battery}%` : '--';
  $('temperature').textContent = status.temperature != null ? `${status.temperature} C` : '--';
  $('speed').textContent = status.current_speed ? status.current_speed.join(', ') : '--';
  $('ai-state').textContent = status.ai_state || '--';
  $('proximity').textContent = status.proximity != null ? `${status.proximity} m` : '--';
  $('safe-direction').textContent = status.safe_direction || '--';
  $('heartbeat').textContent = status.heartbeat != null ? status.heartbeat : '--';

  const fireAvailableInPayload = Object.prototype.hasOwnProperty.call(status, 'fire')
    || status.metadata?.fire_detected
    || status.metadata?.fire_warning;
  if (!fireAvailableInPayload) return;
  const fire = status.fire || (status.metadata && (status.metadata.fire_detected ? 'detected' : status.metadata.fire_warning)) || 'clear';
  const fireEl = $('fire');
  if (fireEl) {
    fireEl.textContent = fire === 'unavailable' ? 'unavailable · no camera' : fire;
    fireEl.style.color = fire === 'detected' ? 'var(--alert)' : (fire === 'unavailable' ? 'var(--muted)' : (fire !== 'clear' ? 'var(--warn)' : ''));
  }
}

function updateSystem(system) {
  if (!system) return;
  $('system-cpu-load').textContent = system.cpu_load_1m == null ? 'unavailable' : `${system.cpu_load_1m}%`;
  $('system-cpu-temp').textContent = system.cpu_temperature_c == null ? 'unavailable' : `${system.cpu_temperature_c} C`;
  $('system-memory').textContent = system.memory_available_mb == null ? 'unavailable' : `${system.memory_available_mb} MB`;
  $('system-disk').textContent = system.disk_free_gb == null ? 'unavailable' : `${system.disk_free_gb} GB`;
}

function updateFireMonitor(monitor) {
  if (!monitor) return;
  const states = {
    simulation: ['SIMULATION · NOT MONITORING', '시뮬레이션 영상은 화재 분석에 사용되지 않습니다. 실제 카메라와 검증된 화재 감지 모델이 필요합니다.'],
    disabled: ['DISABLED', '화재 감시가 설정에서 꺼져 있습니다.'],
    no_camera: ['NO CAMERA', '카메라가 연결되지 않아 화재 감시를 시작할 수 없습니다.'],
    monitoring: ['MONITORING', '화재 감시가 활성화되어 있습니다.'],
  };
  const [label, detail] = states[monitor.state] || ['UNAVAILABLE', '화재 감시 상태를 확인할 수 없습니다.'];
  $('fire-monitor-state').textContent = label;
  $('fire-monitor-detail').textContent = detail;
}

function updateHardware(hardware) {
  if (!hardware) return;
  const simulated = !!hardware.simulation;
  $('simulation-badge').hidden = !simulated;
  updateSystem(hardware.system);
  updateFireMonitor(hardware.fire_monitor);
  const stm32 = hardware.stm32 || {};
  setBadge('stm32-badge', !!stm32.connected);
  $('stm32-detail').textContent = simulated ? 'simulated controller · no physical device' : (stm32.last_error ? `error: ${stm32.last_error}` : (stm32.connected ? 'connected' : 'not connected'));

  const camera = hardware.camera || {};
  setBadge('camera-badge', !!camera.ready);
  $('camera-detail').textContent = simulated ? 'virtual camera · no physical device' : `source: ${camera.source || 'unknown'}`;

  const lidar = hardware.lidar || {};
  setBadge('lidar-badge', !!lidar.ready);
  $('lidar-detail').textContent = simulated ? 'virtual room scan · no physical device' : `port: ${lidar.port || 'unknown'}`;
}

function updateRobotName(name) {
  if (name) $('robot-name').textContent = name;
}

function renderEvents(events) {
  const list = $('events-list');
  if (!events || events.length === 0) {
    list.innerHTML = '<li class="empty">No events yet</li>';
    return;
  }
  list.innerHTML = events
    .slice()
    .reverse()
    .map((event) => {
      const time = new Date(event.timestamp * 1000).toLocaleTimeString();
      const payload = event.payload && Object.keys(event.payload).length ? JSON.stringify(event.payload) : '';
      return `<li><span class="kind">${event.event_type}</span><span>${payload}</span><span class="time">${time}</span></li>`;
    })
    .join('');
}

function renderAlerts(alerts) {
  const list = $('alerts-list');
  if (!alerts || alerts.length === 0) {
    list.innerHTML = '<li class="empty">No alerts</li>';
    return;
  }
  list.innerHTML = alerts
    .map((alert) => {
      const cls = alert.level === 'critical' ? 'alert-crit' : 'alert-warn';
      return `<li class="${cls}"><span class="kind">${alert.code}</span><span>${alert.message}</span></li>`;
    })
    .join('');
}

async function fetchDiagnostics() {
  try {
    const response = await fetch('/diagnostics');
    if (!response.ok) return;
    const payload = await response.json();
    const components = payload.data?.components;
    if (!components) return;
    updateHardware({
      ...components,
      simulation: payload.data.simulation,
      system: payload.data.system,
      fire_monitor: components.fire_monitor,
    });
    const llm = components.llm;
    if (llm) {
      setBadge('llm-badge', !!llm.ready);
      $('llm-detail').textContent = llm.configured ? (llm.ready ? 'ready' : 'configured, not ready') : 'no API key configured';
    }
  } catch (error) {
    setBadge('llm-badge', false);
    $('llm-detail').textContent = 'unavailable';
  }
}

async function fetchTelemetry() {
  try {
    const response = await fetch('/telemetry');
    if (!response.ok) return;
    const payload = await response.json();
    if (!payload.data) return;
    updateRobotName(payload.data.robot_name);
    updateStatus(payload.data);
  } catch (error) {
    // keep last known values on failure
  }
}

async function fetchUptime() {
  try {
    const response = await fetch('/health');
    if (!response.ok) return;
    const payload = await response.json();
    $('uptime').textContent = formatUptime(payload.data?.uptime_seconds || 0);
  } catch (error) {
    $('uptime').textContent = 'uptime --';
  }
}

async function fetchEvents() {
  try {
    const response = await fetch('/events?limit=15');
    if (!response.ok) return;
    const payload = await response.json();
    renderEvents(payload.data?.events);
  } catch (error) {
    // keep last known list on failure
  }
}

async function fetchAlerts() {
  try {
    const response = await fetch('/alerts');
    if (!response.ok) return;
    const payload = await response.json();
    renderAlerts(payload.data?.alerts);
  } catch (error) {
    // keep last known list on failure
  }
}

async function refreshAll() {
  await Promise.all([fetchUptime(), fetchEvents(), fetchAlerts(), fetchDiagnostics(), fetchTelemetry(), pollNav2State()]);
}

async function emergencyStop() {
  $('error').textContent = '';
  try {
    const response = await fetch('/emergency-stop', { method: 'POST' });
    if (!response.ok) throw new Error('Emergency stop request failed');
  } catch (error) {
    $('error').textContent = 'Emergency stop request failed.';
  }
}

async function clearEmergencyStop() {
  $('error').textContent = '';
  try {
    const response = await fetch('/emergency-stop/clear', { method: 'POST' });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'clear failed');
    pollNavState();
  } catch (error) {
    $('error').textContent = 'Emergency-stop clear failed.';
  }
}

async function sendMotion(vx, vy, wz) {
  if (!isManual()) {
    $('motion-result').textContent = '자율주행 모드입니다 — 수동조작으로 전환하세요';
    return;
  }
  $('motion-result').textContent = 'sending...';
  try {
    const response = await fetch('/motion/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vx, vy, wz }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Motion command failed');
    const command = payload.data?.command || { vx, vy, wz };
    $('motion-result').textContent = `sent vx=${command.vx} vy=${command.vy} wz=${command.wz}`;
  } catch (error) {
    $('motion-result').textContent = `failed: ${error.message}`;
  }
}

let motionCommandQueue = Promise.resolve();
function queueMotion(vx, vy, wz) {
  motionCommandQueue = motionCommandQueue.catch(() => {}).then(() => sendMotion(vx, vy, wz));
  return motionCommandQueue;
}

let dpadRepeatTimer = null;
let dpadReleaseTimer = null;
let dpadActiveButton = null;
let dpadStartedAt = 0;
const MIN_DPAD_RUN_MS = 450;

function stopDpadMotion() {
  clearTimeout(dpadReleaseTimer);
  dpadReleaseTimer = null;
  clearInterval(dpadRepeatTimer);
  dpadRepeatTimer = null;
  if (!dpadActiveButton) return Promise.resolve();
  dpadActiveButton = null;
  dpadStartedAt = 0;
  if (isManual()) return queueMotion(0, 0, 0);
  return Promise.resolve();
}

function releaseDpadMotion() {
  if (!dpadActiveButton || dpadReleaseTimer) return;
  const remaining = MIN_DPAD_RUN_MS - (performance.now() - dpadStartedAt);
  if (remaining > 0) {
    dpadReleaseTimer = setTimeout(() => {
      dpadReleaseTimer = null;
      stopDpadMotion();
    }, remaining);
    return;
  }
  stopDpadMotion();
}

function startDpadMotion(button) {
  if (!isManual() || button.disabled) return;
  stopDpadMotion();
  dpadActiveButton = button;
  dpadStartedAt = performance.now();
  const command = () => queueMotion(
    Number(button.dataset.vx),
    Number(button.dataset.vy),
    Number(button.dataset.wz),
  );
  command();
  dpadRepeatTimer = setInterval(command, 150);
}

function showMotion(m) {
  if (!m) return;
  const parts = [`action=${m.action || '?'}`];
  if (m.transcript) parts.unshift(`"${m.transcript}"`);
  if (m.action && m.action !== 'stop' && m.action !== 'none') {
    parts.push(`v=(${(m.vx ?? 0).toFixed(2)}, ${(m.vy ?? 0).toFixed(2)}, ${(m.wz ?? 0).toFixed(2)})`, `${m.duration_s ?? 0}s`);
  }
  if (m.speech) parts.push(`— ${m.speech}`);
  $('text-command-result').textContent = parts.join('  ');
}

async function sendTextCommand() {
  if (!isManual()) {
    $('text-command-result').textContent = '자율주행 모드입니다 — 수동조작으로 전환하세요';
    return;
  }
  const input = $('text-command-input');
  const text = input.value.trim();
  if (!text) return;
  $('text-command-result').textContent = 'thinking...';
  try {
    const response = await fetch('/command/manual', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Command failed');
    showMotion(payload.data?.motion);
    refreshAll();
  } catch (error) {
    $('text-command-result').textContent = `failed: ${error.message}`;
  }
}

// --- Voice command (LLM) -------------------------------------------------
// Press the button to start capturing the microphone, press again to stop and
// send. Audio is encoded to 16 kHz mono WAV client-side (a format Gemini
// accepts directly) and POSTed to /command/voice. Manual mode only.
let voiceRecording = false;
let voiceCtx = null;
let voiceStream = null;
let voiceNode = null;
let voiceSource = null;
let voiceChunks = [];

function setVoiceStatus(text) {
  const el = $('voice-command-status');
  if (el) el.textContent = text;
}

function encodeWav(samples, sampleRate) {
  const buffer = new ArrayBuffer(44 + samples.length * 2);
  const view = new DataView(buffer);
  const w = (offset, str) => { for (let i = 0; i < str.length; i += 1) view.setUint8(offset + i, str.charCodeAt(i)); };
  w(0, 'RIFF');
  view.setUint32(4, 36 + samples.length * 2, true);
  w(8, 'WAVE');
  w(12, 'fmt ');
  view.setUint32(16, 16, true);
  view.setUint16(20, 1, true);          // PCM
  view.setUint16(22, 1, true);          // mono
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true);
  view.setUint16(32, 2, true);
  view.setUint16(34, 16, true);
  w(36, 'data');
  view.setUint32(40, samples.length * 2, true);
  let offset = 44;
  for (let i = 0; i < samples.length; i += 1) {
    const s = Math.max(-1, Math.min(1, samples[i]));
    view.setInt16(offset, s < 0 ? s * 0x8000 : s * 0x7fff, true);
    offset += 2;
  }
  return new Blob([view], { type: 'audio/wav' });
}

async function startVoiceCapture() {
  const AC = window.AudioContext || window.webkitAudioContext;
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia || !AC) {
    setVoiceStatus('이 브라우저/연결에서는 마이크를 쓸 수 없습니다 (HTTPS 또는 localhost 필요)');
    return;
  }
  try {
    voiceStream = await navigator.mediaDevices.getUserMedia({ audio: true });
    voiceChunks = [];
    try { voiceCtx = new AC({ sampleRate: 16000 }); } catch (e) { voiceCtx = new AC(); }
    voiceSource = voiceCtx.createMediaStreamSource(voiceStream);
    voiceNode = voiceCtx.createScriptProcessor(4096, 1, 1);
    voiceNode.onaudioprocess = (ev) => {
      voiceChunks.push(new Float32Array(ev.inputBuffer.getChannelData(0)));
    };
    voiceSource.connect(voiceNode);
    voiceNode.connect(voiceCtx.destination);
  } catch (e) {
    setVoiceStatus('마이크를 시작할 수 없습니다: ' + (e && e.name === 'NotAllowedError' ? '권한 거부됨' : (e.message || e.name)));
    try { if (voiceStream) voiceStream.getTracks().forEach((t) => t.stop()); } catch (_) { /* ignore */ }
    voiceCtx = voiceNode = voiceSource = voiceStream = null;
    return;
  }

  voiceRecording = true;
  const btn = $('voice-command-btn');
  if (btn) { btn.classList.add('recording'); $('voice-btn-label').textContent = '⏹ 녹음 중 — 눌러서 전송'; }
  setVoiceStatus('녹음 중… 말한 뒤 버튼을 다시 누르세요');
}

async function stopVoiceCaptureAndSend() {
  voiceRecording = false;
  const btn = $('voice-command-btn');
  if (btn) { btn.classList.remove('recording'); $('voice-btn-label').textContent = '🎤 마이크 명령 시작'; }

  const rate = voiceCtx ? voiceCtx.sampleRate : 16000;
  try {
    if (voiceNode) voiceNode.disconnect();
    if (voiceSource) voiceSource.disconnect();
    if (voiceStream) voiceStream.getTracks().forEach((t) => t.stop());
    if (voiceCtx) await voiceCtx.close();
  } catch (e) { /* ignore teardown errors */ }
  voiceCtx = voiceNode = voiceSource = voiceStream = null;

  const total = voiceChunks.reduce((n, c) => n + c.length, 0);
  if (total < rate * 0.3) {           // < ~0.3 s captured
    setVoiceStatus('녹음이 너무 짧습니다 — 다시 시도하세요');
    return;
  }
  const merged = new Float32Array(total);
  let off = 0;
  for (const c of voiceChunks) { merged.set(c, off); off += c.length; }
  voiceChunks = [];

  const wav = encodeWav(merged, rate);
  setVoiceStatus('전송 중… Gemini가 명령을 해석합니다');
  $('text-command-result').textContent = 'thinking...';
  try {
    const response = await fetch('/command/voice', {
      method: 'POST',
      headers: { 'Content-Type': 'audio/wav' },
      body: wav,
    });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'voice command failed');
    const m = payload.data?.motion || {};
    setVoiceStatus(m.transcript ? `인식: "${m.transcript}"` : '음성을 인식하지 못했습니다');
    showMotion(m);
    refreshAll();
  } catch (error) {
    setVoiceStatus(`실패: ${error.message}`);
    $('text-command-result').textContent = `failed: ${error.message}`;
  }
}

let voiceBusy = false;
async function toggleVoiceCommand() {
  if (voiceBusy) return;                 // ignore rapid double-clicks
  voiceBusy = true;
  try {
    if (!isManual()) {
      setVoiceStatus('자율주행 모드입니다 — 수동조작으로 전환하세요');
      return;
    }
    if (voiceRecording) {
      await stopVoiceCaptureAndSend();
    } else {
      await startVoiceCapture();
    }
  } finally {
    voiceBusy = false;
  }
}

function refreshCameraFeed() {
  $('camera-feed').src = `/camera/stream?t=${Date.now()}`;
}

async function captureCamera() {
  $('camera-result').textContent = 'capturing...';
  try {
    const response = await fetch('/camera/capture', { method: 'POST' });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Capture failed');
    $('camera-result').textContent = `saved: ${payload.data?.path || 'unknown path'}`;
  } catch (error) {
    $('camera-result').textContent = `failed: ${error.message}`;
  }
}

async function analyzeCamera() {
  $('camera-result').textContent = 'analyzing...';
  try {
    const response = await fetch('/camera/analyze', { method: 'POST' });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Analyze failed');
    const detections = payload.data?.detections || [];
    $('camera-result').textContent = `${payload.data?.summary || 'no summary'} (${detections.length} detections)`;
  } catch (error) {
    $('camera-result').textContent = `failed: ${error.message}`;
  }
}

// --- Real-time SLAM occupancy map ------------------------------------------
let slamState = null;
let slamEventSource = null;
let slamRafRunning = false;
let rosSlamMap = null;
let selectedNav2Goal = null;

function decodeSlamGrid(b64, cellCount) {
  const bin = atob(b64);
  const cells = new Uint8Array(cellCount);
  for (let i = 0; i < cells.length; i += 1) cells[i] = bin.charCodeAt(i);
  return cells;
}

function drawSlam(slam) {
  const canvas = $('slam-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 420;
  const cssH = canvas.clientHeight || 420;
  if (canvas.width !== Math.round(cssW * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const styles = getComputedStyle(document.documentElement);
  ctx.fillStyle = styles.getPropertyValue('--panel') || '#0f1417';
  ctx.fillRect(0, 0, cssW, cssH);

  if (!slam || !slam.grid_b64) {
    ctx.fillStyle = styles.getPropertyValue('--muted') || '#7a8a8f';
    ctx.font = '13px system-ui, sans-serif';
    ctx.fillText('waiting for map…', 16, 24);
    return;
  }

  const size = slam.size;
  const cells = slam.__cells || decodeSlamGrid(slam.grid_b64, size * size);
  slam.__cells = cells;
  const px = cssW / size;
  const res = slam.resolution_m;
  const ox = slam.origin_m.x;
  const oy = slam.origin_m.y;

  // world (m) -> canvas (px); +y up, so flip the row axis
  const wx = (x) => ((x - ox) / res) * px;
  const wy = (y) => cssH - ((y - oy) / res) * px;

  // unknown: transparent · free: faint grey · occupied: warm red
  const oRgb = hexToRgb((styles.getPropertyValue('--alert') || '#ff7a66').trim()) || [255, 122, 102];
  const img = ctx.createImageData(size, size);
  for (let r = 0; r < size; r += 1) {
    for (let c = 0; c < size; c += 1) {
      const v = cells[r * size + c];
      const di = ((size - 1 - r) * size + c) * 4;
      if (v === 1) {
        img.data[di] = 150; img.data[di + 1] = 168; img.data[di + 2] = 172; img.data[di + 3] = 46;
      } else if (v === 2) {
        img.data[di] = oRgb[0]; img.data[di + 1] = oRgb[1]; img.data[di + 2] = oRgb[2]; img.data[di + 3] = 255;
      } else {
        img.data[di + 3] = 0;
      }
    }
  }
  const off = document.createElement('canvas');
  off.width = size; off.height = size;
  off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, 0, 0, cssW, cssH);

  // trajectory trail
  const trail = slam.trail || [];
  if (trail.length > 1) {
    ctx.beginPath();
    ctx.moveTo(wx(trail[0][0]), wy(trail[0][1]));
    for (let i = 1; i < trail.length; i += 1) ctx.lineTo(wx(trail[i][0]), wy(trail[i][1]));
    ctx.strokeStyle = styles.getPropertyValue('--accent') || '#5fe3d4';
    ctx.lineWidth = 2;
    ctx.globalAlpha = 0.85;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // live scan points
  for (const p of slam.scan_points || []) {
    ctx.fillStyle = 'rgba(95,227,212,0.55)';
    ctx.fillRect(wx(p.x) - 1, wy(p.y) - 1, 2, 2);
  }

  // robot pose + heading
  const pose = slam.pose || { x: 0, y: 0, yaw: 0 };
  const rx = wx(pose.x);
  const ry = wy(pose.y);
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = styles.getPropertyValue('--accent') || '#5fe3d4';
  ctx.beginPath();
  ctx.moveTo(9, 0);
  ctx.lineTo(-6, 6);
  ctx.lineTo(-6, -6);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function slamLoop() {
  try { drawSlam(slamState); } catch (e) { /* keep the loop alive */ }
  requestAnimationFrame(slamLoop);
}
function startSlamLoop() {
  if (slamRafRunning) return;
  slamRafRunning = true;
  requestAnimationFrame(slamLoop);
}

function startSlamStream() {
  if (slamEventSource) return;
  try {
    slamEventSource = new EventSource('/slam/stream');
    slamEventSource.onmessage = (event) => {
      try {
        slamState = JSON.parse(event.data);
        const p = slamState.pose || {};
        $('slam-pose-xy').textContent = `${(p.x ?? 0).toFixed(2)}, ${(p.y ?? 0).toFixed(2)} m`;
        $('slam-pose-yaw').textContent = `${Math.round(((p.yaw ?? 0) * 180) / Math.PI)}°`;
        $('slam-explored').textContent = `${Math.round((slamState.explored_frac || 0) * 100)}%`;
        const s = $('slam-stream-state');
        if (s) { s.textContent = 'stream live'; s.style.color = 'var(--accent)'; }
      } catch (e) { /* ignore malformed frame */ }
    };
    slamEventSource.onerror = () => {
      const s = $('slam-stream-state');
      if (s) { s.textContent = 'stream retry'; s.style.color = 'var(--muted)'; }
      slamEventSource.close();
      slamEventSource = null;
      setTimeout(startSlamStream, 3000);
    };
  } catch (e) {
    setTimeout(startSlamStream, 3000);
  }
}

function hexToRgb(hex) {
  const m = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return m ? [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)] : null;
}

const LIDAR_DISPLAY_RANGE_M = 6;
let lidarScanRequestActive = false;

function drawLidarScan(scan, ready) {
  const canvas = $('lidar-scan-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const size = canvas.clientWidth || 420;
  const pixelSize = Math.round(size * dpr);
  if (canvas.width !== pixelSize || canvas.height !== pixelSize) {
    canvas.width = canvas.height = pixelSize;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, size, size);

  const cx = size / 2;
  const cy = size / 2;
  const radius = size / 2 - 27;
  const toXY = (angleDeg, distanceM) => {
    const angle = (angleDeg - 90) * Math.PI / 180;
    const distance = Math.min(distanceM, LIDAR_DISPLAY_RANGE_M) / LIDAR_DISPLAY_RANGE_M * radius;
    return [cx - distance * Math.cos(angle), cy + distance * Math.sin(angle)];
  };

  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(95,227,212,0.025)';
  ctx.fill();
  ctx.font = '10px "DM Mono", ui-monospace, monospace';
  ctx.textAlign = 'left';
  for (const distance of [1.5, 3, 4.5, LIDAR_DISPLAY_RANGE_M]) {
    const ringRadius = distance / LIDAR_DISPLAY_RANGE_M * radius;
    ctx.beginPath();
    ctx.arc(cx, cy, ringRadius, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(206,239,227,0.14)';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'rgba(206,239,227,0.55)';
    ctx.fillText(`${distance}m`, cx + 4, cy - ringRadius + 12);
  }

  ctx.strokeStyle = 'rgba(206,239,227,0.10)';
  ctx.lineWidth = 1;
  for (let angle = 0; angle < 360; angle += 45) {
    const [x, y] = toXY(angle, LIDAR_DISPLAY_RANGE_M);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(x, y);
    ctx.stroke();
  }
  ctx.fillStyle = '#aebfba';
  ctx.textAlign = 'center';
  ctx.fillText('FRONT', cx, 13);
  ctx.fillText('REAR', cx, size - 5);
  ctx.fillText('LEFT', 14, cy + 3);
  ctx.fillText('RIGHT', size - 17, cy + 3);

  if (ready && Array.isArray(scan)) {
    for (const point of scan) {
      const angle = Number(point.angle_deg);
      const distance = Number(point.distance_m);
      if (!Number.isFinite(angle) || !Number.isFinite(distance) || distance <= 0 || distance > LIDAR_DISPLAY_RANGE_M) continue;
      const [x, y] = toXY(angle, distance);
      const near = distance <= 0.6;
      const caution = distance <= 1.2;
      ctx.beginPath();
      ctx.arc(x, y, near ? 2.8 : 1.8, 0, Math.PI * 2);
      ctx.fillStyle = near ? '#ff7a66' : caution ? '#ffce6a' : '#5fe3d4';
      ctx.fill();
    }

    ctx.save();
    ctx.translate(cx, cy);
    ctx.fillStyle = '#cdfb6e';
    ctx.beginPath();
    ctx.moveTo(0, -9);
    ctx.lineTo(6, 6);
    ctx.lineTo(0, 3);
    ctx.lineTo(-6, 6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  } else {
    ctx.fillStyle = '#ffce6a';
    ctx.font = '12px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('LiDAR scan unavailable', cx, cy + radius * 0.55);
  }
}

async function scanLidar() {
  if (lidarScanRequestActive) return;
  lidarScanRequestActive = true;
  $('lidar-scan-state').textContent = 'scanning…';
  $('lidar-scan-error').textContent = '';
  try {
    const response = await fetch('/lidar/scan');
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Scan failed');
    const data = payload.data || {};
    const scan = Array.isArray(data.scan) ? data.scan : [];
    const obstacles = Array.isArray(data.obstacles) ? data.obstacles : [];
    drawLidarScan(scan, !!data.ready);
    $('lidar-proximity').textContent = data.proximity == null ? '--' : `${Number(data.proximity).toFixed(2)} m`;
    $('lidar-safe-direction').textContent = data.safe_direction || '--';
    $('lidar-obstacles').textContent = String(obstacles.length);
    $('lidar-scan-state').textContent = data.ready ? 'LIVE' : 'OFFLINE';
    $('lidar-scan-caption').textContent = data.ready
      ? `front ↑ · range ${LIDAR_DISPLAY_RANGE_M} m · ${scan.length} points · ${new Date().toLocaleTimeString()}`
      : 'front ↑ · scan unavailable · check LiDAR connection';
    if (!data.ready) $('lidar-scan-error').textContent = 'No valid physical LiDAR scan is currently available.';
  } catch (error) {
    $('lidar-obstacles').textContent = 'failed';
    $('lidar-scan-state').textContent = 'ERROR';
    $('lidar-scan-caption').textContent = 'front ↑ · scan unavailable';
    $('lidar-scan-error').textContent = `LiDAR scan failed: ${error.message}`;
    drawLidarScan([], false);
  } finally {
    lidarScanRequestActive = false;
  }
}

/* ---- fire siren (Web Audio, no asset needed) ---- */

let sirenArmed = false;
let sirenCtx = null;
let sirenNodes = null;

function toggleSiren() {
  sirenArmed = !sirenArmed;
  const btn = $('siren-btn');
  btn.textContent = sirenArmed ? '🔊 Siren armed' : '🔇 Siren off';
  btn.classList.toggle('armed', sirenArmed);
  if (!sirenArmed) {
    stopSiren();
    return;
  }
  // The click is the user gesture browsers require before audio can play.
  try {
    sirenCtx = sirenCtx || new (window.AudioContext || window.webkitAudioContext)();
    if (sirenCtx.state === 'suspended') sirenCtx.resume();
  } catch (e) {
    /* no audio available */
  }
}

function startSiren() {
  if (!sirenArmed || !sirenCtx || sirenNodes) return;
  const osc = sirenCtx.createOscillator();
  osc.type = 'sawtooth';
  osc.frequency.value = 660;
  const gain = sirenCtx.createGain();
  gain.gain.value = 0.12;
  const lfo = sirenCtx.createOscillator();
  lfo.frequency.value = 0.8;              // warble rate
  const lfoGain = sirenCtx.createGain();
  lfoGain.gain.value = 320;               // frequency sweep depth
  lfo.connect(lfoGain).connect(osc.frequency);
  osc.connect(gain).connect(sirenCtx.destination);
  osc.start();
  lfo.start();
  sirenNodes = { osc, lfo, gain, lfoGain };
}

function stopSiren() {
  if (!sirenNodes) return;
  const { osc, lfo } = sirenNodes;
  try { osc.stop(); lfo.stop(); } catch (e) { /* already stopped */ }
  Object.values(sirenNodes).forEach((n) => { try { n.disconnect(); } catch (e) {} });
  sirenNodes = null;
}

function updateFireAlert(fire) {
  const level = fire && fire.level;
  const alerting = level && level !== 'clear';
  const banner = $('fire-banner');
  if (banner) {
    banner.hidden = !alerting;
    if (alerting) {
      banner.textContent = level === 'warn'
        ? '🔥 Possible fire — robot halted'
        : '🔥 FIRE DETECTED — robot halted';
    }
  }
  if (alerting) startSiren(); else stopSiren();
}

/* ---- autonomous navigation ---- */

const NAV_MAX_RANGE_M = 4;
const SECTOR_BEARINGS = {
  front: 0, front_left: 45, front_right: -45, left: 90, right: -90, rear: 180,
};

let radarSweep = 0;
let radarColors = null;
function getRadarColors() {
  if (radarColors) return radarColors;
  const s = getComputedStyle(document.documentElement);
  radarColors = {
    accent: s.getPropertyValue('--accent').trim() || '#cdfb6e',
    alert: s.getPropertyValue('--alert').trim() || '#ff7a66',
  };
  return radarColors;
}

function drawRadar(nav) {
  const canvas = $('nav-radar');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // crisp on HiDPI: size the backing store to the CSS box * dpr
  const dpr = window.devicePixelRatio || 1;
  const cssSize = canvas.clientWidth || 340;
  if (canvas.width !== Math.round(cssSize * dpr)) {
    canvas.width = canvas.height = Math.round(cssSize * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const size = cssSize;
  const cx = size / 2;
  const cy = size / 2;
  const radius = size / 2 - 10;
  const { accent, alert } = getRadarColors();

  ctx.clearRect(0, 0, size, size);

  const toXY = (bearingDeg, distM) => {
    const rad = (bearingDeg - 90) * Math.PI / 180;
    const r = Math.min(distM, NAV_MAX_RANGE_M) / NAV_MAX_RANGE_M * radius;
    return [cx - r * Math.cos(rad), cy + r * Math.sin(rad)];
  };

  // faint background disc
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(95,227,212,0.03)';
  ctx.fill();

  // range rings
  ctx.font = '9px "DM Mono", monospace';
  for (let m = 1; m <= NAV_MAX_RANGE_M; m++) {
    const r = (m / NAV_MAX_RANGE_M) * radius;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(206,239,227,0.10)';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'rgba(206,239,227,0.30)';
    ctx.fillText(`${m}m`, cx + 3, cy - r + 11);
  }
  // spokes every 45deg
  ctx.strokeStyle = 'rgba(206,239,227,0.07)';
  for (let a = 0; a < 360; a += 45) {
    const [ex, ey] = toXY(a, NAV_MAX_RANGE_M);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.stroke();
  }

  // rotating sweep
  radarSweep = (radarSweep + 0.05) % (Math.PI * 2);
  const grad = ctx.createConicGradient
    ? ctx.createConicGradient(radarSweep - Math.PI / 2, cx, cy)
    : null;
  if (grad) {
    grad.addColorStop(0, 'rgba(205,251,110,0.16)');
    grad.addColorStop(0.12, 'rgba(205,251,110,0)');
    grad.addColorStop(1, 'rgba(205,251,110,0)');
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = grad;
    ctx.fill();
  }

  // sector clearance wedges
  const sectors = (nav && nav.sectors) || {};
  const halfWidths = { front: 25, front_left: 20, front_right: 20, left: 25, right: 25, rear: 35 };
  for (const [name, bearing] of Object.entries(SECTOR_BEARINGS)) {
    const s = sectors[name];
    if (!s || s.min_distance == null) continue;
    const hw = (halfWidths[name] || 20) * Math.PI / 180;
    const mid = (bearing - 90) * Math.PI / 180;
    const r = Math.min(s.min_distance, NAV_MAX_RANGE_M) / NAV_MAX_RANGE_M * radius;
    const blocked = s.min_distance < 0.4;
    const tight = s.min_distance < 0.8;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, r, mid - hw, mid + hw);
    ctx.closePath();
    ctx.fillStyle = blocked ? 'rgba(255,122,102,0.18)' : tight ? 'rgba(255,206,106,0.12)' : 'rgba(95,227,212,0.09)';
    ctx.fill();
    if (s.source === 'camera') {
      ctx.strokeStyle = 'rgba(95,227,212,0.6)';
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // scan points with glow
  const points = (nav && nav.scan) || [];
  for (const p of points) {
    if (!p.distance_m) continue;
    const near = p.distance_m < 0.6;
    const [x, y] = toXY(p.angle_deg, p.distance_m);
    ctx.beginPath();
    ctx.arc(x, y, near ? 2.4 : 1.7, 0, Math.PI * 2);
    ctx.fillStyle = near ? alert : accent;
    ctx.shadowBlur = near ? 8 : 4;
    ctx.shadowColor = near ? alert : accent;
    ctx.fill();
  }
  ctx.shadowBlur = 0;

  // translation vector (vx forward / vy left) as a motion arrow
  const v = (nav && nav.velocity) || { vx: 0, vy: 0, wz: 0 };
  const speed = Math.hypot(v.vx || 0, v.vy || 0);
  if (speed > 0.001) {
    const k = (Math.min(speed / 0.3, 1) * radius * 0.62) / speed;
    const ex = cx - (v.vy || 0) * k;   // +vy = left = -x
    const ey = cy - (v.vx || 0) * k;   // +vx = forward = -y
    ctx.strokeStyle = 'rgba(205,251,110,0.55)';
    ctx.lineWidth = 2;
    ctx.lineCap = 'round';
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.stroke();
    ctx.setLineDash([]);
    const a = Math.atan2(ey - cy, ex - cx);
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - 7 * Math.cos(a - 0.4), ey - 7 * Math.sin(a - 0.4));
    ctx.lineTo(ex - 7 * Math.cos(a + 0.4), ey - 7 * Math.sin(a + 0.4));
    ctx.closePath();
    ctx.fillStyle = 'rgba(205,251,110,0.7)';
    ctx.fill();
  }

  // the 4-wheel omni robot, lidar at its centre
  drawOmniRobot(ctx, cx, cy, radius * 0.15, v, accent, alert);
}

// wheel spin phase accumulator (FL, FR, RL, RR)
const wheelPhase = [0, 0, 0, 0];
let lastRobotFrame = performance.now();

function drawOmniRobot(ctx, cx, cy, s, vel, accent, alert) {
  const now = performance.now();
  const dt = Math.min((now - lastRobotFrame) / 1000, 0.05);
  lastRobotFrame = now;

  const vx = vel.vx || 0, vy = vel.vy || 0, wz = vel.wz || 0;
  // omni/mecanum inverse kinematics (unit L): FL, FR, RL, RR
  const w = [
    vx - vy - wz,
    vx + vy + wz,
    vx + vy - wz,
    vx - vy + wz,
  ];

  const bodyW = s * 2.0;   // half-width
  const bodyH = s * 2.4;   // half-length
  const wheelW = s * 0.55;
  const wheelH = s * 0.95;
  // corner wheel centres (x=right, y=down in screen space; robot forward = up)
  const wheels = [
    [-bodyW, -bodyH], // FL
    [bodyW, -bodyH],  // FR
    [-bodyW, bodyH],  // RL
    [bodyW, bodyH],   // RR
  ];

  ctx.save();
  ctx.translate(cx, cy);

  // chassis
  ctx.beginPath();
  roundRect(ctx, -bodyW, -bodyH, bodyW * 2, bodyH * 2, s * 0.35);
  ctx.fillStyle = 'rgba(18,26,28,0.68)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(205,251,110,0.55)';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // forward chevron
  ctx.beginPath();
  ctx.moveTo(-s * 0.5, -bodyH * 0.45);
  ctx.lineTo(0, -bodyH * 0.72);
  ctx.lineTo(s * 0.5, -bodyH * 0.45);
  ctx.strokeStyle = accent;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.stroke();

  // wheels
  for (let i = 0; i < 4; i++) {
    wheelPhase[i] += w[i] * dt * 12;
    const [wx, wy] = wheels[i];
    ctx.save();
    ctx.translate(wx, wy);
    // omni roller direction alternates diagonally (X pattern)
    const rollerAngle = (i === 0 || i === 3) ? -Math.PI / 4 : Math.PI / 4;

    // tyre
    ctx.beginPath();
    roundRect(ctx, -wheelW, -wheelH, wheelW * 2, wheelH * 2, wheelW * 0.5);
    const spinning = Math.abs(w[i]) > 0.001;
    ctx.fillStyle = spinning ? 'rgba(205,251,110,0.16)' : 'rgba(206,239,227,0.10)';
    ctx.fill();
    ctx.strokeStyle = spinning ? accent : 'rgba(206,239,227,0.3)';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // rollers (clip to tyre, draw moving diagonal ticks)
    ctx.save();
    ctx.beginPath();
    roundRect(ctx, -wheelW, -wheelH, wheelW * 2, wheelH * 2, wheelW * 0.5);
    ctx.clip();
    ctx.rotate(rollerAngle);
    ctx.strokeStyle = spinning ? 'rgba(205,251,110,0.8)' : 'rgba(206,239,227,0.35)';
    ctx.lineWidth = 1.4;
    const step = wheelW * 0.9;
    const off = ((wheelPhase[i] % step) + step) % step;
    for (let d = -wheelH * 2; d < wheelH * 2; d += step) {
      ctx.beginPath();
      ctx.moveTo(d + off - wheelW * 2, -wheelH * 2);
      ctx.lineTo(d + off + wheelH * 2, wheelH * 2);
      ctx.stroke();
    }
    ctx.restore();
    ctx.restore();
  }

  // lidar puck at centre (rotating scanner mark)
  ctx.beginPath();
  ctx.arc(0, 0, s * 0.6, 0, Math.PI * 2);
  ctx.fillStyle = '#0c1614';
  ctx.fill();
  ctx.strokeStyle = accent;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  const lidarSpin = (now / 1000) * 4;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(s * 0.55 * Math.cos(lidarSpin), s * 0.55 * Math.sin(lidarSpin));
  ctx.strokeStyle = accent;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(0, 0, 2.2, 0, Math.PI * 2);
  ctx.fillStyle = accent;
  ctx.shadowBlur = 10;
  ctx.shadowColor = accent;
  ctx.fill();
  ctx.shadowBlur = 0;

  ctx.restore();
}

function roundRect(ctx, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function renderNavState(nav) {
  if (!nav) return;
  $('nav-decision').textContent = nav.decision || '--';
  $('nav-proximity').textContent = nav.proximity != null ? `${nav.proximity} m` : '--';
  const v = nav.velocity || {};
  $('nav-velocity').textContent = `${(v.vx ?? 0).toFixed(2)}, ${(v.wz ?? 0).toFixed(2)}`;
  $('nav-safe-direction').textContent = nav.safe_direction || '--';
  $('nav-reason').textContent = nav.reason || '';

  const stm32 = nav.stm32 || {};
  const stm32El = $('nav-stm32');
  if (stm32El) {
    const label = !stm32.connected ? 'offline' : (stm32.armed ? 'armed' : 'connected');
    stm32El.textContent = label;
    stm32El.style.color = !stm32.connected ? 'var(--muted)' : (stm32.armed ? 'var(--accent)' : '');
  }
  const batEl = $('nav-battery');
  if (batEl) batEl.textContent = stm32.battery_voltage != null ? `${stm32.battery_voltage} V` : '--';

  updateFireAlert(nav.fire);

  const cam = nav.camera || {};
  const camEl = $('nav-camera-fusion');
  if (camEl) {
    const overrode = Object.keys(cam.overrode || {});
    camEl.textContent = !cam.available
      ? 'camera: off'
      : `camera: floor ${Math.round((cam.floor_frac || 0) * 100)}%` + (overrode.length ? ` · overriding ${overrode.join(', ')}` : '');
    camEl.style.color = overrode.length ? 'var(--data)' : 'var(--muted)';
  }

  const grid = $('nav-sectors');
  if (grid) {
    grid.innerHTML = Object.entries(nav.sectors || {})
      .map(([name, s]) => {
        const d = s.min_distance;
        const cls = d == null ? '' : (d < 0.4 ? 'blocked' : (d < 0.8 ? 'tight' : ''));
        const label = name.replace('_', ' ');
        const tag = s.source === 'camera' ? ' 📷' : '';
        return `<div class="stat ${cls}"><span>${label}${tag}</span><strong>${d == null ? '--' : d + ' m'}</strong></div>`;
      })
      .join('');
  }

  lastNavState = nav;
  try { drawRadar(nav); } catch (e) { /* RAF loop keeps it alive */ }
}

let lastNavState = null;
let radarRafRunning = false;

function radarLoop() {
  try {
    drawRadar(lastNavState || { scan: [], sectors: {}, velocity: { vx: 0, vy: 0, wz: 0 } });
  } catch (e) {
    console && console.warn && console.warn('radar draw', e);
  }
  requestAnimationFrame(radarLoop);
}
function startRadarLoop() {
  if (radarRafRunning) return;
  radarRafRunning = true;
  requestAnimationFrame(radarLoop);
}

let navEventSource = null;
let navPollTimer = null;

function startNavStream() {
  try {
    navEventSource = new EventSource('/navigation/stream');
    navEventSource.onmessage = (event) => {
      $('nav-stream-state').textContent = 'stream live';
      try { renderNavState(JSON.parse(event.data)); } catch (e) { /* ignore */ }
    };
    navEventSource.onerror = () => {
      $('nav-stream-state').textContent = 'stream: polling';
      navEventSource.close();
      navEventSource = null;
      if (!navPollTimer) navPollTimer = setInterval(pollNavState, 700);
    };
  } catch (error) {
    if (!navPollTimer) navPollTimer = setInterval(pollNavState, 700);
  }
}

async function pollNavState() {
  try {
    const response = await fetch('/navigation/state');
    const payload = await response.json();
    if (payload.success) renderNavState(payload.data);
  } catch (error) {
    // keep last frame
  }
}

async function nav2Request(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
  });
  const payload = await response.json();
  if (!response.ok || !payload.success) {
    throw new Error(payload.message || payload.error || `Request failed (${response.status})`);
  }
  return payload.data || {};
}

function showNav2Message(message, isError = false) {
  const el = $('nav2-message');
  if (!el) return;
  el.textContent = message;
  el.classList.toggle('error', isError);
  el.classList.toggle('success', !isError);
}

let nav2SpeedUpdatePending = false;

async function pollNav2State() {
  try {
    const state = await nav2Request('/nav2/state');
    const speedSlider = $('nav2-speed');
    if (!nav2SpeedUpdatePending && document.activeElement !== speedSlider) {
      speedSlider.value = Number(state.max_linear_speed_mps).toFixed(2);
      $('nav2-speed-value').textContent = `${Number(state.max_linear_speed_mps).toFixed(2)} m/s`;
    }
    const control = await nav2Request('/mode');
    applyControlMode(control.mode);
    const badge = $('nav2-stack-state');
    badge.textContent = state.running ? `running · PID ${state.pid}` : 'not running';
    badge.classList.toggle('online', !!state.running);
    badge.classList.toggle('offline', !state.running);
    $('nav2-stop-btn').disabled = !state.running;
    const missionStartButton = $('nav2-mission-start-btn');
    const mission = state.mission_start || {};
    missionStartButton.disabled = !!mission.active || !!state.control_claimed || !!state.armed;
    $('nav2-release-btn').disabled = !state.control_claimed;
    $('nav2-goal-btn').disabled = !state.running || !state.control_claimed || !state.armed
      || (!state.simulation && (!state.slam_running || !state.ros_map_ready));
    $('nav2-cancel-btn').disabled = !state.control_claimed;
    const patrol = state.patrol || {};
    const recording = !!patrol.recording;
    const patrolling = !!patrol.patrolling;
    $('nav2-roam-start-btn').disabled = !!mission.active || !!state.roam_enabled || !!state.control_claimed
      || (state.simulation !== true && (!state.stm32_connected || !state.lidar_ready));
    $('nav2-roam-stop-btn').disabled = !state.roam_enabled;
    $('nav2-patrol-record-start-btn').disabled = !!mission.active || recording || patrolling || !!state.roam_enabled
      || !!state.control_claimed || !state.slam_running || !state.map_pose
      || (state.simulation !== true && (!state.stm32_connected || !state.lidar_ready));
    $('nav2-patrol-record-stop-btn').disabled = !recording;
    $('nav2-patrol-start-btn').disabled = !!mission.active || recording || patrolling || !!state.roam_enabled
      || Number(patrol.waypoint_count || 0) < 2;
    $('nav2-patrol-stop-btn').disabled = !patrolling;
    $('nav2-patrol-message').textContent = patrol.message
      ? `${patrol.message}${recording ? ` · ${patrol.point_count}점 기록` : ''}${patrol.waypoint_count ? ` · ${patrol.waypoint_count}개 지점` : ''}`
      : '저장 경로 없음 · SLAM 지도를 켜고 수동 경로를 기록하세요.';
    const slamBadge = $('ros-slam-state');
    slamBadge.textContent = state.slam_running ? 'running' : 'stopped';
    slamBadge.classList.toggle('online', !!state.slam_running);
    slamBadge.classList.toggle('offline', !state.slam_running);
    $('ros-slam-start-btn').disabled = !!state.slam_running;
    $('ros-slam-stop-btn').disabled = !state.slam_running;
    $('ros-slam-save-btn').disabled = !state.slam_running || !state.ros_map_ready;
    if (state.slam_running) {
      $('ros-slam-message').textContent = state.ros_map_ready
        ? `ROS /map live${state.ros_map_age_seconds == null ? '' : ` · updated ${state.ros_map_age_seconds.toFixed(1)} s ago`}${$('ros-slam-yaml-link').hidden ? '' : ' · saved map ready to download'}`
        : 'SLAM running; waiting for a fresh ROS /map.';
    } else {
      $('ros-slam-message').textContent = 'SLAM Toolbox is stopped. Start it to receive and display the ROS map.';
    }
    const missionStages = {
      starting_nav2: 'Nav2 스택 시작 중…',
      starting_slam: 'SLAM Toolbox 시작 중…',
      waiting_for_map: 'Nav2 활성화와 최신 ROS 지도·실제 LiDAR 스캔을 기다리는 중…',
      claiming_control: 'Nav2 제어권 확보 중…',
      arming_motors: '안전 상태 확인 후 모터 무장 중…',
      sending_goal: '목표 전송 중…',
    };
    if (mission.active) {
      const readinessLabels = {
        nav2: 'Nav2 프로세스',
        slam: 'SLAM',
        action_server: 'Nav2 활성 액션 서버',
        map: '최신 ROS 지도',
        lidar: '최신 실제 LiDAR 스캔',
      };
      const readiness = mission.readiness || {
        nav2: state.running,
        slam: state.slam_running,
        action_server: state.nav2_action_ready,
        map: state.ros_map_ready,
        lidar: state.lidar_ready,
      };
      const waitingFor = Object.entries(readiness)
        .filter(([, ready]) => !ready)
        .map(([check]) => readinessLabels[check] || check);
      const stageMessage = missionStages[mission.stage] || '자율주행 준비 중…';
      showNav2Message(
        mission.stage === 'waiting_for_map' && waitingFor.length
          ? `${stageMessage} 대기 항목: ${waitingFor.join(', ')}`
          : stageMessage
      );
    } else if (
      mission.stage === 'failed' &&
      mission.message &&
      !(
          mission.message === 'Nav2 or SLAM stopped during startup' &&
          state.running &&
          state.slam_running &&
          state.nav2_action_ready &&
          state.ros_map_ready &&
          state.lidar_ready &&
          (state.simulation || state.stm32_connected)
      )
    ) {
      showNav2Message(mission.message, true);
    } else if (!state.running && !state.workspace_built) {
      showNav2Message('Nav2 workspace not built. Build ros2_ws before starting.', true);
    } else if (!state.running && state.last_exit_code != null && state.last_exit_code !== 0) {
      showNav2Message(`Nav2 exited (${state.last_exit_code}); see logs/nav2.log.`, true);
    } else if (state.goal) {
      showNav2Message(`Nav2 목표 등록 · x=${state.goal.x.toFixed(2)} m, y=${state.goal.y.toFixed(2)} m. 주변을 계속 감독하세요.`);
    } else if (mission.stage === 'ready_for_goal') {
      showNav2Message(mission.message || '지도에서 목표 좌표를 지정해 Nav2로 전송하세요.');
    } else if (state.armed) {
      showNav2Message('자율주행 중입니다. 로봇을 계속 감독하세요. 제어 해제·정지 버튼으로 즉시 멈출 수 있습니다.');
    } else if (state.control_claimed) {
      showNav2Message('Nav2 제어권이 예약되었습니다. 모터는 해제 상태입니다.');
    } else if (mission.stage === 'active') {
      showNav2Message('목표를 전송했습니다. 주변을 계속 감독하세요.');
    } else if (state.running) {
      showNav2Message('스택이 준비되었습니다. 자율주행을 시작하고 지도 좌표로 Nav2 목표를 지정하세요.');
    } else {
      showNav2Message('자율주행 시작 후 Nav2·SLAM 준비가 완료되면 지도 좌표로 목표를 지정하세요.');
    }
  } catch (error) {
    showNav2Message(`Nav2 status unavailable: ${error.message}`, true);
  }
}

async function updateNav2Speed() {
  const speed = Number($('nav2-speed').value);
  $('nav2-speed-value').textContent = `${speed.toFixed(2)} m/s`;
  nav2SpeedUpdatePending = true;
  try {
    const result = await nav2Request('/nav2/speed', {
      method: 'POST',
      body: JSON.stringify({ max_linear_speed_mps: speed }),
    });
    $('nav2-speed').value = Number(result.max_linear_speed_mps).toFixed(2);
    $('nav2-speed-value').textContent = `${Number(result.max_linear_speed_mps).toFixed(2)} m/s`;
    showNav2Message(`자율주행 최대 이동 속도를 ${Number(result.max_linear_speed_mps).toFixed(2)} m/s로 설정했습니다.`);
  } catch (error) {
    showNav2Message(`속도 설정 실패: ${error.message}`, true);
  } finally {
    nav2SpeedUpdatePending = false;
  }
}

function drawRosSlamMap(map) {
  const canvas = $('ros-slam-canvas');
  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = canvas.clientWidth || 420;
  const cssHeight = canvas.clientHeight || 420;
  const pixelWidth = Math.round(cssWidth * dpr);
  const pixelHeight = Math.round(cssHeight * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0c1213';
  ctx.fillRect(0, 0, cssWidth, cssHeight);

  if (!map?.grid_b64 || !Number.isInteger(map.width) || !Number.isInteger(map.height)) {
    ctx.fillStyle = '#78908a';
    ctx.font = '13px system-ui, sans-serif';
    ctx.fillText('waiting for ROS /map…', 16, 24);
    return;
  }
  const size = map.width * map.height;
  const cells = map.__cells || decodeSlamGrid(map.grid_b64, size);
  map.__cells = cells;
  const image = ctx.createImageData(map.width, map.height);
  let minX = map.width;
  let minY = map.height;
  let maxX = -1;
  let maxY = -1;
  for (let row = 0; row < map.height; row += 1) {
    for (let column = 0; column < map.width; column += 1) {
      const source = row * map.width + column;
      const value = cells[source] > 127 ? cells[source] - 256 : cells[source];
      const dest = ((map.height - 1 - row) * map.width + column) * 4;
      if (value < 0) {
        image.data[dest] = 12; image.data[dest + 1] = 18; image.data[dest + 2] = 19; image.data[dest + 3] = 255;
      } else {
        minX = Math.min(minX, column);
        minY = Math.min(minY, row);
        maxX = Math.max(maxX, column);
        maxY = Math.max(maxY, row);
        const shade = Math.round(235 - value * 1.8);
        image.data[dest] = shade; image.data[dest + 1] = shade; image.data[dest + 2] = shade; image.data[dest + 3] = 255;
      }
    }
  }
  if (maxX < minX || maxY < minY) {
    ctx.fillStyle = '#78908a';
    ctx.font = '13px system-ui, sans-serif';
    ctx.fillText('waiting for known map cells…', 16, 24);
    return;
  }

  const paddingX = Math.ceil((maxX - minX + 1) * 0.12);
  const paddingY = Math.ceil((maxY - minY + 1) * 0.12);
  const cropLeft = Math.max(0, minX - paddingX);
  const cropRight = Math.min(map.width - 1, maxX + paddingX);
  const cropBottom = Math.max(0, minY - paddingY);
  const cropTop = Math.min(map.height - 1, maxY + paddingY);
  const cropWidth = cropRight - cropLeft + 1;
  const cropHeight = cropTop - cropBottom + 1;
  const sourceTop = map.height - 1 - cropTop;
  const offscreen = document.createElement('canvas');
  offscreen.width = map.width;
  offscreen.height = map.height;
  offscreen.getContext('2d').putImageData(image, 0, 0);
  const scale = Math.min((cssWidth - 16) / cropWidth, (cssHeight - 16) / cropHeight);
  const drawWidth = cropWidth * scale;
  const drawHeight = cropHeight * scale;
  const drawX = (cssWidth - drawWidth) / 2;
  const drawY = (cssHeight - drawHeight) / 2;
  map.__display = { cropLeft, cropTop, cropWidth, cropHeight, scale, drawX, drawY };
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(
    offscreen,
    cropLeft, sourceTop, cropWidth, cropHeight,
    drawX, drawY, drawWidth, drawHeight,
  );
  drawNav2GoalMarker(ctx, map, cells);
}

function drawNav2GoalMarker(ctx, map, cells) {
  if (!selectedNav2Goal || !map.__display || !map.origin_m || !map.resolution_m) return;
  const { cropLeft, cropTop, cropWidth, cropHeight, scale, drawX, drawY } = map.__display;
  const dx = selectedNav2Goal.x - map.origin_m.x;
  const dy = selectedNav2Goal.y - map.origin_m.y;
  const yaw = map.origin_m.yaw || 0;
  const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
  const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
  const column = localX / map.resolution_m;
  const row = localY / map.resolution_m;
  if (column < cropLeft || column >= cropLeft + cropWidth
    || row > cropTop || row <= cropTop - cropHeight) return;
  const cellColumn = Math.floor(column);
  const cellRow = Math.floor(row);
  const value = cells[cellRow * map.width + cellColumn] > 127
    ? cells[cellRow * map.width + cellColumn] - 256
    : cells[cellRow * map.width + cellColumn];
  if (value < 0 || value >= 65) return;
  const x = drawX + (column - cropLeft) * scale;
  const y = drawY + (cropTop - row) * scale;
  ctx.strokeStyle = '#42e6a4';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, 7, 0, Math.PI * 2);
  ctx.moveTo(x - 5, y);
  ctx.lineTo(x + 5, y);
  ctx.moveTo(x, y - 5);
  ctx.lineTo(x, y + 5);
  ctx.stroke();
}

function selectNav2GoalFromMap(event) {
  const map = rosSlamMap;
  if (!map?.__display || !map?.__cells || !map.origin_m || !Number.isFinite(map.resolution_m)) {
    showNav2Message('유효한 ROS 지도가 준비된 뒤 목표 지점을 선택하세요.', true);
    return;
  }
  const canvas = $('ros-slam-canvas');
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const { cropLeft, cropTop, cropWidth, cropHeight, scale, drawX, drawY } = map.__display;
  const column = cropLeft + (x - drawX) / scale;
  const row = cropTop - (y - drawY) / scale;
  if (column < cropLeft || column >= cropLeft + cropWidth
    || row > cropTop || row <= cropTop - cropHeight) {
    showNav2Message('표시된 지도 영역 안의 목표 지점을 선택하세요.', true);
    return;
  }
  const cellColumn = Math.floor(column);
  const cellRow = Math.floor(row);
  const rawValue = map.__cells[cellRow * map.width + cellColumn];
  const occupancy = rawValue > 127 ? rawValue - 256 : rawValue;
  if (occupancy < 0 || occupancy >= 65) {
    showNav2Message('목표는 지도가 확인한 빈 공간에 지정하세요.', true);
    return;
  }
  const localX = (cellColumn + 0.5) * map.resolution_m;
  const localY = (cellRow + 0.5) * map.resolution_m;
  const yaw = map.origin_m.yaw || 0;
  selectedNav2Goal = {
    x: map.origin_m.x + Math.cos(yaw) * localX - Math.sin(yaw) * localY,
    y: map.origin_m.y + Math.sin(yaw) * localX + Math.cos(yaw) * localY,
  };
  $('nav2-goal-x').value = selectedNav2Goal.x.toFixed(2);
  $('nav2-goal-y').value = selectedNav2Goal.y.toFixed(2);
  drawRosSlamMap(map);
  showNav2Message(`지도 목표 선택됨 · x=${selectedNav2Goal.x.toFixed(2)} m, y=${selectedNav2Goal.y.toFixed(2)} m. 도착 방향을 확인한 뒤 목표 전송을 누르세요.`);
}

async function pollRosSlamMap() {
  try {
    const result = await nav2Request('/nav2/slam/map');
    rosSlamMap = result.available ? result.map : null;
    drawRosSlamMap(rosSlamMap);
    const age = result.age_seconds;
    $('ros-slam-map-caption').textContent = rosSlamMap
      ? `${rosSlamMap.width} × ${rosSlamMap.height} cells · ${rosSlamMap.resolution_m.toFixed(2)} m/cell · known-area zoom${age == null ? '' : ` · ${age.toFixed(1)} s ago`}`
      : 'ROS /map occupancy grid';
  } catch (error) {
    $('ros-slam-message').textContent = `ROS map unavailable: ${error.message}`;
  }
}

async function startRosSlam() {
  $('ros-slam-start-btn').disabled = true;
  try {
    const result = await nav2Request('/nav2/slam/start', { method: 'POST' });
    $('ros-slam-message').textContent = result.message || 'ROS SLAM Toolbox started.';
  } catch (error) {
    $('ros-slam-message').textContent = error.message;
  }
  await Promise.all([pollNav2State(), pollRosSlamMap()]);
}

async function stopRosSlam() {
  if (!window.confirm('ROS 2 SLAM Toolbox를 종료할까요? Nav2 제어권이 활성화되어 있으면 먼저 해제해야 합니다.')) return;
  $('ros-slam-stop-btn').disabled = true;
  try {
    const result = await nav2Request('/nav2/slam/stop', { method: 'POST' });
    $('ros-slam-message').textContent = result.message || 'ROS SLAM Toolbox stopped.';
    $('ros-slam-yaml-link').hidden = true;
    $('ros-slam-image-link').hidden = true;
  } catch (error) {
    $('ros-slam-message').textContent = error.message;
  }
  await Promise.all([pollNav2State(), pollRosSlamMap()]);
}

async function saveRosSlamMap() {
  $('ros-slam-save-btn').disabled = true;
  $('ros-slam-message').textContent = 'Saving ROS map…';
  try {
    const result = await nav2Request('/nav2/slam/map/save', { method: 'POST', body: '{}' });
    $('ros-slam-yaml-link').href = result.yaml_url;
    $('ros-slam-image-link').href = result.image_url;
    $('ros-slam-yaml-link').hidden = false;
    $('ros-slam-image-link').hidden = false;
    $('ros-slam-message').textContent = '지도 저장 완료. YAML과 PGM 파일을 각각 다운로드하세요.';
  } catch (error) {
    $('ros-slam-message').textContent = error.message;
  }
  await pollNav2State();
}

function readGoalInputs() {
  const xValue = $('nav2-goal-x').value.trim();
  const yValue = $('nav2-goal-y').value.trim();
  const yawValue = $('nav2-goal-yaw').value.trim();
  if (!xValue || !yValue || !yawValue) throw new Error('목표 X, Y, 방향 값을 모두 입력하세요.');
  const goal = {
    x: Number(xValue),
    y: Number(yValue),
    yaw: Number(yawValue) * Math.PI / 180,
  };
  if (!Number.isFinite(goal.x) || !Number.isFinite(goal.y) || !Number.isFinite(goal.yaw)) {
    throw new Error('목표 좌표와 방향은 유효한 숫자여야 합니다.');
  }
  if (Math.max(Math.abs(goal.x), Math.abs(goal.y)) > 20) {
    throw new Error('목표 좌표는 지도 원점에서 20 m 이내여야 합니다.');
  }
  return goal;
}

async function startAutonomousMission() {
  const button = $('nav2-mission-start-btn');
  button.disabled = true;
  showNav2Message('Nav2·SLAM과 실제 LiDAR를 확인하고 목표 전송을 준비합니다. 모터는 목표를 보내기 전까지 이동하지 않습니다.');
  try {
    const result = await nav2Request('/nav2/mission/start', {
      method: 'POST',
      body: JSON.stringify({}),
    });
    applyControlMode('autonomous');
    showNav2Message(result.message || 'Nav2 준비가 완료됐습니다. 지도 좌표로 목표를 지정하세요.');
  } catch (error) {
    showNav2Message(error.message, true);
  } finally {
    await Promise.all([pollNav2State(), pollRosSlamMap()]);
  }
}

async function releaseNav2() {
  try {
    await nav2Request('/nav2/release', { method: 'POST', body: '{}' });
    showNav2Message('Nav2 control released; STM32 disarmed.');
  } catch (error) {
    showNav2Message(error.message, true);
  }
  await pollNav2State();
}

async function stopNav2Stack() {
  if (!window.confirm('Nav2를 종료하고 STM32 제어권을 해제할까요?')) return;
  $('nav2-stop-btn').disabled = true;
  try {
    await nav2Request('/nav2/stack/stop', { method: 'POST', body: '{}' });
    showNav2Message('Nav2 stopped; STM32 disarmed.');
  } catch (error) {
    showNav2Message(error.message, true);
  }
  await pollNav2State();
}

async function sendNav2Goal() {
  if (!window.confirm('입력한 지도 좌표로 Nav2 목표를 전송합니다. 로봇 주변과 정지 수단을 확인했습니까?')) return;
  try {
    const goal = readGoalInputs();
    const result = await nav2Request('/nav2/goal', {
      method: 'POST',
      body: JSON.stringify(goal),
    });
    showNav2Message(`Goal sent: x=${result.goal.x.toFixed(2)} m, y=${result.goal.y.toFixed(2)} m.`);
  } catch (error) {
    showNav2Message(error.message, true);
  }
  await pollNav2State();
}

async function cancelNav2Goal() {
  try {
    await nav2Request('/nav2/cancel', { method: 'POST', body: '{}' });
    showNav2Message('Nav2 goal cancellation requested.');
  } catch (error) {
    showNav2Message(error.message, true);
  }
}

async function startReactiveRoam() {
  if (!window.confirm('LiDAR 반응형 주변 탐색을 시작합니다. 주변과 정지 수단을 확인했습니까?')) return;
  try {
    await nav2Request('/navigation/roam/start', { method: 'POST', body: '{}' });
    showNav2Message('LiDAR 주변 탐색을 시작했습니다. 이동 중 계속 감독하세요.');
  } catch (error) {
    showNav2Message(error.message, true);
  }
  await pollNav2State();
}

async function stopReactiveRoam() {
  try {
    await nav2Request('/navigation/roam/stop', { method: 'POST', body: '{}' });
    showNav2Message('주변 탐색을 중지하고 모터를 해제했습니다.');
  } catch (error) {
    showNav2Message(error.message, true);
  }
  await pollNav2State();
}

async function startPatrolRecording() {
  if (!window.confirm('경로 기록을 시작합니다. SLAM 지도 위치가 표시되는지 확인하고 수동 조작으로 경로를 운전하세요.')) return;
  try {
    const result = await nav2Request('/nav2/patrol/record/start', { method: 'POST', body: '{}' });
    $('nav2-patrol-message').textContent = result.message || '경로 기록 중입니다. 수동으로 경로를 운전하세요.';
  } catch (error) {
    $('nav2-patrol-message').textContent = error.message;
  }
  await pollNav2State();
}

async function stopPatrolRecording() {
  try {
    const result = await nav2Request('/nav2/patrol/record/stop', { method: 'POST', body: '{}' });
    $('nav2-patrol-message').textContent = result.message || '경로를 저장했습니다.';
  } catch (error) {
    $('nav2-patrol-message').textContent = error.message;
  }
  await pollNav2State();
}

async function startRoutePatrol() {
  if (!window.confirm('저장된 경로를 Nav2로 순찰합니다. 반복 순찰 여부를 확인하고 로봇 주변을 비워 두었습니까?')) return;
  const button = $('nav2-patrol-start-btn');
  button.disabled = true;
  $('nav2-patrol-message').textContent = 'Nav2·SLAM 준비 후 경로 순찰을 시작합니다…';
  try {
    const result = await nav2Request('/nav2/patrol/start', {
      method: 'POST',
      body: JSON.stringify({ repeat: $('nav2-patrol-repeat').checked }),
    });
    $('nav2-patrol-message').textContent = result.patrol?.message || '경로 순찰을 시작했습니다.';
  } catch (error) {
    $('nav2-patrol-message').textContent = error.message;
  }
  await pollNav2State();
}

async function stopRoutePatrol() {
  try {
    const result = await nav2Request('/nav2/patrol/stop', { method: 'POST', body: '{}' });
    $('nav2-patrol-message').textContent = result.message || '경로 순찰을 중지했습니다.';
  } catch (error) {
    $('nav2-patrol-message').textContent = error.message;
  }
  await pollNav2State();
}

// --- Control mode: 'manual' | 'autonomous' -------------------------------
let currentMode = 'manual';
let modeRendered = null;

function isManual() {
  return currentMode === 'manual';
}

function applyControlMode(mode) {
  currentMode = mode;
  if (modeRendered === mode) return;   // arrives ~4 Hz on the nav stream — only touch the DOM on a real change
  modeRendered = mode;
  const manual = mode === 'manual';

  const manualBtn = $('mode-manual-btn');
  const autoBtn = $('mode-auto-btn');
  if (manualBtn) manualBtn.classList.toggle('active', manual);
  if (autoBtn) autoBtn.classList.toggle('active', !manual);

  const ind = $('mode-indicator');
  if (ind) {
    ind.textContent = manual ? 'mode: manual' : 'mode: autonomous';
    ind.style.color = manual ? 'var(--accent)' : 'var(--data)';
  }
  const note = $('nav-mode-note');
  if (note) {
    note.textContent = manual
      ? 'manual mode — start the Nav2 stack and select Nav2 autonomous mode to reserve ROS control'
      : 'Nav2 control selected — motors stay disarmed until you confirm arming in the Nav2 panel';
  }

  // Gate the joystick + LLM command panels to manual mode only.
  document.querySelectorAll('#motion-panel button, #motion-panel input').forEach((el) => {
    el.disabled = !manual;
  });
  document.querySelectorAll('#text-command-panel button, #text-command-panel input').forEach((el) => {
    el.disabled = !manual;
  });
  const motionPanel = $('motion-panel');
  const textPanel = $('text-command-panel');
  if (motionPanel) motionPanel.classList.toggle('mode-locked', !manual);
  if (textPanel) textPanel.classList.toggle('mode-locked', !manual);

  // Abandon any in-progress voice capture when leaving manual mode.
  if (!manual && voiceRecording) {
    voiceRecording = false;
    try {
      if (voiceNode) voiceNode.disconnect();
      if (voiceSource) voiceSource.disconnect();
      if (voiceStream) voiceStream.getTracks().forEach((t) => t.stop());
      if (voiceCtx) voiceCtx.close();
    } catch (e) { /* ignore */ }
    voiceCtx = voiceNode = voiceSource = voiceStream = null;
    voiceChunks = [];
    const btn = $('voice-command-btn');
    if (btn) { btn.classList.remove('recording'); const l = $('voice-btn-label'); if (l) l.textContent = '🎤 마이크 명령 시작'; }
    setVoiceStatus('자율주행 모드로 전환됨 — 녹음 취소');
  }
}

async function setControlMode(mode) {
  const label = mode === 'auto' ? 'autonomous' : 'manual';
  const ind = $('mode-indicator');
  if (ind) ind.textContent = `mode: ${label}…`;
  try {
    if (mode === 'auto') await stopDpadMotion();
    const response = await fetch('/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'mode switch failed');
    applyControlMode(payload.data.mode);
    await Promise.all([pollNavState(), pollNav2State()]);
  } catch (error) {
    if (ind) { ind.textContent = `mode switch failed`; ind.style.color = 'var(--alert)'; }
    showNav2Message(error.message, true);
  }
}

function connectWebSocket() {
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${protocol}://${location.host}/ws`);

  ws.addEventListener('open', () => {
    setConnection(true);
    wsRetryDelay = 1000;
  });

  ws.addEventListener('message', (event) => {
    try {
      const data = JSON.parse(event.data);
      updateRobotName(data.robot_name);
      updateStatus(data.status);
      updateHardware(data.hardware);
    } catch (error) {
      // ignore malformed frames
    }
  });

  ws.addEventListener('close', () => {
    setConnection(false);
    setTimeout(connectWebSocket, wsRetryDelay);
    wsRetryDelay = Math.min(wsRetryDelay * 2, 15000);
  });

  ws.addEventListener('error', () => {
    ws.close();
  });
}

// start the radar/robot render loop first so it survives any later wiring error
startRadarLoop();

$('stop-button').addEventListener('click', emergencyStop);
$('stop-clear-button').addEventListener('click', clearEmergencyStop);
$('refresh-button').addEventListener('click', refreshAll);

document.querySelectorAll('.dpad-btn[data-vx]').forEach((button) => {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    startDpadMotion(button);
    if (button.setPointerCapture) button.setPointerCapture(event.pointerId);
  });
  button.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      startDpadMotion(button);
    }
  });
  button.addEventListener('keyup', (event) => {
    if (event.key === 'Enter' || event.key === ' ') releaseDpadMotion();
  });
});
document.addEventListener('pointerup', releaseDpadMotion);
document.addEventListener('pointercancel', stopDpadMotion);
window.addEventListener('blur', stopDpadMotion);
$('motion-stop-btn').addEventListener('click', () => {
  if (dpadActiveButton) stopDpadMotion();
  else queueMotion(0, 0, 0);
});
$('motion-send-btn').addEventListener('click', () => {
  sendMotion(Number($('vx-input').value) || 0, Number($('vy-input').value) || 0, Number($('wz-input').value) || 0);
});

$('text-command-btn').addEventListener('click', sendTextCommand);
$('text-command-input').addEventListener('keydown', (event) => {
  if (event.key === 'Enter') sendTextCommand();
});
$('voice-command-btn').addEventListener('click', toggleVoiceCommand);

$('camera-refresh-btn').addEventListener('click', refreshCameraFeed);
$('camera-capture-btn').addEventListener('click', captureCamera);
$('camera-analyze-btn').addEventListener('click', analyzeCamera);

$('lidar-refresh-btn').addEventListener('click', scanLidar);
$('mode-manual-btn').addEventListener('click', () => setControlMode('manual'));
$('nav2-mission-start-btn').addEventListener('click', startAutonomousMission);
$('nav2-speed').addEventListener('input', () => {
  $('nav2-speed-value').textContent = `${Number($('nav2-speed').value).toFixed(2)} m/s`;
});
$('nav2-speed').addEventListener('change', updateNav2Speed);
$('ros-slam-canvas').addEventListener('click', selectNav2GoalFromMap);
$('nav2-release-btn').addEventListener('click', releaseNav2);
$('nav2-stop-btn').addEventListener('click', stopNav2Stack);
$('nav2-goal-btn').addEventListener('click', sendNav2Goal);
$('nav2-cancel-btn').addEventListener('click', cancelNav2Goal);
$('nav2-roam-start-btn').addEventListener('click', startReactiveRoam);
$('nav2-roam-stop-btn').addEventListener('click', stopReactiveRoam);
$('nav2-patrol-record-start-btn').addEventListener('click', startPatrolRecording);
$('nav2-patrol-record-stop-btn').addEventListener('click', stopPatrolRecording);
$('nav2-patrol-start-btn').addEventListener('click', startRoutePatrol);
$('nav2-patrol-stop-btn').addEventListener('click', stopRoutePatrol);
$('ros-slam-start-btn').addEventListener('click', startRosSlam);
$('ros-slam-stop-btn').addEventListener('click', stopRosSlam);
$('ros-slam-save-btn').addEventListener('click', saveRosSlamMap);
$('siren-btn').addEventListener('click', toggleSiren);

applyControlMode('manual');

connectWebSocket();
refreshAll();
drawLidarScan([], false);
scanLidar();
setInterval(scanLidar, 1000);
refreshCameraFeed();
startSlamLoop();
startSlamStream();
pollNavState();
startNavStream();
setInterval(refreshAll, 5000);
setInterval(pollNav2State, 1500);
setInterval(pollRosSlamMap, 2000);
pollRosSlamMap();
// /camera/stream is a continuous MJPEG feed now, so the <img> stays live on
// its own — no periodic re-fetch needed (that used to restart the feed
// every 2s, which capped the effective preview rate and caused flicker).
