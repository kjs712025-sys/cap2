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

  const fire = status.fire || (status.metadata && (status.metadata.fire_detected ? 'detected' : status.metadata.fire_warning)) || 'clear';
  const fireEl = $('fire');
  if (fireEl) {
    fireEl.textContent = fire;
    fireEl.style.color = fire === 'detected' ? 'var(--alert)' : (fire !== 'clear' ? 'var(--warn)' : '');
  }
}

function updateHardware(hardware) {
  if (!hardware) return;
  const stm32 = hardware.stm32 || {};
  setBadge('stm32-badge', !!stm32.connected);
  $('stm32-detail').textContent = stm32.last_error ? `error: ${stm32.last_error}` : (stm32.connected ? 'connected' : 'not connected');

  const camera = hardware.camera || {};
  setBadge('camera-badge', !!camera.ready);
  $('camera-detail').textContent = `source: ${camera.source || 'unknown'}`;

  const lidar = hardware.lidar || {};
  setBadge('lidar-badge', !!lidar.ready);
  $('lidar-detail').textContent = `port: ${lidar.port || 'unknown'}`;
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
    updateHardware(components);
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
  await Promise.all([fetchUptime(), fetchEvents(), fetchAlerts(), fetchDiagnostics(), fetchTelemetry()]);
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
    $('motion-result').textContent = `sent vx=${vx} vy=${vy} wz=${wz}`;
  } catch (error) {
    $('motion-result').textContent = `failed: ${error.message}`;
  }
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

// Voice commands are an external-app-only feature (POST /command/voice with a
// recorded audio body). The dashboard intentionally does not record audio.

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

function decodeSlamGrid(b64, size) {
  const bin = atob(b64);
  const cells = new Uint8Array(size * size);
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
  const cells = slam.__cells || decodeSlamGrid(slam.grid_b64, size);
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

async function scanLidar() {
  $('lidar-obstacles').textContent = 'scanning...';
  try {
    const response = await fetch('/lidar/scan');
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'Scan failed');
    $('lidar-proximity').textContent = `${payload.data?.proximity ?? '--'} m`;
    $('lidar-safe-direction').textContent = payload.data?.safe_direction || '--';
    $('lidar-obstacles').textContent = (payload.data?.obstacles || []).length;
  } catch (error) {
    $('lidar-obstacles').textContent = 'failed';
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

  applyControlMode(nav.enabled ? 'autonomous' : 'manual');

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

// --- Control mode: 'manual' | 'autonomous' -------------------------------
let currentMode = 'manual';

function isManual() {
  return currentMode === 'manual';
}

function applyControlMode(mode) {
  currentMode = mode;
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
      ? 'manual mode — switch to Autonomous in Control mode to start auto-drive'
      : 'autonomous mode — reactive lidar + camera obstacle avoidance active';
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
}

async function setControlMode(mode) {
  const label = mode === 'auto' ? 'autonomous' : 'manual';
  const ind = $('mode-indicator');
  if (ind) ind.textContent = `mode: ${label}…`;
  try {
    const response = await fetch('/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.success) throw new Error(payload.message || 'mode switch failed');
    applyControlMode(payload.data.mode);
    pollNavState();
  } catch (error) {
    if (ind) { ind.textContent = `mode switch failed`; ind.style.color = 'var(--alert)'; }
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
$('refresh-button').addEventListener('click', refreshAll);

document.querySelectorAll('.dpad-btn[data-vx]').forEach((button) => {
  button.addEventListener('click', () => {
    sendMotion(Number(button.dataset.vx), Number(button.dataset.vy), Number(button.dataset.wz));
  });
});
$('motion-stop-btn').addEventListener('click', () => sendMotion(0, 0, 0));
$('motion-send-btn').addEventListener('click', () => {
  sendMotion(Number($('vx-input').value) || 0, Number($('vy-input').value) || 0, Number($('wz-input').value) || 0);
});

$('text-command-btn').addEventListener('click', sendTextCommand);
$('text-command-input').addEventListener('keydown', (event) => {
  if (event.key === 'Enter') sendTextCommand();
});

$('camera-refresh-btn').addEventListener('click', refreshCameraFeed);
$('camera-capture-btn').addEventListener('click', captureCamera);
$('camera-analyze-btn').addEventListener('click', analyzeCamera);

$('lidar-refresh-btn').addEventListener('click', scanLidar);
$('mode-manual-btn').addEventListener('click', () => setControlMode('manual'));
$('mode-auto-btn').addEventListener('click', () => setControlMode('auto'));
$('siren-btn').addEventListener('click', toggleSiren);

applyControlMode('manual');

connectWebSocket();
refreshAll();
refreshCameraFeed();
startSlamLoop();
startSlamStream();
pollNavState();
startNavStream();
setInterval(refreshAll, 5000);
// /camera/stream is a continuous MJPEG feed now, so the <img> stays live on
// its own — no periodic re-fetch needed (that used to restart the feed
// every 2s, which capped the effective preview rate and caused flicker).
