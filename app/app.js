const state = {
  recorder: null,
  chunks: [],
  stream: null,
};

const $ = (id) => document.getElementById(id);
const talkButton = $('talk-button');

function setStatus(label, transcript = '') {
  $('status-label').textContent = label;
  if (transcript) $('transcript').textContent = transcript;
}

function showError(message = '') {
  $('error').textContent = message;
}

function setConnection(online) {
  const connection = $('connection');
  connection.textContent = online ? 'Online' : 'Offline';
  connection.classList.toggle('online', online);
  connection.classList.toggle('offline', !online);
}

function updateTelemetry(data) {
  $('mode').textContent = data.mode || '--';
  $('mission').textContent = data.mission || '--';
  $('speed').textContent = data.current_speed ? data.current_speed.join(', ') : '--';
  $('ai-state').textContent = data.ai_state || '--';
}

async function refreshStatus() {
  try {
    const response = await fetch('/telemetry');
    if (!response.ok) throw new Error('Status request failed');
    const payload = await response.json();
    updateTelemetry(payload.data || {});
    setConnection(true);
  } catch (error) {
    setConnection(false);
  }
}

function getAudioMimeType() {
  const candidates = ['audio/webm;codecs=opus', 'audio/webm', 'audio/mp4'];
  return candidates.find((type) => MediaRecorder.isTypeSupported(type)) || '';
}

async function startRecording(event) {
  event.preventDefault();
  showError();
  if (!navigator.mediaDevices || !window.MediaRecorder) {
    showError('This browser does not support microphone recording.');
    return;
  }
  try {
    state.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const mimeType = getAudioMimeType();
    state.recorder = new MediaRecorder(state.stream, mimeType ? { mimeType } : undefined);
    state.chunks = [];
    state.recorder.addEventListener('dataavailable', (event) => {
      if (event.data.size) state.chunks.push(event.data);
    });
    state.recorder.start();
    document.body.classList.add('listening');
    setStatus('Listening...', 'Speak your command');
  } catch (error) {
    showError('Microphone permission is required.');
  }
}

async function stopRecording(event) {
  event.preventDefault();
  if (!state.recorder || state.recorder.state === 'inactive') return;
  const recorder = state.recorder;
  const mimeType = recorder.mimeType || 'audio/wav';
  recorder.addEventListener('stop', async () => {
    state.stream?.getTracks().forEach((track) => track.stop());
    document.body.classList.remove('listening');
    setStatus('Thinking...', 'Sending to Gemini');
    try {
      const response = await fetch('/command/voice', {
        method: 'POST',
        headers: { 'Content-Type': mimeType },
        body: new Blob(state.chunks, { type: mimeType }),
      });
      const payload = await response.json();
      if (!response.ok || !payload.success) throw new Error(payload.message || 'Command failed');
      const intent = payload.data?.intent || {};
      $('intent').textContent = intent.intent || 'idle';
      $('target').textContent = intent.target || intent.message || 'Command received';
      setStatus('Command sent', intent.message || `${intent.intent || 'idle'} / ${intent.target || 'self'}`);
      setConnection(true);
      refreshStatus();
    } catch (error) {
      setStatus('Unable to send', 'Check the robot connection');
      showError(error.message);
      setConnection(false);
    }
  }, { once: true });
  recorder.stop();
}

async function emergencyStop() {
  try {
    await fetch('/emergency-stop', { method: 'POST' });
    $('intent').textContent = 'stop';
    $('target').textContent = 'emergency stop';
    setStatus('Emergency stop', 'Robot motion halted');
    refreshStatus();
  } catch (error) {
    showError('Emergency stop request failed.');
  }
}

talkButton.addEventListener('pointerdown', startRecording);
talkButton.addEventListener('pointerup', stopRecording);
talkButton.addEventListener('pointercancel', stopRecording);
talkButton.addEventListener('pointerleave', (event) => {
  if (state.recorder?.state === 'recording') stopRecording(event);
});
$('stop-button').addEventListener('click', emergencyStop);
$('refresh-button').addEventListener('click', refreshStatus);
refreshStatus();
setInterval(refreshStatus, 5000);
