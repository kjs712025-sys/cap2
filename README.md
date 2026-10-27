# AI Robot Backend

A modular, production-oriented backend for an autonomous robot running on Raspberry Pi 5 with Raspberry Pi OS 64-bit.

## Architecture

- Clean Architecture with dependency injection-friendly service boundaries
- STM32 UART integration for actuation
- Camera, LiDAR, LLM, voice, networking, and planning modules

## Project Structure

```text
AI_Robot/
├── main.py
├── config.py
├── requirements.txt
├── README.md
├── robot/
├── camera/
├── lidar/
├── ai/
├── voice/
├── network/
├── utils/
├── models/
└── tests/
```

## Quick Start

1. Create a virtual environment.
2. Install dependencies: `pip install -r requirements.txt`
3. Start the backend: `./start.sh`

### Local demo without hardware

Set `ROBOT_SIMULATION=1` in `.env` to run without a camera, LiDAR, STM32, or
MQTT broker. The dashboard shows a virtual camera, a room scan with obstacles,
simulated controller telemetry, and SLAM updates. Camera-based navigation
fusion and fire monitoring are disabled because the virtual camera is only a
visual preview. Motion and autonomous navigation commands only update the
simulation; they are never sent to a physical device. The dashboard marks this
mode as **SIMULATION · NO HARDWARE**.
Open the dashboard at `http://127.0.0.1:8000/app/dashboard.html` (the voice
command app at `/app/` also links to it).

The dashboard also reports host CPU load, CPU temperature, available memory,
and free disk space where Linux exposes them. Fire monitoring is shown as
unavailable in simulation or when no camera is connected; a virtual preview is
not treated as a fire-detection source.

Keep autonomous navigation on manual start by setting
`ROBOT_NAV_AUTOSTART=0` and `ROBOT_AUTOSTART_AUTONOMOUS=0`. To start locally,
set `ROBOT_HOST=127.0.0.1` and run `./start.sh`.

### Start dashboard automatically on boot

The `ai-robot.service` systemd unit starts the backend and dashboard after
network startup and restarts the backend if it fails. It uses this checkout's
Python virtual environment and `.env`, skips package installation during boot,
and disables autonomous-motion autostart. With the current simulation
configuration, no hardware is opened. In hardware mode peripherals may connect,
but the service does not arm the STM32 or start either autonomous navigator.

Install or update the service from this checkout:

```bash
sudo install -m 0644 ai-robot.service /etc/systemd/system/ai-robot.service
sudo systemctl daemon-reload
sudo systemctl enable --now ai-robot.service
```

Check its status and logs:

```bash
systemctl status ai-robot.service
journalctl -u ai-robot.service -f
```

Open `http://<robot-ip>:8000/` for the dashboard or `http://<robot-ip>:8000/app/`
for the voice command app.
Press and hold the microphone control, speak a command, then release it to
send the recording to Gemini through `/command/voice`.

For an STM32 Nucleo-F401RE connected through the Raspberry Pi GPIO UART, the
default serial device is `/dev/serial0` at `115200` baud. Connect Raspberry Pi
TX to Nucleo RX, Raspberry Pi RX to Nucleo TX, and connect the grounds. Both
boards must use 3.3V UART logic; do not connect a 5V signal to the Raspberry Pi.
Check the device with `ls -l /dev/serial0` and change `ROBOT_UART_PORT` in
`.env` only if the UART is exposed under another device name.

For a systemd deployment on Raspberry Pi, install the service unit and enable it:

```bash
sudo cp ai-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ai-robot
sudo systemctl start ai-robot
sudo systemctl status ai-robot
```

## API Summary

### Health
- GET `/health`

### Robot Status
- GET `/status`
- GET `/telemetry`

### Motion
- POST `/motion/command` with JSON body `{ "vx": 0.2, "vy": 0.0, "wz": -0.1 }`

On the dashboard, a short direction-button click drives for a bounded 450 ms;
holding it streams joystick setpoints until release. Releasing sends zero
velocity and disarms the STM32. The first nonzero manual command explicitly
arms the board, and command failures are reported instead of being returned
as success. Commands are clamped to the configured manual speed limits.

`robot/stm32.py` speaks a checksummed ASCII line protocol to the STM32 motor
board over `ROBOT_UART_PORT` (the ST-Link virtual COM port when connected by
USB — `/dev/ttyACM0`, or a `/dev/serial/by-id/...-if02` path for stability):

| Pi → STM32 | meaning |
|---|---|
| `A,1*HH` / `A,0*HH` | arm / disarm motor drivers |
| `V,<vx>,<vy>,<wz>*HH` | body velocity setpoint (m/s, m/s, rad/s) |
| `H*HH` | watchdog heartbeat (sent ~1 Hz while auto-driving) |

| STM32 → Pi | folded into robot status |
|---|---|
| `O,<x>,<y>,<yaw>` | wheel odometry → SLAM pose |
| `B,<volts>` | battery voltage |
| `S,<code>` / `E,<msg>` | board status / error |

The legacy reactive navigator remains available internally for LiDAR
perception, but dashboard/API autonomous mode is routed through Nav2. Keep
`ROBOT_NAV_AUTOSTART=0`; enabling it starts the legacy navigator at boot and
is not the supported Nav2 startup path. The STM32 link is retried on a 3 s
backoff if the board is absent or drops.

The matching STM32 firmware is [`firmware/omnibot_stm32/`](firmware/omnibot_stm32/)
— an Arduino sketch for a NUCLEO-F411RE driving two AM-DC2-2D dual motor
drivers (4-wheel omni). It does the omni mixing, a comms watchdog, and `S`/`B`
telemetry; see that folder's README for wiring and the bench-calibration steps.
- POST `/emergency-stop` — halt, drop autonomy, latch `error` mode
- POST `/emergency-stop/clear` — release the latch after a safety check, back to
  stationary `manual` mode (autonomy is not auto-resumed)
- ROS 2 control lease — `POST /nav2/claim`, `/nav2/arm`, `/nav2/cmd_vel`,
  `/nav2/heartbeat`, and `/nav2/release` (the bridge uses these endpoints)

### Android app

A native remote-control client (mirrors the dashboard) lives under
[`app-android/`](app-android/). `app-android/AGENT_PROMPT.md` is a
self-contained brief — the full REST/SSE/MJPEG API, screen list, and
implementation notes — to hand to a coding agent in Android Studio.

### Camera
- POST `/camera/capture`
- GET `/camera/stream`

### LiDAR
- GET `/lidar/scan`

### Control mode

The robot is always in exactly one control mode, and the dashboard's **Control
mode** panel switches it:

- `autonomous` — Nav2 owns the motion-control lease; **manual inputs are
  refused** with `{"error": "autonomous_active"}`. Selecting this mode requires
  a running Nav2 stack, reserves control, and keeps the motors disarmed.
- `manual` — the joystick (`/motion/command`) and the LLM text/voice commands
  (`/command/manual`, `/command/voice`) are live.
  The legacy reactive navigator's perception loop still runs but does not
  issue motion in this supported control flow.

Set `ROBOT_NAV_AUTOSTART=0` (and `ROBOT_AUTOSTART_AUTONOMOUS=0`) in the service
to boot in `manual` instead.

When the ROS 2 bridge claims control, the backend reports `autonomous` and
blocks manual commands until Nav2 releases the control lease or its heartbeat
times out.

Endpoints:

- GET `/mode` → `{ "mode": "manual" | "autonomous" | "error" }`
- POST `/mode` `{ "mode": "manual" | "auto" }` — `auto` reserves Nav2 control
  without arming; `manual` releases Nav2 and disarms

### Autonomous navigation
- GET `/navigation/state` — latest snapshot: decision, per-sector clearances, downsampled scan, velocity
- GET `/navigation/stream` — Server-Sent Events feed of the snapshot (~4 Hz) for the dashboard radar
- POST `/navigation/start` / POST `/navigation/stop` — compatibility aliases
  for selecting Nav2 autonomous / manual mode; they do not arm the motors

#### Nav2 using the existing STM32 and LiDAR

The ROS 2 integration is in [`ros2_ws/src/omnibot_nav2/`](ros2_ws/src/omnibot_nav2/).
It reuses the backend as the sole owner of both serial devices: the backend
publishes YDLIDAR scans through its HTTP API, and ROS sends bounded `cmd_vel`
commands back through the backend. Do not start a second ROS serial driver for
the LiDAR or STM32. The old reactive navigator and Nav2 must not drive at the
same time; the backend control lease rejects conflicting commands.

The Nav2 bridge estimates `odom → base_link` by integrating commanded velocity.
`slam_toolbox` uses LiDAR scan matching to publish map corrections. This is
open-loop odometry, not wheel feedback: wheel slip, load, uneven floors, and
motor variation cause drift, and scan matching can fail in feature-poor spaces.
It is intended for supervised, low-speed evaluation, not safety-rated
autonomy. The provisional outer footprint (340 × 250 mm) and centered LiDAR
pose are in [`nav2/omnibot_geometry.yaml`](nav2/omnibot_geometry.yaml); verify
them physically before navigation.
The speed limits below constrain command values, not measured wheel speed:
the existing STM32 firmware has no velocity feedback, so actual speed must be
measured and calibrated before any floor test.

Build the ROS package on the robot after the backend service is active. The
full CLI launch below is an alternative to the dashboard process controls; do
not run it at the same time as the dashboard-managed ROS processes:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
# Install python3-colcon-common-extensions with apt if colcon is not available.
colcon build --symlink-install --packages-select omnibot_nav2
source install/setup.bash
ros2 launch omnibot_nav2 omnibot_nav2.launch.py
```

The full CLI launch starts the scan/odometry bridge, online `slam_toolbox`, and Nav2.
It does **not** claim motor control or arm the STM32. First verify `/scan`,
`/odom`, and TF (`map → odom → base_link → lidar_link`) while the robot is
stationary. The bridge requires a fresh complete physical scan before it will
arm, and releases/disarms if scans become stale or the backend heartbeat is
lost. With wheels raised and the chassis secured, explicitly claim and arm the
bridge only for supervised checks:

```bash
ros2 service call /omnibot/claim_control std_srvs/srv/Trigger "{}"
ros2 service call /omnibot/arm std_srvs/srv/Trigger "{}"
```

Send a Nav2 goal only after sensor orientation, scan matching, and the
open-loop odometry have been checked. The bridge and backend cap linear
velocity setpoints to 0.10 m/s and angular setpoints to 0.25 rad/s; these are
not measured physical-speed limits. Release control to stop and disarm:

```bash
ros2 service call /omnibot/release_control std_srvs/srv/Trigger "{}"
```

The dashboard's **자율주행 (Nav2)** mode button reserves control for Nav2 only
after the ROS 2 Nav2 stack is running. It never arms the motors. The explicit
**모터 무장** action performs the separate sensor/safety checks and asks for
confirmation; only then can a goal move the robot. **수동조작** releases the
lease and disarms.

The dashboard starts the ROS backend bridge and Nav2 independently from the
**Autonomous navigation → ROS 2 Jazzy · Nav2** panel. The separate
**ROS 2 SLAM Toolbox** panel starts/stops mapping without stopping Nav2. Its
canvas displays the live ROS `/map` occupancy grid; **지도 저장** invokes
`nav2_map_server map_saver_cli` and then exposes YAML and PGM downloads. The
files are written under `maps/nav2_map.*` in the project directory.

Starting either ROS component does not move the robot. On real hardware,
explicit arming is disabled until SLAM is running and a fresh ROS map is being
received, in addition to the physical LiDAR and STM32 checks. Goal X/Y values
are in the current map frame, and the direction is in degrees. **목표 취소**
cancels the active goal; **제어 해제·정지** disarms and releases control.
**Nav2 스택 종료** leaves an independently running SLAM process intact.
SLAM cannot be stopped while Nav2 holds the motor-control lease. These
dashboard actions do not replace the stationary checks or supervised hardware
validation above.

The backend also disarms if the bridge heartbeat is lost. The dashboard's
emergency-stop endpoint remains available. If backend API authentication is
configured, provide the same token to the bridge through `ROBOT_API_TOKEN`.

The launch files and ROS parameters are integration scaffolding; hardware
motion is not auto-started by systemd and has not been validated on the robot.

A reactive obstacle-avoidance loop (`robot/navigation.py`) runs continuously
for situational awareness: each cycle it reads a LiDAR scan, splits it into
angular sectors (front / front-left / front-right / left / right / rear) and
publishes clearances + a driving decision. It only commands motion in
`autonomous` mode (`/navigation/start`, `POST /mode {auto}`, or
`ROBOT_NAV_AUTOSTART=1`) and never while an emergency stop is latched. Policy:
cruise when the front is clear (> 0.9 m), slow and steer toward the roomier
diagonal as it closes in, rotate in place toward the clearest side when blocked
(< 0.4 m ahead), reverse when boxed in. Tune with `ROBOT_NAV_INTERVAL`,
`ROBOT_NAV_FRONT_OFFSET_DEG` (calibrate which LiDAR bearing is "forward"),
`ROBOT_NAV_CRUISE_SPEED`, `ROBOT_NAV_TURN_SPEED`.

#### Camera obstacle fusion

The navigator also runs a monocular free-space estimator on the live camera
frame (`camera/depth.py`). It samples a floor patch at the bottom of the image,
flags every column that deviates from that colour in Lab space as an obstacle,
maps the lowest obstacle row to an approximate distance via a ground-plane
assumption, and projects each column to a bearing through the camera FOV. The
resulting front / front-left / front-right clearances are fused into the LiDAR
sectors by taking the **smaller** of the two (fail-safe), so the robot also
stops for low or glass obstacles the LiDAR misses and for anything inside the
LiDAR's near blind spot. Fused sectors are tagged `"source": "camera"` in
`/navigation/state` and drawn with a dashed teal outline on the dashboard
radar. Runs every `ROBOT_NAV_CAMERA_INTERVAL` s (default 0.5); disable with
`ROBOT_NAV_USE_CAMERA=0`, calibrate the lens with `ROBOT_NAV_CAMERA_FOV_DEG`.
The estimate is only trusted when it sees enough floor and is dropped after
2 s without a fresh frame.

The dashboard shows a live radar (scan points, sector markers, heading arrow),
the current decision/velocity/clearances, Start/Stop controls, and the
live SLAM occupancy map.

### Real-time SLAM

`slam/slam.py` maintains a log-odds occupancy grid (200 × 200 cells at 5 cm,
a 10 m × 10 m window centred on the start pose). The navigator's perception
loop feeds it on **every** cycle, whether or not the robot is driving: the
pose is dead-reckoned from STM32 wheel odometry (`O,x,y,yaw`) when available
and otherwise from the commanded velocity, and each LiDAR beam is ray-cast
into the grid — clearing the free space it passes through and reinforcing the
cell it terminates on. Cells therefore converge on free / occupied / unknown
as the robot moves rather than just accumulating obstacle blobs.

- `GET /slam/stream` — SSE feed (1 Hz) of the packed grid (base64, 1 byte per
  cell: 0 unknown / 1 free / 2 occupied), pose, and the pose trail. The
  dashboard renders this to a canvas with the trajectory and live scan points.
- `GET /slam/status`, `GET /slam/map` — JSON pose + occupied-cell list.
- `GET /slam/map/image` — the grid as a PNG.

### MQTT

The backend publishes to a local `mosquitto` broker (`ROBOT_MQTT_HOST`,
default `localhost:1883`), under `ROBOT_MQTT_BASE_TOPIC` (default `robot`):

| Topic | Payload (retained) |
|---|---|
| `robot/status` | full status + hardware snapshot, 1 Hz |
| `robot/navigation` | decision, velocity, per-sector clearances, 1 Hz |
| `robot/fire` | `{ level, detections: [{label, confidence, box}], timestamp }` — pushed immediately on any change, refreshed 1 Hz |

`level` is `clear` / `warn` / `suppressed` / `stop`.

When a detection is published (any warn-or-higher level):

- **the camera MJPEG stream** (`/camera/stream`) gets a coloured border +
  labelled bounding boxes drawn over the live feed (amber for warn, red for
  stop), persisting for `DETECTION_TTL_S` seconds after the last sighting;
- **autonomous navigation halts** — a soft stop (zero velocity + STM32
  disarm, `mission=fire_halt`) that does *not* latch the emergency stop, so
  it can be restarted from the dashboard once the view clears. Disable with
  `ROBOT_FIRE_HALT_ON_DETECT=0`;
- **the dashboard sounds a siren** (Web Audio, no asset) and shows a banner,
  as long as the "Siren armed" button in the top bar has been clicked once
  (browsers require a user gesture before audio).

`fire` is also included in the `/navigation/state` and `/navigation/stream`
payloads so the dashboard reacts at ~4 Hz. Set `ROBOT_MQTT_ENABLED=0` to
disable publishing.

#### Fire alerts for external apps

The `mosquitto` broker listens on `0.0.0.0:1883` (see
`/etc/mosquitto/conf.d/robot.conf`), so an app on the robot's Wi-Fi can
subscribe to `robot/fire` directly. For clients that only speak HTTP, the same
payload is exposed over the REST API:

- `GET /fire/stream` — SSE feed that mirrors the retained `robot/fire` topic:
  the current state is sent on connect and every change (new detection, level
  change, return to `clear`) is pushed as it happens, with a `: keepalive`
  comment every ~15 s.
- `GET /fire/state` — one-shot fetch of the current fire payload.

Both honour `ROBOT_API_TOKEN` and CORS (`ROBOT_CORS_ORIGINS`), and are listed
under `streams` / `endpoints` in `GET /api`.

### Voice / text manual driving

Speak or type a natural-language command and Gemini turns it into a **timed
manual velocity** that streams to the STM32 and then auto-stops. These only
work in `manual` control mode (see **Control mode** above); in `autonomous`
mode they return `autonomous_active`:

- POST `/command/voice` — raw audio body (`Content-Type` = the recorded MIME);
  Gemini transcribes **and** interprets it. Used by both the dashboard and
  external apps.
- POST `/command/manual` — `{ "text": "왼쪽으로 천천히 돌아" }` — the
  dashboard's text box uses this.
- POST `/command/stop` — cancel any active motion
- POST `/command/text` — legacy intent-planner path (navigate/stop only)

The response `motion` object is `{ action, vx, vy, wz, duration_s, speech,
transcript }`. `action` ∈ `move | turn | strafe | stop | none`. Every command
is self-limiting — it never drives longer than `ROBOT_MANUAL_COMMAND_TIMEOUT`
(default 4 s) and auto-stops after `duration_s`. Gemini even estimates the
duration ("90도 돌아" → ~3 s at 0.5 rad/s). Without a key a Korean/English
keyword fallback handles the common cases.

**Dashboard voice button** — the *Voice / text command* panel has a 🎤 button:
click to start capturing the microphone, click again to stop and send. Audio is
encoded to 16 kHz mono WAV in the browser (a format Gemini accepts directly)
and POSTed to `/command/voice`. `getUserMedia` needs a **secure context**, so
the mic works from `http://localhost:8000/app/` on the Pi or over HTTPS; to use
it from a plain-HTTP LAN address, allowlist the origin in the browser
(Chrome: `chrome://flags/#unsafely-treat-insecure-origin-as-secure`).

`POST /camera/analyze` still uses Gemini for object/scene analysis.

### External / app access

The server binds `0.0.0.0:8000` — reach it from another device on the LAN at
`http://<pi-ip>:8000`. `GET /api` is a discovery endpoint listing every
stream and command endpoint plus whether auth is required.

- **CORS** is open by default (`ROBOT_CORS_ORIGINS=*`); set a comma list to
  restrict web-app origins.
- **Auth**: set `ROBOT_API_TOKEN` to require `Authorization: Bearer <token>`
  (or `?token=<token>`) on every API route. `/health`, `/api`, and `/app/*`
  stay open. Unset → fully open (LAN dev default).
- Feeds an app can consume: `/telemetry`, `/navigation/state`,
  `/navigation/stream` (SSE), `/camera/stream` (MJPEG), the `robot/*` MQTT
  topics, and `/ws` (note: the packaged uvicorn/websockets combo currently
  rejects `/ws` — use SSE/MQTT/polling).

### Fire detection

The backend runs a background fire/smoke monitor on the camera feed every
`ROBOT_FIRE_MONITOR_INTERVAL` seconds (default 2). Detection uses a **local
YOLOv8** model — no cloud API key required — with weights at
`models/fire_detection/fire_yolov8s.pt` (trained fire/smoke model from
[Abonia1/YOLOv8-Fire-and-Smoke-Detection](https://github.com/Abonia1/YOLOv8-Fire-and-Smoke-Detection),
`runs/detect/train/weights/best.pt`). Requires the `ultralytics` package.

Because this model is prone to false positives on bright indoor lighting,
the response is two-tier:

- confidence ≥ `ROBOT_FIRE_WARN_CONFIDENCE` (default `0.4`) → a non-blocking
  `fire_warning` event is logged and `status.metadata["fire_warning"]` is set;
- confidence ≥ `ROBOT_FIRE_STOP_CONFIDENCE` (default `0.7`) → **only if**
  `ROBOT_FIRE_AUTO_STOP=1`, the STM32 gets an emergency stop and the robot
  enters `error` mode with mission `fire_detected`. By default `fire_auto_stop`
  is **off**: the bundled model false-positives on blur/haze/bright light
  frequently enough (observed up to ~0.70 confidence on an out-of-focus frame)
  that halting the robot on it is unsafe. With auto-stop off, a stop-threshold
  detection is logged as `fire_stop_suppressed` and surfaced on the dashboard
  but the robot keeps running. Only enable auto-stop with a vetted model and a
  properly aimed/focused camera.

Independently of `auto_stop`, a warn-or-higher detection also **soft-halts**
autonomous navigation (`fire_halt` — zero velocity + STM32 disarm, no e-stop
latch) so it can be restarted from the dashboard once the view clears. To keep
the noisy model from stop-and-go flapping the robot, this halt is debounced:
it needs `ROBOT_FIRE_HALT_CONSECUTIVE` consecutive sightings (default `2`)
before it disarms — the alert flag, dashboard siren and `robot/fire` topic
still fire on the first frame. Set `ROBOT_FIRE_HALT_ON_DETECT=0` to drop the
navigation halt entirely.

Tune `ROBOT_FIRE_WARN_CONFIDENCE` / `ROBOT_FIRE_STOP_CONFIDENCE` /
`ROBOT_FIRE_AUTO_STOP` / `ROBOT_FIRE_HALT_CONSECUTIVE`, point
`ROBOT_FIRE_MODEL_PATH` at a better model, or set
`ROBOT_FIRE_MONITOR_ENABLED=0` to disable the monitor entirely. The
Gemini-based `/camera/analyze` object analysis is unchanged and still requires
`GEMINI_API_KEY`.

Set `GEMINI_API_KEY` to enable Gemini. `GEMINI_MODEL` defaults to
`gemini-flash-latest`; `GEMINI_API_URL` defaults to the correct
`https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent`
(don't override it). When a Gemini key is configured it is used before the
OpenAI-compatible client.

## Raspberry Pi Service

A systemd unit is provided at [AI_Robot/ai-robot.service](AI_Robot/ai-robot.service).

It starts the backend through [AI_Robot/start.sh](AI_Robot/start.sh), which creates the virtual environment on first run and keeps stdout/stderr unbuffered for easier monitoring.

## Docker

Build and run the backend with Docker Compose:

```bash
docker compose up --build
```

The container exposes port 8000 and maps the local logs and captures directories for easier inspection.

## Raspberry Pi Boot Behavior

The provided service unit starts the backend automatically after boot via [AI_Robot/start.sh](AI_Robot/start.sh). To make the service resilient on a Pi, ensure the working directory and the log/capture paths are writable and that the unit is enabled:

```bash
sudo systemctl enable ai-robot
sudo systemctl daemon-reload
sudo systemctl restart ai-robot
```

## Notes

This project is intentionally structured as an extensible platform for future capabilities such as YOLO, SLAM, face recognition, and cloud AI integration.
