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

On Raspberry Pi boot, the provided systemd service starts the backend and
enters `autonomous` standby automatically. To install it once:

```bash
sudo cp ai-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ai-robot.service
sudo systemctl start ai-robot.service
```

Set `ROBOT_AUTOSTART_AUTONOMOUS=0` to boot in idle mode instead. The robot
starts stationary; movement begins only when the planner receives a command.

Open `http://<robot-ip>:8000/app/` on a phone to use the voice command app.
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
- POST `/emergency-stop`

### Camera
- POST `/camera/capture`
- GET `/camera/stream`

### LiDAR
- GET `/lidar/scan`

### Voice / AI
- POST `/command/text` with JSON body `{ "text": "go forward" }`
- POST `/command/voice` with raw audio body such as WAV or MP3
- POST `/camera/analyze` to capture a frame and analyze visible objects with Gemini

When `GEMINI_API_KEY` is configured, the backend monitors camera index `0`
(`cam0`) every two seconds. If Gemini returns `fire_detected: true`, or detects
`fire`, `flame`, or `smoke`, the STM32 receives an emergency stop and the robot
status changes to `error` with mission `fire_detected`. Set
`ROBOT_FIRE_MONITOR_ENABLED=0` to disable the background monitor.

To use Gemini for intent extraction, set `GEMINI_API_KEY`. The optional
`GEMINI_MODEL` defaults to `gemini-2.0-flash`, and `GEMINI_API_URL` defaults to
`https://gemini.googleapis.com/v1/models/{model}:generate`. When a Gemini key
is configured, Gemini is used before the OpenAI-compatible client.

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
