# AI Robot Backend

A modular, production-oriented backend for an autonomous robot running on Raspberry Pi 5 with Raspberry Pi OS 64-bit.

## Architecture

- Clean Architecture with dependency injection-friendly service boundaries
- OOP design with asyncio-friendly concurrency
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
