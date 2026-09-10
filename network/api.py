"""FastAPI routes for robot status and commands."""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

from config import RobotConfig
from lidar.obstacle import ObstacleDetector
from network.contracts import error, success
from robot.motion import VelocityCommand
from robot.status import RobotMode, RobotStatus
from utils.logger import get_logger

logger = get_logger("network.api")

router = APIRouter()


@router.get("/slam/status")
async def slam_status(request: Request) -> dict[str, Any]:
    """Expose the current SLAM pose and map summary.

    The navigator's perception loop keeps the map live on every LiDAR scan, so
    this endpoint just serialises the current state without touching hardware.
    """
    return success(request.app.state.slam.to_payload(), message="slam_status")


@router.get("/slam/map")
async def slam_map(request: Request) -> dict[str, Any]:
    """Expose the occupancy-grid map produced by the SLAM module."""
    return success(request.app.state.slam.to_payload(), message="slam_map")


@router.get("/slam/map/image")
async def slam_map_image(request: Request) -> Any:
    """Render the current occupancy grid as a PNG image for visualization."""
    image_bytes = request.app.state.slam.render_image()
    return StreamingResponse(iter([image_bytes]), media_type="image/png")


@router.get("/slam/stream")
async def slam_stream(request: Request) -> StreamingResponse:
    """Server-sent event stream of the live occupancy grid for the dashboard."""
    slam = request.app.state.slam

    async def generate() -> Any:
        while True:
            if await request.is_disconnected():
                break
            yield f"data: {json.dumps(slam.snapshot())}\n\n"
            await asyncio.sleep(1.0)

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/api")
async def api_info(request: Request) -> dict[str, Any]:
    """Service discovery for external clients / apps."""
    config = request.app.state.config
    return success(
        {
            "robot_name": config.robot_name,
            "version": request.app.version,
            "auth_required": bool(config.api_token),
            "auth": "Bearer token in Authorization header, or ?token= query param",
            "streams": {
                "camera_mjpeg": "/camera/stream",
                "navigation_sse": "/navigation/stream",
                "slam_sse": "/slam/stream",
                "fire_sse": "/fire/stream",
                "websocket": "/ws",
                "mqtt": {
                    "base_topic": config.mqtt_base_topic,
                    "port": config.mqtt_port,
                    "fire_topic": f"{config.mqtt_base_topic}/fire",
                },
            },
            "endpoints": {
                "status": "/telemetry",
                "diagnostics": "/diagnostics",
                "navigation_state": "/navigation/state",
                "fire_state": "/fire/state",
                "lidar": "/lidar/scan",
                "mode": "GET /mode · POST /mode {mode: auto|manual}",
                "manual_drive": "POST /motion/command  {vx,vy,wz}  (manual mode only)",
                "voice_drive": "POST /command/voice  (raw audio body, manual mode only)",
                "text_drive": "POST /command/manual  {text}  (manual mode only)",
                "stop": "POST /command/stop",
                "autonomy": "POST /navigation/start | /navigation/stop",
                "emergency_stop": "POST /emergency-stop",
            },
        },
        message="api_info",
    )


@router.get("/health")
async def health(request: Request) -> dict[str, Any]:
    """Health endpoint for monitoring."""
    started_at = getattr(request.app.state, "started_at", time.time())
    uptime_seconds = max(0.0, time.time() - started_at)
    return success(
        {
            "status": "ok",
            "uptime_seconds": round(uptime_seconds, 3),
            "mode": request.app.state.status.mode.value,
        },
        message="healthy",
    )


@router.get("/status")
async def robot_status(request: Request) -> RobotStatus:
    """Expose the current robot status."""
    return request.app.state.status


@router.get("/telemetry")
async def telemetry(request: Request) -> dict[str, Any]:
    """Return a structured telemetry payload suitable for Android clients."""
    status = request.app.state.status
    return success(
        {
            "robot_name": request.app.state.config.robot_name,
            "mode": status.mode.value,
            "mission": status.mission,
            "battery": status.battery,
            "temperature": status.temperature,
            "current_speed": list(status.current_speed),
            "ai_state": status.ai_state,
            "proximity": status.proximity,
            "safe_direction": status.safe_direction,
            "heartbeat": status.heartbeat,
            "fire": (
                "detected"
                if status.metadata.get("fire_detected")
                else status.metadata.get("fire_warning", "clear")
            ),
        },
        message="telemetry",
    )


@router.get("/events")
async def recent_events(request: Request, limit: int = 10) -> dict[str, Any]:
    """Return recently recorded robot events."""
    logger = request.app.state.event_logger
    return success({"events": logger.recent(limit=limit)}, message="events")


@router.get("/alerts")
async def alerts(request: Request) -> dict[str, Any]:
    """Return the latest generated alerts."""
    alert_manager = request.app.state.alert_manager
    return success({"alerts": alert_manager.recent()}, message="alerts")


@router.get("/diagnostics")
async def diagnostics(request: Request) -> dict[str, Any]:
    """Expose component-level diagnostics for operations and monitoring."""
    stm32 = getattr(request.app.state, "stm32", None)
    camera = getattr(request.app.state, "camera", None)
    lidar = getattr(request.app.state, "lidar", None)

    stm32_status = getattr(stm32, "status", None)
    stm32_connected = bool(getattr(stm32_status, "connected", False)) if stm32_status else False

    camera_ready = camera.is_ready() if camera and hasattr(camera, "is_ready") else camera is not None
    lidar_ready = lidar.is_ready() if lidar and hasattr(lidar, "is_ready") else lidar is not None
    llm_ready = request.app.state.llm.is_ready() if getattr(request.app.state, "llm", None) else False

    camera_status = getattr(camera, "status", None) if camera else None
    lidar_status = getattr(lidar, "status", None) if lidar else None

    return success(
        {
            "status": "ok",
            "stm32_connected": stm32_connected,
            "camera_ready": camera_ready,
            "lidar_ready": lidar_ready,
            "heartbeat": request.app.state.status.heartbeat,
            "components": {
                "stm32": {
                    "ready": stm32_connected,
                    "connected": stm32_connected,
                    "armed": bool(getattr(stm32_status, "armed", False)),
                    "battery_voltage": getattr(stm32_status, "battery_voltage", None),
                    "last_error": getattr(stm32_status, "last_error", None),
                },
                "camera": {
                    "ready": camera_ready,
                    "source": getattr(request.app.state.config, "camera_source", "unknown"),
                    "connected": bool(camera_ready),
                    "last_error": getattr(camera_status, "last_error", None) if camera_status else None,
                },
                "lidar": {
                    "ready": lidar_ready,
                    "port": getattr(request.app.state.config, "lidar_port", "unknown"),
                    "connected": bool(lidar_ready),
                    "last_error": getattr(lidar_status, "last_error", None) if lidar_status else None,
                },
                "llm": {
                    "ready": llm_ready,
                    "configured": bool(
                        getattr(request.app.state.config, "openai_api_key", None)
                        or getattr(request.app.state.config, "gemini_api_key", None)
                    ),
                },
            },
        },
        message="diagnostics",
    )


@router.post("/motion/command")
async def motion_command(request: Request) -> dict[str, Any]:
    """Accept a joystick velocity command — manual control mode only."""
    blocked = _manual_blocked(request)
    if blocked is not None:
        return blocked

    payload = await request.json()
    vx = float(payload.get("vx", 0.0))
    vy = float(payload.get("vy", 0.0))
    wz = float(payload.get("wz", 0.0))

    status = request.app.state.status
    status.update_speed(vx, vy, wz)
    status.set_mode(RobotMode.MANUAL)
    request.app.state.slam.update_from_motion(vx, vy, wz)

    lidar = getattr(request.app.state, "lidar", None)
    if lidar is not None:
        scan = await asyncio.to_thread(lidar.read_scan)
        points = [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan]
        await asyncio.to_thread(request.app.state.slam.update_from_lidar, points)

    if hasattr(request.app.state, "stm32"):
        request.app.state.stm32.send_velocity(VelocityCommand(vx, vy, wz))

    return success({"command": {"vx": vx, "vy": vy, "wz": wz}}, message="motion_command")


@router.post("/command/text")
async def text_command(request: Request) -> dict[str, Any]:
    """Route a text command to the assistant layer."""
    payload = await request.json()
    text = str(payload.get("text", ""))
    assistant = request.app.state.assistant
    intent = await assistant.handle_user_command(text)
    return success({"intent": intent}, message="text_command")


@router.post("/command/voice")
async def voice_command(request: Request) -> dict[str, Any]:
    """Transcribe spoken audio with Gemini and drive the robot (timed manual teleop).

    Available to the dashboard and to external apps, but only in ``manual``
    control mode.
    """
    blocked = _manual_blocked(request)
    if blocked is not None:
        return blocked
    audio_data = await request.body()
    if not audio_data:
        return error("empty audio body", code="no_audio")
    mime_type = request.headers.get("content-type", "audio/wav").split(";", 1)[0]
    motion = await request.app.state.assistant.drive_from_voice(audio_data, mime_type=mime_type)
    return success({"motion": motion, "intent": motion}, message="voice_command")


@router.post("/command/manual")
async def manual_command(request: Request) -> dict[str, Any]:
    """Drive the robot from a natural-language command (LLM) — manual mode only."""
    blocked = _manual_blocked(request)
    if blocked is not None:
        return blocked
    payload = await request.json()
    text = str(payload.get("text", ""))
    motion = await request.app.state.assistant.drive_from_text(text)
    return success({"motion": motion}, message="manual_command")


@router.post("/command/stop")
async def command_stop(request: Request) -> dict[str, Any]:
    """Cancel any active manual/voice motion."""
    manual = getattr(request.app.state, "manual", None)
    result = manual.stop() if manual is not None else {"action": "stop"}
    return success(result, message="command_stop")


@router.post("/camera/capture")
async def camera_capture(request: Request) -> dict[str, Any]:
    """Capture an image from the camera and return its saved path."""
    camera = request.app.state.camera
    output_path = "captures/latest.jpg"
    try:
        camera.capture_image(output_path)
    except RuntimeError as exc:
        return error(str(exc), code="camera_unavailable")
    return success({"path": output_path}, message="camera_capture")


@router.post("/camera/analyze")
async def camera_analyze(request: Request) -> dict[str, Any]:
    """Capture a frame and analyze it with the configured vision LLM."""
    frame = request.app.state.camera.read_frame()
    if frame is None:
        return success({"summary": "Camera frame unavailable", "detections": []}, message="camera_analyze")
    analysis = await request.app.state.vision.analyze(frame.image)
    if request.app.state.vision.fire_detected(analysis):
        request.app.state.fire_monitor.stop_for_fire()
    return success(analysis, message="camera_analyze")


@router.get("/lidar/scan")
async def lidar_scan(request: Request) -> dict[str, Any]:
    """Expose the latest LiDAR scan and obstacle analysis."""
    lidar = request.app.state.lidar
    detector = ObstacleDetector()
    scan = await asyncio.to_thread(lidar.read_scan)
    obstacles = detector.analyze(scan)
    status = request.app.state.status
    status.proximity = min(point.distance_m for point in scan) if scan else 2.0
    status.safe_direction = detector.safe_direction(scan)
    return success(
        {
            "scan": [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan],
            "obstacles": [
                {"distance_m": obstacle.distance_m, "angle_deg": obstacle.angle_deg, "warning": obstacle.warning}
                for obstacle in obstacles
            ],
            "safe_direction": status.safe_direction,
            "proximity": status.proximity,
        },
        message="lidar_scan",
    )


def _navigator(request: Request):
    navigator = getattr(request.app.state, "navigator", None)
    if navigator is None:
        raise RuntimeError("navigator not configured")
    return navigator


def _control_mode(request: Request) -> str:
    """Current top-level control mode: 'autonomous', 'manual', or 'error'.

    The navigator is the source of truth: while it is enabled the robot drives
    itself and manual inputs (joystick + LLM/voice commands) are refused.
    """
    if request.app.state.status.mode is RobotMode.ERROR:
        return "error"
    navigator = getattr(request.app.state, "navigator", None)
    if navigator is not None and navigator.enabled:
        return "autonomous"
    return "manual"


def _manual_blocked(request: Request) -> dict[str, Any] | None:
    """Error payload if manual control is not allowed right now, else None."""
    mode = _control_mode(request)
    if mode == "autonomous":
        return error(
            "robot is in autonomous mode — switch to manual control first",
            code="autonomous_active",
        )
    if mode == "error":
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    return None


@router.get("/mode")
async def get_mode(request: Request) -> dict[str, Any]:
    """Report the current control mode (autonomous / manual / error)."""
    return success({"mode": _control_mode(request)}, message="mode")


@router.post("/mode")
async def set_mode(request: Request) -> dict[str, Any]:
    """Switch between autonomous driving and manual control.

    ``{"mode": "auto"}`` enables the reactive navigator; ``{"mode": "manual"}``
    stops it and hands control back to the joystick / LLM commands.
    """
    payload = await request.json()
    requested = str(payload.get("mode", "")).strip().lower()
    navigator = _navigator(request)
    if requested in ("auto", "autonomous", "self", "auto_drive"):
        navigator.enable()
    elif requested in ("manual", "teleop", "hand"):
        navigator.disable()
    else:
        return error("mode must be 'auto' or 'manual'", code="bad_mode")
    return success({"mode": _control_mode(request)}, message="mode_set")


def _navigation_snapshot(request: Request) -> dict[str, Any]:
    """Navigation state plus the current fire-detection level (for the siren)."""
    snapshot = dict(_navigator(request).state)
    fire_monitor = getattr(request.app.state, "fire_monitor", None)
    if fire_monitor is not None:
        snapshot["fire"] = fire_monitor.snapshot()
    return snapshot


@router.get("/fire/state")
async def fire_state(request: Request) -> dict[str, Any]:
    """Current fire-detection payload — the same object published on the
    ``<base>/fire`` MQTT topic, for external app clients that poll over HTTP."""
    fire_monitor = getattr(request.app.state, "fire_monitor", None)
    if fire_monitor is None:
        return error("fire monitor not configured", code="fire_monitor_missing")
    return success(fire_monitor.snapshot(), message="fire_state")


@router.get("/fire/stream")
async def fire_stream(request: Request) -> StreamingResponse:
    """Server-sent event stream of fire detections for external app clients.

    Mirrors the retained ``<base>/fire`` MQTT topic: the current state is sent
    on connect and every subsequent change (new detection, level change, or a
    return to ``clear``) is pushed as it happens.
    """
    fire_monitor = getattr(request.app.state, "fire_monitor", None)
    if fire_monitor is None:
        return error("fire monitor not configured", code="fire_monitor_missing")

    async def generate() -> Any:
        last_ts: float | None = None
        idle = 0
        while True:
            if await request.is_disconnected():
                break
            payload = fire_monitor.snapshot()
            if payload.get("timestamp") != last_ts:
                last_ts = payload.get("timestamp")
                idle = 0
                yield f"data: {json.dumps(payload)}\n\n"
            else:
                idle += 1
                if idle >= 15:  # ~15 s keepalive so proxies hold the connection
                    idle = 0
                    yield ": keepalive\n\n"
            await asyncio.sleep(1.0)

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/navigation/state")
async def navigation_state(request: Request) -> dict[str, Any]:
    """Latest reactive-navigation snapshot: decision, per-sector clearances, scan."""
    return success(_navigation_snapshot(request), message="navigation_state")


@router.post("/navigation/start")
async def navigation_start(request: Request) -> dict[str, Any]:
    """Start issuing autonomous motion commands."""
    _navigator(request).enable()
    return success(_navigator(request).state, message="navigation_started")


@router.post("/navigation/stop")
async def navigation_stop(request: Request) -> dict[str, Any]:
    """Stop the robot and stop issuing autonomous motion commands."""
    _navigator(request).disable()
    return success(_navigator(request).state, message="navigation_stopped")


@router.get("/navigation/stream")
async def navigation_stream(request: Request) -> StreamingResponse:
    """Server-sent event stream of the live navigation snapshot for the dashboard."""
    _navigator(request)  # fail fast if the navigator isn't wired

    async def generate() -> Any:
        while True:
            if await request.is_disconnected():
                break
            yield f"data: {json.dumps(_navigation_snapshot(request))}\n\n"
            await asyncio.sleep(0.25)

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/camera/stream")
async def camera_stream(request: Request) -> StreamingResponse:
    """Stream MJPEG frames to the requester."""
    stream = request.app.state.stream

    async def generate() -> Any:
        async for frame in stream.frame_generator():
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


@router.post("/emergency-stop")
async def emergency_stop(request: Request) -> dict[str, str]:
    """Trigger a software emergency stop."""
    logger.warning("Emergency stop requested via API")
    if hasattr(request.app.state, "stm32"):
        request.app.state.stm32.emergency_stop()
    request.app.state.status.set_mode(RobotMode.ERROR)
    return success({"status": "emergency_stop"}, message="emergency_stop")
