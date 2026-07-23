"""FastAPI routes for robot status and commands."""

from __future__ import annotations

import time
from typing import Any

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

from config import RobotConfig
from lidar.obstacle import ObstacleDetector
from network.contracts import error, success
from robot.status import RobotMode, RobotStatus
from utils.logger import get_logger

logger = get_logger("network.api")

router = APIRouter()


@router.get("/slam/status")
async def slam_status(request: Request) -> dict[str, Any]:
    """Expose the current SLAM pose and map summary."""
    return success(request.app.state.slam.to_payload(), message="slam_status")


@router.get("/slam/map")
async def slam_map(request: Request) -> dict[str, Any]:
    """Expose the occupancy-grid map produced by the lightweight SLAM module."""
    lidar = request.app.state.lidar
    scan = lidar.read_scan()
    points = [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan]
    request.app.state.slam.update_from_lidar(points)
    return success(request.app.state.slam.to_payload(), message="slam_map")


@router.get("/slam/map/image")
async def slam_map_image(request: Request) -> Any:
    """Render the current occupancy grid as a PNG image for visualization."""
    lidar = request.app.state.lidar
    scan = lidar.read_scan()
    points = [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan]
    request.app.state.slam.update_from_lidar(points)
    image_bytes = request.app.state.slam.render_image()
    return StreamingResponse(iter([image_bytes]), media_type="image/png")


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
    """Accept a velocity command and apply it to the runtime state."""
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
        scan = lidar.read_scan()
        points = [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan]
        request.app.state.slam.update_from_lidar(points)

    if hasattr(request.app.state, "stm32"):
        request.app.state.stm32.send_velocity(type("C", (), {"vx": vx, "vy": vy, "wz": wz})())

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
    """Transcribe a raw audio request with Gemini and execute the command."""
    audio_data = await request.body()
    mime_type = request.headers.get("content-type", "audio/wav").split(";", 1)[0]
    intent = await request.app.state.assistant.handle_voice_command(audio_data, mime_type=mime_type)
    return success({"intent": intent}, message="voice_command")


@router.post("/camera/capture")
async def camera_capture(request: Request) -> dict[str, Any]:
    """Capture an image from the camera and return its saved path."""
    camera = request.app.state.camera
    output_path = "captures/latest.jpg"
    camera.capture_image(output_path)
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
    scan = lidar.read_scan()
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


@router.get("/camera/stream")
async def camera_stream(request: Request) -> StreamingResponse:
    """Stream MJPEG frames to the requester."""
    stream = request.app.state.stream

    async def generate() -> Any:
        async for frame in stream.frame_generator(max_frames=1):
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
