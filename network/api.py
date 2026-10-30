"""FastAPI routes for robot status and commands."""

from __future__ import annotations

import asyncio
import base64
import binascii
import json
import math
import time
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Request
from fastapi.responses import FileResponse, StreamingResponse

from config import RobotConfig
from lidar.obstacle import ObstacleDetector
from network.contracts import error, success
from robot.motion import VelocityCommand
from robot.status import RobotMode, RobotStatus
from utils.logger import get_logger

logger = get_logger("network.api")

router = APIRouter()
NAV2_MISSION_MAP_TIMEOUT_S = 90.0
NAV2_MISSION_POLL_INTERVAL_S = 0.25
NAV2_ACTION_READY_MAX_AGE_S = 2.0
NAV2_SCAN_MAX_AGE_S = 1.5
NAV2_MAP_POSE_MAX_AGE_S = 1.0


def _nav2_action_is_ready(state: Any) -> bool:
    return state.config.simulation or (
        state.nav2_action_ready
        and time.monotonic() - state.nav2_action_ready_at <= NAV2_ACTION_READY_MAX_AGE_S
    )


def _fresh_physical_scan(state: Any) -> bool:
    lidar_status = state.lidar.status
    return (
        state.lidar.is_ready()
        and lidar_status.last_scan_hardware
        and lidar_status.last_scan is not None
        and time.monotonic() - lidar_status.last_scan_at <= NAV2_SCAN_MAX_AGE_S
    )


def _fresh_ros_map_pose(state: Any) -> bool:
    process = state.nav2_process.state()
    return (
        process.get("slam_running", False)
        and state.ros_slam_map is not None
        and time.time() - state.ros_slam_map_received_at <= 5.0
        and state.nav2_map_pose is not None
        and time.monotonic() - state.nav2_map_pose_at <= NAV2_MAP_POSE_MAX_AGE_S
    )


def _queue_nav2_goal(state: Any, goal: dict[str, float]) -> int:
    state.nav2_goal_sequence += 1
    state.nav2_cancel_sequence += 1
    state.nav2_goal = {
        "x": goal["x"],
        "y": goal["y"],
        "yaw": math.atan2(math.sin(goal["yaw"]), math.cos(goal["yaw"])),
    }
    return state.nav2_goal_sequence


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
            "simulation": config.simulation,
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
                "mode": "GET /mode · POST /mode {mode: auto|manual} (auto reserves Nav2 control; does not arm)",
                "manual_drive": "POST /motion/command  {vx,vy,wz}  (manual mode only)",
                "voice_drive": "POST /command/voice  (raw audio body, manual mode only)",
                "text_drive": "POST /command/manual  {text}  (manual mode only)",
                "stop": "POST /command/stop",
                "autonomy": "POST /navigation/start | /navigation/stop (Nav2 control aliases; arming is separate)",
                "emergency_stop": "POST /emergency-stop",
                "emergency_clear": "POST /emergency-stop/clear",
                "nav2_control": (
                    "POST /nav2/claim → /nav2/arm → POST /nav2/cmd_vel and "
                    "POST /nav2/heartbeat → POST /nav2/release"
                ),
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
            "simulation": request.app.state.config.simulation,
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
    fire_monitor = request.app.state.fire_monitor
    fire_available = (
        request.app.state.config.fire_monitor_enabled
        and not request.app.state.config.simulation
        and request.app.state.camera.is_ready()
    )
    return success(
        {
            "robot_name": request.app.state.config.robot_name,
            "simulation": request.app.state.config.simulation,
            "mode": status.mode.value,
            "mission": status.mission,
            "battery": status.battery,
            "temperature": status.temperature,
            "current_speed": list(status.current_speed),
            "ai_state": status.ai_state,
            "proximity": status.proximity,
            "safe_direction": status.safe_direction,
            "heartbeat": status.heartbeat,
            "fire": "unavailable" if not fire_available else (
                "detected" if status.metadata.get("fire_detected")
                else status.metadata.get("fire_warning", fire_monitor.snapshot().get("level", "clear"))
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
    fire_monitor_enabled = (
        request.app.state.config.fire_monitor_enabled
        and not request.app.state.config.simulation
    )
    fire_monitor_state = (
        "simulation" if request.app.state.config.simulation
        else "disabled" if not request.app.state.config.fire_monitor_enabled
        else "monitoring" if camera_ready
        else "no_camera"
    )

    return success(
        {
            "status": "ok",
            "simulation": request.app.state.config.simulation,
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
                    "board_status": getattr(stm32_status, "board_status", None),
                    "odometry": getattr(stm32_status, "odometry", None),
                    "last_error": getattr(stm32_status, "last_error", None),
                },
                "camera": {
                    "ready": camera_ready,
                    "source": "simulation" if request.app.state.config.simulation else getattr(request.app.state.config, "camera_source", "unknown"),
                    "connected": bool(camera_ready),
                    "last_error": getattr(camera_status, "last_error", None) if camera_status else None,
                },
                "lidar": {
                    "ready": lidar_ready,
                    "port": "virtual" if request.app.state.config.simulation else getattr(request.app.state.config, "lidar_port", "unknown"),
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
                "fire_monitor": {
                    "enabled": fire_monitor_enabled,
                    "state": fire_monitor_state,
                    "camera_ready": camera_ready,
                    "latest": request.app.state.fire_monitor.snapshot(),
                },
            },
            "system": request.app.state.system_monitor.read(),
        },
        message="diagnostics",
    )


@router.post("/motion/command")
async def motion_command(request: Request) -> dict[str, Any]:
    """Accept a joystick velocity command — manual control mode only."""
    blocked = _manual_blocked(request)
    if blocked is not None:
        return blocked

    try:
        payload = await request.json()
        if not isinstance(payload, dict):
            raise TypeError("command must be an object")
        vx, vy, wz = (float(payload.get(axis, 0.0)) for axis in ("vx", "vy", "wz"))
    except (TypeError, ValueError):
        return error("motion command requires numeric vx, vy, and wz values", code="bad_motion_command")
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        return error("motion command values must be finite", code="bad_motion_command")
    linear_limit = request.app.state.config.max_linear_speed
    angular_limit = request.app.state.config.max_angular_speed
    vx = max(-linear_limit, min(linear_limit, vx))
    vy = max(-linear_limit, min(linear_limit, vy))
    wz = max(-angular_limit, min(angular_limit, wz))

    status = request.app.state.status
    status.set_mode(RobotMode.MANUAL)
    request.app.state.slam.update_from_motion(vx, vy, wz)
    if request.app.state.config.simulation:
        scan = request.app.state.lidar.read_scan()
        points = [{"angle_deg": point.angle_deg, "distance_m": point.distance_m} for point in scan]
        request.app.state.slam.update_from_lidar(points)

    stm32 = getattr(request.app.state, "stm32", None)
    if stm32 is None:
        return error("STM32 controller is unavailable", code="stm32_unavailable")
    if any((vx, vy, wz)):
        if not stm32.status.armed and not stm32.arm():
            return error("STM32 could not arm; manual motion was not sent", code="stm32_unavailable")
        sent = stm32.send_velocity(VelocityCommand(vx, vy, wz))
        if sent:
            stm32.send_heartbeat()
    else:
        sent = stm32.disarm()
    if not sent:
        status.update_speed(0.0, 0.0, 0.0)
        return error("STM32 did not accept the manual motion command", code="stm32_unavailable")

    status.update_speed(vx, vy, wz)
    request.app.state.manual_odom_command = {"vx": vx, "vy": vy, "wz": wz}
    request.app.state.manual_odom_command_at = time.monotonic()

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
        await asyncio.to_thread(camera.capture_image, output_path)
    except RuntimeError as exc:
        return error(str(exc), code="camera_unavailable")
    return success({"path": output_path}, message="camera_capture")


@router.post("/camera/analyze")
async def camera_analyze(request: Request) -> dict[str, Any]:
    """Capture a frame and analyze it with the configured vision LLM."""
    frame = await asyncio.to_thread(request.app.state.camera.read_frame)
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
            "ready": (
                request.app.state.config.simulation
                or (lidar.is_ready() and lidar.status.last_scan_hardware)
            ),
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
    """Current top-level control mode: 'autonomous', 'manual', or 'error'."""
    if request.app.state.status.mode is RobotMode.ERROR:
        return "error"
    if getattr(request.app.state, "nav2_control_active", False):
        return "autonomous"
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

    ``{"mode": "auto"}`` reserves control for Nav2 without arming the motors;
    ``{"mode": "manual"}`` releases Nav2 and returns control to the operator.
    """
    payload = await request.json()
    requested = str(payload.get("mode", "")).strip().lower()
    navigator = _navigator(request)
    if requested in ("auto", "autonomous", "self", "auto_drive"):
        claim_error = _claim_nav2_control(request)
        if claim_error is not None:
            return claim_error
    elif requested in ("manual", "teleop", "hand"):
        if getattr(request.app.state, "nav2_control_active", False):
            if not _release_nav2_control(request):
                return error(
                    "STM32 stop/disarm command could not be confirmed",
                    code="stm32_unavailable",
                )
        navigator.disable()
    else:
        return error("mode must be 'auto' or 'manual'", code="bad_mode")
    return success({"mode": _control_mode(request)}, message="mode_set")


def _claim_nav2_control(request: Request) -> dict[str, Any] | None:
    """Reserve exclusive control for Nav2 without enabling motor output."""
    state = request.app.state
    if state.status.mode is RobotMode.ERROR:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if not state.nav2_process.state()["running"]:
        return error("start the Nav2 stack before selecting autonomous mode", code="nav2_not_running")
    if getattr(state, "nav2_control_active", False):
        return None
    if getattr(state.navigator, "enabled", False):
        return error(
            "stop the reactive navigator before selecting Nav2 control",
            code="reactive_navigation_active",
        )
    manual = getattr(state, "manual", None)
    if manual is not None:
        manual.stop()
    state.manual_odom_command = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    state.manual_odom_command_at = 0.0
    if not state.stm32.disarm():
        return error(
            "STM32 stop/disarm command could not be confirmed; Nav2 control was not claimed",
            code="stm32_unavailable",
        )
    state.status.update_speed(0.0, 0.0, 0.0)
    state.nav2_control_active = True
    state.nav2_last_command_at = time.monotonic()
    state.status.set_mode(RobotMode.AUTONOMOUS)
    state.status.mission = "nav2_control_claimed"
    state.status.ai_state = "nav2_disarmed"
    return None


def _release_nav2_control(request: Request) -> bool:
    """Stop and disarm the STM32 before releasing Nav2's control lease."""
    state = request.app.state
    stm32 = getattr(state, "stm32", None)
    state.nav2_cancel_sequence += 1
    state.nav2_goal = None
    if stm32 is None:
        return False
    if not stm32.disarm():
        return False
    state.nav2_control_active = False
    state.nav2_last_command_at = 0.0
    if hasattr(state, "nav2_patrol"):
        state.nav2_patrol.stop()
        state.nav2_patrol_active_goal_sequence = 0
    state.manual_odom_command = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    state.manual_odom_command_at = 0.0
    return True


@router.post("/nav2/claim")
async def nav2_claim(request: Request) -> dict[str, Any]:
    """Reserve exclusive motor control for the ROS 2 bridge without arming."""
    claim_error = _claim_nav2_control(request)
    if claim_error is not None:
        return claim_error
    return success({"claimed": True, "armed": False}, message="nav2_control_claimed")


@router.post("/nav2/arm")
async def nav2_arm(request: Request) -> dict[str, Any]:
    """Explicitly arm the motor controller after the ROS bridge claims control."""
    state = request.app.state
    if not getattr(state, "nav2_control_active", False):
        return error("claim Nav2 control before arming", code="nav2_not_claimed")
    if not state.nav2_process.state()["running"]:
        return error("Nav2 stack is not running", code="nav2_not_running")
    if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if not state.config.simulation and (
        not state.lidar.is_ready() or not state.lidar.status.last_scan_hardware
    ):
        return error("LiDAR is not ready; motors were not armed", code="lidar_unavailable")
    if not state.config.simulation and (
        not state.nav2_process.state()["slam_running"]
        or state.ros_slam_map is None
        or time.time() - state.ros_slam_map_received_at > 5.0
    ):
        return error("ROS SLAM Toolbox has not produced a fresh map", code="slam_map_unavailable")
    if not state.stm32.arm():
        return error("STM32 is unavailable; motors were not armed", code="stm32_unavailable")
    state.status.set_mode(RobotMode.AUTONOMOUS)
    state.status.mission = "nav2_navigation"
    state.status.ai_state = "nav2_ready"
    state.nav2_last_command_at = time.monotonic()
    return success({"claimed": True, "armed": True}, message="nav2_armed")


@router.post("/nav2/heartbeat")
async def nav2_heartbeat(request: Request) -> dict[str, Any]:
    """Keep the Nav2 control lease and STM32 command watchdog alive."""
    state = request.app.state
    if not getattr(state, "nav2_control_active", False):
        return error("claim Nav2 control before sending a heartbeat", code="nav2_not_claimed")
    if state.stm32.status.armed and not _nav2_action_is_ready(state):
        disarmed = _release_nav2_control(request)
        state.status.update_speed(0.0, 0.0, 0.0)
        if state.status.mode is RobotMode.AUTONOMOUS:
            state.status.set_mode(RobotMode.MANUAL)
            state.status.mission = "ready"
            state.status.ai_state = "standby"
        message = (
            "Nav2 action server became unavailable; control released and motors disarmed"
            if disarmed
            else "Nav2 action server became unavailable; STM32 disarm could not be confirmed"
        )
        return error(message, code="nav2_not_ready")
    if (
        state.stm32.status.armed
        and not state.config.simulation
        and (
            not state.nav2_process.state()["slam_running"]
            or state.ros_slam_map is None
            or time.time() - state.ros_slam_map_received_at > 5.0
        )
    ):
        disarmed = _release_nav2_control(request)
        state.status.update_speed(0.0, 0.0, 0.0)
        if state.status.mode is RobotMode.AUTONOMOUS:
            state.status.set_mode(RobotMode.MANUAL)
            state.status.mission = "ready"
            state.status.ai_state = "standby"
        message = (
            "ROS SLAM map became unavailable; Nav2 control released and motors disarmed"
            if disarmed
            else "ROS SLAM map became unavailable; Nav2 control released but STM32 disarm was not confirmed"
        )
        return error(message, code="slam_map_unavailable")
    if not state.stm32.send_heartbeat():
        return error("STM32 heartbeat failed", code="stm32_unavailable")
    state.nav2_last_command_at = time.monotonic()
    return success({"claimed": True}, message="nav2_heartbeat")


@router.post("/nav2/bridge/status")
async def nav2_bridge_status(request: Request) -> dict[str, Any]:
    """Record whether Nav2's NavigateToPose lifecycle action is active."""
    try:
        payload = await request.json()
    except ValueError:
        return error("bridge status must be a JSON object", code="bad_bridge_status")
    if not isinstance(payload, dict) or not isinstance(payload.get("nav2_action_ready"), bool):
        return error("nav2_action_ready must be a boolean", code="bad_bridge_status")
    state = request.app.state
    state.nav2_action_ready = payload["nav2_action_ready"]
    state.nav2_action_ready_at = time.monotonic()
    return success(
        {"nav2_action_ready": state.nav2_action_ready},
        message="nav2_bridge_status",
    )


@router.get("/nav2/state")
async def nav2_state(request: Request) -> dict[str, Any]:
    state = request.app.state
    process = state.nav2_process.state()
    if state.nav2_patrol.state()["recording"] and (
        state.nav2_control_active
        or state.navigator.enabled
        or not _fresh_ros_map_pose(state)
    ):
        state.nav2_patrol.stop()
    return success(
        {
            **process,
            "control_claimed": bool(state.nav2_control_active),
            "armed": bool(state.stm32.status.armed),
            "nav2_action_ready": _nav2_action_is_ready(state),
            "roam_enabled": bool(state.navigator.enabled),
            "patrol": state.nav2_patrol.state(),
            "map_pose": state.nav2_map_pose if _fresh_ros_map_pose(state) else None,
            "map_pose_age_seconds": (
                max(0.0, time.monotonic() - state.nav2_map_pose_at)
                if state.nav2_map_pose is not None
                else None
            ),
            "simulation": bool(state.config.simulation),
            "stm32_connected": bool(state.stm32.status.connected),
            "goal_sequence": state.nav2_goal_sequence,
            "goal": state.nav2_goal,
            "max_linear_speed_mps": state.nav2_max_linear_speed_mps,
            "lidar_ready": state.lidar.is_ready() and (
                state.config.simulation or state.lidar.status.last_scan_hardware
            ),
            "ros_map_ready": (
                state.ros_slam_map is not None
                and time.time() - state.ros_slam_map_received_at <= 5.0
            ),
            "mission_start": getattr(
                state,
                "nav2_mission_start",
                {"active": False, "stage": "idle"},
            ),
            "ros_map_age_seconds": (
                max(0.0, time.time() - state.ros_slam_map_received_at)
                if state.ros_slam_map is not None
                else None
            ),
        },
        message="nav2_state",
    )


@router.post("/nav2/bridge/pose")
async def nav2_bridge_pose(request: Request) -> dict[str, Any]:
    """Record the latest SLAM-corrected robot pose in the map frame."""
    try:
        payload = await request.json()
    except ValueError:
        return error("map pose must be a JSON object", code="bad_map_pose")
    if not isinstance(payload, dict):
        return error("map pose must be a JSON object", code="bad_map_pose")
    try:
        pose = {
            "x": float(payload["x"]),
            "y": float(payload["y"]),
            "yaw": float(payload["yaw"]),
        }
    except (KeyError, TypeError, ValueError):
        return error("map pose requires finite x, y, and yaw", code="bad_map_pose")
    if not all(math.isfinite(value) for value in pose.values()):
        return error("map pose requires finite x, y, and yaw", code="bad_map_pose")
    if max(abs(pose["x"]), abs(pose["y"])) > 20.0:
        return error("map pose is outside the supported 20 m map area", code="bad_map_pose")

    state = request.app.state
    pose["yaw"] = math.atan2(math.sin(pose["yaw"]), math.cos(pose["yaw"]))
    state.nav2_map_pose = pose
    state.nav2_map_pose_at = time.monotonic()
    patrol = state.nav2_patrol
    if patrol.state()["recording"]:
        if state.nav2_control_active or state.navigator.enabled or _control_mode(request) != "manual":
            patrol.stop()
        else:
            patrol.observe_pose(pose)
    return success({"pose": pose}, message="nav2_map_pose")


@router.post("/nav2/patrol/record/start")
async def nav2_patrol_record_start(request: Request) -> dict[str, Any]:
    state = request.app.state
    if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if state.nav2_control_active or state.navigator.enabled:
        return error("switch to manual mode before recording a patrol route", code="autonomous_active")
    if not state.nav2_process.state()["slam_running"] or not _fresh_ros_map_pose(state):
        return error("start SLAM and wait for a fresh map pose before recording", code="slam_pose_unavailable")
    if not state.config.simulation and (
        not state.stm32.status.connected or not _fresh_physical_scan(state)
    ):
        return error("connected STM32 and fresh physical LiDAR are required", code="hardware_unavailable")
    try:
        state.nav2_patrol.start_recording(state.nav2_map_pose)
    except ValueError as exc:
        return error(str(exc), code="patrol_state_invalid")
    return success(state.nav2_patrol.state(), message="patrol_recording_started")


@router.post("/nav2/patrol/record/stop")
async def nav2_patrol_record_stop(request: Request) -> dict[str, Any]:
    state = request.app.state
    if not _fresh_ros_map_pose(state):
        state.nav2_patrol.stop()
        return error(
            "fresh SLAM map pose unavailable; route recording was cancelled",
            code="slam_pose_unavailable",
        )
    try:
        result = state.nav2_patrol.stop_recording(state.nav2_map_pose)
    except (OSError, ValueError) as exc:
        return error(f"could not save patrol route: {exc}", code="patrol_save_failed")
    return success(result, message="patrol_route_saved")


@router.post("/nav2/patrol/start")
async def nav2_patrol_start(request: Request) -> dict[str, Any]:
    state = request.app.state
    try:
        payload = await request.json()
    except ValueError:
        return error("patrol options must be a JSON object", code="bad_patrol_options")
    if not isinstance(payload, dict) or not isinstance(payload.get("repeat", True), bool):
        return error("patrol repeat option must be a boolean", code="bad_patrol_options")
    if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if state.navigator.enabled:
        return error("stop free roam before starting a route patrol", code="reactive_navigation_active")
    patrol_state = state.nav2_patrol.state()
    if patrol_state["recording"]:
        return error("stop route recording before starting patrol", code="patrol_recording")
    if patrol_state["waypoint_count"] < 2:
        return error("record and save a patrol route first", code="patrol_route_unavailable")

    if not state.nav2_control_active:
        mission_result = await nav2_mission_start(request)
        if not mission_result.get("success"):
            return mission_result
    elif not state.stm32.status.armed:
        arm_result = await nav2_arm(request)
        if not arm_result.get("success"):
            return arm_result

    deadline = time.monotonic() + 10.0
    while not _fresh_ros_map_pose(state) and time.monotonic() < deadline:
        await asyncio.sleep(0.1)
    if not _fresh_ros_map_pose(state):
        disarmed = _release_nav2_control(request)
        message = "A fresh SLAM-corrected map pose was unavailable; motors remain disarmed"
        state.nav2_mission_start = {"active": False, "stage": "failed", "message": message}
        return error(message if disarmed else f"{message}; STM32 disarm could not be confirmed",
                     code="slam_pose_unavailable" if disarmed else "stm32_unavailable")

    try:
        first_goal = state.nav2_patrol.start(
            state.nav2_map_pose,
            repeat=payload.get("repeat", True),
        )
    except ValueError as exc:
        disarmed = _release_nav2_control(request)
        return error(
            str(exc) if disarmed else f"{exc}; STM32 disarm could not be confirmed",
            code="patrol_route_unavailable" if disarmed else "stm32_unavailable",
        )

    sequence = _queue_nav2_goal(state, first_goal)
    state.nav2_patrol_active_goal_sequence = sequence
    state.nav2_mission_start = {
        "active": False,
        "stage": "patrolling",
        "message": state.nav2_patrol.state()["message"],
    }
    return success(
        {"patrol": state.nav2_patrol.state(), "goal": state.nav2_goal},
        message="patrol_started",
    )


@router.post("/nav2/patrol/stop")
async def nav2_patrol_stop(request: Request) -> dict[str, Any]:
    state = request.app.state
    state.nav2_patrol.stop()
    state.nav2_patrol_active_goal_sequence = 0
    state.nav2_goal = None
    state.nav2_cancel_sequence += 1
    if state.nav2_control_active and not _release_nav2_control(request):
        return error("patrol stopped but STM32 disarm could not be confirmed", code="stm32_unavailable")
    state.nav2_mission_start = {
        "active": False,
        "stage": "patrol_stopped",
        "message": state.nav2_patrol.state()["message"],
    }
    return success(state.nav2_patrol.state(), message="patrol_stopped")


@router.post("/nav2/goal/result")
async def nav2_goal_result(request: Request) -> dict[str, Any]:
    state = request.app.state
    try:
        payload = await request.json()
    except ValueError:
        return error("goal result must be a JSON object", code="bad_goal_result")
    if (
        not isinstance(payload, dict)
        or isinstance(payload.get("sequence"), bool)
        or not isinstance(payload.get("sequence"), int)
        or isinstance(payload.get("status"), bool)
        or not isinstance(payload.get("status"), int)
    ):
        return error("goal result requires integer sequence and status", code="bad_goal_result")
    if (
        state.nav2_patrol_active_goal_sequence != payload["sequence"]
        or not state.nav2_patrol.state()["patrolling"]
    ):
        return success({"ignored": True}, message="stale_patrol_goal_result")

    if payload["status"] != 4:
        waypoint = state.nav2_patrol.state()["waypoint_index"] + 1
        message = f"Nav2 could not complete patrol waypoint {waypoint} (status {payload['status']})"
        state.nav2_patrol.fail(message)
        state.nav2_patrol_active_goal_sequence = 0
        state.nav2_goal = None
        state.nav2_cancel_sequence += 1
        disarmed = _release_nav2_control(request)
        state.nav2_mission_start = {"active": False, "stage": "failed", "message": message}
        if not disarmed:
            return error(f"{message}; STM32 disarm could not be confirmed", code="stm32_unavailable")
        return success({"patrol": state.nav2_patrol.state()}, message="patrol_waypoint_failed")

    next_goal = state.nav2_patrol.goal_succeeded()
    if next_goal is None:
        state.nav2_patrol_active_goal_sequence = 0
        state.nav2_goal = None
        state.nav2_cancel_sequence += 1
        disarmed = _release_nav2_control(request)
        message = state.nav2_patrol.state()["message"]
        state.nav2_mission_start = {
            "active": False,
            "stage": "patrol_complete",
            "message": message,
        }
        if not disarmed:
            return error(f"{message}; STM32 disarm could not be confirmed", code="stm32_unavailable")
        return success({"patrol": state.nav2_patrol.state()}, message="patrol_completed")

    sequence = _queue_nav2_goal(state, next_goal)
    state.nav2_patrol_active_goal_sequence = sequence
    return success(
        {"patrol": state.nav2_patrol.state(), "goal": state.nav2_goal},
        message="next_patrol_waypoint",
    )


@router.post("/nav2/speed")
async def nav2_set_speed(request: Request) -> dict[str, Any]:
    """Set the maximum Nav2 translational speed, within the hardware limit."""
    try:
        payload = await request.json()
        if not isinstance(payload, dict):
            raise TypeError("speed settings must be an object")
        value = payload["max_linear_speed_mps"]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("speed must be a number")
        speed = float(value)
    except (KeyError, TypeError, ValueError):
        return error("max_linear_speed_mps must be a number from 0.01 to 1.00", code="bad_speed")
    if not math.isfinite(speed) or not 0.01 <= speed <= 1.00:
        return error("max_linear_speed_mps must be from 0.01 to 1.00", code="bad_speed")
    request.app.state.nav2_max_linear_speed_mps = speed
    return success(
        {"max_linear_speed_mps": speed},
        message="Nav2 maximum linear speed updated",
    )


@router.get("/nav2/slam/map")
async def nav2_slam_map(request: Request) -> dict[str, Any]:
    state = request.app.state
    map_data = state.ros_slam_map
    return success(
        {
            "available": map_data is not None,
            "age_seconds": (
                max(0.0, time.time() - state.ros_slam_map_received_at)
                if map_data is not None
                else None
            ),
            "map": map_data,
        },
        message="nav2_slam_map",
    )


@router.post("/nav2/slam/map")
async def update_nav2_slam_map(request: Request) -> dict[str, Any]:
    """Receive the latest OccupancyGrid from the ROS bridge."""
    try:
        body = await request.body()
        if len(body) > 2_800_000:
            return error("ROS OccupancyGrid payload is too large", code="bad_slam_map")
        payload = json.loads(body)
        if not isinstance(payload, dict):
            raise TypeError("map payload must be an object")
        width_raw, height_raw = payload["width"], payload["height"]
        if (
            isinstance(width_raw, bool)
            or isinstance(height_raw, bool)
            or not isinstance(width_raw, int)
            or not isinstance(height_raw, int)
        ):
            raise TypeError("map dimensions must be integers")
        width, height = width_raw, height_raw
        resolution = float(payload["resolution_m"])
        origin = payload["origin_m"]
        origin_x, origin_y = float(origin["x"]), float(origin["y"])
        origin_yaw = float(origin.get("yaw", 0.0))
        grid_b64 = payload["grid_b64"]
        if not isinstance(grid_b64, str):
            raise TypeError("grid_b64 must be a string")
    except (KeyError, TypeError, ValueError, binascii.Error):
        return error("invalid ROS OccupancyGrid payload", code="bad_slam_map")

    if (
        width < 1
        or height < 1
        or width * height > 2_000_000
        or not math.isfinite(resolution)
        or resolution <= 0
        or not all(math.isfinite(value) for value in (origin_x, origin_y, origin_yaw))
        or len(grid_b64) > 2_666_668
    ):
        return error("ROS OccupancyGrid dimensions or metadata are invalid", code="bad_slam_map")
    try:
        grid = base64.b64decode(grid_b64, validate=True)
    except binascii.Error:
        return error("ROS OccupancyGrid grid data is not valid base64", code="bad_slam_map")
    if len(grid) != width * height:
        return error("ROS OccupancyGrid data length does not match its dimensions", code="bad_slam_map")

    request.app.state.ros_slam_map = {
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin_m": {"x": origin_x, "y": origin_y, "yaw": origin_yaw},
        "grid_b64": grid_b64,
    }
    request.app.state.ros_slam_map_received_at = time.time()
    return success({"received": True}, message="nav2_slam_map_updated")


@router.post("/nav2/slam/start")
async def nav2_slam_start(request: Request) -> dict[str, Any]:
    state = request.app.state
    if state.status.mode is RobotMode.ERROR:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    started, message, process_state = await asyncio.to_thread(state.nav2_process.start_slam)
    if not started:
        return error(message, code="slam_start_failed")
    return success(process_state, message=message)


@router.post("/nav2/slam/stop")
async def nav2_slam_stop(request: Request) -> dict[str, Any]:
    state = request.app.state
    if state.nav2_control_active:
        return error("release Nav2 control before stopping SLAM", code="nav2_active")
    stopped, message, process_state = await asyncio.to_thread(state.nav2_process.stop_slam)
    if not stopped:
        return error(message, code="slam_stop_failed")
    state.ros_slam_map = None
    state.ros_slam_map_received_at = 0.0
    return success(process_state, message=message)


@router.post("/nav2/slam/map/save")
async def save_nav2_slam_map(request: Request) -> dict[str, Any]:
    state = request.app.state
    if not state.nav2_process.state()["slam_running"]:
        return error("start SLAM before saving a ROS map", code="slam_not_running")
    if state.ros_slam_map is None:
        return error("no ROS occupancy map is available yet", code="slam_map_unavailable")
    if time.time() - state.ros_slam_map_received_at > 5.0:
        return error("ROS occupancy map is stale; wait for a fresh map update", code="slam_map_unavailable")
    saved, message, files = await asyncio.to_thread(state.nav2_process.save_map)
    if not saved:
        return error(message, code="slam_map_save_failed")
    return success(
        {
            "yaml_url": "/nav2/slam/map/file/nav2_map.yaml",
            "image_url": "/nav2/slam/map/file/nav2_map.pgm",
            **files,
        },
        message=message,
    )


@router.get("/nav2/slam/map/file/{filename}")
async def download_nav2_slam_map(filename: str, request: Request) -> Any:
    if filename not in {"nav2_map.yaml", "nav2_map.pgm"}:
        return error("unknown saved map file", code="map_file_not_found")
    path = Path(request.app.state.nav2_process.maps_dir) / filename
    if not path.is_file():
        return error("saved map file does not exist", code="map_file_not_found")
    return FileResponse(path, filename=filename)


@router.post("/nav2/stack/start")
async def nav2_stack_start(request: Request) -> dict[str, Any]:
    state = request.app.state
    if state.status.mode is RobotMode.ERROR:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if state.nav2_control_active:
        return error("release Nav2 control before restarting the stack", code="nav2_active")
    started, message, process_state = await asyncio.to_thread(state.nav2_process.start)
    if not started:
        return error(message, code="nav2_start_failed")
    return success(process_state, message=message)


@router.post("/nav2/mission/start")
async def nav2_mission_start(request: Request) -> dict[str, Any]:
    """Prepare Nav2 and SLAM, then arm for an operator-specified Nav2 goal."""
    state = request.app.state
    try:
        payload = await request.json()
        if not isinstance(payload, dict):
            raise TypeError("mission options must be an object")
    except (TypeError, ValueError):
        return error("mission options must be a JSON object", code="bad_mission")

    async with state.nav2_mission_lock:
        if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
            return error("emergency stop is latched — clear it first", code="emergency_latched")
        if state.navigator.enabled or state.nav2_patrol.state()["recording"]:
            return error(
                "stop free roam or route recording before starting a Nav2 mission",
                code="autonomous_mode_conflict",
            )
        if state.nav2_control_active or state.stm32.status.armed:
            return error(
                "Nav2 already owns the motor control; release it before starting another mission",
                code="nav2_active",
            )
        if not state.config.simulation and not state.stm32.status.connected:
            return error("STM32 is unavailable; autonomous mission was not started", code="stm32_unavailable")

        readiness_checks: dict[str, bool] | None = None

        def finish_failure(message: str, code: str) -> dict[str, Any]:
            if state.nav2_control_active:
                disarmed = _release_nav2_control(request)
                state.status.update_speed(0.0, 0.0, 0.0)
                if state.status.mode is RobotMode.AUTONOMOUS:
                    state.status.set_mode(RobotMode.MANUAL)
                    state.status.mission = "ready"
                    state.status.ai_state = "standby"
                if not disarmed:
                    message = f"{message}; STM32 disarm could not be confirmed"
                    code = "stm32_unavailable"
            state.nav2_mission_start = {
                "active": False,
                "stage": "failed",
                "message": message,
                **({"readiness": readiness_checks} if readiness_checks is not None else {}),
            }
            return error(message, code=code)

        state.nav2_mission_start = {"active": True, "stage": "starting_nav2"}
        process_state = await asyncio.to_thread(state.nav2_process.state)
        if not process_state["running"]:
            started, message, _ = await asyncio.to_thread(state.nav2_process.start)
            if not started:
                return finish_failure(message, "nav2_start_failed")

        process_state = await asyncio.to_thread(state.nav2_process.state)
        if not process_state["slam_running"]:
            state.nav2_mission_start = {"active": True, "stage": "starting_slam"}
            map_received_before_start = state.ros_slam_map_received_at
            started, message, _ = await asyncio.to_thread(state.nav2_process.start_slam)
            if not started:
                return finish_failure(message, "slam_start_failed")
        else:
            map_received_before_start = 0.0

        state.nav2_mission_start = {"active": True, "stage": "waiting_for_map"}
        deadline = time.monotonic() + NAV2_MISSION_MAP_TIMEOUT_S
        while True:
            process_state = await asyncio.to_thread(state.nav2_process.state)
            if not process_state["running"] or not process_state["slam_running"]:
                return finish_failure("Nav2 or SLAM stopped during startup", "nav2_start_failed")

            map_is_fresh = state.config.simulation or (
                state.ros_slam_map is not None
                and state.ros_slam_map_received_at > map_received_before_start
                and time.time() - state.ros_slam_map_received_at <= 5.0
            )
            lidar_is_ready = state.config.simulation or (
                _fresh_physical_scan(state)
            )
            readiness_checks = {
                "nav2": bool(process_state["running"]),
                "slam": bool(process_state["slam_running"]),
                "action_server": _nav2_action_is_ready(state),
                "map": map_is_fresh,
                "lidar": lidar_is_ready,
            }
            state.nav2_mission_start = {
                "active": True,
                "stage": "waiting_for_map",
                "readiness": readiness_checks,
            }
            if all(readiness_checks.values()):
                break
            if time.monotonic() >= deadline:
                waiting_for = []
                if not readiness_checks["action_server"]:
                    waiting_for.append("Active Nav2 action server")
                if not readiness_checks["map"]:
                    waiting_for.append("fresh ROS map")
                if not readiness_checks["lidar"]:
                    waiting_for.append("physical LiDAR scan")
                return finish_failure(
                    f"Autonomy startup timed out waiting for {', '.join(waiting_for)} after 90 seconds; motors remain disarmed",
                    "autonomy_readiness_timeout",
                )
            await asyncio.sleep(NAV2_MISSION_POLL_INTERVAL_S)

        state.nav2_mission_start = {"active": True, "stage": "claiming_control"}
        claim_error = _claim_nav2_control(request)
        if claim_error is not None:
            state.nav2_mission_start = {
                "active": False,
                "stage": "failed",
                "message": claim_error.get("message", "Could not claim Nav2 control"),
            }
            return claim_error

        state.nav2_mission_start = {"active": True, "stage": "arming_motors"}
        process_state = await asyncio.to_thread(state.nav2_process.state)
        if not process_state["running"] or not process_state["slam_running"]:
            return finish_failure("Nav2 or SLAM stopped before motor arming", "nav2_start_failed")
        if not _nav2_action_is_ready(state):
            return finish_failure(
                "Nav2 NavigateToPose action server became unavailable; motors remain disarmed",
                "nav2_not_ready",
            )
        if not state.config.simulation and (
            state.ros_slam_map is None
            or time.time() - state.ros_slam_map_received_at > 5.0
            or not _fresh_physical_scan(state)
        ):
            return finish_failure(
                "ROS map or physical LiDAR scan became unavailable; motors remain disarmed",
                "autonomy_not_ready",
            )
        if not state.stm32.arm():
            return finish_failure("STM32 is unavailable; motors were not armed", "stm32_unavailable")
        state.status.set_mode(RobotMode.AUTONOMOUS)
        state.status.mission = "nav2_goal_navigation"
        state.status.ai_state = "awaiting_goal"
        state.nav2_last_command_at = time.monotonic()
        state.nav2_cancel_sequence += 1
        state.nav2_goal = None
        state.nav2_mission_start = {
            "active": False,
            "stage": "ready_for_goal",
            "message": "Nav2 is ready; select a goal on the live map.",
        }
        result = {
            "ready_for_goal": True,
            "claimed": True,
            "armed": True,
        }
        return success(result, message="Nav2 is ready for a goal")


@router.post("/nav2/stack/stop")
async def nav2_stack_stop(request: Request) -> dict[str, Any]:
    state = request.app.state
    state.nav2_patrol.stop()
    state.nav2_patrol_active_goal_sequence = 0
    if state.nav2_control_active:
        if not _release_nav2_control(request):
            return error(
                "STM32 stop/disarm command could not be confirmed; Nav2 stack remains running",
                code="stm32_unavailable",
            )
    state.nav2_cancel_sequence += 1
    state.nav2_goal = None
    stopped, message, process_state = await asyncio.to_thread(state.nav2_process.stop)
    if not stopped:
        return error(message, code="nav2_stop_failed")
    state.status.update_speed(0.0, 0.0, 0.0)
    if state.status.mode is RobotMode.AUTONOMOUS:
        state.status.set_mode(RobotMode.MANUAL)
        state.status.mission = "ready"
        state.status.ai_state = "standby"
    return success(process_state, message=message)


@router.post("/nav2/goal")
async def nav2_goal(request: Request) -> dict[str, Any]:
    state = request.app.state
    if state.nav2_patrol.state()["patrolling"]:
        return error("stop route patrol before sending a manual Nav2 goal", code="patrol_active")
    if not state.nav2_process.state()["running"]:
        return error("start the Nav2 stack before sending a goal", code="nav2_not_running")
    if not state.nav2_control_active or not state.stm32.status.armed:
        return error("claim Nav2 control and arm the STM32 before sending a goal", code="nav2_disarmed")
    if not state.config.simulation and (
        not state.lidar.is_ready() or not state.lidar.status.last_scan_hardware
    ):
        return error("physical LiDAR scan is not ready", code="lidar_unavailable")
    if not state.config.simulation and (
        not state.nav2_process.state()["slam_running"]
        or state.ros_slam_map is None
        or time.time() - state.ros_slam_map_received_at > 5.0
    ):
        return error("ROS SLAM Toolbox has not produced a fresh map", code="slam_map_unavailable")
    try:
        payload = await request.json()
        if not isinstance(payload, dict):
            raise TypeError("goal must be an object")
        x, y, yaw = (float(payload[key]) for key in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return error("goal requires finite x, y, and yaw values", code="bad_goal")
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        return error("goal requires finite x, y, and yaw values", code="bad_goal")
    if max(abs(x), abs(y)) > 20.0:
        return error("goal coordinates must be within 20 m of the map origin", code="goal_out_of_range")
    sequence = _queue_nav2_goal(state, {"x": x, "y": y, "yaw": yaw})
    return success(
        {"sequence": sequence, "goal": state.nav2_goal},
        message="nav2_goal_queued",
    )


@router.post("/nav2/cancel")
async def nav2_cancel(request: Request) -> dict[str, Any]:
    state = request.app.state
    if not state.nav2_control_active:
        return error("Nav2 does not currently own control", code="nav2_not_claimed")
    was_patrolling = state.nav2_patrol.state()["patrolling"]
    state.nav2_cancel_sequence += 1
    state.nav2_goal = None
    state.nav2_patrol.stop()
    state.nav2_patrol_active_goal_sequence = 0
    if was_patrolling and not _release_nav2_control(request):
        return error("patrol cancelled but STM32 disarm could not be confirmed", code="stm32_unavailable")
    return success({"cancel_sequence": state.nav2_cancel_sequence}, message="nav2_goal_cancelled")


@router.get("/nav2/goal/next")
async def nav2_next_goal(request: Request, after: int = 0) -> dict[str, Any]:
    state = request.app.state
    goal = state.nav2_goal
    data = {
        "sequence": state.nav2_goal_sequence,
        "cancel_sequence": state.nav2_cancel_sequence,
        "control_claimed": bool(state.nav2_control_active),
        "armed": bool(state.stm32.status.armed),
    }
    if goal is None or state.nav2_goal_sequence <= after:
        return success(
            {
                "goal": None,
                **data,
            },
            message="no_new_nav2_goal",
        )
    return success(
        {
            "goal": goal,
            **data,
        },
        message="nav2_goal",
    )


@router.get("/nav2/odometry/command")
async def nav2_odometry_command(request: Request) -> dict[str, Any]:
    """Return a fresh manual setpoint for bridge open-loop odometry only."""
    state = request.app.state
    command = getattr(state, "manual_odom_command", {})
    command_at = getattr(state, "manual_odom_command_at", 0.0)
    is_manual = not state.nav2_control_active and state.status.mode is RobotMode.MANUAL
    is_fresh = time.monotonic() - command_at <= 0.5
    if not is_manual or not is_fresh:
        command = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    return success({"command": command}, message="manual_odometry_command")


@router.post("/nav2/release")
async def nav2_release(request: Request) -> dict[str, Any]:
    """Stop and disarm before relinquishing ROS 2 motor-control ownership."""
    state = request.app.state
    if not getattr(state, "nav2_control_active", False):
        return success({"claimed": False, "armed": False}, message="nav2_control_released")
    released = _release_nav2_control(request)
    state.status.update_speed(0.0, 0.0, 0.0)
    if state.status.mode is RobotMode.AUTONOMOUS:
        state.status.set_mode(RobotMode.MANUAL)
        state.status.mission = "ready"
        state.status.ai_state = "standby"
    if not released:
        return error("STM32 stop/disarm command could not be confirmed", code="stm32_unavailable")
    return success({"claimed": False, "armed": False}, message="nav2_control_released")


@router.post("/nav2/cmd_vel")
async def nav2_cmd_vel(request: Request) -> dict[str, Any]:
    """Forward one bounded ROS 2 velocity setpoint without touching LiDAR."""
    state = request.app.state
    if not getattr(state, "nav2_control_active", False):
        return error("claim Nav2 control before sending velocity", code="nav2_not_claimed")
    if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
        return error("emergency stop is latched", code="emergency_latched")
    if not state.stm32.status.armed:
        return error("arm the STM32 through /nav2/arm before sending velocity", code="stm32_disarmed")

    try:
        payload = await request.json()
        if not isinstance(payload, dict):
            raise TypeError("velocity payload must be an object")
        values = tuple(float(payload.get(key, 0.0)) for key in ("vx", "vy", "wz"))
    except (TypeError, ValueError):
        return error("vx, vy, and wz must be finite numbers", code="bad_velocity")
    if not all(math.isfinite(value) for value in values):
        return error("vx, vy, and wz must be finite numbers", code="bad_velocity")

    vx, vy = values[:2]
    linear_speed = math.hypot(vx, vy)
    max_linear_speed = min(1.00, state.nav2_max_linear_speed_mps)
    if linear_speed > max_linear_speed:
        scale = max_linear_speed / linear_speed
        vx *= scale
        vy *= scale
    wz = max(-0.25, min(0.25, values[2]))
    if not state.stm32.send_velocity(VelocityCommand(vx, vy, wz)):
        return error("STM32 velocity command failed", code="stm32_unavailable")
    state.nav2_last_command_at = time.monotonic()
    state.status.update_speed(vx, vy, wz)
    return success({"command": {"vx": vx, "vy": vy, "wz": wz}}, message="nav2_cmd_vel")


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


@router.post("/navigation/roam/start")
async def navigation_roam_start(request: Request) -> dict[str, Any]:
    """Start the LiDAR-reactive free-roam mode without claiming Nav2 control."""
    state = request.app.state
    if state.status.mode is RobotMode.ERROR or state.stm32.emergency_latched:
        return error("emergency stop is latched — clear it first", code="emergency_latched")
    if state.nav2_control_active:
        return error("release Nav2 control before starting free roam", code="nav2_active")
    if state.nav2_patrol.state()["recording"] or state.nav2_patrol.state()["patrolling"]:
        return error("stop route recording or patrol before starting free roam", code="patrol_active")
    if not state.config.simulation and not state.stm32.status.connected:
        return error("STM32 is unavailable; free roam was not started", code="stm32_unavailable")
    if not state.config.simulation and not _fresh_physical_scan(state):
        return error("a fresh physical LiDAR scan is required for free roam", code="lidar_unavailable")
    if state.status.metadata.get("fire_alert"):
        return error("free roam is unavailable while a fire alert is active", code="fire_alert_active")

    navigator = _navigator(request)
    navigator.enable()
    if not state.config.simulation and not state.stm32.status.armed:
        navigator.disable()
        return error("STM32 could not be armed; free roam was not started", code="stm32_unavailable")
    return success({"roam_enabled": True}, message="LiDAR free roam started")


@router.post("/navigation/roam/stop")
async def navigation_roam_stop(request: Request) -> dict[str, Any]:
    _navigator(request).disable()
    return success({"roam_enabled": False}, message="LiDAR free roam stopped")


@router.post("/navigation/start")
async def navigation_start(request: Request) -> dict[str, Any]:
    """Select Nav2 autonomous control without arming the motor controller."""
    claim_error = _claim_nav2_control(request)
    if claim_error is not None:
        return claim_error
    return success({"claimed": True, "armed": False}, message="nav2_control_claimed")


@router.post("/navigation/stop")
async def navigation_stop(request: Request) -> dict[str, Any]:
    """Release Nav2 control and stop the robot."""
    if getattr(request.app.state, "nav2_control_active", False):
        if not _release_nav2_control(request):
            return error(
                "STM32 stop/disarm command could not be confirmed",
                code="stm32_unavailable",
            )
    _navigator(request).disable()
    request.app.state.nav2_patrol.stop()
    return success({"claimed": False, "armed": False}, message="nav2_control_released")


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
async def emergency_stop(request: Request) -> dict[str, Any]:
    """Trigger a software emergency stop."""
    logger.warning("Emergency stop requested via API")
    navigator = getattr(request.app.state, "navigator", None)
    if navigator is not None:
        navigator.disable()  # stop autonomous driving; operator must re-arm it
    request.app.state.nav2_control_active = False
    request.app.state.nav2_last_command_at = 0.0
    request.app.state.nav2_cancel_sequence += 1
    request.app.state.nav2_goal = None
    request.app.state.nav2_patrol.stop()
    request.app.state.nav2_patrol_active_goal_sequence = 0
    if hasattr(request.app.state, "stm32"):
        request.app.state.stm32.emergency_stop()
    request.app.state.status.set_mode(RobotMode.ERROR)
    return success({"status": "emergency_stop"}, message="emergency_stop")


@router.post("/emergency-stop/clear")
async def clear_emergency_stop(request: Request) -> dict[str, Any]:
    """Release the emergency-stop latch after an operator safety check.

    Unlatches the STM32, clears the fire flags, and returns the robot to
    ``manual`` mode (stationary) so the operator can drive or re-arm autonomy.
    """
    state = request.app.state
    stm32 = getattr(state, "stm32", None)
    if stm32 is not None:
        stm32.clear_emergency_stop()
    status = state.status
    status.metadata.pop("fire_detected", None)
    status.metadata.pop("fire_alert", None)
    status.last_error = None
    if status.mode is RobotMode.ERROR:
        status.set_mode(RobotMode.MANUAL)
        status.mission = "ready"
        status.ai_state = "standby"
    fire_monitor = getattr(state, "fire_monitor", None)
    if fire_monitor is not None:
        fire_monitor.triggered = False
    logger.info("Emergency stop latch cleared via API")
    return success({"mode": _control_mode(request)}, message="emergency_cleared")
