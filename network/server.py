"""Server composition root for the robot backend."""

from __future__ import annotations

import asyncio
import time

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from ai.assistant import RobotAssistant
from ai.llm import LLMService
from ai.planner import Planner
from camera.camera import Camera
from camera.stream import CameraStream
from camera.vision import VisionService
from config import RobotConfig
from lidar.lidar import LidarSensor
from network.api import router as api_router
from network.mqtt import MqttPublisher
from network.websocket import RobotSocketManager
from robot.manual import ManualController
from robot.navigation import AutonomousNavigator
from robot.stm32 import STM32Controller
from robot.status import RobotMode, RobotStatus
from safety.fire_monitor import FireMonitor
from slam.slam import SimpleSlam
from utils.alerts import AlertManager
from utils.event_log import EventLogger
from utils.logger import configure_logging
from utils.logger import get_logger

logger = get_logger("network.server")


def create_app(config: RobotConfig) -> FastAPI:
    """Create and wire the FastAPI application."""
    configure_logging(config)

    app = FastAPI(title=config.app_name, version="0.1.0")

    origins = [o.strip() for o in config.cors_origins.split(",") if o.strip()]
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins or ["*"],
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    _OPEN_PATHS = ("/health", "/api", "/app", "/docs", "/openapi.json", "/redoc")

    @app.middleware("http")
    async def _api_token_guard(request: Request, call_next):
        token = config.api_token
        if token and not request.url.path.startswith(_OPEN_PATHS):
            supplied = request.headers.get("authorization", "")
            supplied = supplied[7:] if supplied.lower().startswith("bearer ") else request.query_params.get("token", "")
            if supplied != token:
                return JSONResponse({"success": False, "error": "unauthorized"}, status_code=401)
        return await call_next(request)

    app.mount("/app", StaticFiles(directory="app", html=True), name="robot_app")

    stm32 = STM32Controller(config)
    camera = Camera(config)
    lidar = LidarSensor(config)
    llm = LLMService(config)
    vision = VisionService(config.supported_objects, llm=llm, fire_model_path=config.fire_model_path)
    planner = Planner(stm32)
    status = RobotStatus()
    manual = ManualController(stm32=stm32, status=status, config=config)
    assistant = RobotAssistant(
        llm=llm, planner=planner, camera=camera, lidar=lidar, stm32=stm32, manual=manual
    )
    app.state.manual = manual
    mqtt = MqttPublisher(
        host=config.mqtt_host,
        port=config.mqtt_port,
        base_topic=config.mqtt_base_topic,
        enabled=config.mqtt_enabled,
    )
    app.state.mqtt = mqtt
    socket_manager = RobotSocketManager()
    event_logger = EventLogger(file_path="logs/events.jsonl")
    alert_manager = AlertManager()
    slam = SimpleSlam()

    app.state.config = config
    app.state.stm32 = stm32
    app.state.camera = camera
    app.state.lidar = lidar
    app.state.llm = llm
    app.state.vision = vision
    app.state.planner = planner
    app.state.assistant = assistant
    app.state.socket_manager = socket_manager
    app.state.status = status
    if config.autostart_autonomous:
        app.state.status.set_mode(RobotMode.AUTONOMOUS)
        app.state.status.mission = "autonomous_ready"
        app.state.status.ai_state = "autonomous_standby"
    app.state.started_at = time.time()
    app.state.event_logger = event_logger
    app.state.alert_manager = alert_manager
    app.state.slam = slam
    fire_monitor = FireMonitor(
        camera=camera,
        vision=vision,
        stm32=stm32,
        status=app.state.status,
        interval=config.fire_monitor_interval,
        warn_confidence=config.fire_warn_confidence,
        stop_confidence=config.fire_stop_confidence,
        auto_stop=config.fire_auto_stop,
        halt_on_detect=config.fire_halt_on_detect,
        event_logger=event_logger,
        mqtt=mqtt,
    )
    app.state.fire_monitor = fire_monitor
    app.state.fire_monitor_task = None
    stream = CameraStream(camera, fire_monitor=fire_monitor)
    app.state.stream = stream
    navigator = AutonomousNavigator(
        lidar=lidar,
        stm32=stm32,
        status=app.state.status,
        slam=slam,
        config=config,
        event_logger=event_logger,
        camera=camera,
    )
    fire_monitor.navigator = navigator
    manual.navigator = navigator
    app.state.navigator = navigator
    app.state.manual_task = None
    app.state.navigator_task = None
    stm32.initialize()
    camera.initialize()
    lidar.initialize()

    app.include_router(api_router)

    async def _emit_status_loop() -> None:
        while True:
            await asyncio.sleep(1.0)
            if not getattr(app.state, "socket_manager", None):
                break
            app.state.status.heartbeat += 1
            app.state.event_logger.log("heartbeat", {"heartbeat": app.state.status.heartbeat})
            alerts = alert_manager.evaluate(app.state.status.to_payload())
            if alerts:
                app.state.event_logger.log("alert", {"alerts": alerts})
            hardware_payload = {
                "stm32": {
                    "connected": getattr(app.state.stm32, "status", None).connected if getattr(app.state, "stm32", None) else False,
                    "last_error": getattr(getattr(app.state.stm32, "status", None), "last_error", None),
                },
                "camera": {
                    "ready": app.state.camera.is_ready() if getattr(app.state, "camera", None) else False,
                    "source": getattr(config, "camera_source", "unknown"),
                },
                "lidar": {
                    "ready": app.state.lidar.is_ready() if getattr(app.state, "lidar", None) else False,
                    "port": getattr(config, "lidar_port", "unknown"),
                },
            }
            status_payload = {
                "event": "status",
                "robot_name": config.robot_name,
                "status": app.state.status.to_payload(),
                "hardware": hardware_payload,
            }
            await app.state.socket_manager.broadcast_status(status_payload)
            mqtt.publish("status", status_payload, retain=True)
            nav = navigator.state
            mqtt.publish(
                "navigation",
                {k: nav[k] for k in ("enabled", "decision", "reason", "velocity", "sectors", "proximity", "safe_direction") if k in nav},
                retain=True,
            )
            mqtt.publish("fire", fire_monitor.snapshot(), retain=True)

    @app.on_event("startup")
    async def start_status_stream() -> None:
        mqtt.start()
        asyncio.create_task(_emit_status_loop())
        if config.fire_monitor_enabled:
            # Local YOLOv8 detection (camera/vision.py) — no cloud API key needed.
            app.state.fire_monitor_task = asyncio.create_task(fire_monitor.run())
            logger.info("Fire monitor enabled on camera index %s", config.camera_index)
        app.state.navigator_task = asyncio.create_task(navigator.run())
        app.state.manual_task = asyncio.create_task(manual.run())
        logger.info("Autonomous navigation perception loop started")
        if config.nav_autostart:
            # Give the LiDAR / camera a few seconds to enumerate and deliver a
            # first real scan before the robot starts driving itself on boot.
            async def _autostart_navigation() -> None:
                await asyncio.sleep(max(0.0, config.nav_autostart_delay))
                if app.state.status.mode is RobotMode.ERROR:
                    logger.warning("Skipping navigation auto-start: robot is in an error state")
                    return
                navigator.enable()
                logger.info(
                    "Autonomous navigation auto-started (%.0fs after boot)",
                    config.nav_autostart_delay,
                )

            app.state.nav_autostart_task = asyncio.create_task(_autostart_navigation())

    @app.on_event("shutdown")
    async def stop_background_tasks() -> None:
        for name in ("fire_monitor_task", "navigator_task", "manual_task", "nav_autostart_task"):
            task = getattr(app.state, name, None)
            if task is not None:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
        navigator.disable()
        mqtt.stop()

    @app.websocket("/ws")
    async def websocket_endpoint(websocket) -> None:
        await socket_manager.connect(websocket)
        try:
            await socket_manager.broadcast_status(
                {
                    "event": "connected",
                    "robot_name": config.robot_name,
                    "status": app.state.status.to_payload(),
                    "hardware": {
                        "stm32": {
                            "connected": getattr(app.state.stm32, "status", None).connected if getattr(app.state, "stm32", None) else False,
                            "last_error": getattr(getattr(app.state.stm32, "status", None), "last_error", None),
                        },
                        "camera": {
                            "ready": app.state.camera.is_ready() if getattr(app.state, "camera", None) else False,
                            "source": getattr(config, "camera_source", "unknown"),
                        },
                        "lidar": {
                            "ready": app.state.lidar.is_ready() if getattr(app.state, "lidar", None) else False,
                            "port": getattr(config, "lidar_port", "unknown"),
                        },
                    },
                }
            )
            while True:
                await websocket.receive_text()
        except Exception as exc:  # pragma: no cover - defensive path
            logger.exception("WebSocket failure: %s", exc)
        finally:
            socket_manager.disconnect(websocket)

    return app
