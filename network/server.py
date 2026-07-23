"""Server composition root for the robot backend."""

from __future__ import annotations

import asyncio
import time

from fastapi import FastAPI
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
from network.websocket import RobotSocketManager
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
    app.mount("/app", StaticFiles(directory="app", html=True), name="robot_app")

    stm32 = STM32Controller(config)
    camera = Camera(config)
    lidar = LidarSensor(config)
    llm = LLMService(config)
    vision = VisionService(config.supported_objects, llm=llm)
    planner = Planner(stm32)
    assistant = RobotAssistant(llm=llm, planner=planner, camera=camera, lidar=lidar, stm32=stm32)
    stream = CameraStream(camera)
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
    app.state.stream = stream
    app.state.socket_manager = socket_manager
    app.state.status = RobotStatus()
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
    )
    app.state.fire_monitor = fire_monitor
    app.state.fire_monitor_task = None
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
            await app.state.socket_manager.broadcast_status(
                {
                    "event": "status",
                    "robot_name": config.robot_name,
                    "status": app.state.status.to_payload(),
                    "hardware": hardware_payload,
                }
            )

    @app.on_event("startup")
    async def start_status_stream() -> None:
        asyncio.create_task(_emit_status_loop())
        if config.fire_monitor_enabled and config.gemini_api_key:
            app.state.fire_monitor_task = asyncio.create_task(fire_monitor.run())
            logger.info("Gemini fire monitor enabled on camera index %s", config.camera_index)

    @app.on_event("shutdown")
    async def stop_fire_monitor() -> None:
        task = getattr(app.state, "fire_monitor_task", None)
        if task is not None:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

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
