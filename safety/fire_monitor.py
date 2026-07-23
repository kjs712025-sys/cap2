"""Background fire detection and emergency stopping."""

from __future__ import annotations

import asyncio

from camera.camera import Camera
from camera.vision import VisionService
from robot.status import RobotMode, RobotStatus
from robot.stm32 import STM32Controller
from utils.logger import get_logger

logger = get_logger("safety.fire")


class FireMonitor:
    """Periodically analyze cam0 and stop the robot when fire is detected."""

    def __init__(
        self,
        camera: Camera,
        vision: VisionService,
        stm32: STM32Controller,
        status: RobotStatus,
        interval: float = 2.0,
    ) -> None:
        self.camera = camera
        self.vision = vision
        self.stm32 = stm32
        self.status = status
        self.interval = max(0.5, interval)
        self.triggered = False

    async def check_once(self) -> dict[str, object] | None:
        """Analyze one frame and stop the robot if fire is present."""
        frame = self.camera.read_frame()
        if frame is None:
            return None
        analysis = await self.vision.analyze(frame.image)
        if self.vision.fire_detected(analysis):
            self.stop_for_fire()
        return analysis

    def stop_for_fire(self) -> None:
        """Latch the emergency stop so motion cannot continue after detection."""
        self.stm32.emergency_stop()
        self.status.update_speed(0.0, 0.0, 0.0)
        self.status.set_mode(RobotMode.ERROR)
        self.status.mission = "fire_detected"
        self.status.ai_state = "fire_emergency_stop"
        self.status.last_error = "Fire or smoke detected by Gemini vision"
        self.status.metadata["fire_detected"] = "true"
        if not self.triggered:
            logger.critical("Fire detected by Gemini vision; emergency stop engaged")
            self.triggered = True

    async def run(self) -> None:
        """Continuously monitor the configured camera until cancelled."""
        while True:
            try:
                await self.check_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # pragma: no cover - hardware/network dependent
                logger.warning("Fire monitor check failed: %s", exc)
            await asyncio.sleep(self.interval)