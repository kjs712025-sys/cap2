"""Timed manual driving — used by voice / text commands.

A command sets a body velocity that is streamed to the STM32 at a steady rate
and then auto-stops once its (bounded) duration elapses. Voice teleop is not
continuous, so every command is self-limiting for safety.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, Optional

from robot.motion import VelocityCommand
from robot.status import RobotMode, RobotStatus
from robot.stm32 import STM32Controller
from utils.logger import get_logger

logger = get_logger("robot.manual")

_TICK_S = 0.15  # velocity refresh rate while a command is active


class ManualController:
    """Executes bounded manual velocity commands."""

    def __init__(
        self,
        stm32: STM32Controller,
        status: RobotStatus,
        config: Any,
        navigator: Optional[Any] = None,
    ) -> None:
        self.stm32 = stm32
        self.status = status
        self.config = config
        self.navigator = navigator
        self._command = VelocityCommand(0.0, 0.0, 0.0)
        self._deadline = 0.0
        self.last: dict[str, Any] = {"action": "idle", "active": False}

    @property
    def active(self) -> bool:
        return time.monotonic() < self._deadline

    def apply(self, vx: float, vy: float, wz: float, duration_s: float, meta: Optional[dict] = None) -> dict[str, Any]:
        """Start (or replace) a timed manual velocity command."""
        max_lin = self.config.max_linear_speed
        max_ang = self.config.max_angular_speed
        vx = max(-max_lin, min(max_lin, vx))
        vy = max(-max_lin, min(max_lin, vy))
        wz = max(-max_ang, min(max_ang, wz))
        duration_s = max(0.0, min(duration_s, self.config.manual_command_timeout))

        if self.navigator is not None and getattr(self.navigator, "enabled", False):
            self.navigator.disable()

        moving = any((vx, vy, wz)) and duration_s > 0.0
        self._command = VelocityCommand(vx, vy, wz)
        self._deadline = time.monotonic() + duration_s if moving else 0.0

        if moving:
            self.status.set_mode(RobotMode.MANUAL)
            self.status.mission = "manual_teleop"
            self.status.ai_state = "manual_voice"
            self.stm32.arm()
        self.stm32.send_velocity(self._command)
        self.status.update_speed(vx, vy, wz)

        self.last = {
            "action": (meta or {}).get("action", "move" if moving else "stop"),
            "vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3),
            "duration_s": round(duration_s, 2),
            "active": moving,
            **({k: meta[k] for k in ("transcript", "speech") if k in (meta or {})}),
        }
        logger.info("Manual command: %s", self.last)
        return self.last

    def stop(self) -> dict[str, Any]:
        self._command = VelocityCommand(0.0, 0.0, 0.0)
        self._deadline = 0.0
        self.stm32.send_velocity(self._command)
        self.status.update_speed(0.0, 0.0, 0.0)
        if self.status.mode is RobotMode.MANUAL:
            self.status.set_mode(RobotMode.IDLE)
            self.status.mission = "ready"
            self.status.ai_state = "standby"
        self.last = {"action": "stop", "active": False}
        return self.last

    async def run(self) -> None:
        """Stream the active command to the STM32; auto-stop at the deadline."""
        stopped = True
        while True:
            try:
                if self.active:
                    self.stm32.send_velocity(self._command)
                    self.stm32.send_heartbeat()
                    stopped = False
                elif not stopped:
                    self.stop()
                    stopped = True
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # pragma: no cover - hardware dependent
                logger.warning("Manual controller tick failed: %s", exc)
            await asyncio.sleep(_TICK_S)
