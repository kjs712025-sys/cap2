"""Planner that converts LLM intent into robot actions."""

from __future__ import annotations

from typing import Any

from robot.motion import VelocityCommand
from robot.stm32 import STM32Controller
from utils.logger import get_logger

logger = get_logger("ai.planner")


class Planner:
    """Converts high-level intent into concrete navigation actions."""

    def __init__(self, stm32: STM32Controller) -> None:
        self.stm32 = stm32

    def plan(self, intent: dict[str, Any]) -> VelocityCommand:
        """Translate an LLM intent into a motion command."""
        action = intent.get("intent", "idle")
        if action == "navigate":
            logger.info("Planning navigation for: %s", intent)
            return VelocityCommand(0.2, 0.0, 0.0)
        if action == "stop":
            logger.info("Planning stop action")
            return VelocityCommand(0.0, 0.0, 0.0)
        logger.info("No action planned for intent: %s", action)
        return VelocityCommand(0.0, 0.0, 0.0)

    def execute(self, intent: dict[str, Any]) -> None:
        """Execute a plan by sending a command to the STM32 controller."""
        command = self.plan(intent)
        self.stm32.send_velocity(command)
