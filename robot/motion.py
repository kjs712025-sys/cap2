"""Motion planning and velocity conversion utilities."""

from __future__ import annotations

from dataclasses import dataclass

from utils.math_utils import clamp


@dataclass(slots=True)
class VelocityCommand:
    """Velocity command for the robot base."""

    vx: float
    vy: float
    wz: float

    def normalized(self, max_linear: float, max_angular: float) -> "VelocityCommand":
        """Clamp the command within configured motion limits."""
        return VelocityCommand(
            vx=clamp(self.vx, -max_linear, max_linear),
            vy=clamp(self.vy, -max_linear, max_linear),
            wz=clamp(self.wz, -max_angular, max_angular),
        )


class MotionController:
    """Converts navigation intents into STM32-compatible control commands."""

    def __init__(self, max_linear: float, max_angular: float) -> None:
        self.max_linear = max_linear
        self.max_angular = max_angular

    def command_from_intent(self, vx: float, vy: float, wz: float) -> VelocityCommand:
        """Build a normalized motion command."""
        return VelocityCommand(vx=vx, vy=vy, wz=wz).normalized(
            self.max_linear,
            self.max_angular,
        )

    def to_uart_payload(self, command: VelocityCommand) -> str:
        """Serialize command as the STM32 wire protocol."""
        payload = f"V,{command.vx:.3f},{command.vy:.3f},{command.wz:.3f}"
        checksum = sum(ord(ch) for ch in payload) % 256
        return f"{payload}*{checksum:02X}"
