"""Robot status models and state tracking."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class RobotMode(str, Enum):
    """High-level execution mode for the robot."""

    IDLE = "idle"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    CHARGING = "charging"
    ERROR = "error"


@dataclass(slots=True)
class RobotStatus:
    """Runtime state for the robot platform."""

    mode: RobotMode = RobotMode.IDLE
    mission: str = "ready"
    battery: float = 100.0
    temperature: float = 35.0
    current_speed: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ai_state: str = "standby"
    last_error: Optional[str] = None
    heartbeat: int = 0
    proximity: float = 2.0
    safe_direction: str = "forward"
    metadata: dict[str, str] = field(default_factory=dict)

    def update_speed(self, vx: float, vy: float, wz: float) -> None:
        """Update the current motion command in the status object."""
        self.current_speed = (vx, vy, wz)

    def set_mode(self, mode: RobotMode) -> None:
        """Set the robot execution mode."""
        self.mode = mode

    def to_payload(self) -> dict[str, object]:
        """Return a JSON-serializable snapshot of the current robot status."""
        return {
            "mode": self.mode.value,
            "mission": self.mission,
            "battery": self.battery,
            "temperature": self.temperature,
            "current_speed": list(self.current_speed),
            "ai_state": self.ai_state,
            "last_error": self.last_error,
            "heartbeat": self.heartbeat,
            "proximity": self.proximity,
            "safe_direction": self.safe_direction,
            "metadata": self.metadata,
        }
