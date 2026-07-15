"""LiDAR interface for YDLiDAR-style sensors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from config import RobotConfig
from utils.logger import get_logger

logger = get_logger("lidar")


@dataclass(slots=True)
class ScanPoint:
    """Single LiDAR range measurement."""

    angle_deg: float
    distance_m: float


@dataclass(slots=True)
class LidarStatus:
    """Current LiDAR state."""

    connected: bool = False
    last_scan: Optional[list[ScanPoint]] = None
    last_error: Optional[str] = None


class LidarSensor:
    """Placeholder LiDAR implementation for future hardware integration."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.status = LidarStatus()
        self._scan_buffer: list[ScanPoint] = []

    def connect(self) -> None:
        """Connect to the LiDAR device."""
        self.status.connected = True
        self.status.last_error = None
        logger.info("LiDAR connected")

    def is_ready(self) -> bool:
        """Return whether the LiDAR is available for scanning."""
        return self.status.connected

    def initialize(self) -> bool:
        """Attempt to initialize the LiDAR sensor and return whether it is ready."""
        self.connect()
        return self.is_ready()

    def disconnect(self) -> None:
        """Disconnect the LiDAR device."""
        self.status.connected = False
        logger.info("LiDAR disconnected")

    def parse_scan_text(self, text: str) -> list[ScanPoint]:
        """Parse scan payloads such as '0:1.2,30:2.5' or '0=1.2;30=2.5'."""
        points: list[ScanPoint] = []
        if not text:
            return points

        for token in text.replace(";", ",").split(","):
            if not token:
                continue
            token = token.strip()
            if not token:
                continue
            separator = ":" if ":" in token else "="
            if separator not in token:
                continue
            angle_str, distance_str = token.split(separator, 1)
            try:
                points.append(ScanPoint(angle_deg=float(angle_str), distance_m=float(distance_str)))
            except ValueError:
                logger.warning("Skipping malformed LiDAR scan token: %s", token)
        return points

    def _generate_simulated_scan(self) -> list[ScanPoint]:
        """Generate a deterministic scan for local and test environments."""
        return [
            ScanPoint(angle_deg=-30.0, distance_m=2.0),
            ScanPoint(angle_deg=0.0, distance_m=1.2),
            ScanPoint(angle_deg=30.0, distance_m=2.5),
        ]

    def read_scan(self) -> list[ScanPoint]:
        """Return a scan from the connected sensor or a simulated fallback."""
        scan = self._generate_simulated_scan()
        self._scan_buffer = scan
        self.status.last_scan = scan
        return scan
