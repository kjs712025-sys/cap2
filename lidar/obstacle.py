"""Obstacle detection helpers based on LiDAR scans."""

from __future__ import annotations

from dataclasses import dataclass

from lidar.lidar import ScanPoint


@dataclass(slots=True)
class ObstacleReport:
    """Represents a detected obstacle."""

    distance_m: float
    angle_deg: float
    warning: bool


class ObstacleDetector:
    """Inspect LiDAR data and derive obstacle warnings."""

    def __init__(self, safety_distance_m: float = 0.8) -> None:
        self.safety_distance_m = safety_distance_m

    def analyze(self, scan: list[ScanPoint]) -> list[ObstacleReport]:
        """Create obstacle warnings from ranges."""
        reports: list[ObstacleReport] = []
        for point in scan:
            if point.distance_m <= self.safety_distance_m:
                reports.append(
                    ObstacleReport(
                        distance_m=point.distance_m,
                        angle_deg=point.angle_deg,
                        warning=True,
                    )
                )
        return reports

    def safe_direction(self, scan: list[ScanPoint]) -> str:
        """Choose a safe motion direction from scan data."""
        if not scan:
            return "forward"
        closest = min(scan, key=lambda item: item.distance_m)
        return "forward" if closest.distance_m > self.safety_distance_m else "stop"
