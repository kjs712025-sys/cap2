"""Obstacle detection helpers based on LiDAR scans."""

from __future__ import annotations

from dataclasses import dataclass

from lidar.lidar import ScanPoint

# Angular sectors relative to the robot's forward direction (degrees, 0 = front,
# positive = counter-clockwise / left). Each entry: (center, half-width).
SECTORS: dict[str, tuple[float, float]] = {
    "front": (0.0, 25.0),
    "front_left": (45.0, 20.0),
    "front_right": (-45.0, 20.0),
    "left": (90.0, 25.0),
    "right": (-90.0, 25.0),
    "rear": (180.0, 35.0),
}


def angular_distance(a: float, b: float) -> float:
    """Smallest absolute angle between two bearings, in degrees (0-180)."""
    diff = abs((a - b) % 360.0)
    return min(diff, 360.0 - diff)


@dataclass(slots=True)
class ObstacleReport:
    """Represents a detected obstacle."""

    distance_m: float
    angle_deg: float
    warning: bool


class ObstacleDetector:
    """Inspect LiDAR data and derive obstacle warnings and clear directions."""

    def __init__(self, safety_distance_m: float = 0.8, front_offset_deg: float = 0.0) -> None:
        self.safety_distance_m = safety_distance_m
        self.front_offset_deg = front_offset_deg

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

    def sector_distances(self, scan: list[ScanPoint]) -> dict[str, dict[str, float | int | None]]:
        """Nearest obstacle distance and point count per angular sector.

        Sensor bearings are shifted by ``front_offset_deg`` so the "front"
        sector lines up with the robot's actual forward direction. Returns
        ``min_distance = None`` for sectors with no returns.
        """
        result: dict[str, dict[str, float | int | None]] = {
            name: {"min_distance": None, "count": 0} for name in SECTORS
        }
        for point in scan:
            bearing = (point.angle_deg - self.front_offset_deg) % 360.0
            # Fold into -180..180 so front-right sectors (centered at -45/-90) match.
            if bearing > 180.0:
                bearing -= 360.0
            for name, (center, half_width) in SECTORS.items():
                if angular_distance(bearing, center) <= half_width:
                    entry = result[name]
                    entry["count"] = int(entry["count"]) + 1
                    current = entry["min_distance"]
                    if current is None or point.distance_m < current:
                        entry["min_distance"] = point.distance_m
        return result

    def safe_direction(self, scan: list[ScanPoint]) -> str:
        """Pick the clearest heading: forward / left / right / backward / stop."""
        if not scan:
            return "forward"

        sectors = self.sector_distances(scan)

        def clearance(name: str) -> float:
            value = sectors[name]["min_distance"]
            return float(value) if value is not None else float("inf")

        front = clearance("front")
        if front > self.safety_distance_m:
            return "forward"

        options = {
            "left": min(clearance("front_left"), clearance("left")),
            "right": min(clearance("front_right"), clearance("right")),
            "backward": clearance("rear"),
        }
        best_direction, best_clearance = max(options.items(), key=lambda item: item[1])
        if best_clearance <= self.safety_distance_m:
            return "stop"
        return best_direction
