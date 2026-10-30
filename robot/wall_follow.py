"""Low-speed right-wall following and reactive obstacle avoidance."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from lidar.lidar import ScanPoint
from lidar.obstacle import ObstacleDetector
from robot.motion import VelocityCommand


@dataclass(frozen=True, slots=True)
class WallFollowDecision:
    command: VelocityCommand
    phase: str
    reason: str


class RightWallFollower:
    """Keep a right-hand wall nearby while moving forward and avoiding obstacles."""

    def __init__(
        self,
        wall_clearance_m: float = 0.8,
        forward_speed_mps: float = 0.04,
        front_offset_deg: float = 0.0,
    ) -> None:
        self.wall_clearance_m = max(wall_clearance_m, 0.8)
        self.forward_speed_mps = min(forward_speed_mps, 0.04)
        self.detector = ObstacleDetector(front_offset_deg=front_offset_deg)

    MIN_WALL_CLEARANCE_M = 0.65
    OBSTACLE_DISTANCE_M = 1.0
    ESCAPE_CLEARANCE_M = 0.8
    ESCAPE_SPEED_MPS = 0.02

    @staticmethod
    def _distance(sectors: dict[str, dict[str, Any]], name: str) -> float | None:
        value = sectors.get(name, {}).get("min_distance")
        return float(value) if value is not None else None

    @staticmethod
    def _clearance(value: float | None) -> float:
        return value if value is not None else 0.0

    def step(self, scan: list[ScanPoint]) -> WallFollowDecision:
        if not scan:
            return WallFollowDecision(
                VelocityCommand(0.0, 0.0, 0.0),
                "blocked",
                "LiDAR scan is empty; stopping autonomy for safety",
            )

        sectors = self.detector.sector_distances(scan)
        front = self._distance(sectors, "front")
        if front is None:
            return WallFollowDecision(
                VelocityCommand(0.0, 0.0, 0.0),
                "blocked",
                "No forward LiDAR coverage; stopping autonomy for safety",
            )

        front_readings = [
            self._distance(sectors, name)
            for name in ("front", "front_left", "front_right")
        ]
        front_clearance = min(
            (distance for distance in front_readings if distance is not None),
            default=math.inf,
        )
        left_room = self._clearance(self._distance(sectors, "left"))
        right_room = self._clearance(self._distance(sectors, "right"))

        if front_clearance <= self.OBSTACLE_DISTANCE_M:
            if max(left_room, right_room) < self.ESCAPE_CLEARANCE_M:
                return WallFollowDecision(
                    VelocityCommand(0.0, 0.0, 0.0),
                    "blocked",
                    f"Obstacle ahead ({front_clearance:.2f} m); no measured lateral escape path",
                )
            if left_room >= right_room:
                vy, side = self.ESCAPE_SPEED_MPS, "left"
            else:
                vy, side = -self.ESCAPE_SPEED_MPS, "right"
            return WallFollowDecision(
                VelocityCommand(0.0, vy, 0.0),
                "avoid_shift",
                f"Obstacle ahead ({front_clearance:.2f} m); shifting slowly {side} into measured clearance",
            )

        right = self._distance(sectors, "right")
        if right is None:
            return WallFollowDecision(
                VelocityCommand(0.0, 0.0, 0.0),
                "blocked",
                "No right-side LiDAR coverage; stopping autonomy for safety",
            )
        if right > 1.5:
            if max(left_room, right_room) < self.ESCAPE_CLEARANCE_M:
                return WallFollowDecision(
                    VelocityCommand(0.0, 0.0, 0.0),
                    "blocked",
                    "Right wall not found and no measured lateral escape path",
                )
            return WallFollowDecision(
                VelocityCommand(0.0, 0.0, -0.06),
                "search_wall",
                "Searching clockwise for a right-hand wall",
            )

        if right < self.MIN_WALL_CLEARANCE_M:
            if left_room < self.ESCAPE_CLEARANCE_M:
                return WallFollowDecision(
                    VelocityCommand(0.0, 0.0, 0.0),
                    "blocked",
                    f"Right wall too close ({right:.2f} m); left escape path is unknown or too narrow",
                )
            return WallFollowDecision(
                VelocityCommand(0.0, self.ESCAPE_SPEED_MPS, 0.0),
                "avoid_shift",
                f"Right wall too close ({right:.2f} m); shifting slowly left into measured clearance",
            )
        if right > self.wall_clearance_m + 0.1:
            return WallFollowDecision(
                VelocityCommand(0.0, -0.02, 0.0),
                "approach_wall",
                f"Moving toward right wall ({right:.2f} m)",
            )

        return WallFollowDecision(
            VelocityCommand(self.forward_speed_mps, 0.0, 0.0),
            "follow_wall",
            f"Following right wall ({right:.2f} m clearance)",
        )
