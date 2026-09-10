"""Monocular free-space / obstacle estimation for navigation.

No depth sensor: instead we model the drivable floor from a patch just in
front of the robot, then for each image column find how far up that floor
continues before an obstacle edge. Column -> bearing via the camera FOV,
floor height -> a rough ground-plane distance. The result is a per-sector
clearance (front / front_left / front_right) that the navigator fuses with
the LiDAR sectors by taking the smaller (more conservative) value.

This catches things LiDAR misses — obstacles below the scan plane, glass,
dark/absorbing surfaces — at the cost of only approximate range.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from utils.logger import get_logger

logger = get_logger("camera.depth")

_W, _H = 160, 120  # processing resolution


class FloorObstacleEstimator:
    """Estimate forward clearance per sector from a single RGB frame."""

    def __init__(
        self,
        horizontal_fov_deg: float = 62.0,
        near_distance_m: float = 0.15,
        far_distance_m: float = 3.0,
        floor_deviation: float = 3.2,
        horizon_frac: float = 0.55,
    ) -> None:
        self.hfov = horizontal_fov_deg
        self.near = near_distance_m
        self.far = far_distance_m
        self.deviation = floor_deviation
        self.horizon_frac = horizon_frac
        # sector -> (centre_bearing_deg, half_width_deg), matching lidar sectors
        self.sectors = {
            "front": (0.0, 22.0),
            "front_left": (40.0, 18.0),
            "front_right": (-40.0, 18.0),
        }

    def _row_to_distance(self, rows_from_bottom: np.ndarray) -> np.ndarray:
        """Map 'floor rows before an obstacle' to an approximate metric range."""
        span = max(1.0, _H * self.horizon_frac)
        frac = np.clip(rows_from_bottom / span, 0.0, 1.0)
        # ground-plane-ish: distance grows fast near the horizon
        return self.near + (self.far - self.near) * frac ** 1.6

    def estimate(self, image: np.ndarray) -> dict[str, Any]:
        if image is None or image.size == 0:
            return {"available": False}

        small = cv2.resize(image, (_W, _H))
        lab = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2LAB), (5, 5), 0).astype(np.float32)

        # floor reference: trapezoid strip just ahead of the robot
        y0, x0, x1 = int(_H * 0.84), int(_W * 0.34), int(_W * 0.66)
        sample = lab[y0:_H, x0:x1].reshape(-1, 3)
        mean = sample.mean(axis=0)
        std = sample.std(axis=0) + 4.0

        dev = (np.abs(lab - mean) / std).max(axis=2)          # H x W deviation
        not_floor = (dev > self.deviation).astype(np.float32)
        # vertical smoothing so lone speckles don't count as obstacles
        not_floor = cv2.blur(not_floor, (1, 5))
        obstacle = not_floor > 0.55

        floor_frac = float((~obstacle).mean())
        rev = obstacle[::-1, :]
        has = rev.any(axis=0)
        rows_from_bottom = np.where(has, rev.argmax(axis=0), _H).astype(np.float32)
        col_distance = self._row_to_distance(rows_from_bottom)

        cols = np.arange(_W)
        bearing = -((cols + 0.5) / _W - 0.5) * self.hfov  # +left, matches lidar frame

        result: dict[str, Any] = {
            "available": floor_frac > 0.15,
            "floor_frac": round(floor_frac, 3),
            "sectors": {},
        }
        for name, (centre, half) in self.sectors.items():
            m = np.abs(bearing - centre) <= half
            result["sectors"][name] = (
                round(float(col_distance[m].min()), 2) if m.any() else None
            )
        return result
