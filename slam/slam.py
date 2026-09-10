"""Real-time occupancy-grid SLAM.

A lightweight scan-matching-free SLAM: the pose is dead-reckoned from STM32
odometry (or, lacking that, the commanded velocity), and every LiDAR scan is
ray-cast into a fixed log-odds occupancy grid. Each beam clears the free space
it travels through and reinforces the cell it terminates on, so the map
converges on the real environment as the robot moves rather than just
accumulating obstacle blobs.

The navigator drives ``update_from_motion`` + ``update_from_lidar`` on every
perception cycle (~4 Hz), so the map is always live. ``snapshot`` returns a
compact base64 grid for the dashboard SSE stream.
"""

from __future__ import annotations

import base64
import math
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# Grid geometry. 200 cells * 0.05 m = a 10 m x 10 m world, origin at the centre.
GRID = 200
RES = 0.05
_HALF = GRID // 2

# Log-odds update weights and clamps.
L_FREE = -0.4
L_OCC = 0.9
L_MIN = -4.0
L_MAX = 4.0
# Beyond this range a beam only clears free space (an unterminated ray tells us
# nothing about an obstacle at its far end).
MAX_RANGE_M = 6.0

# Classification thresholds for the rendered / streamed map.
OCC_THRESH = 0.7
FREE_THRESH = -0.5

_TRAIL_MAX = 500
_TRAIL_MIN_STEP_M = 0.03


@dataclass(slots=True)
class Pose:
    """2D robot pose in metres and radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(slots=True)
class GridCell:
    """A single occupancy-grid cell (kept for API back-compat)."""

    occupied: bool = False
    cost: float = 0.0


@dataclass(slots=True)
class SlamMap:
    """Occupancy-grid metadata."""

    width: int = GRID
    height: int = GRID
    resolution: float = RES
    cells: list[list[GridCell]] = field(default_factory=list)


class SimpleSlam:
    """Log-odds occupancy-grid SLAM with dead-reckoned pose."""

    def __init__(self) -> None:
        self.pose = Pose()
        self.map = SlamMap()
        self._log = np.zeros((GRID, GRID), dtype=np.float32)
        self._last_update: float = 0.0
        self.scan_points: list[dict[str, float]] = []
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self._motion_covariance: float = 0.02

    # -- pose --------------------------------------------------------------

    def update_from_motion(self, vx: float, vy: float, wz: float, dt: float = 0.1) -> Pose:
        """Integrate a velocity command into the pose with light drift compensation."""
        linear_scale = max(0.0, 1.0 - min(abs(vx) + abs(vy), 0.3) * 0.02)
        angular_scale = max(0.0, 1.0 - min(abs(wz), 0.3) * 0.01)

        # Body-frame velocity rotated into the world frame.
        cos_y, sin_y = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        self.pose.x += (vx * cos_y - vy * sin_y) * dt * linear_scale
        self.pose.y += (vx * sin_y + vy * cos_y) * dt * linear_scale
        self.pose.yaw = (self.pose.yaw + wz * dt * angular_scale) % (2.0 * math.pi)
        self._last_update = dt
        self._push_trail()
        return self.pose

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Adopt an externally supplied pose (e.g. STM32 wheel odometry)."""
        self.pose.x, self.pose.y, self.pose.yaw = float(x), float(y), float(yaw) % (2.0 * math.pi)
        self._push_trail()

    def _push_trail(self) -> None:
        if not self.trail or math.hypot(self.pose.x - self.trail[-1][0], self.pose.y - self.trail[-1][1]) >= _TRAIL_MIN_STEP_M:
            self.trail.append((round(self.pose.x, 3), round(self.pose.y, 3)))
            if len(self.trail) > _TRAIL_MAX:
                self.trail = self.trail[-_TRAIL_MAX:]

    # -- mapping ---------------------------------------------------------

    @staticmethod
    def _to_cell(x: float, y: float) -> tuple[int, int]:
        return int(round(x / RES)) + _HALF, int(round(y / RES)) + _HALF

    def update_from_lidar(
        self, scan_points: list[dict[str, float]] | list[tuple[float, float]]
    ) -> None:
        """Ray-cast every beam into the log-odds grid."""
        self.scan_points = []
        rc, rr = self._to_cell(self.pose.x, self.pose.y)
        if not (0 <= rc < GRID and 0 <= rr < GRID):
            return  # robot has driven off the mapped area

        free_cells: list[np.ndarray] = []
        occ_c: list[int] = []
        occ_r: list[int] = []

        for point in scan_points:
            if isinstance(point, tuple):
                angle_deg, distance_m = point
            else:
                angle_deg = float(point.get("angle_deg", 0.0))
                distance_m = float(point.get("distance_m", 0.0))
            if distance_m <= 0.0:
                continue

            bearing = self.pose.yaw + math.radians(angle_deg)
            hit = min(distance_m, MAX_RANGE_M)
            wx = self.pose.x + hit * math.cos(bearing)
            wy = self.pose.y + hit * math.sin(bearing)
            ec, er = self._to_cell(wx, wy)

            steps = max(1, int(hit / RES))
            cs = np.linspace(rc, ec, steps, endpoint=False).astype(np.int32)
            rs = np.linspace(rr, er, steps, endpoint=False).astype(np.int32)
            free_cells.append(np.stack((rs, cs), axis=1))

            if distance_m <= MAX_RANGE_M and 0 <= ec < GRID and 0 <= er < GRID:
                occ_c.append(ec)
                occ_r.append(er)
                self.scan_points.append(
                    {
                        "angle_deg": round(angle_deg, 1),
                        "distance_m": round(distance_m, 3),
                        "x": round(wx, 3),
                        "y": round(wy, 3),
                    }
                )

        if free_cells:
            fc = np.concatenate(free_cells, axis=0)
            valid = (fc[:, 0] >= 0) & (fc[:, 0] < GRID) & (fc[:, 1] >= 0) & (fc[:, 1] < GRID)
            fc = fc[valid]
            np.add.at(self._log, (fc[:, 0], fc[:, 1]), L_FREE)

        if occ_c:
            np.add.at(self._log, (np.array(occ_r), np.array(occ_c)), L_OCC)

        np.clip(self._log, L_MIN, L_MAX, out=self._log)
        self._last_update = time.time()

    # -- serialisation -------------------------------------------------

    def _packed_grid(self) -> np.ndarray:
        packed = np.zeros((GRID, GRID), dtype=np.uint8)
        packed[self._log < FREE_THRESH] = 1
        packed[self._log > OCC_THRESH] = 2
        return packed

    def snapshot(self) -> dict[str, Any]:
        """Compact live map for the dashboard SSE stream."""
        packed = self._packed_grid()
        return {
            "resolution_m": RES,
            "size": GRID,
            "origin_m": {"x": -_HALF * RES, "y": -_HALF * RES},
            "pose": {"x": round(self.pose.x, 3), "y": round(self.pose.y, 3), "yaw": round(self.pose.yaw, 3)},
            "trail": self.trail[-_TRAIL_MAX:],
            "grid_b64": base64.b64encode(packed.tobytes()).decode("ascii"),
            "occupied_cells": int((packed == 2).sum()),
            "explored_frac": round(float((packed > 0).mean()), 3),
            "scan_points": self.scan_points,
            "updated_at": self._last_update,
        }

    def to_payload(self) -> dict[str, Any]:
        """Full-ish state for the REST endpoints (occupied cells + probability)."""
        prob = 1.0 - 1.0 / (1.0 + np.exp(self._log))
        occ_r, occ_c = np.where(self._log > OCC_THRESH)
        occupied = [
            [int(c), int(r), round(float(prob[r, c]), 2)]
            for r, c in zip(occ_r.tolist(), occ_c.tolist())
        ]
        return {
            "pose": {"x": round(self.pose.x, 3), "y": round(self.pose.y, 3), "yaw": round(self.pose.yaw, 3)},
            "resolution_m": RES,
            "size": GRID,
            "origin_m": {"x": -_HALF * RES, "y": -_HALF * RES},
            "grid": occupied,
            "occupied": occupied,
            "trail": self.trail[-_TRAIL_MAX:],
            "scan_points": self.scan_points,
        }

    def render_image(self) -> bytes:
        """Render the occupancy grid to a PNG bytestring (no external deps)."""

        def chunk(chunk_type: bytes, payload: bytes) -> bytes:
            return (
                struct.pack("!I", len(payload))
                + chunk_type
                + payload
                + struct.pack("!I", zlib.crc32(chunk_type + payload) & 0xFFFFFFFF)
            )

        packed = self._packed_grid()
        # Row 0 at the top: flip so +y points up like a conventional map.
        packed = packed[::-1]
        palette = np.array([[205, 205, 205], [245, 245, 245], [180, 60, 60]], dtype=np.uint8)
        rgb = palette[packed]  # (H, W, 3)

        rows = [b"\x00" + rgb[y].tobytes() for y in range(GRID)]
        raw_data = b"".join(rows)
        ihdr = struct.pack("!IIBBBBB", GRID, GRID, 8, 2, 0, 0, 0)
        png_bytes = b"\x89PNG\r\n\x1a\n"
        png_bytes += chunk(b"IHDR", ihdr)
        png_bytes += chunk(b"IDAT", zlib.compress(raw_data, level=6))
        png_bytes += chunk(b"IEND", b"")
        return png_bytes
