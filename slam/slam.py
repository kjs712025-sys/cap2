"""A lightweight SLAM implementation based on a simple occupancy grid and odometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import math
import struct
import zlib


@dataclass(slots=True)
class Pose:
    """2D robot pose in meters and radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(slots=True)
class GridCell:
    """A single occupancy-grid cell."""

    occupied: bool = False
    cost: float = 0.0


@dataclass(slots=True)
class SlamMap:
    """Simple occupancy grid map."""

    width: int = 50
    height: int = 50
    resolution: float = 0.05
    cells: list[list[GridCell]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.cells:
            self.cells = [
                [GridCell() for _ in range(self.width)]
                for _ in range(self.height)
            ]


class SimpleSlam:
    """A minimal SLAM module that updates pose from motion commands and marks obstacles."""

    def __init__(self) -> None:
        self.pose = Pose()
        self.map = SlamMap()
        self._last_update: float = 0.0
        self.scan_points: list[dict[str, float]] = []
        self._motion_covariance: float = 0.02

    def update_from_motion(self, vx: float, vy: float, wz: float, dt: float = 0.1) -> Pose:
        """Update pose using a motion model with basic drift compensation."""
        linear_scale = max(0.0, 1.0 - min(abs(vx) + abs(vy), 0.3) * 0.02)
        angular_scale = max(0.0, 1.0 - min(abs(wz), 0.3) * 0.01)

        self.pose.x += vx * dt * linear_scale
        self.pose.y += vy * dt * linear_scale
        self.pose.yaw += wz * dt * angular_scale
        self.pose.yaw = self.pose.yaw % (2.0 * math.pi)
        self._last_update = dt
        self._mark_obstacles()
        return self.pose

    def update_from_lidar(self, scan_points: list[dict[str, float]] | list[tuple[float, float]]) -> None:
        """Update the map using a simple polar-to-Cartesian obstacle projection and clustering."""
        self.scan_points = []
        clusters: list[list[tuple[float, float]]] = []

        for point in scan_points:
            if isinstance(point, tuple):
                angle_deg, distance_m = point
            else:
                angle_deg = float(point.get("angle_deg", 0.0))
                distance_m = float(point.get("distance_m", 0.0))
            if distance_m <= 0.0:
                continue
            angle_rad = math.radians(angle_deg)
            world_x = self.pose.x + distance_m * math.cos(self.pose.yaw + angle_rad)
            world_y = self.pose.y + distance_m * math.sin(self.pose.yaw + angle_rad)
            self.scan_points.append({"angle_deg": angle_deg, "distance_m": distance_m, "x": round(world_x, 3), "y": round(world_y, 3)})

            if not clusters or not self._is_close_to_any_cluster(world_x, world_y, clusters):
                clusters.append([(world_x, world_y)])
            else:
                for cluster in clusters:
                    if self._is_close_to_any_cluster(world_x, world_y, [cluster]):
                        cluster.append((world_x, world_y))
                        break

        for cluster in clusters:
            if len(cluster) >= 2:
                center_x = sum(x for x, _ in cluster) / len(cluster)
                center_y = sum(y for _, y in cluster) / len(cluster)
                self._mark_cluster(center_x, center_y, len(cluster))
            else:
                x, y = cluster[0]
                self._mark_cluster(x, y, 1)

    def _mark_obstacles(self) -> None:
        """Add a simple obstacle hint around the current pose."""
        x_idx = int(self.pose.x / self.map.resolution) + self.map.width // 2
        y_idx = int(self.pose.y / self.map.resolution) + self.map.height // 2
        if 0 <= x_idx < self.map.width and 0 <= y_idx < self.map.height:
            self.map.cells[y_idx][x_idx].occupied = True
            self.map.cells[y_idx][x_idx].cost = 1.0

    def _is_close_to_any_cluster(self, world_x: float, world_y: float, clusters: list[list[tuple[float, float]]]) -> bool:
        """Check whether a point belongs to an existing cluster based on a distance threshold."""
        for cluster in clusters:
            for cx, cy in cluster:
                if math.hypot(world_x - cx, world_y - cy) < 0.15:
                    return True
        return False

    def _mark_cluster(self, world_x: float, world_y: float, weight: int) -> None:
        """Mark a cluster of obstacle observations with a weighted update."""
        x_idx = int(world_x / self.map.resolution) + self.map.width // 2
        y_idx = int(world_y / self.map.resolution) + self.map.height // 2
        if 0 <= x_idx < self.map.width and 0 <= y_idx < self.map.height:
            cell = self.map.cells[y_idx][x_idx]
            cell.occupied = True
            cell.cost = min(1.0, cell.cost + 0.18 * weight)

            for offset in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                ox, oy = offset
                nx = x_idx + ox
                ny = y_idx + oy
                if 0 <= nx < self.map.width and 0 <= ny < self.map.height:
                    neighbor = self.map.cells[ny][nx]
                    neighbor.occupied = neighbor.occupied or cell.cost > 0.4
                    neighbor.cost = max(neighbor.cost, min(1.0, cell.cost - 0.1))

    def to_payload(self) -> dict[str, Any]:
        """Serialize the current state for API responses."""
        grid = []
        for row in self.map.cells:
            grid.append([{"occupied": cell.occupied, "cost": cell.cost} for cell in row])
        return {
            "pose": {"x": round(self.pose.x, 3), "y": round(self.pose.y, 3), "yaw": round(self.pose.yaw, 3)},
            "grid": grid,
            "resolution_m": self.map.resolution,
            "scan_points": self.scan_points,
        }

    def render_image(self) -> bytes:
        """Render the occupancy grid to a PNG image bytestring without external dependencies."""
        width = self.map.width
        height = self.map.height

        def chunk(chunk_type: bytes, payload: bytes) -> bytes:
            return struct.pack("!I", len(payload)) + chunk_type + payload + struct.pack("!I", zlib.crc32(chunk_type + payload) & 0xFFFFFFFF)

        rows: list[bytes] = []
        for y in range(height):
            row = bytearray(b"\x00")
            for x in range(width):
                cell = self.map.cells[y][x]
                if cell.occupied:
                    row.extend((180, 60, 60))
                else:
                    row.extend((240, 240, 240))
            rows.append(bytes(row))

        raw_data = b"".join(rows)
        ihdr = struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0)
        png_bytes = b"\x89PNG\r\n\x1a\n"
        png_bytes += chunk(b"IHDR", ihdr)
        png_bytes += chunk(b"IDAT", zlib.compress(raw_data, level=9))
        png_bytes += chunk(b"IEND", b"")
        return png_bytes
