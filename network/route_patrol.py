"""Persist and advance operator-taught Nav2 patrol routes."""

from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any


class RoutePatrol:
    SAMPLE_DISTANCE_M = 0.75
    SAMPLE_YAW_RAD = math.radians(30.0)
    MIN_ROUTE_LENGTH_M = 1.0
    MAX_ROUTE_POINTS = 1000

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.RLock()
        self._route: list[dict[str, float]] = []
        self._recorded: list[dict[str, float]] = []
        self._mode = "idle"
        self._message = "No patrol route saved"
        self._repeat = False
        self._waypoint_index = 0
        self._load()

    @staticmethod
    def _pose(value: Any) -> dict[str, float]:
        if not isinstance(value, dict):
            raise ValueError("map pose must be an object")
        try:
            x, y, yaw = (float(value[key]) for key in ("x", "y", "yaw"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("map pose requires finite x, y, and yaw") from exc
        if not all(math.isfinite(number) for number in (x, y, yaw)):
            raise ValueError("map pose requires finite x, y, and yaw")
        if max(abs(x), abs(y)) > 20.0:
            raise ValueError("map pose is outside the supported 20 m map area")
        return {"x": x, "y": y, "yaw": math.atan2(math.sin(yaw), math.cos(yaw))}

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            points = payload["points"]
            if not isinstance(points, list):
                raise ValueError("patrol route points must be a list")
            route = [self._pose(point) for point in points]
            if len(route) < 2:
                raise ValueError("saved patrol route must contain at least two points")
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Could not load patrol route from {self.path}: {exc}") from exc
        self._route = route
        self._mode = "saved"
        self._message = f"Loaded saved patrol route ({len(route)} points)"

    @staticmethod
    def _distance(a: dict[str, float], b: dict[str, float]) -> float:
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    @staticmethod
    def _yaw_delta(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def _route_length(self, points: list[dict[str, float]]) -> float:
        return sum(self._distance(a, b) for a, b in zip(points, points[1:]))

    def start_recording(self, pose: Any) -> None:
        current = self._pose(pose)
        with self._lock:
            if self._mode == "patrolling":
                raise ValueError("stop the active patrol before recording a route")
            self._recorded = [current]
            self._mode = "recording"
            self._message = "Recording manual driving path"

    def observe_pose(self, pose: Any) -> None:
        current = self._pose(pose)
        with self._lock:
            if self._mode != "recording":
                return
            previous = self._recorded[-1]
            if (
                self._distance(previous, current) < self.SAMPLE_DISTANCE_M
                and self._yaw_delta(previous["yaw"], current["yaw"]) < self.SAMPLE_YAW_RAD
            ):
                return
            if len(self._recorded) >= self.MAX_ROUTE_POINTS:
                self._mode = "failed"
                self._message = "Route recording stopped at the 1000-point limit"
                return
            self._recorded.append(current)
            self._message = f"Recording route ({len(self._recorded)} points)"

    def stop_recording(self, pose: Any) -> dict[str, Any]:
        final_pose = self._pose(pose)
        with self._lock:
            if self._mode != "recording":
                raise ValueError("route recording is not active")
            if (
                self._distance(self._recorded[-1], final_pose) >= 0.15
                or self._yaw_delta(self._recorded[-1]["yaw"], final_pose["yaw"]) >= math.radians(10)
            ):
                if len(self._recorded) >= self.MAX_ROUTE_POINTS:
                    self._mode = "failed"
                    self._message = "Route recording stopped at the 1000-point limit"
                    raise ValueError(self._message)
                self._recorded.append(final_pose)
            length = self._route_length(self._recorded)
            if len(self._recorded) < 2 or length < self.MIN_ROUTE_LENGTH_M:
                self._mode = "saved" if self._route else "failed"
                self._message = "Recorded route must be at least 1 m long"
                self._recorded = []
                raise ValueError(self._message)

            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = self.path.with_suffix(f"{self.path.suffix}.tmp")
            try:
                temporary_path.write_text(
                    json.dumps({"version": 1, "points": self._recorded}, indent=2),
                    encoding="utf-8",
                )
                temporary_path.replace(self.path)
            except OSError:
                raise
            self._route = self._recorded
            self._recorded = []
            self._mode = "saved"
            self._message = f"Saved patrol route ({len(self._route)} points, {length:.1f} m)"
            return self.state()

    def start(self, pose: Any, repeat: bool = True) -> dict[str, float]:
        current = self._pose(pose)
        with self._lock:
            if self._mode == "recording":
                raise ValueError("stop route recording before starting patrol")
            if len(self._route) < 2:
                raise ValueError("record and save a patrol route first")
            self._repeat = bool(repeat)
            self._waypoint_index = min(
                range(len(self._route)),
                key=lambda index: self._distance(current, self._route[index]),
            )
            self._mode = "patrolling"
            self._message = (
                f"Patrolling waypoint {self._waypoint_index + 1} of {len(self._route)}"
            )
            return dict(self._route[self._waypoint_index])

    def goal_succeeded(self) -> dict[str, float] | None:
        with self._lock:
            if self._mode != "patrolling":
                return None
            self._waypoint_index += 1
            if self._waypoint_index >= len(self._route):
                if not self._repeat:
                    self._mode = "complete"
                    self._message = "Patrol route completed"
                    return None
                self._waypoint_index = 0
            self._message = (
                f"Patrolling waypoint {self._waypoint_index + 1} of {len(self._route)}"
            )
            return dict(self._route[self._waypoint_index])

    def fail(self, message: str) -> None:
        with self._lock:
            self._mode = "failed"
            self._message = message

    def stop(self) -> None:
        with self._lock:
            if self._mode == "recording":
                self._recorded = []
                self._mode = "saved" if self._route else "idle"
                self._message = "Route recording cancelled"
            elif self._mode == "patrolling":
                self._mode = "stopped"
                self._message = "Patrol stopped"

    def state(self) -> dict[str, Any]:
        with self._lock:
            points = self._recorded if self._mode == "recording" else self._route
            return {
                "mode": self._mode,
                "message": self._message,
                "point_count": len(points),
                "route_length_m": round(self._route_length(points), 2),
                "waypoint_index": self._waypoint_index,
                "waypoint_count": len(self._route),
                "repeat": self._repeat,
                "recording": self._mode == "recording",
                "patrolling": self._mode == "patrolling",
            }

    def route(self) -> list[dict[str, float]]:
        with self._lock:
            return [dict(point) for point in self._route]
