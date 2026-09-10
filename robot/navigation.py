"""Reactive autonomous navigation from LiDAR scans.

The navigator always runs while the backend is up: every cycle it reads a
LiDAR scan, breaks it into angular sectors and publishes a situational-
awareness snapshot for the dashboard. It only *commands motion* while it is
enabled and the robot is not in an error/emergency state.

The control policy is deliberately simple and reactive (no global planner):
cruise when the path ahead is clear, steer toward the more open diagonal when
it starts to close in, rotate in place toward the clearest side when blocked,
and back away when boxed in.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, Optional

from camera.depth import FloorObstacleEstimator
from config import RobotConfig
from lidar.lidar import LidarSensor, ScanPoint
from lidar.obstacle import ObstacleDetector
from robot.motion import VelocityCommand
from robot.status import RobotMode, RobotStatus
from robot.stm32 import STM32Controller
from slam.slam import SimpleSlam
from utils.event_log import EventLogger
from utils.logger import get_logger

logger = get_logger("robot.navigation")

# Distance thresholds (metres).
FRONT_CLEAR_M = 0.9    # path ahead is open — cruise
FRONT_STOP_M = 0.4     # obstacle this close ahead — do not advance
SIDE_PASS_M = 0.6      # a side/diagonal is passable above this


class AutonomousNavigator:
    """Reactive obstacle-avoidance controller driven by LiDAR scans."""

    def __init__(
        self,
        lidar: LidarSensor,
        stm32: STM32Controller,
        status: RobotStatus,
        slam: SimpleSlam,
        config: RobotConfig,
        event_logger: Optional[EventLogger] = None,
        camera: Optional[Any] = None,
    ) -> None:
        self.lidar = lidar
        self.stm32 = stm32
        self.status = status
        self.slam = slam
        self.config = config
        self.event_logger = event_logger
        self.camera = camera
        self.detector = ObstacleDetector(
            safety_distance_m=FRONT_STOP_M,
            front_offset_deg=getattr(config, "nav_front_offset_deg", 0.0),
        )
        self.interval = max(0.1, getattr(config, "nav_interval", 0.25))
        self.cruise_speed = min(getattr(config, "nav_cruise_speed", 0.15), config.max_linear_speed)
        self.turn_speed = min(getattr(config, "nav_turn_speed", 0.5), config.max_angular_speed)
        self.use_camera = bool(camera) and getattr(config, "nav_use_camera", True)
        self.camera_estimator = FloorObstacleEstimator(
            horizontal_fov_deg=getattr(config, "nav_camera_fov_deg", 62.0)
        )
        self.camera_interval = max(0.2, getattr(config, "nav_camera_interval", 0.5))
        self._camera_result: dict[str, Any] = {"available": False, "sectors": {}}
        self._camera_at = 0.0
        self.enabled = False
        self._stopped_command_sent = False
        self.state: dict[str, Any] = {
            "enabled": False,
            "decision": "idle",
            "reason": "navigator not started",
            "velocity": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            "sectors": {},
            "camera": {"available": False, "sectors": {}},
            "slam": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "trail_len": 1},
            "proximity": None,
            "scan": [],
            "stm32": {"connected": False, "armed": False, "battery_voltage": None, "last_error": None},
            "timestamp": time.time(),
        }

    # -- lifecycle -----------------------------------------------------------

    def enable(self) -> None:
        """Start issuing motion commands."""
        if self.enabled:
            return
        self.enabled = True
        self.state["enabled"] = True
        self._stopped_command_sent = False
        armed = self.stm32.arm()
        self.status.set_mode(RobotMode.AUTONOMOUS)
        self.status.mission = "autonomous_navigation"
        self.status.ai_state = "navigating"
        logger.info("Autonomous navigation enabled (STM32 armed: %s)", armed)
        if self.event_logger is not None:
            self.event_logger.log("navigation", {"state": "enabled", "stm32_armed": armed})

    def disable(self) -> None:
        """Stop the robot and stop issuing motion commands."""
        was_enabled = self.enabled
        self.enabled = False
        self.state["enabled"] = False
        self._send(VelocityCommand(0.0, 0.0, 0.0))
        self.stm32.disarm()
        if self.status.mode is RobotMode.AUTONOMOUS:
            self.status.set_mode(RobotMode.IDLE)
            self.status.mission = "ready"
            self.status.ai_state = "standby"
        if was_enabled:
            logger.info("Autonomous navigation disabled")
            if self.event_logger is not None:
                self.event_logger.log("navigation", {"state": "disabled"})

    # -- control -----------------------------------------------------------

    def _send(self, command: VelocityCommand) -> None:
        command = command.normalized(self.config.max_linear_speed, self.config.max_angular_speed)
        self.stm32.send_velocity(command)
        self.status.update_speed(command.vx, command.vy, command.wz)

    @staticmethod
    def _clear(sectors: dict[str, dict[str, Any]], name: str) -> float:
        value = sectors.get(name, {}).get("min_distance")
        return float(value) if value is not None else float("inf")

    def _decide(self, sectors: dict[str, dict[str, Any]]) -> tuple[VelocityCommand, str, str]:
        """Return (command, decision, human-readable reason)."""
        front = self._clear(sectors, "front")
        front_left = self._clear(sectors, "front_left")
        front_right = self._clear(sectors, "front_right")
        left = self._clear(sectors, "left")
        right = self._clear(sectors, "right")

        if front > FRONT_CLEAR_M:
            # Open ahead — cruise, with a gentle bias toward the roomier diagonal.
            bias = 0.0
            if min(front_left, front_right) < FRONT_CLEAR_M:
                bias = self.turn_speed * 0.3 * (1.0 if front_left > front_right else -1.0)
            return VelocityCommand(self.cruise_speed, 0.0, bias), "cruise", (
                f"front clear ({front:.2f} m)"
            )

        if front > FRONT_STOP_M:
            # Closing in — slow down and steer toward the clearer front diagonal.
            turn = self.turn_speed * (1.0 if front_left >= front_right else -1.0)
            side = "left" if turn > 0 else "right"
            return VelocityCommand(self.cruise_speed * 0.4, 0.0, turn), "avoid", (
                f"obstacle ahead ({front:.2f} m), steering {side}"
            )

        # Blocked directly ahead — rotate in place toward the clearest side.
        left_room = max(front_left, left)
        right_room = max(front_right, right)
        if max(left_room, right_room) > SIDE_PASS_M:
            if left_room >= right_room:
                return VelocityCommand(0.0, 0.0, self.turn_speed), "turn_left", (
                    f"blocked ahead ({front:.2f} m), turning left ({left_room:.2f} m open)"
                )
            return VelocityCommand(0.0, 0.0, -self.turn_speed), "turn_right", (
                f"blocked ahead ({front:.2f} m), turning right ({right_room:.2f} m open)"
            )

        # Boxed in on three sides — reverse while rotating out.
        return VelocityCommand(-self.cruise_speed * 0.4, 0.0, self.turn_speed), "reverse", (
            f"boxed in (front {front:.2f} m), backing out"
        )

    @staticmethod
    def _downsample(scan: list[ScanPoint], step_deg: float = 2.0) -> list[dict[str, float]]:
        """One nearest point per angular bucket — keeps the streamed payload small
        even when read_scan() returns several rotations of points."""
        buckets: dict[int, float] = {}
        for point in scan:
            key = int(point.angle_deg / step_deg)
            if key not in buckets or point.distance_m < buckets[key]:
                buckets[key] = point.distance_m
        return [
            {"angle_deg": round(key * step_deg, 1), "distance_m": round(dist, 3)}
            for key, dist in sorted(buckets.items())
        ]

    def _proximity(self, sectors: dict[str, dict[str, Any]]) -> Optional[float]:
        values = [
            s["min_distance"]
            for s in sectors.values()
            if s.get("min_distance") is not None
        ]
        return round(min(values), 3) if values else None

    def _fuse_camera(self, sectors: dict[str, dict[str, Any]]) -> dict[str, Any]:
        """Merge the cached camera free-space estimate into the LiDAR sectors
        by taking the smaller clearance (fail-safe)."""
        cam = self._camera_result
        if not cam.get("available") or time.monotonic() - self._camera_at > 2.0:
            return {"available": False}
        used = {}
        for name, cam_d in cam.get("sectors", {}).items():
            if cam_d is None or name not in sectors:
                continue
            lidar_d = sectors[name].get("min_distance")
            if lidar_d is None or cam_d < lidar_d:
                sectors[name] = {"min_distance": cam_d, "count": sectors[name].get("count", 0), "source": "camera"}
                used[name] = round(cam_d, 2)
        return {"available": True, "floor_frac": cam.get("floor_frac"), "overrode": used}

    async def _refresh_camera(self) -> None:
        if not self.use_camera or self.camera is None:
            return
        if time.monotonic() - self._camera_at < self.camera_interval:
            return
        frame = self.camera.read_frame()
        if frame is None:
            return
        try:
            self._camera_result = await asyncio.to_thread(self.camera_estimator.estimate, frame.image)
        except Exception as exc:  # pragma: no cover - vision dependent
            logger.debug("camera obstacle estimate failed: %s", exc)
            self._camera_result = {"available": False, "sectors": {}}
        self._camera_at = time.monotonic()

    def step(self, scan: list[ScanPoint]) -> dict[str, Any]:
        """Run one perception + control cycle and update ``self.state``."""
        self.stm32.poll()  # fold any STM32 telemetry (odometry, battery, faults) in
        sectors = self.detector.sector_distances(scan)
        camera_fused = self._fuse_camera(sectors)
        proximity = self._proximity(sectors)
        blocked = self.status.mode is RobotMode.ERROR

        if self.enabled and not blocked:
            command, decision, reason = self._decide(sectors)
            self._send(command)
            self._stopped_command_sent = False
        elif blocked:
            command, decision, reason = VelocityCommand(0.0, 0.0, 0.0), "halted", "emergency stop active"
            if not self._stopped_command_sent:
                self._send(command)
                self._stopped_command_sent = True
        else:
            command, decision, reason = VelocityCommand(0.0, 0.0, 0.0), "idle", "navigation disabled"

        # Real-time SLAM: dead-reckon the pose from odometry / the last command
        # here; the heavier occupancy-grid ray-cast runs off the event loop in
        # run() via _update_slam().
        odom = self.stm32.status.odometry
        if isinstance(odom, (tuple, list)) and len(odom) == 3:
            self.slam.set_pose(*odom)
        elif command.vx or command.vy or command.wz:
            self.slam.update_from_motion(command.vx, command.vy, command.wz, dt=self.interval)

        if proximity is not None:
            self.status.proximity = proximity
        self.status.safe_direction = self.detector.safe_direction(scan)

        self.state = {
            "enabled": self.enabled,
            "decision": decision,
            "reason": reason,
            "velocity": {"vx": round(command.vx, 3), "vy": round(command.vy, 3), "wz": round(command.wz, 3)},
            "sectors": {
                name: {
                    "min_distance": (round(s["min_distance"], 3) if s["min_distance"] is not None else None),
                    "count": s["count"],
                    "source": s.get("source", "lidar"),
                }
                for name, s in sectors.items()
            },
            "camera": camera_fused,
            "proximity": proximity,
            "safe_direction": self.status.safe_direction,
            "slam": {
                "pose": {
                    "x": round(self.slam.pose.x, 3),
                    "y": round(self.slam.pose.y, 3),
                    "yaw": round(self.slam.pose.yaw, 3),
                },
                "trail_len": len(self.slam.trail),
            },
            "scan": self._downsample(scan),
            "stm32": {
                "connected": self.stm32.status.connected,
                "armed": self.stm32.status.armed,
                "battery_voltage": self.stm32.status.battery_voltage,
                "last_error": self.stm32.status.last_error,
            },
            "timestamp": time.time(),
        }
        return self.state

    def _update_slam(self, scan: list[ScanPoint]) -> None:
        """Ray-cast the scan into the occupancy grid (runs in a worker thread)."""
        self.slam.update_from_lidar(
            [{"angle_deg": p.angle_deg, "distance_m": p.distance_m} for p in scan]
        )

    async def run(self) -> None:
        """Continuous perception/control loop until cancelled."""
        last_heartbeat = 0.0
        while True:
            try:
                await self._refresh_camera()
                scan = await asyncio.to_thread(self.lidar.read_scan)
                self.step(scan)
                await asyncio.to_thread(self._update_slam, scan)
                now = time.monotonic()
                if self.enabled and now - last_heartbeat >= 1.0:
                    self.stm32.send_heartbeat()
                    last_heartbeat = now
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # pragma: no cover - hardware dependent
                logger.warning("Navigation cycle failed: %s", exc)
            await asyncio.sleep(self.interval)
