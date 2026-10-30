"""ROS 2 bridge between Nav2 and the existing OmniBot backend."""

from __future__ import annotations

import base64
import json
import math
import os
import queue
import threading
import time
import urllib.error
import urllib.request
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener


_SCAN_BINS = 360
_SCAN_MIN_ANGLE = -math.pi
_SCAN_INCREMENT = 2.0 * math.pi / _SCAN_BINS
_RANGE_MIN_M = 0.12
_RANGE_MAX_M = 12.0
_MAX_LINEAR_MPS = 1.00
_MAX_ANGULAR_RPS = 0.25
_MAX_MANUAL_ODOM_LINEAR_MPS = 0.5
_MAX_MANUAL_ODOM_ANGULAR_RPS = 1.0
_COMMAND_TIMEOUT_S = 0.30
_SCAN_TIMEOUT_S = 1.2
_BACKEND_REQUEST_TIMEOUT_S = 2.0
_LIDAR_REQUEST_TIMEOUT_S = 2.0


class BackendRejectedError(RuntimeError):
    """A completed backend response rejected a bridge request."""


class OmniBotBackendBridge(Node):
    """Translate ROS scan/velocity traffic to the backend's HTTP API."""

    def __init__(self) -> None:
        super().__init__("omnibot_backend_bridge")
        self.declare_parameter("backend_url", "http://127.0.0.1:8000")
        self.declare_parameter("api_token", os.getenv("ROBOT_API_TOKEN", ""))
        self.declare_parameter("scan_poll_period", 0.1)
        self.declare_parameter("lidar_x", 0.0)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", 0.16)
        self.declare_parameter("lidar_yaw", 0.0)
        self.backend_url = str(self.get_parameter("backend_url").value).rstrip("/")
        self.api_token = str(self.get_parameter("api_token").value)
        self.scan_poll_period = max(0.05, float(self.get_parameter("scan_poll_period").value))
        self._lidar_xyz = tuple(
            float(self.get_parameter(name).value)
            for name in ("lidar_x", "lidar_y", "lidar_z")
        )
        self._lidar_yaw = float(self.get_parameter("lidar_yaw").value)

        self._claimed = False
        self._armed = False
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._nav2_action_ready = False
        self._readiness_request_pending = False
        self._manual_odom_vx = 0.0
        self._manual_odom_vy = 0.0
        self._manual_odom_wz = 0.0
        self._manual_odom_received_at = 0.0
        self._last_command_at = 0.0
        self._last_scan_at = 0.0
        self._goal_sequence = 0
        self._cancel_sequence = 0
        self._goal_events: queue.Queue[dict[str, Any]] = queue.Queue()
        self._goal_results: queue.Queue[dict[str, int]] = queue.Queue()
        self._goal_handle = None
        self._goal_handle_sequence = 0
        self._map_pose: dict[str, float] | None = None
        self._map_pose_captured_at = 0.0
        self._last_map_pose_report_at = 0.0
        self._last_odom_at = time.monotonic()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._lock = threading.Lock()
        self._stop_scanner = threading.Event()
        self._stop_map_worker = threading.Event()
        self._stop_goal_worker = threading.Event()
        self._map_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self._last_warning_at: dict[str, float] = {}

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos
        )
        self._cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._bt_navigator_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self._claim_srv = self.create_service(Trigger, "/omnibot/claim_control", self._claim)
        self._arm_srv = self.create_service(Trigger, "/omnibot/arm", self._arm)
        self._release_srv = self.create_service(Trigger, "/omnibot/release_control", self._release)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_timer = self.create_timer(0.05, self._publish_odometry)
        self._heartbeat_timer = self.create_timer(0.2, self._send_heartbeat)
        self._goal_timer = self.create_timer(0.1, self._dispatch_goal_events)
        self._readiness_timer = self.create_timer(0.5, self._check_nav2_readiness)
        self._map_pose_timer = self.create_timer(0.1, self._capture_map_pose)

        self._tf_broadcaster = TransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id = "lidar_link"
        static_tf.transform.translation.x = self._lidar_xyz[0]
        static_tf.transform.translation.y = self._lidar_xyz[1]
        static_tf.transform.translation.z = self._lidar_xyz[2]
        static_tf.transform.rotation.z = math.sin(self._lidar_yaw * 0.5)
        static_tf.transform.rotation.w = math.cos(self._lidar_yaw * 0.5)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._static_tf_broadcaster.sendTransform(static_tf)

        self._scanner = threading.Thread(target=self._scan_loop, daemon=True)
        self._scanner.start()
        self._map_worker = threading.Thread(target=self._map_forward_loop, daemon=True)
        self._map_worker.start()
        self._goal_worker = threading.Thread(target=self._goal_poll_loop, daemon=True)
        self._goal_worker.start()
        self.get_logger().warning(
            "Open-loop odometry is enabled; it integrates commanded velocity and will drift."
        )

    def _request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        if self.api_token:
            headers["Authorization"] = f"Bearer {self.api_token}"
        request = urllib.request.Request(
            self.backend_url + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            timeout = (
                _LIDAR_REQUEST_TIMEOUT_S
                if path == "/lidar/scan"
                else _BACKEND_REQUEST_TIMEOUT_S
            )
            with urllib.request.urlopen(request, timeout=timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, ValueError) as exc:
            raise RuntimeError(f"backend request {method} {path} failed: {exc}") from exc
        if not isinstance(body, dict) or body.get("success") is not True:
            message = body.get("message", "invalid backend response") if isinstance(body, dict) else "invalid backend response"
            raise BackendRejectedError(f"backend rejected {method} {path}: {message}")
        return body

    def _log_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at.get(key, 0.0) >= 5.0:
            self._last_warning_at[key] = now
            self.get_logger().warning(message)

    def _on_map(self, message: OccupancyGrid) -> None:
        info = message.info
        if (
            info.width < 1
            or info.height < 1
            or info.width * info.height > 2_000_000
            or len(message.data) != info.width * info.height
            or not math.isfinite(float(info.resolution))
            or info.resolution <= 0
        ):
            self._log_throttled("invalid_map", "Ignoring invalid or oversized ROS OccupancyGrid.")
            return
        origin = info.origin
        origin_yaw = math.atan2(
            2.0 * (origin.orientation.w * origin.orientation.z + origin.orientation.x * origin.orientation.y),
            1.0 - 2.0 * (origin.orientation.y**2 + origin.orientation.z**2),
        )
        payload = {
            "width": int(info.width),
            "height": int(info.height),
            "resolution_m": float(info.resolution),
            "origin_m": {"x": float(origin.position.x), "y": float(origin.position.y), "yaw": origin_yaw},
            "grid_b64": base64.b64encode(bytes(value & 0xFF for value in message.data)).decode("ascii"),
        }
        try:
            self._map_queue.put_nowait(payload)
        except queue.Full:
            try:
                self._map_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._map_queue.put_nowait(payload)
            except queue.Full:
                self._log_throttled("map_queue", "ROS map forwarding queue is full; latest map update skipped.")

    def _map_forward_loop(self) -> None:
        while not self._stop_map_worker.is_set():
            try:
                payload = self._map_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._request("POST", "/nav2/slam/map", payload)
            except RuntimeError as exc:
                self._log_throttled("map_publish", f"Could not forward the ROS SLAM map: {exc}")

    def _goal_poll_loop(self) -> None:
        next_readiness_report = 0.0
        while not self._stop_goal_worker.is_set():
            try:
                data = self._request(
                    "GET",
                    f"/nav2/goal/next?after={self._goal_sequence}",
                )["data"]
                control_claimed = bool(data["control_claimed"])
                armed = bool(data["armed"])
                with self._lock:
                    self._claimed = control_claimed
                    self._armed = armed if control_claimed else False
                    if not self._claimed or not self._armed:
                        self._vx = self._vy = self._wz = 0.0
                cancel_sequence = int(data["cancel_sequence"])
                if cancel_sequence != self._cancel_sequence:
                    self._cancel_sequence = cancel_sequence
                    self._goal_events.put({"type": "cancel"})
                sequence = int(data["sequence"])
                goal = data.get("goal")
                if goal is not None and sequence > self._goal_sequence:
                    self._goal_sequence = sequence
                    self._goal_events.put(
                        {"type": "goal", "sequence": sequence, "goal": goal}
                    )
            except (KeyError, TypeError, ValueError, RuntimeError) as exc:
                self._log_throttled("goal_poll", f"Nav2 goal polling failed: {exc}")
            self._poll_manual_odometry()
            now = time.monotonic()
            with self._lock:
                map_pose = self._map_pose
                pose_is_fresh = now - self._map_pose_captured_at <= 0.5
            if map_pose is not None and pose_is_fresh and now - self._last_map_pose_report_at >= 0.2:
                try:
                    self._request("POST", "/nav2/bridge/pose", map_pose)
                except RuntimeError as exc:
                    self._log_throttled("map_pose", f"Could not report SLAM map pose: {exc}")
                self._last_map_pose_report_at = now
            while True:
                try:
                    result = self._goal_results.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._request("POST", "/nav2/goal/result", result)
                except RuntimeError as exc:
                    self._log_throttled("goal_result", f"Could not report Nav2 goal result: {exc}")
            if time.monotonic() >= next_readiness_report:
                with self._lock:
                    nav2_action_ready = self._nav2_action_ready
                try:
                    self._request(
                        "POST",
                        "/nav2/bridge/status",
                        {"nav2_action_ready": nav2_action_ready},
                    )
                except RuntimeError as exc:
                    self._log_throttled("nav2_readiness", f"Could not report Nav2 readiness: {exc}")
                next_readiness_report = time.monotonic() + 0.5
            self._stop_goal_worker.wait(0.2)

    def _check_nav2_readiness(self) -> None:
        with self._lock:
            if self._readiness_request_pending:
                return
            self._readiness_request_pending = True
        if not self._bt_navigator_state_client.service_is_ready():
            with self._lock:
                self._nav2_action_ready = False
                self._readiness_request_pending = False
            return
        try:
            future = self._bt_navigator_state_client.call_async(GetState.Request())
        except RuntimeError as exc:
            with self._lock:
                self._nav2_action_ready = False
                self._readiness_request_pending = False
            self._log_throttled("nav2_lifecycle", f"Could not query Nav2 lifecycle state: {exc}")
            return
        future.add_done_callback(self._on_nav2_lifecycle_state)

    def _capture_map_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as exc:
            with self._lock:
                self._map_pose = None
                self._map_pose_captured_at = 0.0
            self._log_throttled("map_pose_tf", f"SLAM map pose is unavailable: {exc}")
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = {
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
            ),
        }
        if not all(math.isfinite(value) for value in pose.values()):
            with self._lock:
                self._map_pose = None
                self._map_pose_captured_at = 0.0
            self._log_throttled("map_pose_tf", "SLAM map pose contains a non-finite value")
            return
        with self._lock:
            self._map_pose = pose
            self._map_pose_captured_at = time.monotonic()

    def _on_nav2_lifecycle_state(self, future: Any) -> None:
        try:
            response = future.result()
            ready = (
                response.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE
                and self._navigate_client.server_is_ready()
            )
        except (AttributeError, RuntimeError) as exc:
            ready = False
            self._log_throttled("nav2_lifecycle", f"Could not read Nav2 lifecycle state: {exc}")
        with self._lock:
            self._nav2_action_ready = ready
            self._readiness_request_pending = False

    def _poll_manual_odometry(self) -> None:
        try:
            data = self._request("GET", "/nav2/odometry/command")["data"]["command"]
            command = {
                axis: max(-_MAX_MANUAL_ODOM_LINEAR_MPS, min(_MAX_MANUAL_ODOM_LINEAR_MPS, float(data[axis])))
                for axis in ("vx", "vy")
            }
            command["wz"] = max(
                -_MAX_MANUAL_ODOM_ANGULAR_RPS,
                min(_MAX_MANUAL_ODOM_ANGULAR_RPS, float(data["wz"])),
            )
            if not all(math.isfinite(value) for value in command.values()):
                raise ValueError("manual odometry command must be finite")
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self._log_throttled("manual_odometry", f"Manual odometry polling failed: {exc}")
            return
        with self._lock:
            if self._claimed:
                return
            self._manual_odom_vx = command["vx"]
            self._manual_odom_vy = command["vy"]
            self._manual_odom_wz = command["wz"]
            self._manual_odom_received_at = time.monotonic()

    def _dispatch_goal_events(self) -> None:
        while True:
            try:
                event = self._goal_events.get_nowait()
            except queue.Empty:
                return
            if event["type"] == "cancel":
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                self._goal_handle = None
                self._goal_handle_sequence = 0
                continue
            if event["type"] != "goal":
                continue
            if not self._navigate_client.server_is_ready():
                self.get_logger().error("Nav2 NavigateToPose action server is unavailable; goal discarded.")
                self._goal_results.put(
                    {"sequence": int(event["sequence"]), "status": 6}
                )
                continue
            goal_data = event["goal"]
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = float(goal_data["x"])
            goal.pose.pose.position.y = float(goal_data["y"])
            yaw = float(goal_data["yaw"])
            goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
            sequence = int(event["sequence"])
            future = self._navigate_client.send_goal_async(goal)
            future.add_done_callback(
                lambda result_future, goal_sequence=sequence: self._on_goal_response(
                    result_future,
                    goal_sequence,
                )
            )
            self.get_logger().info(
                f"Nav2 goal dispatched at map x={goal_data['x']:.2f} y={goal_data['y']:.2f} yaw={yaw:.2f}"
            )

    def _on_goal_response(self, future: Any, sequence: int) -> None:
        try:
            handle = future.result()
        except (AttributeError, RuntimeError) as exc:
            self.get_logger().error(f"Nav2 rejected goal request: {exc}")
            self._goal_results.put({"sequence": sequence, "status": 6})
            return
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal.")
            self._goal_results.put({"sequence": sequence, "status": 6})
            return
        self._goal_handle = handle
        self._goal_handle_sequence = sequence
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, goal_sequence=sequence: self._on_goal_result(
                result_future,
                goal_sequence,
            )
        )

    def _on_goal_result(self, future: Any, sequence: int) -> None:
        try:
            result = future.result()
            status = int(result.status)
            self.get_logger().info(f"Nav2 goal finished with status {status}.")
            self._goal_results.put({"sequence": sequence, "status": status})
        except (AttributeError, RuntimeError) as exc:
            self.get_logger().error(f"Could not read Nav2 goal result: {exc}")
            self._goal_results.put({"sequence": sequence, "status": 6})
        if self._goal_handle_sequence == sequence:
            self._goal_handle = None
            self._goal_handle_sequence = 0

    def _scan_loop(self) -> None:
        while not self._stop_scanner.is_set():
            try:
                response = self._request("GET", "/lidar/scan")
                data = response["data"]
                if data.get("ready") is not True:
                    raise RuntimeError("backend reports physical LiDAR is not ready")
                if not self._publish_scan(data["scan"]):
                    raise RuntimeError("backend scan has no valid range returns")
            except (KeyError, TypeError, RuntimeError) as exc:
                self._log_throttled("scan", f"LiDAR bridge unavailable: {exc}")
            self._stop_scanner.wait(self.scan_poll_period)

    def _publish_scan(self, points: list[dict[str, Any]]) -> bool:
        ranges = [float("inf")] * _SCAN_BINS
        for point in points:
            try:
                angle_deg = float(point["angle_deg"])
                distance = float(point["distance_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(angle_deg) or not math.isfinite(distance):
                continue
            if distance < _RANGE_MIN_M or distance > _RANGE_MAX_M:
                continue
            angle = math.radians(angle_deg)
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            index = int(round((angle - _SCAN_MIN_ANGLE) / _SCAN_INCREMENT)) % _SCAN_BINS
            ranges[index] = min(ranges[index], distance)
        if not any(math.isfinite(value) for value in ranges):
            return False

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "lidar_link"
        scan.angle_min = _SCAN_MIN_ANGLE
        scan.angle_max = math.pi - _SCAN_INCREMENT
        scan.angle_increment = _SCAN_INCREMENT
        scan.time_increment = 0.0
        scan.scan_time = 0.5
        scan.range_min = _RANGE_MIN_M
        scan.range_max = _RANGE_MAX_M
        scan.ranges = ranges
        self._scan_pub.publish(scan)
        with self._lock:
            self._last_scan_at = time.monotonic()
        return True

    def _claim(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._request("POST", "/nav2/claim", {})
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response
        with self._lock:
            self._claimed = True
            self._armed = False
            self._vx = self._vy = self._wz = 0.0
            self._manual_odom_vx = self._manual_odom_vy = self._manual_odom_wz = 0.0
            self._manual_odom_received_at = 0.0
            self._last_command_at = time.monotonic()
        response.success = True
        response.message = "Nav2 control claimed; motors remain disarmed."
        return response

    def _arm(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            claimed = self._claimed
            scan_fresh = time.monotonic() - self._last_scan_at <= _SCAN_TIMEOUT_S
        if not claimed:
            response.success = False
            response.message = "Claim Nav2 control first."
            return response
        if not scan_fresh:
            response.success = False
            response.message = "No fresh valid LiDAR scan; motors remain disarmed."
            return response
        try:
            self._request("POST", "/nav2/arm", {})
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response
        with self._lock:
            self._armed = True
            self._last_command_at = time.monotonic()
        response.success = True
        response.message = "STM32 armed. Nav2 goals may now command motion."
        return response

    def _release(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._request("POST", "/nav2/release", {})
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response
        with self._lock:
            self._claimed = False
            self._armed = False
            self._vx = self._vy = self._wz = 0.0
            self._manual_odom_vx = self._manual_odom_vy = self._manual_odom_wz = 0.0
            self._manual_odom_received_at = 0.0
        response.success = True
        response.message = "Nav2 control released and STM32 disarmed."
        return response

    def _on_cmd_vel(self, message: Twist) -> None:
        with self._lock:
            if not self._claimed or not self._armed:
                self._vx = self._vy = self._wz = 0.0
                self._last_command_at = time.monotonic()
                return
            scan_fresh = time.monotonic() - self._last_scan_at <= _SCAN_TIMEOUT_S

        if not scan_fresh:
            try:
                self._request("POST", "/nav2/release", {})
            except RuntimeError as exc:
                self._log_throttled("stale_scan_release", f"LiDAR stale; backend release failed: {exc}")
            with self._lock:
                self._claimed = False
                self._armed = False
                self._vx = self._vy = self._wz = 0.0
            self._log_throttled("stale_scan", "LiDAR scan timed out; Nav2 control released and STM32 disarmed.")
            return

        command = {
            "vx": max(-_MAX_LINEAR_MPS, min(_MAX_LINEAR_MPS, message.linear.x)),
            "vy": max(-_MAX_LINEAR_MPS, min(_MAX_LINEAR_MPS, message.linear.y)),
            "wz": max(-_MAX_ANGULAR_RPS, min(_MAX_ANGULAR_RPS, message.angular.z)),
        }
        try:
            result = self._request("POST", "/nav2/cmd_vel", command)["data"]["command"]
        except (KeyError, TypeError, RuntimeError) as exc:
            try:
                self._request("POST", "/nav2/release", {})
            except RuntimeError as release_exc:
                self._log_throttled(
                    "cmd_release",
                    f"Velocity forwarding failed; backend stop could not be confirmed: {release_exc}",
                )
            with self._lock:
                self._claimed = False
                self._vx = self._vy = self._wz = 0.0
                self._armed = False
            self._log_throttled("cmd_vel", f"Nav2 velocity forwarding failed; motion disabled: {exc}")
            return
        with self._lock:
            self._vx = float(result["vx"])
            self._vy = float(result["vy"])
            self._wz = float(result["wz"])
            self._last_command_at = time.monotonic()

    def _send_heartbeat(self) -> None:
        with self._lock:
            claimed = self._claimed
            armed = self._armed
            scan_fresh = time.monotonic() - self._last_scan_at <= _SCAN_TIMEOUT_S
        if not claimed:
            return
        if armed and not scan_fresh:
            try:
                self._request("POST", "/nav2/release", {})
            except RuntimeError as exc:
                self._log_throttled("stale_scan_release", f"LiDAR stale; backend release failed: {exc}")
            with self._lock:
                self._claimed = False
                self._armed = False
                self._vx = self._vy = self._wz = 0.0
            self._log_throttled("stale_scan", "LiDAR scan timed out; Nav2 control released and STM32 disarmed.")
            return
        try:
            self._request("POST", "/nav2/heartbeat", {})
        except BackendRejectedError as exc:
            with self._lock:
                self._claimed = False
                self._armed = False
                self._vx = self._vy = self._wz = 0.0
            self._log_throttled("heartbeat_rejected", f"Nav2 lease rejected; bridge disarmed: {exc}")
        except RuntimeError as exc:
            self._log_throttled("heartbeat_retry", f"Nav2 heartbeat failed; retaining lease and retrying: {exc}")

        stale = False
        with self._lock:
            if self._armed and time.monotonic() - self._last_command_at > _COMMAND_TIMEOUT_S:
                stale = any((self._vx, self._vy, self._wz))
                self._vx = self._vy = self._wz = 0.0
                self._last_command_at = time.monotonic()
        if stale:
            try:
                self._request("POST", "/nav2/cmd_vel", {"vx": 0.0, "vy": 0.0, "wz": 0.0})
            except RuntimeError as exc:
                self._log_throttled("timeout_stop", f"Could not forward stale-command stop: {exc}")

    def _publish_odometry(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_odom_at, 0.0), 0.1)
        self._last_odom_at = now
        with self._lock:
            if self._claimed and self._armed:
                vx, vy, wz = self._vx, self._vy, self._wz
            elif time.monotonic() - self._manual_odom_received_at <= 0.5:
                vx, vy, wz = self._manual_odom_vx, self._manual_odom_vy, self._manual_odom_wz
            else:
                vx = vy = wz = 0.0
            mid_yaw = self._yaw + 0.5 * wz * dt
            self._x += (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
            self._y += (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
            self._yaw = math.atan2(math.sin(self._yaw + wz * dt), math.cos(self._yaw + wz * dt))
            x, y, yaw = self._x, self._y, self._yaw

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.10
        odom.twist.covariance[7] = 0.10
        odom.twist.covariance[35] = 0.20
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        self._stop_scanner.set()
        self._stop_map_worker.set()
        self._stop_goal_worker.set()
        if self._scanner.is_alive():
            self._scanner.join(timeout=1.0)
        if self._map_worker.is_alive():
            self._map_worker.join(timeout=1.0)
        if self._goal_worker.is_alive():
            self._goal_worker.join(timeout=1.0)
        with self._lock:
            claimed = self._claimed
        if claimed:
            try:
                self._request("POST", "/nav2/release", {})
            except RuntimeError as exc:
                self.get_logger().error(f"Could not release Nav2 control on shutdown: {exc}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OmniBotBackendBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
