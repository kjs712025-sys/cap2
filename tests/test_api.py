from __future__ import annotations

import base64
import math
import time

from fastapi.testclient import TestClient

from config import RobotConfig
from lidar.lidar import ScanPoint
from network.route_patrol import RoutePatrol
from network.server import create_app


def _set_nav2_running(app) -> None:
    app.state.nav2_process.state = lambda: {
        "running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }


def test_health_and_motion_command() -> None:
    app = create_app(
        RobotConfig(debug=True, simulation=True, llm_enabled=False, enable_websocket=False)
    )
    client = TestClient(app)

    response = client.get("/", follow_redirects=False)
    assert response.status_code == 307
    assert response.headers["location"] == "/app/dashboard.html"

    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["data"]["status"] == "ok"

    payload = {"vx": 0.2, "vy": 0.0, "wz": -0.1}
    response = client.post("/motion/command", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["data"]["command"]["vx"] == 0.2
    assert client.get("/nav2/odometry/command").json()["data"]["command"] == payload


def test_manual_motion_arms_stm32_and_zero_command_disarms() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    client = TestClient(app)

    response = client.post("/motion/command", json={"vx": 0.7, "vy": -0.6, "wz": 2.0})

    assert response.json()["success"] is True
    assert response.json()["data"]["command"] == {"vx": 0.5, "vy": -0.5, "wz": 1.0}
    assert app.state.stm32.status.armed is True

    response = client.post("/motion/command", json={"vx": 0, "vy": 0, "wz": 0})

    assert response.json()["success"] is True
    assert app.state.stm32.status.armed is False
    assert app.state.status.current_speed == (0.0, 0.0, 0.0)
    assert client.get("/nav2/odometry/command").json()["data"]["command"] == {
        "vx": 0,
        "vy": 0,
        "wz": 0,
    }


def test_manual_odometry_is_zero_when_nav2_owns_control() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    client = TestClient(app)
    assert client.post("/motion/command", json={"vx": 0.1, "vy": 0, "wz": 0}).json()["success"]
    assert client.post("/nav2/claim").json()["success"]

    command = client.get("/nav2/odometry/command").json()["data"]["command"]

    assert command == {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    assert client.post("/nav2/release").json()["success"]
    assert client.get("/nav2/odometry/command").json()["data"]["command"] == {
        "vx": 0.0,
        "vy": 0.0,
        "wz": 0.0,
    }


def test_manual_motion_reports_failed_stm32_arm() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.stm32.arm = lambda: False
    client = TestClient(app)

    response = client.post("/motion/command", json={"vx": 0.1, "vy": 0, "wz": 0})

    assert response.json()["success"] is False
    assert response.json()["error"] == "stm32_unavailable"


def test_websocket_connects_and_sends_initial_status() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=True,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    client = TestClient(app)

    with client.websocket_connect("/ws") as websocket:
        message = websocket.receive_json()

    assert message["event"] == "connected"
    assert message["hardware"]["simulation"] is True


def test_camera_stream_endpoint() -> None:
    """/camera/stream is a continuous MJPEG feed with no natural end, so it can't
    be drained through TestClient like a normal request. Call the route
    coroutine directly instead and pull just the first chunk off its
    StreamingResponse, which exercises the same wiring without hanging."""
    import asyncio
    from types import SimpleNamespace

    from network.api import camera_stream

    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    request = SimpleNamespace(app=app)

    async def run() -> None:
        response = await camera_stream(request)
        assert "multipart/x-mixed-replace" in response.media_type
        chunk = await response.body_iterator.__anext__()
        assert chunk

    asyncio.run(run())


def test_lidar_scan_endpoint() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    response = client.get("/lidar/scan")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert len(payload["data"]["scan"]) >= 1
    assert payload["data"]["ready"] is False
    assert payload["data"]["safe_direction"] in {"forward", "left", "right", "backward", "stop"}


def test_nav2_control_requires_claim_and_explicit_arm() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    client = TestClient(app)

    response = client.post("/nav2/cmd_vel", json={"vx": 0.1, "vy": 0.0, "wz": 0.0})
    assert response.json()["error"] == "nav2_not_claimed"

    response = client.post("/nav2/claim")
    assert response.json()["data"]["armed"] is False
    response = client.post("/nav2/arm")
    assert response.json()["data"]["armed"] is True
    assert client.post("/mode", json={"mode": "auto"}).json()["success"] is True
    assert app.state.stm32.status.armed is True

    response = client.post("/nav2/cmd_vel", json={"vx": 0.4, "vy": -0.3, "wz": 1.0})
    command = response.json()["data"]["command"]
    assert abs(command["vx"] - 0.4) < 1e-9
    assert abs(command["vy"] + 0.3) < 1e-9
    assert command["wz"] == 0.25
    assert client.post("/motion/command", json={"vx": 0.1}).json()["error"] == "autonomous_active"

    response = client.post("/nav2/release")
    assert response.json()["data"]["armed"] is False
    assert client.get("/mode").json()["data"]["mode"] == "manual"


def test_nav2_speed_setting_updates_runtime_linear_velocity_limit() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    client = TestClient(app)
    assert client.get("/nav2/state").json()["data"]["max_linear_speed_mps"] == 1.00

    updated = client.post("/nav2/speed", json={"max_linear_speed_mps": 0.03})
    assert updated.json()["success"] is True
    assert client.get("/nav2/state").json()["data"]["max_linear_speed_mps"] == 0.03

    assert client.post("/nav2/claim").json()["success"] is True
    assert client.post("/nav2/arm").json()["success"] is True
    command = client.post(
        "/nav2/cmd_vel",
        json={"vx": 0.04, "vy": -0.03, "wz": 0.0},
    ).json()["data"]["command"]
    assert abs(command["vx"] - 0.024) < 1e-9
    assert abs(command["vy"] + 0.018) < 1e-9

    maximum = client.post("/nav2/speed", json={"max_linear_speed_mps": 1.00})
    assert maximum.json()["success"] is True
    command = client.post(
        "/nav2/cmd_vel",
        json={"vx": 1.0, "vy": -1.0, "wz": 0.0},
    ).json()["data"]["command"]
    assert abs((command["vx"] ** 2 + command["vy"] ** 2) ** 0.5 - 1.00) < 1e-9

    assert client.post("/nav2/speed", json={"max_linear_speed_mps": 0.0}).json()["error"] == "bad_speed"
    assert client.post("/nav2/speed", json={"max_linear_speed_mps": 1.01}).json()["error"] == "bad_speed"
    assert client.post("/nav2/speed", json={"max_linear_speed_mps": True}).json()["error"] == "bad_speed"
    assert client.post("/nav2/release").json()["success"] is True


def test_nav2_claim_requires_confirmed_disarm() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    client = TestClient(app)

    response = client.post("/mode", json={"mode": "auto"}).json()

    assert response["success"] is False
    assert response["error"] == "stm32_unavailable"
    assert app.state.nav2_control_active is False
    assert client.get("/mode").json()["data"]["mode"] == "manual"


def test_nav2_watchdog_disarms_after_bridge_heartbeat_stops() -> None:
    import time

    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    with TestClient(app) as client:
        assert client.post("/nav2/claim").json()["success"] is True
        assert client.post("/nav2/arm").json()["success"] is True
        time.sleep(2.0)
        assert app.state.nav2_control_active is False
        assert app.state.stm32.status.armed is False


def test_nav2_control_poll_reports_lease_and_heartbeat_keeps_armed_state() -> None:
    import time

    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    with TestClient(app) as client:
        assert client.post("/nav2/claim").json()["success"] is True
        assert client.post("/nav2/arm").json()["success"] is True

        control = client.get("/nav2/goal/next").json()["data"]
        assert control["control_claimed"] is True
        assert control["armed"] is True

        for _ in range(4):
            time.sleep(0.15)
            assert client.post("/nav2/heartbeat").json()["success"] is True
        time.sleep(0.2)

        assert app.state.nav2_control_active is True
        assert app.state.stm32.status.armed is True


def test_nav2_arm_refuses_missing_physical_lidar() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    app.state.stm32.disarm = lambda: True
    client = TestClient(app)
    assert client.post("/nav2/claim").json()["success"] is True

    response = client.post("/nav2/arm")

    assert response.json()["error"] == "lidar_unavailable"
    assert app.state.stm32.status.armed is False


def test_nav2_goal_is_queued_and_cancelled() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    client = TestClient(app)

    assert client.post("/nav2/claim").json()["success"] is True
    assert client.post("/nav2/arm").json()["success"] is True
    response = client.post("/nav2/goal", json={"x": 1.2, "y": -0.5, "yaw": 0.25})
    assert response.json()["success"] is True
    assert response.json()["data"]["goal"]["x"] == 1.2
    assert response.json()["data"]["goal"]["y"] == -0.5

    response = client.get("/nav2/goal/next")
    assert response.json()["data"]["goal"]["yaw"] == 0.25
    assert client.post("/nav2/cancel").json()["success"] is True
    assert client.get("/nav2/goal/next").json()["data"]["goal"] is None


def test_nav2_stack_start_and_stop_endpoints() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    running = False

    def process_state() -> dict[str, object]:
        return {
            "running": running,
            "pid": 1234 if running else None,
            "started_at": 1.0 if running else None,
            "last_exit_code": None,
            "workspace_built": True,
            "ros_setup_available": True,
        }

    def start_process() -> tuple[bool, str, dict[str, object]]:
        nonlocal running
        running = True
        return True, "started", process_state()

    def stop_process() -> tuple[bool, str, dict[str, object]]:
        nonlocal running
        running = False
        return True, "stopped", process_state()

    app.state.nav2_process.state = process_state
    app.state.nav2_process.start = start_process
    app.state.nav2_process.stop = stop_process
    client = TestClient(app)

    response = client.post("/nav2/stack/start")
    assert response.json()["success"] is True
    assert client.get("/nav2/state").json()["data"]["running"] is True

    response = client.post("/nav2/stack/stop")
    assert response.json()["success"] is True
    assert client.get("/nav2/state").json()["data"]["running"] is False


def test_nav2_mission_start_prepares_stack_for_goal_navigation() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    process = {"running": False, "slam_running": False}
    starts = {"nav2": 0, "slam": 0}

    def process_state() -> dict[str, object]:
        return {
            **process,
            "pid": 1234 if process["running"] else None,
            "started_at": 1.0 if process["running"] else None,
            "last_exit_code": None,
            "workspace_built": True,
            "ros_setup_available": True,
        }

    def start_nav2() -> tuple[bool, str, dict[str, object]]:
        starts["nav2"] += 1
        process["running"] = True
        return True, "Nav2 started", process_state()

    def start_slam() -> tuple[bool, str, dict[str, object]]:
        starts["slam"] += 1
        process["slam_running"] = True
        return True, "SLAM started", process_state()

    app.state.nav2_process.state = process_state
    app.state.nav2_process.start = start_nav2
    app.state.nav2_process.start_slam = start_slam

    with TestClient(app) as client:
        response = client.post("/nav2/mission/start", json={})

        assert response.json()["success"] is True
        assert response.json()["data"]["armed"] is True
        assert response.json()["data"]["ready_for_goal"] is True
        assert starts == {"nav2": 1, "slam": 1}
        assert app.state.nav2_control_active is True
        assert app.state.stm32.status.armed is True
        assert app.state.nav2_goal is None
        assert app.state.status.current_speed == (0.0, 0.0, 0.0)
        assert app.state.status.mission == "nav2_goal_navigation"
        assert client.get("/nav2/state").json()["data"]["mission_start"]["stage"] == "ready_for_goal"
        goal_response = client.post("/nav2/goal", json={"x": 1.0, "y": -0.5, "yaw": 0.25})
        assert goal_response.json()["success"] is True
        assert goal_response.json()["data"]["goal"] == {"x": 1.0, "y": -0.5, "yaw": 0.25}
        assert client.post("/nav2/release").json()["success"] is True


def test_nav2_mission_start_times_out_without_fresh_hardware_map(
    monkeypatch,
) -> None:
    from network import api

    monkeypatch.setattr(api, "NAV2_MISSION_MAP_TIMEOUT_S", 0.01)
    monkeypatch.setattr(api, "NAV2_MISSION_POLL_INTERVAL_S", 0.001)
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _set_nav2_running(app)
    app.state.nav2_process.state = lambda: {
        "running": True,
        "slam_running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }
    app.state.stm32.status.connected = True
    app.state.stm32.disarm = lambda: True
    app.state.lidar.is_ready = lambda: True
    app.state.lidar.status.last_scan_hardware = True

    with TestClient(app) as client:
        response = client.post("/nav2/mission/start", json={"x": 0.5, "y": 0, "yaw": 0})

        assert response.json()["success"] is False
        assert response.json()["error"] == "autonomy_readiness_timeout"
        assert app.state.nav2_control_active is False
        assert app.state.stm32.status.armed is False
        assert app.state.nav2_mission_start["stage"] == "failed"


def test_nav2_mission_waits_for_active_nav2_action_server(monkeypatch) -> None:
    from network import api

    monkeypatch.setattr(api, "NAV2_MISSION_MAP_TIMEOUT_S", 0.01)
    monkeypatch.setattr(api, "NAV2_MISSION_POLL_INTERVAL_S", 0.001)
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.nav2_process.state = lambda: {
        "running": True,
        "slam_running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }
    app.state.stm32.status.connected = True
    app.state.lidar.is_ready = lambda: True
    app.state.lidar.status.last_scan_hardware = True
    app.state.ros_slam_map = {"width": 1, "height": 1}
    app.state.ros_slam_map_received_at = time.time()

    with TestClient(app) as client:
        response = client.post("/nav2/mission/start", json={"x": 0.5, "y": 0, "yaw": 0})

    assert response.json()["success"] is False
    assert response.json()["error"] == "autonomy_readiness_timeout"
    assert "Active Nav2 action server" in response.json()["message"]
    readiness = app.state.nav2_mission_start["readiness"]
    assert readiness["action_server"] is False
    assert readiness["nav2"] is True
    assert readiness["slam"] is True
    assert app.state.nav2_control_active is False
    assert app.state.stm32.status.armed is False


def test_nav2_bridge_status_reports_action_server_readiness() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )

    with TestClient(app) as client:
        response = client.post(
            "/nav2/bridge/status",
            json={"nav2_action_ready": True},
        )
        state = client.get("/nav2/state").json()["data"]

    assert response.json()["success"] is True
    assert state["nav2_action_ready"] is True


def test_reactive_free_roam_can_be_started_and_stopped() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )

    with TestClient(app) as client:
        started = client.post("/navigation/roam/start")
        state = client.get("/nav2/state").json()["data"]
        stopped = client.post("/navigation/roam/stop")

    assert started.json()["success"] is True
    assert state["roam_enabled"] is True
    assert stopped.json()["success"] is True
    assert app.state.navigator.enabled is False


def test_reactive_free_roam_requires_fresh_hardware_lidar() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.stm32.status.connected = True

    with TestClient(app) as client:
        response = client.post("/navigation/roam/start")

    assert response.json()["success"] is False
    assert response.json()["error"] == "lidar_unavailable"
    assert app.state.navigator.enabled is False


def test_slam_map_pose_is_recorded_for_route_patrol(tmp_path) -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _configure_ready_hardware_mission(app, front_distance_m=2.0)
    app.state.nav2_patrol = RoutePatrol(tmp_path / "patrol.json")
    app.state.ros_slam_map = {"width": 1, "height": 1}
    app.state.ros_slam_map_received_at = time.time()
    app.state.nav2_process.state = lambda: {
        "running": True,
        "slam_running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }

    with TestClient(app) as client:
        pose = client.post("/nav2/bridge/pose", json={"x": 1.0, "y": -0.5, "yaw": 7.0})
        state = client.get("/nav2/state").json()["data"]
        recording = client.post("/nav2/patrol/record/start", json={})
        client.post("/nav2/bridge/pose", json={"x": 1.8, "y": -0.5, "yaw": 0.0})
        client.post("/nav2/bridge/pose", json={"x": 2.6, "y": -0.5, "yaw": 0.0})
        saved = client.post("/nav2/patrol/record/stop", json={})

    assert pose.json()["success"] is True
    assert state["map_pose"]["x"] == 1.0
    assert state["map_pose"]["y"] == -0.5
    assert math.isclose(state["map_pose"]["yaw"], math.atan2(math.sin(7.0), math.cos(7.0)))
    assert recording.json()["success"] is True
    assert saved.json()["success"] is True
    assert saved.json()["data"]["waypoint_count"] == 3
    assert (tmp_path / "patrol.json").exists()


def test_route_patrol_advances_waypoints_and_disarms_when_complete(tmp_path) -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.nav2_patrol = RoutePatrol(tmp_path / "patrol.json")
    app.state.nav2_patrol.start_recording({"x": 0.0, "y": 0.0, "yaw": 0.0})
    app.state.nav2_patrol.stop_recording({"x": 1.2, "y": 0.0, "yaw": 0.0})
    app.state.nav2_patrol.start({"x": 0.0, "y": 0.0, "yaw": 0.0}, repeat=False)
    app.state.nav2_patrol_active_goal_sequence = 4
    app.state.nav2_control_active = True
    app.state.nav2_last_command_at = time.monotonic()
    app.state.stm32.status.armed = True

    def disarm() -> bool:
        app.state.stm32.status.armed = False
        return True

    app.state.stm32.disarm = disarm

    with TestClient(app) as client:
        first = client.post("/nav2/goal/result", json={"sequence": 4, "status": 4})
        next_goal = app.state.nav2_goal
        second_sequence = app.state.nav2_patrol_active_goal_sequence
        last = client.post(
            "/nav2/goal/result",
            json={"sequence": second_sequence, "status": 4},
        )

    assert first.json()["success"] is True
    assert next_goal["x"] == 1.2
    assert last.json()["success"] is True
    assert app.state.nav2_patrol.state()["mode"] == "complete"
    assert app.state.nav2_control_active is False
    assert app.state.stm32.status.armed is False


def test_route_patrol_start_prepares_nav2_and_queues_first_map_goal(tmp_path) -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.nav2_patrol = RoutePatrol(tmp_path / "patrol.json")
    app.state.nav2_patrol.start_recording({"x": 0.0, "y": 0.0, "yaw": 0.0})
    app.state.nav2_patrol.stop_recording({"x": 1.2, "y": 0.0, "yaw": 0.0})
    app.state.nav2_process.state = lambda: {
        "running": True,
        "slam_running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }
    app.state.nav2_map_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    app.state.nav2_map_pose_at = time.monotonic()
    app.state.ros_slam_map = {"width": 1, "height": 1}
    app.state.ros_slam_map_received_at = time.time()
    app.state.stm32.disarm = lambda: True

    def arm() -> bool:
        app.state.stm32.status.armed = True
        return True

    app.state.stm32.arm = arm
    app.state.stm32.send_heartbeat = lambda: True

    with TestClient(app) as client:
        response = client.post("/nav2/patrol/start", json={"repeat": False})
        state = client.get("/nav2/state").json()["data"]

    assert response.json()["success"] is True
    assert response.json()["data"]["goal"]["x"] == 0.0
    assert state["patrol"]["patrolling"] is True
    assert state["control_claimed"] is True
    assert state["armed"] is True


def _configure_ready_hardware_mission(
    app,
    front_distance_m: float,
    right_distance_m: float = 0.5,
) -> None:
    app.state.nav2_process.state = lambda: {
        "running": True,
        "slam_running": True,
        "pid": 1234,
        "started_at": 1.0,
        "last_exit_code": None,
        "workspace_built": True,
        "ros_setup_available": True,
    }
    app.state.nav2_action_ready = True
    app.state.nav2_action_ready_at = time.monotonic()
    app.state.ros_slam_map = {"width": 1, "height": 1}
    app.state.ros_slam_map_received_at = time.time()
    app.state.stm32.status.connected = True

    def disarm() -> bool:
        app.state.stm32.status.armed = False
        return True

    def arm() -> bool:
        app.state.stm32.status.armed = True
        return True

    app.state.stm32.disarm = disarm
    app.state.stm32.arm = arm
    app.state.stm32.send_velocity = lambda _command: True
    app.state.stm32.send_heartbeat = lambda: True
    app.state.lidar.is_ready = lambda: True
    app.state.lidar.status.connected = True
    app.state.lidar.status.last_scan = [
        ScanPoint(angle_deg=angle, distance_m=front_distance_m)
        for angle in (-10.0, 0.0, 10.0)
    ] + [
        ScanPoint(angle_deg=-90.0, distance_m=right_distance_m),
        ScanPoint(angle_deg=45.0, distance_m=1.2),
        ScanPoint(angle_deg=90.0, distance_m=1.2),
    ]
    app.state.lidar.status.last_scan_at = time.monotonic()
    app.state.lidar.status.last_scan_hardware = True
    app.state.lidar.read_scan = lambda: app.state.lidar.status.last_scan


def test_nav2_goal_navigation_waits_for_explicit_goal_and_uses_live_scan() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    _configure_ready_hardware_mission(app, front_distance_m=0.75, right_distance_m=0.4)

    with TestClient(app) as client:
        response = client.post("/nav2/mission/start", json={})
        assert response.json()["success"] is True
        state = client.get("/nav2/state").json()["data"]
        assert state["goal"] is None
        assert state["armed"] is True
        assert app.state.status.current_speed == (0.0, 0.0, 0.0)

        goal = client.post("/nav2/goal", json={"x": 1.0, "y": 0.5, "yaw": 0.0})
        assert goal.json()["success"] is True
        assert client.get("/nav2/goal/next").json()["data"]["goal"] == {
            "x": 1.0,
            "y": 0.5,
            "yaw": 0.0,
        }
        assert client.post("/nav2/cancel").json()["success"] is True
        assert client.post("/nav2/release").json()["success"] is True

    assert app.state.nav2_control_active is False
    assert app.state.stm32.status.armed is False


def test_ros_slam_map_receives_and_validates_occupancy_grid() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    client = TestClient(app)
    map_payload = {
        "width": 2,
        "height": 2,
        "resolution_m": 0.05,
        "origin_m": {"x": -1.0, "y": -2.0, "yaw": 0.0},
        "grid_b64": base64.b64encode(bytes([255, 0, 100, 50])).decode("ascii"),
    }

    response = client.post("/nav2/slam/map", json=map_payload)
    assert response.json()["success"] is True
    data = client.get("/nav2/slam/map").json()["data"]
    assert data["available"] is True
    assert data["map"]["width"] == 2
    assert data["map"]["height"] == 2

    map_payload["grid_b64"] = base64.b64encode(bytes([0])).decode("ascii")
    response = client.post("/nav2/slam/map", json=map_payload)
    assert response.json()["error"] == "bad_slam_map"


def test_ros_slam_process_controls_are_independent() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=True,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    running = False

    def process_state() -> dict[str, object]:
        return {
            "running": False,
            "slam_running": running,
            "bridge_running": running,
            "workspace_built": True,
            "ros_setup_available": True,
        }

    def start_slam() -> tuple[bool, str, dict[str, object]]:
        nonlocal running
        running = True
        return True, "SLAM started.", process_state()

    def stop_slam() -> tuple[bool, str, dict[str, object]]:
        nonlocal running
        running = False
        return True, "SLAM stopped.", process_state()

    app.state.nav2_process.state = process_state
    app.state.nav2_process.start_slam = start_slam
    app.state.nav2_process.stop_slam = stop_slam
    client = TestClient(app)

    assert client.post("/nav2/slam/start").json()["success"] is True
    assert client.get("/nav2/state").json()["data"]["slam_running"] is True
    assert client.post("/nav2/slam/stop").json()["success"] is True
    assert client.get("/nav2/state").json()["data"]["slam_running"] is False


def test_stale_ros_slam_map_releases_armed_nav2_control() -> None:
    import time

    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.nav2_process.state = lambda: {"running": True, "slam_running": True}
    app.state.nav2_control_active = True
    app.state.stm32.status.armed = True
    app.state.stm32.send_heartbeat = lambda: True
    app.state.nav2_action_ready = True
    app.state.nav2_action_ready_at = time.monotonic()

    def disarm() -> bool:
        app.state.stm32.status.armed = False
        return True

    app.state.stm32.disarm = disarm
    app.state.ros_slam_map = {"width": 1, "height": 1}
    app.state.ros_slam_map_received_at = time.time() - 6.0
    client = TestClient(app)

    response = client.post("/nav2/heartbeat")

    assert response.json()["error"] == "slam_map_unavailable"
    assert app.state.nav2_control_active is False
    assert app.state.stm32.status.armed is False


def test_nav2_arm_requires_fresh_ros_map_on_hardware() -> None:
    app = create_app(
        RobotConfig(
            debug=True,
            simulation=False,
            llm_enabled=False,
            enable_websocket=False,
            fire_monitor_enabled=False,
            mqtt_enabled=False,
        )
    )
    app.state.nav2_process.state = lambda: {"running": True, "slam_running": True}
    app.state.lidar.is_ready = lambda: True
    app.state.lidar.status.last_scan_hardware = True
    app.state.stm32.disarm = lambda: True
    client = TestClient(app)

    assert client.post("/nav2/claim").json()["success"] is True
    response = client.post("/nav2/arm")

    assert response.json()["error"] == "slam_map_unavailable"
    assert app.state.stm32.status.armed is False
