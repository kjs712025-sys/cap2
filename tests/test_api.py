from __future__ import annotations

import base64

from fastapi.testclient import TestClient

from config import RobotConfig
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
    assert abs(command["vx"] - 0.08) < 1e-9
    assert abs(command["vy"] + 0.06) < 1e-9
    assert command["wz"] == 0.25
    assert client.post("/motion/command", json={"vx": 0.1}).json()["error"] == "autonomous_active"

    response = client.post("/nav2/release")
    assert response.json()["data"]["armed"] is False
    assert client.get("/mode").json()["data"]["mode"] == "manual"


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
        time.sleep(0.7)
        assert app.state.nav2_control_active is False
        assert app.state.stm32.status.armed is False


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
