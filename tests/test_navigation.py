from __future__ import annotations

from unittest.mock import MagicMock

from fastapi.testclient import TestClient

from config import RobotConfig
from lidar.lidar import ScanPoint
from lidar.obstacle import ObstacleDetector
from network.server import create_app
from robot.navigation import AutonomousNavigator
from robot.status import RobotMode, RobotStatus


def _ring(distance_m: float, exclude: tuple[float, float] | None = None, exclude_dist: float = 3.0) -> list[ScanPoint]:
    """A full 360deg scan at a fixed distance, optionally opening one angular gap."""
    points = []
    for angle in range(0, 360, 2):
        d = distance_m
        if exclude is not None:
            lo, hi = exclude
            in_gap = lo <= angle <= hi if lo <= hi else (angle >= lo or angle <= hi)
            if in_gap:
                d = exclude_dist
        points.append(ScanPoint(angle_deg=float(angle), distance_m=d))
    return points


def _make_navigator() -> tuple[AutonomousNavigator, MagicMock, RobotStatus]:
    stm32 = MagicMock()
    stm32.status.odometry = None
    stm32.status.connected = True
    stm32.status.armed = False
    stm32.status.battery_voltage = None
    stm32.status.last_error = None
    stm32.arm.return_value = True
    status = RobotStatus()
    slam = MagicMock()
    nav = AutonomousNavigator(
        lidar=MagicMock(),
        stm32=stm32,
        status=status,
        slam=slam,
        config=RobotConfig(debug=True),
        event_logger=None,
    )
    return nav, stm32, status


def test_sector_distances_cover_the_scan() -> None:
    detector = ObstacleDetector()
    sectors = detector.sector_distances(_ring(1.5))
    assert sectors["front"]["count"] > 0
    assert sectors["front"]["min_distance"] == 1.5
    assert sectors["rear"]["count"] > 0


def test_disabled_navigator_sends_no_motion() -> None:
    nav, stm32, status = _make_navigator()
    nav.step(_ring(0.3))  # obstacle everywhere
    stm32.send_velocity.assert_not_called()
    assert nav.state["decision"] == "idle"


def test_enabled_navigator_cruises_when_path_is_clear() -> None:
    nav, stm32, _ = _make_navigator()
    nav.enable()
    nav.step(_ring(2.5))
    assert nav.state["decision"] == "cruise"
    assert nav.state["velocity"]["vx"] > 0.0


def test_enabled_navigator_turns_away_from_a_front_obstacle() -> None:
    nav, stm32, _ = _make_navigator()
    nav.enable()
    # Wall close ahead, open gap toward the left (around 60-120deg).
    nav.step(_ring(0.3, exclude=(55, 125), exclude_dist=2.5))
    assert nav.state["decision"] in {"turn_left", "avoid"}
    assert nav.state["velocity"]["wz"] > 0.0


def test_navigator_holds_still_under_emergency_stop() -> None:
    nav, stm32, status = _make_navigator()
    nav.enable()
    status.set_mode(RobotMode.ERROR)
    nav.step(_ring(2.5))
    assert nav.state["decision"] == "halted"
    assert nav.state["velocity"] == {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def test_navigation_endpoints_toggle_state() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    assert client.get("/navigation/state").json()["data"]["enabled"] is False
    assert client.post("/navigation/start").json()["data"]["enabled"] is True
    assert app.state.status.mode.value == "autonomous"
    assert client.post("/navigation/stop").json()["data"]["enabled"] is False


def test_api_info_and_token_guard() -> None:
    from fastapi.testclient import TestClient
    from network.server import create_app

    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False, api_token="secret"))
    client = TestClient(app)

    assert client.get("/api").json()["data"]["auth_required"] is True
    assert client.get("/health").status_code == 200          # open path
    assert client.get("/telemetry").status_code == 401        # guarded
    assert client.get("/telemetry", headers={"Authorization": "Bearer secret"}).status_code == 200
    assert client.get("/telemetry?token=secret").status_code == 200


def test_nav_autostart_enables_navigator_after_delay() -> None:
    import time

    app = create_app(
        RobotConfig(
            debug=True,
            llm_enabled=False,
            enable_websocket=False,
            nav_autostart=True,
            nav_autostart_delay=0.0,
        )
    )
    with TestClient(app) as client:  # context manager runs startup/shutdown events
        mode = "manual"
        for _ in range(40):
            mode = client.get("/mode").json()["data"]["mode"]
            if mode == "autonomous":
                break
            time.sleep(0.05)
        assert mode == "autonomous"


def test_mode_switch_gates_manual_control() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    # boots in manual mode — joystick + LLM commands allowed
    assert client.get("/mode").json()["data"]["mode"] == "manual"
    assert client.post("/motion/command", json={"vx": 0.1, "vy": 0.0, "wz": 0.0}).json()["success"] is True
    assert client.post("/command/manual", json={"text": "stop"}).json()["success"] is True

    # switch to autonomous — manual inputs now refused
    assert client.post("/mode", json={"mode": "auto"}).json()["data"]["mode"] == "autonomous"
    motion = client.post("/motion/command", json={"vx": 0.1, "vy": 0.0, "wz": 0.0}).json()
    assert motion["success"] is False and motion["error"] == "autonomous_active"
    text = client.post("/command/manual", json={"text": "go"}).json()
    assert text["success"] is False and text["error"] == "autonomous_active"

    # back to manual — allowed again
    assert client.post("/mode", json={"mode": "manual"}).json()["data"]["mode"] == "manual"
    assert client.post("/motion/command", json={"vx": 0.0, "vy": 0.0, "wz": 0.0}).json()["success"] is True

    assert client.post("/mode", json={"mode": "sideways"}).json()["error"] == "bad_mode"


def test_fire_state_endpoint_returns_mqtt_payload() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    body = client.get("/fire/state").json()
    assert body["success"] is True
    assert body["data"]["level"] == "clear"
    assert "detections" in body["data"]
    assert not any(k.startswith("_") for k in body["data"])

    assert "/fire/stream" in client.get("/api").json()["data"]["streams"].values()


def test_camera_fusion_overrides_lidar_when_closer() -> None:
    import numpy as np
    from camera.depth import FloorObstacleEstimator

    nav, stm32, status = _make_navigator()
    nav.use_camera = True
    nav.camera = MagicMock()
    frame = MagicMock()
    frame.image = np.full((120, 160, 3), 130, np.uint8)
    nav.camera.read_frame.return_value = frame

    # camera "sees" a close obstacle dead ahead; lidar sector is clear (2.5 m)
    nav._camera_result = {"available": True, "floor_frac": 0.9,
                          "sectors": {"front": 0.35, "front_left": None, "front_right": None}}
    nav._camera_at = 10**9  # fresh
    import time as _t
    nav._camera_at = _t.monotonic()

    nav.step(_ring(2.5))
    assert nav.state["sectors"]["front"]["min_distance"] == 0.35
    assert nav.state["sectors"]["front"]["source"] == "camera"
    assert nav.state["camera"]["available"] is True


def test_camera_estimator_ranks_near_and_far() -> None:
    import numpy as np
    from camera.depth import FloorObstacleEstimator

    est = FloorObstacleEstimator()
    far = np.full((240, 320, 3), 130, np.uint8)
    far[:120, 130:190] = (20, 20, 20)
    near = np.full((240, 320, 3), 130, np.uint8)
    near[:200, 130:190] = (20, 20, 20)
    assert est.estimate(near)["sectors"]["front"] < est.estimate(far)["sectors"]["front"]
