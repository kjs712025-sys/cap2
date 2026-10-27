from __future__ import annotations

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app
from robot.motion import VelocityCommand


def test_simulation_uses_virtual_devices_and_never_opens_hardware() -> None:
    config = RobotConfig(
        simulation=True,
        autostart_autonomous=False,
        fire_monitor_enabled=False,
        mqtt_enabled=False,
        enable_websocket=False,
    )
    app = create_app(config)

    with TestClient(app) as client:
        diagnostics = client.get("/diagnostics").json()["data"]
        assert diagnostics["simulation"] is True
        assert diagnostics["components"]["stm32"]["connected"] is True
        assert diagnostics["components"]["camera"]["ready"] is True
        assert diagnostics["components"]["lidar"]["ready"] is True
        assert diagnostics["components"]["camera"]["source"] == "simulation"
        assert diagnostics["components"]["lidar"]["port"] == "virtual"
        assert diagnostics["components"]["fire_monitor"]["state"] == "simulation"
        assert app.state.navigator.use_camera is False
        assert client.get("/telemetry").json()["data"]["fire"] == "unavailable"
        assert set(diagnostics["system"]) == {
            "cpu_load_1m",
            "cpu_temperature_c",
            "memory_available_mb",
            "disk_free_gb",
        }

        frame = app.state.camera.read_frame()
        assert frame is not None
        assert frame.image.shape == (480, 640, 3)
        assert app.state.camera.capture is None

        scan = app.state.lidar.read_scan()
        assert len(scan) == 72

        assert app.state.stm32.serial is None
        assert app.state.stm32.arm() is True
        assert app.state.stm32.send_velocity(VelocityCommand(0.2, 0.0, 0.1)) is True
        app.state.stm32.poll()
        assert app.state.stm32.status.odometry is not None

        response = client.post("/motion/command", json={"vx": 0.2, "vy": 0, "wz": 0})
        assert response.status_code == 200
        assert response.json()["success"] is True

        response = client.post("/navigation/start")
        assert response.status_code == 200
        assert response.json()["error"] == "nav2_not_running"

        app.state.nav2_process.state = lambda: {"running": True}
        response = client.post("/navigation/start")
        assert response.json()["success"] is True
        assert response.json()["data"] == {"claimed": True, "armed": False}
        assert app.state.stm32.status.armed is False
        assert client.post("/nav2/arm").json()["success"] is True
        assert app.state.stm32.status.armed is True

        health = client.get("/health").json()["data"]
        assert health["simulation"] is True
