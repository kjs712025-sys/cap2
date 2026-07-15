from __future__ import annotations

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app


def test_health_and_motion_command() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

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


def test_camera_stream_endpoint() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    response = client.get("/camera/stream")
    assert response.status_code == 200
    assert "multipart/x-mixed-replace" in response.headers["content-type"]


def test_lidar_scan_endpoint() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    response = client.get("/lidar/scan")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["data"]["safe_direction"] in {"forward", "left", "right", "backward"}
    assert len(payload["data"]["scan"]) >= 1
    assert payload["data"]["safe_direction"] in {"forward", "stop"}
