from __future__ import annotations

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app


def test_slam_status_and_map_endpoints() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    response = client.get("/slam/status")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["data"]["pose"]["x"] == 0.0
    assert payload["data"]["pose"]["y"] == 0.0

    motion_response = client.post(
        "/motion/command",
        json={"vx": 0.2, "vy": 0.0, "wz": 0.0},
    )
    assert motion_response.status_code == 200

    updated = client.get("/slam/status")
    assert updated.status_code == 200
    updated_payload = updated.json()
    assert updated_payload["data"]["pose"]["x"] > 0.0

    map_response = client.get("/slam/map")
    assert map_response.status_code == 200
    map_payload = map_response.json()
    assert map_payload["success"] is True
    assert map_payload["data"]["grid"]
    assert map_payload["data"]["scan_points"]

    image_response = client.get("/slam/map/image")
    assert image_response.status_code == 200
    assert image_response.headers["content-type"].startswith("image/png")
    assert len(image_response.content) > 0
