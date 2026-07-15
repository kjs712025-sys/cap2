from __future__ import annotations

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app


def test_telemetry_endpoint() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    response = client.get("/telemetry")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["data"]["robot_name"] == "OmniBot"
    assert payload["data"]["mode"] in {"idle", "manual", "autonomous", "charging", "error"}
