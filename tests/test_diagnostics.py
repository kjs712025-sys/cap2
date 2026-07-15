from __future__ import annotations

import json

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app
from robot.motion import MotionController, VelocityCommand


def test_diagnostics_endpoint() -> None:
    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    health_response = client.get("/health")
    assert health_response.status_code == 200
    health_payload = health_response.json()
    assert health_payload["success"] is True
    assert health_payload["data"]["status"] == "ok"
    assert health_payload["data"]["uptime_seconds"] >= 0.0

    diagnostics_response = client.get("/diagnostics")
    assert diagnostics_response.status_code == 200
    payload = diagnostics_response.json()
    assert payload["success"] is True
    assert payload["data"]["status"] == "ok"
    assert payload["data"]["camera_ready"] in {True, False}
    assert payload["data"]["components"]["camera"]["ready"] in {True, False}
    assert "connected" in payload["data"]["components"]["camera"]
    assert "last_error" in payload["data"]["components"]["lidar"]

    status_payload = app.state.status.to_payload()
    assert json.dumps(status_payload)

    events_response = client.get("/events?limit=5")
    assert events_response.status_code == 200
    events_payload = events_response.json()
    assert events_payload["success"] is True
    assert "events" in events_payload["data"]

    alerts_response = client.get("/alerts")
    assert alerts_response.status_code == 200
    alerts_payload = alerts_response.json()
    assert alerts_payload["success"] is True
    assert "alerts" in alerts_payload["data"]

    controller = MotionController(max_linear=0.5, max_angular=1.0)
    payload = controller.to_uart_payload(VelocityCommand(0.2, 0.0, -0.1))
    assert payload.startswith("V,0.200,0.000,-0.100")
    assert "*" in payload
