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
    assert payload["data"]["safe_direction"] in {"forward", "left", "right", "backward", "stop"}
