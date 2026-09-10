from __future__ import annotations

import base64

from fastapi.testclient import TestClient

from config import RobotConfig
from network.server import create_app
from slam.slam import GRID, SimpleSlam


def test_raycast_builds_free_space_and_obstacle() -> None:
    slam = SimpleSlam()
    for _ in range(5):
        slam.update_from_lidar([{"angle_deg": 0.0, "distance_m": 1.0}])

    snap = slam.snapshot()
    assert snap["size"] == GRID
    cells = base64.b64decode(snap["grid_b64"])
    assert len(cells) == GRID * GRID
    # A beam straight ahead (1 m) clears the cells in front and marks the hit.
    centre = GRID // 2
    ahead_free = centre + int(0.5 / snap["resolution_m"])
    hit = centre + int(1.0 / snap["resolution_m"])
    assert cells[centre * GRID + ahead_free] == 1  # free
    assert cells[centre * GRID + hit] == 2  # occupied
    assert snap["explored_frac"] > 0.0


def test_trail_follows_pose() -> None:
    slam = SimpleSlam()
    for _ in range(20):
        slam.update_from_motion(0.3, 0.0, 0.0, dt=0.1)
    assert len(slam.trail) > 1
    assert slam.trail[-1][0] > slam.trail[0][0]


def test_snapshot_payload_shape() -> None:
    slam = SimpleSlam()
    slam.update_from_lidar([{"angle_deg": a, "distance_m": 1.5} for a in range(-90, 91, 10)])
    snap = slam.snapshot()
    for key in ("resolution_m", "size", "origin_m", "pose", "trail", "grid_b64", "explored_frac"):
        assert key in snap
    assert isinstance(snap["grid_b64"], str)


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
