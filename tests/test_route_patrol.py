from __future__ import annotations

import json

import pytest

from network.route_patrol import RoutePatrol


def test_recorded_route_persists_and_repeats_from_nearest_waypoint(tmp_path) -> None:
    route_path = tmp_path / "patrol.json"
    patrol = RoutePatrol(route_path)
    patrol.start_recording({"x": 0.0, "y": 0.0, "yaw": 0.0})
    patrol.observe_pose({"x": 0.8, "y": 0.0, "yaw": 0.0})
    saved = patrol.stop_recording({"x": 1.6, "y": 0.0, "yaw": 0.0})

    assert saved["mode"] == "saved"
    assert saved["point_count"] == 3
    assert route_path.exists()
    assert json.loads(route_path.read_text())["version"] == 1

    loaded = RoutePatrol(route_path)
    first_goal = loaded.start({"x": 0.75, "y": 0.0, "yaw": 0.0})
    assert first_goal["x"] == pytest.approx(0.8)
    assert loaded.goal_succeeded()["x"] == pytest.approx(1.6)
    assert loaded.goal_succeeded()["x"] == pytest.approx(0.0)
    assert loaded.state()["patrolling"] is True


def test_non_repeating_patrol_completes_after_last_waypoint(tmp_path) -> None:
    patrol = RoutePatrol(tmp_path / "patrol.json")
    patrol.start_recording({"x": 0.0, "y": 0.0, "yaw": 0.0})
    patrol.stop_recording({"x": 1.2, "y": 0.0, "yaw": 0.0})

    patrol.start({"x": 0.0, "y": 0.0, "yaw": 0.0}, repeat=False)

    assert patrol.goal_succeeded()["x"] == pytest.approx(1.2)
    assert patrol.goal_succeeded() is None
    assert patrol.state()["mode"] == "complete"


def test_recording_rejects_short_routes_and_invalid_poses(tmp_path) -> None:
    patrol = RoutePatrol(tmp_path / "patrol.json")
    patrol.start_recording({"x": 0.0, "y": 0.0, "yaw": 0.0})

    with pytest.raises(ValueError, match="at least 1 m"):
        patrol.stop_recording({"x": 0.2, "y": 0.0, "yaw": 0.0})

    with pytest.raises(ValueError, match="finite"):
        patrol.start_recording({"x": float("nan"), "y": 0.0, "yaw": 0.0})
