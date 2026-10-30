from __future__ import annotations

from lidar.lidar import ScanPoint
from robot.wall_follow import RightWallFollower


def _scan(*points: tuple[float, float]) -> list[ScanPoint]:
    return [
        ScanPoint(angle_deg=angle, distance_m=distance)
        for angle, distance in points
    ]


def test_searches_for_right_wall_when_none_is_in_range() -> None:
    decision = RightWallFollower().step(
        _scan(
            (0.0, 3.0),
            (-45.0, 2.0),
            (45.0, 2.0),
            (-90.0, 2.0),
            (90.0, 2.0),
            (180.0, 2.0),
        )
    )

    assert decision.phase == "search_wall"
    assert decision.command.vx == 0.0
    assert decision.command.vy == 0.0
    assert decision.command.wz < 0.0


def test_moves_sideways_toward_wall_before_following_it() -> None:
    decision = RightWallFollower().step(_scan((0.0, 3.0), (-90.0, 1.0)))

    assert decision.phase == "approach_wall"
    assert decision.command.vx == 0.0
    assert decision.command.vy < 0.0


def test_follows_clear_right_wall_at_requested_low_speed() -> None:
    decision = RightWallFollower().step(_scan((0.0, 3.0), (-90.0, 0.8)))

    assert decision.phase == "follow_wall"
    assert decision.command.vx == 0.04
    assert decision.command.vy == 0.0
    assert decision.command.wz == 0.0


def test_moves_left_when_right_wall_is_inside_safe_clearance() -> None:
    decision = RightWallFollower().step(
        _scan((0.0, 3.0), (-90.0, 0.3), (45.0, 1.2), (90.0, 1.2))
    )

    assert decision.phase == "avoid_shift"
    assert decision.command.vx == 0.0
    assert decision.command.vy > 0.0
    assert decision.command.wz == 0.0
    assert "shifting slowly left" in decision.reason


def test_uses_calibrated_lidar_forward_offset_for_left_escape() -> None:
    decision = RightWallFollower(front_offset_deg=30.0).step(
        _scan(
            (30.0, 2.0),
            (300.0, 0.3),
            (75.0, 1.2),
            (120.0, 1.2),
        )
    )

    assert decision.phase == "avoid_shift"
    assert decision.command.vy > 0.0
    assert "Right wall too close" in decision.reason


def test_shifts_toward_measured_clear_side_when_front_is_blocked() -> None:
    decision = RightWallFollower().step(
        _scan(
            (0.0, 0.35),
            (-45.0, 0.5),
            (45.0, 1.2),
            (-90.0, 0.5),
            (90.0, 1.2),
        )
    )

    assert decision.phase == "avoid_shift"
    assert decision.command.vx == 0.0
    assert decision.command.vy > 0.0
    assert decision.command.wz == 0.0


def test_uses_side_clearance_when_front_diagonal_also_sees_obstacle() -> None:
    decision = RightWallFollower().step(
        _scan(
            (0.0, 0.75),
            (45.0, 0.7),
            (90.0, 1.3),
            (-45.0, 0.8),
            (-90.0, 0.4),
        )
    )

    assert decision.phase == "avoid_shift"
    assert decision.command.vy > 0.0
    assert "shifting slowly left" in decision.reason


def test_stops_instead_of_moving_into_unobserved_space() -> None:
    decision = RightWallFollower().step(_scan((0.0, 0.35)))

    assert decision.phase == "blocked"
    assert decision.command.vx == decision.command.vy == decision.command.wz == 0.0
    assert "no measured lateral escape path" in decision.reason


def test_stops_if_no_clear_turning_space_at_dead_end() -> None:
    decision = RightWallFollower().step(
        _scan(
            (0.0, 0.3),
            (-45.0, 0.3),
            (45.0, 0.3),
            (-90.0, 0.3),
            (90.0, 0.3),
        )
    )

    assert decision.phase == "blocked"
    assert decision.command.vx == 0.0
    assert decision.command.vy == 0.0
    assert decision.command.wz == 0.0


def test_empty_scan_stops_instead_of_guessing_a_path() -> None:
    decision = RightWallFollower().step([])

    assert decision.phase == "blocked"
    assert decision.command.vx == decision.command.vy == decision.command.wz == 0.0
