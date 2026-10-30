from __future__ import annotations

from config import RobotConfig
from lidar.lidar import LidarSensor


def test_lidar_parser_handles_text() -> None:
    sensor = LidarSensor(RobotConfig(debug=True))
    scan = sensor.parse_scan_text("-30:2.0,0:1.2,30:2.5")

    assert len(scan) == 3
    assert scan[1].distance_m == 1.2


def test_missing_hardware_fallback_is_not_marked_as_real_scan() -> None:
    sensor = LidarSensor(RobotConfig(debug=True, simulation=False))

    scan = sensor.read_scan()

    assert scan
    assert sensor.status.last_scan_hardware is False


def test_only_complete_hardware_rotation_is_nav_ready() -> None:
    sensor = LidarSensor(RobotConfig(debug=True, simulation=False))
    sensor.status.connected = True
    sensor.serial = object()
    packets = iter(
        [
            (350.0, 355.0, [4000, 4000]),
            (0.0, 5.0, [4000, 4000]),
        ]
    )
    sensor._read_packet = lambda: next(packets)

    scan = sensor._read_scan_locked()

    assert scan
    assert sensor.status.last_scan_hardware is True


def test_partial_rotation_is_not_nav_ready(monkeypatch) -> None:
    import lidar.lidar as lidar_module

    sensor = LidarSensor(RobotConfig(debug=True, simulation=False))
    sensor.status.connected = True
    sensor.serial = object()
    sensor._read_packet = lambda: (10.0, 20.0, [4000, 4000])
    clock_values = iter([0.0, 0.1, 0.6, 0.6])
    monkeypatch.setattr(lidar_module.time, "monotonic", lambda: next(clock_values))

    scan = sensor._read_scan_locked()

    assert scan
    assert sensor.status.last_scan_hardware is False
