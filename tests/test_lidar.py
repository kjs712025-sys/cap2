from __future__ import annotations

from config import RobotConfig
from lidar.lidar import LidarSensor


def test_lidar_parser_handles_text() -> None:
    sensor = LidarSensor(RobotConfig(debug=True))
    scan = sensor.parse_scan_text("-30:2.0,0:1.2,30:2.5")

    assert len(scan) == 3
    assert scan[1].distance_m == 1.2
