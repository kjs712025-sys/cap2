from __future__ import annotations

import struct
from unittest.mock import MagicMock, patch

from config import RobotConfig
from camera.camera import Camera
from lidar.lidar import LidarSensor


def test_camera_initialize_reports_readiness() -> None:
    camera = Camera(RobotConfig(debug=True))
    with patch("camera.camera.cv2.VideoCapture") as video_capture_cls:
        capture = MagicMock()
        capture.isOpened.return_value = True
        video_capture_cls.return_value = capture

        assert camera.initialize() is True


def _build_ydlidar_packet(angle_deg: float, distance_mm: float) -> bytes:
    """Build a minimal one-sample YDLiDAR X-series packet for tests."""
    raw_angle = int(round(angle_deg * 128)) & 0xFFFF
    raw_distance = int(round(distance_mm * 4)) & 0xFFFF
    return (
        b"\xaa\x55"
        + bytes([0x02, 0x01])  # CT, sample count (LSN)
        + struct.pack("<H", raw_angle)  # start angle
        + struct.pack("<H", raw_angle)  # end angle (single-sample packet)
        + b"\x00\x00"  # checksum, unchecked by the driver
        + struct.pack("<H", raw_distance)
    )


class _FakeLidarSerial:
    """Serves a scripted byte stream in place of a real serial port."""

    def __init__(self, data: bytes) -> None:
        self._buffer = data
        self.is_open = True

    def __call__(self, *args: object, **kwargs: object) -> "_FakeLidarSerial":
        return self

    def read(self, size: int = 1) -> bytes:
        chunk = self._buffer[:size]
        self._buffer = self._buffer[size:]
        return chunk

    def close(self) -> None:
        self.is_open = False


def test_lidar_initialize_reports_readiness() -> None:
    fake_serial = _FakeLidarSerial(_build_ydlidar_packet(0.0, 500.0))
    lidar = LidarSensor(RobotConfig(debug=True))

    with patch("lidar.lidar.serial.Serial", fake_serial):
        assert lidar.initialize() is True


def test_lidar_read_scan_parses_real_packets() -> None:
    # connect() syncs to and consumes the first header it sees, so the first
    # packet in the stream is a throwaway used only to establish sync.
    stream = (
        _build_ydlidar_packet(1.0, 100.0)
        + _build_ydlidar_packet(350.0, 300.0)
        + _build_ydlidar_packet(10.0, 400.0)  # angle drop => end of rotation
    )
    fake_serial = _FakeLidarSerial(stream)
    lidar = LidarSensor(RobotConfig(debug=True))

    with patch("lidar.lidar.serial.Serial", fake_serial):
        assert lidar.initialize() is True
        scan = lidar.read_scan()

    assert [(p.angle_deg, p.distance_m) for p in scan] == [(350.0, 0.3), (10.0, 0.4)]
