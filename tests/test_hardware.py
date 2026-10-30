from __future__ import annotations

import struct
from concurrent.futures import Future
from unittest.mock import MagicMock, patch

import numpy as np

from config import RobotConfig
from camera.camera import Camera, CameraFrame
from lidar.lidar import LidarSensor


def test_camera_initialize_reports_readiness() -> None:
    camera = Camera(RobotConfig(debug=True, camera_source="usb"))
    with patch("camera.camera.cv2.VideoCapture") as video_capture_cls:
        capture = MagicMock()
        capture.isOpened.return_value = True
        video_capture_cls.return_value = capture

        assert camera.initialize() is True


def test_camera_source_override_is_respected(monkeypatch) -> None:
    monkeypatch.setenv("ROBOT_CAMERA_SOURCE", "usb")
    assert RobotConfig.default_camera_source() == "usb"


def test_pi_camera_frame_read_is_non_blocking_until_frame_arrives() -> None:
    camera = Camera(RobotConfig(debug=True, camera_source="pi"))
    picam2 = MagicMock()
    capture_job = MagicMock()
    result = Future()
    capture_job.get_result.side_effect = lambda timeout: result.result(timeout=timeout)
    picam2.capture_array.return_value = capture_job
    camera._picam2 = picam2

    assert camera.read_frame() is None
    picam2.capture_array.assert_called_once_with(wait=False)
    capture_job.get_result.assert_not_called()

    image = np.zeros((2, 2, 3), dtype=np.uint8)
    result.set_result(image)
    frame = camera.read_frame()

    assert frame is not None
    assert frame.image is image
    capture_job.get_result.assert_called_once_with(timeout=0)
    assert camera._pi_capture_job is None


def test_capture_image_waits_for_the_next_camera_frame(tmp_path) -> None:
    camera = Camera(RobotConfig(debug=True, camera_source="pi"))
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    camera.read_frame = MagicMock(side_effect=[None, CameraFrame(image, 0.0)])
    output_path = tmp_path / "capture.jpg"

    with patch("camera.camera.cv2.imwrite", return_value=True) as imwrite:
        camera.capture_image(output_path)

    imwrite.assert_called_once_with(str(output_path), image)


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
        self.read_calls = 0

    def __call__(self, *args: object, **kwargs: object) -> "_FakeLidarSerial":
        return self

    def read(self, size: int = 1) -> bytes:
        self.read_calls += 1
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


def test_lidar_shares_recent_hardware_scan_between_readers() -> None:
    stream = (
        _build_ydlidar_packet(1.0, 100.0)
        + _build_ydlidar_packet(350.0, 300.0)
        + _build_ydlidar_packet(10.0, 400.0)
    )
    fake_serial = _FakeLidarSerial(stream)
    lidar = LidarSensor(RobotConfig(debug=True, simulation=False))

    with patch("lidar.lidar.serial.Serial", fake_serial):
        assert lidar.initialize() is True
        scan = lidar.read_scan()
        read_calls_after_scan = fake_serial.read_calls

        assert lidar.read_scan() == scan

    assert fake_serial.read_calls == read_calls_after_scan
    assert lidar.status.last_scan_hardware is True


def test_lidar_reconnects_after_device_appears() -> None:
    stream = (
        _build_ydlidar_packet(1.0, 100.0)
        + _build_ydlidar_packet(350.0, 300.0)
        + _build_ydlidar_packet(10.0, 400.0)
    )
    fake_serial = _FakeLidarSerial(stream)
    attempts = 0

    def open_serial(*args: object, **kwargs: object) -> _FakeLidarSerial:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise FileNotFoundError("LiDAR device is not available yet")
        return fake_serial

    lidar = LidarSensor(RobotConfig(debug=True, simulation=False))

    with (
        patch("lidar.lidar.serial.Serial", open_serial),
        patch("lidar.lidar.RECONNECT_BACKOFF_S", 0.0),
    ):
        assert lidar.read_scan()
        assert lidar.status.connected is False

        scan = lidar.read_scan()

    assert attempts == 2
    assert lidar.status.connected is True
    assert lidar.status.last_scan_hardware is True
    assert [(p.angle_deg, p.distance_m) for p in scan] == [(350.0, 0.3), (10.0, 0.4)]
