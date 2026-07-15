from __future__ import annotations

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


def test_lidar_initialize_reports_readiness() -> None:
    lidar = LidarSensor(RobotConfig(debug=True))
    assert lidar.initialize() is True
