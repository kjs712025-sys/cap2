from __future__ import annotations

from config import RobotConfig
from camera.stream import CameraStream
from lidar.lidar import LidarSensor


class DummyCamera:
    def __init__(self) -> None:
        self.calls = 0

    def read_frame(self):
        self.calls += 1
        return None


def test_camera_stream_emits_placeholder_when_frame_missing() -> None:
    stream = CameraStream(DummyCamera())

    async def collect() -> list[bytes]:
        frames = []
        async for frame in stream.frame_generator(max_frames=1):
            frames.append(frame)
        return frames

    import asyncio

    frames = asyncio.run(collect())
    assert len(frames) == 1
    assert frames[0]


def test_lidar_parse_scan_text_supports_multiple_separators() -> None:
    lidar = LidarSensor(RobotConfig(debug=True))
    points = lidar.parse_scan_text("0:1.2,30=2.5")
    assert len(points) == 2
    assert points[0].distance_m == 1.2
    assert points[1].distance_m == 2.5
