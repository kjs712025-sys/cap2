from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

import cv2
import numpy as np

from camera.stream import CameraStream


def _decode_first_frame(stream: CameraStream) -> np.ndarray:
    async def run() -> bytes:
        async for jpg in stream.frame_generator(max_frames=1):
            return jpg
        raise AssertionError("no frame produced")

    jpg = asyncio.run(run())
    return cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)


def _camera_with_grey_frame() -> MagicMock:
    camera = MagicMock()
    frame = MagicMock()
    frame.image = np.full((200, 320, 3), 40, dtype=np.uint8)
    camera.read_frame.return_value = frame
    return camera


def test_no_overlay_when_no_fire_boxes() -> None:
    fire_monitor = MagicMock()
    fire_monitor.current_boxes.return_value = []
    fire_monitor.latest = {"level": "clear"}

    img = _decode_first_frame(CameraStream(_camera_with_grey_frame(), fire_monitor=fire_monitor))
    # top edge stays close to the flat grey fill
    assert int(np.max(img[1, :, 2])) < 90


def test_fire_boxes_draw_a_red_border() -> None:
    fire_monitor = MagicMock()
    fire_monitor.current_boxes.return_value = [
        {"label": "Fire", "confidence": 0.88, "box": [50, 40, 180, 150]}
    ]
    fire_monitor.latest = {"level": "stop"}

    img = _decode_first_frame(CameraStream(_camera_with_grey_frame(), fire_monitor=fire_monitor))
    top_edge = img[1, :, :]
    red = np.sum((top_edge[:, 2] > 150) & (top_edge[:, 0] < 80))
    assert red > 100
