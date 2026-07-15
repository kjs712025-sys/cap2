"""Streaming helpers for camera output."""

from __future__ import annotations

import asyncio
from typing import AsyncIterator

import cv2
import numpy as np

from camera.camera import Camera
from utils.logger import get_logger

logger = get_logger("camera.stream")


class CameraStream:
    """Produces JPEG frames for remote consumption."""

    def __init__(self, camera: Camera) -> None:
        self.camera = camera

    async def frame_generator(self, max_frames: int | None = None) -> AsyncIterator[bytes]:
        """Yield encoded JPEG frames asynchronously, falling back to placeholders when needed."""
        frames_emitted = 0
        while True:
            frame = self.camera.read_frame()
            if frame is None:
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder,
                    "no frame",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (255, 255, 255),
                    2,
                )
                _, payload = cv2.imencode(".jpg", placeholder)
                if payload is None:
                    break
                frames_emitted += 1
                yield payload.tobytes()
                if max_frames is not None and frames_emitted >= max_frames:
                    break
                await asyncio.sleep(0.03)
                continue

            _, payload = cv2.imencode(".jpg", frame.image)
            if payload is None:
                break

            frames_emitted += 1
            yield payload.tobytes()
            if max_frames is not None and frames_emitted >= max_frames:
                break
            await asyncio.sleep(0.03)
