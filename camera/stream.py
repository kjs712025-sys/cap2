"""Streaming helpers for camera output."""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Optional

import cv2
import numpy as np

from camera.camera import Camera
from utils.logger import get_logger

logger = get_logger("camera.stream")

_STOP_COLOR = (0, 0, 255)      # red  (BGR)
_WARN_COLOR = (0, 165, 255)    # amber
_LEVEL_COLOR = {"stop": _STOP_COLOR, "suppressed": _STOP_COLOR, "warn": _WARN_COLOR}


class CameraStream:
    """Produces JPEG frames for remote consumption, with a fire-detection overlay."""

    def __init__(self, camera: Camera, fire_monitor: Optional[Any] = None) -> None:
        self.camera = camera
        self.fire_monitor = fire_monitor

    def _draw_fire_boxes(self, image: np.ndarray) -> None:
        """Overlay the fire monitor's most recent bounding boxes onto the frame."""
        if self.fire_monitor is None:
            return
        try:
            boxes = self.fire_monitor.current_boxes()
            level = self.fire_monitor.latest.get("level", "warn")
        except Exception:  # noqa: BLE001 - overlay must never break the stream
            return
        if not boxes:
            return

        color = _LEVEL_COLOR.get(level, _WARN_COLOR)
        h, w = image.shape[:2]
        banner = "FIRE" if level in ("stop", "suppressed") else "FIRE?"
        cv2.rectangle(image, (0, 0), (w - 1, h - 1), color, 3)
        cv2.putText(image, banner, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        for det in boxes:
            box = det.get("box") or []
            if len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label = f"{det.get('label', 'fire')} {float(det.get('confidence', 0)):.2f}"
            cv2.putText(image, label, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

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
                await asyncio.sleep(0.01)
                continue

            image = frame.image
            self._draw_fire_boxes(image)
            _, payload = cv2.imencode(".jpg", image)
            if payload is None:
                break

            frames_emitted += 1
            yield payload.tobytes()
            if max_frames is not None and frames_emitted >= max_frames:
                break
            await asyncio.sleep(0.01)
