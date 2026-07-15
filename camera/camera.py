"""Camera abstraction supporting USB and Raspberry Pi cameras."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from config import RobotConfig
from utils.logger import get_logger

logger = get_logger("camera")


@dataclass(slots=True)
class CameraFrame:
    """Represents a single frame from the camera."""

    image: np.ndarray
    timestamp: float


class Camera:
    """Camera wrapper supporting USB and Pi camera sources."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.capture: Optional[cv2.VideoCapture] = None
        self._is_running = False

    def open(self) -> None:
        """Open the configured camera source."""
        if self.capture and self.capture.isOpened():
            return

        try:
            if self.config.camera_source == "pi":
                index = 0
                self.capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
            else:
                self.capture = cv2.VideoCapture(self.config.camera_index)
        except (cv2.error, OSError) as exc:
            self.capture = None
            logger.warning("Camera open failed: %s", exc)
            return

        if not self.capture.isOpened():
            self.capture = None
            logger.warning("Unable to open camera from source %s", self.config.camera_source)
            return

        self._is_running = True
        logger.info("Camera opened from source %s", self.config.camera_source)

    def is_ready(self) -> bool:
        """Return whether the camera abstraction is configured for use."""
        if self.capture is None:
            self.open()
        return self.capture is not None and self.capture.isOpened()

    def initialize(self) -> bool:
        """Attempt to open the camera and return whether initialization succeeded."""
        self.open()
        return self.is_ready()

    def close(self) -> None:
        """Close the camera capture."""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self._is_running = False

    def read_frame(self) -> Optional[CameraFrame]:
        """Read a single frame from the camera."""
        try:
            if self.capture is None:
                self.open()
            if self.capture is None:
                return None
            ok, image = self.capture.read() if self.capture else (False, None)
            if not ok or image is None:
                return None
            return CameraFrame(image=image, timestamp=cv2.getTickCount())
        except (RuntimeError, cv2.error) as exc:
            logger.warning("Camera read failed: %s", exc)
            return None

    def capture_image(self, output_path: str | Path) -> None:
        """Capture an image and save it to disk."""
        frame = self.read_frame()
        if frame is None:
            raise RuntimeError("Unable to capture frame")
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), frame.image)
        logger.info("Saved camera image to %s", path)

    def start_preview(self) -> None:
        """Show a live preview window."""
        while True:
            frame = self.read_frame()
            if frame is None:
                break
            cv2.imshow("Robot Camera", frame.image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()

    def record_video(self, output_path: str | Path, duration_seconds: int = 5) -> None:
        """Record a short video clip."""
        frame = self.read_frame()
        if frame is None:
            raise RuntimeError("Unable to start recording")

        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            20.0,
            (frame.image.shape[1], frame.image.shape[0]),
        )
        if not writer.isOpened():
            raise RuntimeError("Unable to initialize video writer")

        try:
            for _ in range(duration_seconds * 20):
                current = self.read_frame()
                if current is None:
                    break
                writer.write(current.image)
        finally:
            writer.release()
        logger.info("Saved camera video to %s", path)
