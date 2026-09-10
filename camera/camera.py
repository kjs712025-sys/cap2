"""Camera abstraction supporting USB and Raspberry Pi (CSI) cameras."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from config import RobotConfig
from utils.logger import get_logger

logger = get_logger("camera")

PI_CAMERA_SIZE = (1280, 720)
PI_CAMERA_FPS = 60


@dataclass(slots=True)
class CameraFrame:
    """Represents a single frame from the camera."""

    image: np.ndarray
    timestamp: float


class Camera:
    """Camera wrapper supporting USB (V4L2/UVC) and Pi CSI camera sources.

    ``camera_source == "pi"`` uses picamera2/libcamera, which is required for
    Raspberry Pi Camera Module hardware (CSI) — those sensors stream raw
    Bayer data that a plain V4L2 capture cannot decode into a usable image.
    Any other source falls back to OpenCV's V4L2 capture (USB webcams).
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.capture: Optional[cv2.VideoCapture] = None
        self._picam2: Optional[Any] = None
        self._is_running = False

    def _open_pi_camera(self) -> None:
        """Open the CSI camera via picamera2."""
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            logger.warning("picamera2 not installed: %s", exc)
            return

        try:
            picam2 = Picamera2()
            still_config = picam2.create_video_configuration(
                main={"format": "RGB888", "size": PI_CAMERA_SIZE},
                controls={"FrameRate": PI_CAMERA_FPS},
            )
            picam2.configure(still_config)
            picam2.start()
        except RuntimeError as exc:
            logger.warning("Pi camera open failed: %s", exc)
            self._picam2 = None
            return

        self._picam2 = picam2
        self._is_running = True
        logger.info("Pi camera opened via picamera2 at %sx%s @ %sfps", *PI_CAMERA_SIZE, PI_CAMERA_FPS)

    def open(self) -> None:
        """Open the configured camera source."""
        if self.config.camera_source == "pi":
            if self._picam2 is not None:
                return
            self._open_pi_camera()
            return

        if self.capture and self.capture.isOpened():
            return

        try:
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
        if self.config.camera_source == "pi":
            if self._picam2 is None:
                self.open()
            return self._picam2 is not None
        if self.capture is None:
            self.open()
        return self.capture is not None and self.capture.isOpened()

    def initialize(self) -> bool:
        """Attempt to open the camera and return whether initialization succeeded."""
        self.open()
        return self.is_ready()

    def close(self) -> None:
        """Close the camera capture."""
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except RuntimeError:
                logger.warning("Pi camera stop failed")
            self._picam2 = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self._is_running = False

    def read_frame(self) -> Optional[CameraFrame]:
        """Read a single frame from the camera."""
        if self.config.camera_source == "pi":
            try:
                if self._picam2 is None:
                    self.open()
                if self._picam2 is None:
                    return None
                image = self._picam2.capture_array()
                return CameraFrame(image=image, timestamp=cv2.getTickCount())
            except RuntimeError as exc:
                logger.warning("Pi camera read failed: %s", exc)
                return None

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
