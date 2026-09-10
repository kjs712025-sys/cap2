"""Vision abstractions for object detection and tracking."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import cv2
import numpy as np

from ai.llm import LLMService
from utils.logger import get_logger

logger = get_logger("camera.vision")

# Class names this repo's fire/smoke model actually predicts as "fire".
# ("default" is its third class — background/no-detection, not a hazard.)
_FIRE_LABELS = {"fire", "smoke"}


@dataclass(slots=True)
class Detection:
    """A single object detection result."""

    label: str
    confidence: float
    box: tuple[float, float, float, float]
    track_id: int | None = None


class VisionService:
    """Object/fire detection service, backed by a local YOLOv8 model for fire."""

    def __init__(
        self,
        supported_objects: Iterable[str],
        llm: LLMService | None = None,
        fire_model_path: str | None = None,
    ) -> None:
        self.supported_objects = tuple(supported_objects)
        self.llm = llm
        self.fire_model_path = fire_model_path
        self._fire_model: Optional[object] = None
        self._fire_model_load_failed = False

    def detect(self, image: np.ndarray) -> list[Detection]:
        """Return empty detections until a general-purpose model is attached."""
        return []

    def track(self, detections: Iterable[Detection]) -> list[Detection]:
        """Return detections unchanged for now."""
        return list(detections)

    def _load_fire_model(self) -> Optional[object]:
        """Lazily load the local YOLOv8 fire/smoke model, once."""
        if self._fire_model is not None:
            return self._fire_model
        if self._fire_model_load_failed or not self.fire_model_path:
            return None
        try:
            from ultralytics import YOLO

            self._fire_model = YOLO(self.fire_model_path)
            logger.info("Loaded fire detection model from %s", self.fire_model_path)
        except Exception as exc:  # noqa: BLE001 - report and disable, don't crash the monitor loop
            self._fire_model_load_failed = True
            logger.warning("Fire detection model unavailable (%s): %s", self.fire_model_path, exc)
            return None
        return self._fire_model

    def detect_fire(self, image: np.ndarray, confidence: float = 0.25) -> list[Detection]:
        """Run local YOLOv8 inference and return only fire/smoke detections."""
        model = self._load_fire_model()
        if model is None:
            return []

        results = model.predict(image, verbose=False, conf=confidence)
        detections: list[Detection] = []
        for result in results:
            for box in result.boxes:
                label = str(model.names[int(box.cls[0])])
                if label.lower() not in _FIRE_LABELS:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                detections.append(
                    Detection(label=label, confidence=float(box.conf[0]), box=(x1, y1, x2, y2))
                )
        return detections

    async def analyze(self, image: np.ndarray) -> dict[str, object]:
        """Analyze an image with Gemini when configured."""
        if self.llm is None:
            return {"summary": "Vision LLM is not configured", "detections": []}
        ok, encoded = cv2.imencode(".jpg", image)
        if not ok:
            raise RuntimeError("Unable to encode image for vision analysis")
        return await self.llm.analyze_image(
            encoded.tobytes(),
            supported_objects=self.supported_objects,
        )

    @staticmethod
    def fire_detected(analysis: dict[str, object]) -> bool:
        """Return whether a Gemini vision response indicates fire or smoke."""
        if analysis.get("fire_detected") is True:
            return True
        detections = analysis.get("detections", [])
        if isinstance(detections, list):
            for detection in detections:
                if isinstance(detection, dict):
                    label = str(detection.get("label", "")).lower()
                    if any(term in label for term in ("fire", "flame", "smoke")):
                        return True
        summary = str(analysis.get("summary", "")).lower()
        return any(term in summary for term in ("fire", "flame", "smoke"))
