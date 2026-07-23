"""Vision abstractions for object detection and tracking."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np

from ai.llm import LLMService


@dataclass(slots=True)
class Detection:
    """A single object detection result."""

    label: str
    confidence: float
    box: tuple[float, float, float, float]
    track_id: int | None = None


class VisionService:
    """Placeholder vision service that can be backed by YOLO later."""

    def __init__(self, supported_objects: Iterable[str], llm: LLMService | None = None) -> None:
        self.supported_objects = tuple(supported_objects)
        self.llm = llm

    def detect(self, image: np.ndarray) -> list[Detection]:
        """Return empty detections until a model is attached."""
        return []

    def track(self, detections: Iterable[Detection]) -> list[Detection]:
        """Return detections unchanged for now."""
        return list(detections)

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
        """Return whether a vision response indicates fire or smoke."""
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
