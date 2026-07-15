"""Vision abstractions for object detection and tracking."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(slots=True)
class Detection:
    """A single object detection result."""

    label: str
    confidence: float
    box: tuple[float, float, float, float]
    track_id: int | None = None


class VisionService:
    """Placeholder vision service that can be backed by YOLO later."""

    def __init__(self, supported_objects: Iterable[str]) -> None:
        self.supported_objects = tuple(supported_objects)

    def detect(self, image: np.ndarray) -> list[Detection]:
        """Return empty detections until a model is attached."""
        return []

    def track(self, detections: Iterable[Detection]) -> list[Detection]:
        """Return detections unchanged for now."""
        return list(detections)
