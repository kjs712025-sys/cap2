"""YOLO integration placeholder for future object detection."""

from __future__ import annotations

from typing import Any


class YOLOModel:
    """Placeholder for a future YOLO-based object detector."""

    def __init__(self, model_path: str | None = None) -> None:
        self.model_path = model_path

    def load(self) -> None:
        """Load the ONNX/Torch model."""
        raise NotImplementedError("YOLO support will be implemented later")

    def infer(self, image: Any) -> list[Any]:
        """Run inference on an image."""
        raise NotImplementedError("YOLO support will be implemented later")
