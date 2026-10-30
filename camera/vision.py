"""Vision abstractions for object detection and tracking."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np

from ai.llm import LLMService
from utils.logger import get_logger

logger = get_logger("camera.vision")

# Only the fire/smoke classes are treated as hazards.
_FIRE_LABELS = {"fire", "smoke"}
_NCNN_INPUT_SIZE = 640


class _NCNNFireModel:
    """Run the bundled Firebot NCNN export without a PyTorch dependency."""

    names = ("fire", "smoke")

    def __init__(self, model_path: Path) -> None:
        import ncnn

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = min(2, max(1, os.cpu_count() or 1))
        param_result = self.net.load_param(str(model_path / "model.ncnn.param"))
        model_result = self.net.load_model(str(model_path / "model.ncnn.bin"))
        if param_result != 0 or model_result != 0:
            raise RuntimeError(
                f"NCNN could not load model files from {model_path} "
                f"(param={param_result}, model={model_result})"
            )

    def predict(self, image: np.ndarray, confidence: float) -> list[Detection]:
        height, width = image.shape[:2]
        scale = min(_NCNN_INPUT_SIZE / width, _NCNN_INPUT_SIZE / height)
        resized_width = round(width * scale)
        resized_height = round(height * scale)
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        pad_x = (_NCNN_INPUT_SIZE - resized_width) / 2
        pad_y = (_NCNN_INPUT_SIZE - resized_height) / 2
        left, top = round(pad_x - 0.1), round(pad_y - 0.1)
        right = _NCNN_INPUT_SIZE - resized_width - left
        bottom = _NCNN_INPUT_SIZE - resized_height - top
        padded = cv2.copyMakeBorder(
            resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )

        import ncnn

        tensor = ncnn.Mat.from_pixels(
            padded.data,
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            _NCNN_INPUT_SIZE,
            _NCNN_INPUT_SIZE,
        )
        tensor.substract_mean_normalize([0.0, 0.0, 0.0], [1 / 255, 1 / 255, 1 / 255])
        extractor = self.net.create_extractor()
        if extractor.input("in0", tensor) != 0:
            raise RuntimeError("NCNN rejected the fire-model input tensor")
        result_code, output = extractor.extract("out0")
        if result_code != 0:
            raise RuntimeError("NCNN failed to produce fire-model detections")

        rows = output.numpy().T
        if rows.ndim != 2 or rows.shape[1] != 4 + len(self.names):
            raise RuntimeError(f"Unexpected NCNN fire-model output shape: {output.numpy().shape}")

        boxes: list[list[int]] = []
        scores: list[float] = []
        labels: list[str] = []
        coordinates: list[tuple[float, float, float, float]] = []
        for row in rows:
            class_index = int(np.argmax(row[4:]))
            score = float(row[4 + class_index])
            if score < confidence:
                continue
            x1, y1, x2, y2 = (float(value) for value in row[:4])
            x1 = min(max((x1 - pad_x) / scale, 0.0), float(width))
            y1 = min(max((y1 - pad_y) / scale, 0.0), float(height))
            x2 = min(max((x2 - pad_x) / scale, 0.0), float(width))
            y2 = min(max((y2 - pad_y) / scale, 0.0), float(height))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([round(x1), round(y1), round(x2 - x1), round(y2 - y1)])
            scores.append(score)
            labels.append(self.names[class_index])
            coordinates.append((x1, y1, x2, y2))

        kept = cv2.dnn.NMSBoxes(boxes, scores, confidence, 0.45)
        detections = []
        for index in np.asarray(kept).reshape(-1):
            x1, y1, x2, y2 = coordinates[int(index)]
            detections.append(
                Detection(labels[int(index)], scores[int(index)], (x1, y1, x2, y2))
            )
        return detections


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
        """Lazily load the local NCNN or PyTorch fire/smoke model, once."""
        if self._fire_model is not None:
            return self._fire_model
        if self._fire_model_load_failed or not self.fire_model_path:
            return None
        try:
            model_path = Path(self.fire_model_path)
            if model_path.is_dir():
                self._fire_model = _NCNNFireModel(model_path)
            else:
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

        if isinstance(model, _NCNNFireModel):
            return model.predict(image, confidence)

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
