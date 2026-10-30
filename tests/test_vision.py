from pathlib import Path

import numpy as np

from camera.vision import VisionService


def test_bundled_ncnn_fire_model_runs_without_detections_on_blank_frame() -> None:
    model_path = Path(__file__).parents[1] / "models/fire_detection/fire_ncnn_model"
    vision = VisionService((), fire_model_path=str(model_path))

    detections = vision.detect_fire(np.zeros((360, 640, 3), dtype=np.uint8))

    assert vision._fire_model is not None
    assert detections == []
