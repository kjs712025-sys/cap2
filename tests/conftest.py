"""Pytest configuration for the AI Robot project."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@pytest.fixture(autouse=True)
def _no_real_hardware(monkeypatch):
    """Keep the test suite off the real serial port / camera.

    ``create_app()`` calls ``lidar.initialize()`` and ``camera.initialize()``;
    on the robot host those open ``/dev/ttyUSB0`` and the CSI camera, which
    makes tests race the live service for the physical devices. Tests that
    need specific hardware behaviour patch it themselves.
    """

    def _fail(*args, **kwargs):  # pragma: no cover - trivial
        raise OSError("hardware disabled in tests")

    monkeypatch.setattr("lidar.lidar.serial.Serial", _fail, raising=False)
    monkeypatch.setattr("camera.camera.cv2.VideoCapture", lambda *a, **k: _StubCapture(), raising=False)
    yield


class _StubCapture:
    def isOpened(self):
        return False

    def read(self):
        return False, None

    def release(self):
        return None
