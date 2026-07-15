"""Smoke tests for the robot backend skeleton."""

from __future__ import annotations

import importlib


def test_imports() -> None:
    """Ensure the main package modules import cleanly."""
    modules = [
        "main",
        "config",
        "robot.motion",
        "robot.stm32",
        "camera.camera",
        "camera.vision",
        "lidar.lidar",
        "ai.planner",
        "network.server",
    ]
    for module_name in modules:
        importlib.import_module(module_name)
