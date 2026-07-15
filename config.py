"""Application configuration for the autonomous robot backend."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional


@dataclass(slots=True)
class RobotConfig:
    """Runtime configuration for the robot platform."""

    app_name: str = "AI_Robot"
    debug: bool = False
    log_level: str = "INFO"
    log_file: str = "logs/robot.log"

    uart_port: str = "/dev/ttyAMA0"
    uart_baudrate: int = 115200
    uart_timeout: float = 0.5

    camera_source: str = "usb"
    camera_index: int = 0
    preview_enabled: bool = True

    lidar_port: str = "/dev/ttyUSB0"
    lidar_model: str = "ydlidar_x4"

    openai_api_key: Optional[str] = None
    openai_model: str = "gpt-4o-mini"
    llm_enabled: bool = False

    host: str = "0.0.0.0"
    port: int = 8000

    enable_websocket: bool = True
    enable_rest_api: bool = True

    robot_name: str = "OmniBot"
    max_linear_speed: float = 0.5
    max_angular_speed: float = 1.0

    supported_objects: tuple[str, ...] = field(
        default_factory=lambda: (
            "person",
            "bottle",
            "cup",
            "chair",
            "door",
            "table",
        )
    )

    @classmethod
    def from_env(cls) -> "RobotConfig":
        """Create configuration from environment variables."""
        return cls(
            app_name=os.getenv("ROBOT_APP_NAME", cls.app_name),
            debug=os.getenv("ROBOT_DEBUG", "0") == "1",
            log_level=os.getenv("ROBOT_LOG_LEVEL", cls.log_level),
            log_file=os.getenv("ROBOT_LOG_FILE", cls.log_file),
            uart_port=os.getenv("ROBOT_UART_PORT", cls.uart_port),
            uart_baudrate=int(os.getenv("ROBOT_UART_BAUDRATE", cls.uart_baudrate)),
            uart_timeout=float(os.getenv("ROBOT_UART_TIMEOUT", cls.uart_timeout)),
            camera_source=os.getenv("ROBOT_CAMERA_SOURCE", cls.camera_source),
            camera_index=int(os.getenv("ROBOT_CAMERA_INDEX", cls.camera_index)),
            preview_enabled=os.getenv("ROBOT_PREVIEW_ENABLED", "1") == "1",
            lidar_port=os.getenv("ROBOT_LIDAR_PORT", cls.lidar_port),
            lidar_model=os.getenv("ROBOT_LIDAR_MODEL", cls.lidar_model),
            openai_api_key=os.getenv("OPENAI_API_KEY"),
            openai_model=os.getenv("OPENAI_MODEL", cls.openai_model),
            llm_enabled=os.getenv("ROBOT_LLM_ENABLED", "0") == "1",
            host=os.getenv("ROBOT_HOST", cls.host),
            port=int(os.getenv("ROBOT_PORT", cls.port)),
            enable_websocket=os.getenv("ROBOT_ENABLE_WEBSOCKET", "1") == "1",
            enable_rest_api=os.getenv("ROBOT_ENABLE_REST_API", "1") == "1",
            robot_name=os.getenv("ROBOT_NAME", cls.robot_name),
            max_linear_speed=float(os.getenv("ROBOT_MAX_LINEAR_SPEED", cls.max_linear_speed)),
            max_angular_speed=float(os.getenv("ROBOT_MAX_ANGULAR_SPEED", cls.max_angular_speed)),
        )
