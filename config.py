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

    uart_port: str = "/dev/serial0"
    uart_baudrate: int = 115200
    uart_timeout: float = 0.5

    camera_source: str = "usb"
    camera_index: int = 0
    preview_enabled: bool = True
    fire_monitor_enabled: bool = True
    fire_monitor_interval: float = 2.0
    autostart_autonomous: bool = True

    # Local YOLOv8 fire/smoke detector (see camera/vision.py). Two
    # confidence tiers: detections at/above warn are logged as a
    # non-blocking alert, detections at/above stop trigger a full
    # emergency stop. The gap between them absorbs false positives from
    # bright indoor lighting, which this model is prone to.
    fire_model_path: str = "models/fire_detection/fire_yolov8s.pt"
    fire_warn_confidence: float = 0.4
    fire_stop_confidence: float = 0.7
    # Halt autonomous navigation whenever a fire detection is published
    # (any warn-or-higher level). Unlike fire_auto_stop this is a soft halt:
    # it stops the robot and disarms but does not latch the emergency stop,
    # so navigation can be restarted from the dashboard once the view clears.
    fire_halt_on_detect: bool = True
    # The bundled model false-positives on blur / haze / bright light often
    # enough that autonomously halting on it is unsafe. Off by default: a
    # high-confidence detection is logged as "fire_stop_suppressed" instead of
    # engaging the emergency stop. Set true only with a vetted model + camera.
    fire_auto_stop: bool = False

    lidar_port: str = "/dev/ttyUSB0"
    lidar_model: str = "ydlidar_x4"

    # Reactive autonomous navigation (robot/navigation.py). The perception
    # loop always runs; nav_autostart only controls whether it starts issuing
    # motion commands on boot. nav_front_offset_deg calibrates which LiDAR
    # bearing is the robot's forward direction.
    nav_interval: float = 0.25
    nav_front_offset_deg: float = 0.0
    nav_cruise_speed: float = 0.15
    nav_turn_speed: float = 0.5
    nav_autostart: bool = False
    # Seconds to wait after startup before auto-enabling autonomous driving
    # (only when nav_autostart is set) so the LiDAR / camera can enumerate and
    # deliver a first real scan before the robot moves.
    nav_autostart_delay: float = 8.0
    # Fuse monocular camera free-space detection into the LiDAR sectors
    # (camera/depth.py). Runs every nav_camera_interval seconds.
    nav_use_camera: bool = True
    nav_camera_interval: float = 0.5
    nav_camera_fov_deg: float = 62.0

    # MQTT — the backend publishes fire detections and status snapshots to
    # <mqtt_base_topic>/... on the local broker (see network/mqtt.py).
    mqtt_enabled: bool = True
    mqtt_host: str = "localhost"
    mqtt_port: int = 1883
    mqtt_base_topic: str = "robot"

    openai_api_key: Optional[str] = None
    openai_model: str = "gpt-4o-mini"
    gemini_api_key: Optional[str] = None
    gemini_model: str = "gemini-flash-latest"
    gemini_api_url: str = "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
    llm_enabled: bool = False

    # Voice / text manual driving (Gemini interprets a spoken command into a
    # timed velocity). Commands auto-stop after the interpreted duration, and
    # never run longer than manual_command_timeout.
    manual_default_duration: float = 1.5
    manual_command_timeout: float = 4.0

    # External / app access
    cors_origins: str = "*"
    api_token: Optional[str] = None

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
        defaults = cls()
        return cls(
            app_name=os.getenv("ROBOT_APP_NAME", defaults.app_name),
            debug=os.getenv("ROBOT_DEBUG", "0") == "1",
            log_level=os.getenv("ROBOT_LOG_LEVEL", defaults.log_level),
            log_file=os.getenv("ROBOT_LOG_FILE", defaults.log_file),
            uart_port=os.getenv("ROBOT_UART_PORT", defaults.uart_port),
            uart_baudrate=int(os.getenv("ROBOT_UART_BAUDRATE", defaults.uart_baudrate)),
            uart_timeout=float(os.getenv("ROBOT_UART_TIMEOUT", defaults.uart_timeout)),
            camera_source=os.getenv("ROBOT_CAMERA_SOURCE", defaults.camera_source),
            camera_index=int(os.getenv("ROBOT_CAMERA_INDEX", defaults.camera_index)),
            preview_enabled=os.getenv("ROBOT_PREVIEW_ENABLED", "1") == "1",
            fire_monitor_enabled=os.getenv("ROBOT_FIRE_MONITOR_ENABLED", "1") == "1",
            fire_monitor_interval=float(os.getenv("ROBOT_FIRE_MONITOR_INTERVAL", defaults.fire_monitor_interval)),
            autostart_autonomous=os.getenv("ROBOT_AUTOSTART_AUTONOMOUS", "1") == "1",
            fire_model_path=os.getenv("ROBOT_FIRE_MODEL_PATH", defaults.fire_model_path),
            fire_warn_confidence=float(
                os.getenv("ROBOT_FIRE_WARN_CONFIDENCE", defaults.fire_warn_confidence)
            ),
            fire_stop_confidence=float(
                os.getenv("ROBOT_FIRE_STOP_CONFIDENCE", defaults.fire_stop_confidence)
            ),
            fire_auto_stop=os.getenv("ROBOT_FIRE_AUTO_STOP", "0") == "1",
            fire_halt_on_detect=os.getenv("ROBOT_FIRE_HALT_ON_DETECT", "1") == "1",
            lidar_port=os.getenv("ROBOT_LIDAR_PORT", defaults.lidar_port),
            lidar_model=os.getenv("ROBOT_LIDAR_MODEL", defaults.lidar_model),
            nav_interval=float(os.getenv("ROBOT_NAV_INTERVAL", defaults.nav_interval)),
            nav_front_offset_deg=float(
                os.getenv("ROBOT_NAV_FRONT_OFFSET_DEG", defaults.nav_front_offset_deg)
            ),
            nav_cruise_speed=float(os.getenv("ROBOT_NAV_CRUISE_SPEED", defaults.nav_cruise_speed)),
            nav_turn_speed=float(os.getenv("ROBOT_NAV_TURN_SPEED", defaults.nav_turn_speed)),
            nav_autostart=os.getenv("ROBOT_NAV_AUTOSTART", "0") == "1",
            nav_autostart_delay=float(
                os.getenv("ROBOT_NAV_AUTOSTART_DELAY", defaults.nav_autostart_delay)
            ),
            nav_use_camera=os.getenv("ROBOT_NAV_USE_CAMERA", "1") == "1",
            nav_camera_interval=float(os.getenv("ROBOT_NAV_CAMERA_INTERVAL", defaults.nav_camera_interval)),
            nav_camera_fov_deg=float(os.getenv("ROBOT_NAV_CAMERA_FOV_DEG", defaults.nav_camera_fov_deg)),
            mqtt_enabled=os.getenv("ROBOT_MQTT_ENABLED", "1") == "1",
            mqtt_host=os.getenv("ROBOT_MQTT_HOST", defaults.mqtt_host),
            mqtt_port=int(os.getenv("ROBOT_MQTT_PORT", defaults.mqtt_port)),
            mqtt_base_topic=os.getenv("ROBOT_MQTT_BASE_TOPIC", defaults.mqtt_base_topic),
            openai_api_key=os.getenv("OPENAI_API_KEY"),
            openai_model=os.getenv("OPENAI_MODEL", defaults.openai_model),
            gemini_api_key=os.getenv("GEMINI_API_KEY"),
            gemini_model=os.getenv("GEMINI_MODEL", defaults.gemini_model),
            gemini_api_url=os.environ.get(
                "GEMINI_API_URL",
                "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
            ),
            llm_enabled=os.getenv("ROBOT_LLM_ENABLED", "0") == "1",
            manual_default_duration=float(
                os.getenv("ROBOT_MANUAL_DEFAULT_DURATION", defaults.manual_default_duration)
            ),
            manual_command_timeout=float(
                os.getenv("ROBOT_MANUAL_COMMAND_TIMEOUT", defaults.manual_command_timeout)
            ),
            cors_origins=os.getenv("ROBOT_CORS_ORIGINS", defaults.cors_origins),
            api_token=os.getenv("ROBOT_API_TOKEN"),
            host=os.getenv("ROBOT_HOST", defaults.host),
            port=int(os.getenv("ROBOT_PORT", defaults.port)),
            enable_websocket=os.getenv("ROBOT_ENABLE_WEBSOCKET", "1") == "1",
            enable_rest_api=os.getenv("ROBOT_ENABLE_REST_API", "1") == "1",
            robot_name=os.getenv("ROBOT_NAME", defaults.robot_name),
            max_linear_speed=float(os.getenv("ROBOT_MAX_LINEAR_SPEED", defaults.max_linear_speed)),
            max_angular_speed=float(os.getenv("ROBOT_MAX_ANGULAR_SPEED", defaults.max_angular_speed)),
        )
