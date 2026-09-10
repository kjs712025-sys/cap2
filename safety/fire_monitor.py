"""Background fire detection, emergency stopping, and alerting.

Two-tier confidence response (the local YOLOv8 fire/smoke model false-positives
on bright indoor lighting): a mid-confidence detection only raises a
non-blocking warning; a high-confidence one triggers an emergency stop when
``auto_stop`` is enabled.

On every detection the monitor also:
  * keeps the latest bounding boxes in ``self.latest`` so the camera stream
    can draw them over the live feed, and
  * publishes the detection to the ``<base>/fire`` MQTT topic.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, Optional

from camera.camera import Camera
from camera.vision import Detection, VisionService
from robot.status import RobotMode, RobotStatus
from robot.stm32 import STM32Controller
from utils.event_log import EventLogger
from utils.logger import get_logger

logger = get_logger("safety.fire")

# Rate-limit repeated warning logs / MQTT re-publishes for a persistent
# (usually false-positive) detection.
_WARN_LOG_INTERVAL_S = 30.0
# How long a bounding box stays drawn on the stream after its last sighting.
DETECTION_TTL_S = 4.0


class FireMonitor:
    """Periodically analyze the camera feed and respond to fire/smoke detections."""

    def __init__(
        self,
        camera: Camera,
        vision: VisionService,
        stm32: STM32Controller,
        status: RobotStatus,
        interval: float = 2.0,
        warn_confidence: float = 0.4,
        stop_confidence: float = 0.7,
        auto_stop: bool = False,
        halt_on_detect: bool = True,
        halt_consecutive: int = 2,
        event_logger: Optional[EventLogger] = None,
        mqtt: Optional[Any] = None,
        navigator: Optional[Any] = None,
    ) -> None:
        self.camera = camera
        self.vision = vision
        self.stm32 = stm32
        self.status = status
        self.interval = max(0.5, interval)
        self.warn_confidence = warn_confidence
        self.stop_confidence = stop_confidence
        self.auto_stop = auto_stop
        self.halt_on_detect = halt_on_detect
        # This model false-positives on bright indoor light; require the
        # detection to persist across this many consecutive checks before it
        # actually halts autonomous driving (the alert / siren / MQTT still
        # fire on the first sighting).
        self.halt_consecutive = max(1, halt_consecutive)
        self._actionable_streak = 0
        self.event_logger = event_logger
        self.mqtt = mqtt
        self.navigator = navigator
        self.triggered = False
        self._last_warn_log_at = 0.0
        self._last_suppressed_log_at = 0.0
        self._last_publish_at = 0.0
        self._published_level = "clear"
        # Shared with CameraStream for the live overlay.
        self.latest: dict[str, Any] = {"level": "clear", "detections": [], "timestamp": 0.0}

    def current_boxes(self) -> list[dict[str, Any]]:
        """Fire detections recent enough to still draw on the stream."""
        if time.monotonic() - self.latest.get("_mono", 0.0) > DETECTION_TTL_S:
            return []
        return self.latest.get("detections", [])

    def snapshot(self) -> dict[str, Any]:
        """Public fire payload — identical to what is published on the
        ``<base>/fire`` MQTT topic, for the /fire/state + /fire/stream feeds."""
        return {k: v for k, v in self.latest.items() if not str(k).startswith("_")}

    def _record(self, level: str, detections: list[Detection]) -> None:
        self.latest = {
            "level": level,
            "detections": [
                {
                    "label": d.label,
                    "confidence": round(d.confidence, 3),
                    "box": [round(v, 1) for v in d.box],
                }
                for d in detections
            ],
            "timestamp": time.time(),
            "_mono": time.monotonic(),
        }
        self._publish(level)

    def _publish(self, level: str, *, force: bool = False) -> None:
        if self.mqtt is None:
            return
        now = time.monotonic()
        changed = level != self._published_level
        if not force and not changed and now - self._last_publish_at < _WARN_LOG_INTERVAL_S:
            return
        self._published_level = level
        self._last_publish_at = now
        payload = {k: v for k, v in self.latest.items() if not k.startswith("_")}
        payload["level"] = level
        self.mqtt.publish("fire", payload, retain=True)

    async def check_once(self) -> dict[str, object] | None:
        """Analyze one frame and warn or stop the robot based on confidence."""
        frame = self.camera.read_frame()
        if frame is None:
            return None

        # YOLO inference is a blocking, CPU-bound call (~2s on this hardware);
        # run it off the event loop so it doesn't stall request handling.
        floor = max(0.1, self.warn_confidence - 0.05)
        detections = await asyncio.to_thread(self.vision.detect_fire, frame.image, floor)
        actionable = [d for d in detections if d.confidence >= self.warn_confidence]

        if not actionable:
            self._actionable_streak = 0
            self.status.metadata.pop("fire_warning", None)
            self.status.metadata.pop("fire_alert", None)
            if self.latest["level"] != "clear":
                self._record("clear", [])
            return {"detections": []}

        self._actionable_streak += 1
        best = max(actionable, key=lambda detection: detection.confidence)
        if best.confidence >= self.stop_confidence and self.auto_stop:
            level = "stop"
            self.stop_for_fire(best)
        elif best.confidence >= self.stop_confidence:
            level = "suppressed"
            self._suppressed_stop(best)
        else:
            level = "warn"
            self.warn_for_fire(best)

        # Any actionable detection halts autonomous driving (soft stop) — this
        # is what fires the robot/fire topic and the dashboard siren.
        self._halt_for_fire(level, best)
        self._record(level, actionable)

        return {
            "level": level,
            "detections": [
                {"label": d.label, "confidence": round(d.confidence, 3), "box": [round(v, 1) for v in d.box]}
                for d in actionable
            ],
        }

    def _halt_for_fire(self, level: str, detection: Detection) -> None:
        """Soft-stop the robot on a persistent fire detection (does not latch
        the e-stop). The alert flag is set immediately; the navigation halt
        waits for ``halt_consecutive`` consecutive sightings to ride out the
        model's lighting false-positives."""
        self.status.metadata["fire_alert"] = level
        if not self.halt_on_detect or self._actionable_streak < self.halt_consecutive:
            return
        navigator = self.navigator
        if navigator is not None and getattr(navigator, "enabled", False):
            navigator.disable()  # sends zero velocity + disarms the STM32
            self.status.mission = "fire_halt"
            self.status.ai_state = "fire_alert"
            logger.warning(
                "Fire detection (%s, %s %.2f) — autonomous navigation halted",
                level,
                detection.label,
                detection.confidence,
            )
            if self.event_logger is not None:
                self.event_logger.log(
                    "fire_halt",
                    {"level": level, "label": detection.label, "confidence": round(detection.confidence, 3)},
                )

    def warn_for_fire(self, detection: Detection) -> None:
        """Record a below-stop-threshold detection without halting the robot."""
        self.status.metadata["fire_warning"] = f"{detection.label} {detection.confidence:.2f}"

        now = time.monotonic()
        if now - self._last_warn_log_at < _WARN_LOG_INTERVAL_S:
            return
        self._last_warn_log_at = now

        logger.warning(
            "Possible %s detected (confidence %.2f, below %.2f stop threshold)",
            detection.label,
            detection.confidence,
            self.stop_confidence,
        )
        if self.event_logger is not None:
            self.event_logger.log(
                "fire_warning",
                {"label": detection.label, "confidence": round(detection.confidence, 3)},
            )

    def _suppressed_stop(self, detection: Detection) -> None:
        """A stop-threshold detection while auto_stop is disabled: flag it loudly
        but keep the robot running."""
        self.status.metadata["fire_warning"] = f"{detection.label} {detection.confidence:.2f} (stop suppressed)"

        now = time.monotonic()
        if now - self._last_suppressed_log_at < _WARN_LOG_INTERVAL_S:
            return
        self._last_suppressed_log_at = now

        logger.error(
            "%s detected at %.2f (>= %.2f stop threshold) but fire_auto_stop is off; not halting",
            detection.label,
            detection.confidence,
            self.stop_confidence,
        )
        if self.event_logger is not None:
            self.event_logger.log(
                "fire_stop_suppressed",
                {"label": detection.label, "confidence": round(detection.confidence, 3)},
            )

    def stop_for_fire(self, detection: Optional[Detection] = None) -> None:
        """Latch the emergency stop so motion cannot continue after detection."""
        self.stm32.emergency_stop()
        self.status.update_speed(0.0, 0.0, 0.0)
        self.status.set_mode(RobotMode.ERROR)
        self.status.mission = "fire_detected"
        self.status.ai_state = "fire_emergency_stop"
        label = detection.label if detection else "fire"
        confidence = detection.confidence if detection else None
        self.status.last_error = (
            f"{label} detected by local vision model (confidence {confidence:.2f})"
            if confidence is not None
            else "Fire or smoke detected"
        )
        self.status.metadata["fire_detected"] = "true"
        if self.event_logger is not None:
            self.event_logger.log(
                "fire_detected",
                {"label": label, "confidence": round(confidence, 3) if confidence is not None else None},
            )
        if not self.triggered:
            logger.critical(
                "%s detected (confidence %s); emergency stop engaged", label, confidence
            )
            self.triggered = True

    async def run(self) -> None:
        """Continuously monitor the configured camera until cancelled.

        The retained ``<base>/fire`` baseline is refreshed by the server's
        status loop; ``_record`` publishes immediately on any state change.
        """
        while True:
            try:
                await self.check_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # pragma: no cover - hardware/model dependent
                logger.warning("Fire monitor check failed: %s", exc)
            await asyncio.sleep(self.interval)
