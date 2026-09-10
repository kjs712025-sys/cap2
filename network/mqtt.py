"""Thin MQTT publisher for robot telemetry and events.

Wraps ``paho-mqtt`` with a background network loop and best-effort publishing:
if the broker is down the calls are no-ops rather than raising, so callers
(fire monitor, navigator, status loop) never have to care.
"""

from __future__ import annotations

import json
from typing import Any, Optional

from utils.logger import get_logger

logger = get_logger("network.mqtt")


class MqttPublisher:
    """Best-effort MQTT publisher with a base topic prefix."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 1883,
        base_topic: str = "robot",
        client_id: str = "ai-robot-backend",
        enabled: bool = True,
    ) -> None:
        self.host = host
        self.port = port
        self.base_topic = base_topic.rstrip("/")
        self.enabled = enabled
        self._client: Optional[Any] = None
        self.connected = False

    def start(self) -> None:
        """Connect to the broker and start the background network loop."""
        if not self.enabled:
            return
        try:
            import paho.mqtt.client as mqtt

            try:
                client = mqtt.Client(
                    mqtt.CallbackAPIVersion.VERSION2, client_id="ai-robot-backend"
                )
            except AttributeError:  # paho-mqtt < 2.0
                client = mqtt.Client(client_id="ai-robot-backend")

            def _on_connect(*_args: Any) -> None:
                self.connected = True
                logger.info("MQTT connected to %s:%s", self.host, self.port)

            def _on_disconnect(*_args: Any) -> None:
                self.connected = False
                logger.warning("MQTT disconnected from %s:%s", self.host, self.port)

            client.on_connect = _on_connect
            client.on_disconnect = _on_disconnect
            client.reconnect_delay_set(min_delay=1, max_delay=30)
            client.connect_async(self.host, self.port, keepalive=30)
            client.loop_start()
            self._client = client
        except Exception as exc:  # noqa: BLE001 - never let MQTT break startup
            logger.warning("MQTT publisher unavailable: %s", exc)
            self._client = None

    def stop(self) -> None:
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._client = None
        self.connected = False

    def publish(self, subtopic: str, payload: Any, *, retain: bool = False, qos: int = 0) -> None:
        """Publish ``payload`` (JSON-encoded unless already a str/bytes) to
        ``<base_topic>/<subtopic>``. No-op if the broker isn't reachable."""
        if self._client is None:
            return
        topic = f"{self.base_topic}/{subtopic.lstrip('/')}"
        if not isinstance(payload, (str, bytes)):
            payload = json.dumps(payload, default=str)
        try:
            self._client.publish(topic, payload, qos=qos, retain=retain)
        except Exception as exc:  # noqa: BLE001
            logger.debug("MQTT publish to %s failed: %s", topic, exc)
