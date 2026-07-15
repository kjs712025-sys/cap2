"""Alert generation helpers for robot monitoring."""

from __future__ import annotations

from typing import Any


class AlertManager:
    """Create simple alerts from robot status snapshots."""

    def __init__(self) -> None:
        self._alerts: list[dict[str, Any]] = []

    def evaluate(self, status_payload: dict[str, Any]) -> list[dict[str, Any]]:
        """Return alerts triggered by the current status snapshot."""
        alerts: list[dict[str, Any]] = []

        battery = float(status_payload.get("battery", 100.0))
        temperature = float(status_payload.get("temperature", 0.0))
        proximity = float(status_payload.get("proximity", 2.0))

        if battery < 20.0:
            alerts.append({"level": "warning", "code": "low_battery", "message": "Battery level is low"})
        if temperature > 70.0:
            alerts.append({"level": "warning", "code": "overheat", "message": "Temperature is too high"})
        if proximity < 0.5:
            alerts.append({"level": "critical", "code": "obstacle_close", "message": "Obstacle detected too close"})

        self._alerts = alerts
        return alerts

    def recent(self) -> list[dict[str, Any]]:
        """Return the latest generated alerts."""
        return list(self._alerts)
