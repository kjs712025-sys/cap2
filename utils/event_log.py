"""Simple event logging utilities for robot state history."""

from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path
from typing import Any


class EventLogger:
    """Store recent robot events in memory and optionally persist them to disk."""

    def __init__(self, max_events: int = 100, file_path: str | None = None) -> None:
        self.max_events = max_events
        self.file_path = Path(file_path) if file_path else None
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)

    def log(self, event_type: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        """Record a new event and return the stored entry."""
        entry = {
            "timestamp": time.time(),
            "event_type": event_type,
            "payload": payload or {},
        }
        self._events.append(entry)
        if self.file_path is not None:
            self.file_path.parent.mkdir(parents=True, exist_ok=True)
            with self.file_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(entry) + "\n")
        return entry

    def recent(self, limit: int = 10) -> list[dict[str, Any]]:
        """Return the most recent events."""
        return list(self._events)[-limit:]
