"""Wake-word detection placeholder."""

from __future__ import annotations


class WakeWordDetector:
    """Placeholder for wake-word activation."""

    def __init__(self) -> None:
        self.enabled = False

    def start(self) -> None:
        """Enable wake-word listening."""
        self.enabled = True

    def stop(self) -> None:
        """Disable wake-word listening."""
        self.enabled = False
