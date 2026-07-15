"""Mathematical helpers for motion control."""

from __future__ import annotations


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a numeric value to a range."""
    return max(minimum, min(value, maximum))
