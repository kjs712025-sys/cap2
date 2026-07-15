"""Simple API response contract helpers for client-friendly payloads."""

from __future__ import annotations

from typing import Any


def success(payload: dict[str, Any] | None = None, *, message: str = "ok") -> dict[str, Any]:
    """Wrap a successful payload in a standard shape."""
    return {"success": True, "message": message, "data": payload or {}}


def error(message: str, *, code: str = "error") -> dict[str, Any]:
    """Wrap an error payload in a standard shape."""
    return {"success": False, "message": message, "error": code}
