"""WebSocket support for real-time robot communication."""

from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import WebSocket

from utils.logger import get_logger

logger = get_logger("network.websocket")


class RobotSocketManager:
    """Manage WebSocket clients connected to the robot backend."""

    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        """Add a new client to the manager."""
        await websocket.accept()
        self.clients.add(websocket)
        logger.info("WebSocket client connected")

    def disconnect(self, websocket: WebSocket) -> None:
        """Remove a client from the manager."""
        self.clients.discard(websocket)
        logger.info("WebSocket client disconnected")

    async def broadcast(self, message: str) -> None:
        """Broadcast a message to all connected clients."""
        for websocket in list(self.clients):
            try:
                await websocket.send_text(message)
            except Exception as exc:  # pragma: no cover - defensive path
                logger.exception("Failed to broadcast to client: %s", exc)
                self.disconnect(websocket)

    async def broadcast_status(self, status_payload: dict[str, Any]) -> None:
        """Broadcast a structured telemetry payload to all clients."""
        message = json.dumps(status_payload)
        await self.broadcast(message)

    async def stream_status(self, status_provider: Any, interval_seconds: float = 1.0) -> None:
        """Continuously broadcast status updates while clients are connected."""
        while self.clients:
            await self.broadcast_status(status_provider())
            await asyncio.sleep(interval_seconds)
