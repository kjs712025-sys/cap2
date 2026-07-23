"""UART communication layer for the STM32 motor controller."""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Optional

import serial

from config import RobotConfig
from robot.motion import VelocityCommand
from utils.logger import get_logger

logger = get_logger("stm32")


@dataclass(slots=True)
class STM32Status:
    """Connection status for the STM32 controller."""

    connected: bool = False
    last_heartbeat: Optional[float] = None
    last_error: Optional[str] = None


class STM32Controller:
    """Simple UART controller for an STM32-based motion board."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.status = STM32Status()
        self._receive_buffer = ""
        self.emergency_latched = False

    def connect(self) -> None:
        """Open the UART connection if possible."""
        if self.serial and self.serial.is_open:
            self.status.connected = True
            return
        try:
            self.serial = serial.Serial(
                self.config.uart_port,
                self.config.uart_baudrate,
                timeout=self.config.uart_timeout,
            )
            self.status.connected = bool(self.serial.is_open)
            self.status.last_error = None
            logger.info("STM32 connected on %s", self.config.uart_port)
        except (serial.SerialException, FileNotFoundError, OSError) as exc:
            self.status.connected = False
            self.status.last_error = str(exc)
            logger.warning("STM32 unavailable: %s", exc)

    def disconnect(self) -> None:
        """Close the UART connection if open."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except (serial.SerialException, OSError):
                logger.warning("STM32 close failed")
        self.status.connected = False
        logger.info("STM32 disconnected")

    def initialize(self) -> None:
        """Attempt to connect to the STM32 on startup and keep state updated."""
        self.connect()

    def _parse_message(self, message: str) -> dict[str, str]:
        """Parse a simple newline-delimited STM32 message."""
        parts = message.strip().split(",")
        if not parts:
            return {}
        return {"kind": parts[0], "payload": ",".join(parts[1:])}

    def read_messages(self) -> list[dict[str, str]]:
        """Read pending serial messages and return parsed frames."""
        if self.serial and self.serial.is_open:
            try:
                data = self.serial.read(self.serial.in_waiting or 64).decode("utf-8", errors="ignore")
            except (serial.SerialException, OSError):
                data = ""
            if data:
                self._receive_buffer += data

        messages: list[dict[str, str]] = []
        while "\n" in self._receive_buffer:
            line, self._receive_buffer = self._receive_buffer.split("\n", 1)
            if line:
                messages.append(self._parse_message(line))
        return messages

    def _build_payload(self, command: VelocityCommand, command_id: str = "V") -> str:
        """Build a wire-format command using a checksum for robustness."""
        payload = f"{command_id},{command.vx:.3f},{command.vy:.3f},{command.wz:.3f}"
        checksum = sum(ord(ch) for ch in payload) % 256
        return f"{payload}*{checksum:02X}\n"

    def send_velocity(self, command: VelocityCommand) -> None:
        """Send a velocity command to the STM32."""
        if self.emergency_latched and any((command.vx, command.vy, command.wz)):
            logger.warning("Emergency stop latched; blocking velocity command: %s", command)
            return
        if not self.serial or not self.serial.is_open:
            self.connect()
        if not self.serial or not self.serial.is_open:
            logger.info("STM32 offline; skipping command: %s", command)
            return
        payload = self._build_payload(command, "V")
        self.serial.write(payload.encode("utf-8"))
        self.serial.flush()
        logger.debug("Sent STM32 command: %s", payload.strip())

    def emergency_stop(self) -> None:
        """Command the robot to stop immediately."""
        self.emergency_latched = True
        self.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
        logger.warning("Emergency stop triggered")

    def clear_emergency_stop(self) -> None:
        """Release the emergency latch after an operator safety check."""
        self.emergency_latched = False
        logger.info("Emergency stop latch cleared")

    async def heartbeat_loop(self) -> None:
        """Send periodic heartbeat updates while connected."""
        while self.status.connected:
            await asyncio.sleep(1.0)
            if not self.serial or not self.serial.is_open:
                self.connect()
                if not self.serial or not self.serial.is_open:
                    break
            self.serial.write(b"H\n")
            self.serial.flush()
            self.status.last_heartbeat = asyncio.get_running_loop().time()
            self.read_messages()
