"""UART command layer for the STM32 motion controller.

Wire protocol (newline-terminated ASCII frames, each with a ``*HH`` checksum
of the payload bytes, modulo 256):

    Pi -> STM32
      ``A,1``                 arm / enable motor drivers
      ``A,0``                 disarm / disable motor drivers
      ``V,<vx>,<vy>,<wz>``    body velocity setpoint (m/s, m/s, rad/s)
      ``H``                   heartbeat / watchdog ping

    STM32 -> Pi (parsed by :meth:`poll`)
      ``O,<x>,<y>,<yaw>``     wheel-odometry pose (m, m, rad)
      ``B,<volts>``           battery voltage
      ``S,<code>``            status code (``ok`` / ``armed`` / ``fault`` ...)
      ``E,<message>``         error string

The controller tolerates the STM32 being absent or disconnecting mid-drive:
writes that fail mark the link down and reconnection is retried on a backoff
so a hot control loop never blocks on a missing device.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import serial

from config import RobotConfig
from robot.motion import VelocityCommand
from utils.logger import get_logger

logger = get_logger("stm32")

_RECONNECT_BACKOFF_S = 3.0


@dataclass(slots=True)
class STM32Status:
    """Connection and telemetry state for the STM32 controller."""

    connected: bool = False
    armed: bool = False
    last_heartbeat: Optional[float] = None
    last_command: Optional[float] = None
    last_error: Optional[str] = None
    battery_voltage: Optional[float] = None
    odometry: Optional[tuple[float, float, float]] = None
    board_status: Optional[str] = None


class STM32Controller:
    """UART controller for an STM32-based motion board."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self.status = STM32Status()
        self._receive_buffer = ""
        self.emergency_latched = False
        self._next_reconnect_at = 0.0
        self._last_command = VelocityCommand(0.0, 0.0, 0.0)

    # -- connection --------------------------------------------------------

    def connect(self) -> None:
        """Open the UART link, backing off between repeated failures."""
        if self.serial and self.serial.is_open:
            self.status.connected = True
            return
        if time.monotonic() < self._next_reconnect_at:
            return
        try:
            self.serial = serial.Serial(
                self.config.uart_port,
                self.config.uart_baudrate,
                timeout=self.config.uart_timeout,
            )
            self.status.connected = bool(self.serial.is_open)
            self.status.last_error = None
            self._receive_buffer = ""
            logger.info("STM32 connected on %s", self.config.uart_port)
        except (serial.SerialException, FileNotFoundError, OSError) as exc:
            self.serial = None
            self.status.connected = False
            self.status.armed = False
            self.status.last_error = str(exc)
            self._next_reconnect_at = time.monotonic() + _RECONNECT_BACKOFF_S
            logger.warning("STM32 unavailable: %s", exc)

    def disconnect(self) -> None:
        """Close the UART link if open."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except (serial.SerialException, OSError):
                logger.warning("STM32 close failed")
        self.serial = None
        self.status.connected = False
        self.status.armed = False
        logger.info("STM32 disconnected")

    def initialize(self) -> None:
        """Attempt to connect to the STM32 on startup."""
        self.connect()

    def _mark_link_down(self, exc: Exception) -> None:
        self.status.connected = False
        self.status.armed = False
        self.status.last_error = str(exc)
        self.serial = None
        self._next_reconnect_at = time.monotonic() + _RECONNECT_BACKOFF_S
        logger.warning("STM32 link lost: %s", exc)

    # -- outgoing frames --------------------------------------------------

    @staticmethod
    def _frame(payload: str) -> bytes:
        checksum = sum(payload.encode("utf-8")) % 256
        return f"{payload}*{checksum:02X}\n".encode("utf-8")

    def _write(self, payload: str) -> bool:
        """Send one frame; return whether it actually went out."""
        if not self.serial or not self.serial.is_open:
            self.connect()
        if not self.serial or not self.serial.is_open:
            return False
        try:
            self.serial.write(self._frame(payload))
            self.serial.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            self._mark_link_down(exc)
            return False

    def _build_payload(self, command: VelocityCommand, command_id: str = "V") -> str:
        """Kept for compatibility: full framed velocity string."""
        payload = f"{command_id},{command.vx:.3f},{command.vy:.3f},{command.wz:.3f}"
        return self._frame(payload).decode("utf-8")

    def arm(self) -> bool:
        """Enable the motor drivers before autonomous driving."""
        if self._write("A,1"):
            self.status.armed = True
            logger.info("STM32 armed")
            return True
        logger.info("STM32 offline; cannot arm")
        return False

    def disarm(self) -> bool:
        """Stop and disable the motor drivers."""
        self._write("V,0.000,0.000,0.000")
        sent = self._write("A,0")
        self.status.armed = False
        if sent:
            logger.info("STM32 disarmed")
        return sent

    def send_velocity(self, command: VelocityCommand) -> bool:
        """Send a body-velocity setpoint to the STM32."""
        if self.emergency_latched and any((command.vx, command.vy, command.wz)):
            logger.warning("Emergency stop latched; blocking velocity command: %s", command)
            return False
        payload = f"V,{command.vx:.3f},{command.vy:.3f},{command.wz:.3f}"
        if self._write(payload):
            self._last_command = command
            self.status.last_command = time.monotonic()
            logger.debug("Sent STM32 command: %s", payload)
            return True
        logger.info("STM32 offline; skipping command: %s", command)
        return False

    def send_heartbeat(self) -> bool:
        """Ping the STM32 watchdog."""
        if self._write("H"):
            self.status.last_heartbeat = time.monotonic()
            return True
        return False

    def emergency_stop(self) -> None:
        """Command the robot to stop immediately and latch the stop."""
        self.emergency_latched = True
        self._write("V,0.000,0.000,0.000")
        self._write("A,0")
        self.status.armed = False
        logger.warning("Emergency stop triggered")

    def clear_emergency_stop(self) -> None:
        """Release the emergency latch after an operator safety check."""
        self.emergency_latched = False
        logger.info("Emergency stop latch cleared")

    # -- incoming frames ------------------------------------------------

    def _parse_message(self, message: str) -> dict[str, str]:
        """Parse a simple newline-delimited STM32 message."""
        parts = message.strip().split(",")
        if not parts:
            return {}
        return {"kind": parts[0], "payload": ",".join(parts[1:])}

    def read_messages(self) -> list[dict[str, str]]:
        """Read pending serial bytes and return parsed frames.

        Also retries the connection (subject to the reconnect backoff) so a
        transient USB drop recovers even while the robot is idle and nothing
        is writing to the link — poll() is called every perception cycle
        regardless of arm state, so this is the only regular opportunity to
        notice the port came back.
        """
        if not self.serial or not self.serial.is_open:
            self.connect()
        if self.serial and self.serial.is_open:
            try:
                data = self.serial.read(self.serial.in_waiting or 64).decode("utf-8", errors="ignore")
            except (serial.SerialException, OSError) as exc:
                self._mark_link_down(exc)
                data = ""
            if data:
                self._receive_buffer += data

        messages: list[dict[str, str]] = []
        while "\n" in self._receive_buffer:
            line, self._receive_buffer = self._receive_buffer.split("\n", 1)
            line = line.split("*", 1)[0].strip()
            if line:
                messages.append(self._parse_message(line))
        return messages

    def poll(self) -> list[dict[str, str]]:
        """Read STM32 telemetry frames and fold them into ``self.status``."""
        messages = self.read_messages()
        for message in messages:
            kind = message.get("kind", "")
            payload = message.get("payload", "")
            try:
                if kind == "O":
                    x, y, yaw = (float(v) for v in payload.split(","))
                    self.status.odometry = (x, y, yaw)
                elif kind == "B":
                    self.status.battery_voltage = float(payload)
                elif kind == "S":
                    self.status.board_status = payload or "ok"
                    self.status.last_heartbeat = time.monotonic()
                elif kind == "E":
                    self.status.last_error = payload
                    logger.warning("STM32 reported error: %s", payload)
            except ValueError:
                logger.debug("Unparseable STM32 frame: %s", message)
        return messages
