"""LiDAR interface for YDLiDAR X-series (X2/X4) sensors."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial

from config import RobotConfig
from utils.logger import get_logger

logger = get_logger("lidar")

PACKET_HEADER = b"\xaa\x55"
DEFAULT_BAUDRATE = 128000
SCAN_TIMEOUT_S = 0.5
CONNECT_TIMEOUT_S = 1.0


@dataclass(slots=True)
class ScanPoint:
    """Single LiDAR range measurement."""

    angle_deg: float
    distance_m: float


@dataclass(slots=True)
class LidarStatus:
    """Current LiDAR state."""

    connected: bool = False
    last_scan: Optional[list[ScanPoint]] = None
    last_scan_hardware: bool = False
    last_error: Optional[str] = None


class LidarSensor:
    """YDLiDAR X-series sensor read over a serial (USB-UART) connection.

    The X2/X4 stream scan packets continuously as soon as they are powered —
    no start command is required. If no real sensor is configured or
    reachable, falls back to a small simulated scan so the rest of the app
    (obstacle detection, SLAM, dashboard) keeps working without hardware.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.status = LidarStatus()
        self._scan_buffer: list[ScanPoint] = []
        self.serial: Optional[serial.Serial] = None
        # Serial reads aren't reentrant; the navigator loop and the REST
        # endpoints can both call read_scan() concurrently.
        self._read_lock = threading.Lock()

    def connect(self) -> None:
        """Open the serial port and confirm a valid YDLiDAR packet is seen."""
        if self.serial and self.serial.is_open:
            self.status.connected = True
            return
        try:
            self.serial = serial.Serial(
                self.config.lidar_port,
                DEFAULT_BAUDRATE,
                timeout=1.0,
            )
        except (serial.SerialException, OSError) as exc:
            self.serial = None
            self.status.connected = False
            self.status.last_error = str(exc)
            logger.warning("LiDAR unavailable: %s", exc)
            return

        if self._resync_to_header(CONNECT_TIMEOUT_S):
            self.status.connected = True
            self.status.last_scan_hardware = False
            self.status.last_error = None
            logger.info("LiDAR connected on %s", self.config.lidar_port)
        else:
            self.status.connected = False
            self.status.last_error = "No valid YDLiDAR packet received"
            if self.serial is not None:
                try:
                    self.serial.close()
                except (serial.SerialException, OSError):
                    logger.warning("LiDAR close failed after packet timeout")
            self.serial = None
            logger.warning(
                "LiDAR port %s opened but no valid packets seen", self.config.lidar_port
            )

    def _resync_to_header(self, timeout_s: float) -> bool:
        """Advance the serial stream to the next packet header (0xAA 0x55)."""
        if not self.serial:
            return False
        window = b""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            byte = self.serial.read(1)
            if not byte:
                continue
            window = (window + byte)[-2:]
            if window == PACKET_HEADER:
                return True
        return False

    def is_ready(self) -> bool:
        """Return whether the LiDAR is available for scanning."""
        return self.status.connected

    def initialize(self) -> bool:
        """Attempt to initialize the LiDAR sensor and return whether it is ready."""
        if self.config.simulation:
            self.status.connected = True
            self.status.last_error = None
            self.status.last_scan = self._generate_virtual_scan()
            logger.info("Using simulated LiDAR; no physical device will be opened")
            return True
        self.connect()
        return self.is_ready()

    def disconnect(self) -> None:
        """Disconnect the LiDAR device."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except (serial.SerialException, OSError):
                logger.warning("LiDAR close failed")
            self.serial = None
        self.status.connected = False
        self.status.last_scan_hardware = False
        logger.info("LiDAR disconnected")

    def parse_scan_text(self, text: str) -> list[ScanPoint]:
        """Parse scan payloads such as '0:1.2,30:2.5' or '0=1.2;30=2.5'."""
        points: list[ScanPoint] = []
        if not text:
            return points

        for token in text.replace(";", ",").split(","):
            if not token:
                continue
            token = token.strip()
            if not token:
                continue
            separator = ":" if ":" in token else "="
            if separator not in token:
                continue
            angle_str, distance_str = token.split(separator, 1)
            try:
                points.append(ScanPoint(angle_deg=float(angle_str), distance_m=float(distance_str)))
            except ValueError:
                logger.warning("Skipping malformed LiDAR scan token: %s", token)
        return points

    def _generate_simulated_scan(self) -> list[ScanPoint]:
        """Generate a deterministic scan for local and test environments."""
        return [
            ScanPoint(angle_deg=-30.0, distance_m=2.0),
            ScanPoint(angle_deg=0.0, distance_m=1.2),
            ScanPoint(angle_deg=30.0, distance_m=2.5),
        ]

    @staticmethod
    def _generate_virtual_scan() -> list[ScanPoint]:
        """Return a full scan of a small virtual room with two obstacles."""
        points: list[ScanPoint] = []
        obstacles = ((1.35, 0.0, 0.35), (2.0, 1.1, 0.4))
        for angle_deg in range(0, 360, 5):
            angle = math.radians(angle_deg)
            dx, dy = math.cos(angle), math.sin(angle)
            distance = min(
                3.0 / abs(dx) if abs(dx) > 1e-6 else float("inf"),
                3.0 / abs(dy) if abs(dy) > 1e-6 else float("inf"),
            )
            for center_x, center_y, radius in obstacles:
                projection = center_x * dx + center_y * dy
                perpendicular_sq = center_x**2 + center_y**2 - projection**2
                discriminant = radius**2 - perpendicular_sq
                if projection > 0 and discriminant >= 0:
                    distance = min(distance, projection - math.sqrt(discriminant))
            points.append(ScanPoint(angle_deg=float(angle_deg), distance_m=round(distance, 3)))
        return points

    def _read_packet(self) -> Optional[tuple[float, float, list[int]]]:
        """Read and decode one YDLiDAR data packet.

        Returns (start_angle_deg, end_angle_deg, raw_samples) or None if the
        stream desynced or a read timed out partway through a packet.
        """
        if not self.serial or not self._resync_to_header(SCAN_TIMEOUT_S):
            return None

        header = self.serial.read(8)
        if len(header) < 8:
            return None
        sample_count = header[1]
        start_angle_raw = header[2] | (header[3] << 8)
        end_angle_raw = header[4] | (header[5] << 8)
        # header[6:8] is the packet checksum; not verified here.

        payload = self.serial.read(sample_count * 2)
        if len(payload) < sample_count * 2:
            return None

        samples = [
            payload[2 * i] | (payload[2 * i + 1] << 8) for i in range(sample_count)
        ]
        start_angle = (start_angle_raw >> 1) / 64.0
        end_angle = (end_angle_raw >> 1) / 64.0
        return start_angle, end_angle, samples

    def read_scan(self) -> list[ScanPoint]:
        """Return one full rotation of points from the sensor, or a simulated fallback."""
        if self.config.simulation:
            scan = self._generate_virtual_scan()
            self._scan_buffer = scan
            self.status.last_scan = scan
            self.status.last_scan_hardware = False
            return scan
        if not self.status.connected or not self.serial:
            scan = self._generate_simulated_scan()
            self._scan_buffer = scan
            self.status.last_scan = scan
            self.status.last_scan_hardware = False
            return scan

        with self._read_lock:
            return self._read_scan_locked()

    def _read_scan_locked(self) -> list[ScanPoint]:
        points: list[ScanPoint] = []
        previous_end_angle: Optional[float] = None
        completed_rotation = False
        deadline = time.monotonic() + SCAN_TIMEOUT_S

        try:
            while time.monotonic() < deadline:
                packet = self._read_packet()
                if packet is None:
                    continue
                start_angle, end_angle, samples = packet
                count = len(samples)
                if count == 0:
                    continue

                angle_span = (end_angle - start_angle) % 360.0
                angle_step = angle_span / (count - 1) if count > 1 else 0.0
                for i, raw in enumerate(samples):
                    distance_mm = raw / 4.0
                    if distance_mm < 10.0:
                        # A zero or near-zero reading means "no return" on this
                        # sensor, not an obstacle 1cm away — treating it as a
                        # real distance would make ObstacleDetector see a
                        # false obstacle at the robot's own position.
                        continue
                    angle = (start_angle + angle_step * i) % 360.0
                    points.append(
                        ScanPoint(angle_deg=round(angle, 2), distance_m=round(distance_mm / 1000.0, 3))
                    )

                # A full rotation completed once the running angle wraps back down.
                if previous_end_angle is not None and end_angle < previous_end_angle and points:
                    completed_rotation = True
                    break
                previous_end_angle = end_angle
        except (serial.SerialException, OSError) as exc:
            self.status.connected = False
            self.status.last_scan_hardware = False
            self.status.last_error = str(exc)
            if self.serial is not None:
                try:
                    self.serial.close()
                except (serial.SerialException, OSError):
                    logger.warning("LiDAR close failed after read error")
                self.serial = None
            logger.warning("LiDAR read failed: %s", exc)

        scan = points or self._generate_simulated_scan()
        self._scan_buffer = scan
        self.status.last_scan = scan
        self.status.last_scan_hardware = completed_rotation and bool(points)
        return scan
