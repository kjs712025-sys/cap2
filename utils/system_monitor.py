"""Read lightweight Linux host-health metrics without optional dependencies."""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Any

PROC_ROOT = Path("/proc")
THERMAL_ROOT = Path("/sys/class/thermal")


class SystemMonitor:
    """Collect CPU load, thermal, memory, and disk metrics when available."""

    def read(self) -> dict[str, Any]:
        return {
            "cpu_load_1m": self._cpu_load(),
            "cpu_temperature_c": self._cpu_temperature(),
            "memory_available_mb": self._memory_available(),
            "disk_free_gb": self._disk_free(),
        }

    @staticmethod
    def _cpu_load() -> float | None:
        try:
            load = float((PROC_ROOT / "loadavg").read_text(encoding="ascii").split()[0])
            cpu_count = os.cpu_count() or 1
            return round(load / cpu_count * 100.0, 1)
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _cpu_temperature() -> float | None:
        readings: list[float] = []
        for zone in THERMAL_ROOT.glob("thermal_zone*"):
            try:
                zone_type = (zone / "type").read_text(encoding="ascii").strip().lower()
                if not any(term in zone_type for term in ("cpu", "soc", "package")):
                    continue
                raw = int((zone / "temp").read_text(encoding="ascii").strip())
            except (OSError, ValueError):
                continue
            temperature = raw / 1000.0 if raw > 1000 else float(raw)
            if -20.0 <= temperature <= 150.0:
                readings.append(temperature)
        return round(max(readings), 1) if readings else None

    @staticmethod
    def _memory_available() -> int | None:
        try:
            for line in (PROC_ROOT / "meminfo").read_text(encoding="ascii").splitlines():
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) // 1024
        except (OSError, ValueError, IndexError):
            return None
        return None

    @staticmethod
    def _disk_free() -> float | None:
        try:
            return round(shutil.disk_usage("/").free / (1024**3), 1)
        except OSError:
            return None
