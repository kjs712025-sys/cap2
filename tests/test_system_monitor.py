from __future__ import annotations

from typing import NamedTuple

from utils import system_monitor
from utils.system_monitor import SystemMonitor


def test_system_monitor_reads_linux_health_metrics(tmp_path, monkeypatch) -> None:
    proc = tmp_path / "proc"
    proc.mkdir()
    (proc / "loadavg").write_text("2.00 1.50 1.00 1/100 10", encoding="ascii")
    (proc / "meminfo").write_text("MemAvailable: 2048000 kB\n", encoding="ascii")

    zone = tmp_path / "thermal" / "thermal_zone0"
    zone.mkdir(parents=True)
    (zone / "type").write_text("cpu-thermal\n", encoding="ascii")
    (zone / "temp").write_text("48500\n", encoding="ascii")

    class DiskUsage(NamedTuple):
        total: int
        used: int
        free: int

    monkeypatch.setattr(system_monitor, "PROC_ROOT", proc)
    monkeypatch.setattr(system_monitor, "THERMAL_ROOT", zone.parent)
    monkeypatch.setattr(system_monitor.os, "cpu_count", lambda: 4)
    monkeypatch.setattr(system_monitor.shutil, "disk_usage", lambda _: DiskUsage(20, 10, 10 * 1024**3))

    assert SystemMonitor().read() == {
        "cpu_load_1m": 50.0,
        "cpu_temperature_c": 48.5,
        "memory_available_mb": 2000,
        "disk_free_gb": 10.0,
    }


def test_system_monitor_reports_missing_linux_metrics_as_unavailable(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(system_monitor, "PROC_ROOT", tmp_path / "missing-proc")
    monkeypatch.setattr(system_monitor, "THERMAL_ROOT", tmp_path / "missing-thermal")

    metrics = SystemMonitor().read()

    assert metrics["cpu_load_1m"] is None
    assert metrics["cpu_temperature_c"] is None
    assert metrics["memory_available_mb"] is None
