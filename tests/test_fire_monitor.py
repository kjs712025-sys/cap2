from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

from camera.vision import Detection
from robot.status import RobotMode, RobotStatus
from safety.fire_monitor import FireMonitor


class _StubFrame:
    image = object()


def _make_monitor(
    fire_detections: list[Detection],
    auto_stop: bool = False,
    navigator: MagicMock | None = None,
) -> tuple[FireMonitor, MagicMock, RobotStatus]:
    camera = MagicMock()
    camera.read_frame.return_value = _StubFrame()
    vision = MagicMock()
    vision.detect_fire.return_value = fire_detections
    stm32 = MagicMock()
    status = RobotStatus()
    monitor = FireMonitor(
        camera=camera,
        vision=vision,
        stm32=stm32,
        status=status,
        warn_confidence=0.4,
        stop_confidence=0.7,
        auto_stop=auto_stop,
        mqtt=MagicMock(),
        navigator=navigator,
    )
    return monitor, stm32, status


def _box() -> tuple[float, float, float, float]:
    return (0.0, 0.0, 10.0, 10.0)


def test_mid_confidence_detection_warns_without_stopping() -> None:
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.45, _box())])

    asyncio.run(monitor.check_once())

    stm32.emergency_stop.assert_not_called()
    assert status.mode is not RobotMode.ERROR
    assert status.metadata.get("fire_warning") == "smoke 0.45"


def test_high_confidence_detection_triggers_emergency_stop_when_auto_stop_on() -> None:
    monitor, stm32, status = _make_monitor([Detection("Fire", 0.82, _box())], auto_stop=True)

    asyncio.run(monitor.check_once())

    stm32.emergency_stop.assert_called_once()
    assert status.mode is RobotMode.ERROR
    assert status.metadata.get("fire_detected") == "true"


def test_high_confidence_detection_is_suppressed_when_auto_stop_off() -> None:
    monitor, stm32, status = _make_monitor([Detection("Fire", 0.82, _box())], auto_stop=False)

    asyncio.run(monitor.check_once())

    stm32.emergency_stop.assert_not_called()
    assert status.mode is not RobotMode.ERROR
    assert "suppressed" in status.metadata.get("fire_warning", "")


def test_low_confidence_detection_is_ignored() -> None:
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.30, _box())])

    asyncio.run(monitor.check_once())

    stm32.emergency_stop.assert_not_called()
    assert "fire_warning" not in status.metadata


def test_cleared_view_drops_stale_warning() -> None:
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.45, _box())])
    asyncio.run(monitor.check_once())
    assert "fire_warning" in status.metadata

    monitor.vision.detect_fire.return_value = []
    asyncio.run(monitor.check_once())
    assert "fire_warning" not in status.metadata


def test_detection_publishes_boxes_to_mqtt_and_overlay() -> None:
    monitor, stm32, status = _make_monitor([Detection("Fire", 0.9, (10.0, 20.0, 30.0, 40.0))], auto_stop=True)

    asyncio.run(monitor.check_once())

    boxes = monitor.current_boxes()
    assert boxes and boxes[0]["box"] == [10.0, 20.0, 30.0, 40.0]
    topic, payload = monitor.mqtt.publish.call_args.args[:2]
    assert topic == "fire"
    assert payload["level"] == "stop"
    assert payload["detections"][0]["label"] == "Fire"


def test_actionable_detection_halts_autonomous_navigation() -> None:
    navigator = MagicMock()
    navigator.enabled = True
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.5, _box())], navigator=navigator)

    # First sighting: alert flag set, but navigation not halted yet (debounce).
    asyncio.run(monitor.check_once())
    assert status.metadata.get("fire_alert") == "warn"
    navigator.disable.assert_not_called()

    # Second consecutive sighting: now the soft halt engages.
    asyncio.run(monitor.check_once())
    navigator.disable.assert_called_once()
    assert status.mission == "fire_halt"


def test_single_flicker_detection_does_not_halt_navigation() -> None:
    navigator = MagicMock()
    navigator.enabled = True
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.5, _box())], navigator=navigator)

    asyncio.run(monitor.check_once())          # one sighting
    monitor.vision.detect_fire.return_value = []
    asyncio.run(monitor.check_once())          # then clear -> streak resets

    navigator.disable.assert_not_called()


def test_cleared_view_drops_fire_alert() -> None:
    navigator = MagicMock()
    navigator.enabled = True
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.5, _box())], navigator=navigator)
    asyncio.run(monitor.check_once())
    assert "fire_alert" in status.metadata

    monitor.vision.detect_fire.return_value = []
    asyncio.run(monitor.check_once())
    assert "fire_alert" not in status.metadata


def test_boxes_expire_after_ttl() -> None:
    monitor, stm32, status = _make_monitor([Detection("smoke", 0.5, _box())])
    asyncio.run(monitor.check_once())
    assert monitor.current_boxes()

    monitor.latest["_mono"] -= 999  # pretend the sighting is old
    assert monitor.current_boxes() == []


def test_snapshot_matches_mqtt_payload_and_hides_private_keys() -> None:
    monitor, stm32, status = _make_monitor([Detection("Fire", 0.9, _box())], auto_stop=True)
    asyncio.run(monitor.check_once())

    snap = monitor.snapshot()
    _, mqtt_payload = monitor.mqtt.publish.call_args.args[:2]
    assert snap == mqtt_payload
    assert not any(k.startswith("_") for k in snap)
    assert snap["level"] == "stop"
