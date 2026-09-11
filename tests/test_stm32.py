from __future__ import annotations

from unittest.mock import MagicMock, patch

from config import RobotConfig
from robot.motion import VelocityCommand
from robot.stm32 import STM32Controller


def _connected_controller() -> tuple[STM32Controller, MagicMock]:
    controller = STM32Controller(RobotConfig(debug=True))
    port = MagicMock()
    port.is_open = True
    port.in_waiting = 0
    controller.serial = port
    controller.status.connected = True
    return controller, port


def test_frame_has_payload_and_checksum() -> None:
    frame = STM32Controller._frame("V,0.100,0.000,-0.200")
    assert frame.endswith(b"\n")
    body, checksum = frame.strip().split(b"*")
    assert body == b"V,0.100,0.000,-0.200"
    assert int(checksum, 16) == sum(body) % 256


def test_arm_and_send_velocity_write_frames() -> None:
    controller, port = _connected_controller()

    assert controller.arm() is True
    assert controller.status.armed is True
    assert controller.send_velocity(VelocityCommand(0.15, 0.0, -0.3)) is True

    written = b"".join(call.args[0] for call in port.write.call_args_list)
    assert b"A,1*" in written
    assert b"V,0.150,0.000,-0.300*" in written


def test_emergency_stop_latches_and_blocks_motion() -> None:
    controller, port = _connected_controller()
    controller.emergency_stop()
    assert controller.emergency_latched is True
    assert controller.status.armed is False
    assert controller.send_velocity(VelocityCommand(0.2, 0.0, 0.0)) is False


def test_poll_folds_odometry_and_battery_into_status() -> None:
    controller, port = _connected_controller()
    port.in_waiting = 64
    port.read.return_value = b"O,1.5,-0.2,0.79\nB,11.8\nS,armed\n"

    controller.poll()

    assert controller.status.odometry == (1.5, -0.2, 0.79)
    assert controller.status.battery_voltage == 11.8
    assert controller.status.board_status == "armed"


def test_stm32_parser_handles_messages() -> None:
    controller = STM32Controller(RobotConfig(debug=True))
    controller._receive_buffer = "OK,ready\nERR,stall\n"

    messages = controller.read_messages()

    assert len(messages) == 2
    assert messages[0]["kind"] == "OK"
    assert messages[1]["payload"] == "stall"


def test_poll_retries_connection_when_link_is_down() -> None:
    """A transient USB drop must recover on its own: poll() runs every
    perception cycle whether or not the robot is armed, so it has to retry
    the connection itself rather than only reconnecting opportunistically
    when something happens to write a command."""
    controller = STM32Controller(RobotConfig(debug=True))
    assert controller.serial is None

    with patch("robot.stm32.serial.Serial") as serial_cls:
        serial_instance = MagicMock()
        serial_instance.is_open = True
        serial_instance.in_waiting = 0
        serial_instance.read.return_value = b""
        serial_cls.return_value = serial_instance

        controller.poll()

    assert controller.status.connected is True
    assert controller.serial is serial_instance


def test_stm32_connect_uses_serial_port_when_available() -> None:
    controller = STM32Controller(RobotConfig(debug=True))

    with patch("robot.stm32.serial.Serial") as serial_cls:
        serial_instance = MagicMock()
        serial_instance.is_open = True
        serial_cls.return_value = serial_instance

        controller.connect()

    assert controller.status.connected is True
