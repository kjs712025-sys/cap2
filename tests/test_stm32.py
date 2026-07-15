from __future__ import annotations

from unittest.mock import MagicMock, patch

from config import RobotConfig
from robot.stm32 import STM32Controller


def test_stm32_parser_handles_messages() -> None:
    controller = STM32Controller(RobotConfig(debug=True))
    controller._receive_buffer = "OK,ready\nERR,stall\n"

    messages = controller.read_messages()

    assert len(messages) == 2
    assert messages[0]["kind"] == "OK"
    assert messages[1]["payload"] == "stall"


def test_stm32_connect_uses_serial_port_when_available() -> None:
    controller = STM32Controller(RobotConfig(debug=True))

    with patch("robot.stm32.serial.Serial") as serial_cls:
        serial_instance = MagicMock()
        serial_instance.is_open = True
        serial_cls.return_value = serial_instance

        controller.connect()

    assert controller.status.connected is True
