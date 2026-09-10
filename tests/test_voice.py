from __future__ import annotations

import asyncio

from unittest.mock import MagicMock

from ai.assistant import RobotAssistant
from ai.llm import LLMService
from ai.planner import Planner
from camera.camera import Camera
from config import RobotConfig
from lidar.lidar import LidarSensor
from robot.manual import ManualController
from robot.status import RobotStatus
from robot.stm32 import STM32Controller
from voice.speech import SpeechRecognizer
from voice.tts import TextToSpeech


def _assistant() -> tuple[RobotAssistant, ManualController]:
    config = RobotConfig(debug=True, llm_enabled=False)  # no key -> keyword fallback
    llm = LLMService(config)
    stm32 = MagicMock()
    manual = ManualController(stm32=stm32, status=RobotStatus(), config=config)
    assistant = RobotAssistant(
        llm=llm,
        planner=Planner(stm32),
        camera=Camera(config),
        lidar=LidarSensor(config),
        stm32=stm32,
        speech_recognizer=SpeechRecognizer(),
        tts=TextToSpeech(),
        manual=manual,
    )
    return assistant, manual


def test_voice_forward_drives_the_robot() -> None:
    assistant, manual = _assistant()
    result = asyncio.run(assistant.handle_voice_command("go forward"))
    assert result["action"] == "move"
    assert result["vx"] > 0 and result["vy"] == 0 and result["wz"] == 0
    manual.stm32.send_velocity.assert_called()


def test_voice_turn_left_sets_yaw() -> None:
    assistant, _ = _assistant()
    result = asyncio.run(assistant.drive_from_text("왼쪽으로 돌아"))
    assert result["action"] == "turn"
    assert result["wz"] > 0


def test_voice_stop_halts_manual() -> None:
    assistant, manual = _assistant()
    asyncio.run(assistant.drive_from_text("앞으로 가"))
    assert manual.active
    asyncio.run(assistant.drive_from_text("멈춰"))
    assert not manual.active


def test_manual_endpoint_drives_from_text() -> None:
    from fastapi.testclient import TestClient
    from network.server import create_app

    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    r = client.post("/command/manual", json={"text": "앞으로 가"}).json()
    assert r["success"] and r["data"]["motion"]["action"] == "move"
    assert client.post("/command/stop").json()["data"]["action"] == "stop"


def test_voice_endpoint_available_in_manual_refused_in_autonomous() -> None:
    from fastapi.testclient import TestClient
    from network.server import create_app

    app = create_app(RobotConfig(debug=True, llm_enabled=False, enable_websocket=False))
    client = TestClient(app)

    # no key -> transcription returns "" -> assistant reports "none", but the
    # request itself is accepted from the dashboard in manual mode
    ok = client.post(
        "/command/voice", content=b"RIFFxxxxWAVE", headers={"Content-Type": "audio/wav"}
    ).json()
    assert ok["success"] is True

    client.post("/mode", json={"mode": "auto"})
    refused = client.post(
        "/command/voice", content=b"RIFFxxxxWAVE", headers={"Content-Type": "audio/wav"}
    ).json()
    assert refused["success"] is False and refused["error"] == "autonomous_active"

    empty = client.post("/mode", json={"mode": "manual"}) and client.post(
        "/command/voice", content=b"", headers={"Content-Type": "audio/wav"}
    ).json()
    assert empty["error"] == "no_audio"
