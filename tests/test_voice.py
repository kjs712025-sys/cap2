from __future__ import annotations

import asyncio

from ai.assistant import RobotAssistant
from ai.llm import LLMService
from ai.planner import Planner
from camera.camera import Camera
from config import RobotConfig
from lidar.lidar import LidarSensor
from robot.stm32 import STM32Controller
from voice.speech import SpeechRecognizer
from voice.tts import TextToSpeech


def test_voice_command_pipeline() -> None:
    config = RobotConfig(debug=True, llm_enabled=False)
    llm = LLMService(config)
    stm32 = STM32Controller(config)
    assistant = RobotAssistant(
        llm=llm,
        planner=Planner(stm32),
        camera=Camera(config),
        lidar=LidarSensor(config),
        stm32=stm32,
        speech_recognizer=SpeechRecognizer(),
        tts=TextToSpeech(),
    )

    result = asyncio.run(assistant.handle_voice_command("go forward"))
    assert result["intent"] == "navigate"
