"""High-level assistant services coordinating perception and planning."""

from __future__ import annotations

from typing import Any

from ai.llm import LLMService
from ai.planner import Planner
from camera.camera import Camera
from lidar.lidar import LidarSensor
from robot.stm32 import STM32Controller
from utils.logger import get_logger
from voice.speech import SpeechRecognizer
from voice.tts import TextToSpeech

logger = get_logger("ai.assistant")


class RobotAssistant:
    """Coordinates AI behaviors without directly coupling modules."""

    def __init__(
        self,
        llm: LLMService,
        planner: Planner,
        camera: Camera,
        lidar: LidarSensor,
        stm32: STM32Controller,
        speech_recognizer: SpeechRecognizer | None = None,
        tts: TextToSpeech | None = None,
    ) -> None:
        self.llm = llm
        self.planner = planner
        self.camera = camera
        self.lidar = lidar
        self.stm32 = stm32
        self.speech_recognizer = speech_recognizer or SpeechRecognizer(llm=llm)
        self.tts = tts or TextToSpeech()

    async def handle_user_command(self, text: str) -> dict[str, Any]:
        """Handle a user command by inferring intent and executing a plan."""
        intent = await self.llm.infer_intent(text)
        self.planner.execute(intent)
        self.tts.speak(f"Executing {intent.get('intent', 'command')}")
        return intent

    async def handle_voice_command(
        self,
        audio_data: Any,
        mime_type: str = "audio/wav",
    ) -> dict[str, Any]:
        """Transcribe an audio payload and route it through the same command handler."""
        text = await self.speech_recognizer.transcribe(audio_data, mime_type=mime_type)
        return await self.handle_user_command(text)
