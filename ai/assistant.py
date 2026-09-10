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
        manual: Any | None = None,
    ) -> None:
        self.llm = llm
        self.planner = planner
        self.camera = camera
        self.lidar = lidar
        self.stm32 = stm32
        self.manual = manual
        self.speech_recognizer = speech_recognizer or SpeechRecognizer(llm=llm)
        self.tts = tts or TextToSpeech()

    async def handle_user_command(self, text: str) -> dict[str, Any]:
        """Handle a user command by inferring intent and executing a plan."""
        intent = await self.llm.infer_intent(text)
        self.planner.execute(intent)
        self.tts.speak(f"Executing {intent.get('intent', 'command')}")
        return intent

    async def drive_from_text(self, text: str) -> dict[str, Any]:
        """Interpret a natural-language command into a timed manual motion."""
        if not (text or "").strip():
            return {"action": "none", "vx": 0.0, "vy": 0.0, "wz": 0.0, "duration_s": 0.0,
                    "speech": "명령을 인식하지 못했습니다", "transcript": text}
        motion = await self.llm.interpret_motion(text)
        if self.manual is not None:
            if motion.get("action") == "stop":
                self.manual.stop()
            elif motion.get("action") not in (None, "none"):
                self.manual.apply(
                    motion.get("vx", 0.0), motion.get("vy", 0.0), motion.get("wz", 0.0),
                    motion.get("duration_s", 1.5), meta=motion,
                )
        self.tts.speak(motion.get("speech", ""))
        return motion

    async def drive_from_voice(self, audio_data: Any, mime_type: str = "audio/wav") -> dict[str, Any]:
        """Transcribe spoken audio and drive the robot from it."""
        text = await self.speech_recognizer.transcribe(audio_data, mime_type=mime_type)
        result = await self.drive_from_text(text)
        result.setdefault("transcript", text)
        return result

    async def handle_voice_command(
        self,
        audio_data: Any,
        mime_type: str = "audio/wav",
    ) -> dict[str, Any]:
        """Transcribe an audio payload and drive the robot (manual teleop)."""
        return await self.drive_from_voice(audio_data, mime_type=mime_type)
