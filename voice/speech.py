"""Speech recognition interface placeholder for Whisper integration."""

from __future__ import annotations

from typing import Any

from ai.llm import LLMService


class SpeechRecognizer:
    """Placeholder wrapper for speech-to-text processing."""

    def __init__(self, model_name: str = "tiny", llm: LLMService | None = None) -> None:
        self.model_name = model_name
        self.llm = llm

    async def transcribe(self, audio_data: Any, mime_type: str = "audio/wav") -> str:
        """Transcribe audio via Gemini, or fall back to a stub."""
        if audio_data is None:
            return ""
        if isinstance(audio_data, str):
            return audio_data.strip()
        if isinstance(audio_data, bytes) and self.llm is not None:
            try:
                transcription = await self.llm.transcribe_audio(audio_data, mime_type=mime_type)
            except Exception:  # noqa: BLE001 - bad audio / API error -> caller decides
                return ""
            if transcription:
                return transcription
        return "" if isinstance(audio_data, bytes) else "voice command"
