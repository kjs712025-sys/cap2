"""Speech recognition interface placeholder for Whisper integration."""

from __future__ import annotations

from typing import Any


class SpeechRecognizer:
    """Placeholder wrapper for speech-to-text processing."""

    def __init__(self, model_name: str = "tiny") -> None:
        self.model_name = model_name

    async def transcribe(self, audio_data: Any) -> str:
        """Return a simple fallback transcription when no model is attached."""
        if audio_data is None:
            return ""
        if isinstance(audio_data, str):
            return audio_data.strip()
        if isinstance(audio_data, bytes):
            return "voice command"
        return "voice command"
