"""Text-to-speech interface placeholder."""

from __future__ import annotations

from utils.logger import get_logger

logger = get_logger("voice.tts")


class TextToSpeech:
    """Placeholder wrapper for TTS synthesis."""

    def speak(self, text: str) -> None:
        """Emit a spoken response via logging for now."""
        if not text:
            return
        logger.info("TTS: %s", text)
