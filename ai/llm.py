"""OpenAI-compatible LLM integration for intent extraction."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from config import RobotConfig
from utils.logger import get_logger

logger = get_logger("ai.llm")


@dataclass(slots=True)
class LLMMessage:
    """Represents a chat message passed to the LLM."""

    role: str
    content: str


@dataclass(slots=True)
class ConversationMemory:
    """Conversation memory used to preserve context."""

    messages: list[LLMMessage] = field(default_factory=list)

    def add(self, message: LLMMessage) -> None:
        """Append a message to memory."""
        self.messages.append(message)


class LLMService:
    """Thin wrapper around an OpenAI-compatible client."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.memory = ConversationMemory()
        self.client: Any | None = None

    def connect(self) -> None:
        """Initialize the LLM client when credentials exist."""
        if not self.config.openai_api_key:
            logger.warning("OpenAI API key not configured")
            self.client = None
            return
        try:
            from openai import OpenAI

            self.client = OpenAI(api_key=self.config.openai_api_key)
            logger.info("LLM client initialized")
        except Exception as exc:  # pragma: no cover - defensive path
            logger.exception("Failed to initialize OpenAI client: %s", exc)

    def build_system_prompt(self) -> str:
        """Create the system prompt that guides the LLM."""
        return (
            "You are the reasoning layer for an autonomous robot. "
            "You must never directly control motors. "
            "Instead, you should infer user intent and return a structured plan."
        )

    def is_ready(self) -> bool:
        """Return whether the LLM service is usable."""
        if self.client is None:
            self.connect()
        return self.client is not None

    async def infer_intent(self, user_text: str) -> dict[str, Any]:
        """Infer the intended action from user text."""
        if not self.client:
            self.connect()
        if not self.client:
            text = (user_text or "").strip().lower()
            if any(keyword in text for keyword in ("go", "forward", "move", "navigate", "drive")):
                return {"intent": "navigate", "target": "front", "message": user_text}
            if any(keyword in text for keyword in ("stop", "halt", "wait")):
                return {"intent": "stop", "target": "self", "message": user_text}
            return {"intent": "idle", "reason": "LLM unavailable"}

        self.memory.add(LLMMessage(role="user", content=user_text))
        logger.info("LLM inference requested")
        return {"intent": "navigate", "target": "front_door", "message": user_text}
