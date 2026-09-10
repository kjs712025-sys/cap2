"""OpenAI-compatible LLM integration for intent extraction."""

from __future__ import annotations

import asyncio
import base64
import json
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit
from urllib.request import Request, urlopen
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
        """Return whether the LLM service is usable (Gemini key or OpenAI client)."""
        if self.config.gemini_api_key:
            return True
        if self.client is None:
            self.connect()
        return self.client is not None

    async def interpret_motion(self, user_text: str) -> dict[str, Any]:
        """Turn a spoken/typed command into a timed velocity for manual driving.

        Returns ``{action, vx, vy, wz, duration_s, speech}``. Uses Gemini when a
        key is configured, else a Korean/English keyword fallback.
        """
        max_lin = self.config.max_linear_speed
        max_ang = self.config.max_angular_speed
        cruise = min(0.15, max_lin)
        turn = min(0.5, max_ang)

        if self.config.gemini_api_key:
            try:
                prompt = (
                    "You drive a 4-wheel omni-directional robot. Convert the user's command "
                    "into ONE motion. Frame: vx forward(+)/back(-) m/s, vy left(+)/right(-) m/s, "
                    f"wz turn-left/ccw(+)/right/cw(-) rad/s. Limits |vx|,|vy|<={max_lin}, |wz|<={max_ang}. "
                    f"Use gentle magnitudes (~{cruise} m/s, ~{turn} rad/s) unless 'fast'/'slow' is said. "
                    'Return ONLY JSON: {"action":"move"|"turn"|"strafe"|"stop"|"none",'
                    '"vx":0.0,"vy":0.0,"wz":0.0,"duration_s":1.5,"speech":"short confirmation '
                    'in the user\'s language"}. duration_s in [0.5,4]. '
                    f"Command: {user_text}"
                )
                raw = await self._generate_with_gemini([{"text": prompt}], response_json=True)
                result = json.loads(raw)
                return self._clamp_motion(result, user_text)
            except Exception as exc:  # pragma: no cover - network-dependent
                logger.warning("Gemini motion interpretation failed: %s", exc)

        return self._keyword_motion(user_text, cruise, turn)

    def _clamp_motion(self, result: dict[str, Any], user_text: str) -> dict[str, Any]:
        max_lin = self.config.max_linear_speed
        max_ang = self.config.max_angular_speed

        def clamp(v: Any, lo: float, hi: float, default: float = 0.0) -> float:
            try:
                return max(lo, min(hi, float(v)))
            except (TypeError, ValueError):
                return default

        return {
            "action": str(result.get("action", "none")),
            "vx": clamp(result.get("vx"), -max_lin, max_lin),
            "vy": clamp(result.get("vy"), -max_lin, max_lin),
            "wz": clamp(result.get("wz"), -max_ang, max_ang),
            "duration_s": clamp(result.get("duration_s"), 0.5, 4.0, 1.5) or 1.5,
            "speech": str(result.get("speech") or "명령을 실행합니다"),
            "transcript": user_text,
        }

    def _keyword_motion(self, text: str, cruise: float, turn: float) -> dict[str, Any]:
        t = (text or "").lower().strip()
        m = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "duration_s": 1.5}
        if any(k in t for k in ("stop", "멈춰", "정지", "스톱", "그만", "halt")):
            return {"action": "stop", "speech": "정지합니다", "transcript": text, **{k: 0.0 for k in ("vx", "vy", "wz")}, "duration_s": 0.5}
        if any(k in t for k in ("돌아", "회전", "turn", "rotate", "spin")):
            left = any(k in t for k in ("왼", "left", "좌", "ccw"))
            m["wz"] = turn if left else -turn
            action, speech = "turn", ("왼쪽으로 회전합니다" if left else "오른쪽으로 회전합니다")
        elif any(k in t for k in ("뒤", "후진", "back", "reverse")):
            m["vx"] = -cruise
            action, speech = "move", "후진합니다"
        elif any(k in t for k in ("앞", "전진", "직진", "forward", "go", "straight", "이동", "가")):
            if any(k in t for k in ("왼", "left", "좌")):
                m["vy"], action, speech = cruise, "strafe", "왼쪽으로 이동합니다"
            elif any(k in t for k in ("오른", "right", "우")):
                m["vy"], action, speech = -cruise, "strafe", "오른쪽으로 이동합니다"
            else:
                m["vx"], action, speech = cruise, "move", "전진합니다"
        else:
            return {"action": "none", "speech": "명령을 이해하지 못했습니다", "transcript": text, "vx": 0.0, "vy": 0.0, "wz": 0.0, "duration_s": 0.0}
        return {"action": action, "speech": speech, "transcript": text, **m}

    async def infer_intent(self, user_text: str) -> dict[str, Any]:
        """Infer the intended action from user text."""
        if self.config.gemini_api_key:
            try:
                return await self._infer_with_gemini(user_text)
            except Exception as exc:  # pragma: no cover - network-dependent path
                logger.exception("Gemini inference failed: %s", exc)

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

    async def _infer_with_gemini(self, user_text: str) -> dict[str, Any]:
        """Call Gemini's generate endpoint and parse its JSON intent."""
        prompt = (
            f"{self.build_system_prompt()} Return only valid JSON with keys "
            "intent, target, and message. User command: " + user_text
        )
        response = await self._generate_with_gemini(
            [{"text": prompt}],
            response_json=True,
        )
        result = json.loads(response)
        result.setdefault("message", user_text)
        self.memory.add(LLMMessage(role="user", content=user_text))
        return result

    async def transcribe_audio(self, audio_data: bytes, mime_type: str = "audio/wav") -> str:
        """Transcribe audio with Gemini's multimodal input support."""
        if not self.config.gemini_api_key:
            return ""
        response = await self._generate_with_gemini(
            [
                {"text": "Transcribe this audio. Return only the spoken words."},
                {
                    "inline_data": {
                        "mime_type": mime_type,
                        "data": base64.b64encode(audio_data).decode("ascii"),
                    }
                },
            ]
        )
        return response.strip()

    async def analyze_image(
        self,
        image_data: bytes,
        mime_type: str = "image/jpeg",
        supported_objects: tuple[str, ...] = (),
    ) -> dict[str, Any]:
        """Analyze a camera image and return structured detections."""
        if not self.config.gemini_api_key:
            return {"summary": "Gemini is not configured", "detections": []}
        objects = ", ".join(supported_objects) or "any visible objects"
        response = await self._generate_with_gemini(
            [
                {
                    "text": (
                        "Analyze this image. Return only valid JSON with keys "
                        "fire_detected, summary and detections. Set fire_detected "
                        "to true for any visible fire, flame, or smoke. Each detection must contain "
                        "label, confidence, and box as [left, top, right, bottom]. "
                        f"Focus on these objects: {objects}."
                    )
                },
                {
                    "inline_data": {
                        "mime_type": mime_type,
                        "data": base64.b64encode(image_data).decode("ascii"),
                    }
                },
            ],
            response_json=True,
        )
        return json.loads(response)

    async def _generate_with_gemini(
        self,
        parts: list[dict[str, Any]],
        response_json: bool = False,
    ) -> str:
        """Send multimodal content to the configured Gemini endpoint."""
        url = self.config.gemini_api_url.format(model=self.config.gemini_model)
        query_params = list(parse_qsl(urlsplit(url).query, keep_blank_values=True))
        query_params.append(("key", self.config.gemini_api_key or ""))
        split_url = urlsplit(url)
        url = urlunsplit(split_url._replace(query=urlencode(query_params)))
        payload = {
            "contents": [{"role": "user", "parts": parts}],
        }
        if response_json:
            payload["generationConfig"] = {"responseMimeType": "application/json"}
        request = Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        response_body = await asyncio.to_thread(self._post_json, request)
        response = json.loads(response_body)
        return response["candidates"][0]["content"]["parts"][0]["text"]

    @staticmethod
    def _post_json(request: Request) -> str:
        """Send a JSON request without adding another HTTP dependency."""
        with urlopen(request, timeout=30) as response:
            return response.read().decode("utf-8")
