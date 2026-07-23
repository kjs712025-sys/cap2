from __future__ import annotations

import asyncio
import json
from urllib.request import Request

from ai.llm import LLMService
from config import RobotConfig


class FakeResponse:
    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, *args: object) -> None:
        return None

    def read(self) -> bytes:
        return json.dumps(
            {
                "candidates": [
                    {
                        "content": {
                            "parts": [
                                {"text": '{"intent":"stop","target":"self"}'}
                            ]
                        }
                    }
                ]
            }
        ).encode("utf-8")


def test_gemini_inference_uses_configured_endpoint(monkeypatch) -> None:
    captured: dict[str, Request] = {}

    def fake_urlopen(request: Request, timeout: int) -> FakeResponse:
        captured["request"] = request
        assert timeout == 30
        return FakeResponse()

    monkeypatch.setattr("ai.llm.urlopen", fake_urlopen)
    config = RobotConfig(
        gemini_api_key="test-key",
        gemini_model="test-model",
        gemini_api_url="https://example.test/v1/models/{model}:generate",
    )

    result = asyncio.run(LLMService(config).infer_intent("stop now"))

    assert result == {"intent": "stop", "target": "self", "message": "stop now"}
    assert captured["request"].full_url == (
        "https://example.test/v1/models/test-model:generate?key=test-key"
    )