"""
ros2_lingua_core.backends
--------------------------
Concrete LLM backend implementations.

Each class implements the LLMBackend protocol (complete() method).
Users can also implement their own by following the same interface.

Available backends:
- OpenAIBackend     : Uses OpenAI's chat completion API (GPT-4o, etc.)
- AnthropicBackend  : Uses Anthropic's Claude API
- OllamaBackend     : Uses a locally running Ollama instance (no API key needed)
"""

from typing import Dict, List, Optional


class OpenAIBackend:
    """
    LLM backend using OpenAI's API.

    Requires: pip install openai

    Usage:
        backend = OpenAIBackend(api_key="sk-...", model="gpt-4o")
    """

    def __init__(self, api_key: str, model: str = "gpt-4o"):
        try:
            from openai import OpenAI
        except ImportError:
            raise ImportError(
                "openai package is required for OpenAIBackend. "
                "Install it with: pip install openai"
            )
        self._client = OpenAI(api_key=api_key)  # type: ignore
        self._model = model

    def complete(self, messages: List[Dict[str, str]]) -> str:
        response = self._client.chat.completions.create(
            model=self._model,
            messages=messages,  # type: ignore
            temperature=0.0,    # deterministic output for action grounding
        )
        return response.choices[0].message.content or ""


class AnthropicBackend:
    """
    LLM backend using Anthropic's Claude API.

    Requires: pip install anthropic

    Usage:
        backend = AnthropicBackend(api_key="sk-ant-...", model="claude-sonnet-4-20250514")
    """

    def __init__(
        self,
        api_key: str,
        model: str = "claude-sonnet-4-20250514",
        max_tokens: int = 1024,
    ):
        try:
            import anthropic
        except ImportError:
            raise ImportError(
                "anthropic package is required for AnthropicBackend. "
                "Install it with: pip install anthropic"
            )
        self._client = anthropic.Anthropic(api_key=api_key)  # type: ignore
        self._model = model
        self._max_tokens = max_tokens

    def complete(self, messages: List[Dict[str, str]]) -> str:
        # Separate system message from user/assistant messages
        system_content = ""
        chat_messages = []

        for msg in messages:
            if msg["role"] == "system":
                system_content = msg["content"]
            else:
                chat_messages.append(msg)

        response = self._client.messages.create(
            model=self._model,
            max_tokens=self._max_tokens,
            system=system_content,
            messages=chat_messages,  # type: ignore
        )
        return response.content[0].text


class OllamaBackend:
    """
    LLM backend using a locally running Ollama instance.
    No API key required — great for offline/on-robot use.

    Requires: pip install ollama  AND  ollama running locally

    Usage:
        backend = OllamaBackend(model="llama3.1")
    """

    def __init__(self, model: str = "llama3.1", host: str = "http://localhost:11434"):
        try:
            import ollama
        except ImportError:
            raise ImportError(
                "ollama package is required for OllamaBackend. "
                "Install it with: pip install ollama"
            )
        import ollama as _ollama
        self._ollama = _ollama
        self._model = model
        self._host = host

    def complete(self, messages: List[Dict[str, str]]) -> str:
        response = self._ollama.chat(
            model=self._model,
            messages=messages,
            options={"temperature": 0.0},
        )
        return response["message"]["content"]


class MockBackend:
    """
    A deterministic mock backend for testing and CI.

    Returns a fixed response — useful for unit tests where you don't
    want to make real LLM API calls.

    Usage:
        backend = MockBackend(fixed_response='{"feasible": true, "steps": [...]}')
    """

    def __init__(self, fixed_response: str):
        self._response = fixed_response

    def complete(self, messages: List[Dict[str, str]]) -> str:
        return self._response
