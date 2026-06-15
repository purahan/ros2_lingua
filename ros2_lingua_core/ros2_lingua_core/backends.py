"""
ros2_lingua_core.backends
--------------------------
Concrete LLM backend implementations with robust error handling.

Each backend wraps its underlying client with:
- Specific exception types (LLMTimeoutError, LLMRateLimitError, etc.)
- Retry logic with exponential backoff (via RetryConfig)
- Consistent error messages

Available backends:
- OpenAIBackend     : OpenAI chat completion API
- AnthropicBackend  : Anthropic Claude API
- OllamaBackend     : Local Ollama instance (no API key needed)
- MockBackend       : Deterministic mock for testing
"""

import logging
import time
from dataclasses import dataclass

from .errors import (
    LLMBackendError,
    LLMModelNotFoundError,
    LLMRateLimitError,
    LLMTimeoutError,
)

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Retry configuration
# ------------------------------------------------------------------

@dataclass
class RetryConfig:
    """
    Controls retry behavior for LLM backend calls.

    Attributes:
        max_retries:    Maximum number of retry attempts (0 = no retries)
        base_delay_sec: Initial wait between retries in seconds
        backoff_factor: Multiplier applied to delay on each retry
                        e.g. base=1.0, factor=2.0 -> waits 1s, 2s, 4s
        retry_on_timeout: Whether to retry on timeout errors
        retry_on_rate_limit: Whether to retry on rate limit errors
    """
    max_retries: int = 3
    base_delay_sec: float = 1.0
    backoff_factor: float = 2.0
    retry_on_timeout: bool = True
    retry_on_rate_limit: bool = True

    @classmethod
    def no_retry(cls) -> "RetryConfig":
        """Convenience constructor for disabling retries entirely."""
        return cls(max_retries=0)

    @classmethod
    def aggressive(cls) -> "RetryConfig":
        """More retries with longer backoff — for production use."""
        return cls(max_retries=5, base_delay_sec=2.0, backoff_factor=2.0)


def _with_retry(fn, retry_config: RetryConfig, backend_name: str):
    """
    Execute fn() with retry logic based on retry_config.
    Raises the last exception if all retries are exhausted.
    """
    last_error: Exception | None = None
    delay = retry_config.base_delay_sec

    for attempt in range(retry_config.max_retries + 1):
        try:
            return fn()
        except LLMRateLimitError as e:
            last_error = e
            if not retry_config.retry_on_rate_limit or attempt == retry_config.max_retries:
                raise
            logger.warning(
                f"[{backend_name}] Rate limit hit. "
                f"Retrying in {delay:.1f}s (attempt {attempt+1}/{retry_config.max_retries})..."
            )
        except LLMTimeoutError as e:
            last_error = e
            if not retry_config.retry_on_timeout or attempt == retry_config.max_retries:
                raise
            logger.warning(
                f"[{backend_name}] Timeout. "
                f"Retrying in {delay:.1f}s (attempt {attempt+1}/{retry_config.max_retries})..."
            )
        except LLMModelNotFoundError:
            # Never retry model-not-found — it won't fix itself
            raise
        except LLMBackendError as e:
            last_error = e
            if attempt == retry_config.max_retries:
                raise
            logger.warning(
                f"[{backend_name}] Error: {e}. "
                f"Retrying in {delay:.1f}s (attempt {attempt+1}/{retry_config.max_retries})..."
            )

        time.sleep(delay)
        delay *= retry_config.backoff_factor

    raise last_error


# ------------------------------------------------------------------
# OpenAI Backend
# ------------------------------------------------------------------

class OpenAIBackend:
    """
    LLM backend using OpenAI's API.

    Requires: pip install openai

    Usage:
        backend = OpenAIBackend(api_key="sk-...", model="gpt-4o")
        backend = OpenAIBackend(api_key="sk-...", retry=RetryConfig.aggressive())
    """

    def __init__(
        self,
        api_key: str,
        model: str = "gpt-4o",
        timeout_sec: float = 30.0,
        retry: RetryConfig = None,
    ):
        try:
            from openai import OpenAI
        except ImportError:
            raise ImportError(
                "openai package is required for OpenAIBackend. "
                "Install with: pip install openai"
            ) from None
        self._client = OpenAI(api_key=api_key, timeout=timeout_sec)  # type: ignore
        self._model = model
        self._retry = retry or RetryConfig()

    def complete(self, messages: list[dict[str, str]]) -> str:
        def _call():
            try:
                response = self._client.chat.completions.create(
                    model=self._model,
                    messages=messages,  # type: ignore
                    temperature=0.0,
                )
                return response.choices[0].message.content or ""
            except Exception as e:
                err_str = str(e).lower()
                if "rate limit" in err_str or "429" in err_str:
                    raise LLMRateLimitError(f"OpenAI rate limit: {e}", original=e) from e
                elif "timeout" in err_str or "timed out" in err_str:
                    raise LLMTimeoutError(f"OpenAI timeout: {e}", original=e) from e
                elif "model" in err_str and "not found" in err_str:
                    raise LLMModelNotFoundError(
                        f"OpenAI model '{self._model}' not found.", original=e
                    ) from e
                else:
                    raise LLMBackendError(f"OpenAI error: {e}", original=e) from e

        return _with_retry(_call, self._retry, "OpenAI")


# ------------------------------------------------------------------
# Anthropic Backend
# ------------------------------------------------------------------

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
        timeout_sec: float = 30.0,
        retry: RetryConfig = None,
    ):
        try:
            import anthropic
        except ImportError:
            raise ImportError(
                "anthropic package is required for AnthropicBackend. "
                "Install with: pip install anthropic"
            ) from None
        self._client = anthropic.Anthropic(api_key=api_key)  # type: ignore
        self._model = model
        self._max_tokens = max_tokens
        self._timeout_sec = timeout_sec
        self._retry = retry or RetryConfig()

    def complete(self, messages: list[dict[str, str]]) -> str:
        def _call():
            try:
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
            except Exception as e:
                err_str = str(e).lower()
                if "rate limit" in err_str or "529" in err_str or "overloaded" in err_str:
                    raise LLMRateLimitError(f"Anthropic rate limit: {e}", original=e) from e
                elif "timeout" in err_str:
                    raise LLMTimeoutError(f"Anthropic timeout: {e}", original=e) from e
                elif "model" in err_str and ("not found" in err_str or "invalid" in err_str):
                    raise LLMModelNotFoundError(
                        f"Anthropic model '{self._model}' not found.", original=e
                    ) from e
                else:
                    raise LLMBackendError(f"Anthropic error: {e}", original=e) from e

        return _with_retry(_call, self._retry, "Anthropic")


# ------------------------------------------------------------------
# Ollama Backend
# ------------------------------------------------------------------

class OllamaBackend:
    """
    LLM backend using a locally running Ollama instance.
    No API key required.

    Requires: pip install ollama + Ollama running locally

    Usage:
        backend = OllamaBackend(model="llama3.1")
    """

    def __init__(
        self,
        model: str = "llama3.1",
        host: str = "http://localhost:11434",
        timeout_sec: float = 60.0,
        retry: RetryConfig = None,
    ):
        try:
            import ollama as _ollama
            self._ollama = _ollama
        except ImportError:
            raise ImportError(
                "ollama package is required for OllamaBackend. "
                "Install with: pip install ollama"
            ) from None
        self._model = model
        self._host = host
        self._timeout_sec = timeout_sec
        self._retry = retry or RetryConfig()

    def complete(self, messages: list[dict[str, str]]) -> str:
        def _call():
            try:
                response = self._ollama.chat(
                    model=self._model,
                    messages=messages,
                    options={"temperature": 0.0},
                )
                return response["message"]["content"]
            except Exception as e:
                err_str = str(e).lower()
                if "not found" in err_str or "404" in err_str:
                    raise LLMModelNotFoundError(
                        f"Ollama model '{self._model}' not found. "
                        f"Pull it with: ollama pull {self._model}",
                        original=e,
                    ) from e
                elif "timeout" in err_str or "timed out" in err_str:
                    raise LLMTimeoutError(f"Ollama timeout: {e}", original=e) from e
                elif "connection" in err_str or "refused" in err_str:
                    raise LLMBackendError(
                        f"Cannot connect to Ollama at {self._host}. "
                        "Is Ollama running? Try: ollama serve",
                        original=e,
                    ) from e
                else:
                    raise LLMBackendError(f"Ollama error: {e}", original=e) from e

        return _with_retry(_call, self._retry, "Ollama")


# ------------------------------------------------------------------
# Mock Backend
# ------------------------------------------------------------------

class MockBackend:
    """
    Deterministic mock backend for testing and CI.

    Can be configured to simulate failures for robustness testing.

    Usage:
        # Normal use
        backend = MockBackend('{"feasible": true, "steps": [...]}')

        # Simulate failures
        backend = MockBackend.failing(LLMTimeoutError("timed out"), retries_before_success=2)
    """

    def __init__(self, fixed_response: str):
        self._response = fixed_response
        self._fail_with: Exception | None = None
        self._retries_before_success = 0
        self._call_count = 0

    @classmethod
    def failing(
        cls,
        error: Exception,
        retries_before_success: int = 999,
        success_response: str = "",
    ) -> "MockBackend":
        """
        Create a MockBackend that raises an error for the first
        N calls, then returns success_response.

        Useful for testing retry logic.
        """
        instance = cls(success_response)
        instance._fail_with = error
        instance._retries_before_success = retries_before_success
        return instance

    def complete(self, messages: list[dict[str, str]]) -> str:
        self._call_count += 1
        if self._fail_with and self._call_count <= self._retries_before_success:
            raise self._fail_with
        return self._response
