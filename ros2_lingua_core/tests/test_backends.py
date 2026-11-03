"""
Unit tests for LLM backends that previously ignored host/timeout options.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from ros2_lingua_core.backends import AnthropicBackend, OllamaBackend, RetryConfig


class TestOllamaBackendOptions:
    def test_uses_client_bound_to_host_and_timeout(self) -> None:
        mock_client = MagicMock()
        mock_client.chat.return_value = {"message": {"content": "ok"}}
        mock_mod = MagicMock()
        mock_mod.Client.return_value = mock_client

        with patch.dict("sys.modules", {"ollama": mock_mod}):
            backend = OllamaBackend(
                model="llama3.1",
                host="http://ollama.internal:11434",
                timeout_sec=12.5,
                retry=RetryConfig.no_retry(),
            )
            assert backend.complete([{"role": "user", "content": "hi"}]) == "ok"

        mock_mod.Client.assert_called_once_with(
            host="http://ollama.internal:11434", timeout=12.5
        )
        mock_client.chat.assert_called_once()
        kwargs = mock_client.chat.call_args.kwargs
        assert kwargs["model"] == "llama3.1"
        assert kwargs["options"] == {"temperature": 0.0}


class TestAnthropicBackendOptions:
    def test_passes_timeout_and_temperature(self) -> None:
        mock_client = MagicMock()
        mock_client.messages.create.return_value = SimpleNamespace(
            content=[SimpleNamespace(text="claude says hi")]
        )
        mock_anthropic = MagicMock()
        mock_anthropic.Anthropic.return_value = mock_client

        with patch.dict("sys.modules", {"anthropic": mock_anthropic}):
            backend = AnthropicBackend(
                api_key="sk-test",
                model="claude-test",
                timeout_sec=9.0,
                retry=RetryConfig.no_retry(),
            )
            assert (
                backend.complete(
                    [
                        {"role": "system", "content": "be brief"},
                        {"role": "user", "content": "hi"},
                    ]
                )
                == "claude says hi"
            )

        mock_anthropic.Anthropic.assert_called_once_with(
            api_key="sk-test", timeout=9.0
        )
        kwargs = mock_client.messages.create.call_args.kwargs
        assert kwargs["temperature"] == 0.0
        assert kwargs["model"] == "claude-test"
        assert kwargs["system"] == "be brief"
