import argparse

import pytest
from pytest import MonkeyPatch

from ros2_lingua_mock import cli


@pytest.mark.parametrize(
    ("arguments", "expected_timeout", "expected_instruction"),
    [
        (["ground", "move", "to", "the", "table"], 30.0, "move to the table"),
        (["ground", "--grounding-timeout", "12.5", "move"], 12.5, "move"),
    ],
)
def test_main_passes_grounding_timeout(
    monkeypatch: MonkeyPatch,
    arguments: list[str],
    expected_timeout: float,
    expected_instruction: str,
) -> None:
    received: dict[str, str | float] = {}

    class FakeCLI:
        def __init__(
            self,
            instruction: str,
            namespace: str = "",
            grounding_timeout: float = 30.0,
        ) -> None:
            received["instruction"] = instruction
            received["namespace"] = namespace
            received["timeout"] = grounding_timeout

        def run(self) -> None:
            pass

        def destroy_node(self) -> None:
            pass

    monkeypatch.setattr(cli.rclpy, "init", lambda: None)
    monkeypatch.setattr(cli.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(cli.rclpy.utilities, "remove_ros_args", lambda argv: argv)
    monkeypatch.setattr(cli, "LinguaCLI", FakeCLI)
    monkeypatch.setattr(cli.sys, "argv", ["cli", *arguments])

    cli.main()

    assert received["instruction"] == expected_instruction
    assert received["timeout"] == expected_timeout


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "not-a-number"])
def test_grounding_timeout_must_be_positive_and_finite(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        cli._positive_timeout(value)


def test_run_uses_configured_grounding_timeout(monkeypatch: MonkeyPatch) -> None:
    class FakeFuture:
        def result(self) -> None:
            return None

    class FakeClient:
        def wait_for_service(self, timeout_sec: float) -> bool:
            return True

        def call_async(self, request: object) -> FakeFuture:
            return FakeFuture()

    cli.rclpy.init()
    node = cli.LinguaCLI("move", grounding_timeout=17.5)
    node._client = FakeClient()
    observed = []
    monkeypatch.setattr(
        cli.rclpy,
        "spin_until_future_complete",
        lambda target, future, timeout_sec: observed.append(timeout_sec),
    )
    try:
        node.run()
        assert observed == [17.5]
    finally:
        node.destroy_node()
        if cli.rclpy.ok():
            cli.rclpy.shutdown()
