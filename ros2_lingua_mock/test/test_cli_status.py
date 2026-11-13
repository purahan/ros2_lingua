"""Regression tests for the CLI's execution-status handling (issue #27).

The CLI used to subscribe to ``lingua/execution_status`` with ``std_msgs/String``
while the dispatcher publishes ``ros2_lingua_interfaces/ExecutionStatus``. The
type mismatch meant the CLI never received a status, never printed step or
completion updates, and always sat until its 60 second timeout.

These tests pin down three things:

* the CLI handles every status the dispatcher publishes,
* the CLI prints the step that the status refers to,
* the CLI actually receives ``ExecutionStatus`` messages published on the
  topic (the exact regression from #27).
"""

import inspect
import re
import time

import pytest
import rclpy
from rclpy.node import Node

from ros2_lingua_interfaces.msg import ExecutionStatus
from ros2_lingua_mock import cli

# Every status the dispatcher publishes, the text the CLI prints for it, and
# whether the status ends the execution. Keep this in sync with
# ros2_lingua/ros2_lingua/dispatcher_node.py::_publish_status.
EXPECTED_STATUSES = {
    "STARTED": ("execution started", False),
    "STEP_STARTED": ("step started", False),
    "STEP_COMPLETE": ("step complete", False),
    "STEP_RETRY": ("retry", False),
    "REPLANNING": ("replanning", False),
    "FALLBACK": ("fallback", False),
    "RECOVERY_FAILED": ("recovery failed", True),
    "CANCELLED": ("cancelled", True),
    "COMPLETED": ("execution complete", True),
    "FAILED": ("execution failed", True),
}

# Long enough for DDS discovery between the publisher and the CLI node.
DELIVERY_TIMEOUT = 15.0


@pytest.fixture(scope="module")
def ros_context():
    """A ROS 2 context shared by every test in this module."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def cli_node(ros_context):
    """A LinguaCLI instance, torn down after the test."""
    node = cli.LinguaCLI("go to the table")
    try:
        yield node
    finally:
        node.destroy_node()


def _status_message(status: str, step: str = "navigate_to_location") -> ExecutionStatus:
    msg = ExecutionStatus()
    msg.status = status
    msg.current_step = step
    msg.instruction = "go to the table"
    return msg


def _dispatcher_statuses() -> set[str]:
    """Every status string the dispatcher publishes, read from its source."""
    from ros2_lingua import dispatcher_node

    source = inspect.getsource(dispatcher_node)
    return set(re.findall(r'_publish_status\(\s*"([A-Z_]+)"', source))


def test_dispatcher_publishes_the_expected_statuses() -> None:
    """The dispatcher publishes exactly the statuses the CLI knows about."""
    assert _dispatcher_statuses() == set(EXPECTED_STATUSES)


@pytest.mark.parametrize("status", sorted(EXPECTED_STATUSES))
def test_cli_handles_every_dispatcher_status(cli_node, capsys, status: str) -> None:
    """Every status the dispatcher publishes is reported, not silently dropped."""
    expected_text, terminal = EXPECTED_STATUSES[status]

    cli_node._handle_status(_status_message(status))
    printed = capsys.readouterr().out

    assert printed.strip(), f"CLI printed nothing for status {status}"
    assert expected_text in printed.lower()
    assert cli_node._done is terminal


def test_cli_prints_the_step_a_status_refers_to(cli_node, capsys) -> None:
    """The step that completed is shown to the user (issue #27, task 3)."""
    cli_node._handle_status(_status_message("STEP_COMPLETE", "pick_up_object"))
    printed = capsys.readouterr().out

    assert "pick_up_object" in printed


def test_cli_receives_execution_status_messages(ros_context, capsys) -> None:
    """A published ExecutionStatus reaches the CLI callback.

    With the old ``std_msgs/String`` subscription this message was dropped
    because of the type mismatch, which is what made the CLI hang until its
    timeout.
    """
    publisher = Node("lingua_cli_status_test_pub")
    cli_node = cli.LinguaCLI("go to the table")
    status_pub = publisher.create_publisher(ExecutionStatus, "/lingua/execution_status", 10)
    message = _status_message("COMPLETED", "navigate_to_location")

    deadline = time.monotonic() + DELIVERY_TIMEOUT
    try:
        while time.monotonic() < deadline and not cli_node._done:
            status_pub.publish(message)
            rclpy.spin_once(cli_node, timeout_sec=0.1)
    finally:
        publisher.destroy_node()
        cli_node.destroy_node()

    assert cli_node._done, "CLI never received the ExecutionStatus message"

    printed = capsys.readouterr().out
    assert "navigate_to_location" in printed
