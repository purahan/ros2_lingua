import json
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from ros2_lingua.grounding_node import GroundingNode
from ros2_lingua_core import Capability
from ros2_lingua_interfaces.srv import GroundInstruction, RegisterCapability, UpdateState


class _InfeasiblePlan:
    feasible: bool = False
    reason: str = "test grounding released"

    def to_json(self) -> str:
        return "{}"


def _wait_for_future(future: Future, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()


def test_registry_services_run_while_grounding_is_blocked(monkeypatch) -> None:
    rclpy.init()
    monkeypatch.setattr(
        GroundingNode,
        "_build_engine",
        lambda self: SimpleNamespace(ground=lambda instruction: _InfeasiblePlan()),
    )
    server = GroundingNode()
    client_node = Node("grounding_concurrency_test_client")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    ground_started = threading.Event()
    release_ground = threading.Event()

    def block_grounding(instruction: str) -> _InfeasiblePlan:
        ground_started.set()
        release_ground.wait(5.0)
        return _InfeasiblePlan()

    server._engine.ground = block_grounding

    ground_future = None
    try:
        ground_client = client_node.create_client(GroundInstruction, "lingua/ground")
        register_client = client_node.create_client(
            RegisterCapability, "lingua/register_capability"
        )
        state_client = client_node.create_client(UpdateState, "lingua/update_state")
        assert ground_client.wait_for_service(timeout_sec=3.0)
        assert register_client.wait_for_service(timeout_sec=3.0)
        assert state_client.wait_for_service(timeout_sec=3.0)

        ground_request = GroundInstruction.Request()
        ground_request.instruction = "wait for registry changes"
        ground_future = ground_client.call_async(ground_request)
        assert ground_started.wait(3.0)

        capability = Capability(
            name="registered_during_grounding",
            description="registered while the LLM is busy",
            ros_action="robot/test",
        )
        register_request = RegisterCapability.Request()
        register_request.capability_json = json.dumps(capability.to_dict())
        register_future = register_client.call_async(register_request)
        assert _wait_for_future(register_future, 2.0), "registration waited for grounding to finish"
        assert register_future.result().success

        state_request = UpdateState.Request()
        state_request.state_json = json.dumps({"set": ["robot_ready"], "clear": []})
        state_future = state_client.call_async(state_request)
        assert _wait_for_future(state_future, 2.0), "state update waited for grounding to finish"
        assert state_future.result().success
        assert "registered_during_grounding" in server._registry.names()
        assert "robot_ready" in server._registry.get_state()
        assert not release_ground.is_set()
    finally:
        release_ground.set()
        if ground_future is not None:
            _wait_for_future(ground_future, 2.0)
        executor.shutdown(timeout_sec=3.0)
        spin_thread.join(timeout=3.0)
        server.destroy_node()
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
