"""
ros2_lingua.grounding_node
---------------------------
A ROS 2 Node that runs the GroundingEngine and exposes it as a service.

Other nodes register their capabilities with this node via the
/lingua/register_capability service. Users (or other nodes) send
natural language instructions via the /lingua/ground service, and
receive back a structured ActionPlan as JSON.

The node also publishes the current plan to /lingua/current_plan so
the dispatcher node can pick it up and execute it.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from ros2_lingua_core import (
    Capability,
    CapabilityRegistry,
    GroundingEngine,
)
from ros2_lingua_core.backends import OpenAIBackend, AnthropicBackend, OllamaBackend

# Custom service types (defined in ros2_lingua_interfaces)
from ros2_lingua_interfaces.srv import RegisterCapability, GroundInstruction, UpdateState


class GroundingNode(Node):
    """
    The central node of ros2_lingua.

    Responsibilities:
    - Maintain the CapabilityRegistry (receives registrations from other nodes)
    - Run the GroundingEngine when an instruction arrives
    - Publish the resulting ActionPlan for the DispatcherNode to execute
    - Track robot state updates from other nodes
    """

    def __init__(self):
        super().__init__("lingua_grounding_node")

        # --- Parameters ---
        self.declare_parameter("llm_backend", "openai")     # openai | anthropic | ollama
        self.declare_parameter("llm_model", "gpt-4o")
        self.declare_parameter("llm_api_key", "")
        self.declare_parameter("ollama_host", "http://localhost:11434")
        self.declare_parameter("auto_chain", True)

        # --- Core objects ---
        self._registry = CapabilityRegistry()
        self._engine = self._build_engine()

        # --- Services ---
        self._register_srv = self.create_service(
            RegisterCapability,
            "/lingua/register_capability",
            self._handle_register,
        )
        self._ground_srv = self.create_service(
            GroundInstruction,
            "/lingua/ground",
            self._handle_ground,
        )
        self._state_srv = self.create_service(
            UpdateState,
            "/lingua/update_state",
            self._handle_update_state,
        )

        # --- Publishers ---
        self._plan_pub = self.create_publisher(String, "/lingua/current_plan", 10)
        self._caps_pub = self.create_publisher(String, "/lingua/capabilities", 10)

        # Publish capabilities periodically so new nodes can inspect them
        self._caps_timer = self.create_timer(5.0, self._publish_capabilities)

        self.get_logger().info("GroundingNode ready.")

    def _build_engine(self) -> GroundingEngine:
        backend_name = self.get_parameter("llm_backend").value
        model = self.get_parameter("llm_model").value
        api_key = self.get_parameter("llm_api_key").value
        auto_chain = self.get_parameter("auto_chain").value

        if backend_name == "openai":
            backend = OpenAIBackend(api_key=api_key, model=model)
        elif backend_name == "anthropic":
            backend = AnthropicBackend(api_key=api_key, model=model)
        elif backend_name == "ollama":
            host = self.get_parameter("ollama_host").value
            backend = OllamaBackend(model=model, host=host)
        else:
            raise ValueError(
                f"Unknown llm_backend '{backend_name}'. "
                "Choose from: openai, anthropic, ollama"
            )

        return GroundingEngine(
            registry=self._registry,
            backend=backend,
            auto_chain=auto_chain,
        )

    def _handle_register(self, request, response):
        """Handle a capability registration request."""
        try:
            cap_dict = json.loads(request.capability_json)
            capability = Capability.from_dict(cap_dict)
            self._registry.update(capability)
            self.get_logger().info(f"Registered capability: '{capability.name}'")
            response.success = True
            response.message = f"Capability '{capability.name}' registered."
        except Exception as e:
            self.get_logger().error(f"Failed to register capability: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _handle_ground(self, request, response):
        """Ground a natural language instruction into an ActionPlan."""
        instruction = request.instruction
        self.get_logger().info(f"Grounding instruction: '{instruction}'")

        try:
            plan = self._engine.ground(instruction)
            plan_json = plan.to_json()

            response.success = plan.feasible
            response.plan_json = plan_json
            response.message = plan.reason if not plan.feasible else "OK"

            if plan.feasible:
                # Publish for the dispatcher node
                msg = String()
                msg.data = plan_json
                self._plan_pub.publish(msg)
                self.get_logger().info(
                    f"Plan generated: {len(plan.steps)} step(s)."
                )
            else:
                self.get_logger().warn(
                    f"Instruction not feasible: {plan.reason}"
                )

        except Exception as e:
            self.get_logger().error(f"Grounding error: {e}")
            response.success = False
            response.plan_json = "{}"
            response.message = str(e)

        return response

    def _handle_update_state(self, request, response):
        """Update the robot's symbolic state."""
        try:
            state_data = json.loads(request.state_json)
            set_tokens = set(state_data.get("set", []))
            clear_tokens = set(state_data.get("clear", []))

            for token in set_tokens:
                self._registry.set_state(token)
            for token in clear_tokens:
                self._registry.clear_state(token)

            response.success = True
            response.message = (
                f"State updated. Set: {set_tokens}, Cleared: {clear_tokens}"
            )
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _publish_capabilities(self):
        """Periodically broadcast all registered capabilities."""
        caps = [c.to_dict() for c in self._registry.get_all()]
        msg = String()
        msg.data = json.dumps(caps)
        self._caps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
