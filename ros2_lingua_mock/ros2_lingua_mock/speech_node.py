"""
ros2_lingua_mock.speech_node
------------------------------
Simulates a text-to-speech (TTS) system.

Advertises capabilities:
  - say      : speaks a message aloud

Serves the generic LinguaService interface at mock_speech_node/lingua.
Speech is quick and synchronous, so it uses a service rather than an action.

In a real robot, this would wrap your TTS engine
(espeak, Festival, pyttsx3, ElevenLabs, etc.)
"""

import json
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua_core import Capability, CapabilityParameter
from ros2_lingua_interfaces.srv import LinguaService


class MockSpeechNode(LinguaMixin, Node):
    """
    Simulates TTS by printing messages and waiting a realistic duration
    based on estimated speaking time (roughly 3 words per second).
    """

    def __init__(self):
        Node.__init__(self, "mock_speech_node")
        LinguaMixin.__init__(self)

        self._callback_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)
        self._speech_pub = self.create_publisher(String, "/mock/speech_output", 10)

        # --- LinguaService server ---
        service_name = f"{self.get_name()}/lingua"
        self._lingua_srv = self.create_service(
            LinguaService,
            service_name,
            self._handle_lingua_service,
            callback_group=self._callback_group,
        )

        # Register capabilities
        self.register_lingua_capability(
            Capability(
                name="say",
                description=(
                    "Speaks a message aloud using the robot's text-to-speech system. "
                    "Can be used to greet people, report status, or narrate actions."
                ),
                ros_service=service_name,
                parameters=[
                    CapabilityParameter(
                        name="message",
                        type="string",
                        description="The message to speak aloud",
                        required=True,
                    ),
                ],
                preconditions=[],
                postconditions=[],
                metadata={},
                tags=["speech", "social"],
            )
        )

        self._log("MockSpeechNode ready.")
        self.get_logger().info("MockSpeechNode ready.")

    def _handle_lingua_service(self, request, response):
        response.state_set = []
        response.state_clear = []

        try:
            parameters = json.loads(request.parameters_json) if request.parameters_json else {}
        except json.JSONDecodeError:
            self.get_logger().error(f"Speech: invalid JSON params: {request.parameters_json}")
            response.success = False
            return response

        if request.capability_name != "say":
            self.get_logger().warn(f"Speech: unknown capability '{request.capability_name}'")
            response.success = False
            return response

        response.success = self._say(str(parameters.get("message", "")).strip())
        return response

    def _say(self, message: str) -> bool:
        if not message:
            self.get_logger().warn("Speech: 'say' called without a message")
            return False

        word_count = len(message.split())
        speak_time = max(0.5, word_count / 3.0)

        self.get_logger().info(f"Speech: saying '{message}'")
        self._log(f'🔊 "{message}"')

        # Publish to speech output topic (useful for monitoring in the demo)
        speech_msg = String()
        speech_msg.data = message
        self._speech_pub.publish(speech_msg)

        time.sleep(speak_time)
        return True

    def _log(self, message: str):
        msg = String()
        msg.data = f"[SPEECH] {message}"
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockSpeechNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT may already have shut the context down


if __name__ == "__main__":
    main()
