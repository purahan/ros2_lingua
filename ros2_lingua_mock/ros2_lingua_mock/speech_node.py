"""
ros2_lingua_mock.speech_node
------------------------------
Simulates a text-to-speech (TTS) system.

Advertises capabilities:
  - say      : speaks a message aloud
  - ask      : asks a question and waits for acknowledgement

In a real robot, this would wrap your TTS engine
(espeak, Festival, pyttsx3, ElevenLabs, etc.)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua_core import Capability, CapabilityParameter


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

        # --- Service for 'say' (using a service since TTS is quick and synchronous) ---
        self._say_srv = self.create_service(
            Trigger,
            "humanoid/tts",
            self._handle_say,
            callback_group=self._callback_group,
        )

        # Register capabilities
        self.register_lingua_capability(Capability(
            name="say",
            description=(
                "Speaks a message aloud using the robot's text-to-speech system. "
                "Can be used to greet people, report status, or narrate actions."
            ),
            ros_service="humanoid/tts",
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
            metadata={"category": "speech"},
        ))

        self._log("MockSpeechNode ready.")
        self.get_logger().info("MockSpeechNode ready.")

    def _handle_say(self, request, response):
        # In the mock, the message is passed via request.name (Trigger repurpose)
        # In real usage, use a proper custom service with a 'message' field
        message = getattr(request, "name", "Hello!")

        word_count = len(message.split())
        speak_time = max(0.5, word_count / 3.0)

        self.get_logger().info(f"Speech: saying '{message}'")
        self._log(f"🔊 \"{message}\"")

        # Publish to speech output topic (useful for monitoring in the demo)
        speech_msg = String()
        speech_msg.data = message
        self._speech_pub.publish(speech_msg)

        time.sleep(speak_time)

        response.success = True
        response.message = f"Said: {message}"
        return response

    def _log(self, message: str):
        msg = String()
        msg.data = f"[SPEECH] {message}"
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockSpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
