"""
ros2_lingua_mock/launch/demo.launch.py
----------------------------------------
Starts the complete ros2_lingua demo system with one command:

    ros2 launch ros2_lingua_mock demo.launch.py

Optional arguments:
    llm_backend  : openai | anthropic | ollama (default: ollama)
    llm_model    : model name (default: llama3.1)
    llm_api_key  : API key if using openai or anthropic (default: "")

Examples:
    # Local Ollama (no API key)
    ros2 launch ros2_lingua_mock demo.launch.py

    # OpenAI
    ros2 launch ros2_lingua_mock demo.launch.py llm_backend:=openai llm_api_key:=sk-...

    # Smaller local model
    ros2 launch ros2_lingua_mock demo.launch.py llm_model:=llama3.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch arguments ---
    llm_backend_arg = DeclareLaunchArgument(
        "llm_backend",
        default_value="ollama",
        description="LLM backend to use: openai | anthropic | ollama",
    )
    llm_model_arg = DeclareLaunchArgument(
        "llm_model",
        default_value="llama3.1",
        description="LLM model name",
    )
    llm_api_key_arg = DeclareLaunchArgument(
        "llm_api_key",
        default_value="",
        description="API key (required for openai and anthropic backends)",
    )

    llm_backend = LaunchConfiguration("llm_backend")
    llm_model = LaunchConfiguration("llm_model")
    llm_api_key = LaunchConfiguration("llm_api_key")

    # --- Core nodes ---

    grounding_node = Node(
        package="ros2_lingua",
        executable="grounding_node",
        name="lingua_grounding_node",
        parameters=[{
            "llm_backend": llm_backend,
            "llm_model": llm_model,
            "llm_api_key": llm_api_key,
            "auto_chain": True,
        }],
        output="screen",
    )

    dispatcher_node = Node(
        package="ros2_lingua",
        executable="dispatcher_node",
        name="lingua_dispatcher_node",
        output="screen",
    )

    # --- Mock robot nodes ---
    # Delayed slightly to ensure grounding node is ready to receive registrations

    balance_node = TimerAction(
        period=1.5,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="balance_node",
            name="mock_balance_node",
            output="screen",
        )],
    )

    navigation_node = TimerAction(
        period=2.0,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="navigation_node",
            name="mock_navigation_node",
            output="screen",
        )],
    )

    manipulation_node = TimerAction(
        period=2.5,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="manipulation_node",
            name="mock_manipulation_node",
            output="screen",
        )],
    )

    speech_node = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="speech_node",
            name="mock_speech_node",
            output="screen",
        )],
    )

    # --- Monitor ---
    monitor_node = TimerAction(
        period=3.5,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="robot_monitor",
            name="robot_monitor",
            output="screen",
        )],
    )

    # --- Dashboard ---
    dashboard_node = TimerAction(
        period=4.0,
        actions=[Node(
            package="ros2_lingua_mock",
            executable="dashboard_server",
            name="dashboard_server_node",
            output="screen",
        )],
    )

    return LaunchDescription([
        llm_backend_arg,
        llm_model_arg,
        llm_api_key_arg,
        LogInfo(msg="Starting ros2_lingua demo system..."),
        grounding_node,
        dispatcher_node,
        balance_node,
        navigation_node,
        manipulation_node,
        speech_node,
        monitor_node,
        dashboard_node,
        LogInfo(msg="All nodes launched. Dashboard: http://localhost:8080"),
    ])
