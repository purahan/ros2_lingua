from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Arguments
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Namespace to apply to all lingua nodes (e.g. 'robot_1')",
    )

    llm_backend_arg = DeclareLaunchArgument(
        "llm_backend",
        default_value="openai",
        description="LLM backend to use: 'openai', 'anthropic', 'ollama', or 'mock'",
    )

    llm_model_arg = DeclareLaunchArgument(
        "llm_model",
        default_value="",
        description="LLM model override. Leave empty to use the selected backend's default.",
    )

    ollama_host_arg = DeclareLaunchArgument(
        "ollama_host",
        default_value="http://localhost:11434",
        description="URL for the Ollama server (if using ollama backend)",
    )

    auto_chain_arg = DeclareLaunchArgument(
        "auto_chain",
        default_value="True",
        description="Whether to automatically chain capabilities to satisfy preconditions",
    )

    cache_ttl_sec_arg = DeclareLaunchArgument(
        "cache_ttl_sec",
        default_value="0.0",
        description="Time to live (in seconds) for the plan cache. 0.0 disables caching.",
    )

    allow_demo_mode_arg = DeclareLaunchArgument(
        "allow_demo_mode",
        default_value="False",
        description="Simulate steps whose server is unavailable instead of failing them. "
        "Never enable on a real robot.",
    )

    # Nodes
    grounding_node = Node(
        package="ros2_lingua",
        executable="grounding_node",
        name="lingua_grounding_node",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        parameters=[
            {
                "llm_backend": LaunchConfiguration("llm_backend"),
                "llm_model": ParameterValue(LaunchConfiguration("llm_model"), value_type=str),
                "ollama_host": LaunchConfiguration("ollama_host"),
                "auto_chain": LaunchConfiguration("auto_chain"),
                "cache_ttl_sec": LaunchConfiguration("cache_ttl_sec"),
            }
        ],
    )

    dispatcher_node = Node(
        package="ros2_lingua",
        executable="dispatcher_node",
        name="lingua_dispatcher_node",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        parameters=[{"allow_demo_mode": LaunchConfiguration("allow_demo_mode")}],
    )

    return LaunchDescription(
        [
            robot_namespace_arg,
            llm_backend_arg,
            llm_model_arg,
            ollama_host_arg,
            auto_chain_arg,
            cache_ttl_sec_arg,
            allow_demo_mode_arg,
            grounding_node,
            dispatcher_node,
        ]
    )
