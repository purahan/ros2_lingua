from setuptools import setup, find_packages

setup(
    name="ros2_lingua_core",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        # No ROS dependencies — intentionally lean
    ],
    extras_require={
        "openai": ["openai>=1.0.0"],
        "anthropic": ["anthropic>=0.20.0"],
        "ollama": ["ollama>=0.1.0"],
        "all": ["openai>=1.0.0", "anthropic>=0.20.0", "ollama>=0.1.0"],
    },
    author="ros2_lingua contributors",
    description="ROS-agnostic core for the ros2_lingua LLM-to-action bridge",
    license="Apache-2.0",
)
