"""
ros2_lingua_core
-----------------
The ROS-agnostic core of the ros2_lingua library.

This package contains zero ROS 2 dependencies, making it independently
unit-testable and usable in non-ROS contexts if needed.

Public API:
    from ros2_lingua_core import (
        Capability,
        CapabilityParameter,
        CapabilityRegistry,
        GroundingEngine,
        ActionPlan,
        ActionStep,
        OpenAIBackend,
        AnthropicBackend,
        OllamaBackend,
        MockBackend,
    )
"""

from .schema import Capability, CapabilityParameter
from .registry import CapabilityRegistry
from .grounding import GroundingEngine, ActionPlan, ActionStep, LLMBackend
from .backends import OpenAIBackend, AnthropicBackend, OllamaBackend, MockBackend

__version__ = "0.1.0"
__author__ = "ros2_lingua contributors"

__all__ = [
    "Capability",
    "CapabilityParameter",
    "CapabilityRegistry",
    "GroundingEngine",
    "ActionPlan",
    "ActionStep",
    "LLMBackend",
    "OpenAIBackend",
    "AnthropicBackend",
    "OllamaBackend",
    "MockBackend",
]
