# ros2_lingua

> **Give your robot the ability to understand natural language — on any ROS 2 platform.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Tests](https://img.shields.io/badge/Tests-20%20passed-brightgreen)]()
[![Release](https://img.shields.io/github/v/release/purahan/ros2_lingua)](https://github.com/purahan/ros2_lingua/releases)
[![Issues](https://img.shields.io/github/issues/purahan/ros2_lingua)](https://github.com/purahan/ros2_lingua/issues)

📖 **[Documentation](https://purahan.github.io/ros2_lingua/)**

`ros2_lingua` is a ROS 2 library that bridges the gap between large language models (LLMs) and real robot control. Your nodes declare what they can do through a structured **capability schema**, and the grounding engine automatically translates natural language instructions into validated, dependency-aware execution plans — dispatched over your existing ROS 2 actions and services.

```
User: "go pick up the bottle from the table"

        ┌──────────────────────────────┐
        │       Grounding Engine       │
        │   (+ Backward Chain Planner) │
        └──────────────────────────────┘
                       ↓
   Step 1 → stabilize_robot          [auto-chained prerequisite]
   Step 2 → navigate_to_location     { location_name: "table" }
   Step 3 → pick_up_object           { object_name: "bottle" }
                       ↓
        ┌──────────────────────────────┐
        │        Dispatcher Node       │
        │  ROS 2 Actions  │  Services  │
        └──────────────────────────────┘
```

---

## Table of Contents

- [Why ros2_lingua?](#why-ros2_lingua)
- [Key Features](#key-features)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Trying It Without a Robot](#trying-it-without-a-robot)
- [Supported LLM Backends](#supported-llm-backends)
- [Running Tests](#running-tests)
- [Package Structure](#package-structure)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)

---

## Why ros2_lingua?

Most LLM-to-robot integrations today are one-off scripts — they hardcode topic names, assume a fixed sequence of actions, and break the moment anything changes. There is no standard way for a ROS 2 node to tell the world what it can do, and no safe mechanism to validate what an LLM suggests before it executes on real hardware.

`ros2_lingua` solves this by introducing a **structured capability contract** between your nodes and the LLM. Nodes self-describe their abilities. The engine validates every suggested action before dispatch. Nothing runs on your robot that wasn't explicitly registered.

---

## Key Features

- **Structured capability advertisement** — any ROS 2 node can declare what it can do, what it needs, and what it produces, in a schema the LLM understands
- **Automatic prerequisite chaining** — a backward-chaining planner resolves dependencies between capabilities automatically, so you never hardcode execution sequences
- **Hallucination protection** — every capability the LLM suggests is validated against the registered capability list before a single action is dispatched
- **LLM-agnostic** — works with OpenAI, Anthropic Claude, local Ollama models, or any custom backend you implement
- **Robot-agnostic** — works on any ROS 2 robot: mobile manipulators, wheeled AMRs, robotic arms, drones, AUVs, humanoids
- **ROS-free core** — the schema, registry, and grounding engine have zero ROS 2 dependencies and can be unit tested independently

---

## Architecture

`ros2_lingua` is split into two layers intentionally — the core logic is kept completely free of ROS 2 so it can be tested, extended, and reasoned about independently.

```
┌──────────────────────────────────────────────────────────┐
│                    ros2_lingua_core                       │
│                  (No ROS 2 dependencies)                  │
│                                                           │
│   Capability Schema    CapabilityRegistry                 │
│   GroundingEngine      Backward Chain Planner             │
│   LLM Backends         ActionPlan / ActionStep            │
└──────────────────────────────────────────────────────────┘
                            ↕
┌──────────────────────────────────────────────────────────┐
│                      ros2_lingua                          │
│                    (ROS 2 interface layer)                 │
│                                                           │
│   GroundingNode        DispatcherNode                     │
│   LinguaMixin          ros2_lingua_interfaces             │
└──────────────────────────────────────────────────────────┘
                            ↕
┌──────────────────────────────────────────────────────────┐
│                    Your Robot Nodes                       │
│   NavigationNode   ManipulationNode   AnyOtherNode        │
└──────────────────────────────────────────────────────────┘
```

**`ros2_lingua_core`** handles all the intelligence — capability definitions, the registry, the grounding engine, backward chaining, and LLM backends. Pure Python, no ROS.

**`ros2_lingua`** wraps the core in ROS 2 nodes and services — the `GroundingNode` exposes the engine as a service, the `DispatcherNode` executes plans on your robot, and `LinguaMixin` lets any node self-register its capabilities at boot.

---

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- At least one LLM backend:
  - [Ollama](https://ollama.com/) (recommended for local use, no API key needed)
  - OpenAI API key
  - Anthropic API key

---

## Installation

```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_lingua_ws/src
cd ~/ros2_lingua_ws/src

# Clone the repository
git clone https://github.com/YOUR_USERNAME/ros2_lingua.git

# Copy packages into the workspace
cp -r ros2_lingua/ros2_lingua_core .
cp -r ros2_lingua/ros2_lingua .
cp -r ros2_lingua/ros2_lingua_interfaces .

# Build
cd ~/ros2_lingua_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Quick Start

### 1. Make your node advertise its capabilities

Inherit `LinguaMixin` alongside `Node` and call `register_lingua_capability()` in your constructor:

```python
from rclpy.node import Node
from ros2_lingua import LinguaMixin
from ros2_lingua_core import Capability, CapabilityParameter

class NavigationNode(LinguaMixin, Node):
    def __init__(self):
        Node.__init__(self, "navigation_node")
        LinguaMixin.__init__(self)

        self.register_lingua_capability(Capability(
            name="navigate_to_location",
            description="Moves the robot to a named location in the environment",
            ros_action="robot/navigate_to_pose",
            parameters=[
                CapabilityParameter(
                    name="location_name",
                    type="string",
                    description="Where to go, e.g. 'table', 'door', 'charging_dock'",
                )
            ],
            preconditions=["robot_is_ready"],
            postconditions=["robot_at_location"],
        ))
```

### 2. Launch the grounding and dispatcher nodes

With a local Ollama model (no API key required):
```bash
ros2 run ros2_lingua grounding_node --ros-args \
    -p llm_backend:=ollama \
    -p llm_model:=llama3.1
```

With OpenAI:
```bash
ros2 run ros2_lingua grounding_node --ros-args \
    -p llm_backend:=openai \
    -p llm_model:=gpt-4o \
    -p llm_api_key:=sk-...
```

```bash
# In a second terminal
ros2 run ros2_lingua dispatcher_node
```

### 3. Send a natural language instruction

```bash
ros2 service call /lingua/ground \
    ros2_lingua_interfaces/srv/GroundInstruction \
    "{instruction: 'go to the table and pick up the bottle'}"
```

The engine checks preconditions, resolves the dependency chain, validates every step against registered capabilities, and returns a structured plan ready for dispatch.

---

## Trying It Without a Robot

The entire core library runs with no ROS 2 setup at all. This is a good starting point to understand how the system works before integrating it with your robot:

```bash
cd examples/humanoid_demo
PYTHONPATH=../../ros2_lingua_core python3 humanoid_demo.py
```

Expected output:
```
✅ Registered 5 capabilities.
🤖 Initial state: ['arm_is_free', 'object_in_view']

📢 Instruction: "pick up the bottle from the table"
✅ Plan (3 steps):
   1. stabilize_robot        [auto-chained]
   2. navigate_to_location   { location_name: "table" }
   3. pick_up_object         { object_name: "bottle", arm: "right" }
```

---

## Supported LLM Backends

| Backend | Class | Install |
|---|---|---|
| Ollama (local, no API key) | `OllamaBackend` | `pip install ollama` |
| OpenAI (GPT-4o, etc.) | `OpenAIBackend` | `pip install openai` |
| Anthropic Claude | `AnthropicBackend` | `pip install anthropic` |
| Custom / Testing | Implement `complete(messages) -> str` | — |

You can also bring your own backend by implementing a single method:

```python
class MyBackend:
    def complete(self, messages: list[dict]) -> str:
        # Call your LLM here, return the response string
        ...
```

---

## Running Tests

The core library tests have no ROS 2 dependency and run with plain pytest:

```bash
cd ros2_lingua_core
python3 -m pytest tests/test_core.py -v
```

All 20 tests cover the capability schema, registry, backward chaining planner, and grounding engine — including hallucination protection and malformed LLM response handling.

---

## Package Structure

```
ros2_lingua/
│
├── ros2_lingua_core/               # ROS-agnostic core — independently testable
│   ├── ros2_lingua_core/
│   │   ├── __init__.py             # Public API
│   │   ├── schema.py               # Capability + CapabilityParameter definitions
│   │   ├── registry.py             # CapabilityRegistry + backward chain planner
│   │   ├── grounding.py            # GroundingEngine, ActionPlan, ActionStep
│   │   └── backends.py             # OpenAI, Anthropic, Ollama, Mock backends
│   ├── tests/
│   │   └── test_core.py            # 20 unit tests, no ROS required
│   ├── package.xml
│   └── setup.py
│
├── ros2_lingua/                    # ROS 2 interface layer
│   ├── ros2_lingua/
│   │   ├── grounding_node.py       # Exposes GroundingEngine as a ROS 2 service
│   │   ├── dispatcher_node.py      # Executes ActionPlans over actions/services
│   │   └── capability_mixin.py     # Mixin for self-registering capabilities
│   ├── package.xml
│   └── setup.py
│
├── ros2_lingua_interfaces/         # Custom ROS 2 service + message definitions
│   ├── srv/
│   │   ├── RegisterCapability.srv
│   │   ├── GroundInstruction.srv
│   │   └── UpdateState.srv
│   └── msg/
│       └── ExecutionStatus.msg
│
└── examples/
    └── humanoid_demo/
        └── humanoid_demo.py        # Runnable demo, zero ROS 2 setup needed
```

---

## Roadmap

- [ ] Launch files — start the full system with one command
- [ ] Mock robot nodes — simulate a real robot for testing and demos
- [ ] CLI tool — `ros2 lingua ground "your instruction"`
- [ ] Capability tagging — filter capabilities by category (locomotion, manipulation, etc.)
- [ ] Web dashboard — live visualization of capabilities, state, and execution plans
- [ ] C++ bindings — capability advertisement from C++ controller nodes
- [ ] Voice input — Whisper integration for spoken instructions
- [ ] Integration tests — full ROS 2 pipeline testing with `launch_testing`

---

## Contributing

This project is in active early development. Contributions, issues, and feedback are all welcome. If you're using `ros2_lingua` on your robot, we'd love to hear about it — open an issue and tell us what you're building.

---

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
