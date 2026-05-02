# ros2_lingua

**A ROS 2 library that lets your robot understand natural language — structured, validated, and production-ready.**

ros2_lingua bridges the gap between large language models (LLMs) and real ROS 2 robot control. Nodes advertise what they can do via a structured *capability schema*, and the grounding engine translates natural language instructions into validated, dependency-aware execution plans — dispatched over real ROS 2 actions and services.

```
"pick up the bottle from the table"
        ↓ GroundingEngine
1. stabilize_robot       [auto-chained prerequisite]
2. navigate_to_location  {location_name: "table"}
3. pick_up_object        {object_name: "bottle", arm: "right"}
        ↓ DispatcherNode
    → ROS 2 Actions / Services
```

---

## Why ros2_lingua?

Most LLM-robot integrations are brittle scripts that hardcode topic names and pray the LLM outputs something parseable. ros2_lingua gives you:

- ✅ **Structured capability advertisement** — nodes declare what they can do in a schema the LLM can reason about
- ✅ **Automatic prerequisite chaining** — the engine resolves dependencies (preconditions/postconditions) so you don't have to hardcode sequences
- ✅ **Hallucination protection** — the engine validates every LLM-suggested action against registered capabilities before dispatching
- ✅ **LLM-agnostic** — works with OpenAI, Anthropic Claude, local Ollama models, or any custom backend
- ✅ **ROS-agnostic core** — the schema, registry, and grounding engine have zero ROS 2 dependencies, so they're independently testable

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                ros2_lingua_core                  │  ← No ROS 2 deps
│  CapabilitySchema  Registry  GroundingEngine     │
└─────────────────────────────────────────────────┘
                        ↕
┌─────────────────────────────────────────────────┐
│                  ros2_lingua                     │  ← ROS 2 layer
│   GroundingNode   DispatcherNode   LinguaMixin   │
└─────────────────────────────────────────────────┘
                        ↕
┌─────────────────────────────────────────────────┐
│              Your Robot Nodes                    │
│   NavigationNode   ManipulationNode   etc.       │
└─────────────────────────────────────────────────┘
```

**`ros2_lingua_core`** — Pure Python, no ROS. Schema definitions, registry, grounding engine, LLM backends.

**`ros2_lingua`** — ROS 2 wrapper. GroundingNode (runs the engine as a service), DispatcherNode (executes plans), LinguaMixin (lets any node self-register capabilities).

---

## Quick Start

### 1. Define capabilities in your node

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
            description="Walks the robot to a named location in the environment",
            ros_action="robot/navigate_to_pose",
            parameters=[
                CapabilityParameter(
                    name="location_name",
                    type="string",
                    description="Named location, e.g. 'table', 'door', 'charging_dock'",
                )
            ],
            preconditions=["robot_is_balanced"],
            postconditions=["robot_at_location"],
        ))
```

### 2. Launch the grounding node

```bash
ros2 launch ros2_lingua grounding.launch.py \
    llm_backend:=openai \
    llm_model:=gpt-4o \
    llm_api_key:=sk-...
```

Or with a local Ollama model (no API key):
```bash
ros2 launch ros2_lingua grounding.launch.py \
    llm_backend:=ollama \
    llm_model:=llama3.1
```

### 3. Send a natural language instruction

```bash
ros2 service call /lingua/ground ros2_lingua_interfaces/srv/GroundInstruction \
    "{instruction: 'go pick up the bottle from the table'}"
```

The engine automatically:
- Checks if `robot_is_balanced` is satisfied
- If not, inserts `stabilize_robot` as a prerequisite
- Dispatches `navigate_to_location` → `pick_up_object` in order

---

## Trying It Without a Robot

The core logic runs with zero ROS 2 setup:

```bash
cd examples/humanoid_demo
python3 humanoid_demo.py
```

---

## Supported LLM Backends

| Backend | Class | Requirement |
|---|---|---|
| OpenAI (GPT-4o, etc.) | `OpenAIBackend` | `pip install openai` |
| Anthropic Claude | `AnthropicBackend` | `pip install anthropic` |
| Ollama (local) | `OllamaBackend` | `pip install ollama` + Ollama running |
| Custom / Mock | Implement `complete()` | None |

---

## Running Tests

The core library tests require no ROS 2:

```bash
cd ros2_lingua_core
pytest tests/ -v
```

---

## Package Structure

```
ros2_lingua/
├── ros2_lingua_core/          # ROS-agnostic core (independently testable)
│   ├── ros2_lingua_core/
│   │   ├── schema.py          # Capability, CapabilityParameter
│   │   ├── registry.py        # CapabilityRegistry + backward chaining
│   │   ├── grounding.py       # GroundingEngine, ActionPlan, ActionStep
│   │   └── backends.py        # OpenAI, Anthropic, Ollama, Mock
│   └── tests/
│       └── test_core.py
│
├── ros2_lingua/               # ROS 2 interface layer
│   └── ros2_lingua/
│       ├── grounding_node.py  # Central grounding service node
│       ├── dispatcher_node.py # Action/service dispatcher
│       └── capability_mixin.py
│
└── examples/
    └── humanoid_demo/
        └── humanoid_demo.py   # Full demo, no ROS required
```

---

## Roadmap

- [ ] C++ capability advertisement bindings (for C++ controller nodes)
- [ ] Voice input integration (Whisper → instruction string)
- [ ] Web UI for live capability inspection and instruction sending
- [ ] Feasibility checker using real-time joint state feedback

---

## License

Apache 2.0
