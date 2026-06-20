# ros2_lingua

> **Give your robot the ability to understand natural language — on any ROS 2 platform.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](./ros2_lingua_cpp)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](./LICENSE)
[![CI](https://img.shields.io/github/actions/workflow/status/purahan/ros2_lingua/ci.yml?branch=main&label=CI)](https://github.com/purahan/ros2_lingua/actions)
[![Tests](https://img.shields.io/badge/Tests-41%20unit%20%7C%2016%20integration-brightgreen)](./ros2_lingua_core/tests/)
[![Release](https://img.shields.io/github/v/release/purahan/ros2_lingua)](https://github.com/purahan/ros2_lingua/releases)
[![Issues](https://img.shields.io/github/issues/purahan/ros2_lingua)](https://github.com/purahan/ros2_lingua/issues)

📖 **[Full Documentation](https://purahan.github.io/ros2_lingua/)**

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
- [Capability Schema](#capability-schema)
- [Registry & State](#registry--state)
- [Grounding Engine](#grounding-engine)
- [Backward Chaining](#backward-chaining)
- [Integrating With Your Robot — Python](#integrating-with-your-robot--python)
- [Integrating With Your Robot — C++](#integrating-with-your-robot--c)
- [Multi-Robot Support](#multi-robot-support)
- [LLM Backends](#llm-backends)
- [Error Handling](#error-handling)
- [ROS 2 API Reference](#ros-2-api-reference)
- [Running Tests](#running-tests)
- [Trying It Without a Robot](#trying-it-without-a-robot)
- [Package Structure](#package-structure)
- [Citation](#citation)
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
- **Tag-based filtering** — annotate capabilities with standard tags (`Tags.LOCOMOTION`, `Tags.MANIPULATION`, etc.) and scope grounding to only relevant capability subsets
- **Rich error handling** — 11 specific exception types, all inheriting from `LinguaError`, so you catch exactly what you care about
- **Configurable retries** — `RetryConfig` with exponential back-off for flaky LLM backends or rate-limited APIs
- **Python and C++ support** — Python nodes use `LinguaMixin`; C++ controller nodes use the header-only `ros2_lingua_cpp` package, no compilation overhead
- **Multi-robot namespace support** — run multiple grounding + dispatcher stacks in the same ROS 2 graph, each scoped to its own robot namespace
- **LLM-agnostic** — works with OpenAI, Anthropic Claude, local Ollama models, or any custom backend you implement
- **Robot-agnostic** — works on any ROS 2 robot: mobile manipulators, wheeled AMRs, robotic arms, drones, AUVs, humanoids
- **ROS-free core** — the schema, registry, and grounding engine have zero ROS 2 dependencies and can be unit tested independently (41 tests, run in ~0.2 s)
- **Mock package included** — `ros2_lingua_mock` ships full simulated nodes, a CLI tool, a web dashboard, and a one-command launch file so you can demo and develop without touching a real robot

---

## Architecture

`ros2_lingua` is split into two layers intentionally — the core logic is kept completely free of ROS 2 so it can be tested, extended, and reasoned about independently.

```
┌──────────────────────────────────────────────────────────────┐
│                    ros2_lingua_core                           │
│                  (No ROS 2 dependencies)                      │
│                                                               │
│   Capability / CapabilityParameter / Tags                     │
│   CapabilityRegistry    Backward Chain Planner                │
│   GroundingEngine       ActionPlan / ActionStep               │
│   LLM Backends          RetryConfig    LinguaError hierarchy  │
└──────────────────────────────────────────────────────────────┘
                             ↕
┌──────────────────────────────────────────────────────────────┐
│                      ros2_lingua                              │
│                    (ROS 2 interface layer)                     │
│                                                               │
│   GroundingNode          DispatcherNode                       │
│   LinguaMixin (Python)   ros2_lingua_interfaces               │
└──────────────────────────────────────────────────────────────┘
         ↕                                    ↕
┌────────────────────────┐    ┌───────────────────────────────┐
│   ros2_lingua_cpp       │    │   ros2_lingua_mock (sim)      │
│   (header-only C++)     │    │   BalanceNode  NavNode        │
│   LinguaMixin for C++   │    │   ManipNode    SpeechNode     │
│   rclcpp nodes          │    │   CLI  Dashboard (port 8080)  │
└────────────────────────┘    └───────────────────────────────┘
                             ↕
┌──────────────────────────────────────────────────────────────┐
│                    Your Robot Nodes                           │
│   Python nodes  ·  C++ controller nodes  ·  Any combination  │
└──────────────────────────────────────────────────────────────┘
```

**`ros2_lingua_core`** handles all the intelligence — capability definitions, the registry, the grounding engine, backward chaining, LLM backends, retry logic, and the error hierarchy. Pure Python, no ROS.

**`ros2_lingua`** wraps the core in ROS 2 nodes and services — the `GroundingNode` exposes the engine as a service, the `DispatcherNode` executes plans on your robot, and `LinguaMixin` lets any Python node self-register its capabilities at boot.

**`ros2_lingua_cpp`** is a header-only C++ package that brings the same capability registration to `rclcpp` nodes. Add one `#include`, inherit `LinguaMixin`, and your C++ controller can advertise its capabilities alongside Python nodes — with no runtime overhead.

**`ros2_lingua_mock`** is a fully functional simulation layer — mock robot nodes, a one-line CLI, a live web dashboard, and a launch file that starts the entire system in one command. Use it to explore the API without touching real hardware.

---

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- C++17 (included with Humble's toolchain — needed only for `ros2_lingua_cpp`)
- `rosbridge_suite` (for the web dashboard)
- At least one LLM backend:
  - [Ollama](https://ollama.com/) — recommended for local use, no API key needed
  - OpenAI API key
  - Anthropic API key

---

## Installation

```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_lingua_ws/src
cd ~/ros2_lingua_ws/src

# Clone the repository
git clone https://github.com/purahan/ros2_lingua.git

# Copy packages into the workspace
cp -r ros2_lingua/ros2_lingua_core .
cp -r ros2_lingua/ros2_lingua .
cp -r ros2_lingua/ros2_lingua_interfaces .
cp -r ros2_lingua/ros2_lingua_mock .
cp -r ros2_lingua/ros2_lingua_cpp .       # C++ header-only bindings

# Install rosbridge (needed for the web dashboard)
sudo apt install ros-humble-rosbridge-suite

# Install an LLM backend (pick one or more)
# Option A — Ollama (local, no API key, recommended)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.1

# Option B — OpenAI
pip install openai --break-system-packages

# Option C — Anthropic
pip install anthropic --break-system-packages

# Build
cd ~/ros2_lingua_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Quick Start

The fastest path to a running demo — no real robot required.

**Terminal 1 — rosbridge**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 2 — full demo stack**
```bash
ros2 launch ros2_lingua_mock demo.launch.py
```

**Terminal 3 — send a natural language instruction**
```bash
ros2 run ros2_lingua_mock cli "go to the table and pick up the bottle"
```

Expected output:
```
✅  Plan generated — 3 step(s):

   1.  stabilize_robot        ← auto-chained prerequisite
   2.  navigate_to_location   params: {location_name: "table"}
   3.  pick_up_object         params: {object_name: "bottle"}

🚀  Dispatching to robot...
```

Open **http://localhost:8080** in your browser to see the live mission control dashboard with capability visualization and plan animation.

To use a specific LLM backend with the demo:
```bash
# OpenAI
ros2 launch ros2_lingua_mock demo.launch.py \
    llm_backend:=openai \
    llm_model:=gpt-4o \
    llm_api_key:=sk-...

# Anthropic
ros2 launch ros2_lingua_mock demo.launch.py \
    llm_backend:=anthropic \
    llm_model:=claude-sonnet-4-20250514 \
    llm_api_key:=sk-ant-...
```

### Running only the grounding node (no mock stack)

```bash
# Ollama
ros2 run ros2_lingua grounding_node --ros-args \
    -p llm_backend:=ollama \
    -p llm_model:=llama3.1

# OpenAI
ros2 run ros2_lingua grounding_node --ros-args \
    -p llm_backend:=openai \
    -p llm_model:=gpt-4o \
    -p llm_api_key:=sk-...

# In a second terminal
ros2 run ros2_lingua dispatcher_node
```

### Sending a raw service call

```bash
ros2 service call /lingua/ground \
    ros2_lingua_interfaces/srv/GroundInstruction \
    "{instruction: 'go to the table and pick up the bottle'}"
```

---

## Capability Schema

A `Capability` is the fundamental unit of `ros2_lingua` — the structured description of one thing a node can do.

```python
from ros2_lingua_core import Capability, CapabilityParameter, Tags

navigate = Capability(
    # Identity — used in plans and LLM prompts
    name="navigate_to_location",
    description="Moves the robot to a named location. Known locations: table, door, dock.",

    # ROS 2 interface — exactly one of these
    ros_action="robot/navigate_to_pose",  # or ros_service=

    # Input parameters
    parameters=[
        CapabilityParameter(
            name="location_name",
            type="string",
            description="Where to go, e.g. 'table', 'door', 'charging_dock'",
            required=True,
        ),
    ],

    # Preconditions: must be true before this can run
    preconditions=["robot_is_balanced"],

    # Postconditions: become true after this runs
    postconditions=["robot_at_location"],

    # Tags: for filtering and categorization
    tags=[Tags.LOCOMOTION, Tags.NAVIGATION],
)
```

> **Write `description` for the LLM, not for humans.** These fields are injected directly into the LLM prompt. Be specific. Mention example values and constraints. The more precise the description, the better the grounding quality.

### Standard Tags

| Constant | Value | Use for |
|---|---|---|
| `Tags.LOCOMOTION` | `"locomotion"` | Navigation, driving, walking |
| `Tags.MANIPULATION` | `"manipulation"` | Arms, grippers, pick/place |
| `Tags.BALANCE` | `"balance"` | Stabilization, posture |
| `Tags.SPEECH` | `"speech"` | TTS, STT, voice I/O |
| `Tags.PERCEPTION` | `"perception"` | Cameras, lidar, detection |
| `Tags.SAFETY` | `"safety"` | E-stop, collision avoidance |
| `Tags.INSPECTION` | `"inspection"` | ROVs, drones, industrial |

---

## Registry & State

The `CapabilityRegistry` is the single source of truth — it stores all registered capabilities and the robot's current symbolic state.

```python
from ros2_lingua_core import CapabilityRegistry

registry = CapabilityRegistry()

# --- Registration ---
registry.register(navigate)        # raises DuplicateCapabilityError if already registered
registry.update(navigate)          # register or silently overwrite
registry.unregister("navigate_to_location")

# --- Querying ---
registry.get("navigate_to_location")
registry.get_all()
registry.get_by_tag("locomotion")
registry.get_by_tags(["locomotion", "manipulation"], match="any")   # default
registry.get_by_tags(["manipulation", "social"],    match="all")
registry.get_all_tags()    # sorted unique tag list across all capabilities
registry.get_untagged()    # capabilities with no tags

# --- State management ---
registry.set_state("robot_is_balanced")
registry.clear_state("robot_is_balanced")
registry.get_state()                       # returns current Set[str]
registry.is_satisfied(["robot_is_balanced"])
```

> **Design state token names carefully.** Use specific, unambiguous tokens. Prefer `"robot_is_balanced"` over `"ready"`. Each token should have exactly one capability that produces it to avoid ambiguous dependency resolution.

---

## Grounding Engine

Translates a natural language instruction into an `ActionPlan`. All LLM output is validated against registered capabilities before the plan is returned.

```python
from ros2_lingua_core import GroundingEngine, OllamaBackend, RetryConfig

retry = RetryConfig(
    max_retries=5,
    base_delay_sec=1.0,
    backoff_factor=2.0,
)

engine = GroundingEngine(
    registry=registry,
    backend=OllamaBackend(model="llama3.1", retry=retry),
    auto_chain=True,    # insert prerequisites automatically (default: True)
)

# Ground an instruction
plan = engine.ground("go to the table and pick up the bottle")

# Ground with tag filter — only consider locomotion capabilities
plan = engine.ground("move to position A", tag_filter=["locomotion"])

# Inspect the result
if plan.feasible:
    for step in plan.steps:
        print(step.capability_name, step.parameters)
else:
    print("Not feasible:", plan.reason)
```

> **`ground()` never raises by default.** It catches most errors internally and returns an infeasible `ActionPlan` with a descriptive `reason` string rather than raising. This keeps the grounding node stable under all conditions. Raise-on-error behaviour can be enabled if needed.

### `ActionPlan` fields

| Field | Type | Description |
|---|---|---|
| `steps` | `List[ActionStep]` | Ordered steps to execute |
| `feasible` | `bool` | `False` if the instruction cannot be executed |
| `reason` | `str` | Why it's infeasible (if applicable) |
| `original_instruction` | `str` | The original input string |
| `to_json()` | method | Serialize the plan to a JSON string |

---

## Backward Chaining

The planner automatically resolves capability prerequisites without you hardcoding execution sequences. It works backward from the goal through the dependency graph.

```python
# Given:
#   stabilize_robot  → produces: ["robot_is_balanced"]
#   navigate         → requires: ["robot_is_balanced"],  produces: ["robot_at_location"]
#   pick_up_object   → requires: ["robot_at_location", "arm_is_free"]

# Current robot state: {"arm_is_free"}  ← "robot_is_balanced" NOT yet true

chain = registry.resolve_chain(
    "pick_up_object",
    current_state={"arm_is_free"}
)
# → [stabilize_robot, navigate, pick_up_object]
# stabilize_robot was auto-inserted because navigate required robot_is_balanced
```

When `auto_chain=True` (the default) is set on `GroundingEngine`, this happens transparently every time you call `engine.ground()`.

---

## Integrating With Your Robot — Python

Adding `ros2_lingua` to an existing Python node takes four incremental steps.

### Step 1 — Declare capabilities

```python
# my_robot/capabilities.py
from ros2_lingua_core import Capability, CapabilityParameter, Tags

NAVIGATE = Capability(
    name="navigate_to_location",
    description="Drives the robot to a named location. Known: kitchen, office, dock.",
    ros_action="my_robot/navigate",
    parameters=[CapabilityParameter("location_name", "string", "Destination name")],
    preconditions=["robot_is_ready"],
    postconditions=["robot_at_location"],
    tags=[Tags.LOCOMOTION],
)
```

### Step 2 — Add `LinguaMixin` to your node

```python
from rclpy.node import Node
from ros2_lingua import LinguaMixin
from my_robot.capabilities import NAVIGATE

class MyNavigationNode(LinguaMixin, Node):
    def __init__(self):
        Node.__init__(self, "my_navigation_node")
        LinguaMixin.__init__(self)
        # ... your existing __init__ code ...
        self.register_lingua_capability(NAVIGATE)   # ← this is all it takes
```

### Step 3 — Subclass the dispatcher

```python
from ros2_lingua.dispatcher_node import DispatcherNode
from my_robot_interfaces.action import NavigateTo

class MyRobotDispatcher(DispatcherNode):
    def _call_action(self, action_name, cap_name, params):
        if cap_name == "navigate_to_location":
            client = ActionClient(self, NavigateTo, action_name)
            goal = NavigateTo.Goal()
            goal.location_name = params.get("location_name", "")
            # ... send goal, wait for result ...
            return True
        return super()._call_action(action_name, cap_name, params)
```

### Step 4 — Report state changes

```python
async def _execute_navigate(self, goal_handle):
    result = await self._do_navigation(goal_handle.request)
    if result.success:
        self.update_lingua_state(set_tokens=["robot_at_location"])
        goal_handle.succeed()
    else:
        self.update_lingua_state(clear_tokens=["robot_at_location"])
        goal_handle.abort()
```

---

## Integrating With Your Robot — C++

`ros2_lingua_cpp` is a **header-only** package. There is nothing to compile beyond your own node — just add the dependency and include the header.

### Step 1 — Add the dependency

In your `package.xml`:
```xml
<depend>ros2_lingua_cpp</depend>
```

In your `CMakeLists.txt`:
```cmake
find_package(ros2_lingua_cpp REQUIRED)
ament_target_dependencies(my_node ros2_lingua_cpp ros2_lingua_interfaces)
```

### Step 2 — Inherit `LinguaMixin` and register your capability

```cpp
#include "rclcpp/rclcpp.hpp"
#include "ros2_lingua_cpp/lingua_mixin.hpp"

class MyNavigationNode
    : public rclcpp::Node,
      public ros2_lingua_cpp::LinguaMixin
{
public:
    MyNavigationNode()
        : rclcpp::Node("my_navigation_node"),
          LinguaMixin(this)                     // pass the node pointer
    {
        LinguaCapability cap;
        cap.name           = "navigate_to_location";
        cap.description    = "Drives the robot to a named location. Known: kitchen, office, dock.";
        cap.ros_action     = "my_robot/navigate";
        cap.preconditions  = {"robot_is_ready"};
        cap.postconditions = {"robot_at_location"};
        cap.tags           = {"locomotion"};
        cap.parameters.push_back({
            .name        = "location_name",
            .type        = "string",
            .description = "Destination name",
            .required    = true,
        });

        register_lingua_capability(cap);        // fires the ROS 2 service call
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
```

`LinguaMixin` calls `/lingua/register_capability` on construction and re-registers on reconnect. State updates flow back over `/lingua/update_state`, matching the Python API exactly. See `include/ros2_lingua_cpp/lingua_mixin.hpp` for the full `LinguaCapability` struct definition and all available methods.

---

## Multi-Robot Support

Multiple robots can share a single ROS 2 graph. Each robot runs its own grounding + dispatcher stack under a unique namespace:

```bash
# Robot 1
ros2 launch ros2_lingua_mock demo.launch.py namespace:=robot_1

# Robot 2
ros2 launch ros2_lingua_mock demo.launch.py namespace:=robot_2
```

Each namespace gets its own isolated set of services and topics:

```
/robot_1/lingua/ground
/robot_1/lingua/capabilities
/robot_1/lingua/current_plan

/robot_2/lingua/ground
/robot_2/lingua/capabilities
/robot_2/lingua/current_plan
```

Send instructions per-robot:
```bash
ros2 run ros2_lingua_mock cli --namespace robot_1 "go to the table"
ros2 run ros2_lingua_mock cli --namespace robot_2 "charge at dock"
```

Python and C++ nodes both honour the namespace automatically through the standard ROS 2 node namespace mechanism — no code changes needed in your node.

---

## LLM Backends

`ros2_lingua` is LLM-agnostic. Swap backends without changing any other code.

```python
from ros2_lingua_core import (
    OllamaBackend,
    OpenAIBackend,
    AnthropicBackend,
    MockBackend,
    RetryConfig,
)

# Ollama — local, no API key
backend = OllamaBackend(model="llama3.1")

# OpenAI
backend = OpenAIBackend(api_key="sk-...", model="gpt-4o")

# Anthropic
backend = AnthropicBackend(api_key="sk-ant-...", model="claude-sonnet-4-20250514")

# Mock — for testing and CI/CD, no LLM call made
backend = MockBackend()

# Any backend can receive a RetryConfig
retry = RetryConfig(max_retries=5, base_delay_sec=2.0, backoff_factor=2.0)
backend = OllamaBackend(model="llama3.1", retry=retry)

# Custom backend — implement one method
class MyBackend:
    def complete(self, messages: list[dict]) -> str:
        return my_llm_call(messages)   # return the response string
```

| Backend | API Key | Best for |
|---|---|---|
| `OllamaBackend` | None required | Offline robots, privacy-sensitive |
| `OpenAIBackend` | OpenAI key | Best grounding quality |
| `AnthropicBackend` | Anthropic key | Complex multi-step instructions |
| `MockBackend` | None required | Testing, CI/CD |
| Custom | — | Any other LLM |

---

## Error Handling

11 specific exception types, all inheriting from `LinguaError`. Catch exactly what you care about.

```python
from ros2_lingua_core import (
    LinguaError,                    # base — catch all ros2_lingua errors
    LLMBackendError,                # any LLM connectivity problem
    LLMTimeoutError,                # LLM took too long
    LLMRateLimitError,              # API rate limit hit
    LLMModelNotFoundError,          # model doesn't exist on this backend
    HallucinationError,             # LLM referenced an unknown capability
    UnsatisfiablePreconditionError, # backward chainer could not resolve deps
    StepTimeoutError,               # dispatcher step timed out
    StepFailedError,                # dispatcher step returned failure
)

try:
    plan = engine.ground("pick up the bottle")
except LLMModelNotFoundError as e:
    print(f"Model not found: {e}. Try: ollama pull llama3.1")
except LLMRateLimitError:
    print("Rate limit hit — configure RetryConfig or switch backend")
except LLMBackendError as e:
    print(f"LLM connectivity issue: {e}")
except LinguaError as e:
    print(f"Unexpected ros2_lingua error: {e}")
```

> `GroundingEngine.ground()` catches most errors internally and returns an infeasible `ActionPlan` rather than raising, keeping the grounding node stable. Raise on error explicitly if needed.

---

## ROS 2 API Reference

### Services

| Service | Type | Description |
|---|---|---|
| `/lingua/ground` | `GroundInstruction` | Ground a natural language instruction |
| `/lingua/register_capability` | `RegisterCapability` | Register a capability from any node |
| `/lingua/update_state` | `UpdateState` | Set or clear state tokens |

### Topics

| Topic | Msg Type | Rate | Description |
|---|---|---|---|
| `/lingua/current_plan` | `std_msgs/String` | on change | Latest `ActionPlan` as JSON |
| `/lingua/capabilities` | `std_msgs/String` | 5 Hz | All registered capabilities as JSON |
| `/lingua/execution_status` | `ExecutionStatus` | on change | Step-by-step execution progress |

### Python API Summary

| Name | Type | Package | Description |
|---|---|---|---|
| `Capability` | dataclass | core | Describes one robot capability |
| `CapabilityParameter` | dataclass | core | One input parameter for a capability |
| `Tags` | class | core | Standard tag string constants |
| `CapabilityRegistry` | class | core | Stores capabilities and robot state |
| `GroundingEngine` | class | core | Translates instructions to plans |
| `ActionPlan` | dataclass | core | Ordered, validated execution plan |
| `RetryConfig` | dataclass | core | Controls LLM retry behaviour |
| `LinguaError` | exception | core | Base class for all errors |
| `LinguaMixin` | mixin | ros2_lingua | Self-registration for ROS 2 nodes |
| `DispatcherNode` | Node | ros2_lingua | Executes plans — subclass for your robot |

---

## Running Tests

### Unit tests (no ROS 2 required)

The core library tests have no ROS 2 dependency and run with plain pytest:

```bash
cd ros2_lingua_core
python3 -m pytest tests/test_core.py -v
```

41 tests cover the capability schema, registry, tag filtering, backward chaining planner, grounding engine, error hierarchy, retry logic, and hallucination protection. They run in approximately 0.2 seconds.

### Integration tests (ROS 2 + Ollama required)

```bash
cd ~/ros2_lingua_ws
colcon test --packages-select ros2_lingua
colcon test-result --verbose
```

16 integration tests exercise the full ROS 2 pipeline — grounding node, dispatcher node, state updates, and the mock robot stack.

---

## Trying It Without a Robot

The entire core library runs with zero ROS 2 setup. This is a good starting point to understand how the system works before integrating it with your robot:

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

You can also use the grounding engine directly in a Python script (no ROS 2 process needed):

```python
import sys
sys.path.insert(0, "path/to/ros2_lingua_core")

from ros2_lingua_core import (
    Capability, CapabilityParameter, CapabilityRegistry,
    GroundingEngine, OllamaBackend,
)

registry = CapabilityRegistry()
registry.register(Capability(
    name="navigate",
    description="Moves robot to named location",
    ros_action="robot/navigate",
    parameters=[CapabilityParameter("location", "string", "Destination name")],
))

plan = GroundingEngine(registry, OllamaBackend("llama3.1")).ground("go to the kitchen")
print(plan.to_json())
```

---

## Package Structure

```
ros2_lingua/
│
├── ros2_lingua_core/               # ROS-agnostic core — independently testable
│   ├── ros2_lingua_core/
│   │   ├── __init__.py             # Public API
│   │   ├── schema.py               # Capability, CapabilityParameter, Tags
│   │   ├── registry.py             # CapabilityRegistry + backward chain planner
│   │   ├── grounding.py            # GroundingEngine, ActionPlan, ActionStep
│   │   ├── backends.py             # OpenAI, Anthropic, Ollama, Mock, RetryConfig
│   │   └── errors.py               # LinguaError hierarchy (11 specific types)
│   ├── tests/
│   │   └── test_core.py            # 41 unit tests, no ROS required (~0.2 s)
│   ├── package.xml
│   └── setup.py
│
├── ros2_lingua/                    # ROS 2 interface layer (Python)
│   ├── ros2_lingua/
│   │   ├── grounding_node.py       # Exposes GroundingEngine as a ROS 2 service
│   │   ├── dispatcher_node.py      # Executes ActionPlans over actions/services
│   │   └── capability_mixin.py     # LinguaMixin for self-registering Python nodes
│   ├── package.xml
│   └── setup.py
│
├── ros2_lingua_cpp/                # Header-only C++ bindings
│   ├── include/
│   │   └── ros2_lingua_cpp/
│   │       └── lingua_mixin.hpp    # LinguaMixin + LinguaCapability for rclcpp nodes
│   ├── package.xml
│   └── CMakeLists.txt
│
├── ros2_lingua_interfaces/         # Custom ROS 2 service + message definitions
│   ├── srv/
│   │   ├── RegisterCapability.srv
│   │   ├── GroundInstruction.srv
│   │   └── UpdateState.srv
│   └── msg/
│       └── ExecutionStatus.msg
│
├── ros2_lingua_mock/               # Simulation layer — zero real hardware needed
│   ├── ros2_lingua_mock/
│   │   ├── balance_node.py         # Simulated balance / stabilization controller
│   │   ├── navigation_node.py      # Simulated locomotion node
│   │   ├── manipulation_node.py    # Simulated arm / gripper node
│   │   ├── speech_node.py          # Simulated TTS node
│   │   ├── cli.py                  # CLI: ros2 run ros2_lingua_mock cli "instruction"
│   │   └── dashboard_server.py     # HTTP server — live dashboard on port 8080
│   ├── launch/
│   │   └── demo.launch.py          # Starts everything with one command
│   ├── package.xml
│   └── setup.py
│
├── .github/
│   └── workflows/                  # CI — lint (ruff) + unit + integration tests
├── docs/                           # Source for purahan.github.io/ros2_lingua/
├── examples/
│   └── humanoid_demo/
│       └── humanoid_demo.py        # Runnable demo, zero ROS 2 setup needed
├── ruff.toml                       # Python linting config
├── .gitignore
├── CONTRIBUTING.md
├── LICENSE
└── README.md
```

---

## Citation

If you use `ros2_lingua` in your research or build on it, please cite:

```bibtex
@software{purahan2026ros2lingua,
  author    = {Gupta, Purahan},
  title     = {ros2\_lingua: A Structured LLM Grounding Engine for ROS 2 Robots},
  year      = {2026},
  publisher = {GitHub},
  journal   = {GitHub repository},
  url       = {https://github.com/purahan/ros2_lingua},
  note      = {Apache-2.0 License}
}
```

---

## Roadmap

- [x] Launch files — start the full system with one command
- [x] Mock robot nodes — simulate a real robot for testing and demos
- [x] CLI tool — `ros2 run ros2_lingua_mock cli "your instruction"`
- [x] Capability tagging — filter capabilities by category (locomotion, manipulation, etc.)
- [x] Web dashboard — live visualization of capabilities, state, and execution plans
- [x] C++ bindings — header-only `ros2_lingua_cpp` for capability registration from rclcpp nodes
- [x] Multi-robot namespace support — independent grounding stacks per robot in one graph
- [x] Integration tests — full ROS 2 pipeline testing with `launch_testing`
- [ ] Voice input — Whisper integration for spoken instructions
- [ ] Capability versioning — track schema changes across robot software updates
- [ ] Parameter validation in the dispatcher — reject malformed parameters before dispatch
- [ ] Error recovery planner — automatic replanning on step failure

---

## Contributing

This project is in active early development. Contributions, issues, and feedback are all welcome.

If you're using `ros2_lingua` on your robot, we'd love to hear about it — open an issue and tell us what you're building.

Please read [CONTRIBUTING.md](./CONTRIBUTING.md) before submitting a pull request.

---

## License

Apache 2.0 — see [LICENSE](./LICENSE) for details.
