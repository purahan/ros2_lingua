# Contributing to ros2_lingua

Thanks for taking the time to contribute! This document covers everything you need to get the dev environment running, follow the project conventions, and open a pull request that passes CI.

---

## Table of Contents

- [Development Setup](#development-setup)
- [Running Tests](#running-tests)
- [Linting](#linting)
- [C++ Contributions](#c-contributions)
- [Before Submitting a PR](#before-submitting-a-pr)
- [Areas That Need Help](#areas-that-need-help)

---

## Development Setup

```bash
# 1. Clone and enter the repo
git clone https://github.com/purahan/ros2_lingua.git
cd ros2_lingua

# 2. Create a colcon workspace and copy all packages in
mkdir -p ~/ros2_lingua_ws/src
cp -r ros2_lingua_core   ~/ros2_lingua_ws/src/
cp -r ros2_lingua        ~/ros2_lingua_ws/src/
cp -r ros2_lingua_interfaces ~/ros2_lingua_ws/src/
cp -r ros2_lingua_mock   ~/ros2_lingua_ws/src/
cp -r ros2_lingua_cpp    ~/ros2_lingua_ws/src/

# 3. Build
cd ~/ros2_lingua_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

For faster iteration on Python-only changes, use `--symlink-install` so edits are picked up without rebuilding:

```bash
colcon build --symlink-install
```

---

## Running Tests

### Python unit tests (no ROS 2 required)

```bash
cd ros2_lingua_core
python3 -m pytest tests/test_core.py -v
```

All 41 tests run in ~0.2 s with no nodes, no DDS, and no LLM needed. These must pass before any PR is merged.

### Integration tests (ROS 2 + Ollama required)

Make sure Ollama is running with a model pulled (`ollama pull llama3.1`), then:

```bash
cd ~/ros2_lingua_ws
colcon test --packages-select ros2_lingua ros2_lingua_mock
colcon test-result --verbose
```

The 16 integration tests cover the full ROS 2 pipeline — grounding node, dispatcher node, state updates, multi-robot namespaces, and mock robot stack behaviour.

### C++ tests

```bash
cd ~/ros2_lingua_ws
colcon test --packages-select ros2_lingua_cpp
colcon test-result --verbose
```

C++ tests use `ament_cmake_gtest`. New C++ contributions should include gtest coverage of the added functionality.

---

## Linting

This project uses [Ruff](https://docs.astral.sh/ruff/) for Python linting and formatting. Configuration lives in `ruff.toml` at the repo root.

```bash
# Install Ruff (if not already installed)
pip install ruff --break-system-packages

# Check for issues
ruff check .

# Auto-fix what can be fixed
ruff check --fix .

# Check formatting
ruff format --check .

# Apply formatting
ruff format .
```

CI runs `ruff check` and `ruff format --check` on every push and pull request. PRs with lint failures will not be merged.

For C++, `ament_lint_auto` handles `cpplint` and `ament_cppcheck` as part of `colcon test`. Make sure both pass before opening a C++ PR.

---

## C++ Contributions

`ros2_lingua_cpp` is a **header-only** package. All logic lives in `include/ros2_lingua_cpp/lingua_mixin.hpp`. The build system is `ament_cmake`.

Key conventions:
- Keep the package header-only. Do not introduce `.cpp` source files — the entire value of this package is zero compilation overhead for downstream consumers.
- Follow the ROS 2 C++ style guide (enforced by `ament_cpplint` and `ament_cppcheck`).
- Any new field added to `LinguaCapability` must have a matching field in the Python `Capability` dataclass in `ros2_lingua_core/schema.py`, and vice versa. The two must remain in sync.
- Test with `ament_cmake_gtest`. Place tests under `ros2_lingua_cpp/test/`.

To build only the C++ package:

```bash
cd ~/ros2_lingua_ws
colcon build --packages-select ros2_lingua_cpp ros2_lingua_interfaces
colcon test  --packages-select ros2_lingua_cpp
```

---

## Before Submitting a PR

- [ ] All 41 Python unit tests pass (`pytest tests/test_core.py -v`)
- [ ] All integration tests pass (if your change touches the ROS 2 layer)
- [ ] `ruff check .` returns no errors
- [ ] `ruff format --check .` returns no errors
- [ ] C++ changes: `ament_cpplint` and `ament_cppcheck` pass
- [ ] New capabilities include `preconditions`, `postconditions`, and `tags`
- [ ] New Python features have pytest coverage in `ros2_lingua_core/tests/`
- [ ] New C++ features have gtest coverage in `ros2_lingua_cpp/test/`
- [ ] If you added a field to `LinguaCapability` (C++) or `Capability` (Python), both sides are updated

---

## Areas That Need Help

New to the project? Check the 
[good first issue label](https://github.com/purahan/ros2_lingua/labels/good%20first%20issue) 
for self-contained tasks that don't require deep familiarity with the codebase.

### Good First Issues
See the [issues page](https://github.com/purahan/ros2_lingua/issues) for 
the current list — typically small CLI/docs/mock-node additions.


### Claiming an Issue
Found an issue you'd like to work on? Comment `/assign-me` on it and 
you'll be assigned automatically — no need to wait for a maintainer.

- `/assign-me` — assign yourself to the issue
- `/unassign-me` — release the issue if you can no longer work on it

If an issue is assigned but shows no activity for a while, it will be 
automatically unassigned so others can pick it up.


### Larger Contributions
- **Voice input** — Whisper integration for spoken natural language instructions
- **Capability versioning** — detect and handle schema drift when nodes are updated independently
- **Parameter validation in the dispatcher** — reject plans with malformed or out-of-range parameter values before they are sent to the robot
- **Testing on real robots** — validated integrations with nav2, MoveIt 2, and other common ROS 2 stacks are very welcome; open an issue to coordinate

If you're using `ros2_lingua` on a real robot, we'd especially love to hear 
about it — open an issue and tell us what you're building.
