# Contributing to ros2_lingua

## Development setup
```bash
cd ~/ros2_lingua_ws
colcon build && source install/setup.bash
```

## Running tests
```bash
# Unit tests (no ROS required)
cd ros2_lingua_core && python3 -m pytest tests/test_core.py -v

# Integration tests (Ollama must be running)
colcon test --packages-select ros2_lingua
```

## Before submitting a PR
- All 41 unit tests must pass
- New capabilities should include preconditions, postconditions, and tags
- New features need tests

## Areas that need help
- C++ LinguaMixin bindings
- Parameter validation in the dispatcher
- Error recovery planner
- Testing on real robots (nav2, MoveIt 2, etc.)
