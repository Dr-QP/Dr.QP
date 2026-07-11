---
name: create-state-machine-python
description: 'Create Python state machine implementation with python-statemachine states, transitions, events, and lifecycle management for robotics behavior. Use when implementing robot behavior control or finite state machines in Python, python-statemachine usage, state transitions, or ROS 2 lifecycle integration for rclpy nodes. Keywords: python-statemachine, State, transitions, on_enter, process_event, FSM.'
---

# Create Python State Machine

Generate a Python state machine implementation using the `python-statemachine` library with states, transitions, events, and lifecycle management following robotics behavior patterns.

For C++ state machines, use [create-state-machine-cpp](../create-state-machine-cpp/) instead.

## When to Use This Skill

- Implementing robot behavior control in Python
- Creating finite state machines for system modes
- Need structured state transitions with guards and actions
- Building hierarchical state machines or reactive control systems

## Prerequisites

- Understand required states and their behavior
- Know valid transitions between states
- Identified events that trigger transitions
- Python package exists or will be created

## Inputs

- **Machine Name**: e.g., `robot_behavior`, `motion_control`
- **States**: Comma-separated (e.g., `idle,walking,turning,stopped`)
- **Initial State**: Must be in states list
- **Transitions**, **Events**: Optional, generate common ones
- **ROS 2 Integration**: yes/no for lifecycle node wrapper

## Workflow

### Step 1: Validate Inputs

At least 2 states. State names: valid identifiers (lowercase, underscores). Initial state in list.

### Step 2: Determine Package and Location

`<package>/<package>/<machine_name>_sm.py`.

### Step 3: Design Transition Table

Generate default transitions. Document: states, transitions, events.

### Step 4: Generate Implementation (python-statemachine)

Use `statemachine` library. Define `State` (initial=True for initial), `to()` for transitions. `on_enter_<state>`, `on_exit_<state>`, `before_<transition>`, `on_<transition>`. `process_event()` maps events to transitions. Reference: `packages/runtime/drqp_brain/`.

### Step 5: ROS 2 Integration (Optional)

Node wrapper: publishers for state, subscribers for events, timer for state publishing. Topics: `/<machine_name>/state`, `/<machine_name>/event`.

### Step 6: Test File

Test initial state, valid transition, invalid transition, state sequence, entry/exit callbacks. See [add-test-file-python](../add-test-file-python/).

### Step 7: State Diagram

Add Mermaid diagram to README: `stateDiagram-v2`, states, transitions.

### Step 8: Update Build

Include new files in setup.py.

## Edge Cases

- Circular transitions: Allow state to self
- Multiple transitions: Same state to multiple targets by event
- Invalid events: Handle gracefully
- Guard failures: Log when guard blocks

## Related Resources

- [python-statemachine](https://python-statemachine.readthedocs.io/)
- Example: `packages/runtime/drqp_brain/`
- [create-state-machine-cpp](../create-state-machine-cpp/)
- [create-ros2-package-python](../create-ros2-package-python/), [add-test-file-python](../add-test-file-python/)
