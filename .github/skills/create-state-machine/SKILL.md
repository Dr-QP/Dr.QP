---
name: create-state-machine
description: Create state machine implementation with states, transitions, events, and lifecycle management for robotics behavior. Use when implementing robot behavior control, finite state machines, state transitions, python-statemachine, or ROS 2 lifecycle integration.
---

# Create State Machine

Generate state machine implementation with states, transitions, events, and lifecycle management following robotics behavior patterns.

## When to Use This Skill

- Implementing robot behavior control
- Creating finite state machines for system modes
- Need structured state transitions with guards and actions
- Building hierarchical state machines or reactive control systems

## Prerequisites

- Understand required states and their behavior
- Know valid transitions between states
- Identified events that trigger transitions
- Package exists or will be created

## Inputs

- **Machine Name**: e.g., `robot_behavior`, `motion_control`
- **States**: Comma-separated (e.g., `idle,walking,turning,stopped`)
- **Initial State**: Must be in states list
- **Language**: `cpp` or `python` (default: python)
- **Transitions**, **Events**: Optional, generate common ones
- **ROS 2 Integration**: yes/no for lifecycle node wrapper

## Workflow

### Step 1: Validate Inputs

At least 2 states. State names: valid identifiers (lowercase, underscores). Initial state in list.

### Step 2: Determine Package and Location

Python: `<package>/<package>/<machine_name>_sm.py`. C++: `src/<machine_name>_sm.cpp`, `include/<package>/<machine_name>_sm.hpp`.

### Step 3: Design Transition Table

Generate default transitions. Document: states, transitions, events.

### Step 4: Python (python-statemachine)

Use `statemachine` library. Define `State` (initial=True for initial), `to()` for transitions. `on_enter_<state>`, `on_exit_<state>`, `before_<transition>`, `on_<transition>`. `process_event()` maps events to transitions. Reference: `packages/runtime/drqp_brain/`.

### Step 5: C++

Header: enum State, enum Event, `to_string()` helpers, `StateMachine` class with `process_event()`, callbacks. Implementation: `setup_transitions()`, `execute_transition()`, entry/exit callbacks.

### Step 6: ROS 2 Integration (Optional)

Node wrapper: publishers for state, subscribers for events, timer for state publishing. Topics: `/<machine_name>/state`, `/<machine_name>/event`.

### Step 7: Test File

Test initial state, valid transition, invalid transition, state sequence, entry/exit callbacks.

### Step 8: State Diagram

Add Mermaid diagram to README: `stateDiagram-v2`, states, transitions.

### Step 9: Update Build

Include new files in CMakeLists.txt or setup.py.

## Edge Cases

- Circular transitions: Allow state to self
- Multiple transitions: Same state to multiple targets by event
- Invalid events: Handle gracefully
- Guard failures: Log when guard blocks

## Related Resources

- [python-statemachine](https://python-statemachine.readthedocs.io/)
- Example: `packages/runtime/drqp_brain/`
- [create-ros2-package](../create-ros2-package/), [add-test-file](../add-test-file/)
