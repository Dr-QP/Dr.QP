---
name: create-state-machine-cpp
description: 'Create C++ state machine implementation with enum-based states, transitions, events, and lifecycle management for robotics behavior. Use when implementing robot behavior control or finite state machines in C++, state transition tables, or ROS 2 lifecycle integration for rclcpp nodes. Keywords: C++ state machine, enum State, process_event, transitions, FSM.'
---

# Create C++ State Machine

Generate a C++ state machine implementation with enum-based states, transitions, events, and lifecycle management following robotics behavior patterns.

For Python state machines (the default in this project), use [create-state-machine-python](../create-state-machine-python/) instead.

## When to Use This Skill

- Implementing robot behavior control in C++
- Creating finite state machines for system modes
- Need structured state transitions with guards and actions
- Building hierarchical state machines or reactive control systems

## Prerequisites

- Understand required states and their behavior
- Know valid transitions between states
- Identified events that trigger transitions
- C++ package exists or will be created

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

`src/<machine_name>_sm.cpp`, `include/<package>/<machine_name>_sm.hpp`.

### Step 3: Design Transition Table

Generate default transitions. Document: states, transitions, events.

### Step 4: Generate Implementation

Header: enum State, enum Event, `to_string()` helpers, `StateMachine` class with `process_event()`, callbacks. Implementation: `setup_transitions()`, `execute_transition()`, entry/exit callbacks.

### Step 5: ROS 2 Integration (Optional)

Node wrapper: publishers for state, subscribers for events, timer for state publishing. Topics: `/<machine_name>/state`, `/<machine_name>/event`.

### Step 6: Test File

Test initial state, valid transition, invalid transition, state sequence, entry/exit callbacks. See [add-test-file-cpp](../add-test-file-cpp/).

### Step 7: State Diagram

Add Mermaid diagram to README: `stateDiagram-v2`, states, transitions.

### Step 8: Update Build

Include new files in CMakeLists.txt.

## Edge Cases

- Circular transitions: Allow state to self
- Multiple transitions: Same state to multiple targets by event
- Invalid events: Handle gracefully
- Guard failures: Log when guard blocks

## Related Resources

- [create-state-machine-python](../create-state-machine-python/)
- [create-ros2-package-cpp](../create-ros2-package-cpp/), [add-test-file-cpp](../add-test-file-cpp/)
