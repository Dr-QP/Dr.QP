---
name: create-state-machine-cpp
description: Create a flat, enum-based C++ finite-state machine for robotics behavior, with explicit events, transition validation, and unit tests. Use for a compact C++ behavior controller; use ROS lifecycle management or a dedicated state-machine framework when lifecycle or hierarchical states are required.
---

# Create a C++ State Machine

Implement a small, flat finite-state machine: one active enum state, explicit
events, and a transition table. This skill does not implement hierarchical,
orthogonal, or history states, and it does not make an rclcpp node a ROS 2
lifecycle node. Route lifecycle-node behavior to
[ros2-lifecycle-management](../ros2-lifecycle-management/); choose a dedicated
state-machine framework when nesting or concurrent regions are requirements.

## Gather inputs

Require the target package name, machine name, states, initial state, events,
and valid transitions. Require an explicit decision on any guard and on the
action to take for an invalid event. Verify the package exists, or create it
first with [create-ros2-package-cpp](../create-ros2-package-cpp/).

Use valid C++ identifiers for generated enum values. Do not silently invent
domain transitions: present a proposed table when the caller has not supplied
one and obtain confirmation before implementation.

## Implement the flat machine

Place public code in `include/<package>/<machine_name>_sm.h` and implementation
in `src/<machine_name>_sm.cpp`. Keep the public API small:

```cpp
enum class State { kIdle, kWalking, kStopped };
enum class Event { kStart, kStop, kFault };

class StateMachine {
public:
  explicit StateMachine(State initial_state);
  [[nodiscard]] State state() const noexcept;
  bool processEvent(Event event);
};
```

Represent allowed `(state, event) -> state` transitions in one table or one
`switch` that is straightforward to audit. `processEvent` must return `false`
without changing state when an event is invalid or a guard fails. Invoke exit
actions before state assignment and entry actions after it; keep action
failures observable through the caller's chosen error policy.

Only add an rclcpp adapter when requested. Keep it separate from the state
machine: subscribe for events, publish state changes, and avoid embedding ROS
ownership or spinning rules in the reusable class.

## Test and integrate

Add tests for the initial state, every allowed transition, invalid events,
guard rejection, and entry/exit action order where applicable. Use
[add-test-file-cpp](../add-test-file-cpp/) for the unit-test target and CMake
integration. If validating behavior by starting nodes or a launch file, use
[launch-testing](../launch-testing/) instead.

Add the implementation to the package library target and ensure the public
header is installed with the package. Build and test through the targeted
commands in the linked test/build skills; all ROS and colcon commands must use
`scripts/with-ros-env.sh`.

## Related resources

- [create-ros2-package-cpp](../create-ros2-package-cpp/)
- [add-test-file-cpp](../add-test-file-cpp/)
- [launch-testing](../launch-testing/)
- [ros2-lifecycle-management](../ros2-lifecycle-management/)
