---
type: codebase
description: The robot lifecycle state machine and the drqp_robot_state node that serves it over /robot_event and /robot_state.
source: packages/runtime/drqp_brain/drqp_brain/robot_state
source_digest: sha256:ce4f3f733c08dadc9258d59c63e469dc0e2d08dc5207e3ddbf5310b8973d127f
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/robot_state
---

# robot_state

The robot's lifecycle, as a `python-statemachine` `StateMachine`
(`RobotStateMachine`) wrapped by the `drqp_robot_state` node. The required
behavior is specified in [Robot lifecycle](../../../../spec/robot-lifecycle.md);
this doc records what the code does.

## Public surface

- States:
  - `torque_off` (initial)
  - `initializing`
  - `torque_on`
  - `finalizing`
  - `finalized`
  - `servos_rebooting`
- Events:
  - `initialize`
  - `initializing_done`
  - `turn_off`
  - `finalize`
  - `finalizing_done`
  - `reboot_servos`
  - `servos_rebooting_done`
  - `kill_switch_pressed`, defined as `turn_off | initialize`
- `/robot_event` (`std_msgs/String`, depth 10): the event name to send.
- `/robot_state` (`std_msgs/String`, depth 1, transient-local): the current
  state name.

## How it works

The node sends each received string to the machine. `on_enter_state` publishes
the new state name, so late subscribers get the current state immediately. The
brain (`HexapodBrain.process_robot_state`) reacts to each state:

- `torque_off`: stop the loop and send effort 0.
- `initializing`: stop the loop and play the 3.2 s stand-up trajectory through
  `FollowJointTrajectory`, then emit `initializing_done`.
- `torque_on`: start the loop.
- `finalizing`: play the sit-down trajectory, then emit `finalizing_done`.
- `finalized`: send effort 0.
- `servos_rebooting`: send effort −1 then 0, then emit `servos_rebooting_done`.

## Depends on

- `python-statemachine`, `rclpy`
- `InstanceGuard` in [drqp_brain](../drqp_brain.md)

## Invariants & gotchas

- An event that is not allowed raises `TransitionNotAllowed`. The node catches
  it, logs it, and stays in the current state.
- In `finalized`, both `turn_off` and `initialize` are allowed. The library's
  resolution sends the robot to `initializing`. From the active states,
  `kill_switch_pressed` goes to `torque_off`, and in `servos_rebooting` it is
  refused. `TestKillSwitch` pins all six cases.
- The same event names are hard-coded as strings in the brain, the joystick
  translator, the keyboard GUI, and the MCP server.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_machine.py:25`
  — `RobotStateMachine`
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_machine.py:37`
  — `initialize`
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_machine.py:56`
  — `kill_switch_pressed`
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py:42` —
  transient-local QoS
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py:58` —
  `on_enter_state`
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py:68` —
  rejected event handling
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:885` —
  `process_robot_state`
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:801` — stand-up sequence
- `packages/runtime/drqp_brain/test/test_robot_state_machine.py:146` —
  `TestKillSwitch`
