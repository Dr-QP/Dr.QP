---
type: codebase
description: How a lifecycle event (stand up, sit down, kill, reboot) travels from an input source through the state machine to the brain's trajectories and the servo effort channel.
source: packages/runtime
source_digest: sha256:693b7ca67afe3a62f168b00a176142189821fadb2c1c5070b1d45fb93b00bf5a
verified:
  by: claude-code/opus-5.5
  at: 2026-09-27T21:00:00Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime
---

# flow-robot-lifecycle

Standing up is the running example: `torque_off` → `initializing` → `torque_on`.
The spec is [Robot lifecycle](../spec/robot-lifecycle.md).

## Trace

1. **Event.** An input publishes a `std_msgs/String` event on `/robot_event`.
   The sources are:

   - The translator: PS or touchpad for `kill_switch_pressed`, Start for
     `reboot_servos`, Select for `finalize`.
   - The keyboard GUI.
   - The MCP `robot.boot` tool, which sends `initialize`.

   ([drqp_brain](packages/runtime/drqp_brain.md),
   `joystick_translator_node.py:74`)

2. **Transition.** `drqp_robot_state` sends the event to `RobotStateMachine`. An
   invalid event is logged and dropped.
   ([robot_state](packages/runtime/drqp_brain/robot_state.md),
   `robot_state_node.py:68`)

3. **Publish state.** `on_enter_state` publishes the new state on the
   transient-local `/robot_state`. (`robot_state_node.py:58`)

4. **Brain reaction.** `HexapodBrain.process_robot_state` stops the walk loop
   and plays the stand-up sequence, five points over 3.2 s, through the
   `FollowJointTrajectory` action. Femur, tibia, and coxa are enabled in stages
   through `joint_mask`. (`brain_node.py:885`, `brain_node.py:801`)

5. **Effort channel.** Each trajectory point carries an effort value. The
   hardware interface turns it into a servo mode: torque-off below 0.1, reboot
   below 0, position control otherwise.
   ([drqp_control](packages/runtime/drqp_control.md),
   `a1_16_hardware_interface.cpp:290`)

6. **Completion.** The action result callback publishes `initializing_done` on
   `/robot_event`. The state machine moves to `torque_on`, and the brain resets
   and starts the loop timer (see
   [flow-teleop-command](flow-teleop-command.md)). (`brain_node.py:918`)

## Failure modes

- **Action server absent** (controllers not loaded): `publish_action` waits 5 s,
  logs "Timed out waiting for trajectory action server", and returns without
  emitting `initializing_done`. A rejected goal logs "Goal rejected" and does
  the same. Either way, the state stays `initializing` until a kill or
  `turn_off`. (`joint_trajectory_builder.py:105`)
- **Kill switch during `servos_rebooting`:** refused by the state machine.
- **Robot state node restarted:** it starts again in `torque_off`. The brain
  sees the new latched state and turns torque off.
- **Duplicate stacks on one ROS domain:** blocked by the `drqp_brain` node-name
  check and the domain-scoped `InstanceGuard` lock.
