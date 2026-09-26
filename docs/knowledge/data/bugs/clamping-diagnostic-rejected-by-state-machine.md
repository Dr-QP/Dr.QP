---
type: bug
description: The brain publishes its locomotion_clamping_persistent diagnostic on /robot_event, where drqp_robot_state rejects it as an invalid transition and logs an error.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: node
  resource: packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py
---

# Bug: clamping diagnostic is rejected by the state machine

## Symptom

When a leg's IK target stays clamped for 8 consecutive ticks, the brain
publishes `locomotion_clamping_persistent:<legs>` on `/robot_event`. Every such
message also reaches `drqp_robot_state`, which logs
`Failed to process event locomotion_clamping_persistent:…: Can't … when in …` at
ERROR level. The lifecycle state is unaffected.

## Reproduction

Confirmed with `python-statemachine` 3.2.1 on 2026-09-26:

``` python
RobotStateMachine().send('locomotion_clamping_persistent:left_front')
# statemachine.exceptions.TransitionNotAllowed
```

On the robot or in simulation, walk into a workspace limit long enough to
trigger persistent clamping. The `drqp_robot_state` log shows the error.

## Root cause

`/robot_event` has two roles: lifecycle commands for the state machine and
diagnostic events for observers. The Gazebo test support and
`test_brain_moveit_ik.py` read the diagnostic from `/robot_event`.
`RobotStateMachine` has no such event, and the node sends every string to
`send()`.

## Fix

Not decided. The options:

- **Separate topic.** Move diagnostics to a dedicated topic (for example
  `/robot/diagnostics`, or standard `diagnostic_msgs`) and update the two test
  consumers.
- **Filter in the node.** Have `drqp_robot_state` ignore strings that are not
  machine events. This is cheaper, but keeps the overloaded topic.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:647` — diagnostic
  published on `/robot_event`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:41` —
  `CLAMPING_EVENT_TICKS = 8`
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py:68` —
  rejected event logged as error
- `packages/simulation/drqp_gazebo/test/robot_control_test_support.py:344` —
  test consumer
- `packages/runtime/drqp_brain/test/test_brain_moveit_ik.py:204` — unit-test
  consumer
