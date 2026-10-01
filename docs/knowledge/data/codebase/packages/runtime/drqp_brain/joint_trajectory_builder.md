---
type: codebase
description: Builds 18-joint JointTrajectory messages point by point, with per-point effort, and sends them as a topic message or a FollowJointTrajectory goal.
source: packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py
source_digest: sha256:7f7bc4535edf362371fe61dec11f4d52babcefe0f3b8560c668b3f095fd1fd06
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py
---

# joint_trajectory_builder

The single place where the brain turns joint targets into
`trajectory_msgs/JointTrajectory`. It covers the walking loop's streamed windows
and the scripted lifecycle sequences.

## Public surface

- `JointTrajectoryBuilder(hexapod)`.
- `add_point_from_joint_targets(targets, reach_in_seconds_from_start, effort=1.0, joint_mask=None)`:
  the targets are controller-convention radians, keyed by `drqp/<leg>_<joint>`.
- `publish(pub)`: publishes a plain topic message.
- `publish_action(action_client, node, result_callback)`: sends a
  `FollowJointTrajectory` goal and calls `result_callback` when the result
  arrives.
- `ShutdownAwareNode`: the protocol a caller must satisfy (`_is_shutting_down`,
  `_track_future`, `get_logger`).

## How it works

Joints are emitted in leg order, then coxa, femur, tibia. A joint missing from
the targets raises `KeyError`. Effort goes on every point. A `joint_mask`
restricts the non-zero effort to the listed joint types, which is how the
stand-up sequence enables the servos in stages (see
[Effort as the lifecycle channel](../../../../architecture/effort-lifecycle-channel.md)).

`publish_action` waits up to 5 s for the action server. The goal and result
futures are tracked on the node, so shutdown can cancel them. Callbacks turn
into no-ops once the node is shutting down.

## Depends on

- [drqp_kinematics](../drqp_kinematics.md) (`HexapodModel` for leg order)
- `control_msgs`, `trajectory_msgs`, `rclpy.action`

## Invariants & gotchas

- On a server timeout or a rejected goal, the builder logs and returns, and
  `result_callback` is never called. A lifecycle `*_done` event then never fires
  (see [flow-robot-lifecycle](../../../flow-robot-lifecycle.md)).
- The default effort is 1.0, which means torque on. Callers must pass `effort=0`
  explicitly for torque-off.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py:47` —
  `JointTrajectoryBuilder`
- `packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py:59` —
  `add_point_from_joint_targets`
- `packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py:90` —
  `publish`
- `packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py:105` — 5 s
  server wait
