---
type: architecture
description: Why servo torque, torque-off and reboot travel through the ros2_control effort command interface instead of a dedicated service or node.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: hw
  resource: packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: commit
  resource: git 59315ef "Migrate to ros2_control (#225)", 2025-08-10
---

# Effort as the lifecycle channel

## Decision

Each joint's `effort` command interface carries the servo's power mode, not a
torque. The hardware interface decodes the value on every `write`:

| Effort        | Servo action                                          |
| ------------- | ----------------------------------------------------- |
| `< 0`         | Reboot, once, until the effort changes                |
| `0 ≤ e < 0.1` | Torque off, once                                      |
| `≥ 0.1`       | Position control, turning the servo on the first time |

Values are clamped to [−1, 1]. The brain sets effort on the trajectory points it
already sends:

- Torque-off is one point with effort 0.
- A servo reboot is effort −1, then 0 after 1 s.
- The stand-up sequence turns on the femur, then the tibia, then the coxa, by
  masking which joints get non-zero effort.

## Why

This came in with the ros2_control migration (#225). That migration replaced the
hand-written `pose_setter`, `pose_reader`, and `pose_to_joint_state` nodes: "Use
`position` interface for joint position control. Use `effort` interface for
torque on/off/reset control." One `JointTrajectory` or `FollowJointTrajectory`
goal can then express both motion and power sequencing. No second path into the
hardware interface is needed, and the timing is controlled by the trajectory
controller.

## Rejected alternatives

- **The pre-#225 custom nodes.** A separate pose setter and reader talked to the
  driver directly. They were removed in favor of stock `ros2_controllers`.
- *Not recorded:* a ROS service or a GPIO-style command interface for power
  state was considered, if at all, nowhere in the repo.

## Consequences

- The thresholds 0 and 0.1 are a contract between `drqp_brain` and
  `drqp_control`. Changing either side alone breaks the lifecycle (see
  [Robot lifecycle](../spec/robot-lifecycle.md)).
- `effort` cannot be used as a real torque command without redesign.
- In simulation, `gz_ros2_control` receives the same effort values with its own
  semantics. What they do there is *unknown* from the code.
