---
type: codebase
description: The project's custom ROS message definitions — MovementCommand, RobotCommand, HapticEffect and their constants.
source: packages/runtime/drqp_interfaces
source_digest: sha256:b5be1ac8722efccc3b6dd146e93aae63b137fbe85e00d84f2bbc86e74fe718ac
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_interfaces
---

# drqp_interfaces

A `rosidl` message-only package. It holds the semantic command vocabulary shared
by every input source (joystick translator, keyboard GUI, MCP server) and the
brain, plus the haptic effect message for the game controller.

## Public surface

- `MovementCommand`:
  - `stride_direction` (Vector3)
  - `rotation_speed` (float32)
  - `body_translation` (Vector3)
  - `body_rotation` (Vector3)
  - `gait_type` (string)
- `MovementCommandConstants`: `GAIT_TRIPOD`, `GAIT_RIPPLE`, `GAIT_WAVE`.
- `RobotCommand`: a `header` and a `command` string.
- `RobotCommandConstants`: `REBOOT_SERVOS`.
- `HapticEffect`: an action (`PLAY`, `STOP`, `STOP_ALL`) and an effect type that
  maps to SDL haptic types. The type-specific fields are rumble magnitudes,
  constant level, periodic wave, and ramp.

## Invariants & gotchas

- Lifecycle events do not use `RobotCommand`: `/robot_event` and `/robot_state`
  are plain `std_msgs/String`, using the event and state names of the robot
  state machine.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_interfaces/msg/MovementCommand.msg` — semantic motion
  command
- `packages/runtime/drqp_interfaces/msg/MovementCommandConstants.msg` — gait
  names
- `packages/runtime/drqp_interfaces/msg/HapticEffect.msg` — SDL haptic effect
- `packages/runtime/drqp_interfaces/msg/RobotCommandConstants.msg` —
  `REBOOT_SERVOS`
