---
type: codebase
description: The xacro robot description — body, six legs, IMU and camera frames, per-joint servo mapping for ros2_control, and the Gazebo plugin block.
source: packages/runtime/drqp_control/urdf
source_digest: sha256:dad8ec097440364e955c10888f6db19ab4d38cbb848faec3b23a4615c8d37f40
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_control/urdf
---

# urdf

`drqp.urdf.xacro` is the root. RViz, `robot_state_publisher`, the controller
manager, MoveIt, Gazebo, and the brain's joint-limit parsing all read the same
description.

## Public surface

- **Xacro args:**
  - `robot_name` (`drqp`): the prefix of every link and joint.
  - `hardware_device_address` (`/dev/ttySC0`).
  - `use_gazebo` (`false`).
- **Frames:**
  - `ground` → `drqp/base_link` (lowest point, 0.085 m up) →
    `drqp/base_center_link`.
  - `drqp/imu_link`: rpy `π 0 π/2`.
  - `drqp/camera`.
  - Per leg `<prefix>`: `_coxa`, `_femur`, and `_tibia` revolute joints, plus
    `_foot_link`.
- **Legs:** `left_/right_` × `front/middle/back`, at x ±0.11692 or 0, y ±0.06387
  or ±0.103, with yaw ±π/4, ±π/2, ±3π/4.
- **ros2_control per joint:**
  - `servo_id`: the leg's coxa ID, femur +2, tibia +4. The coxa IDs are: left
    front 1, right front 2, left middle 13, right middle 14, left back 7, right
    back 8.
  - `inverted`, which is true for the right legs' femur and tibia.
  - `offset_rads` and `max_torque` (0.25).
  - `min`/`max`: coxa ±90°, femur −98…90°, tibia −80…110°.
  - `battery_state/battery_voltage`, read from servo 1.

## How it works

`ros2_control.urdf.xacro` emits one `<ros2_control>` system:

- With `use_gazebo`: `gz_ros2_control/GazeboSimSystem`, and `gazebo.xacro` adds
  the ros2_control, odometry-publisher, and IMU plugins.
- Otherwise: `drqp_control/a1_16_hardware_interface` with `device_address`,
  115200 baud, `rw_rate="20"`, `is_async="true"`, and `thread_priority="30"`.

## Depends on

- `xacro`, the `meshes/` in [drqp_control](../drqp_control.md)

## Invariants & gotchas

- On hardware, the servo bus is read and written asynchronously at 20 Hz. The
  controller manager runs at 1000 Hz and the trajectory controller at 100 Hz.
  The brain streams at 25 Hz.
- The `imu_link` mount is duplicated in `balance_controller.py` and must change
  together with it.
- The battery `min`/`max` are informational and must match
  `config/drqp_controllers.yml`, where the broadcaster reads them.
- `test/test_urdf.py` only checks that the xacro exists and parses.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_control/urdf/drqp.urdf.xacro:3` — xacro args
- `packages/runtime/drqp_control/urdf/drqp.urdf.xacro:16` — leg placement
- `packages/runtime/drqp_control/urdf/body.urdf.xacro:44` — `base_center_to_imu`
- `packages/runtime/drqp_control/urdf/ros2_control.urdf.xacro:3` — per-leg joint
  macro
- `packages/runtime/drqp_control/urdf/ros2_control.urdf.xacro:53` — servo ID map
- `packages/runtime/drqp_control/urdf/ros2_control.urdf.xacro:86` — async 20 Hz
  hardware block
- `packages/runtime/drqp_control/urdf/gazebo.xacro:3` — Gazebo plugins
