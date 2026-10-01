---
type: codebase
description: The ros2_control hardware-interface plugin for the A1-16 servos, plus the robot URDF, controller configuration, and control launch files.
source: packages/runtime/drqp_control
source_digest: sha256:16070b5682456df991acc9bc84c216fea6f03297ed029ca28a7e24082dfbd966
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_control
---

# drqp_control

"ROS 2 hardware interface plugin for the Dr.QP hexapod robot, bridging
ros2_control to the A1-16 servo driver." It also owns the robot description,
which covers:

- `urdf/*.xacro`, with a `use_gazebo` switch that selects
  `gz_ros2_control/GazeboSimSystem` in place of the real plugin.
- The meshes and the RViz config.
- The controller manager setup: `joint_state_broadcaster`,
  `battery_state_broadcaster`, and `joint_trajectory_controller` over 18 joints
  named `drqp/<side>_<front|middle|back>_<coxa|femur|tibia>`.

## Contains

[urdf](drqp_control/urdf.md)

## Public surface

- The plugin `drqp_control/a1_16_hardware_interface`, a
  `hardware_interface::SystemInterface`.
- Per-joint `position` and `effort` command interfaces, and `position` and
  `velocity` state interfaces. `battery_voltage` is a state interface.
- `launch/ros2_controller.launch.py` (args `use_gazebo`, `show_control_rviz`),
  which also includes `rsp.launch.py`. `all.launch.py` gives an RViz-only view.
- The `control` executable: an offline CLI (`neutral`, `off`/`relax`, `read`)
  that talks to the servos directly over `drqp_serial`.
- The xacro arg `hardware_device_address` (default `/dev/ttySC0`).

## How it works

- **`on_init`** builds a `RobotConfig` from the URDF joint parameters
  (`servo_id`, `inverted`, `offset_rads`, `max_torque`, min/max,
  `initial_position_rads`). It then opens the serial device, or `MockServo` when
  the address is `mock_servo`.

- **`on_configure`** reads each servo and writes its PWM and position limits to
  RAM. It then seeds the position commands from the current positions.

- **`read`** polls one servo per cycle, round-robin, and only while that servo's
  torque is not on.

- **`write`** turns each joint's effort command into a servo mode:

  - effort < 0: reboot the servo.
  - effort < 0.1: torque off.
  - otherwise: position control, with the servo turned on if needed.

  All servos then go out in one broadcast I-JOG packet. Its playtime is the
  hardware period passed to `write`.

## Depends on

- [drqp_a1_16_driver](drqp_a1_16_driver.md)
- ros2_control, `ros2_controllers`, `pluginlib`, `yaml-cpp`.

## Invariants & gotchas

- Effort is the lifecycle channel. The brain sends effort 0 for torque-off and
  −1 then 0 for a servo reboot; the hardware interface decodes it as above.
  Changing the thresholds breaks the robot state machine.
- While torque is on, the joint position state is set from the commanded value,
  converted back through the joint–servo mapping, not from a servo read.
- The controller manager runs at 1000 Hz, and the trajectory controller and the
  joint-state broadcaster at 100 Hz. The hardware block is declared with
  `rw_rate="20"` and `is_async="true"` (see [urdf](drqp_control/urdf.md)), so
  the servo bus is read and written at 20 Hz on its own thread.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_control/include/drqp_control/a1_16_hardware_interface.h:41`
  — plugin class
- `packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp:66` —
  `on_init`
- `packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp:101` —
  `mock_servo` switch
- `packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp:144` —
  `on_configure`
- `packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp:251` —
  round-robin `read`
- `packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp:290` — effort
  → reboot / torque-off
- `packages/runtime/drqp_control/include/drqp_control/RobotConfig.h:56` —
  `jointToServo`
- `packages/runtime/drqp_control/config/drqp_controllers.yml:3` — controller
  manager rate
- `packages/runtime/drqp_control/urdf/ros2_control.urdf.xacro:80` — Gazebo vs
  hardware plugin
- `packages/runtime/drqp_control/src/Control.cpp:104` — `control` CLI
