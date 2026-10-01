---
type: codebase
description: Python high-level control — the walking loop, gaits, IK orchestration, IMU balance, joystick translation, IMU driver, and the bringup launch file.
source: packages/runtime/drqp_brain
source_digest: sha256:730663b428ef725ee8b341888815c449b015c8503f58e58dc2f703114ecb3675
verified:
  by: claude-code/opus-5.5
  at: 2026-09-27T21:00:00Z
stale_after: 2026-11-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_brain
---

# drqp_brain

"IK solvers and other high level control algorithms." It is an `ament_python`
package with four nodes and `launch/bringup.launch.py`, the launch file for the
whole robot stack. It is also the deploy image's target package and default
command.

## Contains

[robot_state](drqp_brain/robot_state.md)

[walk_controller](drqp_brain/walk_controller.md)

[locomotion_kinematics](drqp_brain/locomotion_kinematics.md)

[joystick_translator_node](drqp_brain/joystick_translator_node.md)

[imu_node](drqp_brain/imu_node.md)

[balance_controller](drqp_brain/balance_controller.md)

[joint_trajectory_builder](drqp_brain/joint_trajectory_builder.md)

[instance_guard](drqp_brain/instance_guard.md)

- `timed_queue.py`: `TimedQueue`, which runs delayed actions on a ROS timer.
  Nothing in the workspace uses it apart from its own test. *Not mapped.*
- `launch/bringup.launch.py`: described in this doc.

## Public surface

- Executables:
  - `drqp_brain` (`HexapodBrain`)
  - `drqp_robot_state`
  - `drqp_joystick_translator`
  - `drqp_imu` (BNO055 over I2C, publishing `/imu/data` at 100 Hz)
- `bringup.launch.py` arguments:
  - `use_gazebo`
  - `load_joystick` (default `false`): starts `game_controller_node`
  - `load_joystick_translator` (default `true`)
  - `load_controllers`
  - `load_imu`
  - `hardware_device_address`
  - `kinematics_backend`
- `HexapodBrain` parameters:
  - `control_rate_hz`: default 25, allowed range 5–100.
  - `kinematics_backend`: `analytic` or `moveit`.
  - `enable_imu_balance`, `imu_balance_gain`, `imu_balance_max_tilt_rad`,
    `imu_balance_timeout_sec`.
  - `omega_max_rad_sec`.
  - `rotation_speed_degrees`: deprecated.
- ROS topics and actions: see [api-ros-interface](../../api-ros-interface.md).

## How it works

`HexapodBrain` holds a `HexapodModel` and a `WalkController`. The walk
controller does time-based SE(2) gait-target generation over
`ParametricGaitGenerator`, with the tripod, ripple, and wave cycle times.

Each timer tick (`_run_loop`), only while in `torque_on`:

1. Advance the walker by `dt`, using the latest `MovementCommand`, or a
   stationary IMU posture correction when balance mode is on.
2. Build a `WALKING_TRAJECTORY_POINTS` (2) foot-target window.
3. Solve it through the selected `LocomotionKinematics` backend:
   - `AnalyticLocomotionKinematics`: `LegModel.solve_ik`, with the collision
     check done by `MoveItPyStateValidator`.
   - `MoveItPyLocomotionKinematics`.
4. Publish a `JointTrajectory` whose points are spaced at the control period.

A tick is skipped when the motion-state key has not changed. Lifecycle reactions
live in [robot_state](drqp_brain/robot_state.md).

## Depends on

- [drqp_kinematics](drqp_kinematics.md)
- [drqp_interfaces](drqp_interfaces.md)
- [drqp_moveit](drqp_moveit.md), used by `moveit_py`
- [drqp_control](drqp_control.md), for the controller launch
- [drqp_joy](drqp_joy.md), launched by bringup
- `python-statemachine`, `numpy`, `scipy`
- [drqp_launch_testing](drqp_launch_testing.md), a test-only dependency

## Invariants & gotchas

- Out-of-range `control_rate_hz` or an unknown `kinematics_backend` raises
  `ValueError` at construction.
- Only one `drqp_brain` node may exist per ROS graph (checked by node name).
  Bringup also takes a domain-scoped file lock (`InstanceGuard`).
- The gait cycle times are fixed in seconds and "preserve the observed speed of
  the old double-advance loop at 8 Hz". Changing the rate must not change the
  gait speed.
- The robot runs `game_controller_node` in its own container, so bringup leaves
  `load_joystick` off and starts only the translator by default.
- The IMU node starts only when `use_gazebo` is false; in simulation, Gazebo
  bridges `/imu/data`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:128` — `HexapodBrain`
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:148` — `control_rate_hz`
  parameter
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:209` — time-based gait
  cycle times
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:484` — `_run_loop`
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:653` —
  `_constrain_balance_correction`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:40` —
  `WALKING_TRAJECTORY_POINTS = 2`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:47` —
  `DEFAULT_CONTROL_RATE_HZ = 25.0`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:594` —
  `AnalyticLocomotionKinematics`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:67` —
  `WalkController`
- `packages/runtime/drqp_brain/drqp_brain/balance_controller.py:60` —
  `apply_imu_balance`
- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:129` —
  `make_launch_instance_guard`
- `packages/runtime/drqp_brain/launch/bringup.launch.py:60` — stack instance
  guard
- `packages/runtime/drqp_brain/launch/bringup.launch.py:78` —
  `load_joystick_translator` declaration
