---
type: codebase
description: The ROS 2 packages that run on the robot — servo transport and driver, ros2_control plugin, brain, kinematics, MoveIt config, joystick node — plus their shared test helpers.
source: packages/runtime
source_digest: sha256:55f1edbcb2cbbacc78f40695ebc350a31671bae0d520fe28eabef1a7ee7d268d
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime
---

# packages/runtime

The colcon packages that make up the robot stack. The deploy image installs
their dependencies with rosdep and builds `--packages-up-to drqp_brain`.
Everything the robot needs to stand, walk, and stop safely lives here.
Simulation-only packages live in `packages/simulation`.

## Contains

[drqp_serial](runtime/drqp_serial.md)

[drqp_a1_16_driver](runtime/drqp_a1_16_driver.md)

[drqp_control](runtime/drqp_control.md)

[drqp_interfaces](runtime/drqp_interfaces.md)

[drqp_joy](runtime/drqp_joy.md)

[drqp_brain](runtime/drqp_brain.md)

[drqp_kinematics](runtime/drqp_kinematics.md)

[drqp_moveit](runtime/drqp_moveit.md)

[drqp_launch_testing](runtime/drqp_launch_testing.md)

[drqp_lint_common](runtime/drqp_lint_common.md)

- `drqp_rapidjson`: a repackaged copy of Tencent RapidJSON, used by
  `drqp_serial` for serial recordings. It is vendored code and *not mapped*.

## How it works

The layering runs bottom-up:

1. `drqp_serial` provides UART and TCP byte streams.
2. `drqp_a1_16_driver` speaks the A1-16 servo protocol over them.
3. `drqp_control` exposes the servos to ros2_control.
4. `drqp_brain` turns semantic commands into joint trajectories for the
   `joint_trajectory_controller`.

`drqp_kinematics` supplies the leg geometry and analytic IK. `drqp_moveit`
supplies the planning scene that the brain uses for collision checks.

## Depends on

- ROS 2 Jazzy (`rclcpp`, `rclpy`, ros2_control, MoveIt 2).
- `packages/vendor` (`sdl3_vendor` for `drqp_joy`, and the patched
  `launch_pytest`).

## Invariants & gotchas

- The deploy build runs rosdep only over `packages/runtime` and
  `packages/vendor`. A runtime package must not depend on a
  `packages/simulation` package.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `docker/ros/deploy/ros-deploy.Dockerfile:33` — `DEPLOY_PACKAGE="drqp_brain"`
- `docker/ros/deploy/ros-deploy.Dockerfile:38` — rosdep `--from-paths` runtime
  and vendor
