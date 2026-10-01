---
type: codebase
description: Gazebo (gz-sim) launch files, worlds and bridge config for the simulated robot, and the launch_pytest simulation suite with smoke and slow tiers.
source: packages/simulation/drqp_gazebo
source_digest: sha256:d26fdaec92485cc04e53ba8d05199a10086e487dc4fc1eda61a7a1bbea27d7ee
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-11-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/simulation/drqp_gazebo
---

# drqp_gazebo

The simulation package. `sim.launch.py` starts gz-sim, spawns the `drqp` model
from the `drqp_control` URDF with `use_gazebo`, and bridges the clock, odometry,
and IMU. It then includes the real `drqp_brain` bringup with `use_gazebo:=true`.
Behind this package's `description` (it says "Dr.QP robot URDF") are its launch
files and CI simulation tests.

## Public surface

- `launch/sim.launch.py` arguments:
  - `world_sdf` (`empty.sdf`)
  - `sim_gui`
  - Spawn pose (`robot_*`)
  - `follow_camera` and `follow_camera_delay`
  - `load_keyboard_control`
  - `spawn_robot`, which also gates the bringup
  - Gazebo SIGINT and SIGTERM timeouts
  - `gz_partition`, defaulting to `<HOSTNAME>:<USER>-domain-<ROS_DOMAIN_ID>`
- `balance_challenge.launch.py` and `balance_challenge_disturbance.launch.py`
  with `worlds/balance_challenge.sdf`.
- `config/drqp_gazebo_bridge.yml`: Gazebo-to-ROS bridges for `clock`, `/odom`
  (from `/model/drqp/odometry`), and `/imu/data`.

## How it works

With the robot spawned, bringup runs the stack unchanged. `gz_ros2_control`
replaces the A1-16 plugin, and the bridged IMU replaces `drqp_imu`.

The tests are `launch_pytest` modules registered through
`drqp_add_ros_isolated_launch_test`, each with a 1800 s timeout:

- Smoke tier: spawn, smoke, and balance-board world.
- Slow tier: movement directions, sustained walking, direction reversal,
  posture, and balance-board response. It runs only when `DRQP_TEST_MODE=slow`.

## Depends on

- [drqp_brain](../runtime/drqp_brain.md),
  [drqp_control](../runtime/drqp_control.md)
- [drqp_keyboard_control](drqp_keyboard_control.md) (optional GUI)
- `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`

## Invariants & gotchas

- A launch instance guard (`drqp_gazebo_sim`) and the per-user, per-domain
  `GZ_PARTITION` keep concurrent simulations from colliding.
- CI runs the slow tier only in the devcontainer job matrix entry with
  `sim_test_mode: slow`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/simulation/drqp_gazebo/launch/sim.launch.py:181` — `gz_partition`
  default
- `packages/simulation/drqp_gazebo/launch/sim.launch.py:310` — includes
  `drqp_brain` bringup
- `packages/simulation/drqp_gazebo/launch/sim.launch.py:326` — sim instance
  guard
- `packages/simulation/drqp_gazebo/config/drqp_gazebo_bridge.yml:15` —
  `/imu/data` bridge
- `packages/simulation/drqp_gazebo/test/CMakeLists.txt:20` — smoke tier list
- `packages/simulation/drqp_gazebo/test/CMakeLists.txt:42` —
  `DRQP_TEST_MODE=slow` gate
- `packages/cmake/RosIsolatedLaunchTest.cmake:1` —
  `drqp_add_ros_isolated_launch_test`
