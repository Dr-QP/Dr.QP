---
type: codebase
description: MoveIt 2 configuration for the hexapod — SRDF with per-leg groups, KDL kinematics, joint limits, controllers, and move_group/demo launch files.
source: packages/runtime/drqp_moveit
source_digest: sha256:db5bed25f71d7ea9a20329a4a22f403ab31e37d4f56b14b534f75baf04d1888a
verified:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
stale_after: 2027-01-10
generated:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
sources:
- id: code
  resource: packages/runtime/drqp_moveit
---

# drqp_moveit

"MoveIt 2 configuration for the Dr.QP hexapod robot." It is a config-only CMake
package.

## Public surface

- `config/drqp.srdf`: one planning group per leg (`left_front_leg`, …) and the
  disabled collision pairs.
- `config/kinematics.yaml`: `kdl_kinematics_plugin/KDLKinematicsPlugin` per leg,
  position-only.
- `joint_limits.yaml`, `move_group.yaml`, `moveit_controllers.yaml`, and
  `ompl_planning.yaml`.
- Launch files:
  - `move_group.launch.py`
  - `demo.launch.py`
  - `demo_gazebo.launch.py`
  - `moveit_rviz.launch.py`
- `moveit_launch_utils.py`: builds the parameters from the `drqp_control` xacro,
  using `use_gazebo` and `hardware_device_address`.

## How it works

The brain loads the same description and semantic parameters in-process through
`moveit_py`. Its bringup calls the helpers in `moveit_launch_utils`. The
planning scene is used for self-collision validation. It is also the IK solver
when `kinematics_backend=moveit`.

## Depends on

- [drqp_control](drqp_control.md) (URDF xacro)
- `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`

## Invariants & gotchas

- `bringup.launch.py` imports `moveit_launch_utils.get_moveit_params` by adding
  this package's installed `launch/` directory to `sys.path`. Renaming or moving
  that module breaks the robot bringup.
- The launch smoke tests (`test/test_*_launch_smoke.py`) verify clean process
  exits with [drqp_launch_testing](drqp_launch_testing.md).

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_moveit/launch/moveit_launch_utils.py:32` —
  `get_description_params`
- `packages/runtime/drqp_moveit/launch/moveit_launch_utils.py:71` —
  `get_move_group_params`
- `packages/runtime/drqp_moveit/config/kinematics.yaml:11` — KDL solver per leg
- `packages/runtime/drqp_moveit/config/drqp.srdf:27` — first leg planning group
