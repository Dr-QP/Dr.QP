---
type: codebase
description: The brain's pluggable IK backends — analytic per-leg IK with MoveIt self-collision validation (default) and full MoveItPy IK — behind one LocomotionKinematics protocol.
source: packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py
source_digest: sha256:87c98d4c1d86c545283884fde72a1af75e19cbddc1c1c7c820f77605c47a6fef
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py
---

# locomotion_kinematics

This module turns a window of foot targets into controller joint targets.
`HexapodBrain` picks the backend from `kinematics_backend`. It also defines the
shared loop constants: `WALKING_TRAJECTORY_POINTS = 2` and
`DEFAULT_CONTROL_RATE_HZ = 25.0`.

## Public surface

- The `LocomotionKinematics` protocol: `ready()`,
  `solve(legs_and_targets, latest_joint_state)`, and
  `controller_joint_names(leg)`.
- `LocomotionKinematicsResult`, with these fields:
  - `joint_targets`
  - `robot_state`
  - `failure_reason`
  - `backend_name`
  - `validated`
  - `clamped_legs`
- `AnalyticLocomotionKinematics(node, hexapod, state_validator)`. Also provides
  `unreachable_legs(...)`, which the brain uses to bound balance corrections.
- `MoveItPyLocomotionKinematics(node, hexapod, is_shutting_down, control_rate_hz)`.
- `MoveItPyStateValidator.validate_joint_targets(joint_targets)`.

## How it works

**Analytic backend (the default):**

1. Solve each leg with `LegModel.solve_ik(clamp=True)`. A leg that is out of the
   workspace or out of limits is clamped and reported in `clamped_legs`, not
   failed.
2. Convert the angles to URDF convention (`model_to_urdf_angles`).
3. Pass the full 18-joint state to `MoveItPyStateValidator`, which rejects it
   only on a planning-scene self-collision.

It is not `ready()` until the URDF joint limits are installed from the node's
`robot_description` parameter.

**MoveIt backend:**

1. Seed a `RobotState` from `/joint_states`. It needs a current joint state.
2. Call `set_from_ik` per leg group, retrying once from the SRDF home pose.
3. Fail the whole tick if any leg fails, then run bounds and collision checks.

The per-call timeout splits half the loop period across
`6 legs × 2 points × 2 attempts`, with a floor of 1 ms.

## Depends on

- [drqp_kinematics](../drqp_kinematics.md): `LegModel.solve_ik` and
  `urdf_limits`.
- [drqp_moveit](../drqp_moveit.md): the SRDF groups and kinematics, through
  `moveit_py`.

## Invariants & gotchas

- The analytic backend asserts that all targets are finite.
- The analytic backend never fails for reachability; persistent clamping only
  shows up as the brain's `locomotion_clamping_persistent` event (see
  [Bug: clamping diagnostic is rejected by the state machine](../../../../bugs/clamping-diagnostic-rejected-by-state-machine.md)).
- MoveIt poses are expressed in `drqp/base_center_link`.
- MoveItPy is created lazily on first use (`_ensure_moveit_py`). The first tick
  after startup pays that cost.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:40` —
  `WALKING_TRAJECTORY_POINTS`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:51` — MoveIt
  timeout budget
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:116` —
  `LocomotionKinematics` protocol
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:137` — MoveIt
  backend
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:208` —
  home-pose retry
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:535` —
  collision check
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:564` —
  `validate_joint_targets`
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:594` —
  analytic backend
- `packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py:686` — URDF
  limit install
