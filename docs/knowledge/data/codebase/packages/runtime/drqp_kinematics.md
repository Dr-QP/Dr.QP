---
type: codebase
description: Pure-Python hexapod geometry and kinematics — leg and body models, forward kinematics, closed-form leg IK, and URDF joint-limit mapping.
source: packages/runtime/drqp_kinematics
source_digest: sha256:81eb1224d2ede90d12859c69b0d637075ec5fbab6f4f07201e61354dd5192880
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_kinematics
---

# drqp_kinematics

"Reusable hexapod geometry and kinematics models." It has no ROS node, only
`numpy`/`scipy` models that the brain and the Jupyter notebooks import. It is
also installed editable into the workspace `.venv` (`pyproject.toml`).

## Public surface

- `HexapodModel`: six `LegModel`s placed by the front, middle, and side offsets,
  with `forward_kinematics`, `move_legs_to`, and a settable `body_transform`.
- `LegModel.solve_ik(target, clamp=True)` returns a `LegIKSolution`:
  - `angles_rad`
  - `reachable`
  - `within_limits`
  - `clamped_target`
  - `limit_margin_rad`
- `LegModel.fk_foot_position`.
- `urdf_limits`:
  - `parse_joint_limits(urdf_xml)`
  - `model_joint_limits_from_urdf`
  - `model_to_urdf_angles` and `urdf_to_model_angles`
- `geometry`: `Point3D`, `AffineTransform`, `Line3D`, `Leg3D`.

## How it works

`solve_ik` works in the leg's local frame:

1. Yaw comes from `atan2`.
2. The femur–tibia pair is solved as a planar two-link problem. A target span
   outside the reachable annulus is scaled onto the annulus edge.
3. The angles are clamped to the installed joint limits.
4. When the target was unreachable or out of limits, the clamped foot position
   is recomputed by FK.

## Invariants & gotchas

- The model angles and the URDF angles differ by fixed offsets: femur −13.11°,
  tibia −32.9°. Every joint-limit comparison goes through `urdf_limits`.
- The joint limits are unset until someone installs them from
  `robot_description`. The brain's analytic backend reports "not ready" until
  then.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_kinematics/drqp_kinematics/models.py:56` —
  `HexapodModel`
- `packages/runtime/drqp_kinematics/drqp_kinematics/models.py:165` — `LegModel`
- `packages/runtime/drqp_kinematics/drqp_kinematics/models.py:296` — `solve_ik`
- `packages/runtime/drqp_kinematics/drqp_kinematics/models.py:38` —
  `LegIKSolution`
- `packages/runtime/drqp_kinematics/drqp_kinematics/urdf_limits.py:27` —
  model→URDF offsets
- `packages/runtime/drqp_kinematics/drqp_kinematics/urdf_limits.py:83` —
  `parse_joint_limits`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/point.py:104` —
  `Point3D`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/transforms.py:28` —
  `AffineTransform`
