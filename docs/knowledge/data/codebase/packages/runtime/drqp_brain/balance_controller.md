---
type: codebase
description: IMU tilt math for the stationary balance mode — body roll/pitch from the IMU quaternion and a clipped counter-rotation.
source: packages/runtime/drqp_brain/drqp_brain/balance_controller.py
source_digest: sha256:6071e227a5d951b96b9a851dc0d0f41bec72534408ea3294e0b4e95042fb02e9
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/balance_controller.py
---

# balance_controller

Two pure functions used by `HexapodBrain` for balance mode, the stationary
posture hold specified in [Balance mode](../../../../spec/balance-mode.md).

## Public surface

- `BASE_CENTER_TO_IMU_ROTATION`: the fixed IMU mount, roll π and yaw π/2.
- `body_tilt_from_imu(orientation) -> Point3D(roll, pitch, 0)`.
- `apply_imu_balance(body_rotation, measured_tilt, *, target_body_tilt, gain, max_tilt_rad) -> Point3D`,
  a rotation vector.

## How it works

1. **Body tilt.** Body orientation = IMU orientation × mount⁻¹, from which the
   body roll and pitch are taken.
2. **Correction.** The error (measured − target) is multiplied by the gain and
   clipped to ±`max_tilt_rad` per axis. The result is composed as a negative
   roll/pitch rotation in front of the requested rotation. Yaw is untouched.

In the brain:

- **Arming.** `process_balance_mode` arms only in `torque_on` with a fresh tilt
  reading, and captures that reading as `target_body_tilt`.
- **Each tick.** `_run_loop` zeroes the stride and applies the correction.
- **Reachability.** `_constrain_balance_correction` bisects a scale factor (16
  steps) until every leg is reachable. It logs
  `stationary_balance_correction_saturated` when it has to shrink the
  correction.

## Depends on

- [drqp_kinematics](../drqp_kinematics.md) (`Point3D`),
  `scipy.spatial.transform`

## Invariants & gotchas

- The mount rotation duplicates the URDF joint `base_center_to_imu`
  (`rpy="π 0 π/2"`) on purpose. TF and the message `frame_id` are "unreliable
  for the simulated IMU". The two copies must be edited together.
- The target is the tilt at arming time, not level. The mode holds the posture
  it started in.
- Brain defaults: `imu_balance_gain` 2.0, `imu_balance_max_tilt_rad` 0.15, and
  an IMU timeout of 1 s, after which balance is dropped.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/balance_controller.py:40` —
  `body_tilt_from_imu`
- `packages/runtime/drqp_brain/drqp_brain/balance_controller.py:60` —
  `apply_imu_balance`
- `packages/runtime/drqp_control/urdf/body.urdf.xacro:44` — `base_center_to_imu`
  joint
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:421` — arming in
  `process_balance_mode`
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:653` —
  `_constrain_balance_correction`
