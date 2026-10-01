---
type: codebase
description: Pure SE(2) gait-target generation — maps a normalized command to a metric body twist, smooths and saturates it, advances gait phase by wall time, and returns world-grounded foot targets.
source:
- packages/runtime/drqp_brain/drqp_brain/walk_controller.py
- packages/runtime/drqp_brain/drqp_brain/parametric_gait_generator.py
source_digest: sha256:f2214e3cbe840eae8d9a1f79a520b61c27e11bd7a250dd1edc9231b59d1f2c64
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/walk_controller.py
---

# walk_controller

"Pure SE(2) gait-target generation and time-based walking state management." It
comes with `parametric_gait_generator.py`, which defines the three gaits.
`HexapodBrain` owns one `WalkController`. The notebooks and
`generate_stride_limits.py` use it too.

## Public surface

- `WalkController(hexapod, step_length, step_height, rotation_speed_degrees, cycle_time_sec, gait, steering_tau_sec=0.35, omega_max_rad_sec=None)`.
- `advance(dt, stride_direction, rotation_direction)`: the only state
  transition.
- `targets_at(phase, steering)`: returns `[(leg, Point3D)]` with no side
  effects.
- `command_to_twist(...)`, `body_transform(translation, rotation)`,
  `apply_feet_targets(...)` (notebook path), and `reset()`.
- Settable properties: `current_gait` and `cycle_time_sec`. Readable:
  `current_phase` and `steering`.
- `SteeringState` and `TwistSaturation`: frozen dataclasses.
- `GaitType` (`ripple`, `wave`, `tripod`) and `ParametricGaitGenerator`, which
  returns per-leg phase offsets.

## How it works

1. **Twist.** `command_to_twist` caps the stride direction at unit norm and
   scales it by `step_length / stance_duration`. It clamps rotation to `[-1, 1]`
   and scales it by `omega_max_rad_sec`. When that is not configured, it is
   derived from `rotation_speed_degrees / stance_duration`.
2. **Smoothing.** `advance` blends the twist toward the target with
   `α = 1 − exp(−dt/τ)`.
3. **Saturation.** `advance` bisects a single scale factor (20 steps) until the
   whole swing and stance path, sampled at 49 phases with analytic
   `solve_ik(clamp=False)`, is reachable for every leg.
4. **Rest snap.** Speeds under 1e-3 snap to zero.
5. **Phase.** The phase advances by `dt / cycle_time_sec` only while there is
   motion.

`targets_at` places each foot at `Exp(s · twist · T_stance)` applied to its
neutral ground position, where `s` is the gait's stride offset. It then lifts
the foot by the gait's `z` offset times `step_height`.

The gait definitions:

| Gait   | Swing duration   | Leg offsets            |
| ------ | ---------------- | ---------------------- |
| wave   | 1/6 of the cycle | legs staggered by 1/6  |
| ripple | 1/3              | legs staggered by 1/6  |
| tripod | 1/2              | two tripods, 1/2 apart |

## Depends on

- [drqp_kinematics](../drqp_kinematics.md) (`Point3D`, `AffineTransform`,
  `LegModel.solve_ik`)

## Invariants & gotchas

- The foot targets are world-grounded. The body pose is applied separately by
  the caller through `hexapod.body_transform`, and `targets_at` deliberately
  ignores the body arguments.
- Saturation scales the linear and angular parts together, so a turn-while-walk
  command keeps its curvature.
- A negative `dt` raises. Rationale: see
  [Time-based gait timing](../../../../architecture/time-based-gait-timing.md).

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:67` —
  `WalkController`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:121` —
  `omega_max_rad_sec` derivation
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:149` —
  `command_to_twist`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:169` — `advance`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:201` — `targets_at`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:279` —
  `_saturate_twist`
- `packages/runtime/drqp_brain/drqp_brain/walk_controller.py:298` —
  `_twist_is_reachable`
- `packages/runtime/drqp_brain/drqp_brain/parametric_gait_generator.py:74` —
  gait phase parameters
- `packages/runtime/drqp_brain/drqp_brain/parametric_gait_generator.py:112` —
  `get_offsets_at_phase_for_leg`
