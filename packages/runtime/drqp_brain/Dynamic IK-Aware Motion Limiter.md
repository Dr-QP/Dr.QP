# Dynamic IK-Aware Motion Limiter

## Summary
- Replace static stride-only clamping as the primary safety mechanism with an online feasibility limiter.
- Treat precomputed stride limits as optional hints, not guarantees.
- For each movement command, evaluate the actual current configuration: body translation, body rotation, body height, gait phase, stride, rotation gait motion, step height, and trajectory window.
- Preserve body pose first, then reduce walking stride/yaw motion until the generated foot targets are IK-valid. If zero walking still fails, reduce body translation/rotation before declaring the pose infeasible.

## Key Changes
- Add a `MotionFeasibilityLimiter` used by `HexapodBrain` before publishing a walking trajectory.
- The limiter proposes the next gait window, checks it with the existing MoveItPy IK path, and if it fails, binary-searches a scalar applied to walking motion:
  - scale planar `stride_direction`
  - scale `rotation_speed`
  - keep `body_translation`, `body_rotation`, and `step_height` unchanged
- If walking scale reaches zero and IK still fails, binary-search a scalar applied to `body_translation` and `body_rotation`.
- If neutral body pose with zero walking also fails, classify the current pose itself as infeasible and skip publishing with a clear warning.
- Keep `WalkController` responsible for generating targets, but let the brain own feasibility search because it already has MoveIt IK and trajectory-window context.
- Preserve gait continuity by snapshotting/restoring walker and body state during candidate checks, then applying only the accepted candidate.

## Behavior
- Normal case: requested command is IK-valid, no scaling.
- Near limit: stride/rotation are reduced dynamically for the current body pose and phase.
- Body pose too aggressive: robot reduces body translation/rotation toward neutral; if neutral body pose still fails, it holds/restores previous motion state and logs that body pose is infeasible even with zero stride.
- Existing directional stride YAML can remain as a cheap first-pass clamp, but MoveIt feasibility becomes authoritative.

## Diagnostics
- Log accepted scale when limiting occurs, for example:
  `IK limiter scaled movement to walking=0.73, body=1.00 for gait=tripod phase=0.81`
- On failure at zero stride, log body translation, body rotation, gait, phase, and failed leg/reason when available.
- Optionally expose the current limiter scale as a diagnostic/status field later.

## Test Plan
- Unit-test limiter with a fake IK predicate:
  - accepts full command when feasible
  - binary-searches down to the largest feasible stride scale
  - keeps body pose unchanged
  - scales body pose when zero walking motion is still infeasible
  - returns failure when neutral body pose with zero walking is still infeasible
- Add `WalkController`/brain tests confirming failed probes do not permanently advance gait phase or mutate body transform.
- Add MoveItPy-backed smoke coverage for the known `y=0.9` case with body pose variants if practical.

## Assumptions
- Priority policy is body-first: preserve body translation/rotation/height before walking stride, then reduce body pose only after walking motion reaches zero.
- MoveIt/KDL remains authoritative.
- URDF joint limits remain authoritative.
- Static/generated stride tables are optimization hints, not the final safety boundary.
