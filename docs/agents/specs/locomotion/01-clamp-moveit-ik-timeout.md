# Spec 01: Clamp MoveIt IK timeout

- **Status**: proposed
- **Fixes**: F1 (critical)
- **Depends on**: nothing — land first
- **Packages**: `drqp_brain`
- **Size**: XS (one constant + tests)

## Objective

Remove the multi-second control-loop stall mode caused by the per-call IK timeout being ~200×
the loop budget.

## Current behavior

`MOVEIT_IK_TIMEOUT_SEC = 2.0` in
`packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py` is passed to every
`robot_state.set_from_ik(...)` call (primary attempt and home-seed retry). A single tick of
`HexapodBrain.loop()` (8 Hz → 125 ms budget) solves up to 6 legs × 2 trajectory-window points,
each with a possible retry: worst case ≈ 24 × 2.0 s of blocking inside the mutually-exclusive
loop callback group. The MoveIt-side configuration
(`drqp_moveit/config/kinematics.yaml`, `kinematics_solver_timeout: 0.05`) is overridden by this
Python constant.

## Target behavior

- The per-call timeout is derived from the loop budget, not hand-picked: with up to
  6 legs × 2 window points × 2 attempts = 24 calls per tick, the budget inequality
  `24 × timeout ≤ 0.5 / fps` gives `timeout ≤ ~2.6 ms` at 8 Hz. (A hand-picked 20 ms would
  allow a 480 ms worst case — far over the 62.5 ms half-period budget.)
- Worst-case IK time per tick (all legs, all window points, all retries) fits within half the
  loop period, leaving budget for validation and publication.
- On timeout the existing failure path (warning + skip tick) is taken — no behavior change other
  than bounded latency.

## Implementation notes

1. Define `MOVEIT_IK_TIMEOUT_SEC` as a derived expression, not a literal:
   `MOVEIT_IK_TIMEOUT_SEC = (0.5 / fps) / (legs × window_points × attempts)` using the same
   constants the loop uses (≈ 2.6 ms at 8 Hz, 6 legs, 2 points, 2 attempts).
2. Add a derived assertion tying the constant to the loop budget:
   `6 legs × 2 points × 2 attempts × timeout ≤ 0.5 / fps`. Prefer a module-level constant
   expression over prose so a future `fps` change trips it.
3. If ~2.6 ms proves too short for KDL to converge reliably at default stride (validate via the
   Gazebo launch test below), do **not** raise the timeout — reduce the worst-case call count
   instead and re-derive: e.g. drop the home-seed retry for the second window point (it is
   phase-adjacent to the first, so the previous solution is already a good seed), giving
   18 calls ≈ 3.5 ms each.
4. Confirm `kinematics.yaml` values (`timeout: 0.05`, `attempts: 3`) still make sense as the
   MoveIt-side defaults for non-loop users (RViz, launch tests); do not change them in this
   spec.

## Behavior changes

Extreme foot targets that previously succeeded only after long numeric searching may now fail
fast and skip the tick. This is acceptable: those targets were already producing multi-second
freezes, and the stride-limit clamp keeps normal walking inside the easily-solvable envelope.

## Out of scope

- Replacing the numeric solver (spec 04).
- Changing retry/rollback semantics (spec 04).

## Test plan (write first)

- Unit test in `test_locomotion_kinematics.py`: assert the timeout constant satisfies the
  budget inequality above for the current `fps` and `walking_trajectory_points` values.
- Unit test: `solve()` passes the constant (not a hardcoded literal) to `set_from_ik` (existing
  fake-based tests likely cover the call signature; extend the fake to record the timeout
  argument).

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain
```

Then run the Gazebo launch test `test_brain_moveit_ik.py` and confirm walking still starts and
no `MoveItPy IK failed` warnings appear during straight-line tripod walking.

## Acceptance criteria

- [ ] `MOVEIT_IK_TIMEOUT_SEC` is derived from the loop budget; worst-case total IK time per
      tick (all calls) ≤ half the loop period.
- [ ] Budget inequality encoded in a test.
- [ ] All existing `drqp_brain` unit and launch tests pass.
- [ ] Straight/diagonal walking in Gazebo shows no IK-failure warnings at default stride.
