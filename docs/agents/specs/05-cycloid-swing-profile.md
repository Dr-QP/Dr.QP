# Spec 05: Cycloid swing profile

- **Status**: proposed
- **Fixes**: F6, F13
- **Depends on**: 02 (soft — landing after it avoids double test churn; the math is independent)
- **Packages**: `drqp_brain`
- **Size**: S

## Objective

Replace the linear-x / sine-z swing with a cycloid profile so the foot has zero velocity at
liftoff and touchdown, eliminating touchdown impact and liftoff scuffing.

## Current behavior

`packages/runtime/drqp_brain/drqp_brain/parametric_gait_generator.py`,
`get_offsets_at_phase_for_leg`:

- Swing: `x` linearly interpolated `−½ → +½` step; `z = sin(π t) · step_height`.
- `dz/dt` at `t = 1` is `−π · step_height / T_swing` — the foot lands at its **peak** descent
  rate; `dx/dt` is discontinuous at both swing boundaries (instant reversal between stance and
  swing speeds).
- Phase wraparound uses `leg_phase %= 1.000001` (F13).
- Stance is linear `+½ → −½` — this is *correct* (constant body velocity) and must not change.

## Target design

Swing, for normalized swing time `t ∈ [0, 1]` (half-step `h = step_length / 2`):

```text
x(t) = −h + step_length · (t − sin(2πt) / 2π)
z(t) = step_height · (1 − cos(2πt)) / 2
```

Properties (assert these in tests, they are the point of the change):

- `x(0) = −h`, `x(1) = +h`; `x′(0) = x′(1) = 0`
- `z(0) = z(1) = 0`; `z′(0) = z′(1) = 0`; apex `z(½) = step_height`
- `x` strictly monotonic on (0, 1)

Notes:

- Keep the generator parametric (unit step scaled downstream) — the offline stride-limit sweep
  and the walk controller both rely on that contract.
- Zero *foot* velocity at the boundaries in the gait-offset frame means the touchdown velocity
  mismatch with stance is bounded by the stance speed itself (previously stance + peak swing
  speed); full ground-speed matching is a later refinement on top of twist steering (spec 06),
  not this spec.
- Replace `leg_phase %= 1.000001` with `math.fmod(leg_phase, 1.0)` and an explicit half-open
  `[0, 1)` convention comment (F13).
- The swing/stance split and the per-gait phase-offset tables do not change.

## Behavior changes

Foot flight path changes shape (same endpoints, same apex height, same duration). Expect
visibly smoother touchdowns in Gazebo; stride-limit certification is unaffected in x-extent but
**regenerate `stride_limits.yaml`** anyway since intermediate swing positions differ (the sweep
solves every phase sample).

## Out of scope

- Duty-factor changes (tripod overlap margin, F12) — that belongs to the gait sequencer work.
- Bézier/obstacle-clearance shaping; cycloid is sufficient and dependency-free.

## Test plan (write first)

Extend `test_parametric_gait_generator.py`:

- Boundary values: `x(0) = −h`, `x(1) = +h`, `z(0) = z(1) = 0`, `z(½) = step_height`
  (numerically, at the phase samples nearest the boundaries).
- Zero boundary velocity: finite-difference `x` and `z` at the first/last 1% of swing; assert
  the derivative magnitude is < 5% of the mid-swing derivative.
- Monotonic `x` over swing; stance unchanged (regression-pin the stance samples to the current
  linear values).
- Continuity across the swing↔stance boundary: position difference between last swing sample
  and first stance sample → 0 as sampling density grows.
- Wraparound: phases 0.0, 1.0, 1.0 + ε map consistently under the new `fmod` convention.
- Update any golden traces in `test_walk_controller.py` that encode sine-profile z values.

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain
scripts/with-ros-env.sh ros2 run drqp_brain generate_stride_limits   # regenerate + commit YAML
```

Gazebo bringup: walk each gait; feet should land without bounce (visual check; if the sim
exposes contact forces, compare peak touchdown force before/after).

## Acceptance criteria

- [ ] Cycloid x/z with test-enforced zero boundary velocities and preserved endpoints/apex.
- [ ] Stance math untouched (regression-pinned).
- [ ] `fmod`-based wraparound; epsilon hack removed.
- [ ] `stride_limits.yaml` regenerated and committed.
- [ ] All `drqp_brain` tests pass.
