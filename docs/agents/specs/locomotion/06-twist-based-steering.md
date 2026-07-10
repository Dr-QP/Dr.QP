# Spec 06: Twist-based steering

- **Status**: proposed
- **Fixes**: F7 (high), F16, F22 (partial)
- **Depends on**: 02 (pure targets), 03 (workspace checks), 04 (analytic solver / clamping)
- **Packages**: `drqp_brain`
- **Size**: L

## Objective

Replace the weighted position-mixing of translation and rotation with foot targets derived
from a commanded planar body twist `ξ = (v_x, v_y, ω_z)`, so that translation and rotation
compose exactly, each leg gets its geometrically correct stride, reachability is enforced by
scaling one scalar, and stride-limit clamping covers combined motion.

## Current behavior

`packages/runtime/drqp_brain/drqp_brain/walk_controller.py`:

- Translation: nominal x-stride rotated by a direction transform built from the (L1-normalized)
  joystick direction.
- Rotation: neutral foot positions rotated about body-z by
  `rotation_speed_degrees · rotation_direction · gait_offsets.x`.
- Combination: `foot_target = stride_target · w_s + rotation_target · w_r` with
  `w_s + w_r = 1` — commanding rotation attenuates translation and vice versa; the two inputs
  have different units (F7).
- Stride magnitude uses `|x| + |y|` (L1) in the controller and in
  `DirectionalStrideLimits.clamp_direction` — diagonal commands get ~41% more stride before
  clipping (F16).
- `stride_limits.yaml` certifies translation only; combined stride+rotation relies on runtime
  IK failure as the safety net (F22).

## Target design

### Twist command

- Map `MovementCommand` to a twist:
  `v = stride_direction.xy · step_length / T_stance` (m/s),
  `ω = rotation_speed · ω_max` (rad/s), where `ω_max` replaces `rotation_speed_degrees`
  (keep the default equivalent: 45°/step ⇒ `ω_max = radians(45) / cycle_time · duty` — derive
  and document the exact equivalence so default turning rate is preserved).
- Smoothing/slew from spec 02 applies to the twist components (`SteeringState` becomes the
  smoothed twist).

### Per-leg stride from the twist

For each leg `i` with neutral foot position `r_i` (the existing `leg_tips_on_ground` entry) and
gait offset parameter `s(phase) ∈ [−½, +½]` (the existing parametric x-offset — swing sweeps
−½→+½, stance +½→−½):

```text
Δθ = ω · T_stance                        # yaw per stride
Δp = v · T_stance                        # body-frame translation per stride (2D)

# body pose along the stride, from the SE(2) exponential map Exp(s · ξ̂ · T_stance):
θ(s) = s · Δθ
p(s) = V(θ(s)) · s · Δp                  # V(θ) = [[sin θ, −(1 − cos θ)],
                                         #         [1 − cos θ,     sin θ]] / θ ;  V(0) = I
                                         # (use the Taylor series near θ = 0)

# stance foot target in the body frame, keeping the foot world-fixed at r_i
# (body pose at s = 0 taken as identity):
foot_i(s) = R_z(−θ(s)) · (r_i − p(s))    # foot_i(0) = r_i
z: unchanged — swing height added exactly as today
```

- This is the exact SE(2) body-twist trajectory: under simultaneous translation and rotation
  the body's translation is coupled to yaw through `V(θ)`. Independently adding `s·Δp` to a
  rotated foot (`R_z(s·Δθ)·r_i + s·Δp`) is only a first-order approximation and does **not**
  keep a stance foot world-fixed — it must not be used. Pure rotation falls out as the
  `Δp = 0` case, pure translation as the `Δθ = 0` case (`V → I`). **Delete** the direction
  transform, the separate rotation transform, and the mixing weights.
- Stance keeps the foot exactly on this path (that is what makes the body motion consistent);
  swing retraces it with the spec 05 profile applied to `s` and z.

### Saturation: one scalar

- For a proposed twist, sample each leg's **full path** `foot_i(s)` over `s ∈ [−½, +½]` and
  check reachability of every sample with `LegModel.solve_ik` (spec 03) — plus the certified
  polar table as a cheap precheck if kept. Endpoint checks alone are insufficient: the
  workspace has an inner-annulus bound and joint-limit cuts, so an interior point of the path
  can be unreachable while both endpoints solve.
- If any leg is unreachable, scale the _twist_ by a single factor `k ∈ (0, 1]` (binary search
  or closed-form against the workspace boundary) so all legs fit. This replaces
  `DirectionalStrideLimits.clamp_direction` L1 logic and covers combined stride+rotation and
  body-pose offsets exactly (F22 for the runtime path).
- Keep `stride_limits.yaml` + generator as offline/CI certification (per spec 04); the runtime
  check subsumes it for clamping purposes. Feed `clamped_legs`/persistent saturation from
  spec 04 into the same `k`.
- All norms Euclidean (F16).

### Odometry (cheap win, optional but recommended)

Integrate the committed twist per tick and publish `nav_msgs/Odometry` (open-loop,
`odom → base` TF optional/parameter-gated). Mark covariance high; this is dead reckoning.

## Behavior changes

- Combined stride+rotation no longer attenuates translation; joystick response is isotropic.
  Full-deflection diagonal speed changes (it was L1-inflated before) — retune
  `step_length`/`ω_max` defaults if teleop feel regresses.
- `rotation_speed_degrees` parameter is replaced (provide a deprecation mapping in the node).

## Out of scope

- Gait sequencer / graceful stop (F17) — separate follow-up.
- Ground-speed-matched touchdown refinement.
- Contact/terrain anything.

## Test plan (write first)

- **Pure translation equivalence**: `Δθ = 0` reproduces today's straight-line foot paths
  (within tolerance) for the same `step_length` — regression against golden traces.
- **Pure rotation equivalence**: `Δp = 0` reproduces today's turn-in-place arcs at the derived
  `ω_max` default.
- **Composition exactness**: for combined `v, ω`, each stance foot stays fixed in the _world_
  frame implied by the twist: `R_z(θ(s))·foot_i(s) + p(s) = r_i` for densely sampled `s`, where
  the body pose `(θ(s), p(s))` is obtained by **numerically integrating the twist** (small-step
  ODE), not by reusing the implementation's closed-form `V(θ)` — otherwise the test only proves
  the implementation agrees with itself, not that the path is the SE(2) exponential.
- **Isotropy**: command magnitude → stride magnitude is independent of direction (sweep 16
  directions, Euclidean).
- **Saturation scalar**: an excessive twist is scaled so every sampled target along each leg's
  full path solves with `reachable=True` (include a case where endpoints are reachable but an
  interior path point is not); scaling preserves direction (`v`, `ω` scaled together).
- **Angular-distance regression**: keep/extend the existing combined stride+rotation
  quantitative test (issue #397 lineage) — it should get _stronger_ guarantees under the twist
  formulation.
- Launch test: combined stride+rotation walking in Gazebo with no clamp warnings at default
  command magnitudes.

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain
```

Gazebo teleop: straight, strafe, turn-in-place, and arcs in all three gaits; verify no
translation slowdown while turning and no IK/clamp warnings at stick extremes.

## Acceptance criteria

- [ ] Foot targets derived from the SE(2) exponential map
      (`foot_i(s) = R_z(−θ(s))·(r_i − p(s))` with `p(s) = V(θ(s))·s·Δp`); mixing weights,
      direction transform, and per-leg rotation transform deleted.
- [ ] Euclidean norms throughout; isotropy test passes.
- [ ] Single-scalar twist saturation using analytic reachability over the full sampled
      stance/swing path; combined motion never relies on IK failure as the limiter.
- [ ] Pure-translation and pure-rotation regressions match previous behavior at defaults.
- [ ] Deprecation path for `rotation_speed_degrees`.
- [ ] All unit + launch tests pass.
