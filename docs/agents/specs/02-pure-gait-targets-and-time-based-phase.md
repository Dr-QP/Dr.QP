# Spec 02: Pure gait targets & time-based phase

- **Status**: proposed
- **Fixes**: F2 (high), F15, F27, F28
- **Depends on**: nothing (01 should land first as a hotfix)
- **Packages**: `drqp_brain`
- **Size**: M

## Objective

Make foot-target generation a pure function of explicit state so that (a) the gait phase
advances exactly once per control tick, by measured wall time; (b) the trajectory lookahead
window can be evaluated without side effects; (c) command smoothing is expressed as a time
constant independent of tick rate.

## Current behavior

- `WalkController.next_step_targets()` mutates `current_phase`, `current_direction`, and
  `current_rotation_direction` on every call
  (`packages/runtime/drqp_brain/drqp_brain/walk_controller.py`).
- `HexapodBrain.loop()` calls it once for "now" and `_build_walking_feet_target_window()` calls
  it again to fill the 2-point trajectory window (`brain_node.py`), so each successful tick
  advances the phase by **two** steps while ticks arrive at 1× fps. Actual cycle time is half
  what `phase_steps_per_cycle` implies, and walking speed silently changes if
  `walking_trajectory_points` changes.
- Phase advances by a fixed `phase_step` per call regardless of real elapsed time; a delayed
  tick slows the robot.
- Steering smoothing is `interpolate(target, 0.3)` per tick — the effective time constant is
  welded to fps.
- `_snapshot_motion_state()` / `_restore_motion_state()` and `_foot_targets_window_key()` exist
  to undo/deduplicate the side effects.
- `WalkController(phase_steps_per_cycle=self.fps / 2.5)` in `setup_hexapod()` is dead — the
  value is overwritten every tick from `self.phase_steps_per_cycle[self.gait_index]`.

## Target design

### WalkController API

```python
class WalkController:
    # pure: no attribute mutation, safe to call for any phase
    def targets_at(self, phase: float, steering: SteeringState) -> list[tuple[LegModel, Point3D]]: ...

    # the only state transition; called once per control tick
    def advance(self, dt: float, stride_direction: Point3D, rotation_direction: float) -> None:
        # smooth commands with alpha = 1 - exp(-dt / tau)
        # phase += dt / cycle_time  (cycle_time per current gait)
```

- `SteeringState` is a small frozen dataclass holding the smoothed direction and rotation (what
  is now `current_direction` / `current_rotation_direction`).
- Body pose (`body_direction`, `body_rotation`) is passed into `targets_at` explicitly rather
  than applied to the shared hexapod as a side effect; applying targets to the `HexapodModel`
  stays a separate explicit call (`apply_feet_targets`), as today.
- Configuration moves from "steps per cycle" to `cycle_time_sec` per gait (equivalent, but
  states the intent and removes the fps coupling). Keep a translation shim if tests rely on the
  old parameter name.
- Smoothing parameter becomes `steering_tau_sec` (choose the value that reproduces today's
  effective response at 8 Hz with alpha 0.3: `tau = -dt / ln(1 - 0.3) ≈ 0.35 s`).

### Brain loop

```python
def loop(self):
    dt = measured since previous tick (node clock), clamped to [0, 2 / fps]
    self.walker.advance(dt, stride_direction, rotation_direction)
    window = [
        self.walker.targets_at(self.walker.current_phase + k * (1 / self.fps) / cycle_time, steering)
        for k in range(self.walking_trajectory_points)
    ]
    # solve + publish as today
```

- No snapshot/restore: on IK failure or "nothing changed", simply do not publish — the walker
  state was never speculatively mutated. Delete `_snapshot_motion_state`,
  `_restore_motion_state`.
- `_foot_targets_window_key` dedupe: replace with a direct comparison of the committed inputs
  (phase, steering state, body pose, gait). If any changed since the last publish, publish.
- Remove the dead `phase_steps_per_cycle=self.fps / 2.5` constructor argument (F27).

### Speed calibration

Preserving *observed* robot speed is required: today's code advances 2 phase steps per tick, so
when the double-advance is removed, halve the effective cycle time to compensate. Concretely:
current `phase_steps_per_cycle = [20, 25, 40]` at 8 fps with double-advance ⇒ observed cycle
times `[1.25, 1.5625, 2.5]` s. Set `cycle_time_sec = {tripod: 1.25, ripple: 1.5625, wave: 2.5}`
and assert them in tests.

## Out of scope

- Twist steering (spec 06) — keep the existing mixing math inside `targets_at` for now, just
  side-effect free.
- Raising fps (spec 08).

## Test plan (write first)

- `targets_at` purity: calling twice with the same arguments returns equal targets and leaves
  all walker attributes untouched (compare `__dict__` snapshots).
- Phase advance: after `advance(dt)`, `current_phase` increased by exactly `dt / cycle_time`;
  loop-level test with a fake clock asserting one phase step per tick regardless of
  `walking_trajectory_points` ∈ {1, 2, 4}.
- Observed cycle time: walking at full stride for one simulated cycle returns each foot to its
  starting offset after `cycle_time_sec` (per gait) — this is the regression test for the
  speed-calibration table above.
- Smoothing: `advance` twice with dt and once with 2·dt produce (approximately) the same
  smoothed steering; alpha formula asserted directly.
- Brain-node test: IK failure tick publishes nothing and does not move the phase backwards
  (replaces the snapshot/restore tests in `test_brain_node.py`).
- Update existing `test_walk_controller.py` ramping/data-driven assertions to the new API.

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain
```

Gazebo `test_bringup_launch.py` + manual/scripted teleop: tripod cycle time ≈ 1.25 s (count
steps over 10 s), no visible speed change versus the previous build.

## Acceptance criteria

- [ ] `targets_at` is pure; `advance` is the only mutator; snapshot/restore deleted.
- [ ] Phase advances once per tick by measured `dt`; window evaluation has no side effects.
- [ ] Walking speed is independent of `walking_trajectory_points` (test-enforced).
- [ ] Observed per-gait cycle times match the calibration table (test-enforced).
- [ ] Steering smoothing expressed as `steering_tau_sec`; response equivalent at 8 Hz.
- [ ] Dead constructor parameter removed; dedupe key replaced by input comparison.
- [ ] All `drqp_brain` unit + launch tests pass.
