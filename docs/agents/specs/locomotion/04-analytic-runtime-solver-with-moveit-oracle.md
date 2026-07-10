# Spec 04: Analytic runtime solver, MoveIt as oracle

- **Status**: proposed
- **Fixes**: F3 (high), F4 (high), F5 (partial — full removal of per-tick validation is gated
  on spec 06)
- **Depends on**: 02 (pure targets), 03 (limit-aware analytic IK)
- **Packages**: `drqp_brain`
- **Size**: L — the core of the migration

## Objective

Solve walking IK in the control loop with the analytic solver from spec 03: deterministic,
microsecond-fast, with per-leg graceful clamping instead of all-or-nothing tick rollback.
Demote MoveItPy to what it is good at: offline stride-limit calibration, launch-test
validation, a solver-agreement oracle, and — until spec 06 covers the combined command
envelope — the per-tick self-collision safety check.

## Current behavior

`packages/runtime/drqp_brain/drqp_brain/locomotion_kinematics.py` +
`brain_node.py`:

- Every tick, each leg is solved by `RobotState.set_from_ik` (KDL numeric, position-only),
  seeded from `/joint_states`, with a retry from the SRDF home pose; then the complete 18-joint
  state is validated for bounds and self-collision.
- Numeric iteration can converge to different femur/tibia branches depending on the seed
  (nondeterminism the retry only papers over).
- Any single failure aborts and rolls back the whole tick → the robot freezes at the same phase
  and retries identically forever (F4).
- Bounds + self-collision are re-proven 8×/s for geometry already certified offline by
  `stride_limits.yaml` (F5).

## Target design

### Backend abstraction

Define the solver interface where it already implicitly exists —
`solve(legs_and_targets, latest_joint_state) -> LocomotionKinematicsResult` — and add:

```python
class AnalyticLocomotionKinematics:
    """Closed-form runtime kinematics backed by drqp_kinematics.LegModel.solve_ik."""

    def ready(self) -> bool: ...           # True once joint limits are installed from URDF
    def solve(self, legs_and_targets, latest_joint_state) -> LocomotionKinematicsResult: ...
```

- `LocomotionKinematicsResult` gains `clamped_legs: tuple[str, ...]` (legs whose target was
  clamped to the workspace/limit boundary). `succeeded` stays `failure_reason is None`;
  clamping is **not** a failure.
- Joint output must be in controller convention (radians, URDF joint names
  `drqp/<leg>_<coxa|femur|tibia>`), i.e. apply the model→URDF offsets
  (`kFemurOffsetAngle`, `kTibiaOffsetAngle`) inside the backend exactly once, mirroring what
  `apply_joint_targets`/`JointTrajectoryBuilder` expect today (spec 07 later consolidates the
  convention; keep the mapping in one named helper so 07 only touches that helper).
- `latest_joint_state` is unused for solving (closed form needs no seed) but keep it in the
  signature for interface compatibility; MoveIt backend still uses it.

### Node parameter

- `kinematics_backend`: `analytic` (default) | `moveit`. The MoveIt path remains fully
  functional behind the flag for A/B comparison and fallback during rollout.
- With the analytic backend, `_ik_ready()` no longer requires `/joint_states` before walking
  (keep the wait anyway — the trajectory controller needs joint states to be up; just decouple
  the reason in code comments).

### Failure semantics (F4)

- Replace tick rollback with per-leg degradation: clamped legs track their boundary pose; the
  tick still publishes. Log (throttled) a warning listing clamped legs.
- The hard failures left: self-collision validation failure (the safety net for the
  not-yet-certified combined envelope — see below), non-finite results (assert — should be
  impossible) and missing joint names. These skip publication, as today, but with the pure
  targets from spec 02 there is no state to roll back.
- Emit a `/robot_event` (or diagnostics) signal when clamping persists > N ticks, as the hook
  for future twist-scaling (spec 06 consumes this properly).

### MoveIt as oracle (F5)

- `MoveItPyLocomotionKinematics` stays for: `generate_stride_limits`, launch tests, and a new
  **agreement test** (below).
- Runtime bounds validation is subsumed immediately: joint limits are enforced analytically per
  solve.
- Runtime **self-collision validation stays in the hot path for now**: the analytic backend
  solves the legs, then validates the assembled 18-joint state with the existing planning-scene
  check before publication. `stride_limits.yaml` certifies **translation-only** strides;
  combined rotation, body pose, and IMU-balance offsets remain uncovered until spec 06's
  twist-level reachability scaling covers the combined envelope. Dropping the per-tick
  collision check is deferred to spec 06 (or to an extended offline certification of the
  combined envelope, whichever lands first) — do not remove it in this spec.
- Keep the class and its tests intact; do not delete any MoveIt config.

## Behavior changes

- Foot targets marginally outside the workspace now produce clamped motion instead of a frozen
  robot. Walking inside the certified stride limits is unchanged (test-enforced, below).
- Branch flips from numeric seeds disappear; joint trajectories become deterministic given the
  same command stream.

## Out of scope

- Twist steering and twist-level saturation handling (spec 06).
- Removing the MoveIt in-process node from bringup (it can stay; only the per-tick usage
  changes). Evaluate startup-cost removal later.

## Test plan (write first)

- **Solver agreement (the oracle test)**: sweep the walking envelope — all gaits × 16
  directions × the certified `stride_limits.yaml` step lengths × all phases (reuse the sweep
  machinery from `generate_stride_limits.make_moveit_step_length_checker`) — and assert
  analytic vs MoveIt joint targets agree within 1e-3 rad for every reachable target. Mark it
  slow/nightly if runtime is prohibitive for PR CI, but it must exist and run in CI at some
  cadence.
- **Determinism**: same targets ⇒ identical joint targets across repeated solves (analytic
  backend).
- **Per-leg clamping**: one leg targeted outside the workspace ⇒ result succeeds, that leg
  reported in `clamped_legs`, other five legs' targets solved exactly.
- **Backend flag**: node test that `kinematics_backend:=moveit` still routes through
  MoveItPyLocomotionKinematics (existing fakes) and `analytic` never constructs MoveItPy.
- **Launch test** (Gazebo, real moveit_py absent from hot path): extend
  `test_brain_moveit_ik.py` with an analytic-backend variant asserting the robot walks a
  straight line and a combined stride+rotation segment without IK warnings. Per workspace
  policy, validate on the real launch stack — unit fakes don't catch binding-level crashes.
- **Latency budget**: measure `solve()` wall time for 6 legs × 2 points in a unit test;
  assert < 5 ms (generous; expected ≪ 1 ms).

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain drqp_kinematics
```

Gazebo: bringup with default (analytic) backend — walk all three gaits, rotate in place,
combined stride+rotation, toggle balance mode. Then repeat with `kinematics_backend:=moveit`
and confirm parity.

## Acceptance criteria

- [ ] `AnalyticLocomotionKinematics` is the default runtime backend; MoveIt backend selectable
      by parameter.
- [ ] Oracle agreement test across the certified envelope passes (≤ 1e-3 rad).
- [ ] Per-leg clamping replaces tick rollback; robot degrades instead of freezing
      (launch-test demonstrated with an intentionally excessive stride command).
- [ ] Hot path performs no per-tick MoveIt IK solves with the analytic backend; the
      planning-scene self-collision validation remains in place (its removal is gated on
      spec 06).
- [ ] `generate_stride_limits` output unchanged (still MoveIt-based, byte-identical YAML).
- [ ] All unit + launch tests pass for both backends.
