# Balance-mode safety implementation record

This spike investigates the slow Gazebo failure in
[PR #443 run 29560942132](https://github.com/Dr-QP/Dr.QP/actions/runs/29560942132/job/87993556892?pr=443)
and the validity of the walking-plus-balance tests. The recommended product contract is:

> Balance mode is a stationary, reachability-bounded body-posture hold. It does not support
> simultaneous walking until contact-aware locomotion is designed and validated.

That contract agrees with the existing locomotion analysis: the current IMU controller is an
attitude-only posture controller with no foot-contact or support-polygon awareness (F18/F19).

## Raw findings

### What consumed the one-hour CI job

- CTest started 15 `drqp_gazebo` test executables with automatic parallelism and ran four Gazebo
  processes concurrently.
- `test_imu_balance_motion` completed after 758.34 seconds. Its six cases passed only after the
  right-stride case launched four complete simulations and failed its first three attempts.
- The job reached `14/15` at 07:17:46 UTC. The missing test was
  `test_robot_control_balance_mode`, which had started at 07:03:22 and was still running when the
  job was canceled at 07:50:23. Its buffered pytest output was never emitted.
- CTest's timeout is explicitly disabled (`TIMEOUT 0`, displayed as 10,000,000 seconds). Pytest's
  600-second timeout applies to each parameterized test attempt, while the collection hook adds
  three retries to every launch test. One deterministic failure may therefore consume four
  600-second attempts before moving to the next of eight tilt scenarios.
- The blanket retry conflicts with the repository launch-test guidance. Retries are intended for
  a confirmed nondeterministic process-shutdown crash, not behavior assertions.

### What the movement evidence says

The linked CI run measured:

| Command                  | Balance off | Balance on evidence                 |
| ------------------------ | ----------- | ----------------------------------- |
| Forward full stride      | +0.649 m    | +0.019 m                            |
| Backward full stride     | -0.666 m    | -0.017 m                            |
| Right full stride        | -0.073 m    | +0.001, -0.002, +0.018, then passed |
| Left full stride         | +0.060 m    | +0.049 m                            |
| Diagonal `(0.66, -0.77)` | not sampled | 0.047 m magnitude                   |

The balance-mode assertions use a threshold of only 0.008 m. They can pass while forward/backward
travel has collapsed by about 97%, so they do not establish useful simultaneous walking and
balancing.

The logs report `Analytic IK clamped legs` during the failed and passing balance-motion attempts.
A standalone reproduction of right motion passed in 35.89 seconds with -0.058 m travel, but still
clamped `right_middle`. This combination points to reach/limit and contact-physics sensitivity,
not a permanent numerical-IK freeze.

### Static balance also reaches the workspace boundary

A standalone `pitch-positive` board-tilt case passed in 42.05 seconds, but repeatedly reported
clamping of the back legs. Immobilizing the gait removes one source of target displacement, but
does not by itself make the current 0.15-radian posture correction safe or reachable. The
supported tilt envelope must be derived from full six-leg reachability and must not rely on
post-solve joint clamping.

### The MoveIt wording is stale, with one important nuance

- Analytic `LegModel.solve_ik(..., clamp=True)` is the default runtime leg solver.
- MoveIt is no longer the real-time IK solver, so `MoveIt IK freeze`,
  `assert_no_moveit_ik_failures`, and the file-level regression narrative are inaccurate.
- MoveIt is still in the hot loop as `MoveItPyStateValidator`: each analytic candidate is assembled
  into a `RobotState` and checked for self-collision. It is therefore accurate to call MoveIt the
  runtime collision oracle, not the runtime IK solver.
- The harness watches only legacy `IK failed` / `IK rejected` text. It does not fail on current
  analytic clamping or the current `Kinematics rejected ...` warning.

## Prioritized issues

| Priority | Status   | Issue                                                                  |
| -------- | -------- | ---------------------------------------------------------------------- |
| P0       | Resolved | Blanket retries multiply deterministic behavior failures by four.      |
| P0       | Resolved | Slow Gazebo tests have no outer CTest deadline below the job timeout.  |
| P0       | Resolved | Walking-plus-balance is asserted although it is not a supported mode.  |
| P1       | Resolved | Static balance can persistently clamp legs at the tested tilt.         |
| P1       | Resolved | Automatic Gazebo parallelism makes physics timing and motion unstable. |
| P1       | Resolved | Test diagnostics still describe legacy MoveIt IK behavior.             |
| P2       | Resolved | Eight full-simulation tilt cases duplicate axis/sign math coverage.    |

## Recommended implementation order

1. Constrain slow Gazebo CI and retries. **Completed on this branch.**
2. Make balance mode stationary and reachability-bounded. **Completed on this branch.**
3. Retire stale movement regressions and update diagnostics. **Completed on this branch.**

## Commands run during the spike

```text
scripts/with-ros-env.sh python3 -m colcon build --symlink-install \
  --packages-up-to drqp_gazebo

DRQP_TEST_MODE=slow PYTHONPATH=packages/simulation/drqp_gazebo/test \
  scripts/with-ros-env.sh python3 -m pytest \
  packages/simulation/drqp_gazebo/test/test_locomotion_direction_reversal.py::\
test_direction_reversal_from_forward_to_backward -p no:retry -vv -s -rA

DRQP_TEST_MODE=slow PYTHONPATH=packages/simulation/drqp_gazebo/test \
  scripts/with-ros-env.sh python3 -m pytest \
  packages/simulation/drqp_gazebo/test/test_robot_control_stationary_posture.py::\
test_stationary_posture_levels_body_on_pure_pitch -p no:retry -vv -s -rA
```

The incremental build completed for 16 packages, and both retained isolated launch tests passed.
The stationary pure-pitch case can report pre-solve correction saturation; the reachability bound
handles that condition without a persistent post-solve clamping event.
