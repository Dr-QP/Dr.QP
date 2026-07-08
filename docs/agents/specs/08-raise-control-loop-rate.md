# Spec 08: Raise control loop rate

- **Status**: proposed
- **Fixes**: F25
- **Depends on**: 02 (time-based phase), 04 (analytic solver) — 06 recommended first so
  steering latency gains are realized together
- **Packages**: `drqp_brain`
- **Size**: M (small diff, mostly measurement and tuning)

## Objective

Raise the walking control loop from 8 Hz to 25–50 Hz now that the hot path is analytic,
reducing steering latency and unlocking the balance-bandwidth headroom needed by later
(contact/balance v2) work.

## Current behavior

- `self.fps = 8` in `HexapodBrain.__init__` (`brain_node.py`); the loop timer, trajectory
  point spacing (`point_index / self.fps`), and (pre-spec-02) phase stepping and smoothing all
  derive from it.
- End-to-end steering latency ≈ smoothing τ + 1–2 ticks ≈ 350–600 ms.
- MoveIt per-tick solving made higher rates impractical; after spec 04 the solve cost is ≪ 1 ms.

## Target design

- `fps` becomes a declared ROS parameter `control_rate_hz` (default **25**, validated range
  [5, 100]). After specs 02/05/06, gait speed, smoothing, and steering are all
  time-parameterized, so changing the rate must not change robot speed or feel — that is the
  core invariant to test.
- Trajectory window: keep `walking_trajectory_points = 2` with spacing `1 / control_rate_hz`.
  At 25 Hz this gives the joint_trajectory_controller 40 ms segments — verify the JTC accepts
  and splines them (it does for position-only points, but confirm no
  `goal_time_tolerance`/update-rate interaction in `drqp_control` configs; the JTC
  `update_rate` must be ≥ 2× `control_rate_hz`).
- Publication dedupe (input comparison from spec 02) means idle ticks stay cheap; verify no
  publish storm when stationary.
- **Budget instrumentation**: add a lightweight tick-duration measurement (min/mean/max over a
  sliding window) exposed on `/diagnostics` or a debug log line every N seconds. Acceptance is
  defined against this measurement, not vibes.
- Hardware ceiling: the A1-16 serial bus is the real constraint on the physical robot — 18
  servos × position write at 25 Hz must fit the bus baud rate. Check
  `drqp_a1_16_driver`/`drqp_control` update rates and, if the bus saturates, keep
  `control_rate_hz` at 25 for hardware launch files and 50 for simulation ones (parameter
  per launch file, same code path).

## Behavior changes

- Steering feels snappier (same τ, less discretization delay). Robot speed and gait cycle
  times must be provably unchanged (spec 02's cycle-time tests re-run at multiple rates).

## Out of scope

- Splitting gait/IK into a separate node or executor tuning beyond what measurement demands.
- Balance controller gains retune (balance v2 work owns that; note that P-gain stability
  improves at higher rate, so no regression is expected).

## Test plan (write first)

- **Rate invariance**: run the walker (sim-time unit test with fake clock) at 8, 25, and 50 Hz
  with identical command streams; assert per-gait cycle times and per-foot path geometry match
  within tolerance (the spec 02 cycle-time tests parameterized by rate).
- **Budget**: unit-level timing test — full tick (targets + solve + trajectory build) at 25 Hz
  budget < 40 ms with ≥ 4× margin on the dev container; log actuals.
- **Launch test**: bringup at 25 Hz in Gazebo — walking all gaits, no JTC goal rejections, no
  missed-deadline warnings, tick-duration diagnostic max < period.
- **Idle behavior**: stationary robot publishes no trajectories (dedupe holds at high rate).

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_brain
```

Gazebo at 25 Hz and 50 Hz: teleop feel check, diagnostics review. On hardware (if available):
25 Hz soak walk ≥ 5 min watching for serial bus errors / servo command drops in the driver
logs before changing the hardware launch default.

## Acceptance criteria

- [ ] `control_rate_hz` parameter (default 25) replaces the hardcoded `fps`; all derived
      timings follow it.
- [ ] Rate-invariance tests pass at 8/25/50 Hz.
- [ ] Tick-duration instrumentation in place; measured max ≪ period at default rate.
- [ ] Gazebo launch tests green at 25 Hz; hardware launch default only raised after bus soak
      test.
- [ ] No trajectory publications while stationary.
