# 02 — Make balance mode stationary and reachability-bounded

## Goal

Treat balance mode as a stationary posture hold. Suppress stale or concurrent movement commands,
and bound attitude correction before any leg target requires workspace or joint-limit clamping.

## Behavior changes

- Enabling balance mode stops the gait and captures the current body attitude target.
- While balance mode is enabled, stride, yaw, body-translation, and user body-rotation commands do
  not alter foot targets. Only the balance posture correction is applied.
- Movement commands received during balance mode are not replayed later. Disabling balance mode
  requires a fresh movement command before walking resumes.
- Balance mode remains available only while the robot is armed and IMU data is fresh. Existing
  stale-IMU behavior remains fail-safe.

## TDD red

1. Unit-test that entering balance mode zeroes the active semantic movement state.
2. Unit-test that movement callbacks cannot re-arm gait motion while balance mode is active.
3. Unit-test that disabling balance mode leaves the robot stationary until a fresh command.
4. Unit-test that posture correction is scaled against all six analytic leg workspaces before
   solving, rather than relying on `solve_ik(..., clamp=True)` afterward.
5. Add one Gazebo test that commands motion before and during balance mode and verifies negligible
   odometry movement while the body posture remains controlled.
6. Add one representative two-axis board-tilt test that asserts no persistent clamping event.

## Implementation constraints

- Keep `/robot/balance_mode` message compatibility.
- Use analytic full-state reachability to find a scalar correction bound for the current stance.
  Do not guess a new fixed `imu_balance_max_tilt_rad` from the present flaky tests.
- Surface correction saturation through a named event or diagnostic with the constrained legs.
- Do not describe the current controller as contact-aware balance. It is a stationary body-posture
  controller until contact state and support-margin logic land.
- Keep MoveIt collision validation semantics unchanged in this spec; removing it from the hot loop
  is a separate safety decision.

## Acceptance criteria

- A held joystick command cannot move the robot while balance mode is active.
- Disabling balance mode cannot resume a command published before or during balance mode.
- Every target in the documented static tilt envelope is analytically reachable and within joint
  limits for all six legs without clamping.
- The representative Gazebo test shows body leveling and less than the agreed stationary odometry
  tolerance.
- Hardware documentation states the validated tilt envelope and fail-safe behavior.

## Non-goals

- Walking while balancing.
- Contact estimation, support-polygon control, terrain adaptation, or dynamic stabilization.
