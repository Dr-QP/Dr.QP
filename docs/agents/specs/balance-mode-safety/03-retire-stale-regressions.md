# 03 — Retire stale movement regressions and update diagnostics

## Goal

Align the slow suite and its language with analytic runtime IK and the stationary balance-mode
contract.

## Dependencies

- Spec 01 for bounded execution.
- Spec 02 for the stationary behavior contract.

## TDD red

1. Add unit coverage for the stationary balance behavior before deleting contradictory Gazebo
   movement assertions.
2. Add a harness test proving current `Kinematics rejected ...` warnings are captured without
   calling them MoveIt IK failures.
3. Add a harness test for the persistent analytic-clamping diagnostic/event.

## Implementation

- Delete the five cardinal/diagonal walking-plus-balance cases from
  `test_imu_balance_motion.py` and their support helpers.
- Preserve the direction-reversal regression only if it still protects an independent locomotion
  contract. Move it to a locomotion-named file and describe analytic reachability/clamping; it is
  unrelated to balance mode.
- Replace `ik_failure_messages` and `assert_no_moveit_ik_failures` with backend-neutral
  kinematics-rejection diagnostics. Distinguish:
  - analytic workspace/joint-limit clamping;
  - complete-state collision rejection by the MoveIt validator;
  - optional legacy MoveIt IK backend failure.
- Rename balance documentation and tests to “stationary posture” where doing so does not break
  public topic compatibility.
- Reduce the eight Gazebo tilt directions to representative physics cases. Cover pure axis,
  diagonal, and sign symmetry in fast unit tests of quaternion/mount/correction math.

## Acceptance criteria

- No default-path test claims that MoveIt performs real-time IK.
- No test requires locomotion while stationary balance mode is enabled.
- A real analytic rejection or persistent clamp cannot silently pass under a legacy log filter.
- The retained slow tests each protect a distinct physics behavior and have a documented reason
  for requiring Gazebo.
