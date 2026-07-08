# Spec 07: Single source for servo zero offsets

- **Status**: proposed
- **Fixes**: F26, F10 (partial)
- **Depends on**: 04 (so only one solver's conventions are in play)
- **Packages**: `drqp_brain`, `drqp_control` (URDF), possibly `drqp_a1_16_driver`
- **Size**: M (small diff, high blast radius — mostly verification work)

## Objective

Make one layer own the mechanical zero-offset calibration between the kinematic model and the
physical servos, so MoveIt, Gazebo, RViz, the analytic solver, and the hardware agree by
construction and no angle is offset twice or zero times.

## Current behavior

- `kFemurOffsetAngle = −13.11°`, `kTibiaOffsetAngle = −32.9°` live in
  `packages/runtime/drqp_brain/drqp_brain/joint_trajectory_builder.py` and are:
  - **added** when converting model angles → controller positions
    (`add_point_from_hexapod`), and
  - **subtracted** when converting solved controller positions → model angles
    (`HexapodBrain.apply_joint_targets` in `brain_node.py`).
- IK results (MoveIt or, post-spec-04, the analytic backend's output helper) are published
  _without_ the offsets, while `add_point_from_hexapod` applies them — two conventions coexist
  in the same message stream depending on which code path built the trajectory point. It works
  because each path is internally consistent, but it is a trap for the next contributor (F26).
- The offsets encode where the model's zero pose sits relative to the URDF/servo zero — i.e.
  assembly geometry (bracket angles), not per-unit servo calibration.

## Target design

Decision to make first (spike, ~1 hour): confirm the offsets are pure assembly geometry by
checking the URDF link frames vs. the `LegModel` conventions. Then:

1. **Model speaks URDF convention at its boundary.** The conversion (model zero ↔ URDF zero)
   moves into `drqp_kinematics` as named constants/functions next to `LegModel`
   (`model_to_urdf_angles(...)`, `urdf_to_model_angles(...)`), with the geometric derivation
   documented in the docstring (why −13.11°: femur bracket angle; why −32.9°: tibia bracket).
   Spec 03/04's single mapping helper is refactored to call these.
2. **`JointTrajectoryBuilder` becomes convention-free**: it accepts controller-convention
   radians only. `add_point_from_hexapod` calls the conversion explicitly at its call sites
   (initialization/finalization sequences in `brain_node.py`), making the conversion visible
   exactly where model poses are used directly.
3. **Delete** `kFemurOffsetAngle` / `kTibiaOffsetAngle` from `joint_trajectory_builder.py` and
   the inverse application from `apply_joint_targets` (it uses `urdf_to_model_angles`).
4. **Do not** fold the offsets into URDF joint origins: the URDF matches the physical robot and
   is shared with Gazebo/MoveIt; moving model-convention artifacts into it would change
   simulation geometry. The model, not the robot description, is the nonstandard party. (If the
   spike proves the URDF is the wrong-zero party instead, invert this step and document it —
   but expect the URDF to be correct, since Gazebo walking currently matches hardware.)

## Behavior changes

None observable. Every published joint position must be bit-identical before/after for the same
command stream — that is the primary acceptance test.

## Out of scope

- Per-servo trim calibration (a real future feature — belongs in `drqp_a1_16_driver` or
  ros2*control offsets; note it in the docstring as explicitly \_not* this mechanism).
- Radians-native model internals beyond the boundary conversion (rest of F10).

## Test plan (write first)

- **Golden trajectory equivalence**: capture the full `JointTrajectory` message sequence for
  (a) the initialization sequence, (b) the finalization sequence, (c) one walking cycle per
  gait with fixed commands — before the change; assert byte/float-identical output after.
- Round-trip: `urdf_to_model_angles(model_to_urdf_angles(x)) == x`.
- Grep-level guard: no occurrence of the numeric literals `13.11` / `32.9` outside
  `drqp_kinematics` (cheap lint test that prevents re-duplication).
- Launch tests: `test_bringup_launch.py`, `test_brain_moveit_ik.py` (both backends), and the
  initialization/finalization sequences on Gazebo — poses must be visually identical (feet
  positions in the default stance unchanged; assert via TF or joint states in the launch test
  rather than eyeballing).

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_kinematics drqp_brain
```

If hardware is available: run the initialization sequence on the physical robot before
merging — zero-offset regressions fold legs into the chassis; do it torque-limited / on a
stand first.

## Acceptance criteria

- [ ] Offset constants exist in exactly one module (`drqp_kinematics`), with geometric
      derivation documented.
- [ ] `JointTrajectoryBuilder` and `apply_joint_targets` are convention-free / use the shared
      helpers.
- [ ] Golden trajectory equivalence tests pass (init, finalize, walking × 3 gaits).
- [ ] Literal-duplication guard test in place.
- [ ] Launch tests pass; default stance foot positions unchanged.
