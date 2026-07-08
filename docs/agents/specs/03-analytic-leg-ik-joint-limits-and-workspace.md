# Spec 03: Analytic leg IK — joint limits & workspace clamping

- **Status**: proposed
- **Fixes**: F8, F9, F10, F11, F23
- **Depends on**: nothing (parallel with 02)
- **Packages**: `drqp_kinematics`, `drqp_brain` (limit parsing), no runtime behavior change yet
- **Size**: M

## Objective

Upgrade the existing closed-form solver in `drqp_kinematics` into a runtime-grade IK library:
radians-native, joint-limit aware, with explicit workspace clamping and a structured result —
so spec 04 can swap it into the control loop. This spec is pure library work; the runtime keeps
using MoveIt until spec 04.

## Current behavior

`packages/runtime/drqp_kinematics/drqp_kinematics/models.py`:

- `LegModel.inverse_kinematics()` implements the correct yaw–pitch–pitch closed form
  (coxa from `atan2`, femur/tibia by law of cosines) with `safe_arccos` clamping unreachable
  targets to the boundary.
- The solvability flag is returned but ignored by every runtime caller
  (`WalkController.__move_feet`, `LegModel.move_to`).
- No joint-limit checking; the URDF limits are hand-duplicated in
  `drqp_brain/generate_stride_limits.py` (`JOINT_LIMITS_DEGREES`) with a "must match the URDF"
  comment.
- API is degrees-based; `forward_kinematics()` allocates an `AffineTransform` chain and
  matplotlib label strings per call.

## Target design

### Structured solve result

```python
@dataclass(frozen=True)
class LegIKSolution:
    angles_rad: tuple[float, float, float]   # coxa, femur, tibia — model convention
    reachable: bool          # target inside workspace before clamping
    within_limits: bool      # solution satisfies joint limits before limit clamping
    clamped_target: Point3D  # target actually solved (== input when reachable)
    limit_margin_rad: float  # min distance of any joint to its nearest limit
```

New method `LegModel.solve_ik(foot_target, *, clamp=True) -> LegIKSolution`; the existing
degree-based `inverse_kinematics()` remains as a thin wrapper (notebooks depend on it) and is
marked for eventual deprecation.

### Joint limits

- `LegModel` gains `joint_limits_rad: tuple[(lo, hi), (lo, hi), (lo, hi)] | None`
  (coxa, femur, tibia), threaded through `HexapodModel.__init__`.
- Add one shared parser `drqp_kinematics.urdf_limits.parse_joint_limits(robot_description_xml)`
  returning `{joint_name: (lower, upper)}` from `<limit lower= upper=>` tags. It must work on
  the xacro-processed URDF already loaded by `drqp_brain` and by `generate_stride_limits`.
- Replace `JOINT_LIMITS_DEGREES` in `generate_stride_limits.py` with the parser output (F23).
  Note the model↔URDF convention offsets (`kFemurOffsetAngle`, `kTibiaOffsetAngle` in
  `drqp_brain/joint_trajectory_builder.py`): URDF limits must be converted into model
  convention when installed on `LegModel`. Centralize that conversion next to the parser and
  cover it with a test (spec 07 later consolidates the convention itself).

### Workspace clamping

Given the localized target (after `to_local`), clamp before solving:

1. Coxa yaw: clamp `atan2(y, x)` into the coxa limit range; rotate the target onto the boundary
   azimuth if outside.
2. Radial/vertical plane: the reachable set around the coxa end is the annulus
   `|l_femur − l_tibia| ≤ span ≤ l_femur + l_tibia` (span measured from the femur joint after
   subtracting `coxa_length`), further cut by femur/tibia limits. Clamp the planar target
   radially onto the nearest annulus boundary; if the limit-cut makes the solved angles violate
   limits, clamp angles to limits and report `within_limits=False` with the resulting FK foot
   position as `clamped_target`.
3. `reachable` reflects the pre-clamp test; the returned angles are always finite, always
   within limits when `clamp=True`.

### Performance (F11)

- Split the notebook-facing FK (labels, `Line3D` chains) from a lean
  `fk_foot_position(angles_rad) -> np.ndarray` used by the solver and tests.
- Stretch goal (do not block on it): `HexapodModel.solve_all_ik(targets: (6,3) array)`
  vectorized with numpy. Land the scalar version first.

## Out of scope

- Swapping the runtime solver (spec 04).
- Self-collision checking — the analytic layer never checks collisions; envelope certification
  stays with the MoveIt-based stride-limit generation.
- Elbow-branch selection: keep the single existing branch (it matches the physical assembly);
  document this in the class docstring.

## Test plan (write first)

- **FK∘IK round-trip property test**: for a grid (or hypothesis-style sampling) of joint
  angles inside limits, `solve_ik(fk(angles)).angles_rad ≈ angles` within 1e-9 rad and
  `reachable and within_limits`.
- **Clamp behavior**: targets outside the annulus (too far / too close / below coxa yaw range)
  return `reachable=False`, finite angles within limits, and `fk(angles) == clamped_target`
  with `clamped_target` on the workspace boundary (distance check).
- **Limit parsing**: parse the real robot_description (xacro of
  `drqp_control/urdf/drqp.urdf.xacro` with `mock_servo`, as `generate_stride_limits` already
  does) and assert the values match the current URDF numbers for all 18 joints; assert
  `generate_stride_limits` produces identical YAML output before/after the swap to the parser.
- **Solvability flag propagation**: `move_to` result no longer silently dropped — either
  consumed or the legacy path documented (full consumption happens in spec 04; here just keep
  the wrapper truthful).
- Degrees wrapper regression: existing `test_solver.py` / notebook-oriented tests keep passing
  unchanged.

## Verification

```bash
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain
scripts/with-ros-env.sh colcon test --packages-select drqp_kinematics drqp_brain
```

Regenerate `stride_limits.yaml` with `generate_stride_limits` and diff against the committed
file — it must be identical (this spec changes no geometry or solving behavior on the MoveIt
path).

## Acceptance criteria

- [ ] `LegIKSolution` API with reachability, limit status, clamped target, and margin.
- [ ] Radians-native `solve_ik`; degrees API preserved as wrapper.
- [ ] Joint limits parsed from the URDF in one place; `JOINT_LIMITS_DEGREES` deleted.
- [ ] Workspace clamping returns boundary solutions for unreachable targets.
- [ ] FK∘IK round-trip property test in place and passing.
- [ ] `stride_limits.yaml` regeneration is byte-identical.
- [ ] All `drqp_kinematics` and `drqp_brain` tests pass.
