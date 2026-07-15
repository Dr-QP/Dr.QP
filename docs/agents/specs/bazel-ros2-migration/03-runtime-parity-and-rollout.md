# Spec 03: Runtime parity and rollout decision

- **Status**: proposed
- **Depends on**: [Spec 02](02-interfaces-and-control-proof.md)
- **Size**: XL

## Objective

Expand from the interface/control proof to a representative complete runtime
graph, then make an evidence-based decision to retain Bazel as optional,
require it in CI, or stop the migration.

## Scope and order

Convert and validate in this order:

1. Python runtime packages `drqp_kinematics`, `drqp_brain`, and
   `drqp_keyboard_control`.
2. `drqp_gazebo`, including worlds, bridge configuration, launch files, and
   existing focused simulation tests.
3. `drqp_moveit`, including configuration, URDF/xacro dependencies, launch
   files, and existing MoveIt smoke tests.
4. Remaining first-party packages and vendored packages only when required by
   the selected target set.

Do not widen the scope merely to achieve `bazel build //...`; each package must
have a declared runtime and test rationale.

## Runtime design

- Package resources must be loaded from Bazel runfiles or an explicitly created
  runtime prefix, never from an undeclared source-relative path.
- ROS launch tests retain their current assertions and process-exit checks.
  Preserve their fixture scope and retry behavior.
- Python packages use the Jazzy/Python 3.12 toolchain. Test console entry
  points, imports, package resource lookup, and launch execution.
- Gazebo and MoveIt dependencies may initially be supplied as host-installed
  system dependencies. Record their exact version, ABI boundary, and whether
  each is hermetic. Do not describe the result as fully hermetic while any
  runtime dependency is host supplied.
- Keep ament/colcon packaging metadata in sync with Bazel targets. A source,
  resource, dependency, or test change must update both declarations until one
  system is explicitly retired.

## CI rollout

1. Add a non-required Bazel CI job for the Spec 02 target set. Publish test
   XML and concise timing/cache metrics.
2. Add the representative Gazebo and MoveIt smoke targets after they pass
   locally in the approved ROS environment.
3. Retain colcon as the required build/test path throughout a 20-consecutive
   default-branch-build observation window.
4. Promote Bazel to a required CI job only if all global validation criteria
   in the program README hold and maintainers approve the measured cost.
5. Keep a one-click CI rollback that makes the Bazel job optional or disables
   it without changing production source or the colcon release job.

## Required measurements

Record each measurement in a committed migration report or CI summary:

| Metric            | Required comparison                                              |
| ----------------- | ---------------------------------------------------------------- |
| Clean build time  | Bazel vs. incremental colcon for the selected graph              |
| No-op build time  | Two consecutive builds with no source changes                    |
| Focused test time | The same interfaces/control and launch test targets              |
| Cache behavior    | Local and, if enabled, remote cache hit/miss rates               |
| Disk consumption  | Bazel output base plus external repositories                     |
| Failure diagnosis | Time and logs needed to identify one intentionally induced error |

Measurements inform the decision; they do not override correctness or runtime
parity requirements.

## Test plan

- Run every existing `drqp_gazebo` and `drqp_moveit` smoke/launch test selected
  for the migration under both Bazel and colcon.
- Prove that each launch can locate its parameters, worlds, meshes, plugins,
  URDF/xacro, and Python modules from the Bazel runtime environment.
- Perform one clean checkout Bazel build/test on CI and one developer machine
  using documented prerequisites only.
- For each rules-fork update, run the full selected parity suite before
  updating the pinned revision.
- Make an intentionally invalid message, plugin XML entry, and missing runtime
  data dependency in a throwaway branch; confirm Bazel reports actionable,
  deterministic failures and that no stale output masks them.

## Acceptance criteria

- [ ] Python runtime packages run on Python 3.12 with declared imports,
      console entry points, and package resources available from Bazel outputs.
- [ ] Selected Gazebo and MoveIt smoke tests pass with the existing behavioral
      assertions and process-exit checks.
- [ ] Runtime resources, plugin loading, and ROS package discovery do not
      rely on the source checkout or a sourced colcon overlay.
- [ ] Colcon and Bazel pass independently for 20 consecutive default-branch
      builds covering the selected graph.
- [ ] Fork updates are pinned, reviewed, and gated by the parity suite.
- [ ] Cost, setup, cache, disk, and failure-diagnosis evidence is recorded and
      maintainers make a documented promote/retain-optional/stop decision.
- [ ] Disabling the Bazel job leaves the existing colcon release path intact.
