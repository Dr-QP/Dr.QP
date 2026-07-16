# Spec 07: MoveIt runtime parity

- **Status**: proposed
- **Depends on**: [Spec 06](06-gazebo-runtime-parity.md)
- **Packages**: `drqp_moveit` plus the already migrated runtime closure
- **Size**: M

## Objective

Package the MoveIt configuration and launch graph as Bazel runtime data, then
run the repository's three launch smoke tests and full runtime test with
unchanged assertions. MoveIt binaries, Python/message modules, and plugins are
an explicit host boundary from the same pinned Jazzy development image used by
Spec 06.

## Public Bazel label contract

Create `packages/runtime/drqp_moveit/BUILD.bazel` with:

- `:runtime_data` — `package.xml`, all files under `config/`, all five Python
  files under `launch/` including `moveit_launch_utils.py`, and declared
  `drqp_control` data dependencies;
- `:move_group` — runnable `launch/move_group.launch.py` deployment;
- `:demo` — runnable `launch/demo.launch.py` deployment;
- `:demo_gazebo` — runnable `launch/demo_gazebo.launch.py` deployment;
- `:moveit_rviz` — runnable `launch/moveit_rviz.launch.py` deployment;
- `:moveit_smoke_tests` — the three existing smoke files;
- `:moveit_runtime_test` — `test_moveit_runtime_launch.py`;
- `:moveit_tests` — aggregate of smoke and runtime targets.

Preserve the launch API and defaults. In particular, do not redeclare,
explicitly forward, or bind launch arguments owned by included launch files;
externally set arguments flow through the launch context.

## Host boundary

Under `--config=host-ros`, expose only the MoveIt/Jazzy packages required by
`drqp_moveit/package.xml` and its tests, including move_group, planners,
kinematics, planning interface, visualization/config builders, controller
manager, `moveit_msgs`, and related message packages.

All first-party Python modules, launch/config/SRDF/URDF data, generated
`drqp_interfaces`, and test support must remain Bazel-owned. Host MoveIt and
message bindings may resolve under `/opt/ros/jazzy`; record their exact package
versions and shared-library origins.

Before the full suite, add a compatibility probe target that imports the host
MoveIt message/service modules in the Bazel Python 3.12/rclpy test environment,
constructs and serializes representative `moveit_msgs` request/message types,
and exits cleanly. This catches CPython/type-support/ABI conflicts before a
multi-minute launch obscures the cause.

Public label:
`//packages/runtime/drqp_moveit:host_moveit_compatibility_test`.

## Existing tests to preserve

### Smoke tests — 300 seconds each

- `test_move_group_launch_smoke.py`;
- `test_demo_launch_smoke.py`;
- `test_demo_gazebo_launch_smoke.py`.

### Runtime test — 600 seconds

- `test_moveit_runtime_launch.py`.

Use the existing `conftest.py` and `moveit_launch_smoke_test_support.py` as
declared test deps/data. Preserve function-scoped launch fixtures, flaky retry
markers, unique Gazebo partition construction, readiness service checks,
motion/IK/state-validity assertions, planning-scene obstacle checks, analytic-
IK comparison, and per-process exit-code verification.

Do not reduce the full runtime test to service availability. Its motion,
failure-case, and collision assertions are the behavioral proof.

## Runtime/resource requirements

- `drqp_moveit` config and launch lookup resolves from Bazel runfiles.
- `drqp_control` URDF/xacro/meshes and `drqp_gazebo` world/config resolve from
  their Bazel data targets.
- MoveIt executables/plugins and `moveit_msgs` resolve only beneath the recorded
  `/opt/ros/jazzy` host boundary.
- Generated robot description, semantic description, joint limits, kinematics,
  OMPL, controller mappings, and planning pipeline parameters reach move_group.
- `demo_gazebo` starts the existing controller/simulation graph without source-
  relative paths or a workspace overlay.

Extend the common resource-origin assertion to cover `drqp_moveit` and the
specific host MoveIt packages. Fail if first-party paths point into either the
source package directory or workspace `install/`.

## Isolation

Tag all launch tests `requires-network`, assign unique ROS/Gazebo domains, and
disable remote execution. Preserve existing test timeouts and pytest retries;
do not multiply Bazel-level retries on top of pytest-retry. Ensure every retry
gets a fresh function-scoped launch fixture/event loop where the test expects
relaunch behavior.

## Verification

Start with no workspace colcon overlay:

```bash
scripts/with-ros-env.sh scripts/bazel.sh test --config=host-ros \
  //packages/runtime/drqp_moveit:host_moveit_compatibility_test
scripts/with-ros-env.sh scripts/bazel.sh test --config=host-ros \
  //packages/runtime/drqp_moveit:moveit_smoke_tests
scripts/with-ros-env.sh scripts/bazel.sh test --config=host-ros \
  //packages/runtime/drqp_moveit:moveit_runtime_test
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to \
  drqp_moveit
scripts/with-ros-env.sh colcon test --packages-select drqp_moveit
scripts/with-ros-env.sh colcon test-result --verbose
```

Repeat compatibility and smoke targets after Bazel clean expunge. Record test
names/counts/durations, resource origins, host versions/library paths, retries,
and both results in `evidence/07-moveit.md`.

## Stop conditions

Stop with the smallest compatibility/launch reproducer if:

- host MoveIt Python or type-support modules cannot coexist with the Bazel
  Python/rclpy runtime;
- a MoveIt/Gazebo plugin ABI mismatch crashes a process;
- a launch requires workspace colcon output or source-tree working directory;
- preserving retry scope causes stale event-loop behavior; or
- the full test passes only after removing an existing behavioral assertion.

## Allowed files

- `drqp_moveit` BUILD/test registration and shared resource-origin checks;
- the explicit host-ROS Bazel configuration and version documentation;
- generic rules-fork runtime fixes with immutable pin updates when proven;
- this program's evidence/status documentation.

## Acceptance criteria

- [ ] The compatibility probe serializes host MoveIt messages in the Bazel
      Python runtime without ABI/type-support errors.
- [ ] All three smoke tests and the full runtime test pass under Bazel and
      colcon with their existing assertions, fixture scopes, retries, and
      process-exit checks.
- [ ] First-party resources are Bazel-owned and host resources are confined to
      the recorded `/opt/ros/jazzy` boundary.
- [ ] Clean-expunge compatibility/smoke tests do not consume workspace colcon
      output.
- [ ] Host versions and non-hermetic boundaries are recorded honestly.
- [ ] `evidence/07-moveit.md` contains the required handoff data.
