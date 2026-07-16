# Spec 06: Gazebo runtime parity

- **Status**: proposed
- **Depends on**: [Spec 05](05-python-and-launch-support.md)
- **Packages**: `drqp_gazebo` plus the already migrated runtime closure
- **Size**: M/L

## Objective

Run the existing Gazebo smoke and slow behavior suites with all first-party
code/resources supplied by Bazel. Treat Gazebo, ros_gz, and gz_ros2_control as
an explicit versioned host boundary from the pinned Jazzy development image.

## Host boundary

Use only `scripts/bazel.sh --config=host-ros` through
`scripts/with-ros-env.sh`. The config may expose `/opt/ros/jazzy` packages for:

- Gazebo Harmonic and its transport/messages;
- `ros_gz_sim`, `ros_gz_bridge`, and their message packages;
- `gz_ros2_control` and its Gazebo plugin;
- other system libraries reached exclusively by those host processes.

First-party packages (`drqp_*`), generated interfaces, controller plugin,
launch files, xacro, config, worlds, and test support must resolve from Bazel
runfiles/ament setup. The wrapper must reject this workspace's `install/` even
when `scripts/with-ros-env.sh` would otherwise add it.

Record the development image digest, architecture, `ROS_DISTRO`, and exact
dpkg versions for every host ROS/Gazebo package in
`evidence/06-gazebo.md`. Do not call the result hermetic.

## Public Bazel label contract

Create `packages/simulation/drqp_gazebo/BUILD.bazel` with:

- `:runtime_data` — `package.xml`, `config/drqp_gazebo_bridge.yml`, all three
  launch files, `worlds/balance_challenge.sdf`, and test fixture SDF/data;
- `:sim` — runnable `launch/sim.launch.py` deployment with all first-party
  nodes/data declared;
- `:gazebo_smoke_tests` — the default three launch tests;
- `:gazebo_full_tests` — the twelve `DRQP_TEST_MODE=slow` launch tests;
- `:gazebo_tests` — smoke, full, and fast non-launch data/xacro tests.

Do not add a `//...` aggregate or migrate `drqp_robot_mcp`.

## Required test inventory

### Smoke group

Match `drqp_gazebo/test/CMakeLists.txt` exactly:

- `test_robot_control_spawn.py`;
- `test_robot_smoke.py`;
- `test_balance_board_world.py`.

### Full group

- `test_robot_control_balance_mode.py`;
- `test_balance_board_armed.py`;
- `test_balance_board_motion_response.py`;
- `test_imu_balance_motion.py`;
- movement backward, forward, sustained-forward, left, right, and rotation;
- armed- and disarmed-posture verification.

Use the existing files and shared `robot_control_test_support.py`; do not create
reduced Bazel-only smoke assertions. Preserve pytest markers, parameterization,
fixture scope, retry semantics, readiness waits, and process-exit checks.

Register `test_gazebo_urdf.py` as a fast test in both build systems if it is not
already collected by ament. This is an allowed parity correction because it
validates the same checked-in model for both paths.

## Runtime/resource requirements

- `FindPackageShare('drqp_gazebo')`, `drqp_control`, `drqp_brain`, and
  `drqp_keyboard_control` must resolve to Bazel-owned runtime data.
- `FindPackageShare` for host packages must resolve beneath `/opt/ros/jazzy`.
- xacro must find all control URDF fragments and meshes from declared data.
- Gazebo must find the world and gz_ros2_control plugin without a source-tree
  current working directory.
- ros_gz_bridge must load `drqp_gazebo_bridge.yml` and establish the topics
  asserted by existing tests.
- Preserve the existing xacro selection exactly: `use_gazebo:=false` loads the
  Bazel-built `drqp_control/a1_16_hardware_interface`, while
  `use_gazebo:=true` loads host `gz_ros2_control/GazeboSimSystem`.

Add one resource-origin assertion to common test support that logs and checks
the resolved share paths above. It must fail if a first-party path contains the
source package directory or workspace `install/`.

## Isolation and timeouts

Each Bazel launch test must set unique `ROS_DOMAIN_ID` and `GZ_PARTITION`, run
with network access enabled only through the `requires-network` tag, and avoid
remote execution. Preserve CMake's 300-second default and 480-second balance-
mode timeout. Use Bazel `size = "large"`/`timeout` metadata rather than adding
test-body sleeps.

The smoke group is the PR/default CI gate. The full group must pass once on the
implementation PR and becomes the scheduled/default-branch observation target
defined in Spec 08.

## Verification

Start from a clean checkout with no `install/` overlay:

```bash
scripts/with-ros-env.sh scripts/bazel.sh test --config=host-ros \
  //packages/simulation/drqp_gazebo:gazebo_smoke_tests
scripts/with-ros-env.sh scripts/bazel.sh test --config=host-ros \
  //packages/simulation/drqp_gazebo:gazebo_full_tests
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to \
  drqp_gazebo
DRQP_TEST_MODE=slow scripts/with-ros-env.sh colcon test --packages-select \
  drqp_gazebo
scripts/with-ros-env.sh colcon test-result --verbose
```

Repeat the Bazel smoke group after `scripts/bazel.sh clean --expunge`. Record
test names/counts/durations, resolved resource origins, image/package versions,
host boundary, and both results in `evidence/06-gazebo.md`.

## Stop conditions

Stop with the smallest failing launch and process logs if:

- first-party package lookup requires a sourced colcon overlay;
- a Gazebo plugin ABI mismatch crashes the process;
- Bazel launch wrapping loses an existing exit-code assertion or retry; or
- tests pass only when run from the source directory or after another test.

## Allowed files

- `drqp_gazebo` BUILD/test registration and a shared resource-origin assertion;
- `bazel/` host-ROS config/helpers and pinned development-image documentation;
- generic rules-fork launch/runtime fixes with pin updates when proven;
- this program's evidence/status documentation.

## Acceptance criteria

- [ ] Smoke and full groups contain the exact existing behavioral tests and
      pass under Bazel and colcon without unconditional skips.
- [ ] Every first-party package/resource origin is Bazel-owned and every host
      package origin is under the recorded `/opt/ros/jazzy` boundary.
- [ ] DDS/Gazebo domains, timeouts, retry behavior, and per-process exit checks
      are preserved.
- [ ] Clean-expunge smoke succeeds without workspace colcon outputs.
- [ ] Host image digest and dependency versions are recorded; the result is
      described as non-hermetic.
- [ ] `evidence/06-gazebo.md` contains the required handoff data.
