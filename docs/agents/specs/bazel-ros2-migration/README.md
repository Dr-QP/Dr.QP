# Bazel ROS 2 Jazzy migration investigation

## Decision

Treat a Bazel build for this workspace as a staged migration, with ament and
colcon retained as the release-build and compatibility baseline until Bazel
meets the validation criteria below. Start from a repository-owned fork of
`mvukov/rules_ros2` at the `feature/jazzy` head, pinned to an immutable commit.

The fork is a safety boundary, not a claim of permanent ownership. It permits
the project to backport fixes, review upstream changes deliberately, and keep a
known-good revision available if upstream force-pushes, changes direction, or
the Jazzy pull request is not merged promptly. The fork must preserve upstream
attribution and be regularly reconciled with upstream.

Do not remove `package.xml`, `ament_cmake`/`ament_python`, or colcon jobs in
the initial migration. ROS package metadata and ament builds remain necessary
for distro interoperability and provide an independent behavioral oracle.

## Raw findings

### Workspace scope

The workspace contains 17 ROS 2 packages: 16 with `CMakeLists.txt` and seven
with `setup.py`. The project mixes C++ ament packages, Python ament packages,
and vendored ROS Python packages.

`drqp_interfaces` defines five custom `.msg` files. `drqp_control` exports a
`hardware_interface::SystemInterface` through `pluginlib`; it also depends on
ros2_control. The runtime graph uses rclcpp, rclpy, launch, Gazebo,
ros_gz_bridge, MoveIt, xacro, and launch-pytest integration tests. These are
the migration's highest-value compatibility surfaces.

### Jazzy rules_ros2 status

Upstream `mvukov/rules_ros2` pull request #558 is an open, ready-for-review
57-commit `feature/jazzy` branch. It updates the ROS source set to Jazzy,
updates Python to 3.12, and contains fixes for C, C++, and Python ROSIDL code
generation. The author reports a clean-cache `bazel test //...` and CI; a
downstream contributor reports that one repository works on the branch.

The branch is not merged into `main` and still has review and integration
work. In particular, discussion identifies dependent IDL-support patches and a
related open IDL pull request (#617). It is therefore promising but not yet a
versioned upstream release suitable for an unpinned dependency.

### Consequence

The Jazzy branch removes the previous fundamental blocker: a Bazel migration
is now reasonable to investigate. It does not remove the need to prove this
workspace's own message generation, dynamic plugin discovery, Python runtime
layout, and simulation/MoveIt behavior.

## Issue register

### P0 — generated interfaces must be correct and consumable

The workspace's custom messages sit beneath both C++ and Python packages.
Incorrect ROSIDL code generation or runfiles/import layout invalidates the
migration. Spec 02 must establish this before any broad conversion.

### P0 — runtime plugin and launch behavior can diverge from compilation

Successful Bazel compilation alone does not prove that pluginlib can find the
hardware plugin, or that ROS launch, Gazebo, and MoveIt locate resources at
runtime. Spec 03 must run the existing behavior tests under Bazel-produced
artifacts before a Bazel build can be promoted.

### P1 — upstream branch availability and regressions

PR #558 is open and may change. A project fork pinned by full commit SHA is
required. Updates must be promoted only through the compatibility suite and a
recorded changelog.

### P1 — dual-build drift

During migration, colcon and Bazel can silently compile different source lists,
definitions, generated files, plugins, or runtime data. Specs must give each
target a parity test and explicitly name the authoritative source locations.

### P2 — developer and CI cost

Bazel's dependency fetching, disk footprint, test execution model, and editor
integration may increase local setup cost. Capture cold and incremental timing,
cache hit rate, and documented setup steps before making Bazel required.

## Program order

| Order | Spec                                                                         | Outcome                                                                       |
| ----- | ---------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| 1     | [01 — Fork and Bazel foundation](01-fork-and-bazel-foundation.md)            | A pinned, reproducible Jazzy rules base and a non-default Bazel entry point.  |
| 2     | [02 — Interfaces and control proof](02-interfaces-and-control-proof.md)      | Custom messages and the ros2_control plugin build and run from Bazel outputs. |
| 3     | [03 — Runtime parity and rollout decision](03-runtime-parity-and-rollout.md) | Launch, simulation, MoveIt, Python, CI, and release decision are validated.   |

## Global validation criteria

The migration is not complete merely when `bazel build //...` succeeds. Before
Bazel may become a required CI build, all of the following must hold:

- [ ] The rules fork and every fetched dependency are pinned by immutable
      revision and integrity hash; a documented update procedure verifies
      source provenance and licenses.
- [ ] A clean checkout builds the selected Bazel target set without consuming
      artifacts from `build/`, `install/`, or `log/`.
- [ ] Bazel generates and compiles every `drqp_interfaces` message for C++ and
      Python consumers; the generated types have the same ROS type names and
      wire-compatible behavior as the colcon baseline.
- [ ] The `drqp_control` hardware plugin is discoverable and loadable by a
      ROS 2 process built and launched from Bazel outputs.
- [ ] Existing focused unit, launch, and simulation tests pass under Bazel for
      the selected migration surface, with equivalent assertions and no
      unconditional skips.
- [ ] The Gazebo and MoveIt smoke tests demonstrate resource, parameter,
      URDF/xacro, plugin, and launch-file discovery from Bazel outputs.
- [ ] Python nodes import and execute with Python 3.12 from Bazel runfiles;
      console entry points and package resources remain available.
- [ ] Colcon and Bazel jobs run independently in CI during the transition, and
      both pass on the same commit for at least 20 consecutive default-branch
      builds before a promotion decision.
- [ ] Cold-build duration, no-op incremental duration, remote-cache hit rate
      when enabled, disk consumption, and developer setup are measured and
      reviewed against the documented reason for adopting Bazel.
- [ ] A rollback is documented: disabling the optional Bazel job restores the
      existing colcon release path without source or artifact conversion.

## Scope boundaries

- Replacing ROS 2 middleware or its package/resource-index conventions.
- Removing ament metadata or colcon builds during the proof stages.
- Treating a Bazel build as a substitute for hardware safety validation.
- Migrating every package before custom interfaces, plugin loading, and runtime
  launch behavior have passed their gates.

## Sources

- ROS 2, [the ament build-system design](https://design.ros2.org/articles/ament.html).
- mvukov/rules_ros2, [Jazzy support pull request #558](https://github.com/mvukov/rules_ros2/pull/558).
- mvukov/rules_ros2, [OMG IDL support pull request #617](https://github.com/mvukov/rules_ros2/pull/617).
