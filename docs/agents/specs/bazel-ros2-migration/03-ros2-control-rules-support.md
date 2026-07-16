# Spec 03: ros2_control rules support

- **Status**: proposed
- **Depends on**: [Spec 02](02-generated-interfaces.md)
- **Repositories**: `Dr-QP/rules_ros2` and module pin files in this workspace
- **Size**: M/L

## Objective

Extend the pinned Jazzy rules fork with the smallest source-built ros2_control
closure needed by `drqp_control`. Keep this dependency work out of the
first-party plugin PR so a rules-focused agent can validate ABI and target
coverage independently.

## Current gap

The candidate rules revision exposes rclcpp, pluginlib, common interfaces,
ament index, yaml-cpp, and launch, but it does not expose ros2_control or
ros2_controllers repositories. `drqp_control` cannot honestly compile against
`hardware_interface::SystemInterface` or run its controller-manager test until
that gap is closed.

Do not satisfy compile-time dependencies by importing headers or libraries from
`/opt/ros/jazzy`. Mixing the fork's source-built rclcpp/pluginlib with binary
ros2_control libraries creates an unproven ABI boundary. Gazebo and MoveIt may
use an explicit host boundary later; the hardware-interface ABI may not.

## Required dependency closure

Start from the same core Jazzy source snapshot used by the rules fork
(`release-jazzy-20250820`). Because that core manifest does not include
ros2_control, record the exact `ros/rosdistro` commit used to resolve compatible
Jazzy releases on that snapshot date, then pin the resulting source tags/SHAs
for this public target surface:

| Repository role    | Required public Bazel targets                                                                                           |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| ros2_control core  | `hardware_interface`, `controller_interface`, `controller_manager`, `controller_manager_msgs`, `transmission_interface` |
| ros2_controllers   | `joint_state_broadcaster`, `joint_trajectory_controller`, `battery_state_broadcaster`, `ros2_controllers_test_nodes`    |
| control interfaces | C/C++/Python support required for `control_msgs` and any controller-manager messages                                    |
| support libraries  | `realtime_tools`, `generate_parameter_library`, `control_toolbox`, and other dependencies reached by the named targets  |

Use repository names consistent with existing rules conventions, such as
`@ros2_control`, `@ros2_controllers`, and `@ros2_control_msgs`. The final exact
labels must be listed in the fork's documentation and
`evidence/03-ros2-control-rules.md`.

Build only the packages reached by the table. Do not add every controller,
hardware component, example, benchmark, or test from the upstream repositories.

## Fork implementation requirements

1. Add each source archive to the rules resolver with an immutable URL, SHA256,
   and strip prefix. Record license and provenance beside existing repository
   definitions.
2. Add explicit BUILD adapters following the fork's existing patterns for
   interface generation, C++ libraries, plugins, Python tools, and runtime data.
3. Export only headers, compile definitions, libraries, plugins, and data that
   an upstream CMake consumer of the named ROS package receives. Do not flatten
   include paths or make all targets globally visible to hide missing deps.
4. Preserve upstream plugin class names and base types. Controller plugins must
   be discoverable through the fork's ament setup/runfiles machinery.
5. Add all new repositories to the bzlmod extension and its documented
   `use_repo` list. Regenerate resolver output rather than hand-editing generated
   repository declarations without updating their source input.
6. Keep generic fixes in the fork, add them to `UPSTREAM.md`, and open/link an
   upstream issue or PR. Workspace-specific aliases belong in this workspace,
   not the shared fork.

## Tests to add first in the fork

Add a small rules integration example that contains:

- a one-joint URDF with a minimal mock implementation of
  `hardware_interface::SystemInterface`;
- a plugin target built with the fork's `ros2_plugin` rule;
- a C++ unit test that constructs the interface and exercises configure,
  activate, read, write, deactivate, and cleanup callbacks;
- a controller-manager launch test that loads the mock hardware plus
  `joint_state_broadcaster` and `joint_trajectory_controller`, verifies both
  become active, then checks every launched process exits cleanly.

Public fork test labels:

- `//ros2/test/ros2_control:hardware_interface_tests`
- `//ros2/test/ros2_control:controller_manager_tests`

The launch test is tagged `requires-network`, uses a unique `ROS_DOMAIN_ID`,
and has a 120-second timeout. It must run entirely from fork-built artifacts
without `/opt/ros/jazzy` or a sourced overlay.

## Workspace pin update

After the fork tests pass:

- update `MODULE.bazel` to the new immutable fork commit/archive integrity;
- regenerate `MODULE.bazel.lock`;
- import only the new repositories needed by the required target surface;
- update fork provenance in `bazel/README.md` and
  `evidence/03-ros2-control-rules.md`.

Do not add first-party package BUILD files in this spec.

## Verification

In a clean clone of the fork:

```bash
bazel test //ros2/test/ros2_control:hardware_interface_tests
bazel test //ros2/test/ros2_control:controller_manager_tests
bazel clean --expunge
bazel test //ros2/test/ros2_control:all
```

In this workspace after updating the pin:

```bash
scripts/bazel.sh mod graph
scripts/bazel.sh query '@ros2_control//...'
scripts/bazel.sh query '@ros2_controllers//...'
scripts/bazel.sh test //bazel/tests:all
```

Save the resolved repository versions, archive hashes, public labels, fork test
results on x86_64, and any unavailable aarch64 dependency in
`evidence/03-ros2-control-rules.md`.

## Stop conditions

Stop and hand off a minimal reproducer instead of weakening the design if:

- the required source versions cannot coexist with the fork's pinned Jazzy
  rclcpp/pluginlib/type-support versions;
- an upstream dependency requires an unpinned generator or network access
  during a build action;
- plugin discovery works only with `/opt/ros/jazzy` or colcon output; or
- the required controller-manager launch cannot shut down cleanly.

## Allowed files

- Rules fork repository definitions, BUILD adapters, patches, tests, docs, and
  generated resolver outputs for the named closure;
- this workspace's `MODULE.bazel`, `MODULE.bazel.lock`, `bazel/README.md`, and
  program evidence/status documentation.

## Acceptance criteria

- [ ] Every required source archive and transitive dependency is immutable and
      license/provenance data is recorded.
- [ ] The named hardware-interface and controller targets build from source on
      the fork's Jazzy rclcpp/pluginlib closure.
- [ ] The fork hardware lifecycle test passes.
- [ ] The controller-manager test loads both controllers through pluginlib and
      verifies clean process exits without host ROS libraries.
- [ ] This workspace pins the tested fork commit and locked dependency graph.
- [ ] `evidence/03-ros2-control-rules.md` contains the required handoff data.
