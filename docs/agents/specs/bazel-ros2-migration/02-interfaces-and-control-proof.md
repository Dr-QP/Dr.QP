# Spec 02: Interfaces and control proof

- **Status**: proposed
- **Depends on**: [Spec 01](01-fork-and-bazel-foundation.md)
- **Size**: L

## Objective

Prove that the Jazzy rules fork correctly generates the workspace's custom ROS
messages and can build, discover, and load the `drqp_control` ros2_control
hardware plugin.

## Scope

Convert the direct build graph for:

- `drqp_interfaces`, including all five `.msg` definitions;
- `drqp_serial`, `drqp_a1_16_driver`, and their direct build dependencies;
- `drqp_control`, its C++ libraries/tests, configuration, URDF/xacro files,
  and `drqp_control_plugin.xml`.

Do not migrate Gazebo, MoveIt, application Python packages, or broad launch
testing in this spec.

## Design requirements

### ROSIDL

- Model each interface package with the rules fork's supported interface rules.
- Generate C and C++ artifacts required by C++ consumers and Python artifacts
  required by Python consumers.
- Declare message files and message-package dependencies explicitly; no globbed
  generated files or manual checked-in generated code.
- Preserve package name `drqp_interfaces` and each existing fully qualified
  message type name.

### Control plugin and resources

- Build the hardware implementation as the shared-library form required by
  pluginlib/ros2_control.
- Declare `drqp_control_plugin.xml`, YAML, URDF/xacro, meshes, and launch
  resources as runtime data rather than relying on the source checkout's
  relative paths.
- Provide a Bazel run/test environment that makes the package and plugin
  discoverable without a colcon overlay. If a minimal compatibility wrapper is
  necessary, keep it inside the Bazel runfiles environment and document it.
- Preserve existing C++ compiler options, visibility, and test dependencies;
  do not weaken warnings or remove tests to make the conversion pass.

## Parity tests

Add focused Bazel tests in addition to retaining the existing colcon tests:

1. A C++ test constructs and serializes each custom message used by control.
2. A Python test imports every generated `drqp_interfaces.msg` type and
   validates its ROS type name.
3. A cross-language test publishes/serializes a representative
   `RobotCommand`, then verifies equivalent values on the receiving side.
4. A plugin-discovery test starts a minimal ros2_control process or equivalent
   loader and asserts that `drqp_control::a1_16_hardware_interface` is found
   through `drqp_control_plugin.xml`.
5. Existing focused `drqp_control` C++ tests execute under Bazel with the same
   test data and assertions as the colcon path.

Run the cross-language and plugin tests using the repository's ROS environment
requirements, but never source colcon `install/setup.bash`. When ROS sockets or
logs require it, use the repository's approved ROS execution environment.

## Test plan

- Build `drqp_interfaces` in an empty Bazel output base and inspect declared
  output labels rather than source paths.
- Run the C++, Python, cross-language, plugin-discovery, and existing C++ test
  sets under Bazel twice: from a clean output base and incrementally.
- Run the same semantic tests through colcon. Compare ROS type names, plugin
  class identifiers, and test outcomes; investigate every difference.
- Confirm that deleting `build/`, `install/`, and `log/` does not change a
  Bazel test outcome.

## Acceptance criteria

- [ ] All five custom `.msg` definitions generate C++, C, and Python support
      required by their current consumers.
- [ ] C++ and Python consumers use matching fully qualified ROS type names.
- [ ] A representative cross-language message exchange passes using only
      Bazel-produced artifacts.
- [ ] The hardware interface shared library and plugin description are present
      in runtime data and pluginlib loads the declared class successfully.
- [ ] Existing focused `drqp_control` C++ tests pass under both build systems
      without changed assertions or disabled warnings.
- [ ] Bazel execution remains independent of colcon outputs.
