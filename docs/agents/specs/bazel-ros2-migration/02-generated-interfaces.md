# Spec 02: Generated interfaces

- **Status**: proposed
- **Depends on**: [Spec 01](01-fork-and-bazel-foundation.md)
- **Packages**: `drqp_interfaces`
- **Size**: M

## Objective

Prove that the pinned Jazzy rules generate usable C, C++, and Python artifacts
for every custom message and that C++ and Python processes communicate with the
same type support. Stop here if interface generation is not trustworthy.

## Source-of-truth inventory

The BUILD graph must list these files explicitly:

| Message                            | External interface dependency |
| ---------------------------------- | ----------------------------- |
| `msg/HapticEffect.msg`             | none                          |
| `msg/MovementCommandConstants.msg` | none                          |
| `msg/MovementCommand.msg`          | `geometry_msgs/Vector3`       |
| `msg/RobotCommandConstants.msg`    | none                          |
| `msg/RobotCommand.msg`             | `std_msgs/Header`             |

Preserve package name `drqp_interfaces` and ROS names
`drqp_interfaces/msg/<Message>`. Do not check generated code into the source
tree and do not glob generator outputs.

## Public Bazel label contract

Create `packages/runtime/drqp_interfaces/BUILD.bazel` with these public labels:

- `//packages/runtime/drqp_interfaces:interfaces` —
  `ros2_interface_library` containing all five message files;
- `//packages/runtime/drqp_interfaces:interfaces_c` — C generated/type-support
  closure;
- `//packages/runtime/drqp_interfaces:interfaces_cpp` — C++ generated/type-
  support closure;
- `//packages/runtime/drqp_interfaces:interfaces_py` — Python generated/type-
  support closure;
- `//packages/runtime/drqp_interfaces:interface_tests` — test suite aggregating
  the three tests below.

Use `@ros2_common_interfaces//:geometry_msgs` and
`@ros2_common_interfaces//:std_msgs` as explicit base-interface dependencies;
use the corresponding `c_*`, `cpp_*`, and `py_*` targets for generated consumer
closures. Record any unavoidable first-party label deviation in
`evidence/02-interfaces.md` and keep the five public first-party labels stable
for downstream specs.

## Tests to add first

Put shared semantic test sources under
`packages/runtime/drqp_interfaces/test/` and register them in both Bazel and
ament/CMake. Add only the required test dependencies to `package.xml`:
`ament_cmake_gtest`, `ament_cmake_pytest`, `launch_pytest`,
`drqp_launch_testing`, and the ROS serialization/runtime packages actually
included by the test sources. Do not add test-only packages as runtime
dependencies.

### 1. C and C++ generation test

Create `test/test_generated_interfaces.cpp`, a focused GTest that:

- includes the generated C and C++ headers for every message;
- initializes/finalizes each C message struct;
- default-constructs and assigns representative values to each C++ message;
- asserts `rosidl_generator_traits::name<T>()` equals the fully qualified ROS
  name for all five types;
- serializes and deserializes `MovementCommand` and `RobotCommand` through ROS
  serialization and compares every populated field.

Use non-default representative values, including a `RobotCommand.header` stamp,
`RobotCommand.command`, all `MovementCommand` vectors, rotation, and gait. This
test must link the generated type support rather than passing as a header-only
compile check.

Public label:
`//packages/runtime/drqp_interfaces:interfaces_cpp_test`.

### 2. Python generation test

Create `test/test_generated_interfaces.py`, a pytest that imports all five
classes from `drqp_interfaces.msg`, then resolves each with
`rosidl_runtime_py.utilities.get_message` using its full ROS name. Populate and
round-trip the same `MovementCommand` and `RobotCommand` values used by the C++
test with `rclpy.serialization.serialize_message` and `deserialize_message`.

Public label:
`//packages/runtime/drqp_interfaces:interfaces_py_test`.

### 3. C++ to Python transport test

Create `test/interface_transport_publisher.cpp` and
`test/test_cross_language_transport.py`. The publisher sends one
`RobotCommand` with a fixed stamp and command string on a test-private topic
using reliable, transient-local QoS. The launch-pytest subscriber waits with a
bounded timeout, compares every field, and requests shutdown. Assert the
publisher and subscriber exit cleanly through `drqp_launch_testing` on the
colcon path and equivalent per-process checks on the Bazel path.

The test must use an isolated `ROS_DOMAIN_ID`, be tagged `requires-network`,
and have a 60-second Bazel/CTest timeout. It must not invoke `ros2 topic`, use a
workspace overlay, or depend on timing sleeps for correctness.

Public label:
`//packages/runtime/drqp_interfaces:interfaces_cross_language_test`.

## Known upstream failure gate

PR #558 has a report of conflicting generator actions when the C, C++, and
Python aspects consume the same interface target. Build all three closures in
one invocation specifically to exercise that case:

```bash
scripts/bazel.sh build \
  //packages/runtime/drqp_interfaces:interfaces_c \
  //packages/runtime/drqp_interfaces:interfaces_cpp \
  //packages/runtime/drqp_interfaces:interfaces_py
```

If it fails in the rules implementation:

1. reduce the failure to a minimal target in the Dr-QP rules fork;
2. add a rules-fork regression test;
3. contribute or link the generic fix upstream;
4. document the local patch in `UPSTREAM.md`;
5. update the immutable fork revision, archive integrity, module lock, and
   `evidence/02-interfaces.md` together.

Do not work around the collision by generating one language per build,
checking in output, disabling type description generation, or making the
targets mutually exclusive.

## Verification

```bash
scripts/bazel.sh test //packages/runtime/drqp_interfaces:interface_tests
scripts/bazel.sh clean --expunge
scripts/bazel.sh test //packages/runtime/drqp_interfaces:interface_tests
scripts/with-ros-env.sh colcon build --packages-up-to drqp_interfaces
scripts/with-ros-env.sh colcon test --packages-select drqp_interfaces
scripts/with-ros-env.sh colcon test-result --verbose
```

After the Bazel run, remove any workspace `build/`, `install/`, and `log/`, then
repeat the Bazel suite to prove it consumes only declared inputs.

Record target labels, exact ROS type-name outputs, both test-system results,
architecture, and any fork patch in `evidence/02-interfaces.md`.

## Allowed files

- `packages/runtime/drqp_interfaces/BUILD.bazel`, its existing CMake/manifest,
  and new focused test sources;
- shared Bazel helpers under `bazel/` only when specific to ROSIDL;
- fork/module pin files only for a proven upstream generator fix;
- this program's evidence/status documentation.

Do not touch control, joy, Gazebo, MoveIt, or application Python packages.

## Acceptance criteria

- [ ] One Bazel invocation generates C, C++, and Python support for all five
      messages without conflicting actions.
- [ ] Generated C structs compile/link and generated C++ types report the exact
      five fully qualified ROS names.
- [ ] Python imports, full-name resolution, serialization, and deserialization
      pass for all five types.
- [ ] The representative C++ publisher/Python subscriber exchange passes with
      bounded waits and clean process exits.
- [ ] The same semantic test sources pass under Bazel and colcon without skips
      or changed assertions.
- [ ] Deleting colcon outputs does not change the Bazel result.
- [ ] `evidence/02-interfaces.md` contains the required handoff data.
