# Spec 04: C++ libraries and control plugin

- **Status**: proposed
- **Depends on**: [Spec 03](03-ros2-control-rules-support.md)
- **Packages**: `drqp_rapidjson`, `drqp_serial`, `drqp_a1_16_driver`, `drqp_control`
- **Size**: M

## Objective

Build the complete first-party C++ dependency chain for `drqp_control`, package
its runtime data, and prove pluginlib can discover and instantiate the hardware
interface from Bazel runfiles. Preserve the existing source files, compiler
standards, warnings, test cases, and colcon behavior.

## Public Bazel label contract

Create BUILD files in the existing ROS package directories with these labels:

| Package             | Public labels                                                                                                                           |
| ------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `drqp_rapidjson`    | `:drqp_rapidjson`                                                                                                                       |
| `drqp_serial`       | `:drqp_serial`, `:drqp_serial_tests`                                                                                                    |
| `drqp_a1_16_driver` | `:drqp_a1_16_driver`, `:drqp_a1_16_driver_tests`                                                                                        |
| `drqp_control`      | `:drqp_control`, `:control`, `:drqp_ros2_control_plugin`, `:runtime_data`, `:control_tests`; reserve `:control_launch_test` for Spec 05 |

Downstream labels therefore include, for example,
`//packages/runtime/drqp_control:drqp_ros2_control_plugin`.

## Source and build mapping

Translate the current CMake targets without changing ownership:

- `drqp_rapidjson`: header-only library over the checked-in
  `include/drqp_rapidjson/**`; retain Tencent license data.
- `drqp_serial`: the seven `.cpp` files in its current `add_library`, public
  headers, C++17, PIC, Threads, platform `rt`, Boost.Asio, and
  `-Wno-unused-parameter`.
- `drqp_a1_16_driver`: `MockServo.cpp`, `XYZrobotServo.cpp`, public headers,
  C++17, `-Wno-invalid-offsetof`, and dependencies on serial, rclcpp, and
  `drqp_interfaces:interfaces_cpp`.
- `drqp_control`: `RobotConfig.cpp` as the C++20 `drqp_control` library,
  `Control.cpp` as `control`, and `a1_16_hardware_interface.cpp` as the shared
  ros2_control plugin. Preserve yaml-cpp, rclcpp/lifecycle, sensor/std message,
  ament-index, pluginlib, hardware-interface, and driver dependencies.

Keep the repository warning baseline (`-Wall -Wextra -Wpedantic`) and the
target-specific exceptions above. Do not raise or lower a target's C++ standard
to make dependency resolution easier.

Use explicit source/header lists. A non-recursive header glob is permitted only
for the vendored RapidJSON tree if every matched file remains a checked-in input
and the evidence records the query result.

## Runtime data and plugin metadata

`//packages/runtime/drqp_control:runtime_data` must explicitly include every
checked-in file under `config/`, `launch/`, `urdf/`, `meshes/`, and `rviz/`,
plus `package.xml` and `drqp_control_plugin.xml`. It must contribute ament-index
package metadata in the Bazel runfiles environment.

The existing XML is the source of truth:

- library: `drqp_ros2_control`;
- class name: `drqp_control/a1_16_hardware_interface`;
- class type: `drqp_control::a1_16_hardware_interface`;
- base: `hardware_interface::SystemInterface`.

Configure the fork's `ros2_plugin` rule with the same values. Add a parity test
that parses the checked-in XML and compares it with the class loader's declared
classes so duplicated metadata cannot drift silently.

No target may locate data by an absolute source-tree path. Where an existing
test uses `TEST_DATA_DIR_IN_SOURCE_TREE`, refactor the shared test fixture to
accept a runfiles-resolved data path and update the CMake registration to pass
the installed/build-tree equivalent. Do not duplicate the test body.

## Tests to wire before implementation

Reuse the existing test sources and assertions:

- `drqp_serial/test/{TestSerialPort,TestSerialRecordingProxy,TestSerialTransferConfig}.cpp`
  with all JSON fixtures;
- `drqp_a1_16_driver/test/TestXYZrobotServo.cpp` with every JSON fixture, using
  its default mock/recording path and never real hardware;
- `drqp_control/test/TestRobotConfig.cpp` with its three YAML fixtures;
- `drqp_control/test/test_a1_16_hardware_interface_unit.cpp` with GMock;
- `drqp_control/test/test_urdf.py`, resolving xacro and meshes from Bazel
  runtime data.

Pin a Catch2 dependency compatible with the current `catch_ros2` test sources.
A narrow Bazel-only compatibility header/target under `bazel/compat/catch_ros2`
is allowed; rewriting test cases or maintaining Bazel-only copies is not.

Add one new C++ plugin-discovery test. It must:

1. create a `pluginlib::ClassLoader<hardware_interface::SystemInterface>`;
2. assert the exact public class name is declared;
3. instantiate it without a controller manager or serial device;
4. verify the instance type/base relationship; and
5. exit with no dependency on `/opt/ros/jazzy` or a colcon overlay.

Aggregate all tests in
`//packages/runtime/drqp_control:control_tests`. The full controller-manager
launch test remains registered under colcon; reserve public label
`//packages/runtime/drqp_control:control_launch_test` for Spec 05, after the
repository's patched launch stack is available to Bazel.

## Verification

```bash
scripts/bazel.sh build \
  //packages/runtime/drqp_control:control \
  //packages/runtime/drqp_control:drqp_ros2_control_plugin
scripts/bazel.sh test //packages/runtime/drqp_control:control_tests
scripts/bazel.sh clean --expunge
scripts/bazel.sh test //packages/runtime/drqp_control:control_tests
scripts/with-ros-env.sh colcon build --packages-up-to drqp_control
scripts/with-ros-env.sh colcon test --packages-select \
  drqp_serial drqp_a1_16_driver drqp_control
scripts/with-ros-env.sh colcon test-result --verbose
```

Delete workspace colcon outputs and repeat the Bazel suite. Record public
labels, source/data inventory, compiler versions, Catch2 compatibility choice,
plugin declaration output, and both test results in
`evidence/04-control-plugin.md`.

## Allowed files

- BUILD files and narrowly required shared test-fixture changes in the four
  owning packages;
- `MODULE.bazel`, its lock, and `bazel/compat/**` for immutable C++ test
  dependencies;
- fork/module pin files only for a proven generic ros2_control rules fix;
- this program's evidence/status documentation.

Do not migrate joy, Python applications, broad launch tests, Gazebo, or MoveIt.

## Acceptance criteria

- [ ] All four libraries/executables build with the same source sets, language
      standards, warning policy, and dependency visibility as CMake.
- [ ] Every runtime resource is a declared input and xacro validation succeeds
      without a source-tree path.
- [ ] Existing serial, driver, RobotConfig, hardware lifecycle, and URDF tests
      pass from shared sources under both build systems.
- [ ] Plugin metadata matches the checked-in XML and pluginlib instantiates
      `drqp_control/a1_16_hardware_interface` from Bazel runfiles.
- [ ] No test opens real servo hardware or reads workspace colcon output.
- [ ] `evidence/04-control-plugin.md` contains the required handoff data.
