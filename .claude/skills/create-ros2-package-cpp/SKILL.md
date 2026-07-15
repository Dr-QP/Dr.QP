---
name: create-ros2-package-cpp
description: Create a new ROS 2 C++ ament_cmake package in this workspace, including its manifest, CMake targets, and focused C++ unit-test scaffold. Use for a new rclcpp library or executable package; use the Python package skill for ament_python packages and launch-testing for process-level tests.
---

# Create a ROS 2 C++ Package

Create a package with a library-first layout unless the requested node has no
reusable logic. Keep production code in a library and make executables thin
entry points; this makes the library directly testable.

Use [create-ros2-package-python](../create-ros2-package-python/) for an
`ament_python` package. Route tests that start ROS processes to
[launch-testing](../launch-testing/), rather than adding a C++ unit-test
target for them.

## Gather and validate inputs

Require a package name, target location, description, license, maintainer,
and the ROS/package dependencies actually used. Package names are lowercase
letters, digits, and underscores; use the workspace's `drqp_` prefix for new
project packages. Stop if the destination already exists.

This scaffold's coverage example applies only to the normal layout
`packages/<group>/<package>` (for example, `packages/runtime/drqp_example`):
the workspace helper is then at `../../cmake/ClangCoverage.cmake`. For a
package elsewhere, do not copy that relative include or its coverage calls;
ask for the appropriate project-level coverage integration.

## Create the layout

```
<package>/
├── CMakeLists.txt
├── package.xml
├── include/<package>/<library>.h
├── src/<library>.cpp
├── src/<node>_main.cpp       # only when an executable is requested
├── test/test_<library>.cpp
├── launch/                   # only when a launch file is needed
└── config/                   # only when configuration is needed
```

Use the workspace's predominant `.h` public-header convention. Do not create
empty directories, placeholder files, or a README unless the package needs
user-facing setup or usage documentation.

## Configure build targets

Use `ament_package_xml()` to obtain manifest dependency groups, find each
build dependency, declare C++17 or a higher requested standard on every C++
target, and install headers and runnable executables. Adapt this pattern to
the actual targets and dependencies:

```cmake
cmake_minimum_required(VERSION 3.22)
project(<package_name>)

find_package(ament_cmake REQUIRED)
ament_package_xml()
foreach(dep ${${PROJECT_NAME}_BUILD_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(<library_target> src/<library>.cpp)
target_compile_features(<library_target> PUBLIC cxx_std_17)
target_include_directories(<library_target> PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(<library_target> ${${PROJECT_NAME}_BUILD_DEPENDS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS <library_target>
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  include(../../cmake/ClangCoverage.cmake)
  drqp_library_enable_coverage(<library_target>)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Add the focused test target described below.
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})
ament_package()
```

For an executable, use `add_executable`, link it to the library, apply
`ament_target_dependencies`, and install it to `lib/${PROJECT_NAME}`. Add only
the dependencies the code uses to `package.xml`; use `<depend>` unless the
dependency is genuinely build- or runtime-only. Include `ament_lint_auto` and
`drqp_lint_common` as test dependencies. Do not add `ament_cmake_pytest` to a
C++-only package.

## Add focused C++ tests

The current workspace default for ordinary C++ unit tests is Catch2 via
`catch_ros2`; see [add-test-file-cpp](../add-test-file-cpp/) for the complete
target recipe. Add `<test_depend>catch_ros2</test_depend>` when using it.
Use `ament_cmake_gmock` only when interaction assertions or mocks are needed,
and add its test dependency only then. Do not add a Python test framework for
either case.

For launch or multi-process behavior, add the launch-test dependencies that
[launch-testing](../launch-testing/) specifies and follow that skill instead.

## Validate

From the workspace root, use the ROS wrapper and incremental selectors:

```bash
scripts/with-ros-env.sh python3 -m colcon build --packages-up-to <package_name>
scripts/with-ros-env.sh python3 -m colcon test --packages-select <package_name>
scripts/with-ros-env.sh python3 -m colcon test-result --verbose
```

If ROS is unavailable locally, follow the escalation in
[ros2-workspace-build](../ros2-workspace-build/) or
[ros2-workspace-testing](../ros2-workspace-testing/). Inspect the package's
logs under `log/latest_build/` or `log/latest_test/` when a command fails.

## Related resources

- [add-test-file-cpp](../add-test-file-cpp/)
- [ros2-workspace-build](../ros2-workspace-build/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
- [AGENTS.md](../../../AGENTS.md)
