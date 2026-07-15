---
name: add-test-file-cpp
description: Add or update a focused C++ unit test and its ament_cmake integration in this ROS 2 workspace. Use for testing existing C++ libraries, classes, or functions with the repository's Catch2 convention, or with GMock when mocks are necessary; use launch-testing for tests that start ROS processes.
---

# Add a C++ Test File

Test public behavior through the package library target. Keep each test file
focused on one component or behavior; split unrelated concerns into separate
test targets.

Use [add-test-file-python](../add-test-file-python/) for Python code and
[launch-testing](../launch-testing/) for a launch, node, or multi-process
integration test. A C++ test that merely calls a node class directly remains a
unit test; it must initialise and shut down rclcpp safely when needed.

## Determine the test shape

Require the source/header path and package name; derive the package only when
the path unambiguously identifies it. Inspect the public API and the package's
existing `CMakeLists.txt` before choosing the framework.

The workspace predominantly uses Catch2 through `catch_ros2` for normal C++
unit tests. Use it by default. Choose GMock only when a collaborator must be
mocked or call expectations are central to the behavior under test. Do not
mix frameworks in a target and do not add `ament_cmake_pytest` for C++ tests.

Place the file at:

```
<package_root>/test/test_<component>.cpp
```

Match the package's public-header extension; the usual convention here is
`#include "<package>/<component>.h"`.

## Add a Catch2 unit test

Add `<test_depend>catch_ros2</test_depend>` to `package.xml`. In the package's
top-level `CMakeLists.txt`, ensure the `BUILD_TESTING` block includes the
workspace helpers, then define and link the test target:

```cmake
if(BUILD_TESTING)
  include(../../cmake/ClangCoverage.cmake)
  include(../../cmake/Catch2Extras.cmake)
  find_package(catch_ros2 REQUIRED)

  add_executable(test_<component> test/test_<component>.cpp)
  target_link_libraries(test_<component> PRIVATE
    <library_target>
    catch_ros2::catch_ros2_with_main)
  ament_target_dependencies(test_<component> <ros_dependencies_used_by_test>)
  drqp_test_enable_coverage(test_<component>)
  add_catch2_unit_test(test_<component>)
endif()
```

This coverage include is valid only for packages at
`packages/<group>/<package>`. For any other package location, omit the two
coverage lines until its project-level coverage helper is identified. Link the
library target even if it is already linked to ROS dependencies, and list any
additional direct ROS dependencies required by the test.

Start a focused Catch2 file with the repository headers:

```cpp
#include <catch_ros2/catch.hpp>

#include "<package>/<component>.h"

TEST_CASE("<Component> handles valid input")
{
  const <Component> component{/* required configuration */};

  CHECK(component.operation(/* input */) == /* expected */);
}
```

## Add a GMock test only when mocks are necessary

Add `<test_depend>ament_cmake_gmock</test_depend>` and use this complete
target recipe:

```cmake
if(BUILD_TESTING)
  include(../../cmake/ClangCoverage.cmake)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_<component> test/test_<component>.cpp)
  target_link_libraries(test_<component> PRIVATE <library_target>)
  ament_target_dependencies(test_<component> <ros_dependencies_used_by_test>)
  drqp_test_enable_coverage(test_<component>)
endif()
```

Use `#include <gmock/gmock.h>`, fixture setup/teardown only for state shared
by each test, and explicit expectations on mocked collaborators. `ament_add_gmock`
provides the GTest runner; do not add a separate `main` or link another test
main. Apply the same package-location restriction to the coverage lines.

## Validate

Run the smallest relevant package commands from the workspace root:

```bash
scripts/with-ros-env.sh python3 -m colcon build --packages-up-to <package_name>
scripts/with-ros-env.sh python3 -m colcon test --packages-select <package_name>
scripts/with-ros-env.sh python3 -m colcon test-result --verbose
```

For failures, read `log/latest_test/<package_name>/stdout_stderr.log` or the
timestamped `streams.log`. Use [ros2-workspace-testing](../ros2-workspace-testing/)
for reruns, coverage, or ROS-environment escalation.

## Related resources

- [find-test-files](../find-test-files/)
- [launch-testing](../launch-testing/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
