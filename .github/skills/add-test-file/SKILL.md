---
name: add-test-file
description: Add unit or integration test file with fixtures, test cases, and build integration for C++ or Python. Use when adding tests for existing source files, following TDD workflow, or need GTest/pytest scaffolding with CMakeLists.txt or setup.py integration.
---

# Add Test File

Generate a test file with appropriate fixtures, test cases, and build system integration for an existing source file in a ROS 2 package.

## When to Use This Skill

- Adding tests for an existing source file
- Creating unit tests for classes or functions
- Setting up integration tests for ROS 2 nodes
- Need test fixtures and scaffolding that match project conventions
- Following TDD workflow (write tests first)

## Prerequisites

- Source file exists and location is known
- Package build files (CMakeLists.txt or setup.py) accessible
- Understand unit vs integration test needs

## Inputs

- **Source File**: Path to source (or current file in editor)
- **Test Type**: `unit` or `integration`
- **Package Name**: Derive from path or ask
- **Test Name**: From source file name or custom

## Workflow

### Step 1: Analyze Source File

1. Read source file path
2. Determine type (C++ `.cpp/.hpp` or Python `.py`)
3. Extract package name from path
4. Identify: classes, public methods, free functions, ROS 2 nodes, dependencies

### Step 2: Determine Test Location

- C++: `<package_root>/test/test_<source_base>.cpp`
- Python: `<package_root>/test/test_<source_base>.py`

### Step 3: Check Existing Tests

If exists: offer to append or create `test_<name>_integration.cpp` etc.

### Step 4: Generate Test File

**C++ Unit:**

```cpp
#include "<package_name>/<header>.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class <ClassName>Test : public ::testing::Test { ... };
TEST_F(<ClassName>Test, ConstructorInitializesCorrectly) { ... }
TEST_F(<ClassName>Test, MethodNameHandlesValidInput) { ... }
TEST_F(<ClassName>Test, MethodNameHandlesInvalidInput) { ... }
```

**C++ Integration:** Include `rclcpp/rclcpp.hpp`, init/shutdown in SetUp/TearDown.

**Python Unit:** pytest fixtures, `Test<ClassName>` class, assert patterns.

**Python Node Integration:** Use pytest class-level setup/teardown for rclpy init/shutdown.
Use `@pytest.fixture(autouse=True)` with `request.addfinalizer()` for per-test node/publisher/subscription cleanup.

**Python Launch Integration:** Use `launch_pytest` for tests that launch ROS 2 nodes via `LaunchDescription`.
Write launch tests as **plain functions** (not test classes), share a node/harness through a `@pytest.fixture`,
and **verify per-process exit codes** with `drqp_launch_testing` (`<test_depend>drqp_launch_testing</test_depend>`).
Never ship a no-op `shutdown=True` test: that mechanism only checks the aggregate launch _service_ return code,
not that processes exited 0. Full rules + an executable behavior matrix: see
[launch-testing.instructions.md](../../instructions/launch-testing.instructions.md) and
`packages/runtime/drqp_launch_testing/test/shutdown_behavior/SPEC.md`.

Choose the pattern by test count:

- **Single launch test → function scope (default) + generator test** (`yield` once: body before shutdown,
  exit-code check after, on the same simulation). No separate shutdown function.
- **Several tests sharing one expensive simulation → `scope='module'` + a separate `shutdown=True` test.**
  Only use module scope to share a simulation; function scope is the default.

Do NOT pair a separate `shutdown=True` function with a function-/class-scoped fixture (it launches a different,
throwaway simulation), and do NOT make a non-function-scoped test a generator (raises `TypeError`).

**Fixture ordering gotcha:** a shared/autouse fixture that assumes the launched system is running MUST declare
`generate_test_description` as a parameter (value unused, `# noqa: ARG002`); `@pytest.mark.launch` is only an
inherited marker and does not guarantee the launch starts first. A `ready_delay` (`TimerAction` before
`ReadyToTest`) is required so processes are up before tests and the shutdown body sees populated exit codes.

```python
import launch_pytest
import pytest
import rclpy
from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node


@launch_pytest.fixture  # function scope (default)
def generate_test_description():
    launch_description = LaunchDescription([
        Node(package='<pkg>', executable='<exe>', output='screen'),
        TimerAction(period=2.0, actions=[ReadyToTest()]),
    ])
    return launch_description, track_process_exit_codes(launch_description)


@pytest.fixture
def consumer(generate_test_description):  # noqa: ARG001 (drives the launch)
    rclpy.init()
    node = rclpy.create_node('test_<node_class>_consumer')
    yield node
    rclpy.try_shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_<node_class>(consumer, generate_test_description):
    # ... exercise the launched node via `consumer` ...
    yield  # simulation tears down here
    assert_processes_exited_cleanly(generate_test_description[1])
```

For the multi-test (module-scope) variant, see the instructions file linked above.

### Step 5: Update Build Configuration

C++: Add `ament_add_gmock(test_<name> test/test_<name>.cpp)` and dependencies; call `drqp_test_enable_coverage(test_<name>)`. Python: Auto-discovered by pytest; add test deps to package.xml if needed.

### Step 6: Verify package.xml Dependencies

C++: `ament_lint_auto`, `drqp_lint_common`, `ament_cmake_gmock`, `ament_cmake_pytest`. Python: `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

## Edge Cases

- **Existing test**: Append or create separate file
- **No public interface**: Warn
- **Missing deps**: Add to package.xml
- **Header-only (C++)**: Test the header

## Related Resources

- [find-test-files](../find-test-files/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
- [Engineering guidelines](../../instructions/engineering.instructions.md)
