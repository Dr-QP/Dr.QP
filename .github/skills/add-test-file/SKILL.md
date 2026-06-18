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
Decorate `generate_test_description` with `@launch_pytest.fixture`; mark test classes with `@pytest.mark.launch(fixture=generate_test_description)`.
Add a shutdown test function decorated with `@pytest.mark.launch(fixture=generate_test_description, shutdown=True)`.

```python
import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node

@launch_pytest.fixture
def generate_test_description():
    return LaunchDescription([
        Node(package='<pkg>', executable='<exe>', output='screen'),
        TimerAction(period=2.0, actions=[ReadyToTest()]),
    ])

@pytest.mark.launch(fixture=generate_test_description)
class Test<NodeClass>:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.try_shutdown()

    @pytest.fixture(autouse=True)
    def _node_setup(self, request):
        self.node = rclpy.create_node('test_<node_class>_consumer')
        request.addfinalizer(self.node.destroy_node)

    def test_smoke(self):
        pass

@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_<node_class>_shutdown():
    pass
```

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
