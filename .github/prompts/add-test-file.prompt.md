---
description: 'Add unit or integration test file with fixtures, test cases, and build integration for C++ or Python'
name: 'add-test-file'
agent: 'agent'
tools: ['read', 'edit', 'search']
argument-hint: '<source-file> [--type unit|integration] [--package package-name]'
---

# Add Test File

Generate a test file with appropriate fixtures, test cases, and build system integration for an existing source file in a ROS 2 package.

## When to Use This Prompt

- Adding tests for an existing source file
- Creating unit tests for classes or functions
- Setting up integration tests for ROS 2 nodes
- Need test fixtures and scaffolding that match project conventions
- Following TDD workflow (write tests first)

## Prerequisites

- Source file exists and you know its location
- Package build files (CMakeLists.txt or setup.py) are accessible
- Understand whether writing unit or integration tests
- Have basic knowledge of the source file's public interface

## Inputs

### Required Inputs

- **Source File** `${input:sourceFile:${file}}`: Path to source file to test (can use current file in editor)
- **Test Type** `${input:testType:unit}`: Type of test (`unit` or `integration`)

### Optional Inputs

- **Package Name**: Derive from source file path or ask user
- **Test Name**: Generate from source file name or specify custom name
- **Additional Fixtures**: Specific test fixtures or mocks needed

## Workflow

### Step 1: Analyze Source File

1. Read the source file path from input or `${file}` context
2. Determine file type (C++ `.cpp/.hpp/.h` or Python `.py`)
3. Extract package name from file path (e.g., `packages/runtime/drqp_control/`)
4. Parse source file to identify:
   - Classes and their public methods
   - Free functions
   - ROS 2 nodes and their topics/services
   - Dependencies and includes

### Step 2: Determine Test File Location and Name

#### For C++ Files:
- Test location: `<package_root>/test/`
- Test file name: `test_<source_base_name>.cpp`
- Example: `src/MyClass.cpp` → `test/test_MyClass.cpp`

#### For Python Files:
- Test location: `<package_root>/test/`
- Test file name: `test_<source_base_name>.py`
- Example: `my_module/my_class.py` → `test/test_my_class.py`

### Step 3: Check for Existing Test File

1. Check if test file already exists
2. If exists:
   - Offer to append new test cases
   - Or create separate test file with suffix (e.g., `test_MyClass_integration.cpp`)
3. If doesn't exist, proceed with creation

### Step 4: Generate Test File Content

#### For C++ Unit Tests:

Generate test file with:
```cpp
// Copyright notice (match project style)

#include "<package_name>/<header_file>.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace <package_name> {

class <ClassName>Test : public ::testing::Test {
protected:
  void SetUp() override {
    // Setup test fixtures
  }

  void TearDown() override {
    // Cleanup
  }

  // Test fixture members
};

TEST_F(<ClassName>Test, ConstructorInitializesCorrectly) {
  // Test basic construction
  EXPECT_NO_THROW({
    // Create instance
  });
}

TEST_F(<ClassName>Test, MethodNameHandlesValidInput) {
  // Test normal operation
  // EXPECT_EQ, EXPECT_TRUE, etc.
}

TEST_F(<ClassName>Test, MethodNameHandlesInvalidInput) {
  // Test edge cases
}

TEST_F(<ClassName>Test, MethodNameHandlesEmptyInput) {
  // Test boundary conditions
}

}  // namespace <package_name>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

#### For C++ Integration Tests:

Include ROS 2 test utilities:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

class <NodeName>IntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(<NodeName>IntegrationTest, NodePublishesCorrectly) {
  // Integration test with ROS 2 infrastructure
}
```

#### For Python Unit Tests:

Generate pytest test file:
```python
"""Tests for <module_name>."""

import pytest
from <package_name>.<module_path> import <ClassName>


class Test<ClassName>:
    """Test suite for <ClassName>."""

    @pytest.fixture
    def instance(self):
        """Create instance for testing."""
        return <ClassName>()

    def test_constructor_initializes_correctly(self, instance):
        """Test that constructor initializes object correctly."""
        assert instance is not None

    def test_method_name_with_valid_input(self, instance):
        """Test method with valid input."""
        result = instance.method_name(valid_input)
        assert result == expected_value

    def test_method_name_with_invalid_input(self, instance):
        """Test method handles invalid input."""
        with pytest.raises(ValueError):
            instance.method_name(invalid_input)

    def test_method_name_with_empty_input(self, instance):
        """Test method handles empty input."""
        result = instance.method_name([])
        assert result == []
```

#### For Python Integration Tests (ROS 2):

```python
"""Integration tests for <node_name>."""

import pytest
import rclpy
from rclpy.node import Node
from <package_name>.<module> import <NodeClass>


class Test<NodeClass>Integration:
    """Integration test suite for <NodeClass>."""

    @pytest.fixture
    def ros_context(self):
        """Initialize ROS context."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_node_publishes_messages(self, ros_context):
        """Test that node publishes messages correctly."""
        node = <NodeClass>()
        # Integration test logic
        node.destroy_node()
```

### Step 5: Update Build Configuration

#### For C++ Tests (Update CMakeLists.txt):

1. Find the testing section (usually after `if(BUILD_TESTING)`)
2. Add test executable:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add this for the new test
  ament_add_gmock(test_<name>
    test/test_<name>.cpp
  )
  target_link_libraries(test_<name>
    <library_name>
  )
  ament_target_dependencies(test_<name>
    <dependencies>
  )
  drqp_test_enable_coverage(test_<name>)
endif()
```

3. Ensure coverage macro is called: `drqp_test_enable_coverage(test_<name>)`

#### For Python Tests:

Python tests are auto-discovered by pytest. No setup.py changes needed unless:
- Adding new test dependencies to package.xml
- Configuring special pytest options in setup.cfg

### Step 6: Verify Test Dependencies in package.xml

Ensure test dependencies are present:

#### For C++:
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>drqp_lint_common</test_depend>
<test_depend>ament_cmake_gmock</test_depend>
<test_depend>ament_cmake_pytest</test_depend>
```

#### For Python:
```xml
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
<test_depend>python3-pytest</test_depend>
```

Add missing dependencies if needed.

## Output Expectations

### Success Criteria

- Test file created with appropriate structure
- Test fixtures and scaffolding match project conventions
- Build system updated (for C++)
- Test dependencies verified in package.xml
- Test file includes multiple test cases covering:
  - Normal/happy path
  - Edge cases
  - Error conditions
  - Boundary conditions

### Generated Output Summary

```
✅ Created test file: <test_file_path>
📋 Test type: <unit|integration>
🎯 Testing: <source_file>

📝 Generated test cases:
   - Constructor/initialization test
   - Valid input test
   - Invalid input test
   - Edge case tests

🔨 Build integration:
   - Updated CMakeLists.txt with test target (C++)
   - Test auto-discovery configured (Python)
   - Coverage enabled

🧪 Run tests:
   colcon test --packages-select <package_name>
   colcon test-result --verbose
```

## Validation Steps

1. **Verify test file exists**: Check file was created in correct location
2. **Check test structure**: Ensure test fixtures and cases are present
3. **Validate build integration**: For C++, verify CMakeLists.txt was updated
4. **Build tests**: Run `colcon build --packages-select <package_name>`
5. **Run tests**: Execute `colcon test --packages-select <package_name>`
6. **Check test discovery**: Verify tests are found and executed

## Quality Assurance

- [ ] Test file follows naming convention (`test_*.cpp` or `test_*.py`)
- [ ] Test class/suite name matches source file
- [ ] Multiple test cases cover different scenarios
- [ ] Test fixtures properly set up and torn down
- [ ] For C++: CMakeLists.txt updated with test target
- [ ] For C++: Coverage macro called (`drqp_test_enable_coverage`)
- [ ] Test dependencies verified in package.xml
- [ ] Tests include assertions (EXPECT_*, assert)
- [ ] Test names are descriptive and follow convention
- [ ] Integration tests include ROS 2 initialization/cleanup

## Edge Cases

- **Existing test file**: Offer to append or create separate file
- **No public interface**: Source file has no testable public interface - warn user
- **Missing dependencies**: Test dependencies not in package.xml - add them
- **Complex fixtures**: Source requires complex setup - generate TODO comments
- **Header-only library** (C++): Create test for header file
- **ROS 2 node without class**: Create functional tests for node executable

## Related Resources

- [find-test-files skill](/.github/skills/find-test-files/SKILL.md)
- [ros2-workspace-testing skill](/.github/skills/ros2-workspace-testing/SKILL.md)
- [TDD Red agent](/.github/agents/tdd-red.agent.md) - For test-first development
- [Engineering guidelines](/.github/instructions/engineering.instructions.md)
- [GTest Documentation](https://google.github.io/googletest/)
- [Pytest Documentation](https://docs.pytest.org/)
