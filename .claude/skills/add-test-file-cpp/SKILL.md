---
name: add-test-file-cpp
description: 'Add or modify a C++ unit or integration test file with GTest/GMock fixtures, test cases, and CMakeLists.txt build integration. Use when adding tests for existing C++ source files (.cpp/.hpp), following TDD workflow for C++ code, or need GTest scaffolding wired into ament_cmake. Keywords: gtest, gmock, TEST_F, ament_add_gmock, CMakeLists.txt tests, C++ test.'
---

# Add C++ Test File

Generate a GTest/GMock test file with appropriate fixtures, test cases, and CMake build integration for an existing C++ source file in a ROS 2 package.

For Python sources, use [add-test-file-python](../add-test-file-python/) instead.

## When to Use This Skill

- Adding tests for an existing C++ source file (`.cpp`/`.hpp`)
- Creating unit tests for C++ classes or free functions
- Setting up integration tests for rclcpp nodes
- Need GTest/GMock fixtures and scaffolding that match project conventions
- Following TDD workflow (write tests first)

## Prerequisites

- Source file exists and location is known
- Package `CMakeLists.txt` accessible
- Understand unit vs integration test needs

## Inputs

- **Source File**: Path to source (or current file in editor)
- **Test Type**: `unit` or `integration`
- **Package Name**: Derive from path or ask
- **Test Name**: From source file name or custom

## Workflow

### Step 1: Analyze Source File

1. Read source file path
2. Extract package name from path
3. Identify: classes, public methods, free functions, ROS 2 nodes, dependencies

### Step 2: Determine Test Location

`<package_root>/test/test_<source_base>.cpp`

### Step 3: Check Existing Tests

If exists: offer to append or create `test_<name>_integration.cpp` etc.

### Step 4: Generate Test File

**Unit:**

```cpp
#include "<package_name>/<header>.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class <ClassName>Test : public ::testing::Test { ... };
TEST_F(<ClassName>Test, ConstructorInitializesCorrectly) { ... }
TEST_F(<ClassName>Test, MethodNameHandlesValidInput) { ... }
TEST_F(<ClassName>Test, MethodNameHandlesInvalidInput) { ... }
```

**Integration:** Include `rclcpp/rclcpp.hpp`, init/shutdown in SetUp/TearDown.

### Step 5: Update Build Configuration

Add `ament_add_gmock(test_<name> test/test_<name>.cpp)` and dependencies; call `drqp_test_enable_coverage(test_<name>)`.

### Step 6: Verify package.xml Dependencies

Test deps: `ament_lint_auto`, `drqp_lint_common`, `ament_cmake_gmock`, `ament_cmake_pytest`.

## Edge Cases

- **Existing test**: Append or create separate file
- **No public interface**: Warn
- **Missing deps**: Add to package.xml
- **Header-only**: Test the header

## Related Resources

- [add-test-file-python](../add-test-file-python/) — Python test scaffolding
- [find-test-files](../find-test-files/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
