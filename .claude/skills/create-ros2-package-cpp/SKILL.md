---
name: create-ros2-package-cpp
description: 'Create new ROS 2 C++ (ament_cmake) package with proper structure, CMakeLists.txt, package.xml, and GTest scaffolding. Use when creating a new C++ ROS 2 package from scratch, setting up CMakeLists.txt, or scaffolding an rclcpp node or library following colcon conventions. Keywords: C++ package, ament_cmake, CMakeLists.txt, rclcpp, new package.'
---

# Create ROS 2 C++ Package

Generate a complete ROS 2 C++ (`ament_cmake`) package with proper structure, build configuration, and test scaffolding following project conventions.

For Python packages, use [create-ros2-package-python](../create-ros2-package-python/) instead.
For mixed C++/Python packages, apply this skill for the C++ side and combine with the Python skill for `setup.py` and Python module structure.

## When to Use This Skill

- Creating a new ROS 2 C++ package from scratch
- Need proper package structure with package.xml, CMakeLists.txt, and test setup
- Setting up a new rclcpp node or C++ library in the workspace

## Prerequisites

- Working in a ROS 2 workspace with colcon build system
- Have identified package name
- Know required dependencies (optional)

## Inputs

### Required

- **Package Name**: Lowercase with underscores (e.g., `drqp_my_package`)

### Optional

- **Package Path**: Location for package (default: `packages/runtime`)
- **Dependencies**: Comma-separated ROS 2 dependencies (e.g., `rclcpp,std_msgs,sensor_msgs`)
- **Description**, **Maintainer**, **License**: Defaults from git config or sensible values

## Workflow

### Step 1: Gather Requirements

1. Request package name if not provided
2. Validate package name: lowercase, numbers, underscores only; `drqp_` prefix for this project; no hyphens or special characters
3. Ask for optional dependencies and description

### Step 2: Determine Package Location

1. Use `packages/runtime/` or provided path
2. Full path: `{workspaceFolder}/{packagePath}/{packageName}`
3. Verify directory does not already exist; if exists, stop and report error

### Step 3: Create Package Structure

```
<package_name>/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/<package_name>/
├── src/
├── test/
├── launch/
└── config/
```

### Step 4: Generate CMakeLists.txt

Include: project metadata, C++20, compiler warnings, `include(../../cmake/ClangCoverage.cmake)`, dependency finding via `ament_package_xml()`, library/executable targets, installation, `ament_lint_auto`, `drqp_lint_common`. Reference: `packages/runtime/drqp_control/CMakeLists.txt`.

### Step 5: Generate package.xml

Package format 3, name, version 0.0.0, description, maintainer, license. Build tool: `ament_cmake`. Runtime and test dependencies. Test deps: `ament_lint_auto`, `drqp_lint_common`, `ament_cmake_gmock`, `ament_cmake_pytest`.

### Step 6: README.md

Basic sections: Overview, Dependencies, Building (`colcon build --packages-up-to <pkg>`), Testing (`colcon test --packages-select <pkg>`, `colcon test-result --verbose`), Usage.

### Step 7: Test Scaffolding

`test/test_<package_name>.cpp` with GTest, `ament_add_gmock`. See [add-test-file-cpp](../add-test-file-cpp/) for templates.

### Step 8: Directory Placeholders

`launch/.gitkeep`, `config/.gitkeep`.

## Validation

1. Verify all required files exist
2. Check package.xml and CMakeLists.txt syntax
3. Optional: `colcon build --packages-select <package_name>` and `colcon test`

## Edge Cases

- **Existing package**: Stop and report error
- **Invalid name**: Reject hyphens, uppercase, special characters
- **Missing deps**: Warn if rclcpp not specified

## Related Resources

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [create-ros2-package-python](../create-ros2-package-python/)
- [ros2-workspace-build](../ros2-workspace-build/)
- [Coding Conventions in AGENTS.md](../../../AGENTS.md)
- Example: `packages/runtime/drqp_control/`
