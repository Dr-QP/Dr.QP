---
name: create-ros2-package
description: Create new ROS 2 package with proper structure, build files, and test scaffolding. Use when creating a new ROS 2 package from scratch, setting up package.xml, CMakeLists.txt, setup.py, or need C++, Python, or mixed package structure following colcon conventions.
---

# Create ROS 2 Package

Generate a complete ROS 2 package with proper structure, build configuration, and test scaffolding following project conventions.

## When to Use This Skill

- Creating a new ROS 2 package from scratch
- Need proper package structure with package.xml, build files, and test setup
- Want to follow project conventions for C++, Python, or mixed packages
- Setting up a new node or library in the workspace

## Prerequisites

- Working in a ROS 2 workspace with colcon build system
- Have identified package name and type (C++, Python, or mixed)
- Know required dependencies (optional)

## Inputs

### Required

- **Package Name**: Lowercase with underscores (e.g., `drqp_my_package`)
- **Package Type**: `cpp`, `python`, or `mixed`

### Optional

- **Package Path**: Location for package (default: `packages/runtime`)
- **Dependencies**: Comma-separated ROS 2 dependencies (e.g., `rclcpp,std_msgs,sensor_msgs`)
- **Description**, **Maintainer**, **License**: Defaults from git config or sensible values

## Workflow

### Step 1: Gather Requirements

1. Request package name if not provided
2. Validate package name: lowercase, numbers, underscores only; `drqp_` prefix for this project; no hyphens or special characters
3. Request package type (cpp/python/mixed) if not provided
4. Ask for optional dependencies and description

### Step 2: Determine Package Location

1. Use `packages/runtime/` or provided path
2. Full path: `{workspaceFolder}/{packagePath}/{packageName}`
3. Verify directory does not already exist; if exists, stop and report error

### Step 3: Create Package Structure

**C++ packages:**

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

**Python packages:**

```
<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/<package_name>
├── <package_name>/__init__.py
├── test/__init__.py
├── launch/
└── .coveragerc
```

**Mixed:** Combine both structures.

### Step 4: Generate CMakeLists.txt (C++ or Mixed)

Include: project metadata, C++20, compiler warnings, `include(../../cmake/ClangCoverage.cmake)`, dependency finding via `ament_package_xml()`, library/executable targets, installation, `ament_lint_auto`, `drqp_lint_common`. Reference: `packages/runtime/drqp_control/CMakeLists.txt`.

### Step 5: Generate package.xml

Package format 3, name, version 0.0.0, description, maintainer, license. Build tool: `ament_cmake` or `ament_python`. Runtime and test dependencies. Test deps: C++ — `ament_lint_auto`, `drqp_lint_common`, `ament_cmake_gmock`, `ament_cmake_pytest`; Python — `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

### Step 6: Generate setup.py (Python or Mixed)

`find_packages(exclude=['test'])`, data files, entry points, and for `ament_python` packages set `tests_require=['pytest']` so `colcon test` runs Python tests. Reference: `packages/runtime/drqp_brain/setup.py`.

### Step 7: setup.cfg and .coveragerc (Python)

Create with install options and coverage configuration.

### Step 8: README.md

Basic sections: Overview, Dependencies, Building (`colcon build --packages-up-to <pkg>`), Testing (`colcon test --packages-select <pkg>`, `colcon test-result --verbose`), Usage.

### Step 9: Test Scaffolding

C++: `test/test_<package_name>.cpp` with GTest, `ament_add_gmock`. Python: `test/__init__.py`, `test/test_<package_name>.py` with pytest.

### Step 10: Directory Placeholders

`launch/.gitkeep`, `config/.gitkeep` (C++), `resource/` marker (Python).

## Validation

1. Verify all required files exist
2. Check package.xml and build file syntax
3. Optional: `colcon build --packages-select <package_name>` and `colcon test`

## Edge Cases

- **Existing package**: Stop and report error
- **Invalid name**: Reject hyphens, uppercase, special characters
- **Missing deps**: Warn if rclcpp/rclpy not specified
- **Invalid type**: Only `cpp`, `python`, `mixed`

## Related Resources

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ros2-workspace-build](../ros2-workspace-build/)
- [Engineering guidelines](../../instructions/engineering.instructions.md)
- Examples: `packages/runtime/drqp_control/`, `packages/runtime/drqp_brain/`
