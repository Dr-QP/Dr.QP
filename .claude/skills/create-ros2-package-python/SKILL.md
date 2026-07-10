---
name: create-ros2-package-python
description: 'Create new ROS 2 Python (ament_python) package with proper structure, setup.py, package.xml, and pytest scaffolding. Use when creating a new Python ROS 2 package from scratch, setting up setup.py, setup.cfg, or scaffolding an rclpy node following colcon conventions. Keywords: Python package, ament_python, setup.py, rclpy, new package.'
---

# Create ROS 2 Python Package

Generate a complete ROS 2 Python (`ament_python`) package with proper structure, build configuration, and test scaffolding following project conventions.

For C++ packages, use [create-ros2-package-cpp](../create-ros2-package-cpp/) instead.
For mixed C++/Python packages, apply this skill for the Python side (`setup.py`, module structure, `.coveragerc`) and combine with the C++ skill for CMakeLists.txt and C++ structure.

## When to Use This Skill

- Creating a new ROS 2 Python package from scratch
- Need proper package structure with package.xml, setup.py, and test setup
- Setting up a new rclpy node or Python library in the workspace

## Prerequisites

- Working in a ROS 2 workspace with colcon build system
- Have identified package name
- Know required dependencies (optional)

## Inputs

### Required

- **Package Name**: Lowercase with underscores (e.g., `drqp_my_package`)

### Optional

- **Package Path**: Location for package (default: `packages/runtime`)
- **Dependencies**: Comma-separated ROS 2 dependencies (e.g., `rclpy,std_msgs,sensor_msgs`)
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

### Step 4: Generate package.xml

Package format 3, name, version 0.0.0, description, maintainer, license. Build tool: `ament_python`. Runtime and test dependencies. Test deps: `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

### Step 5: Generate setup.py

`find_packages(exclude=['test'])`, data files, entry points, and declare pytest via `extras_require={'test': ['pytest']}` (the removed `tests_require` option is ignored by modern setuptools and emits a warning) so `colcon test` runs Python tests. Reference: `packages/runtime/drqp_brain/setup.py`.

### Step 6: setup.cfg and .coveragerc

Create with install options and coverage configuration.

### Step 7: README.md

Basic sections: Overview, Dependencies, Building (`colcon build --packages-up-to <pkg>`), Testing (`colcon test --packages-select <pkg>`, `colcon test-result --verbose`), Usage.

### Step 8: Test Scaffolding

`test/__init__.py`, `test/test_<package_name>.py` with pytest (never unittest). For ROS 2 node integration or launch tests use `launch_pytest`: add `@launch_pytest.fixture` on `generate_test_description` and `@pytest.mark.launch(fixture=generate_test_description)` on test functions. See [add-test-file-python](../add-test-file-python/) for templates.

### Step 9: Directory Placeholders

`launch/.gitkeep`, `resource/` marker.

## Validation

1. Verify all required files exist
2. Check package.xml and setup.py syntax
3. Optional: `colcon build --packages-select <package_name>` and `colcon test`

## Edge Cases

- **Existing package**: Stop and report error
- **Invalid name**: Reject hyphens, uppercase, special characters
- **Missing deps**: Warn if rclpy not specified

## Related Resources

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [create-ros2-package-cpp](../create-ros2-package-cpp/)
- [ros2-workspace-build](../ros2-workspace-build/)
- [Coding Conventions in AGENTS.md](../../../AGENTS.md)
- Example: `packages/runtime/drqp_brain/`
