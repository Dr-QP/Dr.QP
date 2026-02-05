---
description: 'Create a new ROS 2 package with proper structure, build files, and test scaffolding'
name: 'create-ros2-package'
agent: 'agent'
tools: ['read', 'edit', 'search']
argument-hint: '<package-name> [--type cpp|python|mixed] [--deps dep1,dep2]'
---

# Create ROS 2 Package

Generate a complete ROS 2 package with proper structure, build configuration, and test scaffolding following project conventions.

## When to Use This Prompt

- Creating a new ROS 2 package from scratch
- Need proper package structure with package.xml, build files, and test setup
- Want to follow project conventions for C++, Python, or mixed packages
- Setting up a new node or library in the workspace

## Prerequisites

- Working in a ROS 2 workspace with colcon build system
- Have identified package name and type (C++, Python, or mixed)
- Know required dependencies (optional)

## Inputs

### Required Inputs

- **Package Name** `${input:packageName:my_package}`: Name of the package (lowercase with underscores, e.g., `drqp_my_package`)
- **Package Type** `${input:packageType:cpp}`: Type of package (`cpp`, `python`, or `mixed`)

### Optional Inputs

- **Package Path** `${input:packagePath:packages/runtime}`: Location for the package (default: `packages/runtime`)
- **Dependencies**: Comma-separated list of ROS 2 dependencies (e.g., `rclcpp,std_msgs,sensor_msgs`)
- **Description**: Brief package description (default: "TODO: Package description")
- **Maintainer Name**: Package maintainer (default: from git config)
- **Maintainer Email**: Maintainer email (default: from git config)
- **License**: Package license (default: `MIT`)

## Workflow

### Step 1: Gather Requirements

1. Request package name if not provided
2. Validate package name follows ROS 2 naming conventions:
   - Lowercase letters, numbers, underscores only
   - Should start with `drqp_` prefix for this project
   - Must not contain hyphens or special characters
3. Request package type (cpp/python/mixed) if not provided
4. Ask for optional dependencies and description

### Step 2: Determine Package Location

1. Check if package path is provided, otherwise use `packages/runtime/`
2. Create full package path: `${workspaceFolder}/${packagePath}/${packageName}`
3. Verify the directory doesn't already exist
4. If exists, STOP and report error

### Step 3: Create Package Structure

Based on package type, create the following structure:

#### For C++ Packages:
```
<package_name>/
├── CMakeLists.txt       # Build configuration
├── package.xml          # Package metadata
├── README.md           # Package documentation
├── include/<package_name>/  # Public headers
├── src/                # Source files
├── test/               # Test files
├── launch/             # Launch files (optional)
└── config/             # Configuration files (optional)
```

#### For Python Packages:
```
<package_name>/
├── package.xml          # Package metadata
├── setup.py            # Python package setup
├── setup.cfg           # Setup configuration
├── README.md           # Package documentation
├── resource/<package_name>  # Resource marker
├── <package_name>/     # Python module
│   └── __init__.py
├── test/               # Test files
│   └── __init__.py
├── launch/             # Launch files (optional)
└── .coveragerc         # Coverage configuration
```

#### For Mixed Packages:
Combine both C++ and Python structures above.

### Step 4: Generate CMakeLists.txt (C++ or Mixed)

Create `CMakeLists.txt` with:
- Project metadata
- C++ standard (C++20)
- Compiler warnings and flags
- Coverage support inclusion: `include(../../cmake/ClangCoverage.cmake)`
- Dependency finding using `ament_package_xml()` pattern
- Library and executable targets
- Installation rules
- Testing configuration with `ament_lint_auto` and `drqp_lint_common`

**Template Reference**: Use `/home/runner/work/Dr.QP/Dr.QP/packages/runtime/drqp_control/CMakeLists.txt` as reference.

### Step 5: Generate package.xml

Create `package.xml` with:
- Package format 3 (ROS 2)
- Name, version (0.0.0), description
- Maintainer and license
- Build tool dependency (`ament_cmake` or `ament_python`)
- Runtime dependencies provided by user
- Test dependencies:
  - For C++: `ament_lint_auto`, `drqp_lint_common`, `ament_cmake_gmock`, `ament_cmake_pytest`
  - For Python: `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`
- Export section with build type

**Template Reference**: Use existing packages as reference.

### Step 6: Generate setup.py (Python or Mixed)

For Python packages, create `setup.py` with:
- Package name and version
- `find_packages(exclude=['test'])`
- Data files (resource index, package.xml, launch files)
- Install requirements (`setuptools`)
- Tests require (`pytest`)
- Maintainer info
- Entry points for console scripts (if creating nodes)

**Template Reference**: Use `/home/runner/work/Dr.QP/Dr.QP/packages/runtime/drqp_brain/setup.py` as reference.

### Step 7: Generate setup.cfg and .coveragerc (Python)

For Python packages:
- Create `setup.cfg` with install options
- Create `.coveragerc` with coverage configuration

### Step 8: Create README.md

Generate README.md with:
```markdown
# <Package Name>

<Description>

## Overview

TODO: Add package overview

## Dependencies

- List dependencies here

## Building

```bash
colcon build --packages-up-to <package_name>
```

## Testing

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Usage

TODO: Add usage instructions
```

### Step 9: Create Test Scaffolding

#### For C++ Packages:
- Create `test/test_<package_name>.cpp` with basic GTest structure
- Include in CMakeLists.txt using `ament_add_gmock`

#### For Python Packages:
- Create `test/__init__.py`
- Create `test/test_<package_name>.py` with pytest structure
- Include basic flake8 and copyright tests

### Step 10: Create Directory Placeholders

Create empty directories with `.gitkeep` files:
- `launch/.gitkeep` (if not populated)
- `config/.gitkeep` (if not populated, C++ only)
- `resource/` with marker file (Python only)

## Output Expectations

### Success Criteria

- Package directory created at specified location
- All required files generated with proper content
- Build files (CMakeLists.txt/setup.py) are valid and compilable
- package.xml follows ROS 2 package format 3
- Test scaffolding is in place
- README.md provides basic documentation

### Generated Files Summary

Report the following:
```
✅ Created ROS 2 <type> package: <package_name>
📁 Location: <full_path>
📝 Files created:
   - package.xml
   - CMakeLists.txt (or setup.py)
   - README.md
   - Test scaffolding in test/
   - Directory structure complete

🔨 Next steps:
   1. Update package description in package.xml
   2. Add source files to src/ (or <package_name>/ for Python)
   3. Build: colcon build --packages-up-to <package_name>
   4. Test: colcon test --packages-select <package_name>
```

## Validation Steps

After package creation:

1. **Verify structure**: Ensure all required files and directories exist
2. **Validate package.xml**: Check XML syntax and required fields
3. **Check build file**: Verify CMakeLists.txt or setup.py syntax
4. **Test build** (optional): Run `colcon build --packages-select <package_name>`
5. **Test discovery** (optional): Run `colcon test --packages-select <package_name>`

## Quality Assurance

- [ ] Package name follows naming conventions (lowercase, underscores, drqp_ prefix)
- [ ] All required files are created
- [ ] Build files include coverage support
- [ ] Test dependencies include drqp_lint_common (C++) or standard linters (Python)
- [ ] README.md provides clear usage instructions
- [ ] package.xml includes all specified dependencies
- [ ] Directory structure matches project conventions
- [ ] No syntax errors in generated files

## Edge Cases

- **Existing package**: If package already exists, report error and stop
- **Invalid package name**: Reject names with hyphens, uppercase, or special characters
- **Missing dependencies**: Warn if common dependencies (like rclcpp/rclpy) are not specified
- **Invalid package type**: Only accept 'cpp', 'python', or 'mixed'
- **Git config unavailable**: Use sensible defaults for maintainer info

## Related Resources

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ros2-workspace-build skill](/.github/skills/ros2-workspace-build/SKILL.md)
- [Engineering guidelines](/.github/instructions/engineering.instructions.md)
- Example C++ package: `packages/runtime/drqp_control/`
- Example Python package: `packages/runtime/drqp_brain/`
