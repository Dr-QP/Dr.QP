---
name: ros2-dependency-management
description: Manage ROS 2 workspace dependencies using rosdep, pip, and package.xml. Use when asked to install dependencies, setup Python virtual environment, configure package dependencies, or resolve missing dependencies. Supports C++ dependencies, Python packages, tool dependencies, and dependency installation workflows.

---

# ROS 2 Dependency Management

Manage workspace dependencies across C++, Python, and system tools using rosdep and pip.

## When to Use This Skill

- Install all workspace dependencies
- Add Python development dependencies
- Configure package.xml dependencies
- Set up Python virtual environment
- Install tool dependencies via rosdep
- Resolve "package not found" errors
- Update dependency cache
- Manage dependency versions

## Prerequisites

- ROS 2 Jazzy installation
- Workspace root directory available
- Python 3.8+ for virtual environment
- `scripts/ros-dep.sh` available for automated installation
- `rosdep` command available (installed with ROS 2)
- Internet access for downloading packages

## Dependency Types

| Type | Managed By | Configuration | Purpose |
|------|-----------|---------------|---------|
| **Tool Dependencies** | rosdep | `package.xml` build tools | Build system, linters, CI tools |
| **C++ Runtime/Build Deps** | rosdep | `package.xml` `<depend>` | C++ libraries and headers |
| **Python Runtime Deps** | rosdep or setup.py | `package.xml` or `setup.cfg` | Production Python packages |
| **Python Dev Deps** | pip + requirements.txt | `requirements.txt` | Development tools (pytest, ruff, docs) |

## Step-by-Step Workflows

### Workflow 1: Install All Workspace Dependencies (Recommended Initial Setup)

Complete dependency installation for the workspace.

1. Navigate to workspace root:
   ```bash
   cd <workspace_root>
   ```

2. Run automated dependency installation:
   ```bash
   ./scripts/ros-dep.sh
   ```

   This script automatically:
   - Runs `rosdep update` (updates package registry)
   - Installs all tool dependencies from `package.xml` files
   - Installs all C++ runtime/build dependencies
   - Ensures system dependencies are satisfied

3. Verify installation completed without errors (check final output)

4. Installation is complete; proceed to Python venv setup if needed

### Workflow 2: Set Up Python Virtual Environment

Isolate Python development dependencies from system Python.

1. Create virtual environment:
   ```bash
   python3 -m venv .venv
   ```

2. Activate virtual environment:
   ```bash
   source .venv/bin/activate
   ```

3. Install development dependencies:
   ```bash
   .venv/bin/python3 -m pip install -r requirements.txt --use-pep517
   ```

   This installs:
   - Test frameworks (pytest, coverage)
   - Code quality tools (ruff, type checkers)
   - Documentation tools (sphinx, autodoc)
   - Development utilities

4. Verify installation:
   ```bash
   python3 -m pytest --version
   ```

5. Deactivate when done:
   ```bash
   deactivate
   ```

### Workflow 3: Update ROS Dependency Cache

Refresh rosdep package registry (required after ROS updates).

1. Update rosdep cache:
   ```bash
   rosdep update
   ```

2. Verify update completed:
   ```bash
   rosdep list | head -20
   ```

### Workflow 4: Add C++ Dependency to Package

Add a new C++ library to a package's build/runtime dependencies.

1. Edit package's `package.xml`:
   ```bash
   nano packages/runtime/<package_name>/package.xml
   ```

2. Add dependency in appropriate section:
   ```xml
   <!-- For build-time dependencies -->
   <build_depend>new_library</build_depend>

   <!-- For runtime dependencies -->
   <run_depend>new_library</run_depend>

   <!-- For both build and runtime -->
   <depend>new_library</depend>
   ```

3. Install the new dependency:
   ```bash
   rosdep install --from-paths packages/runtime/<package_name> --ignore-src -y
   ```

4. Rebuild the package:
   ```bash
   source scripts/setup.bash
   python3 -m colcon build --packages-select <package_name>
   ```

### Workflow 5: Add Python Package Dependency

Add Python package to development or production use.

**For Development Dependencies** (pytest, docs, linting):

1. Add package name to `requirements.txt`:
   ```bash
   echo "new-package>=1.0.0" >> requirements.txt
   ```

2. Install updated requirements:
   ```bash
   source .venv/bin/activate
   python3 -m pip install -r requirements.txt --use-pep517
   deactivate
   ```

**For Production Dependencies** (used by package code):

1. Edit package's `setup.py` or `package.xml`:
   ```bash
   # For setup.py-based packages:
   nano packages/runtime/<package_name>/setup.py
   ```

2. Add to `install_requires` list in `setup.py`:
   ```python
   install_requires=[
       'existing-package>=1.0',
       'new-package>=2.0',
   ]
   ```

   OR edit `package.xml` for rosdep-managed dependencies:
   ```xml
   <build_depend>python3-new-package</build_depend>
   <run_depend>python3-new-package</run_depend>
   ```

3. Rebuild the package:
   ```bash
   source scripts/setup.bash
   python3 -m colcon build --packages-select <package_name>
   ```

### Workflow 6: Troubleshoot Missing Dependency

Resolve "package not found" or "library not found" errors.

1. Identify the missing package from error message

2. Search rosdep registry:
   ```bash
   rosdep search <package_name>
   ```

3. If found in rosdep, add to appropriate `package.xml` or `setup.py`

4. If not in rosdep:
   - Check if available via pip: `pip search <package_name>`
   - Or check GitHub/documentation for installation instructions
   - Add as custom dependency with full installation steps

5. Re-install dependencies and rebuild:
   ```bash
   ./scripts/ros-dep.sh
   source scripts/setup.bash
   python3 -m colcon build --packages-up-to <package_name>
   ```

## Dependency File Locations

| File | Purpose | Scope |
|------|---------|-------|
| `packages/runtime/*/package.xml` | C++ and ROS dependencies per package | Package-level |
| `packages/runtime/*/setup.py` | Python package dependencies | Package-level (Python packages only) |
| `requirements.txt` | Development tools and docs | Workspace-level |
| `.venv/` | Virtual environment | Local development only |
| `scripts/ros-dep.sh` | Automated dependency installation | Workspace setup |

## Common Dependency Management Commands

```bash
# Update rosdep registry
rosdep update

# Search for available package
rosdep search <package_name>

# Check if dependency is satisfied
rosdep check --from-paths packages/runtime/<package_name>

# Install dependencies for specific path
rosdep install --from-paths packages/runtime/<package_name> --ignore-src -y

# List all installed system dependencies
rosdep list | grep "^python3"

# Activate Python venv
source .venv/bin/activate

# Install/upgrade pip
python3 -m pip install --upgrade pip

# Show installed pip packages
pip list

# Install with specific version
pip install "package_name==1.2.3"
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "rosdep: command not found" | ROS 2 not installed | Install ROS 2 Jazzy first |
| "package not found in registry" | Package not in rosdep registry | Use pip or install from source |
| Python package import fails | venv not activated or package not installed | Run `source .venv/bin/activate` and `pip install -r requirements.txt` |
| "Permission denied" on install | User lacks write permission | Use `sudo` or fix directory permissions |
| Circular dependency errors | Package depends on itself transitively | Review `package.xml` dependencies |
| Stale dependency cache | Old dependency versions cached | Run `rosdep update` and `pip cache purge` |
| Version conflicts | Different packages require incompatible versions | Review version constraints and update `setup.py` |

## References

- See the dependency management guide in your project documentation or official ROS 2 docs for detailed workflows
- See the official rosdep reference documentation for rosdep commands and options
- Optionally use an `./scripts/ros-dep.sh` helper script in your repository for automated installation
- Refer to `package.xml` templates in ROS 2 documentation or your project for dependency configuration examples

