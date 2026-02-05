---
name: ros2-environment-setup
description: Set up ROS 2 development environment for local and remote development. Use when asked to configure workspace environment, initialize ROS setup, configure Python virtual environment, or use devcontainer. Supports local development, remote development via devcontainer, environment variable configuration, and venv management.

---

# ROS 2 Environment Setup

Initialize and configure the ROS 2 development environment for both local and remote development workflows.

## When to Use This Skill

- Initialize workspace after cloning
- Set up local development environment
- Configure remote development with devcontainer
- Initialize Python virtual environment
- Update environment variables for build system
- Configure IDE for ROS 2 development
- Reset or troubleshoot environment issues
- Switch between development and production Python environments

## Prerequisites

- ROS 2 Jazzy installed at `/opt/ros/jazzy/`
 - Workspace cloned to your workspace root
- Bash shell available (sourcing scripts)
- For local development: system dependencies installed (CMake, compilers, etc.)
- For remote development: Docker and VS Code Remote Extensions
- Python 3.8+ available

## Environment Architecture

```
ROS 2 Environment Hierarchy:
├── Base ROS 2 (/opt/ros/jazzy/setup.bash)
│   ├── ROS_DISTRO=jazzy
│   ├── ROS_PYTHON_VERSION=3
│   └── Core ROS 2 packages
├── Workspace Overlay (install/local_setup.bash)
│   ├── Workspace packages
│   └── Built packages
└── Python venv (.venv/)
    └── Development dependencies
```

## Step-by-Step Workflows

### Workflow 1: Initialize Local Development Environment

Set up a fresh workspace for local development.

1. Clone or navigate to workspace:
   ```bash
   cd <workspace_root>
   ```

2. Source the ROS environment setup script:
   ```bash
   source scripts/setup.bash
   ```

   This script:
   - Sources `/opt/ros/jazzy/setup.bash` (base ROS)
   - Sources `install/local_setup.bash` if available (workspace overlay)
   - Activates production Python environment
   - Sets ROS_DISTRO, ROS_PACKAGE_PATH, and other ROS variables

3. Verify ROS environment is configured:
   ```bash
   echo $ROS_DISTRO          # Should print: jazzy
   echo $ROS_PACKAGE_PATH    # Should show workspace packages
   which ros2                # Should find ROS 2 executable
   ```

4. Environment is ready for development

### Workflow 2: Set Up Python Virtual Environment for Development

Create isolated Python environment for development tools.

1. Create virtual environment (one-time setup):
   ```bash
   python3 -m venv .venv
   ```

2. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

   Your shell prompt changes to show `(.venv)` prefix

3. Install development dependencies:
   ```bash
   python3 -m pip install --upgrade pip
   python3 -m pip install -r requirements.txt --use-pep517
   ```

   Installs:
   - Testing frameworks (pytest, coverage)
   - Code quality tools (ruff, mypy, type checkers)
   - Documentation tools (sphinx, autodoc)
   - Jupyter notebooks for prototyping

4. Verify installation:
   ```bash
   python3 -m pytest --version
   python3 -c "import ruff; print('ruff ready')"
   ```

5. Exit venv when done:
   ```bash
   deactivate
   ```

**Note**: Virtual environment is for development only; ROS 2 runtime uses system Python

### Workflow 3: Update Python Virtual Environment

Refresh venv dependencies after requirements.txt changes.

1. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

2. Use convenience flag with setup script:
   ```bash
   deactivate  # Exit any active venv
   source scripts/setup.bash --update-venv
   ```

   This automatically:
   - Re-creates/updates the venv
   - Installs latest dependencies from requirements.txt
   - Leaves venv activated for immediate use

3. Verify updates completed:
   ```bash
   pip list | grep pytest
   ```

4. Deactivate when done:
   ```bash
   deactivate
   ```

### Workflow 4: Configure Remote Development with Devcontainer

Set up VS Code devcontainer for remote/GitHub Codespaces development.

1. Open workspace in VS Code:
   ```bash
   code <workspace_root>
   ```

2. Install "Dev Containers" extension (if not already installed)

3. Open Command Palette: `Cmd+Shift+P` (Mac) or `Ctrl+Shift+P` (Linux/Windows)

4. Search for: "Dev Containers: Reopen in Container"

5. VS Code rebuilds the devcontainer:
   - **Base image**: `ghcr.io/dr-qp/jazzy-ros-desktop:edge`
   - **Build time**: 5-15 minutes (first time only)
   - **Subsequent opens**: < 10 seconds

6. Devcontainer automatically:
   - Sets up ROS 2 Jazzy environment
   - Creates Python venv
   - Installs all dependencies via rosdep
   - Configures development tools (clangd, debuggers, test viewers)

7. Verify devcontainer environment:
   ```bash
   echo $ROS_DISTRO
   python3 -m pytest --version
   ```

**Devcontainer Features**:
- Persistent volumes: Build artifacts, install, logs, caches persist across sessions
- Network: Proper ROS communication (DDS, IPC)
- Development tools: Clangd, debuggers, test viewers pre-configured
- Consistent environment: Identical across all developer machines and CI

### Workflow 5: Configure Build System Environment Variables

Set compiler and build system options for optimal performance.

1. Source the base ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Configure for development builds with clang:
   ```bash
   export CMAKE_EXPORT_COMPILE_COMMANDS=1
   export CC=clang
   export CXX=clang++
   ```

   **What these do:**
   - `CMAKE_EXPORT_COMPILE_COMMANDS=1`: Generates compile_commands.json for IDE
   - `CC=clang`: Use Clang C compiler (faster, better error messages)
   - `CXX=clang++`: Use Clang C++ compiler

3. Verify environment:
   ```bash
   echo $CC          # Should print: clang
   echo $CXX         # Should print: clang++
   ls -la build/compile_commands.json  # IDE integration
   ```

4. Build with optimized environment:
   ```bash
   python3 -m colcon build --packages-up-to <package_name>
   ```

### Workflow 6: Verify Complete Environment Setup

Check that all components are properly configured.

1. Verify ROS 2 core:
   ```bash
   source scripts/setup.bash
   ros2 topic list        # Should work (may be empty if no nodes running)
   ros2 service list      # Should work
   ros2 node list         # Should work
   ```

2. Verify workspace packages:
   ```bash
   ros2 pkg list | grep drqp     # Should show workspace packages
   ```

3. Verify Python setup:
   ```bash
   python3 -c "import rclpy; print('ROS 2 Python client ready')"
   ```

4. Verify development tools:
   ```bash
   source .venv/bin/activate
   python3 -m pytest --version
   python3 -m ruff --version
   deactivate
   ```

5. If all commands succeed, environment is fully configured

### Workflow 7: Reset Environment (Troubleshooting)

Clean up environment and start fresh.

1. Deactivate any active Python venv:
   ```bash
   deactivate
   ```

2. Clear environment variables:
   ```bash
   unset ROS_DISTRO ROS_PACKAGE_PATH CC CXX CMAKE_EXPORT_COMPILE_COMMANDS
   ```

3. Open a new terminal or shell

4. Re-run setup:
   ```bash
   source scripts/setup.bash
   ```

5. For clean venv, remove and recreate:
   ```bash
   rm -rf .venv
   python3 -m venv .venv
   source .venv/bin/activate
   python3 -m pip install -r requirements.txt --use-pep517
   ```

## Environment Variables Reference

| Variable | Purpose | Set By | Value |
|----------|---------|--------|-------|
| `ROS_DISTRO` | ROS 2 distribution | setup.bash | `jazzy` |
| `ROS_PACKAGE_PATH` | Package search paths | setup.bash | `$(pwd)/install/` |
| `AMENT_PREFIX_PATH` | Installed package prefix | setup.bash | `$(pwd)/install/` |
| `CMAKE_EXPORT_COMPILE_COMMANDS` | IDE integration | Manual | `1` |
| `CC` | C compiler | Manual | `clang` |
| `CXX` | C++ compiler | Manual | `clang++` |
| `PYTHONPATH` | Python module search path | setup.bash | Includes workspace packages |
| `VIRTUAL_ENV` | Active venv path | venv activation | `$(pwd)/.venv` |

## Directory Structure

```
<workspace_root>/
├── scripts/
│   └── setup.bash           # Main environment setup script
├── install/                 # Built/installed packages
│   ├── local_setup.bash     # Workspace overlay setup
│   └── <packages>/
├── build/                   # Build artifacts
│   └── compile_commands.json # For IDE integration
├── .venv/                   # Python virtual environment
│   ├── bin/activate         # Activation script
│   └── lib/python3.x/
└── .devcontainer/
    └── devcontainer.json    # Remote development config
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Command 'ros2' not found" | ROS 2 not sourced | Run `source scripts/setup.bash` |
| "Package not found" error | Workspace overlay not loaded | Ensure `install/local_setup.bash` exists, rebuild if needed |
| Python import "rclpy" fails | ROS 2 Python client not available | Install: `rosdep install --from-paths packages --ignore-src -y` |
| Venv not activating | Venv not created or corrupted | Delete `.venv` and recreate: `python3 -m venv .venv` |
| Devcontainer won't start | Docker not running or insufficient space | Start Docker daemon and check disk space |
| Conflicting Python versions | Multiple Python environments active | Deactivate venv: `deactivate`, then verify `python3 --version` |
| Build uses wrong compiler | Environment variables not set | Export: `export CC=clang && export CXX=clang++` |
| IDE can't find includes | Compile commands missing | Build with `CMAKE_EXPORT_COMPILE_COMMANDS=1` enabled |

## References

- Devcontainer configuration: `.devcontainer/devcontainer.json`

