---
name: ros2-workspace-build
description: Build ROS 2 Jazzy workspace using colcon. Use when asked to build a package, enable debug symbols, enable coverage instrumentation, compile workspace with specific flags, or perform incremental vs full builds. Supports single-package builds, dependency chains, debug builds, and coverage-enabled compilation.

---

# ROS 2 Workspace Build

Build ROS 2 packages efficiently using colcon with support for incremental development builds, full workspace builds, debug symbols, and coverage instrumentation.

## When to Use This Skill

- Build a specific package and its dependencies
- Compile only a single package without dependencies
- Build with debug symbols for debugging
- Enable coverage instrumentation for test analysis
- Perform full workspace builds
- Troubleshoot compilation errors
- Clean and rebuild the workspace

## Prerequisites

- ROS 2 Jazzy installation
- Workspace initialized with colcon
- `scripts/setup.bash` available in workspace root
- Build tools: CMake, Ninja/Make, compiler (clang or gcc)

## Step-by-Step Workflows

### Workflow 1: Incremental Package Build (Recommended for Development)

Use for rapid iteration when developing a specific package.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Build the package and its dependencies:
   ```bash
   python3 -m colcon build \
     --symlink-install \
     --event-handlers console_cohesion+ \
     --packages-up-to <package_name>
   ```

3. Verify build success by checking `build/<package_name>/` and `install/<package_name>/` directories

**Typical execution time**: 2-5 minutes depending on package size

### Workflow 2: Build Single Package Only (No Dependencies)

Use when you've only modified internal package code.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Build only the specified package:
   ```bash
   python3 -m colcon build \
     --symlink-install \
     --event-handlers console_cohesion+ \
     --packages-select <package_name>
   ```

**When to use**: You know the package's dependencies are already built

### Workflow 3: Build with Debug Symbols

Use when you need to debug code execution or attach a debugger.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Build with debug symbols:
   ```bash
   python3 -m colcon build \
     --symlink-install \
     --event-handlers console_cohesion+ \
     --packages-up-to <package_name> \
     --cmake-args -GNinja -D CMAKE_BUILD_TYPE=Debug
   ```

3. Debug symbols will be available in the build artifacts

### Workflow 4: Build with Coverage Instrumentation

Use when preparing to run tests with coverage analysis.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Build with coverage enabled:
   ```bash
   python3 -m colcon build \
     --symlink-install \
     --event-handlers console_cohesion+ \
     --packages-up-to <package_name> \
     --cmake-args -GNinja -D CMAKE_BUILD_TYPE=Debug -D DRQP_ENABLE_COVERAGE=ON
   ```

3. After running tests, coverage data will be available in `build/<package_name>/coverage.info` (C++) or `.coverage` (Python)

### Workflow 5: Full Workspace Build

**WARNING**: Full builds take 10-20+ minutes. Only use when explicitly requested or making cross-cutting changes.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Build entire workspace with all optimizations:
   ```bash
   export CMAKE_EXPORT_COMPILE_COMMANDS=1
   export CC=clang
   export CXX=clang++

   python3 -m colcon build \
     --symlink-install \
     --event-handlers console_cohesion+ \
     --base-paths /opt/ros/overlay_ws \
     --cmake-args \
       -GNinja \
       -D CMAKE_BUILD_TYPE=Debug \
       -D DRQP_ENABLE_COVERAGE=ON \
       --no-warn-unused-cli
   ```

3. Wait for all packages to complete (monitor progress in console)

## Common Build Options Reference

| Option | Purpose | Example |
|--------|---------|---------|
| `--packages-up-to <pkg>` | Build package and all dependencies | `--packages-up-to drqp_serial` |
| `--packages-select <pkg>` | Build only specified package | `--packages-select drqp_control` |
| `--symlink-install` | Use symlinks for Python (faster) | Always recommended |
| `--event-handlers console_cohesion+` | Cleaner console output | Always recommended |
| `-GNinja` | Use Ninja build system (faster) | `-GNinja` |
| `-D CMAKE_BUILD_TYPE=Debug` | Build with debug symbols | For debugging |
| `-D DRQP_ENABLE_COVERAGE=ON` | Enable coverage instrumentation | For test coverage |

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Command 'colcon' not found" | Environment not sourced | Run `source scripts/setup.bash` |
| Compilation errors in C++ code | Missing dependencies or code errors | Check `log/latest_build/` for details |
| "Package not found" error | Package doesn't exist or wrong name | Verify package name in `packages/runtime/` or `packages/simulation/` |
| Build takes very long | Full workspace build or large package | Use `--packages-select` or `--packages-up-to` for incremental builds |
| "Permission denied" errors | Cannot write to install directory | Check workspace permissions with `ls -la install/` |
| Linker errors after changes | Stale build artifacts | Run cleanup: `./scripts/clean.fish` then rebuild |

## Build Output Locations

- **Build artifacts**: `build/<package_name>/`
- **Installed packages**: `install/<package_name>/`
- **Build logs**: `log/latest_build/`
- **Compile commands (for IDE)**: `build/compile_commands.json`
- **Coverage data (C++)**: `build/<package_name>/coverage.info`
- **Coverage data (Python)**: `build/<package_name>/.coverage`

## References

- See [colcon build reference](./references/colcon-build-reference.md) for complete option documentation
- See [build configuration template](./templates/build-config.template) for custom build profiles
- Use helper script [build-helper.sh](./scripts/build-helper.sh) for automated builds

