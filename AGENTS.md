# Agents Guidelines

## General Engineering Principles

For comprehensive engineering standards and best practices, refer to [shared engineering guidelines](/.github/instructions/engineering.instructions.md).

**Quick reference**:

- Prioritize readability and clarity
- Apply Clean Code and SOLID principles
- Handle edge cases and exceptions explicitly
- Use consistent, idiomatic patterns
- Follow language-specific conventions

### When in Doubt

Consult with #subAgent principal-software-engineer

## Available Skills

This workspace provides specialized Agent Skills that enhance your capabilities:

- **[ros2-workspace-build](/.github/skills/ros2-workspace-build/SKILL.md)**: Build ROS 2 packages with colcon (incremental builds, debug symbols, coverage)
- **[ros2-workspace-testing](/.github/skills/ros2-workspace-testing/SKILL.md)**: Test packages and generate coverage reports
- **[ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md)**: Manage workspace dependencies via rosdep and pip
- **[ros2-environment-setup](/.github/skills/ros2-environment-setup/SKILL.md)**: Initialize and configure the development environment
- **[code-review-standards](/.github/skills/code-review-standards/SKILL.md)**: Write PR descriptions and conduct code reviews

## ROS 2 Development Environment

This is a ROS 2 Jazzy workspace using colcon build system for C++ and Python packages.

For detailed workflow guidance, use the relevant **Agent Skill** for your task:

- **Environment Setup**: Use [ros2-environment-setup](/.github/skills/ros2-environment-setup/SKILL.md)
- **Building**: Use [ros2-workspace-build](/.github/skills/ros2-workspace-build/SKILL.md)
- **Testing**: Use [ros2-workspace-testing](/.github/skills/ros2-workspace-testing/SKILL.md)
- **Dependencies**: Use [ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md)

## Package Structure

The workspace contains the following package categories:

- **runtime/**: Core runtime packages
  - `drqp_interfaces`: ROS message/service definitions
  - `drqp_serial`: Serial communication driver
  - `drqp_a1_16_driver`: A1-16 servo controller driver
  - `drqp_control`: Robot control nodes
  - `drqp_brain`: High-level robot behavior and state machine
  - `drqp_lint_common`: Common linting configurations
  - `drqp_rapidjson`: RapidJSON vendor package

- **simulation/**: Simulation packages
  - `drqp_gazebo`: Gazebo simulation integration

- **cmake/**: CMake utilities and macros

### CI/CD Pipeline

The GitHub Actions workflow (`.github/workflows/ci.yml`) runs:

1. **Build development Docker image** (`jazzy-ros-desktop`)
2. **Build ROS workspace** with coverage enabled
3. **Run all tests** with coverage collection
4. **Process coverage reports** and upload to Codecov
5. **Test devcontainer** to ensure it works
6. **Build deployment image** for production use

**Key CI commands:**

```bash
# Install dependencies
./scripts/ros-dep.sh

# Build with coverage
python3 -m colcon build --mixin coverage-pytest ninja rel-with-deb-info \
  --event-handlers=console_cohesion+ --symlink-install \
  --cmake-args -D DRQP_ENABLE_COVERAGE=ON

# Run tests
python3 -m colcon test --mixin coverage-pytest \
  --event-handlers=console_cohesion+ --return-code-on-test-failure

# Process coverage
./packages/cmake/llvm-cov-export-all.py ./build
```

### Dependency Management

For comprehensive dependency management guidance (C++, Python, rosdep, pip), see the [ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md) skill.

### Cleaning Build Artifacts

```bash
# Clean build, install, and log directories
./scripts/clean.fish
```

## Best Practices for Agents

1. **Always source setup.bash** before any build or test operation
2. **Use incremental builds** (`--packages-up-to <pkg>`) during development
3. **Test specific packages** (`--packages-select <pkg>`) for rapid iteration
4. **Use devcontainer** when running as a remote agent
5. **Only run full builds/tests** when explicitly requested or necessary
6. **Collect test output** from `build/<package_name>/test_results/` for analysis
7. **Check build logs** in `log/latest_build/` if builds fail
8. **Use `--symlink-install`** for faster Python development iteration
9. **Enable coverage** with `--mixin coverage-pytest` when testing
10. **Re-run failed tests** with `--packages-select-test-failures` to save time

## Code Review Standards

For comprehensive code review guidance and PR description standards, see the [code-review-standards](/.github/skills/code-review-standards/SKILL.md) skill.
