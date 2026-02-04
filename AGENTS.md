# Agents guidelines

## General Instructions

- Always prioritize readability and clarity.
- For algorithm-related code, include explanations of the approach used.
- Write code with good maintainability practices, including comments on why certain design decisions were made.
- Handle edge cases and write clear exception handling.
- For libraries or external dependencies, mention their usage and purpose in comments.
- Use consistent naming conventions and follow language-specific best practices.
- Write concise, efficient, and idiomatic code that is also easily understandable.
- Avoid deep nesting; refactor code into smaller functions when necessary.
- Avoid inlining existing code.
- Prefer use of existing libraries and frameworks over custom implementations.
- Follow Clean Code principles where applicable.

### When in Doubt

When in doubt consult with #subAgent principal-software-engineer

### When doing code reviews

- Always use neutral and positive language.
- Always write PR description in imperative mood.
- Write a comprehensive pull request description in the PR body, including what was changed and why.
- Never write per file changes table in PR description.
- Never write line/change counts in PR description.
- Never add copilot advertisement text, requests for copilot improvement or any other AI tool info to the PR description.
- Never use invisible structures or html comments in PR description.
- Treat PR description to the highest quality of git commit message.

## ROS 2 Development Environment

This is a ROS 2 Jazzy workspace using colcon build system for C++ and Python packages.

### Environment Setup

#### ROS Environment Initialization

Always source the setup script before running builds or tests:

```bash
source scripts/setup.bash
```

The setup script does the following:

1. Sources `/opt/ros/jazzy/setup.bash` (base ROS installation)
2. Sources `install/local_setup.bash` (workspace overlay) if it exists
3. Sets up the production Python virtual environment

**Optional flags:**

- `--update-venv`: Update the Python virtual environment with latest dependencies

#### Devcontainer for Remote Development

When running as a remote agent or in GitHub Codespaces, **ALWAYS use the devcontainer** for testing and development.

The devcontainer configuration is in `.devcontainer/devcontainer.json`:

- **Base image**: `ghcr.io/dr-qp/jazzy-ros-desktop:edge`
- **Workspace path**: `/opt/ros/overlay_ws/`
- **Post-attach commands**: Automatically sets up Python venv and runs rosdep
- **Volumes**: Persists build artifacts, install directory, logs, and caches across sessions

The devcontainer provides a consistent environment with:

- ROS 2 Jazzy fully installed
- All system dependencies pre-configured
- Proper network and IPC settings for ROS communication
- Development tools (clangd, debuggers, test viewers)

### Building the Workspace

#### Incremental Builds (Recommended for Development)

For rapid iteration when developing a specific package, build **only that package and its dependencies**:

```bash
source scripts/setup.bash
python3 -m colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --packages-up-to <package_name>
```

**Common options:**

- `--packages-up-to <pkg>`: Build package and all its dependencies
- `--packages-select <pkg>`: Build only the specified package (no dependencies)
- `--symlink-install`: Use symlinks for Python packages (faster iteration)
- `--event-handlers console_cohesion+`: Cleaner console output
- `--cmake-args -GNinja`: Use Ninja build system (faster than Make)
- `--cmake-args -D CMAKE_BUILD_TYPE=Debug`: Build with debug symbols
- `--cmake-args -D DRQP_ENABLE_COVERAGE=ON`: Enable coverage instrumentation

**Examples:**

```bash
# Build only drqp_serial and its dependencies
python3 -m colcon build --symlink-install --packages-up-to drqp_serial

# Build only drqp_control (no dependencies)
python3 -m colcon build --symlink-install --packages-select drqp_control

# Build with debug symbols and coverage
python3 -m colcon build --symlink-install --packages-up-to drqp_brain \
  --cmake-args -GNinja -D CMAKE_BUILD_TYPE=Debug -D DRQP_ENABLE_COVERAGE=ON
```

#### Full Workspace Build (Use Only When Requested)

**WARNING: Full builds take significant time (10-20+ minutes). Only use when explicitly requested or when making cross-cutting changes affecting multiple packages.**

Full build command from `Dr.QP.code-workspace`:

```bash
source scripts/setup.bash
python3 -m colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --base-paths ${workspaceFolder} \
  --cmake-args \
    -GNinja \
    -D CMAKE_BUILD_TYPE=Debug \
    -D DRQP_ENABLE_COVERAGE=ON \
    --no-warn-unused-cli
```

**Environment variables for the build:**

```bash
export CMAKE_EXPORT_COMPILE_COMMANDS=1
export CC=clang
export CXX=clang++
```

#### Build Output Locations

- **Build artifacts**: `build/<package_name>/`
- **Installed packages**: `install/<package_name>/`
- **Build logs**: `log/latest_build/`
- **Compile commands**: `build/compile_commands.json` (for clangd/IDE integration)

### Testing the Workspace

#### Test Specific Package (Recommended for Development)

For rapid iteration, run tests **only for the package you're developing**:

```bash
source scripts/setup.bash --update-venv
python3 -m colcon test \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure \
  --packages-select <package_name>
```

**Common test options:**

- `--packages-select <pkg>`: Test only the specified package
- `--mixin coverage-pytest`: Enable coverage collection for Python/C++ tests
- `--event-handlers console_cohesion+ summary+ status+`: Enhanced test output
- `--return-code-on-test-failure`: Exit with non-zero code on test failure
- `--packages-select-test-failures`: Re-run only previously failed tests

**Examples:**

```bash
# Test only drqp_serial package
python3 -m colcon test --packages-select drqp_serial \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure

# Test with coverage enabled
python3 -m colcon test --packages-select drqp_brain \
  --mixin coverage-pytest \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure

# Re-run only failed tests
python3 -m colcon test --packages-select-test-failures \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure
```

#### Full Workspace Tests (Use Only When Requested)

**WARNING: Full test suite can take 10-20+ minutes. Only use when explicitly requested or before final integration.**

```bash
source scripts/setup.bash --update-venv
python3 -m colcon test \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure \
  --mixin coverage-pytest
```

#### Test Output Locations

- **Test results**: `build/<package_name>/test_results/<package_name>/*.xunit.xml`
- **Test logs**: `log/latest_test/<package_name>/`
- **Coverage data**:
  - C++: `build/<package_name>/coverage.info` (LCOV format)
  - Python: `build/<package_name>/.coverage` (Coverage.py format)

#### Collecting and Viewing Test Results

**View test summary:**

```bash
python3 -m colcon test-result --all
python3 -m colcon test-result --verbose  # Detailed output
```

**Generate coverage reports:**

```bash
# Process LLVM coverage for C++ packages
./packages/cmake/llvm-cov-export-all.py ./build

# View coverage in VS Code using Coverage Gutters extension
# Coverage files are automatically discovered from build/**/*.info
```

**View test results in browser:**

```bash
# Launch xunit-viewer for interactive test result browsing
npx -y xunit-viewer -r build --server -o build/xunit-index.html \
  -i Test.xml -i coverage.xml -i package.xml --watch
# Opens on http://localhost:3000
```

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

#### Tool dependencies

Tool dependencies are managed via rosdep.

#### C++ dependencies

C++ and CMake dependencies are managed via rosdep.

#### Python dependencies

Development and docs dependencies are managed via `requirements.txt` and installed in `.venv/`:

```bash
python3 -m venv .venv
.venv/bin/python3 -m pip install -r requirements.txt --use-pep517
```

Python production dependencies are managed via rosdep and specified in `package.xml` files.
If python dependency is not available via rosdep registry it is managed via the `setup.py`/`setup.cfg` package dependencies.

#### Install ROS dependencies

```bash
./scripts/ros-dep.sh
```

This script:

1. Runs `rosdep update`
2. Installs all package dependencies from `package.xml` files
3. Ensures system dependencies are satisfied

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
