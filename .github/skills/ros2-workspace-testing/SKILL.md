---
name: ros2-workspace-testing
description: Test ROS 2 packages and analyze results using colcon. Use when asked to run tests, check code coverage, view test results, debug test failures, or generate coverage reports. Supports single-package testing, full workspace tests, coverage collection, and result visualization.
---

# ROS 2 Workspace Testing

Run ROS 2 tests efficiently and analyze results with comprehensive coverage reporting and interactive result visualization.

## When to Use This Skill

- Run tests for a specific package
- Execute full workspace test suite
- Collect code coverage data
- Analyze test results and failures
- Generate coverage reports
- View test results in browser
- Re-run only previously failed tests
- Debug test failures

## Prerequisites

- ROS 2 Jazzy installation
- `scripts/with-ros-env.sh` available in workspace root for ROS 2 commands
- Packages already built with `colcon build`
- Python 3.8+ for coverage tools
- Optional: Node.js for interactive result viewer (xunit-viewer)

## Step-by-Step Workflows

### Workflow 1: Test Specific Package (Recommended for Development)

Use for rapid iteration when developing and testing a single package.

1. Install generated workspace Python requirements, then source the ROS environment:

   ```bash
   ./scripts/ros-dep.sh
   source scripts/setup.bash
   ```

2. Run tests for the package only through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --packages-select <package_name>
   ```

   For the full `drqp_gazebo` launch suite, opt out of the default smoke-only mode:

   ```bash
   DRQP_TEST_MODE=slow scripts/with-ros-env.sh python3 -m colcon test \
      --event-handlers console_cohesion+ summary+ status+ \
      --return-code-on-test-failure \
      --packages-select drqp_gazebo \
      --mixin coverage-pytest
   ```

   Without `DRQP_TEST_MODE=slow`, `drqp_gazebo` runs only the smoke subset and skips non-smoke slow launch tests.

3. Check test results in console output (summary appears at end)

4. View detailed test logs in `log/latest_test/<package_name>/`

**Typical execution time**: 1-5 minutes depending on test count

### Workflow 2: Test with Coverage Collection

Use when you need code coverage metrics alongside test execution.

1. Install generated workspace Python requirements, then source the ROS environment:

   ```bash
   ./scripts/ros-dep.sh
   source scripts/setup.bash
   ```

2. Run tests with coverage enabled through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --packages-select <package_name> \
     --mixin coverage-pytest
   ```

3. Coverage data is collected automatically:
   - **C++ tests**: `build/<package_name>/coverage.info` (LCOV format)
   - **Python tests**: `build/<package_name>/.coverage` (Coverage.py format)

### Workflow 3: Full Workspace Test Suite

**WARNING**: Full test suite takes 10-20+ minutes. Only run when explicitly requested or before final integration.

1. Install generated workspace Python requirements, then source the ROS environment:

   ```bash
   ./scripts/ros-dep.sh
   source scripts/setup.bash
   ```

2. Run all tests with coverage through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --mixin coverage-pytest
   ```

3. Wait for all packages to complete (monitor progress in console)

### Workflow 4: Re-run Failed Tests Only

Use to save time when retesting after fixing failures.

1. Install generated workspace Python requirements, then source the ROS environment:

   ```bash
   ./scripts/ros-dep.sh
   source scripts/setup.bash
   ```

2. Re-run only previously failed tests through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --packages-select-test-failures
   ```

3. Check which tests now pass in the summary

### Workflow 5: Analyze Test Results

Examine test output and failure details.

1. View test summary across workspace through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test-result --all
   ```

2. View verbose test results with error details:

   ```bash
   scripts/with-ros-env.sh python3 -m colcon test-result --verbose
   ```

3. Find detailed test logs:
   ```bash
   ls -la log/latest_test/
   ```

### Workflow 6: Generate Coverage Reports

Process LLVM coverage data and prepare for analysis.

1. Process LLVM coverage for C++ packages through the ROS wrapper:

   ```bash
   scripts/with-ros-env.sh ./packages/cmake/llvm-cov-export-all.py ./build
   ```

2. Coverage data is exported to formats compatible with:
   - Codecov integration
   - VS Code Coverage Gutters extension
   - Web-based coverage viewers

3. View coverage in VS Code:
   - Install "Coverage Gutters" extension
   - Coverage files auto-discovered from `build/**/*.info`

### Workflow 7: View Test Results in Browser (Interactive)

Explore test results with interactive visualization.

1. Launch xunit-viewer server:

   ```bash
   npx -y xunit-viewer -r build --server -o build/xunit-index.html \
     -i Test.xml -i coverage.xml -i package.xml --watch
   ```

2. Open browser to `http://localhost:3000`

3. Browse test results, failures, and coverage interactively

4. Stop server with `Ctrl+C`

## Test Output Structure

| Location                                            | Contains               | Format      |
| --------------------------------------------------- | ---------------------- | ----------- |
| `build/<package_name>/test_results/<package_name>/` | Test execution results | xunit.xml   |
| `log/latest_test/<package_name>/`                   | Test execution logs    | Text/JSON   |
| `build/<package_name>/coverage.info`                | C++ coverage data      | LCOV        |
| `build/<package_name>/.coverage`                    | Python coverage data   | Coverage.py |

## Python ROS Test Structure

- Python ROS tests may use pytest functions or `unittest.TestCase` classes. Match the package's existing style instead of migrating solely for ROS context management.
- For pytest-style tests in `ament_python` packages, declare pytest via `extras_require={'test': ['pytest']}` in `setup.py` (not the removed `tests_require` option), `python3-pytest` in `package.xml`, and add a `test/__init__.py` `load_tests(...)` bridge when colcon discovers the package through unittest.
- For plain pytest suites that do not need external pytest plugins, the `load_tests(...)` bridge may set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` around `pytest.main(...)` to avoid unrelated global plugin incompatibilities.
- When pytest tests construct ROS nodes, use fixtures to initialize `rclpy` and shut it down with `rclpy.try_shutdown()` or only shut it down when the fixture initialized it.
- When unittest tests construct ROS nodes, initialize `rclpy` once per class with `setUpClass()` and register `cls.addClassCleanup(rclpy.try_shutdown)`.
- Keep node, publisher, subscription, client, and action cleanup at the test level with fixture finalizers or `self.addCleanup(...)`.

## Troubleshooting

| Issue                                  | Cause                                      | Solution                                                                     |
| -------------------------------------- | ------------------------------------------ | ---------------------------------------------------------------------------- |
| "No tests found"                       | Package has no tests or wrong package name | Verify `test/` directory exists in package                                   |
| Tests fail with "module not found"     | Generated Python dependencies missing      | Run `./scripts/ros-dep.sh` after the build, then `source scripts/setup.bash` |
| "Command 'pytest' not found"           | Developer `.venv` not synced               | Run `uv sync` and activate `.venv` when needed                               |
| Coverage data not generated            | Coverage not enabled in build              | Rebuild with `--mixin coverage-pytest` during build                          |
| Xunit-viewer won't start               | Node.js or npx not available               | Install Node.js or use alternative viewer                                    |
| Test results missing or incomplete     | Build artifacts cleaned                    | Rebuild packages before testing                                              |
| Tests timeout or hang                  | Test blocking on I/O or infinite loop      | Check test logs in `log/latest_test/`                                        |
| `drqp_gazebo` launch tests are skipped | Default smoke-only mode is active          | Re-run with `DRQP_TEST_MODE=slow` to include the full Gazebo suite           |

## References

- Refer to the official colcon documentation for complete `colcon test` option details
- Refer to ROS 2 and colcon coverage guides for coverage instrumentation best practices
- If your workspace includes helper scripts for automated test execution, document how to use them here
- If you use custom coverage processing tools, document their usage and configuration here
