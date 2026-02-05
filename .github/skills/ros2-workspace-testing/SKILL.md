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
- `scripts/setup.bash` available in workspace root
- Packages already built with `colcon build`
- Python 3.8+ for coverage tools
- Optional: Node.js for interactive result viewer (xunit-viewer)

## Step-by-Step Workflows

### Workflow 1: Test Specific Package (Recommended for Development)

Use for rapid iteration when developing and testing a single package.

1. Source the ROS environment with venv updates:
   ```bash
   source scripts/setup.bash --update-venv
   ```

2. Run tests for the package only:
   ```bash
   python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --packages-select <package_name>
   ```

3. Check test results in console output (summary appears at end)

4. View detailed test logs in `log/latest_test/<package_name>/`

**Typical execution time**: 1-5 minutes depending on test count

### Workflow 2: Test with Coverage Collection

Use when you need code coverage metrics alongside test execution.

1. Source the ROS environment with venv updates:
   ```bash
   source scripts/setup.bash --update-venv
   ```

2. Run tests with coverage enabled:
   ```bash
   python3 -m colcon test \
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

1. Source the ROS environment with venv updates:
   ```bash
   source scripts/setup.bash --update-venv
   ```

2. Run all tests with coverage:
   ```bash
   python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --mixin coverage-pytest
   ```

3. Wait for all packages to complete (monitor progress in console)

### Workflow 4: Re-run Failed Tests Only

Use to save time when retesting after fixing failures.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash --update-venv
   ```

2. Re-run only previously failed tests:
   ```bash
   python3 -m colcon test \
     --event-handlers console_cohesion+ summary+ status+ \
     --return-code-on-test-failure \
     --packages-select-test-failures
   ```

3. Check which tests now pass in the summary

### Workflow 5: Analyze Test Results

Examine test output and failure details.

1. View test summary across workspace:
   ```bash
   python3 -m colcon test-result --all
   ```

2. View verbose test results with error details:
   ```bash
   python3 -m colcon test-result --verbose
   ```

3. Find detailed test logs:
   ```bash
   ls -la log/latest_test/
   ```

### Workflow 6: Generate Coverage Reports

Process LLVM coverage data and prepare for analysis.

1. Source the ROS environment:
   ```bash
   source scripts/setup.bash
   ```

2. Process LLVM coverage for C++ packages:
   ```bash
   ./packages/cmake/llvm-cov-export-all.py ./build
   ```

3. Coverage data is exported to formats compatible with:
   - Codecov integration
   - VS Code Coverage Gutters extension
   - Web-based coverage viewers

4. View coverage in VS Code:
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

| Location | Contains | Format |
|----------|----------|--------|
| `build/<package_name>/test_results/<package_name>/` | Test execution results | xunit.xml |
| `log/latest_test/<package_name>/` | Test execution logs | Text/JSON |
| `build/<package_name>/coverage.info` | C++ coverage data | LCOV |
| `build/<package_name>/.coverage` | Python coverage data | Coverage.py |

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "No tests found" | Package has no tests or wrong package name | Verify `test/` directory exists in package |
| Tests fail with "module not found" | Python dependencies missing | Run `source scripts/setup.bash --update-venv` |
| "Command 'pytest' not found" | venv not updated | Run `source scripts/setup.bash --update-venv` |
| Coverage data not generated | Coverage not enabled in build | Rebuild with `--mixin coverage-pytest` during build |
| Xunit-viewer won't start | Node.js or npx not available | Install Node.js or use alternative viewer |
| Test results missing or incomplete | Build artifacts cleaned | Rebuild packages before testing |
| Tests timeout or hang | Test blocking on I/O or infinite loop | Check test logs in `log/latest_test/` |

## References

- Refer to the official colcon documentation for complete `colcon test` option details
- Refer to ROS 2 and colcon coverage guides for coverage instrumentation best practices
- If your workspace includes helper scripts for automated test execution, document how to use them here
- If you use custom coverage processing tools, document their usage and configuration here

