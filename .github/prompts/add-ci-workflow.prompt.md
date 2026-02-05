---
description: 'Create GitHub Actions workflow file configured for ROS 2 workspace build, test, coverage, or documentation'
name: 'add-ci-workflow'
agent: 'agent'
tools: ['read', 'edit', 'search']
argument-hint: '[--type build|test|coverage|docs]'
---

# Add CI/CD Workflow

Generate GitHub Actions workflow file for ROS 2 workspace automation including build, test, coverage collection, and documentation generation.

## When to Use This Prompt

- Adding CI/CD automation to the project
- Creating build workflows for ROS 2 packages
- Setting up automated testing and coverage reporting
- Configuring documentation generation
- Need workflow templates that follow project conventions

## Prerequisites

- GitHub repository with ROS 2 workspace
- Understanding of workflow purpose (build, test, coverage, docs)
- Docker setup available (for containerized builds)
- Codecov or similar service configured (for coverage workflows)

## Inputs

### Required Inputs

- **Workflow Type** `${input:workflowType:test}`: Type of workflow (`build`, `test`, `coverage`, `docs`, or `full`)

### Optional Inputs

- **Workflow Name**: Custom name for the workflow (default: based on type)
- **Trigger Events**: When to run (default: `[push, pull_request]`)
- **ROS Distro**: ROS 2 distribution (default: `jazzy`)
- **Target Packages**: Specific packages to build/test (default: all)
- **Branch Filter**: Branches to run on (default: `main`)

## Workflow

### Step 1: Determine Workflow Configuration

Based on workflow type, determine:

| Type | Purpose | Key Steps | Artifacts |
|------|---------|-----------|-----------|
| `build` | Build workspace | Install deps, build packages | Build logs |
| `test` | Run tests | Build, run tests, collect results | Test results |
| `coverage` | Code coverage | Build with coverage, test, upload | Coverage reports |
| `docs` | Generate docs | Build docs, deploy to pages | Documentation |
| `full` | Complete CI/CD | All of the above | All artifacts |

### Step 2: Create Workflow File

Create `.github/workflows/<workflow_name>.yml`

#### Basic Workflow Structure:

```yaml
name: <Workflow Name>

on:
  push:
    branches:
      - main
  pull_request:
    branches:
      - main
  merge_group:
    types:
      - checks_requested

concurrency:
  group: ${{ github.workflow }}-${{ github.ref || github.run_id }}
  cancel-in-progress: true

env:
  ROS_DISTRO: jazzy

jobs:
  <job_name>:
    name: <Job Display Name>
    runs-on: ubuntu-24.04
    timeout-minutes: 30
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      # Additional steps based on workflow type
```

### Step 3: Add Build Job

For `build`, `test`, `coverage`, or `full` workflows:

```yaml
  build:
    name: Build ROS 2 Workspace
    runs-on: ubuntu-24.04
    container:
      image: ghcr.io/dr-qp/jazzy-ros-desktop:latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          ./scripts/ros-dep.sh

      - name: Build workspace
        run: |
          source /opt/ros/${{ env.ROS_DISTRO }}/setup.bash
          python3 -m colcon build \
            --mixin ninja rel-with-deb-info \
            --event-handlers=console_cohesion+ \
            --symlink-install

      - name: Upload build logs
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: build-logs
          path: log/
```

### Step 4: Add Test Job

For `test`, `coverage`, or `full` workflows:

```yaml
  test:
    name: Run Tests
    runs-on: ubuntu-24.04
    needs: build
    container:
      image: ghcr.io/dr-qp/jazzy-ros-desktop:latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Install dependencies
        run: ./scripts/ros-dep.sh

      - name: Build workspace
        run: |
          source /opt/ros/${{ env.ROS_DISTRO }}/setup.bash
          python3 -m colcon build \
            --mixin ninja rel-with-deb-info \
            --event-handlers=console_cohesion+ \
            --symlink-install

      - name: Run tests
        run: |
          source install/setup.bash
          python3 -m colcon test \
            --event-handlers=console_cohesion+ \
            --return-code-on-test-failure

      - name: Get test results
        if: always()
        run: |
          python3 -m colcon test-result --verbose

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-results
          path: |
            build/*/test_results/
            log/
```

### Step 5: Add Coverage Job

For `coverage` or `full` workflows:

```yaml
  coverage:
    name: Code Coverage
    runs-on: ubuntu-24.04
    container:
      image: ghcr.io/dr-qp/jazzy-ros-desktop:latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Install dependencies
        run: ./scripts/ros-dep.sh

      - name: Build with coverage
        run: |
          source /opt/ros/${{ env.ROS_DISTRO }}/setup.bash
          python3 -m colcon build \
            --mixin coverage-pytest ninja rel-with-deb-info \
            --event-handlers=console_cohesion+ \
            --symlink-install \
            --cmake-args -D DRQP_ENABLE_COVERAGE=ON

      - name: Run tests with coverage
        run: |
          source install/setup.bash
          python3 -m colcon test \
            --mixin coverage-pytest \
            --event-handlers=console_cohesion+ \
            --return-code-on-test-failure

      - name: Process coverage
        run: |
          ./packages/cmake/llvm-cov-export-all.py ./build

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v4
        with:
          token: ${{ secrets.CODECOV_TOKEN }}
          files: ./coverage.xml
          flags: unittests
          name: ros2-coverage

      - name: Upload coverage artifacts
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: coverage-reports
          path: |
            coverage.xml
            htmlcov/
```

### Step 6: Add Documentation Job

For `docs` or `full` workflows:

```yaml
  docs:
    name: Generate Documentation
    runs-on: ubuntu-24.04
    container:
      image: ghcr.io/dr-qp/jazzy-ros-desktop:latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          ./scripts/ros-dep.sh
          apt-get update && apt-get install -y doxygen graphviz

      - name: Build workspace
        run: |
          source /opt/ros/${{ env.ROS_DISTRO }}/setup.bash
          python3 -m colcon build --mixin ninja

      - name: Generate documentation
        run: |
          # Add documentation generation commands
          # Example: rosdoc2 build

      - name: Upload documentation
        uses: actions/upload-artifact@v4
        with:
          name: documentation
          path: docs/

      - name: Deploy to GitHub Pages
        if: github.ref == 'refs/heads/main'
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./docs
```

### Step 7: Add Matrix Strategy (Optional)

For testing across multiple configurations:

```yaml
  test-matrix:
    name: Test Matrix
    runs-on: ubuntu-24.04
    strategy:
      matrix:
        ros_distro: [jazzy, rolling]
        include:
          - ros_distro: jazzy
            container: ghcr.io/dr-qp/jazzy-ros-desktop:latest
          - ros_distro: rolling
            container: ros:rolling
      fail-fast: false
    container:
      image: ${{ matrix.container }}
    steps:
      # Test steps here
```

### Step 8: Add Conditional Execution

Add path filters to run only when relevant files change:

```yaml
jobs:
  paths-filter:
    name: Check if build needed
    runs-on: ubuntu-24.04
    outputs:
      should_run: ${{ steps.filter.outputs.pass }}
    steps:
      - uses: actions/checkout@v4
      - uses: dorny/paths-filter@v2
        id: filter
        with:
          filters: |
            pass:
              - 'packages/**'
              - 'CMakeLists.txt'
              - '.github/workflows/**'

  build:
    needs: paths-filter
    if: needs.paths-filter.outputs.should_run == 'true'
    # Build job steps
```

## Output Expectations

### Success Criteria

- Workflow file created in `.github/workflows/`
- Workflow includes all necessary jobs for the type
- Jobs properly depend on each other (test needs build, etc.)
- Artifacts are uploaded for debugging
- Workflow follows project conventions (uses project Docker images, scripts)
- Proper error handling and always() conditions for cleanup

### Generated Output Summary

```
✅ Created GitHub Actions workflow: <workflow_name>
📋 Workflow type: <type>
🔄 Triggers: <events>

📝 Jobs included:
   - <job_1>: <description>
   - <job_2>: <description>
   - ...

⚙️  Configuration:
   - ROS Distro: <ros_distro>
   - Docker image: <image>
   - Timeout: <minutes> minutes

📦 Artifacts:
   - <artifact_1>
   - <artifact_2>

🧪 Test the workflow:
   1. Commit and push: git add .github/workflows/<workflow_name>.yml && git commit && git push
   2. Check workflow runs: gh run list --workflow=<workflow_name>
   3. View logs: gh run view <run_id>
```

## Validation Steps

1. **Validate YAML syntax**: Use `yamllint` or GitHub Actions validator
2. **Check workflow in UI**: Navigate to Actions tab in GitHub
3. **Test workflow**: Push a commit to trigger the workflow
4. **Monitor execution**: Watch workflow run in GitHub Actions
5. **Check artifacts**: Verify artifacts are uploaded correctly
6. **Review logs**: Ensure all steps complete successfully

## Quality Assurance

- [ ] Workflow file placed in `.github/workflows/`
- [ ] YAML syntax is valid
- [ ] All required jobs are included
- [ ] Job dependencies are correct (uses `needs:`)
- [ ] Artifacts uploaded with `if: always()` for debugging
- [ ] Proper ROS 2 environment sourcing
- [ ] Uses project conventions (Docker images, scripts)
- [ ] Timeout configured to prevent hanging
- [ ] Concurrency control to cancel outdated runs
- [ ] Secrets properly referenced (CODECOV_TOKEN, etc.)

## Edge Cases

- **No Docker image**: Provide fallback to install ROS 2 directly
- **Custom dependencies**: Add steps to install additional dependencies
- **Private repositories**: Handle authentication for accessing private dependencies
- **Long-running tests**: Increase timeout or split into multiple jobs
- **Large artifacts**: Compress before uploading or use retention settings
- **Matrix failures**: Use `fail-fast: false` to see all results

## Related Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [ROS 2 CI/CD Tutorial](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html#testing-and-linting)
- [Codecov Documentation](https://docs.codecov.com/)
- Existing workflow: `.github/workflows/ci.yml`
- [ros2-workspace-build skill](/.github/skills/ros2-workspace-build/SKILL.md)
- [ros2-workspace-testing skill](/.github/skills/ros2-workspace-testing/SKILL.md)
