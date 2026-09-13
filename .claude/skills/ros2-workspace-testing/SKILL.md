---
name: ros2-workspace-testing
description: Run and investigate ROS 2 package tests with colcon in this workspace. Use when testing a package, rerunning failures, collecting coverage, or locating ROS test logs.
---

# ROS 2 Workspace Testing

Build first with [ros2-workspace-build](../ros2-workspace-build/SKILL.md),
normally using `--packages-up-to <package_name>`. Run test commands from the
workspace root through `scripts/with-ros-env.sh`.

## Test one package

```bash
scripts/with-ros-env.sh python3 -m colcon test \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure \
  --packages-select <package_name>
```

Use the coverage mixin when coverage is requested:

```bash
scripts/with-ros-env.sh python3 -m colcon test \
  --event-handlers console_cohesion+ summary+ status+ \
  --return-code-on-test-failure \
  --packages-select <package_name> \
  --mixin coverage-pytest
```

For `drqp_gazebo`'s complete launch suite, add `DRQP_TEST_MODE=slow` to the
command environment. Full-workspace tests are expensive and require an
explicit request.

## Re-run and inspect failures

```bash
scripts/with-ros-env.sh python3 -m colcon test \
  --return-code-on-test-failure \
  --packages-select-test-failures

scripts/with-ros-env.sh python3 -m colcon test-result --all --verbose
```

Read `log/latest_test/<package_name>/stdout_stderr.log` or `streams.log` for a
failing package. Use `PYTEST_ADDOPTS=-rA` only when output from passing pytest
or `launch_pytest` tests is needed.

## Reproduce a CI test failure locally

CI uploads the colcon logs and xUnit reports as run artifacts. Download them
before reproducing, so the local run is checking the same failure:

```bash
gh api repos/<owner>/<repo>/actions/runs/<run-id>/artifacts \
  | grep -o '"name":"[^"]*"' | grep 'colcon-'

gh run download <run-id> --repo <owner>/<repo> \
  -n colcon-logs-<arch> -n colcon-test-reports-<arch> \
  -D ./.tmp/actions-run-<run-id>
```

Artifact names are `colcon-logs-<arch>` and `colcon-test-reports-<arch>`.
Inspect the downloaded xUnit XML and colcon log directories first, then
reproduce the specific package:

```bash
scripts/with-ros-env.sh colcon test --packages-select <package_name>
scripts/with-ros-env.sh colcon test-result --verbose
```

For a coverage regression flagged in review, rebuild with coverage enabled
before re-testing:

```bash
scripts/with-ros-env.sh colcon build --packages-up-to <package_name> \
  --cmake-args -DDRQP_ENABLE_COVERAGE=ON
scripts/with-ros-env.sh colcon test --packages-select <package_name> \
  --mixin coverage-pytest
```

Use the `/agentdev:extract-github-actions-logs` skill to resolve a run or job
URL into the `gh` commands that fetch its logs.

## Python test conventions

Write ROS Python tests with pytest and fixtures. For launch integration tests,
use [launch-testing](../launch-testing/SKILL.md). Do not add legacy
class-based test bridges.

## Dependency failures

Do **not** run `scripts/ros-dep.sh` before every test. On a fresh ROS image or
after changing `package.xml`, install declared ROS/apt dependencies with
[ros2-dependency-management](../ros2-dependency-management/SKILL.md). After a
successful build, run its generated-requirements step only when a built
package's declared PyPI runtime requirement is missing. Then rerun the
specific test.

## No ROS installation on the host

If the wrapper cannot source ROS, run the same command via
`/agentdev:microvm-sandbox` with Docker, or `/agentdev:remote-codespace-session`
without it. Both forward the command verbatim, so pass the ROS wrapper as part
of it:

```bash
scripts/with-ros-env.sh python3 -m colcon test --packages-select <package_name>
```
