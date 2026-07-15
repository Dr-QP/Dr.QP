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
[microvm-sandbox](../microvm-sandbox/SKILL.md) with Docker, or
[remote-codespace-session](../remote-codespace-session/SKILL.md) without it.
