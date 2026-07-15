---
name: ros2-workspace-build
description: Build ROS 2 Jazzy packages with colcon in this workspace. Use when compiling a package, its dependency chain, a debug or coverage build, or when inspecting a build failure.
---

# ROS 2 Workspace Build

Run these commands from the workspace root. Always use the ROS wrapper.

## Incremental build

Build the requested package and its workspace dependencies during normal
development:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --packages-up-to <package_name>
```

Use `--packages-select <package_name>` only when its dependencies are already
built and known compatible. `--packages-up-to` is the default handoff to
testing: after it succeeds, test the same package with
`--packages-select <package_name>`.

## Debug or coverage build

Append CMake options without dropping the incremental selector:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --symlink-install \
  --packages-up-to <package_name> \
  --cmake-args -GNinja -D CMAKE_BUILD_TYPE=Debug \
    -D DRQP_ENABLE_COVERAGE=ON
```

Use a full-workspace build only when explicitly requested. See
[ros2-workspace-testing](../ros2-workspace-testing/SKILL.md) for the matching
test and coverage commands.

## Failures and missing ROS

Inspect `log/latest_build/` after a build failure. A package can live anywhere
under `packages/`; discover it with `rg --files packages -g package.xml` rather
than assuming `packages/runtime/`.

If the wrapper reports that ROS is unavailable on the host, run the same
command via [microvm-sandbox](../microvm-sandbox/SKILL.md) when Docker is
available; otherwise use
[remote-codespace-session](../remote-codespace-session/SKILL.md). Do not retry
the command locally.
