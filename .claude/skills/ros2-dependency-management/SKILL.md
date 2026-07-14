---
name: ros2-dependency-management
description: Add, declare, and install ROS 2, apt, and Python dependencies for packages in this workspace. Use when resolving a missing dependency, editing package dependency metadata, syncing Python tooling, or installing generated package requirements after a build.
---

# ROS 2 Dependency Management

Keep the dependency declaration aligned with how and where the dependency is
used. Package directories may occur anywhere below `packages/`; locate a
package with `rg --files packages -g package.xml` rather than assuming a
`packages/runtime` path.

| Dependency | Declare it in | Install it with |
| --- | --- | --- |
| ROS package, apt library, compiler, or runtime system dependency | The consuming package's `package.xml` (`<depend>`, `<build_depend>`, or `<exec_depend>`) | `rosdep` |
| PyPI runtime dependency of an `ament_python` package | Its packaging metadata (`install_requires` or the active equivalent), plus `package.xml` when it has a rosdep key | The post-build generated-requirements helper when system Python needs it |
| Repository development, documentation, or notebook tool | Root `pyproject.toml` | `uv sync` into `.venv` |

Do not add a PyPI-only package to `package.xml` without a real rosdep key, and
do not put ROS runtime dependencies solely in `pyproject.toml`.

## ROS and apt dependencies

After changing `package.xml`, or when provisioning a fresh ROS environment,
run the workspace helper:

```bash
./scripts/ros-dep.sh
```

It sources the workspace ROS environment, runs `rosdep install --from-paths`
for all workspace packages (which installs only unresolved system packages),
then checks generated `requires.txt` files beneath `build/` and `install/`.
The generated-requirements phase is a no-op before a build produces entries.
It may use `sudo` for system installation, so do not run it as a routine test
setup step.

For a single package, pass that package directory to rosdep after sourcing
ROS; use the actual discovered path:

```bash
scripts/with-ros-env.sh rosdep install --from-paths <package_directory> \
  --ignore-src -y
```

## Python dependencies

For development tools, sync the project environment (Python **>=3.12**):

```bash
uv sync
```

For a new PyPI runtime dependency, add it to the consuming Python package's
packaging metadata, then build its dependency chain:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --packages-up-to <package_name> --symlink-install
```

If that built package subsequently fails to import its declared runtime
dependency under the ROS system interpreter, run the targeted post-build
phase:

```bash
./scripts/ros-dep.sh
```

The helper reads generated `requires.txt` from `build/` and `install/` and
installs only the declared generated requirements into the ROS system
interpreter. It is not a substitute for `uv sync` and must not be repeated
before every test.

## Diagnose a missing dependency

Identify the provider from the build or import error. Check the ROS index with
`rosdep resolve <key>` for a candidate rosdep key; consult the package's
upstream PyPI or project documentation for a PyPI dependency. Do not use the
deprecated legacy PyPI search CLI. Update the appropriate declaration, install
only the affected dependency class, then rebuild with `--packages-up-to`.

If the ROS wrapper cannot source a host installation, follow the escalation in
[ros2-environment-setup](../ros2-environment-setup/SKILL.md) before attempting
rosdep or colcon.
