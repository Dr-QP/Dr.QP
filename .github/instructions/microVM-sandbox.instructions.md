---
description: 'Guidelines for working in sandbox microVM'
applyTo: '**/*'
---

# microVM sandbox workflows

## Development Container

All build, test, and lint commands run inside the devcontainer defined in `.devcontainer/devcontainer.json`. The cloud VM does not have ROS 2 installed natively.

The update script starts Docker and brings up the devcontainer using:

```bash
devcontainer up --workspace-folder /workspace \
  --mount "type=bind,source=/var/run/docker.sock,target=/var/run/docker.sock"
```

The host Docker socket mount lets `scripts/devcontainer-start-docker.sh` (`postStartCommand`) detect that Docker is already accessible and exit cleanly, avoiding a redundant `dockerd` startup inside the container. The `devcontainer up` command runs all lifecycle hooks defined in `devcontainer.json`: `postCreateCommand` (directory ownership), `postStartCommand` (Docker check), and `postAttachCommand` (rosdep install, Python venv creation, xpra startup).

### Running Commands

Use `devcontainer exec` to run commands inside the container:

```bash
devcontainer exec --workspace-folder /workspace bash -c "<command>"
```

Build or ROS 2 commands, run them through the wrapper inside the exec:

```bash
devcontainer exec --workspace-folder /workspace bash -lc "
  scripts/with-ros-env.sh <ros2 command here>
"
```

For test runs that need generated workspace Python dependencies (for example `python-statemachine`), run `./scripts/ros-dep.sh` **after a successful `colcon build`** before `source scripts/setup.bash` and `colcon test`. The helper scans `build/` and `install/` for generated `requires.txt` files and installs them into the container's system interpreter with `pip --break-system-packages`. Inside the devcontainer these directories exist as Docker volume mount points, but they may be empty until the first build — that should no-op cleanly. Build, test, and lint commands should mirror those used in the repository CI workflows (see `.github/workflows/ci.yml`).

### Gotchas

- The colcon mixin config lives at `/root/.colcon/` inside the image and is pre-configured.
- `--symlink-install` is required for Python coverage collection and hot-reloading.
- Some packages emit CMake warnings about unused `DRQP_ENABLE_COVERAGE` — these are benign.
- The `drqp_gazebo` simulation tests run headless and take ~20 seconds.
- Generated workspace Python requirements are installed into the container's system interpreter by `./scripts/ros-dep.sh`; the developer `.venv` remains separate for local tooling.
- The cloud VM's host Docker daemon must be running before `devcontainer up`.
