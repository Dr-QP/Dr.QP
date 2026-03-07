---
description: 'Guidelines for working in sandbox microVM'
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

Before build or ROS 2 commands, source the setup script inside the exec:

```bash
devcontainer exec --workspace-folder /workspace bash -c "
  export ROS_DISTRO=jazzy CC=clang CXX=clang++ CMAKE_EXPORT_COMPILE_COMMANDS=1
  source scripts/setup.bash
  <ros2 commands here>
"
```

For test runs that need the production venv (with `python-statemachine`), use `source scripts/setup.bash --update-venv` **after a successful `colcon build`**. The `--update-venv` flag scans `build/` and `install/` for `requires.txt` files. Inside the devcontainer these directories exist as Docker volume mount points, but they will be empty until the first build — so `find` may return nothing (harmless). Outside the devcontainer the directories may not exist at all, producing noisy errors. Build, test, and lint commands are documented in the CI/CD Pipeline section above.

### Gotchas

- The colcon mixin config lives at `/root/.colcon/` inside the image and is pre-configured.
- `--symlink-install` is required for Python coverage collection and hot-reloading.
- Some packages emit CMake warnings about unused `DRQP_ENABLE_COVERAGE` — these are benign.
- The `drqp_gazebo` simulation tests run headless and take ~20 seconds.
- The production venv (`.venv-prod`) is separate from the dev venv (`.venv`). The `setup.bash --update-venv` flag populates `.venv-prod` from `requires.txt` files in `build/` and `install/` directories.
- The cloud VM's host Docker daemon must be running before `devcontainer up`. The update script starts it with `sudo dockerd &>/tmp/dockerd.log &`. The host daemon uses `fuse-overlayfs` as configured in `/etc/docker/daemon.json` because the cloud VM runs inside a Firecracker VM where the default `overlay2` driver is not supported. This is unrelated to Docker inside the devcontainer (which is handled by `scripts/devcontainer-start-docker.sh`).
