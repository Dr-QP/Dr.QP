---
name: microvm-sandbox
description: Run ROS 2 build, test, or lint commands through this repository's devcontainer when a cloud microVM host has no ROS installation. Use for devcontainer up or exec, Docker-backed sandbox testing, and local ROS-environment escalation.
---

# microVM Sandbox

Use this skill after `scripts/with-ros-env.sh` establishes that ROS is absent
from the host and a Docker daemon is available. If Docker is unavailable, use
[remote-codespace-session](../remote-codespace-session/SKILL.md) instead.

Start the development container from the repository root when it is not
already running:

```bash
devcontainer up --workspace-folder /workspace \
  --mount "type=bind,source=/var/run/docker.sock,target=/var/run/docker.sock"
```

Run the original ROS command inside it, retaining the workspace wrapper:

```bash
devcontainer exec --workspace-folder /workspace bash -lc \
  'scripts/with-ros-env.sh python3 -m colcon build --packages-up-to <package_name>'
```

Translate test commands directly from
[ros2-workspace-testing](../ros2-workspace-testing/SKILL.md), preserving their
package selectors. After a successful build, run `./scripts/ros-dep.sh` inside
the container only if a built package needs generated PyPI runtime
requirements; it installs requirements emitted in `build/` or `install/` into
the container's system interpreter. It does not replace `uv sync`, which
manages the developer `.venv`.
