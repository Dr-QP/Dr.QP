---
type: codebase
description: The two container images — the ROS desktop/dev image the devcontainer and CI run in, and the multi-stage deploy image that runs on the robot — plus Ansible provisioning.
source: docker/ros
source_digest: sha256:2dc994de0f9308e65f848b57f2abff68198ae8233aa7643329c0962f36eda5d6
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: docker/ros
---

# docker/ros

The image definitions. CI publishes both images as
`ghcr.io/dr-qp/<distro>-ros-desktop` and `ghcr.io/dr-qp/<distro>-ros-deploy`.

## Contains

[ansible](ros/ansible.md)

## Public surface

- **`desktop/ros-desktop.Dockerfile`:** builds on the digest-pinned
  `ghcr.io/plume-works/agent-desktop` image and runs the Ansible
  `20_ros_setup.yml` playbook in the image. The devcontainer, CI, and CodeQL
  jobs run in this image.

- **`deploy/ros-deploy.Dockerfile`** has three stages:

  1. `cacher`: `vcs import` the repo at `GIT_SHA`.
  2. `builder`, on the desktop image: rosdep over `packages/runtime` and
     `packages/vendor`, then `colcon build --packages-up-to drqp_brain` with the
     `ninja rel-with-deb-info` mixins.
  3. `deploy`, on `ros-base`: copies `install/` and runs as the `rosdeploy`
     user.

  The default command is `ros2 launch drqp_brain bringup.launch.py`.

- **`deploy/ros_entrypoint.sh`:** sources ROS and the overlay, then runs
  `exec "$@"`.

- **`deploy/install-overlay-python-requirements.py`:** installs the generated
  overlay Python requirements in one pip call. `scripts/ros-dep.sh` also uses
  it.

## Depends on

- [packages/runtime](../packages/runtime.md) (the deploy build input)

## Invariants & gotchas

- The deploy image's default command passes no launch arguments, so
  `load_joystick` stays `false` and `drqp_joystick_translator` is not started.
  The on-robot joystick container runs only `game_controller_node`. From the
  code alone, nothing on the robot translates `/joy` into
  `/robot/movement_command`. Whether another path covers this is *unknown*.
- `deploy/build.sh` tags its local build `ghcr.io/dr-qp/ros-deploy`, which
  differs from the `jazzy-ros-deploy` name that CI publishes and Ansible pulls.
- The deploy image clones the repo from `GIT_REPO` at `GIT_SHA`, not from the
  build context, so a local build uses the pushed commit.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `docker/ros/desktop/ros-desktop.Dockerfile:5` — pinned base image
- `docker/ros/desktop/ros-desktop.Dockerfile:29` — Ansible ROS setup in-image
- `docker/ros/deploy/ros-deploy.Dockerfile:9` — `cacher` stage
- `docker/ros/deploy/ros-deploy.Dockerfile:33` — `DEPLOY_PACKAGE="drqp_brain"`
- `docker/ros/deploy/ros-deploy.Dockerfile:87` — default bringup command
- `docker/ros/deploy/build.sh:4` — local image tag
- `.github/workflows/ci.yml:357` — published deploy image name
