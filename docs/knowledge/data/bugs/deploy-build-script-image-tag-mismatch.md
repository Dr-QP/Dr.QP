---
type: bug
description: docker/ros/deploy/build.sh tags local builds ghcr.io/dr-qp/ros-deploy, not the jazzy-ros-deploy name that CI publishes and Ansible pulls.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: build
  resource: docker/ros/deploy/build.sh
- id: ci
  resource: .github/workflows/ci.yml
---

# Bug: deploy build script uses the wrong image tag

## Symptom

A locally built deploy image is named `ghcr.io/dr-qp/ros-deploy`. The robot
services pull `ghcr.io/dr-qp/jazzy-ros-deploy:edge`, so a local build never
replaces what the robot runs. Pushing it would also create a stray package.

## Reproduction

1. Run `docker/ros/deploy/build.sh`.
2. `docker images` lists `ghcr.io/dr-qp/ros-deploy`, not
   `ghcr.io/dr-qp/jazzy-ros-deploy`.

## Root cause

`build.sh` hard-codes `IMAGE=ghcr.io/dr-qp/ros-deploy`. CI builds
`${REGISTRY}/dr-qp/${ROS_DISTRO}-ros-deploy`, and Ansible uses
`jazzy-ros-deploy:edge`.

## Fix

Set `IMAGE="ghcr.io/dr-qp/${ROS_DISTRO}-ros-deploy"` in `build.sh`. The script
already defines `ROS_DISTRO=jazzy`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `docker/ros/deploy/build.sh:4` — `IMAGE=ghcr.io/dr-qp/ros-deploy`
- `.github/workflows/ci.yml:357` — published name `<distro>-ros-deploy`
- `docker/ros/ansible/playbooks/100_startup_service.yml:35` — pulled
  `jazzy-ros-deploy:edge`
