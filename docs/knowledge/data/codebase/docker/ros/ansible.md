---
type: codebase
description: Ansible playbooks and roles that install ROS 2 into images and hosts, prepare the Raspberry Pi, and run the robot stack as systemd-managed containers.
source: docker/ros/ansible
source_digest: sha256:670617de8a8642d211a5b2db03b9f16f683708a87925b40c3f4ea5e29c6c2c20
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: docker/ros/ansible
---

# ansible

"Ansible playbooks to set up a ROS 2 development environment." They also
provision the robot. The user guide is `docs/source/GettingStarted/ansible.md`.

## Public surface

- Inventories: `localhost.yml`, `real-robots.yml`, `virtual-bots.yml`.
- Numbered playbooks, run in order:
  - `0_start_virtual_bots` and `9999_stop_virtual_bots`
  - `1_pam_ssh_agent_auth`
  - `5_raspberry_pi_setup`: SC16IS752 overlay in `config.txt`, power settings,
    and networkd wait.
  - `10_install_docker`
  - `20_ros_setup`: the role chain `extra_facts` → repos → dev tools → colcon →
    rosdep → prebuilt or source ROS install → patches → dependencies → clang.
  - `100_startup_service`
  - `200_pair_controller`
- 17 roles under `roles/`.

## How it works

`100_startup_service` installs three systemd units:

- `drqp-control-service`: runs `ghcr.io/dr-qp/jazzy-ros-deploy:edge` with host
  networking, IPC, and PID. It passes every `/dev/ttySC*` device and adds the
  dialout group.
- `drqp-joystick-service`: a watch loop that restarts a second container running
  `ros2 run drqp_joy game_controller_node` whenever the set of input devices
  changes.
- `drqp-docker-prune-service`.

## Depends on

- [docker/ros](../ros.md) (the deploy image)

## Invariants & gotchas

- Docker cannot hot-plug `/dev/input/event*`, even with `--privileged`. The
  joystick therefore runs in its own container that restarts on reconnect.
- The containers run as the host's primary user and group, so that shared-memory
  DDS works across them.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `docker/ros/ansible/playbooks/100_startup_service.yml:35` — deploy image tag
- `docker/ros/ansible/playbooks/100_startup_service.yml:68` — control container
  `docker run`
- `docker/ros/ansible/playbooks/100_startup_service.yml:124` — joystick
  container command
- `docker/ros/ansible/playbooks/20_ros_setup.yml:20` — ROS setup role chain
- `docker/ros/ansible/playbooks/5_raspberry_pi_setup.yml:23` — SC16IS752
  `config.txt`
