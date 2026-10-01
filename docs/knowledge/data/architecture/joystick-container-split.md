---
type: architecture
description: Why the robot runs the gamepad node in its own auto-restarting container, separate from the control stack container.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: ansible
  resource: docker/ros/ansible/playbooks/100_startup_service.yml
- id: commit
  resource: git 728f8e4 "Setup service on robot pi that brings up deployment container on startup (#165)"
---

# Joystick in its own container

## Decision

On the Pi, `100_startup_service.yml` installs two systemd services that run the
same deploy image:

- **`drqp-control-service`** runs the default bringup. It gets every
  `/dev/ttySC*` device and the dialout group.
- **`drqp-joystick-service`** is a shell loop that watches `/dev/input/event*`.
  When the device set changes, it stops the joystick container and starts it
  again with the new `--device` list, running only
  `ros2 run drqp_joy game_controller_node`.

Both containers use host networking, IPC, and PID, and run as the host's primary
user and group.

## Why

The playbook's header comment records the reasons:

- Docker passes only the devices that exist at container start. Bluetooth
  gamepads are not present at OS startup and come and go.
- Passing `/dev/input/event*` into a long-lived container breaks reconnection,
  probably because Docker keeps the original device nodes open. The joystick
  then cannot reconnect until the container restarts.
- Restarting a container that holds only the joy node is cheap and does not
  disturb the control stack.
- Matching the host user and sharing IPC lets shared-memory DDS work between the
  two containers.

## Rejected alternatives

- **`--device` passthrough found at startup:** fails for devices that appear
  later.
- **`--device-cgroup-rule='c 13:* rwm'` hot-plug rules:** "doesn't work for some
  reason for input/event devices".
- **`--privileged`:** also passes only the devices present at start.
- **Forcing DDS over UDPv4** (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`) in place of a
  shared user and IPC: noted as an option, not used.

## Consequences

- Nodes that must run with the gamepad have to be placed in one of the two
  containers on purpose. The joystick translator fell between them until bringup
  gave it its own `load_joystick_translator` argument; it now runs in the
  control container (see
  [Bug: deployed robot has no joystick translator](../bugs/deployed-robot-has-no-joystick-translator.md)).
