---
type: codebase
description: Advisory file locks, scoped per ROS domain, that stop a second brain, state node, bringup or simulation from starting on the same host.
source: packages/runtime/drqp_brain/drqp_brain/instance_guard.py
source_digest: sha256:b455917a9504013a1a2027705cfa51421937b5ce6b0737eb447e8310e9629a18
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/instance_guard.py
---

# instance_guard

This module prevents duplicate processes on one host with `fcntl.flock`
exclusive, non-blocking locks. Each lock is named per ROS domain, so separate
domains (for example parallel CI jobs) never collide.

## Public surface

- `InstanceGuard(name, lock_dir=None)`: `acquire()` (raises
  `InstanceAlreadyRunningError`), `try_acquire()`, `release()`, and use as a
  context manager. It writes the holder's PID into the lock file.
- `domain_instance_guard(name, lock_dir=None, ros_domain_id=None)`: creates the
  lock `<name>-domain-<ROS_DOMAIN_ID>`.
- `make_launch_instance_guard(name)`: an `OpaqueFunction` launch action that
  holds the lock for the life of the launch.
- `get_runtime_directory(app)`: `$ROS_HOME/<app>`, or `~/.ros/<app>`. Locks live
  in its `tmp/` subdirectory.

## How it works

The users are:

- `drqp_brain` and `drqp_robot_state` wrap `main` in
  `domain_instance_guard(...)`, then exit with an error on contention.
- `bringup.launch.py` guards `drqp_brain_stack`, and `sim.launch.py` guards
  `drqp_gazebo_sim`.
- When the launch guard finds the lock taken, it logs an error and emits
  `Shutdown`. It releases the lock on `OnShutdown`.

## Invariants & gotchas

- The locks are per host and per filesystem: `ROS_HOME` or `$HOME`. Two
  containers, or two machines, on one domain are not caught by the lock.
  `drqp_brain` also checks the ROS graph for an existing node named `drqp_brain`
  for that case.
- `flock` is released by the kernel when the process dies, so a stale lock file
  cannot block a restart.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:32` —
  `InstanceGuard`
- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:54` — non-blocking
  `try_acquire`
- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:92` — runtime
  directory
- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:116` —
  `domain_instance_guard`
- `packages/runtime/drqp_brain/drqp_brain/instance_guard.py:136` — launch guard
  shutdown on contention
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:119` — graph-wide
  node-name check
