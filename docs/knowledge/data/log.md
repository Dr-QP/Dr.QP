# Project Update Log

The history of this workspace, newest first. The `ship` skill appends a dated
group on every release; any skill that creates or retires a document adds a line
to the current day's group.

## 2026-09-27

- **Update**: Fixed two [bugs](bugs.md): the deployed robot now starts the
  joystick translator by default (new `load_joystick_translator` bringup
  argument), and bringup declares `load_joystick` once.

## 2026-09-26

- **Creation**: Mapped the codebase in [codebase](codebase.md): 20 components, 3
  flows (teleop command, robot lifecycle, CI), and 2 interfaces (ROS graph,
  robot MCP), each pinned to a `source_digest` (`/agentdev:iwe-map` initial
  mode, commit `map: 20 components, 3 flows, 2 interfaces`).
- **Creation**: Recorded the mapping findings: five open [bugs](bugs.md)
  (deployed robot has no joystick translator, clamping diagnostic rejected by
  the state machine, deploy build tag mismatch, duplicate `load_joystick`, stale
  MCP copy), one proposed [feature](features.md) (translator haptic effects),
  and four [architecture](architecture.md) decisions (effort lifecycle channel,
  time-based gait timing, joystick container split, serial record/replay).
- **Creation**: Extended the [codebase](codebase.md) map by 15 docs:
  `packages/cmake`, `drqp_lint_common`, `py_packages` (with
  `sphinxcontrib_spotlight` and the stale MCP copy), seven `drqp_brain` modules,
  `drqp_kinematics/geometry`, `drqp_control/urdf`, and the `drqp_launch_testing`
  shutdown-behavior suite. Filed two more [bugs](bugs.md): IMU covariance misuse
  and balance toggle desync.

## 2026-08-01

- **Initialization**: Created the project workspace — [product](product.md) as
  the foundation, with [plans](plans.md), [backlog](backlog.md),
  [milestones](milestone.md), and [someday](someday.md) for work, and
  [features](features.md), [bugs](bugs.md), and [releases](releases.md) for
  delivery.
- **Creation**: Seeded the reference side — [spec](spec.md),
  [codebase](codebase.md), [architecture](architecture.md), and
  [concept](concept.md) — with example documents showing the shape of each type.
