# Project Update Log

The history of this workspace, newest first. The `ship` skill appends a dated
group on every release; any skill that creates or retires a document adds a line
to the current day's group.

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

## 2026-08-01

- **Initialization**: Created the project workspace — [product](product.md) as
  the foundation, with [plans](plans.md), [backlog](backlog.md),
  [milestones](milestone.md), and [someday](someday.md) for work, and
  [features](features.md), [bugs](bugs.md), and [releases](releases.md) for
  delivery.
- **Creation**: Seeded the reference side — [spec](spec.md),
  [codebase](codebase.md), [architecture](architecture.md), and
  [concept](concept.md) — with example documents showing the shape of each type.
