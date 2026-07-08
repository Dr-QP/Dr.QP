---
id: RM-06
title: Autonomous navigation (Nav2)
status: proposed
depends_on: [RM-05]
packages: [drqp_gazebo, drqp_robot_mcp, 'NEW: drqp_navigation']
---

# RM-06 — Navigation

## Objective

Nav2 drives the robot to map goals and named locations in sim (CI) and in the home, through the
RM-02 `cmd_vel` path, preemptable by joystick and kill switch.

## Interfaces

- NEW package `drqp_navigation`: Nav2 bringup launch + params (`nav2_params.yaml`), footprint
  polygon (~0.35 × 0.30 m), map from RM-05 `nav2_map_server`.
- Controller: MPPI (preferred) or DWB with `vy` enabled — omnidirectional; velocity/accel limits
  from RM-02 `velocity_model.yaml` (single source: generate or cross-check in test).
- `cmd_vel` wiring: Nav2 output → `twist_mux` input `/cmd_vel_nav` (priority < joystick) →
  `/cmd_vel`.
- Goal API: standard `NavigateToPose` / `NavigateThroughPoses` actions; NEW node
  `named_goal_server`: `robot/go_to_location` action or service resolving `locations.yaml` →
  `NavigateToPose`.
- MCP tools in `drqp_robot_mcp`: `robot.go_to(location_or_pose)`, `robot.cancel_goal`,
  `robot.nav_status`.
- Recoveries: builtin backup + spin; NEW behavior plugin/BT node `re_settle` (stop, lower and
  re-raise body via existing trajectory path).

## Acceptance criteria

- [ ] Launch test (apartment world): `NavigateToPose` 3 m away through doorway succeeds within
      timeout; final pose within 0.20 m / 15°.
- [ ] Omni test: goal directly beside robot reached using lateral velocity (assert `vy` commanded).
- [ ] Preemption test: joystick activity during nav ⇒ Nav2 `cmd_vel` ignored by mux within one
      cycle; nav resumes/cancels per config.
- [ ] Safety: goals rejected unless `/robot_state == torque_on`; kill switch mid-nav ⇒ trajectory
      stops, state machine handles as today (test).
- [ ] `robot.go_to("kitchen")` via MCP succeeds in sim.
- [ ] Hardware: ≥ 20-run success-rate measurement documented (target ≥ 80 % first pass).

## Constraints

- Local costmap starts with static map + inflation only; obstacle layers (mono floor-plane
  detection, ToF sensor) are separate follow-up specs — leave plugin slots configured.
- No Dr.QP-specific forks of Nav2; configuration and thin plugins only.
- Nav2 stack may run off-board; only `twist_mux` + brain are safety-local. Stale `/cmd_vel_nav`
  handled by RM-02 watchdog.
