---
id: RM-02
title: Metric velocity interface (cmd_vel)
status: proposed
depends_on: [RM-01]
packages: [drqp_brain, drqp_gazebo, drqp_interfaces]
---

# RM-02 — Metric velocity interface

## Objective

Robot executes `geometry_msgs/Twist` on `/cmd_vel` at true metric velocity (±20 % in sim);
joystick teleop unchanged; commands arbitrated joystick > autonomy.

## Interfaces

- Subscribe `/cmd_vel` (`geometry_msgs/Twist`): use `linear.x`, `linear.y`, `angular.z`;
  ignore others. Watchdog: no message for 0.5 s ⇒ ramp to stop.
- Keep `/robot/movement_command` (`MovementCommand`) for semantic control (gait selection, body
  pose). Both feed the walk controller through one arbitration point.
- Add `twist_mux` (ros package) in bringup: inputs `/cmd_vel_teleop` (future), `/cmd_vel_nav`;
  output `/cmd_vel`. Initially joystick stays on its semantic path; mux priorities documented.

## Design

1. **Velocity model**: per gait, fit `v_body = f(stride_length, phase_rate)` using sim
   ground-truth `/odom` (bridge exists) driven via `robot.walk_for_duration` MCP tool or a launch
   script. Store fitted coefficients in `drqp_brain/config/velocity_model.yaml` (per gait:
   max_vx, max_vy, max_wz, linear coefficients). Notebook with the fit goes to
   `docs/source/notebooks/`.
2. **Inverse mapping in brain**: new module `drqp_brain/velocity_mapper.py`
   (`Twist → stride_direction, rotation_speed, phase_rate`), saturating to
   `stride_limits.yaml` and model maxima. Unit-test pure function.
3. **Brain integration**: `/cmd_vel` subscriber in `brain_node.py` (or a small adapter node
   translating Twist → `MovementCommand` + phase-rate extension — prefer adapter node
   `cmd_vel_adapter` to keep brain untouched; extend `MovementCommand` with optional
   `float32 speed_scale` only if required).
4. Zero-velocity and stale input produce the same idle behavior as neutral joystick.

## Acceptance criteria

- [ ] Unit tests: mapper saturation, watchdog timing, zero handling.
- [ ] Launch test (`drqp_gazebo/test/`): publish 0.1 m/s forward 10 s ⇒ ground-truth
      displacement 1.0 m ± 20 %; analogous lateral, yaw (±20 %), combined.
- [ ] `teleop_twist_keyboard` drives the sim robot with no custom code.
- [ ] All existing movement/balance launch tests stay green.
- [ ] Hardware tape-measure results at 3 speeds recorded in docs.

## Constraints

- Do not remove/repurpose `MovementCommand`; do not alter state-machine safety semantics
  (`/cmd_vel` accepted only in `torque_on`).
- Brain loop stays 8 Hz.
