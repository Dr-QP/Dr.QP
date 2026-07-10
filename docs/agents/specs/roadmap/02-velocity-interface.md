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
  pose). Both feed the walk controller through one arbitration point **in the brain**, and that
  arbiter — not the mux — is what enforces "joystick > autonomy": the joystick stays on its
  semantic path outside `twist_mux`, so the mux cannot see it. Arbitration rule: joystick is
  *active* while a non-neutral `MovementCommand` arrived within a freshness window (0.5 s,
  matching the watchdog); while active, `/cmd_vel` motion input is ignored; when the joystick
  goes stale or neutral, `/cmd_vel` resumes. Document the rule next to the arbiter.
- Add `twist_mux` (ros package) in bringup: inputs `/cmd_vel_teleop` (future), `/cmd_vel_nav`;
  output `/cmd_vel`. The mux arbitrates only among `cmd_vel` sources (nav vs future
  twist-teleop); mux priorities documented.

## Relationship to the locomotion migration (read first)

This spec has **two implementations depending on whether locomotion spec 06 (twist steering) has
landed** — they are not both built:

- **If locomotion 06 is in** (recommended path): velocity is already metric by construction. The
  commanded body twist `ξ = (v_x, v_y, ω_z)` *is* the walk controller's input, so this spec
  reduces to a thin `Twist → ξ` adapter plus `twist_mux`, the watchdog, and the joystick/autonomy
  arbiter below. **Skip the empirical velocity-model fit and `velocity_mapper.py`.** Keep the
  ground-truth displacement test as an acceptance cross-check on spec 06's metric claim.
- **If locomotion 06 is not yet in** (interim path): build the empirical mapper in the "Design"
  section below so `cmd_vel` works on today's position-mixed steering, and treat it as throwaway
  scaffolding retired when spec 06 lands.

See [Program relationships](../README.md#program-relationships).

## Design (interim path — only if locomotion 06 has not landed)

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
- [ ] Simultaneous-input arbitration test: joystick activity while `/cmd_vel` streams ⇒
      joystick wins within one control cycle; joystick released/neutral ⇒ `/cmd_vel` resumes
      after the freshness window (node-level or launch test).
- [ ] Launch test (`drqp_gazebo/test/`): publish 0.1 m/s forward 10 s ⇒ ground-truth
      displacement 1.0 m ± 20 %; analogous lateral, yaw (±20 %), combined.
- [ ] `teleop_twist_keyboard` drives the sim robot with no custom code.
- [ ] All existing movement/balance launch tests stay green.
- [ ] Hardware tape-measure results at 3 speeds recorded in docs.

## Constraints

- Do not remove/repurpose `MovementCommand`; do not alter state-machine safety semantics
  (`/cmd_vel` accepted only in `torque_on`).
- Brain loop stays 8 Hz.
