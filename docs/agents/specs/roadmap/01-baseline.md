---
id: RM-01
title: Architecture baseline and hardening
status: proposed
depends_on: []
packages: [drqp_control, drqp_brain, drqp_a1_16_driver, drqp_gazebo]
---

# RM-01 — Baseline and hardening

## Objective

Freeze and document the current architecture contract (frames, topics, QoS), measure hardware
limits that later specs depend on, and finish battery telemetry.

## Tasks

1. **Frame/topic contract doc + test**
   - Document canonical frames (`drqp/base_link`, `drqp/base_center_link`, `drqp/imu_link`,
     `drqp/camera`, per-leg links) and all public topics with types and QoS in
     `docs/source/Dev/roadmap/01-baseline.md` (table) or a referenced page.
   - Add launch test in `drqp_gazebo/test/` asserting after sim bringup: TF resolves
     `drqp/base_link → drqp/imu_link` and `→ drqp/camera`; topics `/robot_state` (latched),
     `/imu/data`, `/joint_states`, `/joint_trajectory_controller/joint_trajectory` exist with
     expected types.
2. **Servo bus benchmark** (hardware; script + doc, no CI)
   - Script in `scripts/` measuring A1-16 UART round-trip: N position writes + position/status
     reads per cycle across all 18 servos; report achievable Hz for read-set sizes
     {pos-only, pos+temp+volt}.
   - Record results in docs; this number gates RM-09 policy rate and RM-03 odometry quality.
3. **Battery telemetry**
   - `ros2_control.urdf.xacro` already declares a `battery_state` sensor; finish plumbing in
     `a1_16_hardware_interface.cpp` (A1-16 reports voltage) → publish `sensor_msgs/BatteryState`
     on `/battery_state` via a broadcaster.
   - Sim: publish a static/plausible battery state so consumers are testable.
4. **Resource baseline**
   - Capture CPU/mem of full bringup on Pi (script + one-time doc table). Later specs budget
     against it.

## Acceptance criteria

- [ ] Contract launch test green in CI.
- [ ] `/battery_state` published in sim and on hardware bringup.
- [ ] Bus-rate and resource numbers committed to docs.

## Out of scope

Any behavioral change to gaits, state machine, or teleop.
