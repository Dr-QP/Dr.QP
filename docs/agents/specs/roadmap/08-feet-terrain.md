---
id: RM-08
title: Foot contact sensing and terrain adaptation
status: proposed
depends_on: [RM-02]
packages: [drqp_interfaces, drqp_brain, drqp_control, drqp_gazebo]
---

# RM-08 — Feet and terrain

## Objective

Source-agnostic foot-contact interface, implemented first by gz-sim contact sensors and hardware
FSR/switch feet, consumed by leg odometry, touchdown adaptation, and a stability reflex. Designed
so future torque-feedback servos (XC430-T240BB-T class) replace the source without consumer
changes.

## Interfaces

- NEW msg `drqp_interfaces/FootContacts`: `std_msgs/Header header`; per leg (fixed order matching
  `HexapodModel.legs`): `bool[6] in_contact`, `float32[6] force_estimate` (N, NaN if unknown),
  `float32[6] confidence`. Topic `/feet/contacts`, ≥ gait tick rate.
- Sim source: gz-sim `contact` sensors on tibia-tip collision in `gazebo.xacro`; NEW node or
  bridge mapping gz contacts → `FootContacts`.
- Hardware source: foot switch/FSR per leg → MCU (RP2040/ADC) → serial/I2C → NEW node
  `feet_sensor_node` (`drqp_control` or new pkg) publishing `FootContacts` with debounce +
  per-foot calibration YAML.
- Future source: joint-load estimator from servo current (out of scope; interface must not
  assume binary-only).

## Consumers

1. **Leg odometry v2** (`drqp_brain`): parameter `stance_source: gait_phase | contacts`;
   contacts mode excludes non-contact feet and slip-flagged feet.
2. **Touchdown adaptation** (`WalkController`): parameter-gated; end swing early on contact,
   extend search (lower foot up to `max_probe_depth`) when contact missing at expected touchdown.
3. **Stability supervisor** (`drqp_brain`): stance polygon + IMU tilt margin → below threshold ⇒
   freeze stride + lower body; publishes `/robot_event` diagnostic. Fast path, no external deps.

## Sim assets

Terrain test world: flat + 1 cm threshold strip + compliant patch (low-stiffness surface) in
`drqp_gazebo/worlds/`.

## Acceptance criteria

- [ ] Sim launch test: walking on flat ground, contact pattern matches gait stance phases
      (tripod: 3/3 alternation) within timing tolerance.
- [ ] Odometry drift benchmark (RM-03 test) re-run with `stance_source: contacts`; result ≤
      gait-phase baseline (record both).
- [ ] Threshold world test: robot crosses threshold + compliant patch without tipping
      (IMU tilt < limit) with touchdown adaptation on; fails/struggles with it off (documented).
- [ ] Stability supervisor test on existing balance-board world: induced tilt ⇒ reflex fires.
- [ ] Consumers degrade to `gait_phase` mode automatically when `/feet/contacts` stale (test).
- [ ] Hardware: one-leg prototype validated (contact latency, debounce), then six-leg; rug drift
      test recorded.

## Constraints

- All consumers must run unchanged with either source; no consumer subscribes to raw sensor
  topics.
- Touchdown adaptation must preserve existing tests when disabled (default off until sim
  acceptance passes).
