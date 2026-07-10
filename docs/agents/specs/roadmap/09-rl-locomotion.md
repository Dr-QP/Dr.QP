---
id: RM-09
title: Reinforcement-learned locomotion
status: proposed
depends_on: [RM-02] # observation upgrades from RM-08; policy rate gated by RM-01 bus benchmark
packages:
  [
    drqp_control,
    drqp_gazebo,
    'NEW: drqp_rl (training, non-colcon ok)',
    'NEW: drqp_rl_runtime',
  ]
---

# RM-09 — RL locomotion

## Objective

Velocity-conditioned learned gait: policy trained in a massively parallel simulator, validated in
Gazebo with the existing test suite, deployed as a runtime mode (`parametric | learned`) behind
the same `/cmd_vel` interface, with a hard safety supervisor.

## Platform constraints (fixed inputs to design)

- A1-16 servos: **position control only**; policy action = 18 joint position targets tracked by
  servo-internal PD. No torque interface. Servo temperature IS readable at the register level,
  but ros2_control does not currently expose it — exposing per-servo temperature (state
  interfaces in `a1_16_hardware_interface.cpp` + a broadcaster/diagnostics publisher, e.g.
  `/servo_temps`) is a deliverable of stage 4 and a prerequisite for the stage 5 temperature
  trip.
- Policy rate 25–50 Hz gated by RM-01 bus benchmark; the parametric brain (whichever loop rate
  and solver it runs — 8 Hz MoveItPy today, analytic `control_rate_hz` after the locomotion
  migration) is bypassed entirely by the RL runtime, which drives the controllers directly.
- Observations v1: joint positions (read-back), IMU orientation (gravity vector) + angular
  velocity, previous action, commanded `(vx, vy, wz)`. Joint velocities: filtered numeric
  derivative (randomize filter in training). v2 (RM-08): foot contact booleans.

## Stages

1. **System ID** (hardware bench, one leg): log commanded-vs-actual position step/chirp
   responses per joint type; fit actuator model (PD gains, velocity/torque limits, backlash,
   command latency). Deliver `docs` note + model params file in `drqp_rl`.
2. **Training env** (`drqp_rl`, Python, uv-managed, not a colcon package): URDF
   (`drqp_control/urdf/`) → MJCF (MuJoCo MJX) or USD (Isaac Lab) conversion script; velocity
   tracking task; PPO; rewards: cmd tracking, upright, height, action-rate penalty, energy,
   foot-slip, air-time regularity; domain randomization: mass ±20 %, friction 0.4–1.25, latency
   10–40 ms, PD gains ±20 %, IMU noise/bias, push perturbations. Curriculum: stand → flat omni
   tracking → pushes → rough terrain (RM-08 worlds' geometry) → separate fall-recovery policy.
3. **Export + Gazebo gate**: ONNX export; run policy in gz-sim via `drqp_rl_runtime`; pass the
   same movement launch tests as parametric mode (forward/backward/lateral/rotate/sustained).
4. **Runtime** (`drqp_rl_runtime`, colcon package): node loading ONNX (onnxruntime), subscribes
   `/cmd_vel`, `/imu/data`, `/joint_states` (+ `/feet/contacts` when available), publishes joint
   targets (dedicated high-rate controller chain — direct `forward_position_controller` or
   trajectory points at policy rate; decide against bus benchmark). Mode switch: parameter +
   `/robot_event` integration; transitions only through a neutral stance. Also delivers the
   servo-temperature exposure (hardware-interface state interfaces + `/servo_temps`
   broadcaster/diagnostics) required by the stage 5 supervisor.
5. **Safety supervisor** (separate node in its own process — NOT inside the policy runtime,
   otherwise a policy-process crash kills the supervisor with it): subscribes `/imu/data`,
   `/joint_states`, the policy's published joint targets, and `/servo_temps` (stage 4). Trip
   conditions — tilt > limit, joint target jump > limit, obs staleness, servo temp > limit,
   policy-process heartbeat/watchdog timeout (this is what catches a crashed or hung policy
   node). Trip ⇒ freeze to neutral stance / handover to parametric stack / `torque_off` per
   severity. Non-bypassable: the policy has no path to the actuators that the supervisor does
   not monitor and cannot override.
6. **Hardware rollout**: tethered/harnessed → free walking on hard floor → surfaces matrix;
   benchmark vs parametric (velocity tracking error, disturbance recovery, threshold crossing).

## Acceptance criteria

- [ ] Actuator model documented; sim step response matches bench within stated tolerance.
- [ ] Trained policy: tracking error < 15 % across command envelope in trainer eval.
- [ ] Gazebo gate: existing movement + balance-board launch tests pass in `learned` mode.
- [ ] Mode-switch launch test: parametric ↔ learned during `torque_on` via neutral stance, no
      fall (IMU tilt bounded).
- [ ] Supervisor unit/launch tests: each trip condition fires and degrades as specified,
      including a killed/hung policy process (supervisor survives and trips the watchdog).
- [ ] Hardware benchmark table committed (learned vs parametric).

## Constraints

- Training code may live outside colcon (`drqp_rl/` with own `pyproject.toml`), but conversion
  and runtime are CI-covered.
- No RL code in the safety path; the supervisor is deterministic (not learned) and runs
  isolated in its own process.
- Gazebo remains validation-only; never train in it.
