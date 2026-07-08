---
id: RM-09
title: Reinforcement-learned locomotion
status: proposed
depends_on: [RM-02]   # observation upgrades from RM-08; policy rate gated by RM-01 bus benchmark
packages: [drqp_control, drqp_gazebo, "NEW: drqp_rl (training, non-colcon ok)", "NEW: drqp_rl_runtime"]
---

# RM-09 — RL locomotion

## Objective

Velocity-conditioned learned gait: policy trained in a massively parallel simulator, validated in
Gazebo with the existing test suite, deployed as a runtime mode (`parametric | learned`) behind
the same `/cmd_vel` interface, with a hard safety supervisor.

## Platform constraints (fixed inputs to design)

- A1-16 servos: **position control only**; policy action = 18 joint position targets tracked by
  servo-internal PD. No torque interface. Servo temperature IS readable — monitor it.
- Policy rate 25–50 Hz gated by RM-01 bus benchmark; brain's 8 Hz MoveItPy path is bypassed
  entirely by the runtime.
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
   `/robot_event` integration; transitions only through a neutral stance.
5. **Safety supervisor** (inside runtime): trip conditions — tilt > limit, joint target jump >
   limit, obs staleness, servo temp > limit, watchdog. Trip ⇒ freeze to neutral stance /
   handover to parametric stack / `torque_off` per severity. Non-bypassable.
6. **Hardware rollout**: tethered/harnessed → free walking on hard floor → surfaces matrix;
   benchmark vs parametric (velocity tracking error, disturbance recovery, threshold crossing).

## Acceptance criteria

- [ ] Actuator model documented; sim step response matches bench within stated tolerance.
- [ ] Trained policy: tracking error < 15 % across command envelope in trainer eval.
- [ ] Gazebo gate: existing movement + balance-board launch tests pass in `learned` mode.
- [ ] Mode-switch launch test: parametric ↔ learned during `torque_on` via neutral stance, no
      fall (IMU tilt bounded).
- [ ] Supervisor unit/launch tests: each trip condition fires and degrades as specified.
- [ ] Hardware benchmark table committed (learned vs parametric).

## Constraints

- Training code may live outside colcon (`drqp_rl/` with own `pyproject.toml`), but conversion
  and runtime are CI-covered.
- No RL code in the safety path except the supervisor (which is deterministic, not learned).
- Gazebo remains validation-only; never train in it.
