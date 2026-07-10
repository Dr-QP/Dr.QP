# Phase 9 — Reinforcement-learned locomotion

The extended goal: stop telling the robot _how_ to walk and let it learn gaits that exploit its
actual geometry, servo dynamics, and terrain — smoother, faster, more robust than the hand-tuned
parametric gaits, including skills the current stack cannot express at all (fall recovery,
stepping over obstacles, expressive pet motion).

This phase is a long arc that runs **mostly in simulation, in parallel** with Phases 5–8. Its
hardware deployment quality is capped by two things measured earlier: the **servo bus rate**
(Phase 1) and **foot contact sensing** (Phase 8).

## Ground truth about the platform (design constraints)

- **A1-16 servos are position-controlled.** No torque interface, no current feedback. The policy
  therefore outputs **joint position targets** tracked by the servos' internal PD — exactly the
  action space used by most successful sim-to-real quadruped work, so this is a constraint, not a
  blocker. Torque-feedback servos (XC430-T240BB-T class) later improve _observations_ and enable
  compliance, but are not prerequisites.
- **Control rate**: learned policies typically run at 25–50 Hz. The current brain publishes at
  8 Hz through MoveItPy IK — the policy path **bypasses the brain**: a dedicated runtime node
  feeds position targets straight to the ros2_control layer. Whether 50 Hz is reachable depends
  on the measured A1-16 bus round-trip (Phase 1); the fallback is 25 Hz with action interpolation
  in the hardware interface.
- **Observations available**: joint positions (bus read-back), IMU orientation + angular velocity
  - linear acceleration, previous actions, commanded velocity; foot contacts after Phase 8.
    No joint velocities from hardware → estimate by filtering positions (and randomize that
    filter in training).

## Pipeline

```text
URDF (drqp_control) ─▶ training model (MJCF/USD) ─▶ massively parallel sim training
                          │                            (Isaac Lab or MuJoCo MJX; PPO)
 system ID of A1-16 ──────┘                                     │
 (step-response logs → PD gains, backlash, latency)             ▼
                                                    policy.onnx (velocity-conditioned)
                                                                │
 Gazebo validation gate (same worlds/tests as CI) ◀─────────────┤
                                                                ▼
                              drqp_rl_runtime node: /cmd_vel + obs → 25–50 Hz joint targets
                                        (same Twist interface as Phase 2 — Nav2 never knows)
```

1. **Model conversion + system ID.** Export the URDF to the trainer's format; identify the servo
   model by logging commanded-vs-actual position step responses on a bench leg. Actuator fidelity
   is _the_ sim-to-real lever for position-controlled robots.
2. **Training environment.** Isaac Lab (GPU, mature legged-robot tasks) or MuJoCo MJX are the
   sane choices in 2026; Gazebo is far too slow for training and stays as the _validation_ gate.
   Task: velocity-command tracking (`v_x, v_y, ω_z` — deliberately identical to the Phase 2
   interface) with standard rewards (tracking, upright, energy, foot-slip, action smoothness)
   and heavy domain randomization (mass, friction, latency 10–40 ms, PD gains, IMU noise).
3. **Curriculum**: stand → walk on flat → omnidirectional tracking → push recovery → rough
   terrain (thresholds/rugs from Phase 8's test worlds) → fall recovery as a separate policy.
4. **Deployment runtime**: minimal C++/Python node loading ONNX, subscribing to the same
   `/cmd_vel` and safety topics, with a **supervisor**: tilt/joint-limit/watchdog trips → freeze
   into the classical stack or `torque_off`. Gait selection becomes a mode: `parametric` |
   `learned`, switchable at runtime and by the state machine.
5. **Evaluation**: the existing `drqp_gazebo` movement/balance launch tests run identically
   against the learned mode — the parametric gait is the baseline to beat (velocity accuracy,
   disturbance rejection on the balance board, threshold crossing).

## Why this can work on this robot (and what to watch)

Hobby-grade sim-to-real hexapods with position-controlled smart servos are well within published
results — hexapods are statically stable, which makes the task _easier_ than quadrupeds. The
honest risks:

- **Bus rate**: if the A1-16 bus cannot close 25 Hz with read-back, observations degrade
  (open-loop joint state). Mitigations: reduce read set (IMU-heavy observations), or this becomes
  the concrete justification for the XC430 upgrade.
- **Sim-to-real gap in the feet**: rubber tips on hard floor vs sim friction — randomize wide,
  validate on multiple real surfaces early with a _conservative_ policy before chasing speed.
- **Servo wear/heat**: learned policies love high-frequency dither; penalize action rate hard and
  monitor servo temperature (the A1-16 does report it) in the runtime supervisor.
- Keep expectations staged: matching the parametric gait robustly is the first win; exceeding it
  and adding fall recovery is the second; expressive learned pet motion is the stretch.

## Milestones

1. Bench system ID of one leg; documented actuator model.
2. Training env with velocity-tracking task; policy walks in the trainer.
3. Gazebo validation gate green (spawn, forward/lateral/rotate tests in `learned` mode).
4. Runtime node + supervisor + mode switch; CI covers mode switching safety.
5. Hardware: tethered first steps → free walking → benchmark vs parametric gaits.
6. Push/fall recovery policy; (stretch) terrain curriculum with Phase 8 contacts in observations.

## Definition of done

- `ros2 param set … gait_mode learned` and the robot walks the same Nav2 goals as Phase 6, at
  least matching parametric-gait reliability, with the supervisor demonstrably catching induced
  failures.

This closes the roadmap: an autonomous, communicating, self-taught hexapod pet — with every layer
below it still testable, swappable, and understandable.
