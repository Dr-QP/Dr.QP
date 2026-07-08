# Roadmap: from joystick hexapod to autonomous AI home pet

Dr.QP today is a hexapod that walks nicely under joystick control: three parametric gaits,
MoveIt-validated IK, IMU-assisted balancing, a Gazebo simulation with a solid launch-test suite.
The ultimate goal is much bigger: **an autonomous AI home pet** that navigates the home on its own,
sees with a front-facing camera, communicates with people, and eventually *learns its own
locomotion* with reinforcement learning to exploit its servos and geometry to the fullest.

This section is the map between those two points. It is written for humans; each phase also has a
matching AI-agent spec in
[`docs/agents/specs/roadmap/`](https://github.com/Dr-QP/Dr.QP/tree/main/docs/agents/specs/roadmap)
that turns the same content into precise, testable work items.

## The destination

- **Navigate around the home autonomously** — build a map, know where it is, walk to goals, avoid
  obstacles and pets/people.
- **Communicate** — hear, speak, and hold simple conversations; behave like a pet, not a terminal.
- **Sense** — IMU (done), fixed front-facing Pi camera (next), feet contact sensors (later),
  torque-feedback servos such as Dynamixel XC430-T240BB-T (far future).
- **Learn** — reinforcement-learning locomotion trained in simulation and deployed to hardware,
  replacing (or augmenting) the hand-crafted parametric gaits.

## Phase overview

| # | Phase | Unlocks | Depends on |
|---|-------|---------|------------|
| 1 | {doc}`01-baseline` | Shared understanding, hardening targets | — |
| 2 | {doc}`02-velocity-interface` | Standard `cmd_vel` control, metric motion | 1 |
| 3 | {doc}`03-state-estimation` | `odom → base_link` TF, fused pose | 2 |
| 4 | {doc}`04-camera` | Images on robot and in sim | 1 |
| 5 | {doc}`05-localization-mapping` | "Where am I?" in the home | 3, 4 |
| 6 | {doc}`06-navigation` | "Go to the kitchen" | 5 |
| 7 | {doc}`07-interaction` | Voice, LLM brain, pet personality | 4 (partially parallel) |
| 8 | {doc}`08-feet-terrain` | Ground truth contact, rough terrain | 2 |
| 9 | {doc}`09-rl-locomotion` | Learned gaits, full hardware potential | 2, 8 (sim work parallel) |

Dependency sketch — phases on separate branches can run in parallel:

```text
1 baseline
├── 2 velocity interface ── 3 state estimation ──┐
│                                                ├── 5 SLAM ── 6 Nav2 navigation
├── 4 camera ────────────────────────────────────┘      │
│        └────────────── 7 voice + LLM interaction ─────┴── "AI home pet" milestone
├── 8 feet sensors / terrain
└── 9 RL locomotion (sim first, hardware after 2; benefits from 8)
```

## Principles that shape the plan

1. **Standard interfaces first.** Nav2, SLAM packages, teleop tools, and simulators all speak
   `geometry_msgs/Twist`, `nav_msgs/Odometry`, `sensor_msgs/Image`, and TF. The earlier Dr.QP
   speaks them natively, the more of the ROS ecosystem plugs in for free. The custom
   `MovementCommand` stays as the *semantic* layer (body pose, gait choice) on top.
2. **Simulation is the contract.** Every phase lands with Gazebo coverage and `launch_pytest`
   tests, exactly like the existing `drqp_gazebo` suite. Hardware bring-up follows sim, never
   precedes it.
3. **The Pi is small; the home is networked.** A Raspberry Pi cannot run visual SLAM, an LLM, ASR,
   and gait IK at once. The architecture explicitly allows heavy perception/AI nodes to run on a
   home server over Wi-Fi (DDS makes this transparent), while everything needed for *safety* —
   servo control, balance, fall protection, kill switch — stays on the robot.
4. **Each phase is useful on its own.** No phase requires a later one to justify itself: metric
   velocity control improves teleop, state estimation improves balance, the camera enables
   snapshots and person detection long before SLAM.

## Where the code lives today

| Layer | Package(s) |
|-------|------------|
| Serial + servo protocol | `drqp_serial`, `drqp_a1_16_driver` |
| Hardware interface, URDF, controllers | `drqp_control` (ros2_control + `joint_trajectory_controller`) |
| Kinematics models | `drqp_kinematics`, `drqp_moveit` |
| Gaits, IK orchestration, state machine, IMU | `drqp_brain` |
| Input | `drqp_joy` (SDL3 game controller), `drqp_keyboard_control` |
| Messages | `drqp_interfaces` (`MovementCommand`, `RobotCommand`, `HapticEffect`) |
| Simulation + tests | `drqp_gazebo` |
| Agent tooling | `drqp_robot_mcp` (FastMCP server driving sim and robot) |

```{toctree}
:maxdepth: 1

01-baseline
02-velocity-interface
03-state-estimation
04-camera
05-localization-mapping
06-navigation
07-interaction
08-feet-terrain
09-rl-locomotion
```
