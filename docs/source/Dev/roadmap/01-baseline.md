# Phase 1 — Architecture baseline and hardening

Before adding autonomy, it pays to know exactly what exists, and to close the handful of gaps that
every later phase would otherwise trip over. This guide is the "you are here" marker.

## What Dr.QP does today

The runtime is a classic layered teleop pipeline:

```text
DualSense pad ──/joy──▶ joystick_translator ──/robot/movement_command──▶ drqp_brain
                                │                                           │
                                └──/robot_event──▶ robot_state node        │ WalkController
                                        │        (python-statemachine)     │ + ParametricGaitGenerator
                                        └──/robot_state──────────────▶     │ + MoveItPy IK validation
                                                                            ▼
                              /joint_trajectory_controller/joint_trajectory (8 Hz window)
                                                                            ▼
                    ros2_control: joint_trajectory_controller ──▶ A1-16 hardware interface
                                                                            ▼
                                                 UART bus ──▶ 18 × XYZrobot A1-16 servos
```

Key facts, verified in code:

- **Control loop**: `drqp_brain` runs its walking loop at **8 Hz** (`brain_node.py`, `self.fps = 8`)
  and publishes 2-point trajectory windows; smoothness comes from the trajectory controller
  interpolating between points.
- **Semantic command layer**: `MovementCommand` carries normalized (−1…1) stride direction,
  rotation speed, body translation/rotation, and gait name. Nothing in the pipeline is metric
  (m/s, rad/s) yet.
- **Gaits**: tripod, ripple, wave — generated parametrically per leg phase, with directional
  stride limits loaded from `drqp_brain/config/stride_limits.yaml`.
- **IK**: `MoveItPyLocomotionKinematics` runs MoveItPy *in-process* for IK and whole-robot state
  validation (collision-aware). Seeded from live `/joint_states`.
- **Balance**: `/imu/data` (BNO055 on hardware, gz-sim IMU plugin in simulation) feeds a balance
  controller that scales stride and counter-rotates the body; toggled via `/robot/balance_mode`.
- **Lifecycle**: a `python-statemachine` graph (`torque_off → initializing → torque_on →
  finalizing → finalized`, plus `servos_rebooting`) driven by `/robot_event`, with scripted
  init/finalize trajectories.
- **Simulation**: `drqp_gazebo` spawns the URDF in gz-sim, bridges `/clock`, `/odom`
  (ground-truth model odometry), and `/imu/data`; the launch-test suite covers spawn, postures,
  movement in every direction, rotation, and balance-board scenarios.
- **Agent access**: `drqp_robot_mcp` already exposes `simulation.*` and `robot.*` MCP tools
  (boot, move, walk_for_duration, recordings) — an important seed for the later LLM phase.

## What is *not* there yet

These gaps define the roadmap:

| Gap | Blocks |
|-----|--------|
| No `geometry_msgs/Twist` interface; commands are normalized, not metric | Nav2, standard teleop, RL action bridging |
| No odometry from the robot itself (`/odom` in sim is Gazebo ground truth) | SLAM, navigation, drift-aware balance |
| No `odom → base_link` TF; TF tree is `robot_state_publisher` only | Everything nav |
| Camera is an empty URDF link — no driver, no sim sensor, no bridge | Vision, SLAM, person detection |
| No microphone/speaker hardware or audio stack | Voice interaction |
| No feet contact sensing (and A1-16 servos expose no torque feedback) | Terrain adaptation, contact-aware RL |
| 8 Hz brain loop and MoveItPy-in-the-loop IK | High-rate RL policies (they will bypass this path) |

## Hardening tasks worth doing now

1. **Document the TF and topic contract.** Write down (and test) the canonical frame names
   (`drqp/base_link`, `drqp/imu_link`, `drqp/camera`) and the QoS of every public topic. Later
   phases add frames; a contract prevents breakage.
2. **Measure the servo bus.** Record the achievable round-trip rate of the A1-16 UART bus
   (positions written + read per cycle). This number decides the maximum RL policy rate in
   Phase 9 and the leg-odometry quality in Phase 3. Store the result in the docs.
3. **Battery/power telemetry.** The `ros2_control` config already sketches a `battery_state`
   sensor; finishing it gives autonomy a "go home and charge" signal later and protects packs now.
4. **CPU headroom audit.** Capture baseline CPU/memory of the full bringup on the Pi. Each later
   phase must state its budget against this baseline.

## Definition of done

- An architecture page (this one) reviewed and linked from the docs index.
- Frame/topic contract documented and enforced by a launch test.
- Servo bus rate and Pi resource baseline measured and recorded.
- Battery telemetry published on hardware bringup.

Next: {doc}`02-velocity-interface`.
