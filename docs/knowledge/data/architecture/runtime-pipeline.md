---
type: architecture
description: The layered teleop runtime — input, semantic command, brain loop, ros2_control, servo bus — and the design decisions visible in the code.
generated:
  by: claude-code/opus-5
  at: 2026-09-25T00:00:00Z
sources:
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
  title: walking loop, lifecycle reactions, balance mode
- id: state
  resource: packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_machine.py
  title: robot lifecycle state machine
- id: roadmap
  resource: docs/source/Dev/roadmap/01-baseline.md
  title: architecture baseline narrative (predates locomotion specs 04–08)
---

# Runtime pipeline

Dr.QP's runtime is a layered teleop pipeline. Each layer is a separate ROS 2
package, and the layers talk only through topics and actions.

``` text
DualSense ─/joy─▶ joystick_translator ─/robot/movement_command─▶ drqp_brain
   (drqp_joy)          │                                         │ WalkController
                       └─/robot_event─▶ drqp_robot_state         │ + ParametricGaitGenerator
                                             │                   │ + analytic IK (MoveIt scene check)
                                             └─/robot_state─────▶│
                                                                  ▼
                        /joint_trajectory_controller/joint_trajectory
                        (+ FollowJointTrajectory action for scripted sequences)
                                                                  ▼
          ros2_control joint_trajectory_controller ─▶ A1-16 hardware interface (drqp_control)
                                                                  ▼
                       drqp_a1_16_driver ─▶ drqp_serial ─▶ UART bus ─▶ 18 × A1-16 servos
```

## Layers

| Layer        | Package                                   | Role                                                                                        |
| ------------ | ----------------------------------------- | ------------------------------------------------------------------------------------------- |
| Input        | `drqp_joy`, `drqp_keyboard_control`       | Gamepad (SDL3 with haptics) or simulation GUI                                               |
| Translation  | `drqp_brain` (`joystick_translator_node`) | `/joy` → `MovementCommand` and `/robot_event`                                               |
| Lifecycle    | `drqp_brain` (`robot_state`)              | Owns the robot state; publishes `/robot_state` (transient-local)                            |
| Locomotion   | `drqp_brain` (`brain_node`)               | Gait, IK, and balance loop at `control_rate_hz` (default 25 Hz)                             |
| Kinematics   | `drqp_kinematics`, `drqp_moveit`          | Geometry models; MoveIt planning scene for self-collision checks                            |
| Hardware     | `drqp_control`                            | ros2_control plugin, URDF, and controller configuration                                     |
| Transport    | `drqp_a1_16_driver`, `drqp_serial`        | Servo protocol over UART or TCP                                                             |
| Simulation   | `drqp_gazebo`                             | gz-sim with the same controllers; bridges `/clock`, `/odom` (ground truth), and `/imu/data` |
| Agent access | `drqp_robot_mcp`                          | MCP tools that drive the simulation and the robot                                           |

## Decisions visible in the code

- **Semantic command layer.** `MovementCommand` carries normalized values (−1…1)
  for stride direction, rotation speed, body translation, and body rotation,
  plus a gait name. Nothing on this path is metric yet. A future `/cmd_vel` path
  (roadmap RM-02) is meant to sit beside this layer, not replace it.
- **The lifecycle is separate from locomotion.** A dedicated `drqp_robot_state`
  node runs a `python-statemachine` graph, driven by string events on
  `/robot_event`. The brain reacts to `/robot_state`: it starts or stops the
  loop timer and plays scripted initialization and finalization trajectories
  through the `FollowJointTrajectory` action. When a sequence finishes, the
  brain publishes a `*_done` event back.
- **Trajectory windows, not per-joint commands.** Each tick publishes a short
  `JointTrajectory` whose points are spaced `1 / control_rate_hz` apart. The
  controller interpolates between points to keep motion smooth. A tick is
  skipped when the motion-state key is unchanged, so a stationary robot does not
  republish.
- **Analytic IK by default.** The `kinematics_backend` parameter defaults to
  `analytic`; `moveit` remains selectable for comparison. MoveIt stays
  in-process so the planning scene can check whole-robot self-collision.
- **One instance per ROS domain.** `domain_instance_guard` refuses to start a
  second `drqp_robot_state` or brain node on the same domain.
- **Simulation is the test contract.** `drqp_gazebo` runs the real controllers,
  and its `launch_pytest` suite covers spawning, postures, movement, rotation,
  and balance. Every behavior change is expected to land with a scenario there.

## Unknown / not yet designed

- Odometry from the robot itself and the `odom → base_link` TF. Today `/odom`
  exists only in simulation, as Gazebo ground truth.
- Where off-board nodes run and how they are deployed. The roadmap states the
  principle, but nothing is implemented.
- Measured servo-bus round-trip rate and Pi CPU baseline (RM-01).
