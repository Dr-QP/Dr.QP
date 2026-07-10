# Roadmap specs (AI-agent oriented)

Machine-oriented work specifications for evolving Dr.QP from a joystick-controlled hexapod into an
autonomous AI home pet. Human-narrative counterparts live in `docs/source/Dev/roadmap/` (same
slugs; RM-10/RM-11 human guides are TODO); read the guide for rationale, use the spec for
execution.

> **Foundation:** this roadmap was written against today's walking stack, which the
> [locomotion migration](../locomotion/README.md) replaces. Read
> [Program relationships](../README.md#program-relationships) before RM-02/03/08/09 — the
> migration changes the velocity mapping (RM-02), the loop rate, and the IK hot path those phases
> assume.

## Conventions

- Each spec has YAML frontmatter: `id`, `title`, `status` (`proposed | in-progress | done`),
  `depends_on` (spec ids), `packages` (existing packages likely touched; `NEW:` prefix = create).
- Acceptance criteria are testable; prefer `launch_pytest` tests under the owning package's
  `test/` dir, following `.github/instructions/launch-testing.instructions.md`.
- Build/test via `scripts/with-ros-env.sh colcon build --packages-up-to <pkg>` and
  `colcon test --packages-select <pkg>`; see `AGENTS.md`.
- Simulation-first: every capability lands in `drqp_gazebo` (worlds/bridge/tests) before hardware.
- Frames: `odom → drqp/base_link` owned by EKF (spec 03); `map → odom` owned by localizer
  (spec 05). Robot links are prefixed `drqp/` per URDF (`packages/runtime/drqp_control/urdf/`).
- Do not put heavy perception/LLM nodes in the safety path. Safety-critical topics
  (`/robot_event`, `/robot_state`, joint trajectories, kill switch) keep their current semantics.

## Specs

| id    | file                                                     | depends_on                   |
| ----- | -------------------------------------------------------- | ---------------------------- |
| RM-01 | [01-baseline.md](01-baseline.md)                         | —                            |
| RM-02 | [02-velocity-interface.md](02-velocity-interface.md)     | RM-01                        |
| RM-03 | [03-state-estimation.md](03-state-estimation.md)         | RM-02                        |
| RM-04 | [04-camera.md](04-camera.md)                             | RM-01                        |
| RM-05 | [05-localization-mapping.md](05-localization-mapping.md) | RM-03, RM-04                 |
| RM-06 | [06-navigation.md](06-navigation.md)                     | RM-05                        |
| RM-07 | [07-interaction.md](07-interaction.md)                   | RM-04 (nav tools need RM-06) |
| RM-08 | [08-feet-terrain.md](08-feet-terrain.md)                 | RM-02                        |
| RM-09 | [09-rl-locomotion.md](09-rl-locomotion.md)               | RM-02 (obs upgrades: RM-08)  |
| RM-10 | [10-obstacle-perception.md](10-obstacle-perception.md)   | RM-04, RM-06                 |
| RM-11 | [11-docking-charging.md](11-docking-charging.md)         | RM-01, RM-05, RM-06          |

Parallelizable branches: {RM-02→03→05→06→(10, 11)}, {RM-04}, {RM-07 voice/LLM core}, {RM-08},
{RM-09 sim work}. RM-10 (live obstacle source) unblocks RM-06's **hardware** navigation milestone;
RM-11 (self-charging) closes the unattended-autonomy loop and is the capstone of the nav branch.

## Current-architecture anchors (verified 2026-07)

- Walking pipeline: `packages/runtime/drqp_brain/drqp_brain/brain_node.py` (8 Hz loop,
  `WalkController`, MoveItPy IK via `locomotion_kinematics.py`, publishes
  `/joint_trajectory_controller/joint_trajectory`).
- Semantic commands: `drqp_interfaces/msg/MovementCommand.msg` (normalized −1..1, gait name)
  published by `joystick_translator_node.py` on `/robot/movement_command`.
- State machine: `drqp_brain/robot_state/robot_state_machine.py`
  (`torque_off/initializing/torque_on/finalizing/finalized/servos_rebooting`), events on
  `/robot_event`, state on `/robot_state` (latched).
- IMU: `drqp_brain/imu_node.py` (BNO055, `/imu/data`); balance in `balance_controller.py`.
- Sim: `packages/simulation/drqp_gazebo` — `launch/sim.launch.py`, bridge
  `config/drqp_gazebo_bridge.yml` (`/clock`, `/odom` ground truth, `/imu/data`), tests in
  `test/`.
- Hardware chain: `drqp_serial` → `drqp_a1_16_driver` → `drqp_control`
  (`a1_16_hardware_interface.cpp`, ros2_control, `joint_trajectory_controller`).
- MCP tools: `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py`
  (`simulation.*`, `robot.*` incl. `robot.move`, `robot.walk_for_duration`, recordings).
- Camera: URDF link `drqp/camera` exists (`body.urdf.xacro`), **no** driver/sim sensor/bridge.
- No `cmd_vel`, no robot-generated odometry, no `map`/`odom` frames today.
