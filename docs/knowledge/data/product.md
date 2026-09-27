---
type: tracker
description: What the product is, who it is for, and the decisions every plan and spec derives from.
stage: living
generated:
  by: claude-code/opus-5
  at: 2026-09-25T12:00:00Z
sources:
- id: readme
  resource: README.md
- id: roadmap
  resource: docs/source/Dev/roadmap/index.md
- id: specs
  resource: docs/agents/specs/README.md
- id: interview
  resource: maintainer interview 2026-09-25
---

# Product

## What is it

Dr.QP is an open-source ROS 2 hexapod robot on its way from joystick-driven
walker to autonomous AI home pet.

Today Dr.QP walks under DualSense game-controller control. It uses three
parametric gaits (tripod, ripple, wave), closed-form per-leg inverse kinematics
(IK) with whole-robot self-collision validation in MoveIt, and an IMU balance
mode that holds a stationary body posture. A Gazebo (gz-sim) simulation is
covered by a `launch_pytest` suite. The `drqp_robot_mcp` MCP server lets AI
agents boot, drive and record both the simulation and the robot.

The destination is an autonomous home pet that maps and navigates the home, sees
through a front-facing camera, talks with people, and eventually learns its own
locomotion with reinforcement learning. The path there is the nine-phase roadmap
in `docs/source/Dev/roadmap/`, with agent-ready specs in
`docs/agents/specs/roadmap/`.

**Stage.** The locomotion foundation program (`docs/agents/specs/locomotion/`,
specs 01–08) has landed: an analytic IK hot path, twist steering, time-based
gaits, and a `control_rate_hz` loop parameter that defaults to 25 Hz. None of
the autonomy phases RM-02…RM-09 is implemented yet; for example, there is no
`/cmd_vel` subscriber. The work in progress is the balance-mode-safety program
(`docs/agents/specs/balance-mode-safety/`). It makes balance a bounded,
stationary posture mode and stabilizes its Gazebo CI tests.

## Users

- **The maintainer, as a personal maker project.** Success means a robot that
  walks well on real hardware and keeps moving up the roadmap.
- **Other hexapod builders** who want to reproduce or fork the robot, or reuse
  its packages (servo driver, ros2_control plugin, kinematics, launch-test
  helpers). Success means the build and install docs work and the packages are
  reusable.
- **Learners and researchers** who use it as a testbed for ROS 2, legged-robot
  IK and gaits, reinforcement learning, and agent-driven development. The
  Jupyter notebooks in `docs/source/notebooks/` and the MCP server serve this
  group.

**Not for:**

- Consumers who want a finished product. There is no kit, no support, and no
  turnkey pet; "home pet" is the long-term direction, not a promise.
- Developers on non-Linux hosts outside the devcontainer. Only Linux and the
  devcontainer are supported for development.

## Platforms

- **Robot:** a Raspberry Pi 5 running Ubuntu 24.04 (Noble) and ROS 2 Jazzy. It
  drives 18 × XYZrobot A1-16 servos over a UART bus through an SC16IS752
  I2C–UART HAT, with a BNO055 IMU. Deployment uses the
  `ghcr.io/dr-qp/jazzy-ros-deploy` image; the Pi setup is automated with Ansible
  (`docker/ros/ansible`).
- **Development:** the devcontainer (`ghcr.io/dr-qp/jazzy-ros-desktop`) on any
  desktop OS that runs Docker, or a native Ubuntu 24.04 host. Linux is the only
  tested host OS.
- **Simulation:** Gazebo (gz-sim) through `drqp_gazebo`.
- **Docs:** Sphinx, published on Read the Docs (drqp.readthedocs.io).
- **Distribution:** source only, from GitHub (Dr-QP/Dr.QP). Nothing is published
  to package registries.

## Stack

- **Build:** a colcon workspace of ROS 2 Jazzy packages. Always run commands
  through `scripts/with-ros-env.sh`, e.g.
  `scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to <pkg>`.
  Python tooling lives in the `uv`-managed `.venv` (`pyproject.toml`).
- **C++ (ament_cmake, C++17+):**
  - `drqp_serial`: UART and TCP transport.
  - `drqp_a1_16_driver`: A1-16 servo protocol.
  - `drqp_control`: the ros2_control hardware interface, URDF, and controllers.
  - `drqp_joy`: the SDL3 game-controller node, with haptics.
  - `drqp_interfaces`: messages (`MovementCommand`, `RobotCommand`,
    `HapticEffect`).
  - `drqp_moveit`: the MoveIt 2 configuration.
  - `drqp_gazebo`: simulation launch files, worlds, and launch tests.
- **Python (ament_python):**
  - `drqp_brain`: the walking loop, gaits, IK orchestration, balance, the
    `robot_state` state machine, and the joystick translator.
  - `drqp_kinematics`: hexapod geometry and kinematics models.
  - `drqp_keyboard_control`: the simulation GUI controller.
  - `drqp_robot_mcp`: the FastMCP server.
  - `drqp_launch_testing`: `launch_pytest` helpers.
- **Layout:**
  - `packages/runtime/` and `packages/simulation/` hold the workspace packages.
  - `packages/vendor/` holds the vendored `launch` (with a patched
    `launch_pytest`) and `sdl3_vendor`.
  - `docs/source/` holds the human docs and notebooks; `docs/agents/specs/`
    holds the agent specs; `docs/knowledge/` is this workspace.
  - `docker/` and `hardware/` hold images and Pi setup.
- **Tests:** pytest (never unittest), `launch_pytest` for node and launch
  integration, and Catch2 (GMock when mocks are needed) for C++. Run them with
  `scripts/with-ros-env.sh colcon test --packages-select <pkg>`; logs go to
  `log/latest_test/<pkg>/`.
- **Lint and format:** run ruff first, then `ament_flake8`, which is the CI gate
  (`scripts/python-lint-check.sh`). C++ uses clang-format. Super-Linter runs in
  CI. Codecov tracks coverage.

## Constraints

- **Simulation first, then hardware.** Every change to runtime behavior lands
  with Gazebo coverage and `launch_pytest` tests. Hardware bring-up follows the
  simulation and never comes first.
- **Safety stays on the robot.** Servo control, balance, fall protection, the
  kill switch, and the robot state machine run on the Pi. Heavy perception and
  AI nodes (SLAM, LLM, ASR) may run on a home server over Wi-Fi through DDS.
  They must never be required for the robot to stop safely.
- **Pi CPU budget.** Each phase or plan that adds on-robot work states its CPU
  and memory budget against the measured Pi baseline. That baseline is still to
  be captured in RM-01.
- **MIT license, standard ROS interfaces.** The project is MIT-licensed; new
  dependencies must have compatible licenses. Prefer standard ROS messages and
  conventions (`geometry_msgs/Twist`, `nav_msgs/Odometry`, `sensor_msgs/*`, TF)
  over custom ones. `MovementCommand` stays as the semantic layer on top.

## Authoring rules

- **Code anchors:** verify every file path, line, parameter name, and default
  value against the current checkout. Never cite them from memory or from
  narrative docs, which drift over time.
- **Plans that add nodes or topics:** state the CPU and memory budget, where the
  node runs (on the robot or off-board), the QoS of each topic, and any TF
  frames added or consumed.
- **Plan tasks that touch code:** name the unit, integration, or simulation
  (`launch_pytest`) test that is written first, following TDD.

## Changelog

- 2026-08-01 — document created.
- 2026-09-25 — filled all sections from the codebase and a maintainer interview
  (`/agentdev:iwe-setup`).
- 2026-09-27 — reduced the code-anchor rule to the rule itself.
