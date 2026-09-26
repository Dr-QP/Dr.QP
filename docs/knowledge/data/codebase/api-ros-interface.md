---
type: codebase
description: The ROS topics and actions the robot stack exposes to other nodes — semantic commands, lifecycle events/state, balance toggle, IMU, joystick, and the trajectory controller.
source:
- packages/runtime/drqp_brain
- packages/runtime/drqp_joy
- packages/runtime/drqp_control/config
source_digest: sha256:11c5a25755cc7a5b18a57544939697d80cde21113877e3e2b3e67183556f5c29
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_brain
---

# api-ros-interface

The ROS graph surface that an external node (a new input source, a planner, or a
test) can rely on. Everything is in the global namespace.

## Topics

| Topic                                           | Type                              | QoS                      | Publisher → subscriber                                  |
| ----------------------------------------------- | --------------------------------- | ------------------------ | ------------------------------------------------------- |
| `/robot/movement_command`                       | `drqp_interfaces/MovementCommand` | depth 10                 | translator, keyboard GUI, MCP → `drqp_brain`            |
| `/robot_event`                                  | `std_msgs/String`                 | depth 10                 | translator, GUI, MCP, `drqp_brain` → `drqp_robot_state` |
| `/robot_state`                                  | `std_msgs/String`                 | depth 1, transient-local | `drqp_robot_state` → `drqp_brain`, MCP                  |
| `/robot/balance_mode`                           | `std_msgs/Bool`                   | latched                  | translator, GUI → `drqp_brain`                          |
| `/imu/data`                                     | `sensor_msgs/Imu`                 | depth 10                 | `drqp_imu` or Gazebo bridge → `drqp_brain`              |
| `/joy`                                          | `sensor_msgs/Joy`                 | depth 10                 | `drqp_joy` → translator                                 |
| `/joy/set_feedback`                             | `sensor_msgs/JoyFeedback`         | depth 10                 | translator → `drqp_joy`                                 |
| `/joy/set_haptic`                               | `drqp_interfaces/HapticEffect`    | depth 10                 | no publisher in the repo → `drqp_joy`                   |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth 10                 | `drqp_brain` → controller                               |
| `/joint_states`                                 | `sensor_msgs/JointState`          | depth 10                 | broadcaster → `drqp_brain`                              |

## Actions

- `/joint_trajectory_controller/follow_joint_trajectory`
  (`control_msgs/FollowJointTrajectory`): used by the brain for the stand-up,
  sit-down, and reboot sequences.

## Contract

- The lifecycle event and state strings are the `RobotStateMachine` event and
  state names ([robot_state](packages/runtime/drqp_brain/robot_state.md)).
- While balance mode is on, `MovementCommand`s are ignored.
- `/imu/data` older than `imu_balance_timeout_sec` (1 s) disables balance mode.
- `/odom` exists only in simulation, bridged from Gazebo.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:224` — movement command
  subscription
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:240` — `/robot_state`
  subscription
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:254` — trajectory
  publisher
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:322` — trajectory action
  client
- `packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py:45` —
  `/robot_state` publisher
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:96` —
  latched balance toggle
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:145` — `/imu/data`
  publisher
- `packages/runtime/drqp_joy/src/game_controller.cpp:188` — `joy` publisher
