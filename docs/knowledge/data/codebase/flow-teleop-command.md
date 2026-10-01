---
type: codebase
description: How a DualSense stick movement becomes a servo I-JOG packet — joy node, translator, brain loop, trajectory controller, hardware interface.
source: packages/runtime
source_digest: sha256:693b7ca67afe3a62f168b00a176142189821fadb2c1c5070b1d45fb93b00bf5a
verified:
  by: claude-code/opus-5.5
  at: 2026-09-27T21:00:00Z
stale_after: 2026-11-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime
---

# flow-teleop-command

The main control path while the robot is in `torque_on`. The keyboard GUI and
the MCP server join at step 3 by publishing `MovementCommand` directly.

## Trace

1. **Joy node.** `GameController` polls SDL on a wall timer and publishes
   `sensor_msgs/Joy` on `joy`. ([drqp_joy](packages/runtime/drqp_joy.md),
   `game_controller.cpp:188`)

2. **Axis mapping.** `JoystickTranslatorNode._joy_callback` runs
   `JoystickInputHandler.process_joy_message`. How the axes map depends on the
   control mode:

   - Walk: left stick → stride, left trigger → stride z, right x → rotation.
   - BodyPosition and BodyRotation modes use the sticks for the body pose.

   Button events go out separately:

   - D-pad: gait.
   - L1: control mode.
   - R1: balance toggle.
   - PS and touchpad: kill switch.
   - Start: servo reboot.
   - Select: finalize.

   ([drqp_brain](packages/runtime/drqp_brain.md),
   `joystick_input_handler.py:95`, `joystick_translator_node.py:60`)

3. **Semantic command.** Every `Joy` message causes one `MovementCommand` on
   `/robot/movement_command`. (`joystick_translator_node.py:128`)

4. **Brain intake.** `HexapodBrain.process_movement_command` stores the command
   and selects the gait, unless balance mode is on, in which case it drops the
   command. (`brain_node.py:374`)

5. **Control tick.** `_run_loop` runs at `control_rate_hz` (25 Hz by default).
   It advances the `WalkController`, builds a 2-point foot-target window, and
   solves IK:

   - Analytic `LegModel.solve_ik`, with a MoveIt collision check.
   - Or MoveIt IK.

   It skips publishing if the motion-state key has not changed.
   (`brain_node.py:484`, `brain_node.py:561`;
   [drqp_kinematics](packages/runtime/drqp_kinematics.md))

6. **Trajectory.** `JointTrajectoryBuilder.publish` sends a `JointTrajectory` to
   `/joint_trajectory_controller/joint_trajectory`, with points at 1/rate and
   2/rate seconds. (`joint_trajectory_builder.py:90`)

7. **Controller.** `joint_trajectory_controller` interpolates the trajectory at
   100 Hz and writes the `position` and `effort` command interfaces.
   ([drqp_control](packages/runtime/drqp_control.md), `drqp_controllers.yml:13`)

8. **Hardware.** `a1_16_hardware_interface::write` maps the joints to servo
   positions and sends one broadcast I-JOG packet over the UART.
   ([drqp_control](packages/runtime/drqp_control.md),
   [drqp_a1_16_driver](packages/runtime/drqp_a1_16_driver.md),
   `a1_16_hardware_interface.cpp:274`)

## Failure modes

- **IK not ready:** the URDF limits or the MoveIt scene are not loaded yet. The
  tick returns without publishing.
- **Kinematics rejects the targets:** the brain logs a warning and skips the
  publish. The robot holds the last trajectory.
- **Translator not running:** step 2 has no node when bringup runs with
  `load_joystick_translator:=false`.
- **Serial failures during `read`** are logged at a 5 s throttle and do not stop
  the controller.
