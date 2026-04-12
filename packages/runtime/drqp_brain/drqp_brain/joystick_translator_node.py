#!/usr/bin/env python3
#
# Copyright (c) 2017-2025 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import argparse
from typing import Optional

from drqp_brain.haptics import (
    HapticFeedbackScheduler,
    ScheduledFeedbackCommand,
    control_mode_feedback_pattern,
    gait_feedback_pattern,
)
from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.joystick_input_handler import JoystickInputHandler
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.utilities
import sensor_msgs.msg
import std_msgs.msg


class JoystickTranslatorNode(rclpy.node.Node):
    """
    ROS node that translates joystick input to semantic robot commands.

    Subscribes to /joy topic and publishes MovementCommand messages
    and robot events, abstracting away hardware-specific joystick
    details into application-level semantics. State machine events
    are published directly to /robot_event.
    """

    def __init__(
        self,
        haptic_feedback_scheduler: Optional[HapticFeedbackScheduler] = None,
    ):
        super().__init__('joystick_translator')

        # Track gait state for movement commands
        self.gait_index = 0
        self.gaits = [
            MovementCommandConstants.GAIT_TRIPOD,
            MovementCommandConstants.GAIT_RIPPLE,
            MovementCommandConstants.GAIT_WAVE,
        ]

        # Set up joystick input handler with button callbacks
        button_callbacks = {
            ButtonIndex.DpadLeft: lambda b, e: self._prev_gait(),
            ButtonIndex.DpadRight: lambda b, e: self._next_gait(),
            ButtonIndex.L1: lambda b, e: self._publish_control_mode_change(),
            ButtonIndex.PS: lambda b, e: self._publish_event(
                'kill_switch_pressed'
            ),
            ButtonIndex.TouchpadButton: lambda b, e: self._publish_event(
                'kill_switch_pressed'
            ),
            ButtonIndex.Start: lambda b, e: self._publish_event(
                'reboot_servos'
            ),
            ButtonIndex.Select: lambda b, e: self._publish_event('finalize'),
        }
        self.joystick_input_handler = JoystickInputHandler(
            button_callbacks=button_callbacks
        )

        # Subscribe to joystick input
        self.joystick_sub = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self._joy_callback, qos_profile=10
        )

        # Publishers for semantic commands
        self.movement_command_pub = self.create_publisher(
            MovementCommand, '/robot/movement_command', qos_profile=10
        )
        self.robot_event_pub = self.create_publisher(
            std_msgs.msg.String, '/robot_event', qos_profile=10
        )
        self.joy_feedback_pub = self.create_publisher(
            sensor_msgs.msg.JoyFeedback,
            '/joy/set_feedback',
            qos_profile=10,
        )
        self.haptic_feedback_scheduler = (
            haptic_feedback_scheduler or HapticFeedbackScheduler()
        )
        self._pending_feedback_commands: list[ScheduledFeedbackCommand] = []
        self._feedback_timer = self.create_timer(
            0.02, self._dispatch_pending_feedback
        )
        self._haptics_warning_logged = False

        self.get_logger().info('Joystick translator node initialized')

    def _joy_callback(self, joy_msg: sensor_msgs.msg.Joy):
        """
        Process joystick message and publish semantic commands.

        Parameters
        ----------
        joy_msg : sensor_msgs.msg.Joy
            Raw joystick input message

        """
        # Process joystick input through handler
        self.joystick_input_handler.process_joy_message(joy_msg)

        # Publish movement command
        self._publish_movement_command()

    def _publish_movement_command(self):
        """Publish current movement state as semantic command."""
        msg = MovementCommand()

        # Convert Point3D to Vector3
        msg.stride_direction = (
            self.joystick_input_handler.direction.to_vector3()
        )
        msg.rotation_speed = float(self.joystick_input_handler.rotation_speed)
        msg.body_translation = (
            self.joystick_input_handler.body_translation.to_vector3()
        )
        msg.body_rotation = (
            self.joystick_input_handler.body_rotation.to_vector3()
        )
        msg.gait_type = self.gaits[self.gait_index]

        self.movement_command_pub.publish(msg)

    def _publish_event(self, event: str):
        """
        Publish a robot state machine event.

        Parameters
        ----------
        event : str
            Event identifier (e.g., 'kill_switch_pressed', 'finalize')

        """
        msg = std_msgs.msg.String()
        msg.data = event

        self.robot_event_pub.publish(msg)
        self.get_logger().info(f'Published event: {event}')

    def _prev_gait(self):
        """Switch to previous gait type."""
        self._set_gait_index((self.gait_index - 1) % len(self.gaits))

    def _next_gait(self):
        """Switch to next gait type."""
        self._set_gait_index((self.gait_index + 1) % len(self.gaits))

    def _publish_control_mode_change(self):
        """Handle control mode change."""
        previous_mode = self.joystick_input_handler.control_mode
        self.joystick_input_handler.next_control_mode()

        if self.joystick_input_handler.control_mode == previous_mode:
            return

        self.get_logger().info(
            'Switching control mode: '
            f'{self.joystick_input_handler.control_mode}'
        )
        self._schedule_haptic_feedback(
            control_mode_feedback_pattern(
                self.joystick_input_handler.control_mode
            )
        )

    def _set_gait_index(self, new_index: int):
        """
        Update gait state and queue haptic feedback for confirmed changes.
        """
        if new_index == self.gait_index:
            return

        self.gait_index = new_index
        gait_name = self.gaits[self.gait_index]
        self.get_logger().info(f'Switching gait: {gait_name}')
        self._schedule_haptic_feedback(gait_feedback_pattern(gait_name))

    def _schedule_haptic_feedback(self, pattern):
        """Queue haptic feedback commands without blocking the control path."""
        commands = self.haptic_feedback_scheduler.schedule(pattern)
        if not commands:
            return

        pending_commands = [
            command
            for command in self._pending_feedback_commands
            if command.channel_id != pattern.channel_id
        ]

        if len(pending_commands) != len(self._pending_feedback_commands):
            pending_commands.append(
                ScheduledFeedbackCommand(
                    due_at=self.haptic_feedback_scheduler.now(),
                    channel_id=pattern.channel_id,
                    intensity=0.0,
                )
            )

        pending_commands.extend(commands)
        self._pending_feedback_commands = sorted(
            pending_commands,
            key=lambda command: command.due_at,
        )

    def _dispatch_pending_feedback(self):
        """Publish any haptic commands whose scheduled time has arrived."""
        if not self._pending_feedback_commands:
            return

        now = self.haptic_feedback_scheduler.now()

        while (
            self._pending_feedback_commands
            and self._pending_feedback_commands[0].due_at <= now
        ):
            command = self._pending_feedback_commands.pop(0)
            self._publish_haptic_command(command)

    def _reset_pending_feedback(self):
        """
        Discard pending commands and reset scheduler state for those channels.

        Resetting allows re-selection of the same gait/mode to trigger fresh
        feedback once the haptics backend becomes available again.
        """
        dropped_channels = {
            command.channel_id for command in self._pending_feedback_commands
        }
        self._pending_feedback_commands.clear()
        for channel_id in dropped_channels:
            self.haptic_feedback_scheduler.reset_channel(channel_id)

    def _publish_haptic_command(self, command: ScheduledFeedbackCommand):
        """Publish a single JoyFeedback command."""
        feedback = sensor_msgs.msg.JoyFeedback()
        feedback.type = sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE
        feedback.id = command.channel_id
        feedback.intensity = float(command.intensity)

        try:
            self.joy_feedback_pub.publish(feedback)
        except Exception as exc:  # noqa: BLE001
            if not self._haptics_warning_logged:
                self.get_logger().warning(
                    'Failed to publish DualSense haptic feedback: '
                    f'{exc}. Continuing without controller feedback'
                )
                self._haptics_warning_logged = True
            self._reset_pending_feedback()


def main():
    """Entry point for joystick translator node."""
    node = None
    try:
        parser = argparse.ArgumentParser(
            'Joystick to semantic command translator ROS node'
        )
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])
        rclpy.init()
        node = JoystickTranslatorNode(**vars(args))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        if node is not None:
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
