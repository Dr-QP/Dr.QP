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

from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.joystick_input_handler import JoystickInputHandler
from drqp_interfaces.msg import (
    MovementCommand,
    MovementCommandConstants,
    RobotCommand,
    RobotCommandConstants,
)
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.utilities
import sensor_msgs.msg
import std_msgs.msg


class JoystickTranslatorNode(rclpy.node.Node):
    """
    ROS node that translates joystick input to semantic robot commands.

    Subscribes to /joy topic and publishes MovementCommand and RobotCommand messages,
    abstracting away hardware-specific joystick details into application-level semantics.
    """

    def __init__(self):
        super().__init__('joystick_translator')

        # Track gait state for movement commands
        self.gait_index = 0
        self.gaits = [
            MovementCommandConstants.GAIT_TRIPOD,
            MovementCommandConstants.GAIT_RIPPLE,
            MovementCommandConstants.GAIT_WAVE,
        ]

        # Set up joystick input handler with button callbacks
        self.joystick_input_handler = JoystickInputHandler(
            button_callbacks={
                ButtonIndex.DpadLeft: lambda b, e: self._prev_gait(),
                ButtonIndex.DpadRight: lambda b, e: self._next_gait(),
                ButtonIndex.L1: lambda b, e: self._publish_control_mode_change(),
                ButtonIndex.PS: lambda b, e: self._publish_command(
                    RobotCommandConstants.KILL_SWITCH
                ),
                ButtonIndex.TouchpadButton: lambda b, e: self._publish_command(
                    RobotCommandConstants.KILL_SWITCH
                ),
                ButtonIndex.Start: lambda b, e: self._publish_command(
                    RobotCommandConstants.REBOOT_SERVOS
                ),
                ButtonIndex.Select: lambda b, e: self._publish_command(
                    RobotCommandConstants.FINALIZE
                ),
            }
        )

        # Subscribe to joystick input
        self.joystick_sub = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self._joy_callback, qos_profile=10
        )

        # Publishers for semantic commands
        self.movement_command_pub = self.create_publisher(
            MovementCommand, '/robot/movement_command', qos_profile=10
        )
        self.robot_command_pub = self.create_publisher(
            RobotCommand, '/robot/command', qos_profile=10
        )

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
        msg.stride_direction = self.joystick_input_handler.direction.to_vector3()
        msg.rotation_speed = float(self.joystick_input_handler.rotation_speed)
        msg.body_translation = self.joystick_input_handler.body_translation.to_vector3()
        msg.body_rotation = self.joystick_input_handler.body_rotation.to_vector3()
        msg.gait_type = self.gaits[self.gait_index]

        self.movement_command_pub.publish(msg)

    def _publish_command(self, command: str):
        """
        Publish a robot command.

        Parameters
        ----------
        command : str
            Command identifier (e.g., 'kill_switch', 'finalize', 'reboot_servos')

        """
        msg = RobotCommand()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = command

        self.robot_command_pub.publish(msg)
        self.get_logger().info(f'Published command: {command}')

    def _prev_gait(self):
        """Switch to previous gait type."""
        self.gait_index = (self.gait_index - 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index]}')

    def _next_gait(self):
        """Switch to next gait type."""
        self.gait_index = (self.gait_index + 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index]}')

    def _publish_control_mode_change(self):
        """Handle control mode change."""
        self.joystick_input_handler.next_control_mode()
        self.get_logger().info(
            f'Switching control mode: {self.joystick_input_handler.control_mode}'
        )


def main():
    """Entry point for joystick translator node."""
    node = None
    try:
        parser = argparse.ArgumentParser('Joystick to semantic command translator ROS node')
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
