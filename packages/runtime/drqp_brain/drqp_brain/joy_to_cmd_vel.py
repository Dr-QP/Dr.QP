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

from drqp_brain.joystick_input_handler import ControlMode, JoystickInputHandler
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.utilities
import sensor_msgs.msg


class JoyToCmdVel(rclpy.node.Node):
    """
    ROS node for converting joystick input to Twist commands.

    Subscribes to /joy topic and publishes to /cmd_vel topic.
    Maps joystick axes to linear and angular velocity commands.
    """

    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # Use handler only in Walk mode for converting joystick to cmd_vel
        self.joystick_input_handler = JoystickInputHandler()
        self.joystick_input_handler.control_mode = ControlMode.Walk

        self.joy_sub = self.create_subscription(
            sensor_msgs.msg.Joy,
            '/joy',
            self.joy_callback,
            qos_profile=10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=10,
        )

    def joy_callback(self, joy: sensor_msgs.msg.Joy):
        """
        Process joystick input and publish Twist message.

        Parameters
        ----------
        joy : sensor_msgs.msg.Joy
            The joystick message to process

        """
        # Use existing joystick input handler to process axes (Walk mode only)
        self.joystick_input_handler.process_joy_message(joy)

        # Publish Walk mode movement as Twist
        # Linear: x = forward/backward, y = left/right, z = up/down (trigger)
        # Angular: z = rotation (yaw)
        twist = Twist()
        twist.linear.x = float(self.joystick_input_handler.direction.x)
        twist.linear.y = float(self.joystick_input_handler.direction.y)
        twist.linear.z = float(self.joystick_input_handler.direction.z)
        twist.angular.z = float(self.joystick_input_handler.rotation_speed)

        self.cmd_vel_pub.publish(twist)


def main():
    node = None
    try:
        parser = argparse.ArgumentParser('Joy to cmd_vel converter')
        filtered_args = rclpy.utilities.remove_ros_args()
        parser.parse_args(args=filtered_args[1:])
        rclpy.init()
        node = JoyToCmdVel()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
