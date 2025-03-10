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

# Standard Python imports
import argparse
import math
import sys


# ROS 2 imports
import rclpy
import rclpy.node

import drqp_interfaces.msg

class RobotBrain(rclpy.node.Node):

    def get_param(self, name):
        return self.get_parameter(name).value

    def __init__(self):
        super().__init__('drqp_brain')

        self.pose_async_publisher = self.create_publisher(drqp_interfaces.msg.MultiAsyncPositionCommand, '/pose_async', qos_profile=10)
        self.timer = self.create_timer(1, self.on_timer)

        self.alpha = 0
        self.beta = 0
        self.gamma = 0

        self.current_frame = 0
        self.sequence = [
            (0, 0, 0), # x, y, z
        ]

    def on_timer(self):
        self.frame = self.sequence[self.current_frame]
        self.current_frame += 1
        if self.current_frame >= len(self.sequence):
            self.current_frame = 0

        self.solve_for(*self.frame)

    def solve_for(self, x, y, z):
        print(f"Solving for {x}, {y}, {z}")

def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    node = RobotBrain()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()
