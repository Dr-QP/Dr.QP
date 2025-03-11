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
import numpy as np

# ROS 2 imports
import rclpy
import rclpy.node

import sensor_msgs.msg
import drqp_interfaces.msg

# based on https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/

def safe_acos(num):
    if num < -1 or num > 1:
        return False, 0
    return True, math.acos(num)

class RobotBrain(rclpy.node.Node):

    def get_param(self, name):
        return self.get_parameter(name).value

    def __init__(self):
        super().__init__("drqp_brain")

        self.pose_async_publisher = self.create_publisher(
            drqp_interfaces.msg.MultiAsyncPositionCommand, "/pose_async", qos_profile=10)
        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, "/joint_states", qos_profile=10)

        self.alpha = 0
        self.beta = 0
        self.gamma = 0

        # TODO (anton-matosov): Use robot description and TF to get these values instead of using hard coded values
        self.coxa = 0.053
        self.femur = 0.066225
        self.tibia = 0.120531

        x = 0.07
        y = x
        z = -0.1
        self.current_frame = 0
        sequence_z_down = [
            # x, y, z
            (x, 0.04, z, "forward-a-bit-left-z"),
            (x, 0.04, z - 0.005, "forward-a-bit-left-z-0.005"),
            (x, 0.04, z - 0.01, "forward-a-bit-left-z-0.01"),
            (x, 0.04, z - 0.015, "forward-a-bit-left-z-0.015"),
            (x, 0.04, z - 0.02, "forward-a-bit-left-z-0.02"),
            (x, 0.04, z - 0.025, "forward-a-bit-left-z-0.025"),
            (x, 0.04, z - 0.03, "forward-a-bit-left-z-0.03"),
            (x, 0.04, z - 0.035, "forward-a-bit-left-z-0.035"),
            (x, 0.04, z - 0.04, "forward-a-bit-left-z-0.04"),

            # These values are technically not reachable, but algorithm doesn't blow up
            (x, 0.04, z - 0.045, "forward-a-bit-left-z-0.045"),
            (x, 0.04, z - 0.05, "forward-a-bit-left-z-0.05"),
            (x, 0.04, z - 0.055, "forward-a-bit-left-z-0.055"),
        ]

        sequence_all_quadrants = [
            # All quadrants, 1/8 step
            # x, y, z
            (x, 0, z, "forward"),
            (x, y, z, "forward-left"),
            (0, y, z, "left"),
            (-x, y, z, "backward-left"),
            (-x, 0, z, "backward"),
            (-x, -y, z, "backward-right"),
            (0, -y, z, "right"),
            (x, -y, z, "forward-right"),
        ]

        x = 0.1
        y = x
        z = -0.05
        scalar = 0.04
        sequence_xy_little_circle = [
            # x, y, z
            (x + math.cos(i) * scalar, y + math.sin(i) * scalar, z, f"xy-circle step {i}") for i in np.linspace(0, np.pi * 2, 32)
        ]

        x = 0.1
        y = x
        z = -0.05 # Current algo has a limit of never going above 0 as it uses absolute value of z
        scalar = 0.04
        sequence_yz_little_circle = [
            # x, y, z
            (x, y + math.sin(i) * scalar, z + math.cos(i) * scalar, f"yz-circle step {i}") for i in np.linspace(0, np.pi * 2, 32)
        ]

        x = 0.1
        y = x
        z = -0.05
        scalar = 0.04
        sequence_xz_little_circle = [
            # x, y, z
            (x + math.sin(i) * scalar, y, z + math.cos(i) * scalar, f"xz-circle step {i}") for i in np.linspace(0, np.pi * 2, 32)
        ]



        self.sequence = sequence_xy_little_circle

        self.current_test_frame = 0
        self.test_angles = [
            # gamma, alpha, beta
            (0, math.pi / 2, math.pi), # Straight leg out
            (0, math.pi / 2 + math.pi / 16, math.pi), # Straight leg out, femur a bit up
            (0, math.pi / 2 + math.pi / 8, math.pi), # Straight leg out, femur a bit up + 1
            (0, math.pi / 2 + math.pi / 4, math.pi), # Straight leg out, femur a bit up + 2

            (0, math.pi / 2, math.pi), # Straight leg out
            (0, math.pi / 2, math.pi + math.pi / 16), # Straight leg out, Tibia a bit up
            (0, math.pi / 2, math.pi + math.pi / 8), # Straight leg out, Tibia a bit up + 1
            (0, math.pi / 2, math.pi + math.pi / 4), # Straight leg out, Tibia a bit up + 2
        ]


        self.timer = self.create_timer(5 / len(self.sequence), self.on_timer)

    def on_timer(self):
        self.frame = self.sequence[self.current_frame]

        test = False
        if test:
            self.gamma, self.alpha, self.beta = self.test_angles[self.current_test_frame]
            solved = True
        else:
            solved, self.alpha, self.beta, self.gamma = self.solve_for(*self.frame)

        if solved:
            self.publish()

        if test:
            self.current_test_frame += 1
            if self.current_test_frame >= len(self.test_angles):
                self.current_test_frame = 0
        else:
            self.current_frame += 1
            if self.current_frame >= len(self.sequence):
                self.current_frame = 0
                print("===========================   DONE   ===========================")

    def solve_for(self, x, y, z, pose_name):
        print(f"Solving for {x=}, {y=}, {z=}, pose: {pose_name}")

        # ROS is using right hand side coordinates system
        #
        # X - forward
        # Y - left
        # Z - up
        #
        # (x=1, y=0, z=0) - is Forward

        #  (view from the top)
        #                      ^
        #            @ (x, y) /|\
        #             \        |
        #              \       |
        #               *      |
        #                \     |
        #              L1 \    |
        #                  *  g|  g - gamma
        #                   \--|
        #                    \ |
        #                     \| X
        #   <------------------+
        #                  Y    0
        #
        self.gamma = math.atan2(y, x)

        # <img src=https://oscarliang.com/wp-content/uploads/2012/01/2-IK-side1.jpg />
        #  (view from the side)
        #                             ^
        #                            /|\
        #                             |
        #                             |    a - alpha
        #             *\ Femur        |    b - beta
        #            /`b\             |
        #           /    \     Coxa   |
        #    Tibia /   a1(*-----------|
        #         /  L _/(|        ^  |
        #        /  _/  a2|   Zoff |  |
        # (y, z)/_/       |        V  | Z
        #   <--@----------------------+
        #      |                      0
        #      |<-------- L1 -------->|
        Z_offset = abs(z)
        L1 = math.sqrt(x ** 2 + y ** 2)
        L = math.sqrt(Z_offset ** 2 + (L1 - self.coxa) ** 2)
        alpha1_acos_input = Z_offset / L
        solvable, alpha1 = safe_acos(alpha1_acos_input)

        if not solvable:
            print(f"Can't solve `alpha1` for {x=}, {y=}, {z=}, pose: {pose_name}")
            return False, 0, 0, 0

        alpha2_acos_input = (self.femur ** 2 + L ** 2 - self.tibia ** 2) / (2 * self.femur * L)
        solvable, alpha2 = safe_acos(alpha2_acos_input)
        self.alpha = alpha1 + alpha2

        if not solvable:
            print(f"Can't solve `alpha2` for {x=}, {y=}, {z=}, pose: {pose_name}")
            return False, 0, 0, 0

        beta_acos_input = (self.tibia ** 2 - self.femur ** 2 - L ** 2) / (2 * self.tibia * self.femur)
        solvable, self.beta = safe_acos(beta_acos_input)

        if not solvable:
            print(f"Can't solve `beta` for {x=}, {y=}, {z=}, pose: {pose_name}")
            return False, 0, 0, 0

        print(f"Solved {self.alpha=} {self.beta=}, {self.gamma=}, pose: {pose_name}\n")
        return True, self.alpha, self.beta, self.gamma

    def publish(self):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        leg = "dr_qp/front_left_"
        msg.name = [
            leg + "coxa",
            leg + "femur",
            leg + "tibia",
        ]

        msg.position = [
            self.gamma,
            math.pi / 2 -self.alpha,
            math.pi - self.beta,
        ]
        self.joint_state_pub.publish(msg)


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

if __name__ == "__main__":
    main()
