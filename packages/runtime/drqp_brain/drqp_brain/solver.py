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

import math


def safe_acos(num):
    if num < -1.01 or num > 1.01:
        return False, 0
    if num < -1.0:
        num = -1.0
    if num > 1.0:
        num = 1.0
    return True, math.acos(num)


class StubLogger:
    """Stub logger that does nothing."""

    def debug(self, *args, **kwargs):
        pass


class Solver:
    """
    Inverse kinematics solver for 3DOF leg.

    Math is based on https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
    """

    def __init__(self, coxa, femur, tibia, logger=None):
        """
        Initialize the solver with leg dimensions.

        coxa - length of the coxa in meters
        femur - length of the femur in meters
        tibia - length of the tibia in meters
        logger - ROS2 logger instance (typically node.get_logger())
        """
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

        self.logger = logger or StubLogger()

    def get_logger(self):
        return self.logger

    def solve(self, x, y, z):
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
        #        /  _/  a2|  Z_off |  |
        # (y, z)/_/       |        V  | Z
        #   <--@----------------------+
        #      |                      0
        #      |<-------- L1 -------->|
        Z_offset = -z
        L1 = math.hypot(x, y)
        L = math.hypot(Z_offset, L1 - self.coxa)
        alpha1_acos_input = Z_offset / L
        solvable, alpha1 = safe_acos(alpha1_acos_input)

        if not solvable:
            print(f"Can't solve `alpha1` for {x=}, {y=}, {z=}")
            return False, 0, 0, 0

        alpha2_acos_input = (self.femur**2 + L**2 - self.tibia**2) / (2 * self.femur * L)
        solvable, alpha2 = safe_acos(alpha2_acos_input)
        self.alpha = alpha1 + alpha2

        if not solvable:
            print(f"Can't solve `alpha2` for {x=}, {y=}, {z=}")
            return False, 0, 0, 0

        beta_acos_input = (self.tibia**2 + self.femur**2 - L**2) / (2 * self.tibia * self.femur)
        solvable, self.beta = safe_acos(beta_acos_input)

        if not solvable:
            print(f"Can't solve `beta` for {x=}, {y=}, {z=}")
            return False, 0, 0, 0

        self.get_logger().debug(
            f'Solved  for {x=}, {y=}, {z=}, {self.alpha=} {self.beta=}, {self.gamma=}',
            throttle_duration_sec=0.01,
        )
        return True, self.alpha, self.beta, self.gamma
