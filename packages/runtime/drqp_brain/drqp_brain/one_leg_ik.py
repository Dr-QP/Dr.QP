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

from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = Quaternion()
    q.x = cj*sc - sj*cs
    q.y = cj*ss + sj*cc
    q.z = cj*cs - sj*sc
    q.w = cj*cc + sj*ss

    return q

# based on https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/

def safe_acos(num):
    if num < -1.01 or num > 1.01:
        return False, 0
    if num < -1.0:
        num = -1.0
    if num > 1.0:
        num = 1.0
    return True, math.acos(num)

# 'only_forward', 'only_left', 'only_down', 'all_quadrants', 'xy_little_circle', 'xz_little_circle', 'yz_little_circle', 'all_circles'

x = 0.0
y = 0.03
z = -0.07
sequence_z_down = [
    # x, y, z
    (x, y, z, 'forward-a-bit-left-z'),
    (x, y, z - 0.005, 'forward-a-bit-left-z-0.005'),
    (x, y, z - 0.01, 'forward-a-bit-left-z-0.01'),
    (x, y, z - 0.015, 'forward-a-bit-left-z-0.015'),
    (x, y, z - 0.02, 'forward-a-bit-left-z-0.02'),
    (x, y, z - 0.025, 'forward-a-bit-left-z-0.025'),
    (x, y, z - 0.03, 'forward-a-bit-left-z-0.03'),
    (x, y, z - 0.035, 'forward-a-bit-left-z-0.035'),
    (x, y, z - 0.04, 'forward-a-bit-left-z-0.04'),

    # These values are technically not reachable, but algorithm doesn't blow up
    (x, y, z - 0.045, 'forward-a-bit-left-z-0.045'),
    (x, y, z - 0.05, 'forward-a-bit-left-z-0.05'),
    (x, y, z - 0.055, 'forward-a-bit-left-z-0.055'),
]

sequence_all_quadrants = [
    # All quadrants, 1/8 step
    # x, y, z
    (x, 0, z, 'forward'),
    (x, y, z, 'forward-left'),
    (0, y, z, 'left'),
    (-x, y, z, 'backward-left'),
    (-x, 0, z, 'backward'),
    (-x, -y, z, 'backward-right'),
    (0, -y, z, 'right'),
    (x, -y, z, 'forward-right'),
]

coxa = 0.053
femur = 0.066225
tibia = 0.123
max_leg_reach = coxa + femur + tibia
sequence_only_forward = [
    (max_leg_reach, 0.0, 0.0, 'forward'),
]

sequence_only_left = [
    (0.0, max_leg_reach, 0.0, 'left'),
]

sequence_only_down = [
    (0.0, 0.0, -(femur + tibia), 'down'),
]

steps = 64
x = 0.15
y = 0.0
z = -0.08
scalar = 0.05
sequence_xy_little_circle = [
    # x, y, z
    (x + math.cos(i) * scalar, y + math.sin(i) * scalar, z, f'xy-circle step {i}') for i in np.linspace(np.pi, np.pi * 3, steps)
]

sequence_xz_little_circle = [
    # x, y, z
    (x + math.sin(i) * scalar, y, z + math.cos(i) * scalar, f'xz-circle step {i}') for i in np.linspace(0, np.pi * 2, steps)
]
sequence_xz_little_circle_last_quarter = [
    # x, y, z
    (x + math.sin(i) * scalar, y, z + math.cos(i) * scalar, f'xz-circle step {i}') for i in np.linspace(np.pi * 1.5, np.pi * 2, int(steps / 4))
]

sequence_yz_little_circle = [
    # x, y, z
    (x, y + math.sin(i) * scalar, z + math.cos(i) * scalar, f'yz-circle step {i}') for i in np.linspace(0, np.pi * 2, steps)
]

repeat_circles = 3
all_circles = sequence_xy_little_circle * repeat_circles \
    + sequence_xz_little_circle_last_quarter \
    + sequence_xz_little_circle  * repeat_circles \
    + sequence_yz_little_circle  * repeat_circles \
    + [*reversed(sequence_xz_little_circle_last_quarter)]

sequences = {
    'only_forward': sequence_only_forward,
    'only_left': sequence_only_left,
    'only_down': sequence_only_down,
    'all_quadrants': sequence_all_quadrants,
    'xy_little_circle': sequence_xy_little_circle,
    'xz_little_circle': sequence_xz_little_circle,
    'yz_little_circle': sequence_yz_little_circle,
    'all_circles': all_circles,
}


test_angles_femur = [
    # gamma, alpha, beta
    (0, math.pi / 2, math.pi), # Straight leg out
    (0, math.pi / 2 + math.pi / 16, math.pi), # Straight leg out, femur a bit up
    (0, math.pi / 2 + math.pi / 8, math.pi), # Straight leg out, femur a bit up + 1
    (0, math.pi / 2 + math.pi / 4, math.pi), # Straight leg out, femur a bit up + 2
]

test_angles_tibia = [
    # gamma, alpha, beta
    (0, math.pi / 2, math.pi), # Straight leg out
    (0, math.pi / 2, math.pi + math.pi / 16), # Straight leg out, Tibia a bit up
    (0, math.pi / 2, math.pi + math.pi / 8), # Straight leg out, Tibia a bit up + 1
    (0, math.pi / 2, math.pi + math.pi / 4), # Straight leg out, Tibia a bit up + 2
]

test_angles_straight_leg = [
    # gamma, alpha, beta
    (0, math.pi / 2, math.pi), # Straight leg out
]

test_angles = {
    'femur': test_angles_femur,
    'tibia': test_angles_tibia,
    'straight_leg': test_angles_straight_leg,
}

# Joint offsets from math model to physical robot
kFemurOffsetAngle = -13.11
kFemurOffsetRad = kFemurOffsetAngle * math.pi / 180
kTibiaOffsetAngle = -32.9
kTibiaOffsetRad = kTibiaOffsetAngle * math.pi / 180

class RobotBrain(rclpy.node.Node):

    def get_param(self, name):
        return self.get_parameter(name).value

    def __init__(self, parsed_args):
        super().__init__('drqp_brain')

        self.pose_async_publisher = self.create_publisher(
            drqp_interfaces.msg.MultiAsyncPositionCommand, '/pose_async', qos_profile=10)
        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/joint_states', qos_profile=50)

        self.test = parsed_args.test
        self.sequence = sequences[parsed_args.sequence]
        self.sequence_repeat = parsed_args.sequence_repeat
        self.test_angles = test_angles[parsed_args.test_angles]

        self.alpha = 0
        self.beta = 0
        self.gamma = 0

        # TODO (anton-matosov): Use robot description and TF to get these values instead of using hard coded values
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_frame = 0
        self.current_test_frame = 0


        step_time = parsed_args.cycle_time_seconds / len(self.test_angles if self.test else self.sequence)
        self.timer = self.create_timer(step_time, self.on_timer)

    def on_timer(self):
        self.frame = self.sequence[self.current_frame]

        if self.test:
            self.gamma, self.alpha, self.beta = self.test_angles[self.current_test_frame]
            max_distance = self.coxa + self.femur + self.tibia
            self.frame = (max_distance, 0.0, 0.0, 'forward')
            solved = True
        else:
            solved, self.alpha, self.beta, self.gamma = self.solve_for(*self.frame)

        if solved:
            # self.publish_joints()
            self.publish_pose()
        self.broadcast_tf(self.frame)

        if self.test:
            self.current_test_frame += 1
            if self.current_test_frame >= len(self.test_angles):
                self.current_test_frame = 0
        else:
            self.current_frame += 1
            if self.current_frame >= len(self.sequence):
                self.current_frame = 0
                print('===========================   Sequence completed   ===========================')
                if self.sequence_repeat > 1:
                    self.sequence_repeat -= 1
                else:
                    self.timer.cancel()

    def solve_for(self, x, y, z, pose_name):
        # print(f'Solving for {x=}, {y=}, {z=}, pose: {pose_name}')

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
        Z_offset = -z
        L1 = math.hypot(x, y)
        L = math.hypot(Z_offset, L1 - self.coxa)
        alpha1_acos_input = Z_offset / L
        solvable, alpha1 = safe_acos(alpha1_acos_input)

        if not solvable:
            print(f'Can\'t solve `alpha1` for {x=}, {y=}, {z=}, pose: {pose_name}')
            return False, 0, 0, 0

        alpha2_acos_input = (self.femur ** 2 + L ** 2 - self.tibia ** 2) / (2 * self.femur * L)
        solvable, alpha2 = safe_acos(alpha2_acos_input)
        self.alpha = alpha1 + alpha2

        if not solvable:
            print(f'Can\'t solve `alpha2` for {x=}, {y=}, {z=}, pose: {pose_name}')
            return False, 0, 0, 0

        beta_acos_input = (self.tibia ** 2 + self.femur ** 2 - L ** 2) / (2 * self.tibia * self.femur)
        solvable, self.beta = safe_acos(beta_acos_input)

        if not solvable:
            print(f'Can\'t solve `beta` for {x=}, {y=}, {z=}, pose: {pose_name}')
            return False, 0, 0, 0

        print(f'Solved  for {x=}, {y=}, {z=}, {self.alpha=} {self.beta=}, {self.gamma=}, pose: {pose_name}')
        return True, self.alpha, self.beta, self.gamma

    def publish_joints(self):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        leg = 'dr_qp/front_left_'
        msg.name = [
            leg + 'coxa',
            leg + 'femur',
            leg + 'tibia',
        ]

        msg.position = [
            self.gamma,
            math.pi / 2 -self.alpha + kFemurOffsetRad,
            math.pi - self.beta + kTibiaOffsetRad,
        ]
        self.joint_state_pub.publish(msg)

    def publish_pose(self):
        msg = drqp_interfaces.msg.MultiAsyncPositionCommand()

        def rad_to_pos(angle):
            return int(angle * 1023 / (2 * math.pi)) + 512

        final_gamma = self.gamma
        final_alpha = math.pi / 2 -self.alpha + kFemurOffsetRad
        final_beta = math.pi - self.beta + kTibiaOffsetRad

        coxa_servo_pos = rad_to_pos(final_gamma)
        femur_servo_pos = rad_to_pos(final_alpha)
        tibia_servo_pos = rad_to_pos(math.pi - self.beta + kTibiaOffsetRad)

        coxa_servo_pos_right = rad_to_pos(-final_gamma)
        femur_servo_pos_right = rad_to_pos(-final_alpha)
        tibia_servo_pos_right = rad_to_pos(-final_beta)

        playtime = 0
        msg.positions = [
            drqp_interfaces.msg.AsyncPositionCommand(id=1, position=coxa_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=3, position=femur_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=5, position=tibia_servo_pos, playtime=playtime),

            drqp_interfaces.msg.AsyncPositionCommand(id=13, position=coxa_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=15, position=femur_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=17, position=tibia_servo_pos, playtime=playtime),

            drqp_interfaces.msg.AsyncPositionCommand(id=7, position=coxa_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=9, position=femur_servo_pos, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=11, position=tibia_servo_pos, playtime=playtime),

            drqp_interfaces.msg.AsyncPositionCommand(id=2, position=coxa_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=4, position=femur_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=6, position=tibia_servo_pos_right, playtime=playtime),

            drqp_interfaces.msg.AsyncPositionCommand(id=14, position=coxa_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=16, position=femur_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=18, position=tibia_servo_pos_right, playtime=playtime),

            drqp_interfaces.msg.AsyncPositionCommand(id=8, position=coxa_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=10, position=femur_servo_pos_right, playtime=playtime),
            drqp_interfaces.msg.AsyncPositionCommand(id=12, position=tibia_servo_pos_right, playtime=playtime),
        ]
        self.pose_async_publisher.publish(msg)


    def broadcast_tf(self, frame):
        x, y, z, pose_name = frame
        transforms = []
        t = TransformStamped()
        transforms.append(t)

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'dr_qp/front_left_coxa_servo'
        t.child_frame_id = 'front_left_target'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0


        coxa_joint = TransformStamped()
        transforms.append(coxa_joint)

        coxa_joint.header.stamp = self.get_clock().now().to_msg()
        coxa_joint.header.frame_id = 'dr_qp/front_left_coxa_servo'
        coxa_joint.child_frame_id = 'coxa_joint'
        coxa_joint.transform.rotation = quaternion_from_euler(0, 0, self.gamma)


        femur_joint = TransformStamped()
        transforms.append(femur_joint)

        femur_joint.header.stamp = self.get_clock().now().to_msg()
        femur_joint.header.frame_id = 'coxa_joint'
        femur_joint.child_frame_id = 'femur_joint'
        femur_joint.transform.translation.x = self.coxa
        femur_joint.transform.rotation = quaternion_from_euler(0, math.pi / 2 -self.alpha, 0)


        tibia_joint = TransformStamped()
        transforms.append(tibia_joint)

        tibia_joint.header.stamp = self.get_clock().now().to_msg()
        tibia_joint.header.frame_id = 'femur_joint'
        tibia_joint.child_frame_id = 'tibia_joint'
        tibia_joint.transform.translation.x = self.femur
        tibia_joint.transform.rotation = quaternion_from_euler(0, math.pi - self.beta, 0)


        leg_tip = TransformStamped()
        transforms.append(leg_tip)

        leg_tip.header.stamp = self.get_clock().now().to_msg()
        leg_tip.header.frame_id = 'tibia_joint'
        leg_tip.child_frame_id = 'leg_tip'
        leg_tip.transform.translation.x = self.tibia
        leg_tip.transform.translation.y = 0.
        leg_tip.transform.translation.z = 0.
        leg_tip.transform.rotation.x = 0.0
        leg_tip.transform.rotation.y = 0.0
        leg_tip.transform.rotation.z = 0.0
        leg_tip.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transforms)

def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--cycle-time-seconds', type=float, default=8)
    parser.add_argument('--sequence', type=str, default='all_circles', choices=sequences.keys())
    parser.add_argument('--test-angles', type=str, default='straight_leg', choices=test_angles.keys())
    parser.add_argument('--sequence-repeat', type=int, default=2)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    node = RobotBrain(parsed_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # lgtm [py/empty-except]
        pass

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
