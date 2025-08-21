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

from typing import Callable

from control_msgs.action import FollowJointTrajectory
from drqp_brain.models import HexapodModel
import numpy as np
from rclpy.action import ActionClient
import rclpy.node
import rclpy.publisher
import rclpy.time
import trajectory_msgs.msg

kFemurOffsetAngle = -13.11
kTibiaOffsetAngle = -32.9


class JointTrajectoryBuilder:
    def __init__(self, hexapod: HexapodModel):
        self.hexapod = hexapod
        self.points = []

    def add_point_from_hexapod(self, seconds_from_start, effort=1.0, joint_mask=None):
        positions = []
        efforts = []
        self.joint_names = []

        for leg in self.hexapod.legs:
            for joint, angle in [
                ('coxa', leg.coxa_angle),
                ('femur', leg.femur_angle + kFemurOffsetAngle),
                ('tibia', leg.tibia_angle + kTibiaOffsetAngle),
            ]:
                if joint_mask is not None and joint not in joint_mask:
                    efforts.append(0.0)
                else:
                    efforts.append(effort)
                positions.append(float(np.radians(angle)))
                self.joint_names.append(f'dr_qp/{leg.label.name}_{joint}')

        self.add_point(positions, efforts, seconds_from_start)

    def add_point(self, positions, effort, seconds_from_start):
        self.points.append(
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions=positions,
                effort=effort,
                time_from_start=rclpy.time.Duration(seconds=seconds_from_start).to_msg(),
            )
        )

    def publish(self, pub: rclpy.publisher.Publisher):
        msg = trajectory_msgs.msg.JointTrajectory(joint_names=self.joint_names, points=self.points)
        pub.publish(msg)

    def publish_action(
        self,
        action_client: ActionClient,
        node: rclpy.node.Node,
        result_callback: Callable,
    ):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory_msgs.msg.JointTrajectory(
            joint_names=self.joint_names, points=self.points
        )

        goal_handle_future = action_client.send_goal_async(goal)

        def result_response_callback(future):
            node.get_logger().debug(f'Result received: {future.result().result}')
            result_callback()

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                node.get_logger().error('Goal rejected')
                return

            node.get_logger().debug('Goal accepted')

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_response_callback)

        goal_handle_future.add_done_callback(goal_response_callback)
