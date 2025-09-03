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


import unittest

from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import rclpy
from rclpy.action import ActionClient
import rclpy.time
from trajectory_msgs.msg import JointTrajectoryPoint


def generate_test_description():
    drqp_control_launch_path = PathJoinSubstitution(
        [
            FindPackageShare('drqp_control'),
            'launch',
        ]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    drqp_control_launch_path / 'ros2_controller.launch.py'
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'hardware_device_address': 'mock_servo',
                }.items(),
            ),
            # Wait is done by trajectory_client.wait_for_server()
            ReadyToTest(),
        ]
    )


class TestA116HardwareInterface(unittest.TestCase):
    """Test the pose_setter node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_servo_driver')
        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        self.trajectory_client.wait_for_server()

    def tearDown(self):
        self.node.destroy_node()

    # def test_zero_position_zero_effort(self):
    #     self._check_position_control(position=0, effort=0, expected_position=0)

    # def test_position_control_effort_off(self):
    #     self._check_position_control(position=1, effort=0, expected_position=1)

    def test_position_control_effort_on(self):
        self._check_position_control(position=0, effort=1, expected_position=0)
        self._check_position_control(position=0.5, effort=1, expected_position=0.5)
        self._check_position_control(position=1, effort=1, expected_position=1)

    # def test_position_control_effort_reboot(self):
    #     self._check_position_control(position=0, effort=-1, expected_position=0)
    #     self._check_position_control(position=1, effort=-1, expected_position=1)
    #     self._check_position_control(position=0, effort=-10, expected_position=0)

    # def test_position_control_invalid_position(self):
    #     self._check_position_control(position=float('inf'), effort=1, expected_position=0)
    #     self._check_position_control(position=-float('inf'), effort=1, expected_position=0)

    # def test_position_control_invalid_effort(self):
    #     self._check_position_control(position=0, effort=-float('inf'), expected_position=0)
    #     self._check_position_control(position=1, effort=-float('inf'), expected_position=1)

    def _check_position_control(self, position, effort, expected_position):
        joint_names = [
            'dr_qp/left_front_coxa',
            'dr_qp/left_front_femur',
            'dr_qp/left_front_tibia',
            'dr_qp/right_front_coxa',
            'dr_qp/right_front_femur',
            'dr_qp/right_front_tibia',
            'dr_qp/left_middle_coxa',
            'dr_qp/left_middle_femur',
            'dr_qp/left_middle_tibia',
            'dr_qp/right_middle_coxa',
            'dr_qp/right_middle_femur',
            'dr_qp/right_middle_tibia',
            'dr_qp/left_back_coxa',
            'dr_qp/left_back_femur',
            'dr_qp/left_back_tibia',
            'dr_qp/right_back_coxa',
            'dr_qp/right_back_femur',
            'dr_qp/right_back_tibia',
        ]

        target_point = JointTrajectoryPoint()
        target_point.time_from_start = rclpy.time.Duration(seconds=1.5).to_msg()
        target_point.positions = [position] * len(joint_names)
        target_point.effort = [effort] * len(joint_names)

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [target_point]
        trajectory_goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'test_frame'

        goal_handle_future = self.trajectory_client.send_goal_async(trajectory_goal)
        rclpy.spin_until_future_complete(self.node, goal_handle_future)
        goal_handle = goal_handle_future.result()
        self.assertIsNotNone(goal_handle)
        if goal_handle is None:
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result: FollowJointTrajectory.Result | None = None
        result = result_future.result().result

        self.assertIsNotNone(result)
        if result is None:
            return
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)


# Post-shutdown tests
@post_shutdown_test()
class TestA116HardwareInterfaceShutdown(unittest.TestCase):
    """Test the a1_16_hardware_interface shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        asserts.assertExitCodes(proc_info)
