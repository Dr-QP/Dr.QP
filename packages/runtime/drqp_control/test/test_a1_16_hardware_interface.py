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


from pathlib import Path
import unittest

from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import rclpy
from rclpy.action import ActionClient
import rclpy.time
from trajectory_msgs.msg import JointTrajectoryPoint

recording = False


def generate_test_description():
    device_address_name = '/dev/ttySC0' if recording else 'playback'
    test_data_dir = Path(__file__).parent / 'test_data'
    hardware_device_address = (
        f'{device_address_name}|{test_data_dir / "integration-a1_16_hardware_interface.json"}'
    )
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
                    'hardware_device_address': hardware_device_address,
                }.items(),
            ),
            # Launch tests after delay
            TimerAction(period=4.0, actions=[ReadyToTest()]),
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
            self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.run_duration = 20
        self.max_messages = 10

    def tearDown(self):
        self.node.destroy_node()

    def test_write_position_control_interface(self, proc_output):
        """Check whether servo goals are written to the servos when using position control interface."""

        self.issue_move_command()
        #     end_time = time.time() + self.run_duration
        #     while time.time() < end_time:
        #         torque_on = TorqueOn()
        #         torque_on.joint_names = joints
        #         torque_on.torque_on = [True] * servo_count
        #         pub_torque_on.publish(torque_on)

        #         test_goal = MultiServoPositionGoal()
        #         test_goal.header.stamp = self.node.get_clock().now().to_msg()
        #         test_goal.header.frame_id = 'test_frame'
        #         test_goal.mode = MultiServoPositionGoal.MODE_SYNC
        #         test_goal.goals = [
        #             ServoPositionGoal(
        #                 joint_name=joint,
        #                 position_as_radians=position_as_radians,
        #                 playtime_ms=playtime_ms,
        #             )
        #             for joint in joints
        #         ]

        #         pub_goals.publish(test_goal)

        #         for _ in range(10):
        #             rclpy.spin_once(self.node, timeout_sec=0.01)
        #         if len(msgs_received) > self.max_messages:
        #             break
        #     self.assertGreater(len(msgs_received), self.max_messages)

        #     for msg in msgs_received:
        #         self.assertEqual(len(msg.servos), servo_count)
        #         joints_to_check = list(joints)
        #         for servo in msg.servos:
        #             self.assertAlmostEqual(
        #                 servo.position_as_radians,
        #                 position_as_radians,
        #                 delta=0.01,
        #             )
        #             joints_to_check.remove(servo.joint_name)
        #         self.assertListEqual(joints_to_check, [], 'Not all servos were received')

    def issue_move_command(self):
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
        position_as_radians = 0.5

        target_point = JointTrajectoryPoint()
        target_point.time_from_start = rclpy.time.Duration(seconds=1.5).to_msg()
        target_point.positions = [position_as_radians] * len(joint_names)
        target_point.effort = [1.0] * len(joint_names)

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
