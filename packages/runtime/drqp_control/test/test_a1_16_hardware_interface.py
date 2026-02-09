# Copyright (c) 2017-present Anton Matosov
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
from control_msgs.msg import DynamicJointState, InterfaceValue
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import numpy as np
import rclpy
from rclpy.action import ActionClient
import rclpy.time
from trajectory_msgs.msg import JointTrajectoryPoint


def generate_test_description():
    drqp_controllers_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('drqp_control'),
            'launch',
            'ros2_controller.launch.py',
        ]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(drqp_controllers_launch_file),
                launch_arguments={
                    'use_gazebo': 'false',
                    'hardware_device_address': 'mock_servo',
                }.items(),
            ),
            TimerAction(period=0.1, actions=[ReadyToTest()]),
        ]
    )


default_interface_value = -123.321
neutral_position = 0.1


class TestA116HardwareInterface(unittest.TestCase):
    """Test the pose_setter node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_servo_driver_' + self.id().replace('.', '_'))
        self._wait_for_controller(self.node)
        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        self.assertTrue(self.trajectory_client.wait_for_server(timeout_sec=10.0))

        self.dynamic_joint_states_sub = self.node.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10,
        )
        self.joint_names = [
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
        self.non_joint_interface_names = ['battery_state']
        self._reset_feedback()

        # Get to the initial position
        self._check_position_control(
            position=neutral_position,
            effort=1,
            expected_position=neutral_position,
        )

    def _wait_for_controller(self, node):
        needed_controllers = ['joint_trajectory_controller', 'joint_state_broadcaster']
        check_controllers_running(node, needed_controllers)

    def _reset_feedback(self):
        self.joint_positions = [default_interface_value] * len(self.joint_names)
        self.joint_efforts = [default_interface_value] * len(self.joint_names)
        self.last_feedback = rclpy.time.Time(clock_type=rclpy.time.ClockType.ROS_TIME)

    def dynamic_joint_states_callback(self, msg: DynamicJointState):
        joint_positions = [default_interface_value] * len(self.joint_names)
        joint_efforts = [default_interface_value] * len(self.joint_names)

        interface_values = list[InterfaceValue](msg.interface_values)
        for name, values in zip(msg.joint_names, interface_values):
            if name in self.non_joint_interface_names:
                continue

            joint_index = self.joint_names.index(name)
            for interface_name, value in zip(values.interface_names, values.values):
                if interface_name == 'position':
                    joint_positions[joint_index] = value
                elif interface_name == 'effort':
                    joint_efforts[joint_index] = value

        self.joint_positions = joint_positions
        self.joint_efforts = joint_efforts
        self.last_feedback = rclpy.time.Time.from_msg(msg.header.stamp)
        # self.node.get_logger().info(f'Feedback received: {self.joint_positions}')

    def tearDown(self):
        self.trajectory_client.destroy()
        self.dynamic_joint_states_sub.destroy()
        self.node.destroy_node()
        self._reset_feedback()

    def test_node_start(self):
        check_node_running(self.node, 'robot_state_publisher')
        check_node_running(self.node, 'controller_manager')
        check_node_running(self.node, 'battery_state_broadcaster')
        check_node_running(self.node, 'joint_state_broadcaster')
        check_node_running(self.node, 'joint_trajectory_controller')

    def test_joint_states_published(self):
        check_if_js_published('/joint_states', self.joint_names)

    def test_effort_off(self):
        self._effort_test(0)

    def test_effort_reboot(self):
        self._effort_test(-1)

    def test_effort_infinite(self):
        self._effort_test(float('inf'))

    def test_effort_negative_infinite(self):
        self._effort_test(-float('inf'))

    def test_position_control_nan_effort(self):
        self._effort_test(float('nan'))

    def _effort_test(self, effort):
        # Go to the target effort
        self._check_position_control(
            position=neutral_position,
            effort=effort,
            expected_position=neutral_position,
        )

        if effort > 0:
            position = 1.0
            expected_position = 1.0
        else:
            position = 1.0
            expected_position = neutral_position

        # Try to change the position with effort under test
        # Check that the position didn't change
        self._check_position_control(
            position=position, effort=effort, expected_position=expected_position
        )

    def test_position_control_effort_on(self):
        self._check_position_control(position=1, effort=1, expected_position=1)

    def test_position_control_infinite_position(self):
        self._check_position_control(
            position=float('inf'),
            effort=1,
            expected_position=1.5,
            tolerance=0.5,
        )

    def test_position_control_negative_infinite_position(self):
        self._check_position_control(
            position=-float('inf'),
            effort=1,
            expected_position=-1.5,
            tolerance=0.5,
        )

    def test_position_control_nan_position(self):
        # Sending NaN will move the servos to the minimum position
        self._check_position_control(
            position=float('nan'),
            effort=1,
            expected_position=-1.5,
            tolerance=0.5,
        )

    def _check_position_control(self, position, effort, expected_position=None, tolerance=0.05):
        target_point = JointTrajectoryPoint()
        target_point.time_from_start = rclpy.time.Duration(seconds=0.02).to_msg()
        target_point.positions = [position] * len(self.joint_names)
        target_point.effort = [effort] * len(self.joint_names)

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = self.joint_names
        trajectory_goal.trajectory.points = [target_point]
        trajectory_goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'test_frame'

        goal_handle_future = self.trajectory_client.send_goal_async(trajectory_goal)
        rclpy.spin_until_future_complete(self.node, goal_handle_future)
        goal_handle = goal_handle_future.result()
        self.assertIsNotNone(goal_handle)
        if goal_handle is None:
            self.fail('Goal handle is None')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result: FollowJointTrajectory.Result | None = result_future.result().result

        self.assertIsNotNone(result)
        if result is None:
            self.fail('Result is None')
            return
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        # Wait for the feedback to be updated
        result_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=0.05)
        timeout = result_time + rclpy.time.Duration(seconds=3)

        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.last_feedback > result_time:
                joint_positions = self.joint_positions
                break
            self.assertLess(self.node.get_clock().now(), timeout)

        self.assertTrue(
            np.all(np.isfinite(joint_positions)),
            msg=f'Actual positions are not finite: {joint_positions}',
        )

        # # TODO(anton-matosov): Use dynamic_joint_state to check the actual position
        if expected_position is not None:
            self.assertTrue(
                np.allclose(
                    np.array(joint_positions),
                    np.array([expected_position] * len(self.joint_names)),
                    atol=tolerance,
                ),
                msg=f'Requested position {position} with effort {effort},'
                f' Expected position {expected_position},'
                f' got {joint_positions},',
            )


# Post-shutdown tests
@post_shutdown_test()
class TestA116HardwareInterfaceShutdown(unittest.TestCase):
    """Test the a1_16_hardware_interface shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        asserts.assertExitCodes(proc_info)
