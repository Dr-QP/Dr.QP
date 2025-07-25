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
import time
import unittest

from drqp_interfaces.msg import (
    MultiServoPositionGoal,
    MultiServoState,
    ServoPositionGoal,
    TorqueOn,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import rclpy

recording = False


def generate_test_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    device_address_name = '/dev/ttySC0' if recording else 'playback'
    test_data_dir = Path(__file__).parent / 'test_data'
    pose_setter_device = f'{device_address_name}|{test_data_dir / "integration-pose-setter.json"}'
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use sim time if true',
            ),
            Node(
                package='drqp_control',
                executable='pose_setter',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'device_address': pose_setter_device,
                    }
                ],
            ),
            # Launch tests 0.5 s later
            TimerAction(period=0.5, actions=[ReadyToTest()]),
        ]
    )


class TestServoDriverNodes(unittest.TestCase):
    """Test the pose_setter node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_servo_driver')
        self.run_duration = 20
        self.max_messages = 10

    def tearDown(self):
        self.node.destroy_node()

    def test_pose_setter_subscribes_servo_goals(self, proc_output):
        """Check whether servo_goals messages are read by pose_setter node."""
        msgs_received: list[MultiServoState] = []
        pub_goals = self.node.create_publisher(MultiServoPositionGoal, '/servo_goals', 10)
        sub_states = self.node.create_subscription(
            MultiServoState, '/servo_states', lambda msg: msgs_received.append(msg), 10
        )
        pub_torque_on = self.node.create_publisher(TorqueOn, '/servo_torque_on', 10)

        joints = [
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
        servo_count = len(joints)
        position_as_radians = 0.1
        playtime_ms = 1500
        try:
            end_time = time.time() + self.run_duration
            while time.time() < end_time:
                torque_on = TorqueOn()
                torque_on.joint_names = joints
                torque_on.torque_on = [True] * servo_count
                pub_torque_on.publish(torque_on)

                test_goal = MultiServoPositionGoal()
                test_goal.header.stamp = self.node.get_clock().now().to_msg()
                test_goal.header.frame_id = 'test_frame'
                test_goal.mode = MultiServoPositionGoal.MODE_SYNC
                test_goal.goals = [
                    ServoPositionGoal(
                        joint_name=joint,
                        position_as_radians=position_as_radians,
                        playtime_ms=playtime_ms,
                    )
                    for joint in joints
                ]

                pub_goals.publish(test_goal)

                for _ in range(10):
                    rclpy.spin_once(self.node, timeout_sec=0.01)
                if len(msgs_received) > self.max_messages:
                    break
            self.assertGreater(len(msgs_received), self.max_messages)

            for msg in msgs_received:
                self.assertEqual(len(msg.servos), servo_count)
                joints_to_check = list(joints)
                for servo in msg.servos:
                    self.assertAlmostEqual(
                        servo.position_as_radians,
                        position_as_radians,
                        delta=0.01,
                    )
                    joints_to_check.remove(servo.joint_name)
                self.assertListEqual(joints_to_check, [], 'Not all servos were received')
        finally:
            self.node.destroy_subscription(sub_states)
            self.node.destroy_publisher(pub_goals)


# Post-shutdown tests
@post_shutdown_test()
class TestPoseSetterShutdown(unittest.TestCase):
    """Test the pose_setter node shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        asserts.assertExitCodes(proc_info)
