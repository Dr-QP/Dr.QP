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

import time
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
import std_msgs.msg


@pytest.mark.launch_test
def generate_test_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use sim time if true',
            ),
            Node(
                package='drqp_brain',
                executable='drqp_robot_state',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            # Launch tests 0.5 s later
            TimerAction(period=0.5, actions=[ReadyToTest()]),
        ]
    )


class TestRobotStateMachineNode(unittest.TestCase):
    """Test the drqp_robot_state node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_state_consumer')

    def tearDown(self):
        self.node.destroy_node()

    def test_can_accept_events(self, proc_output):
        """Check whether events are accepted."""
        pass

    def test_publishes_state(self, proc_output):
        """Check whether state is published."""
        msgs_received = []
        self.robot_state_sub = self.node.create_subscription(
            std_msgs.msg.String, '/robot_state', lambda msg: msgs_received.append(msg), 10
        )

        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.01)
                if len(msgs_received) > 0:
                    break
            self.assertGreater(len(msgs_received), 0)
            self.assertEqual(msgs_received[0].data, 'torque_off')
        finally:
            self.node.destroy_subscription(self.robot_state_sub)


# Post-shutdown tests
@post_shutdown_test()
class TestRobotStateMachineNodeShutdown(unittest.TestCase):
    """Test the drqp_robot_state node shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        asserts.assertExitCodes(proc_info)
