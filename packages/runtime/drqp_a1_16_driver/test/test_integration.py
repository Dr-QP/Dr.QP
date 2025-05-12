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
import os
import sys
import time
import unittest

import rclpy
from turtlesim.msg import Pose

import launch
import launch_ros
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
from launch_testing.actions import ReadyToTest


# def generate_test_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')


#     return LaunchDescription(
#         [
#             DeclareLaunchArgument(
#                 name='use_sim_time',
#                 default_value='false',
#                 choices=['true', 'false'],
#                 description='Use sim time if true',
#             ),
#             DeclareLaunchArgument(
#                 name='load_drivers',
#                 default_value='true',
#                 choices=['true', 'false'],
#                 description='Load drqp_a1_16_driver pose_setter',
#             ),
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     [
#                         'launch',
#                         '/pose_setter.launch.py',
#                     ]
#                 ),
#                 launch_arguments={'use_sim_time': use_sim_time}.items(),
#             ),
#             # Launch tests 0.5 s later
#             TimerAction(period=0.5, actions=[ReadyToTest()]),
#         ]
#     )
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                # Can also reference here to a launch file in `app/launch`
                # to avoid duplication
                # Just a simple node here, yet can launch anything, e.g. Gazebo
                launch_ros.actions.Node(
                    package='turtlesim',
                    namespace='',
                    executable='turtlesim_node',
                    name='turtle1',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        # context
        {},
    )


# Active tests
class TestTurtleSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_turtlesim')

    def tearDown(self):
        self.node.destroy_node()

    def test_logs_spawning(self, proc_output):
        """Check whether logging properly."""
        proc_output.assertWaitFor('Spawning turtle [turtle1] at x=', timeout=15, stream='stderr')

    def test_publishes_pose(self, proc_output):
        """Check whether pose messages published."""
        msgs_rx = []
        sub = self.node.create_subscription(
            Pose, 'turtle1/pose', lambda msg: msgs_rx.append(msg), 10
        )
        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if len(msgs_rx) > 30:
                    break
            self.assertGreater(len(msgs_rx), 30)
        finally:
            self.node.destroy_subscription(sub)


# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestTurtleSimShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
