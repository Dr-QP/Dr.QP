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
from unittest import mock

from control_msgs.action import FollowJointTrajectory
from drqp_brain.brain_node import HexapodBrain
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Vector3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
import std_msgs.msg


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        [
            Node(
                executable=FindExecutable(name='python3'),
                arguments=[
                    '-m',
                    'coverage',
                    'run',
                    ExecutableInPackage(package='drqp_brain', executable='drqp_brain'),
                ],
                output='screen',
            ),
            # Launch tests 3s later
            TimerAction(period=3.0, actions=[ReadyToTest()]),
        ]
    )


class TestBrainNode(unittest.TestCase):
    """Test the drqp_brain node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_brain_consumer')

        # Publishers for sending commands to brain node
        self.movement_pub = self.node.create_publisher(
            MovementCommand, '/robot/movement_command', 10
        )

        # Publisher for robot state (brain node subscribes to this)
        self.state_pub = self.node.create_publisher(std_msgs.msg.String, '/robot_state', 10)

    def tearDown(self):
        self.node.destroy_node()

    def test_nothing(self, proc_output):
        """Smoke check."""
        pass

    def test_movement_command_processing(self, proc_output):
        """Test that brain node can process movement commands."""
        # Create a movement command
        cmd = MovementCommand()
        cmd.stride_direction = Vector3(x=1.0, y=0.0, z=0.0)
        cmd.rotation_speed = 0.5
        cmd.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD

        # Publish the command
        self.movement_pub.publish(cmd)

        # Spin to allow processing
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # If we get here without errors, the test passes
        # (brain node should process the command without crashing)

    def test_trajectory_action_client_is_created_lazily(self, proc_output):
        """Only create the action client when an action sequence is requested."""
        with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
            action_client = action_client_cls.return_value
            action_client.send_goal_async.return_value.add_done_callback = mock.Mock()

            brain = HexapodBrain()
            try:
                action_client_cls.assert_not_called()

                brain.reboot_servos()

                action_client_cls.assert_called_once_with(
                    brain,
                    FollowJointTrajectory,
                    '/joint_trajectory_controller/follow_joint_trajectory',
                )
            finally:
                brain.destroy_node()

    def test_destroy_node_destroys_action_client(self, proc_output):
        """Destroy the action client explicitly during node shutdown."""
        with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
            action_client = action_client_cls.return_value
            action_client.send_goal_async.return_value.add_done_callback = mock.Mock()

            brain = HexapodBrain()
            brain.reboot_servos()
            brain.destroy_node()

            action_client.destroy.assert_called_once_with()


# Post-shutdown tests
@post_shutdown_test()
class TestBrainNodeShutdown(unittest.TestCase):
    """Test the drqp_brain node shutdown."""

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        asserts.assertExitCodes(proc_info)
