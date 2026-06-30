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

"""Launch and unit tests for the drqp_brain node."""

from unittest import mock

from control_msgs.action import FollowJointTrajectory
from drqp_brain.brain_node import _assert_no_existing_brain_node, HexapodBrain
from drqp_brain.instance_guard import InstanceAlreadyRunningError
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
from geometry_msgs.msg import Vector3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
import pytest
import rclpy


def test_existing_brain_node_detection_rejects_duplicate_ros_node():
    """Refuse startup when the ROS graph already contains drqp_brain."""
    node = mock.Mock()
    node.get_node_names_and_namespaces.return_value = [
        ('robot_state_publisher', '/'),
        ('drqp_brain', '/'),
    ]

    with pytest.raises(InstanceAlreadyRunningError, match='/drqp_brain'):
        _assert_no_existing_brain_node(node)


def test_trajectory_action_client_is_created_lazily(rclpy_context):  # noqa: ARG001 (needs rclpy)
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


def test_destroy_node_destroys_action_client(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Destroy the action client explicitly during node shutdown."""
    with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
        action_client = action_client_cls.return_value
        action_client.send_goal_async.return_value.add_done_callback = mock.Mock()

        brain = HexapodBrain()
        brain.reboot_servos()
        brain.destroy_node()

        action_client.destroy.assert_called_once_with()


@launch_pytest.fixture
def generate_test_description():
    """Launch the drqp_brain node and record process exit codes."""
    launch_description = LaunchDescription(
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
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.fixture
def consumer(generate_test_description):  # noqa: ARG001 (drives the launch)
    """Own rclpy init/shutdown and provide a consumer node for the launched node."""
    rclpy.init()
    node = rclpy.create_node('test_brain_consumer')
    yield node
    rclpy.try_shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_movement_command_processing(consumer, generate_test_description):
    """Process a movement command, then verify the launched node exits cleanly."""
    cmd = MovementCommand()
    cmd.stride_direction = Vector3(x=1.0, y=0.0, z=0.0)
    cmd.rotation_speed = 0.5
    cmd.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
    cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
    cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD

    movement_pub = consumer.create_publisher(MovementCommand, '/robot/movement_command', 10)
    movement_pub.publish(cmd)

    # Spin to allow processing; if we get here without errors the brain node
    # processed the command without crashing.
    rclpy.spin_once(consumer, timeout_sec=0.1)

    # Function-scoped generator: the launched node is torn down at the yield,
    # then the post-yield body verifies it exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
