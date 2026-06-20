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

"""Launch test for the drqp_robot_state node."""

from pathlib import Path
import time

from drqp_brain.instance_guard import InstanceGuard
from drqp_brain.robot_state import robot_state_node
from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import std_msgs.msg


def test_robot_state_main_refuses_duplicate_instance(monkeypatch):
    """Refuse startup when another robot state node owns the instance lock."""
    monkeypatch.setenv('ROS_HOME', str(Path.cwd() / '.tmp' / 'ros_home'))

    with InstanceGuard('drqp_robot_state'):
        assert robot_state_node.main() == 1


@launch_pytest.fixture
def generate_test_description():
    """Launch the drqp_robot_state node and record process exit codes."""
    launch_description = LaunchDescription(
        [
            Node(
                executable=FindExecutable(name='python3'),
                arguments=[
                    '-m',
                    'coverage',
                    'run',
                    ExecutableInPackage(package='drqp_brain', executable='drqp_robot_state'),
                ],
                output='screen',
            ),
            # Launch tests 0.5 s later
            TimerAction(period=0.5, actions=[ReadyToTest()]),
        ]
    )
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.fixture
def consumer(generate_test_description):  # noqa: ARG001 (drives the launch)
    """Own rclpy init/shutdown and provide a consumer node for the launched node."""
    rclpy.init()
    node = rclpy.create_node('test_state_consumer')
    yield node
    rclpy.try_shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_processes_events_and_publishes_state(consumer, generate_test_description):
    """Check whether events are processed, then verify a clean shutdown."""
    msgs_received = []

    event_pub = consumer.create_publisher(std_msgs.msg.String, '/robot_event', qos_profile=10)
    qos_profile = QoSProfile(depth=1)
    # make state available to late joiners
    qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    robot_state_sub = consumer.create_subscription(
        std_msgs.msg.String, '/robot_state', lambda msg: msgs_received.append(msg), qos_profile
    )

    try:
        end_time = time.time() + 5

        while time.time() < end_time:
            rclpy.spin_once(consumer, timeout_sec=0.01)
            event_pub.publish(std_msgs.msg.String(data='initialize'))

            if len(msgs_received) > 0 and msgs_received[-1].data == 'initializing':
                break

        assert len(msgs_received) > 0
        assert msgs_received[-1].data == 'initializing'
    finally:
        consumer.destroy_subscription(robot_state_sub)
        consumer.destroy_publisher(event_pub)

    # Function-scoped generator: the launched node is torn down at the yield,
    # then the post-yield body verifies it exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
