# Copyright (c) 2026 Anton Matosov
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

"""Smoke test that the move_group launch reaches a ready state."""

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare
from moveit_launch_smoke_test_support import assert_move_group_ready
import pytest


@launch_pytest.fixture
def generate_test_description():
    """Launch move_group + bringup and record process exit codes."""
    move_group_launch = PathJoinSubstitution(
        [FindPackageShare('drqp_moveit'), 'launch', 'move_group.launch.py']
    )
    bringup_launch = PathJoinSubstitution(
        [FindPackageShare('drqp_brain'), 'launch', 'bringup.launch.py']
    )

    launch_description = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                launch_arguments={'hardware_device_address': 'mock_servo'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    'use_gazebo': 'false',
                    'load_joystick': 'false',
                    'load_controllers': 'true',
                    'load_imu': 'false',
                    'hardware_device_address': 'mock_servo',
                }.items(),
            ),
            TimerAction(period=2.0, actions=[ReadyToTest()]),
        ]
    )
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.mark.launch(fixture=generate_test_description)
def test_launch_reaches_ready_state(move_group, generate_test_description):  # noqa: ARG001
    assert_move_group_ready()
    # Function-scoped generator: the stack tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
