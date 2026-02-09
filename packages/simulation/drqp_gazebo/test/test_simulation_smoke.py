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

import subprocess
import unittest

from controller_manager.test_utils import (
    check_controllers_running,
    check_node_running,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ProcInfoHandler
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rosgraph_msgs.msg import Clock


def ensure_gz_sim_not_running():
    # Kill any remaining Gazebo processes
    # See https://github.com/ros2/launch/issues/545 for details
    shell_cmd = ['pkill', '-9', '-f', '^gz sim']
    subprocess.run(shell_cmd, check=False)


@pytest.mark.launch_test
def generate_test_description():
    ensure_gz_sim_not_running()

    simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare('drqp_gazebo'),
            'launch',
            'sim.launch.py',
        ]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={'gui': 'false'}.items(),
            ),
            TimerAction(period=5.0, actions=[ReadyToTest()]),
        ]
    )


class TestSimulationWiring(unittest.TestCase):
    """Ensure Gazebo simulation brings up the expected ROS graph."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_gazebo_sim_smoke')

    def tearDown(self):
        self.node.destroy_node()

    def test_nodes_and_clock(self):
        """Ensure ROS nodes from the bringup come online and Gazebo time is bridged."""
        for node_name in ('robot_state_publisher', 'drqp_brain', 'drqp_robot_state'):
            check_node_running(self.node, node_name, timeout=20.0)
        self._wait_for_clock()

    def test_controllers_are_active(self):
        """Verify ros2_control controllers are alive inside Gazebo."""
        check_controllers_running(
            self.node,
            [
                'joint_state_broadcaster',
                'joint_trajectory_controller',
            ],
            timeout=20.0,
        )

    def _wait_for_clock(self, timeout_sec=20.0):
        with WaitForTopics([('/clock', Clock)], timeout=timeout_sec) as wait_for_topics:
            self.assertTrue(wait_for_topics.wait(), 'Did not receive /clock from Gazebo bridge')


@post_shutdown_test()
class TestSimulationShutdown(unittest.TestCase):
    """Verify processes exit cleanly after the launch test finishes."""

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()
        ensure_gz_sim_not_running()

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally (except Gazebo)."""
        # Gazebo is SIGTERMed by the launch file, so we need to filter it out
        proc_info = self._filter_out_gazebo(proc_info)
        asserts.assertExitCodes(proc_info)

    def _filter_out_gazebo(self, proc_info):
        """Filter out Gazebo processes from the list."""
        filtered_proc_info = ProcInfoHandler()
        skipped_procs = ('gazebo', 'gz', 'bridge_node')
        for proc_name in proc_info.process_names():
            if not any(skip in proc_name for skip in skipped_procs):
                filtered_proc_info.append(proc_info[proc_name])
        return filtered_proc_info
