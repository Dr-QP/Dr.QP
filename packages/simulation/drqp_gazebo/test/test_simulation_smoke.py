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

from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rosgraph_msgs.msg import Clock


@pytest.mark.launch_test
def generate_test_description():
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
    """Smoke test to make sure Gazebo simulation brings up the expected ROS graph."""

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
        self._wait_for_nodes({'robot_state_publisher', 'drqp_brain', 'drqp_robot_state'})
        self._wait_for_clock()

    def test_controllers_are_active(self):
        """Verify ros2_control controllers are alive inside Gazebo."""
        self._wait_for_controllers(
            [
                'joint_state_broadcaster',
                'joint_trajectory_controller',
            ]
        )

    def _wait_for_nodes(self, expected_names, timeout_sec=20.0):
        end_time = time.time() + timeout_sec
        names = set()
        while time.time() < end_time:
            names = {name.lstrip('/') for name, _ in self.node.get_node_names_and_namespaces()}
            if expected_names.issubset(names):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        missing_nodes = expected_names - names
        self.fail(f'Missing expected nodes: {", ".join(sorted(missing_nodes))}')

    def _wait_for_clock(self, timeout_sec=30.0):
        clock_msgs = []
        subscription = self.node.create_subscription(
            Clock,
            '/clock',
            lambda msg: clock_msgs.append(msg),
            10,
        )
        try:
            end_time = time.time() + timeout_sec
            while time.time() < end_time and not clock_msgs:
                rclpy.spin_once(self.node, timeout_sec=0.25)
            self.assertGreater(len(clock_msgs), 0, 'Did not receive /clock from Gazebo bridge')
        finally:
            self.node.destroy_subscription(subscription)

    def _wait_for_controllers(self, expected_controllers, timeout_sec=20.0):
        service_name = self._find_list_controllers_service(timeout_sec=timeout_sec / 2)
        client = self.node.create_client(ListControllers, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec / 2),
            'controller_manager service not available',
        )

        request = ListControllers.Request()
        end_time = time.time() + timeout_sec
        last_states = {}
        while time.time() < end_time:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            result = future.result()
            if result:
                last_states = {
                    controller.name: controller.state for controller in result.controller
                }
                if all(last_states.get(name) == 'active' for name in expected_controllers):
                    return
            time.sleep(0.5)

        self.fail(
            'Controllers not active: '
            + ', '.join(f'{name}={last_states.get(name)}' for name in expected_controllers)
        )

    def _find_list_controllers_service(self, timeout_sec):
        target_service_type = 'controller_manager_msgs/srv/ListControllers'
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            for name, service_types in self.node.get_service_names_and_types():
                if target_service_type in service_types:
                    return name
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('Could not find a ListControllers service from controller_manager')


@post_shutdown_test()
class TestSimulationShutdown(unittest.TestCase):
    """Verify processes exit cleanly after the launch test finishes."""

    def test_exit_codes(self, proc_info):
        asserts.assertExitCodes(proc_info)
