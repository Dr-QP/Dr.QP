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

import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
from moveit_msgs.srv import GetMotionPlan
import rclpy


def build_smoke_test_description(
    launch_file_name: str,
    *,
    launch_arguments: dict[str, str] | None = None,
    ready_delay: float = 2.0,
) -> LaunchDescription:
    launch_path = PathJoinSubstitution(
        [FindPackageShare('drqp_moveit_config'), 'launch', launch_file_name]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments=(launch_arguments or {}).items(),
            ),
            TimerAction(period=ready_delay, actions=[ReadyToTest()]),
        ]
    )


def _filter_shutdown_processes(proc_info):
    filtered_proc_info = type(proc_info)()
    skipped_procs = ('gazebo', 'gz', 'bridge_node', 'move_group')
    for proc_name in proc_info.process_names():
        if not any(skip in proc_name for skip in skipped_procs):
            filtered_proc_info.append(proc_info[proc_name])
    return filtered_proc_info


class MoveItLaunchSmokeTestCase(unittest.TestCase):
    __test__ = False

    READY_TIMEOUT = 60.0

    def setUp(self) -> None:
        rclpy.init()
        self.addCleanup(rclpy.shutdown)

    def test_launch_reaches_ready_state(self, proc_info):
        del proc_info

        node = rclpy.create_node('moveit_launch_smoke_test')
        self.addCleanup(node.destroy_node)
        motion_plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.addCleanup(motion_plan_client.destroy)
        self.assertTrue(
            motion_plan_client.wait_for_service(timeout_sec=self.READY_TIMEOUT),
            '/plan_kinematic_path service is not available',
        )


@post_shutdown_test()
class MoveItLaunchSmokeShutdownTestCase(unittest.TestCase):
    """Verify the launch exits cleanly after smoke tests complete."""

    __test__ = False

    def test_exit_codes(self, proc_info):
        asserts.assertExitCodes(_filter_shutdown_processes(proc_info))
