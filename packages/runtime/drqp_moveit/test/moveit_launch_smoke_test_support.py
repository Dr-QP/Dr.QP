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

import os
import re

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_pytest.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare
from moveit_msgs.srv import GetMotionPlan
import rclpy


def build_test_gz_partition(test_name: str) -> str:
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    sanitized_name = re.sub(r'[^a-z0-9]+', '-', test_name.lower()).strip('-')
    if not sanitized_name:
        sanitized_name = 'test'
    return f'drqp-domain-{domain_id}-{sanitized_name}'


def build_smoke_test_description(
    launch_file_name: str,
    *,
    launch_arguments: dict[str, str] | None = None,
    ready_delay: float = 2.0,
) -> LaunchDescription:
    launch_path = PathJoinSubstitution(
        [FindPackageShare('drqp_moveit'), 'launch', launch_file_name]
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


class MoveItLaunchSmokeTestCase:
    __test__ = False

    READY_TIMEOUT = 60.0

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()

    @classmethod
    def teardown_class(cls) -> None:
        rclpy.try_shutdown()

    def test_launch_reaches_ready_state(self):
        node = rclpy.create_node('moveit_launch_smoke_test')
        motion_plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        try:
            assert motion_plan_client.wait_for_service(timeout_sec=self.READY_TIMEOUT), \
                '/plan_kinematic_path service is not available'
        finally:
            motion_plan_client.destroy()
            node.destroy_node()
