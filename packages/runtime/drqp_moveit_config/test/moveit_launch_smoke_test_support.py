import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare


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


class MoveItLaunchSmokeTestCase(unittest.TestCase):
    def test_launch_reaches_ready_state(self, proc_info):
        del proc_info


@post_shutdown_test()
class MoveItLaunchSmokeShutdownTestCase(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        asserts.assertExitCodes(proc_info)
