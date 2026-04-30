import subprocess

import pytest

from moveit_launch_smoke_test_support import (
    MoveItLaunchSmokeShutdownTestCase,
    MoveItLaunchSmokeTestCase,
    build_smoke_test_description,
)


def _ensure_gz_sim_not_running() -> None:
    subprocess.run(['pkill', '-9', '-f', '^gz sim'], check=False)


@pytest.mark.launch_test
def generate_test_description():
    _ensure_gz_sim_not_running()
    return build_smoke_test_description(
        'demo_gazebo.launch.py',
        launch_arguments={'show_rviz': 'false', 'sim_gui': 'false'},
        ready_delay=5.0,
    )


class TestDemoGazeboLaunchSmoke(MoveItLaunchSmokeTestCase):
    pass


class TestDemoGazeboLaunchShutdown(MoveItLaunchSmokeShutdownTestCase):
    pass
