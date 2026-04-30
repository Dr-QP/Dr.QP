import pytest

from moveit_launch_smoke_test_support import (
    MoveItLaunchSmokeShutdownTestCase,
    MoveItLaunchSmokeTestCase,
    build_smoke_test_description,
)


@pytest.mark.launch_test
def generate_test_description():
    return build_smoke_test_description(
        'demo.launch.py',
        launch_arguments={'show_rviz': 'false'},
    )


class TestDemoLaunchSmoke(MoveItLaunchSmokeTestCase):
    pass


class TestDemoLaunchShutdown(MoveItLaunchSmokeShutdownTestCase):
    pass
