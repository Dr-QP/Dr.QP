import pytest

from moveit_launch_smoke_test_support import (
    MoveItLaunchSmokeShutdownTestCase,
    MoveItLaunchSmokeTestCase,
    build_smoke_test_description,
)


@pytest.mark.launch_test
def generate_test_description():
    return build_smoke_test_description('move_group.launch.py')


class TestMoveGroupLaunchSmoke(MoveItLaunchSmokeTestCase):
    pass


class TestMoveGroupLaunchShutdown(MoveItLaunchSmokeShutdownTestCase):
    pass
