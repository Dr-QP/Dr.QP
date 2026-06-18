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

import launch_pytest
from moveit_launch_smoke_test_support import (
    build_smoke_test_description,
    MoveItLaunchSmokeTestCase,
)
import pytest
import rclpy
import rclpy.time

_SKIP_NO_DISPLAY = pytest.mark.skipif(
    not os.environ.get('DISPLAY'),
    reason='RViz smoke test requires a graphical display',
)


@launch_pytest.fixture
def generate_test_description():
    return build_smoke_test_description(
        'demo.launch.py',
        launch_arguments={'show_rviz': 'true'},
        ready_delay=5.0,
    )


@_SKIP_NO_DISPLAY
@pytest.mark.launch(fixture=generate_test_description)
class TestDemoLaunchRvizSmoke(MoveItLaunchSmokeTestCase):
    """Smoke test for the demo.launch.py RViz visualization."""

    __test__ = True

    def test_only_one_rviz_node_starts(self):
        node = rclpy.create_node('rviz_count_checker')
        try:
            deadline = node.get_clock().now() + rclpy.time.Duration(seconds=10.0)
            rviz_nodes: list[str] = []
            while node.get_clock().now() < deadline:
                rviz_nodes = [name for name in node.get_node_names() if 'rviz' in name]
                if rviz_nodes:
                    break
                rclpy.spin_once(node, timeout_sec=0.5)
            assert len(rviz_nodes) == 1, f'Expected exactly one RViz node, found: {rviz_nodes}'
        finally:
            node.destroy_node()


@_SKIP_NO_DISPLAY
@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_demo_launch_rviz_shutdown():
    pass
