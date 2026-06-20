# Copyright (c) 2017-2026 Anton Matosov
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

import pytest
import rclpy
from robot_control_test_support import GazeboRobotControlBase


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip slow tests unless DRQP_TEST_MODE=slow is set."""
    if os.environ.get('DRQP_TEST_MODE') == 'slow':
        return

    skip_slow = pytest.mark.skip(reason='Slow test excluded from default CI run')
    for item in items:
        if 'slow' in item.keywords:
            item.add_marker(skip_slow)


@pytest.fixture
def robot(generate_test_description):  # noqa: ARG001 (drives launch + sim readiness)
    """
    Provide a harness instance bound to the launched simulation.

    Function-scoped by default: single-test files launch one simulation for their
    one test. Files that must share a simulation across several tests override
    this with a ``scope='module'`` ``robot`` fixture (see
    test_robot_control_nodes_and_clock.py). Owns ``rclpy`` init/shutdown and waits
    for simulation readiness before yielding the harness.
    """
    rclpy.init()
    harness = GazeboRobotControlBase()
    harness.setup_node()
    yield harness
    rclpy.try_shutdown()
