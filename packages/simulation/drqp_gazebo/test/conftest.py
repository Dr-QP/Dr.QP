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
    """Configure slow-test selection and retries for isolated launch tests."""
    run_slow_tests = os.environ.get('DRQP_TEST_MODE') == 'slow'
    skip_slow = pytest.mark.skip(reason='Slow test excluded from default CI run')
    for item in items:
        # Every launch fixture in this package is function-scoped, so pytest-retry
        # can relaunch it after transient discovery or shutdown failures.
        if 'launch' in item.keywords and 'flaky' not in item.keywords:
            item.add_marker(pytest.mark.flaky(retries=3))
        if not run_slow_tests and 'slow' in item.keywords:
            item.add_marker(skip_slow)


@pytest.hookimpl(tryfirst=True)
def pytest_runtest_call(item: pytest.Item) -> None:
    """Surface harness setup failures where pytest-retry can relaunch them."""
    for fixture_name in ('robot', 'board'):
        harness = item.funcargs.get(fixture_name)
        setup_error = getattr(harness, 'setup_error', None)
        if setup_error is not None:
            raise setup_error


@pytest.fixture
def robot(generate_test_description):  # noqa: ARG001 (drives launch + sim readiness)
    """
    Provide a harness instance bound to the launched simulation.

    Function-scoped by default: single-test files launch one simulation for their
    one test. Files that must share a simulation across several tests override
    this with a ``scope='module'`` ``robot`` fixture. Owns ``rclpy`` init/shutdown and waits
    for simulation readiness before yielding the harness.

    Asserts no "MoveIt IK failed"/"IK rejected" warnings were logged by drqp_brain
    during the test, so every simulation test catches this failure mode
    automatically rather than only the tests written specifically to provoke it.
    """
    rclpy.init()
    harness = GazeboRobotControlBase()
    harness.setup_error = None
    try:
        try:
            harness.setup_node()
        except Exception as error:  # noqa: BLE001 - re-raised by pytest_runtest_call
            # pytest-retry cannot retry fixture-setup failures. Preserve the
            # original exception and surface it in the call phase instead.
            harness.setup_error = error
        yield harness
        if harness.setup_error is None:
            harness.assert_no_moveit_ik_failures()
    finally:
        if hasattr(harness, 'node') and rclpy.ok():
            harness.node.destroy_node()
        rclpy.try_shutdown()
