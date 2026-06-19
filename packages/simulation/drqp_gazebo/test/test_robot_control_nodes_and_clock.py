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

"""
Verify simulation nodes are running, the clock is bridged, and processes exit cleanly.

Structured as a functions-only launch_pytest module: a module-scoped launch
fixture starts one simulation shared by every test, a module-scoped ``robot``
fixture exposes a single ``GazeboRobotControlBase`` harness instance, and a final
``shutdown=True`` test verifies per-process exit codes once the simulation has
been torn down.
"""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_pytest
from launch_testing import asserts
from launch_testing.proc_info_handler import ProcInfoHandler
import pytest
import rclpy
from robot_control_test_support import (
    create_simulation_launch_description,
    GazeboRobotControlBase,
)

# Processes the simulation launch file tears down with SIGTERM on shutdown; they
# routinely report a non-zero return code on exit, so they are excluded from the
# clean-exit assertion. Mirrors the pre-launch_pytest post-shutdown filter and is
# validated by the slow Gazebo/MoveIt CI suites that exercise process teardown.
SHUTDOWN_KILLED_PROCESSES = ('gazebo', 'gz', 'bridge_node', 'move_group')


@launch_pytest.fixture(scope='module')
def generate_test_description():
    """
    Launch one simulation for the module and record process exit codes.

    Returns the launch description together with a ``ProcInfoHandler`` populated
    by an ``OnProcessExit`` handler, so the final shutdown test can assert that
    every non-simulator process exited cleanly.
    """
    proc_info = ProcInfoHandler()
    launch_description = create_simulation_launch_description()
    launch_description.add_action(
        RegisterEventHandler(OnProcessExit(on_exit=lambda event, _ctx: proc_info.append(event)))
    )
    return launch_description, proc_info


@pytest.fixture(scope='module')
def robot(generate_test_description):  # noqa: ARG001 (drives launch + sim readiness)
    """Provide a single shared harness instance for all tests in the module."""
    rclpy.init()
    harness = GazeboRobotControlBase()
    harness.setup_node()
    yield harness
    rclpy.try_shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_nodes_and_clock(robot):
    robot.assert_nodes_and_clock()


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_are_active(robot):
    robot.assert_controllers_are_active()


@pytest.mark.launch(fixture=generate_test_description)
def test_imu_data_is_published(robot):
    robot.assert_imu_data()


@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_simulation_processes_exit_cleanly(generate_test_description):
    """
    Assert every non-simulator process exited cleanly after shutdown.

    launch_pytest's ``shutdown=True`` mechanism only checks the aggregate launch
    *service* return code, which is unaffected by managed processes exiting
    non-zero. This body runs after the simulation has fully shut down, so the
    recorded exit codes can be checked per process (with the simulator allowlist).
    """
    _launch_description, proc_info = generate_test_description
    filtered = ProcInfoHandler()
    for name in proc_info.process_names():
        if not any(skip in name for skip in SHUTDOWN_KILLED_PROCESSES):
            filtered.append(proc_info[name])
    asserts.assertExitCodes(filtered)
