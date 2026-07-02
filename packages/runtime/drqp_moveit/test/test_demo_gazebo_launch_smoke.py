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

"""Smoke test that the MoveIt + Gazebo demo launch reaches a ready state."""

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
import launch_pytest
from moveit_launch_smoke_test_support import (
    assert_move_group_ready,
    build_smoke_test_description,
    build_test_gz_partition,
)
import pytest


@launch_pytest.fixture
def generate_test_description():
    """Launch the MoveIt+Gazebo demo and record process exit codes."""
    launch_description = build_smoke_test_description(
        'demo_gazebo.launch.py',
        launch_arguments={
            'show_rviz': 'false',
            'sim_gui': 'false',
            'gz_partition': build_test_gz_partition('demo_gazebo_launch_smoke'),
        },
        ready_delay=5.0,
    )
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.mark.flaky(retries=3)
@pytest.mark.launch(fixture=generate_test_description)
def test_launch_reaches_ready_state(move_group, generate_test_description):  # noqa: ARG001
    # Retries via pytest-retry: the post-yield shutdown exit-code check below is
    # intermittently flaky (see issue #408). assert_move_group_ready() failures
    # are genuine regressions and reproduce identically on every retry, so they
    # still fail the build.
    assert_move_group_ready()
    # Function-scoped generator: the stack tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
