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
Characterise the balance board fixture in isolation, with no robot.

This is the first of three layered balance-board tests. It verifies the tilting
mechanism alone reaches every commanded roll/pitch angle, so that any failure in
the higher-level tests (robot riding the board, robot disturbing the board) can
be attributed to the robot or controller rather than the world fixture itself.
"""

import math

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
import launch_pytest
import pytest
from robot_control_test_support import (
    BalanceBoardWorldBase,
    create_board_only_launch_description,
)

_TILT_MAGNITUDE = 0.12
_TILT_DIAGONAL = _TILT_MAGNITUDE / math.sqrt(2)

_TILT_SCENARIOS = [
    (0.0, +_TILT_MAGNITUDE),  # pitch+
    (+_TILT_DIAGONAL, +_TILT_DIAGONAL),  # roll+ pitch+
    (+_TILT_MAGNITUDE, 0.0),  # roll+
    (+_TILT_DIAGONAL, -_TILT_DIAGONAL),  # roll+ pitch-
    (0.0, -_TILT_MAGNITUDE),  # pitch-
    (-_TILT_DIAGONAL, -_TILT_DIAGONAL),  # roll- pitch-
    (-_TILT_MAGNITUDE, 0.0),  # roll-
    (-_TILT_DIAGONAL, +_TILT_DIAGONAL),  # roll- pitch+
]


@launch_pytest.fixture
def generate_test_description():
    """Launch only Gazebo and the board world (no robot) and record exit codes."""
    launch_description = create_board_only_launch_description()
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.fixture
def board(generate_test_description):  # noqa: ARG001 (drives the launch)
    """
    Provide a board-only harness bound to the launched world.

    The board-only launch starts no robot, ROS bridge, or control stack, and the
    board helpers talk to Gazebo over the ``gz`` CLI, so no ``rclpy`` node is
    needed; the fixture only waits for the board world to start serving poses.
    """
    harness = BalanceBoardWorldBase()
    harness.wait_for_board_world_ready()
    yield harness


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_board_reaches_all_tilt_angles(board, generate_test_description):
    for board_roll, board_pitch in _TILT_SCENARIOS:
        board._assert_board_reaches_tilt(board_roll, board_pitch)

    # Function-scoped generator: the board world tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly. The
    # board-only launch starts only Gazebo, which is filtered out, so the check
    # passes trivially when nothing else ran.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
