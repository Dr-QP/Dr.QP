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

"""Verify balance mode keeps the robot body level across board tilt directions."""

import math

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
import launch_pytest
import pytest
from robot_control_test_support import create_balance_board_launch_description

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
    """Launch Gazebo with the robot riding the balance board and record exit codes."""
    launch_description = create_balance_board_launch_description()
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_balance_mode_levels_body_on_all_tilt_angles(robot, generate_test_description):
    robot._arm_robot()
    initial_roll, initial_pitch = robot._sample_base_roll_pitch(
        settle_sim_time_sec=robot.POSE_SETTLE_DURATION
    )
    robot._set_balance_mode(True)

    for board_roll, board_pitch in _TILT_SCENARIOS:
        robot._assert_body_level_at_board_tilt(
            board_roll, board_pitch, initial_roll, initial_pitch
        )
        robot._reset_board_and_balance_mode(initial_roll, initial_pitch)

    # Function-scoped generator: the simulation tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
