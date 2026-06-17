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

"""Characterise the balance board fixture in isolation, with no robot.

This is the first of three layered balance-board tests. It verifies the tilting
mechanism alone reaches every commanded roll/pitch angle, so that any failure in
the higher-level tests (robot riding the board, robot disturbing the board) can
be attributed to the robot or controller rather than the world fixture itself.
"""

import math

from launch_testing import post_shutdown_test
import pytest
from robot_control_test_support import (
    BalanceBoardWorldBase,
    create_board_only_launch_description,
)
from simulation_shutdown_test_case import SimulationShutdownBase

_TILT_MAGNITUDE = 0.12
_TILT_DIAGONAL = _TILT_MAGNITUDE / math.sqrt(2)

_TILT_SCENARIOS = [
    (0.0, +_TILT_MAGNITUDE),              # pitch+
    (+_TILT_DIAGONAL, +_TILT_DIAGONAL),   # roll+ pitch+
    (+_TILT_MAGNITUDE, 0.0),              # roll+
    (+_TILT_DIAGONAL, -_TILT_DIAGONAL),   # roll+ pitch-
    (0.0, -_TILT_MAGNITUDE),              # pitch-
    (-_TILT_DIAGONAL, -_TILT_DIAGONAL),   # roll- pitch-
    (-_TILT_MAGNITUDE, 0.0),              # roll-
    (-_TILT_DIAGONAL, +_TILT_DIAGONAL),   # roll- pitch+
]


@pytest.mark.slow
@pytest.mark.launch_test
def generate_test_description():
    return create_board_only_launch_description()


@pytest.mark.slow
class TestBalanceBoardWorld(BalanceBoardWorldBase):
    """Verify the board fixture reaches every commanded tilt with no robot present."""

    def test_board_reaches_all_tilt_angles(self):
        for board_roll, board_pitch in _TILT_SCENARIOS:
            self._assert_board_reaches_tilt(board_roll, board_pitch)


@post_shutdown_test()
class TestSimulationShutdown(SimulationShutdownBase):
    """Verify processes exit cleanly after the launch test finishes."""

    def test_exit_codes(self, proc_info):
        # The board-only launch starts no robot or bridge processes; only Gazebo
        # runs and it is SIGTERMed on teardown. With nothing left after filtering
        # Gazebo out, there are no exit codes to assert.
        if not self._filter_out_gazebo(proc_info).process_names():
            return
        self.assert_exit_codes(proc_info)
