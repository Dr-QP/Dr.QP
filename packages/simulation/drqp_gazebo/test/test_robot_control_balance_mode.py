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

import math

from launch_testing import post_shutdown_test
import pytest
from robot_control_test_support import (
    create_balance_board_launch_description,
    GazeboRobotControlBase,
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
    return create_balance_board_launch_description()


@pytest.mark.slow
class TestGazeboRobotControlBalanceMode(GazeboRobotControlBase):
    """Verify balance mode keeps the robot body level across all tilt directions."""

    def test_balance_mode_levels_body_on_all_tilt_angles(self):
        self._arm_robot()
        initial_roll, initial_pitch = self._sample_base_roll_pitch(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )
        self._set_balance_mode(True)

        for board_roll, board_pitch in _TILT_SCENARIOS:
            self._assert_body_level_at_board_tilt(
                board_roll, board_pitch, initial_roll, initial_pitch
            )
            self._reset_board_and_balance_mode(initial_roll, initial_pitch)


@post_shutdown_test()
class TestSimulationShutdown(SimulationShutdownBase):
    """Verify processes exit cleanly after the launch test finishes."""

    def test_exit_codes(self, proc_info):
        self.assert_exit_codes(proc_info)
