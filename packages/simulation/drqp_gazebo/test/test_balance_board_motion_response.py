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

"""Verify the level board holds its alignment while the robot moves on it.

This is the third of three layered balance-board tests. The board is left level
and held there by its joint position controllers; the robot then steps in place
and shifts its body forward. It checks that the robot's own motion does not
back-drive the board off level, validating that the lightened board responds
gently rather than being disturbed by foot contacts.
"""

from launch_testing import post_shutdown_test
import pytest
from robot_control_test_support import (
    create_balance_board_launch_description,
    GazeboRobotControlBase,
)
from simulation_shutdown_test_case import SimulationShutdownBase


@pytest.mark.slow
@pytest.mark.launch_test
def generate_test_description():
    return create_balance_board_launch_description()


@pytest.mark.slow
class TestBalanceBoardMotionResponse(GazeboRobotControlBase):
    """Verify robot motion on a level board does not tilt the board off level."""

    def test_board_holds_level_during_robot_motion(self):
        self._arm_robot()
        self._assert_board_stays_level('before robot motion')

        # Step on the spot: stride_direction.z drives the gait with no heading.
        self._publish_movement_command(stride_z=1.0)
        self._wait_for_sim_time(self.MOTION_RESPONSE_DURATION)
        self._assert_board_stays_level('while stepping in place')
        self._publish_movement_command()  # stop stepping

        # Shift the body forward over the stance.
        self._publish_movement_command(body_translation_x=1.0)
        self._wait_for_sim_time(self.MOTION_RESPONSE_DURATION)
        self._assert_board_stays_level('while moving body forward')
        self._publish_movement_command()  # stop


@post_shutdown_test()
class TestSimulationShutdown(SimulationShutdownBase):
    """Verify processes exit cleanly after the launch test finishes."""

    def test_exit_codes(self, proc_info):
        self.assert_exit_codes(proc_info)
