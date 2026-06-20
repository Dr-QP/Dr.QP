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
Verify the level board holds its alignment while the robot moves on it.

This is the third of three layered balance-board tests. The board is left level
and held there by its joint position controllers; the robot then steps in place
and shifts its body forward. It checks that the robot's own motion does not
back-drive the board off level, validating that the lightened board responds
gently rather than being disturbed by foot contacts.
"""

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
import launch_pytest
import pytest
from robot_control_test_support import create_balance_board_launch_description


@launch_pytest.fixture
def generate_test_description():
    """Launch Gazebo with the robot riding the balance board and record exit codes."""
    launch_description = create_balance_board_launch_description()
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_board_holds_level_during_robot_motion(robot, generate_test_description):
    robot._arm_robot()
    robot._assert_board_stays_level('before robot motion')

    # Step on the spot: stride_direction.z drives the gait with no heading.
    robot._publish_movement_command(stride_z=1.0)
    robot._wait_for_sim_time(robot.MOTION_RESPONSE_DURATION)
    robot._assert_board_stays_level('while stepping in place')
    robot._publish_movement_command()  # stop stepping

    # Shift the body forward over the stance.
    robot._publish_movement_command(body_translation_x=1.0)
    robot._wait_for_sim_time(robot.MOTION_RESPONSE_DURATION)
    robot._assert_board_stays_level('while moving body forward')
    robot._publish_movement_command()  # stop

    # Function-scoped generator: the simulation tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
