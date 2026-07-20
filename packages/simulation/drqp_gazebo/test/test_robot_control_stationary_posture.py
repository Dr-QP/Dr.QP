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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Verify stationary posture behavior on a physical Gazebo balance board."""

from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
import launch_pytest
import pytest
from robot_control_test_support import create_balance_board_launch_description

_PURE_PITCH_TILT = 0.10
_REACHABLE_TWO_AXIS_TILT = 0.06


@launch_pytest.fixture
def generate_test_description():
    """Launch Gazebo with the robot riding the balance board and record exit codes."""
    launch_description = create_balance_board_launch_description()
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_stationary_posture_levels_body_on_pure_pitch(
    robot,
    generate_test_description,
):
    """Level the body against one pure-axis contact-physics disturbance."""
    robot._arm_robot()
    initial_roll, initial_pitch = robot._sample_base_roll_pitch(
        settle_sim_time_sec=robot.POSE_SETTLE_DURATION
    )
    robot._set_balance_mode(True)

    robot._assert_body_level_at_board_tilt(
        0.0,
        _PURE_PITCH_TILT,
        initial_roll,
        initial_pitch,
    )

    # This case requires Gazebo to prove the posture command works through board,
    # foot-contact, and controller physics rather than quaternion math alone.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_stationary_posture_ignores_stale_and_concurrent_motion(
    robot,
    generate_test_description,
):
    """Hold a reachable diagonal posture without replaying operator motion."""
    robot.assert_stationary_balance_ignores_movement(
        board_roll=_REACHABLE_TWO_AXIS_TILT,
        board_pitch=_REACHABLE_TWO_AXIS_TILT,
    )

    # This case requires Gazebo for the odometry and persistent-contact contract;
    # fast unit tests own axis/sign symmetry and analytic envelope coverage.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
