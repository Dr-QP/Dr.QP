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

# Analytic reachability envelope of the balance hold, swept from the drqp_kinematics
# IK model using this controller's stance (forward_kinematics(0, -35, 130)), the
# URDF joint limits, and the rotation apply_imu_balance commands at gain 2.0. The
# largest tilt for which all six legs still solve:
#
#   direction     |tilt|   per axis   first blocked leg
#   roll only     0.0551   0.0551     left_middle   (right_middle for -roll)
#   pitch only    0.0546   0.0546     left_back, right_back  (front pair for -pitch)
#   diagonal      0.0471   0.0333     left_back     (the corner, for +roll +pitch)
#
# One foot-travel limit of about 21mm sets all three. Roll lifts the side legs and
# pitch lifts the back pair; on a diagonal the corner leg between them takes both
# displacements, so its per-axis budget is 60% of the single-axis budget.
#
# Staying inside the envelope is necessary but not sufficient. At 0.025 rad per
# axis the hold still fails, and it fails the other way: the body reaches roughly
# three times the board tilt instead of falling short of it, so the loop is
# diverging rather than running out of reach. See Dr-QP/Dr.QP#453.
_PURE_PITCH_TILT = 0.10  # past the bound on purpose; symmetric binding converges
_REACHABLE_TWO_AXIS_TILT = 0.025  # 25% margin under the 0.0333 diagonal bound


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


@pytest.mark.skip(reason='Two-axis hold diverges, envelope is not the cause: Dr-QP/Dr.QP#453')
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
