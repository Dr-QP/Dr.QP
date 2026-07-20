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

"""Unit tests for backend-neutral Gazebo harness diagnostics."""

from geometry_msgs.msg import Pose
import pytest
from rcl_interfaces.msg import Log
from robot_control_test_support import GazeboRobotControlBase
from scipy.spatial.transform import Rotation as R
import std_msgs.msg


def make_harness() -> GazeboRobotControlBase:
    """Create only the diagnostic state without starting ROS or Gazebo."""
    harness = GazeboRobotControlBase.__new__(GazeboRobotControlBase)
    harness.analytic_clamping_messages = []
    harness.complete_state_rejection_messages = []
    harness.kinematics_rejection_messages = []
    harness.legacy_moveit_ik_failure_messages = []
    harness.robot_events = []
    harness.persistent_clamping_events = []
    return harness


def test_rosout_callback_classifies_current_kinematics_diagnostics() -> None:
    """Capture default analytic and collision diagnostics without stale naming."""
    harness = make_harness()

    harness._rosout_callback(Log(name='drqp_brain', msg='Analytic IK clamped legs: left_back'))
    harness._rosout_callback(Log(name='drqp_brain', msg='RobotState is in self-collision'))
    harness._rosout_callback(
        Log(
            name='drqp_brain',
            msg='Kinematics rejected the current foot targets; skipping trajectory publish',
        )
    )
    harness._rosout_callback(Log(name='drqp_brain', msg='MoveItPy IK failed for left_front'))

    assert harness.analytic_clamping_messages == ['Analytic IK clamped legs: left_back']
    assert harness.complete_state_rejection_messages == ['RobotState is in self-collision']
    assert harness.kinematics_rejection_messages == [
        'Kinematics rejected the current foot targets; skipping trajectory publish'
    ]
    assert harness.legacy_moveit_ik_failure_messages == ['MoveItPy IK failed for left_front']


def test_persistent_analytic_clamping_fails_backend_neutral_check() -> None:
    """A persistent clamp event must fail even if no legacy IK text is logged."""
    harness = make_harness()
    harness._robot_event_callback(
        std_msgs.msg.String(data='locomotion_clamping_persistent:left_back')
    )

    with pytest.raises(AssertionError, match='Persistent analytic clamping'):
        harness.assert_no_kinematics_failures()


def test_current_kinematics_rejection_fails_backend_neutral_check() -> None:
    """A current default-path rejection must not evade a legacy text filter."""
    harness = make_harness()
    harness._rosout_callback(
        Log(
            name='drqp_brain',
            msg='Kinematics rejected the current foot targets; skipping trajectory publish',
        )
    )

    with pytest.raises(AssertionError, match='selected kinematics backend'):
        harness.assert_no_kinematics_failures()


def test_transient_analytic_clamping_remains_observable_but_allowed() -> None:
    """A transient per-tick clamp is recorded without claiming a hard rejection."""
    harness = make_harness()
    harness._rosout_callback(Log(name='drqp_brain', msg='Analytic IK clamped legs: right_middle'))

    harness.assert_no_kinematics_failures()


def test_stable_body_wait_returns_the_pose_that_met_the_criterion() -> None:
    """Do not resample a later oscillation phase after stability was established."""
    harness = make_harness()
    harness.robot_pose = Pose()
    x, y, z, w = R.from_euler('xyz', [0.01, -0.02, 0.0]).as_quat()
    harness.robot_pose.orientation.x = x
    harness.robot_pose.orientation.y = y
    harness.robot_pose.orientation.z = z
    harness.robot_pose.orientation.w = w
    harness.robot_pose_stamp_ns = 1
    harness._wait_for_new_pose = lambda _stamp: None
    harness._wait_for_sim_time = lambda _duration: None
    sim_times = iter([0, int(harness.POSE_SETTLE_DURATION * 1_000_000_000)])
    harness._current_sim_time_ns = lambda: next(sim_times)

    def satisfy_stability(predicate, _timeout, _message) -> None:
        assert not predicate()
        assert predicate()

    harness._spin_until_sim_time = satisfy_stability

    roll, pitch = harness._wait_for_stable_body_level(
        initial_roll=0.0,
        initial_pitch=0.0,
        board_roll=0.10,
        board_pitch=0.10,
    )

    assert roll == pytest.approx(0.01)
    assert pitch == pytest.approx(-0.02)
