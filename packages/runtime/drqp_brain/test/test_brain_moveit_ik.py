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

"""Brain-loop tests for pure gait targets and selectable kinematics backends."""

from unittest import mock

from drqp_brain.brain_node import (
    CONTROL_RATE_MAX_HZ,
    CONTROL_RATE_MIN_HZ,
    DEFAULT_CONTROL_RATE_HZ,
    HexapodBrain,
    TickDurationWindow,
)
from drqp_brain.locomotion_kinematics import (
    AnalyticLocomotionKinematics,
    CLAMPING_EVENT_TICKS,
    LocomotionKinematicsResult,
    MoveItPyLocomotionKinematics,
)
from drqp_kinematics.geometry import AffineTransform, Point3D
import numpy as np
import pytest
import rclpy
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time


def test_analytic_backend_is_default_without_constructing_moveit_solver():
    """Default startup selects analytic IK and only retains scene validation."""
    with mock.patch('drqp_brain.brain_node.MoveItPyLocomotionKinematics') as moveit_solver_cls:
        brain = HexapodBrain()
        try:
            assert isinstance(brain.kinematics, AnalyticLocomotionKinematics)
            moveit_solver_cls.assert_not_called()
        finally:
            brain.destroy_node()


def test_control_rate_parameter_defaults_to_25_hz_and_drives_timer_spacing():
    """The declared control rate owns the timer and trajectory point timings."""
    brain = HexapodBrain()
    try:
        assert brain.control_rate_hz == DEFAULT_CONTROL_RATE_HZ == 25.0
        assert brain.loop_timer.timer_period_ns == 40_000_000
        assert brain.walking_trajectory_points == 2

        window = brain._build_walking_feet_target_window(Point3D([0, 0, 0]), Point3D([0, 0, 0]))
        assert len(window) == 2
    finally:
        brain.destroy_node()


def test_control_rate_parameter_sets_trajectory_point_spacing():
    """The two-point trajectory window retains one control-period spacing per point."""
    brain = HexapodBrain()
    try:
        brain.current_movement.stride_direction.x = 1.0
        joint_targets = _make_joint_targets(brain)
        with (
            _ik_ready_patch(brain),
            mock.patch.object(brain, 'solve_joint_targets', return_value=joint_targets),
            mock.patch.object(brain, 'apply_joint_targets'),
            mock.patch('drqp_brain.brain_node.JointTrajectoryBuilder') as builder_cls,
        ):
            brain.loop()

        trajectory = builder_cls.return_value
        assert [
            call.kwargs['reach_in_seconds_from_start']
            for call in trajectory.add_point_from_joint_targets.call_args_list
        ] == pytest.approx([0.04, 0.08])
    finally:
        brain.destroy_node()


@pytest.mark.parametrize('control_rate_hz', [CONTROL_RATE_MIN_HZ, CONTROL_RATE_MAX_HZ])
def test_control_rate_parameter_rejects_out_of_range_values(control_rate_hz):
    """The ROS parameter protects the timer from impractical update rates."""
    invalid_rate = 4.0 if control_rate_hz == CONTROL_RATE_MIN_HZ else 101.0

    with pytest.raises(InvalidParameterValueException, match='control_rate_hz'):
        HexapodBrain(
            parameter_overrides=[
                rclpy.Parameter('control_rate_hz', rclpy.Parameter.Type.DOUBLE, invalid_rate)
            ]
        )


def _run_one_second_of_tripod_gait(control_rate_hz):
    """Advance a fixed duration using deterministic timer timestamps."""
    brain = HexapodBrain(
        parameter_overrides=[
            rclpy.Parameter('control_rate_hz', rclpy.Parameter.Type.DOUBLE, float(control_rate_hz))
        ]
    )
    try:
        timestamps = iter(
            Time(nanoseconds=round(tick * 1_000_000_000 / control_rate_hz))
            for tick in range(control_rate_hz)
        )
        brain.get_clock = mock.Mock(return_value=mock.Mock(now=lambda: next(timestamps)))
        brain._ik_ready = mock.Mock(return_value=False)
        brain.current_movement.stride_direction.x = 1.0

        for _ in range(control_rate_hz):
            brain.loop()

        targets = brain.walker.targets_at(brain.walker.current_phase, brain.walker.steering)
        return brain.walker.current_phase, _target_values(targets)
    finally:
        brain.destroy_node()


def _target_values(targets):
    return [(float(target.x), float(target.y), float(target.z)) for _, target in targets]


def test_gait_phase_and_foot_paths_are_rate_invariant():
    """Equivalent elapsed time preserves gait phase and path geometry at every rate."""
    baseline_phase, baseline_targets = _run_one_second_of_tripod_gait(8)

    for control_rate_hz in (25, 50):
        phase, targets = _run_one_second_of_tripod_gait(control_rate_hz)
        assert phase == pytest.approx(baseline_phase, abs=1e-9)
        assert np.asarray(targets) == pytest.approx(np.asarray(baseline_targets), abs=1e-8)


def test_tick_duration_window_reports_sliding_min_mean_and_max():
    """Tick instrumentation retains only the configured sliding measurement window."""
    window = TickDurationWindow(capacity=3)
    for duration_sec in (0.004, 0.002, 0.006, 0.003):
        window.record(duration_sec)

    statistics = window.statistics
    assert statistics.sample_count == 3
    assert statistics.minimum_sec == pytest.approx(0.002)
    assert statistics.mean_sec == pytest.approx(0.011 / 3)
    assert statistics.maximum_sec == pytest.approx(0.006)


def test_loop_records_tick_duration_below_its_default_period():
    """Runtime instrumentation exposes the budget consumed by an idle control tick."""
    brain = HexapodBrain()
    try:
        brain._ik_ready = mock.Mock(return_value=False)
        with mock.patch('drqp_brain.brain_node.perf_counter', side_effect=[10.0, 10.001]):
            brain.loop()

        statistics = brain.tick_duration_window.statistics
        assert statistics.sample_count == 1
        assert statistics.maximum_sec == pytest.approx(0.001)
        assert statistics.maximum_sec < 1.0 / brain.control_rate_hz
    finally:
        brain.destroy_node()


def test_moveit_backend_parameter_keeps_fallback_solver():
    """The moveit parameter value routes solving through the legacy backend."""
    brain = HexapodBrain(
        parameter_overrides=[rclpy.Parameter('kinematics_backend', value='moveit')]
    )
    try:
        assert isinstance(brain.kinematics, MoveItPyLocomotionKinematics)
    finally:
        brain.destroy_node()


def test_persistent_analytic_clamping_emits_robot_event():
    """Persistent per-leg degradation exposes the future twist-scaling hook."""
    brain = HexapodBrain()
    try:
        clamped_result = LocomotionKinematicsResult(
            joint_targets=_make_joint_targets(brain),
            backend_name='analytic',
            clamped_legs=('left_front',),
        )
        brain.kinematics.solve = mock.Mock(return_value=clamped_result)
        brain.robot_event_pub.publish = mock.Mock()

        for _ in range(CLAMPING_EVENT_TICKS):
            assert brain._solve_walking_trajectory_targets([[]]) is not None

        brain.robot_event_pub.publish.assert_called_once()
        assert (
            brain.robot_event_pub.publish.call_args.args[0].data
            == 'locomotion_clamping_persistent:left_front'
        )
    finally:
        brain.destroy_node()


@pytest.fixture(autouse=True)
def rclpy_context():
    """Provide a ROS context for each node test."""
    rclpy.init()
    try:
        yield
    finally:
        rclpy.try_shutdown()


def _make_joint_targets(brain: HexapodBrain) -> dict[str, float]:
    return {
        f'drqp/{leg.label.name}_{joint_name}': 0.0
        for leg in brain.hexapod.legs
        for joint_name in ('coxa', 'femur', 'tibia')
    }


def _ik_ready_patch(brain: HexapodBrain):
    return mock.patch.object(brain, '_ik_ready', return_value=True)


@pytest.mark.parametrize('window_points', [1, 2, 4])
def test_loop_advances_phase_once_regardless_of_window_size(window_points):
    """Lookahead target evaluation must not change the control phase."""
    brain = HexapodBrain()
    try:
        brain.walking_trajectory_points = window_points
        brain.current_movement.stride_direction.x = 1.0
        brain.get_clock = mock.Mock(return_value=mock.Mock(now=lambda: Time(nanoseconds=0)))
        joint_targets = _make_joint_targets(brain)

        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain.walker,
                'targets_at',
                wraps=brain.walker.targets_at,
            ) as targets_at,
            mock.patch.object(brain, 'solve_joint_targets', return_value=joint_targets),
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()

        assert brain.walker.current_phase == pytest.approx(1.0 / (brain.control_rate_hz * 1.25))
        assert targets_at.call_count == window_points
        assert publish_mock.call_count == 1
    finally:
        brain.destroy_node()


def test_loop_failure_does_not_roll_back_motion_state():
    """A failed IK tick leaves the already-advanced state intact."""
    brain = HexapodBrain()
    try:
        brain.current_movement.stride_direction.x = 1.0
        with (
            _ik_ready_patch(brain),
            mock.patch.object(brain, 'solve_joint_targets', return_value=None),
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()

        assert brain.walker.current_phase == pytest.approx(1.0 / (brain.control_rate_hz * 1.25))
        assert brain.walker.current_direction.x > 0.0
        publish_mock.assert_not_called()
    finally:
        brain.destroy_node()


def test_stationary_ticks_do_not_solve_or_publish_after_deduplication():
    """Frozen phase and unchanged inputs suppress idle solver work."""
    brain = HexapodBrain()
    try:
        with (
            _ik_ready_patch(brain),
            mock.patch.object(brain, 'solve_joint_targets') as solve_mock,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()
            brain.loop()

        assert brain.walker.current_phase == pytest.approx(0.0)
        solve_mock.assert_not_called()
        publish_mock.assert_not_called()
    finally:
        brain.destroy_node()


def test_loop_retries_failed_targets_without_rewinding_phase():
    """A failed target is retried on the next changed gait input."""
    brain = HexapodBrain()
    try:
        brain.current_movement.stride_direction.x = 1.0
        joint_targets = _make_joint_targets(brain)
        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain,
                'solve_joint_targets',
                side_effect=[None, joint_targets, joint_targets, joint_targets],
            ) as solve_mock,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()
            first_phase = brain.walker.current_phase
            brain.loop()

        assert brain.walker.current_phase > first_phase
        assert solve_mock.call_count == 3
        publish_mock.assert_called_once()
    finally:
        brain.destroy_node()


def test_body_only_command_publishes_even_when_gait_is_stationary():
    """Dedupe compares body pose as a committed input, not foot targets alone."""
    brain = HexapodBrain()
    try:
        brain.current_movement.body_translation.z = 0.16
        joint_targets = _make_joint_targets(brain)
        with (
            _ik_ready_patch(brain),
            mock.patch.object(brain, 'solve_joint_targets', return_value=joint_targets),
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()

        publish_mock.assert_called_once()
        assert brain.walker.current_phase == pytest.approx(0.0)
    finally:
        brain.destroy_node()


def test_make_pose_stamped_converts_target_into_base_frame():
    """Verify IK targets are expressed in the declared base frame."""
    brain = HexapodBrain()
    try:
        body_transform = AffineTransform.from_translation([0.1, -0.2, 0.3])
        brain.hexapod.body_transform = body_transform
        leg = next(iter(brain.hexapod.legs))
        foot_target = Point3D([0.5, 0.4, -0.1])

        pose = brain._make_pose_stamped(leg, foot_target)
        expected = body_transform.inverse.apply_point(foot_target)

        assert pose.pose.position.x == pytest.approx(expected.x)
        assert pose.pose.position.y == pytest.approx(expected.y)
        assert pose.pose.position.z == pytest.approx(expected.z)
    finally:
        brain.destroy_node()


def test_loop_warns_once_while_waiting_for_initial_joint_state():
    """Startup readiness checks do not spam logs while the controller is idle."""
    brain = HexapodBrain()
    try:
        with mock.patch.object(brain.get_logger(), 'warning') as warning_mock:
            brain.loop()
            brain.loop()

        warning_mock.assert_called_once_with('No joint state available from trajectory controller')
    finally:
        brain.destroy_node()
