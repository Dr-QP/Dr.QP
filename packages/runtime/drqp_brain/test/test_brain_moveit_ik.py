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

from drqp_brain.brain_node import HexapodBrain
from drqp_brain.locomotion_kinematics import (
    AnalyticLocomotionKinematics,
    CLAMPING_EVENT_TICKS,
    LocomotionKinematicsResult,
    MoveItPyLocomotionKinematics,
)
from drqp_kinematics.geometry import AffineTransform, Point3D
import pytest
import rclpy
from rclpy.time import Time


def test_analytic_backend_is_default_without_constructing_moveit_solver():
    """Default startup selects analytic IK and only retains scene validation."""
    with mock.patch(
        'drqp_brain.brain_node.MoveItPyLocomotionKinematics'
    ) as moveit_solver_cls:
        brain = HexapodBrain()
        try:
            assert isinstance(brain.kinematics, AnalyticLocomotionKinematics)
            moveit_solver_cls.assert_not_called()
        finally:
            brain.destroy_node()


def test_moveit_backend_parameter_keeps_fallback_solver():
    """The moveit parameter value routes solving through the legacy backend."""
    brain = HexapodBrain(
        parameter_overrides=[
            rclpy.Parameter('kinematics_backend', value='moveit')
        ]
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

        assert brain.walker.current_phase == pytest.approx(0.125 / 1.25)
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

        assert brain.walker.current_phase == pytest.approx(0.125 / 1.25)
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

        warning_mock.assert_called_once_with(
            'No joint state available from trajectory controller'
        )
    finally:
        brain.destroy_node()
