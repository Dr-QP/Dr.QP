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

from unittest import mock

from drqp_brain.brain_node import HexapodBrain
from drqp_kinematics.geometry import AffineTransform, Point3D
import pytest
import rclpy


@pytest.fixture(autouse=True)
def ros_context():
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


def _make_feet_targets(brain: HexapodBrain):
    return [(leg, leg.tibia_end.copy()) for leg in brain.hexapod.legs]


def _ik_ready_patch(brain: HexapodBrain):
    return mock.patch.object(brain, '_ik_ready', return_value=True)


def test_loop_uses_moveit_joint_targets_instead_of_leg_move_to():
    """Loop should consume MoveIt joint targets without falling back to custom leg IK."""
    brain = HexapodBrain()
    try:
        joint_targets = _make_joint_targets(brain)

        for leg in brain.hexapod.legs:
            leg.move_to = mock.Mock(side_effect=AssertionError('custom IK must not run'))

        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain,
                'solve_joint_targets',
                create=True,
                return_value=joint_targets,
            ) as solve_joint_targets,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
            mock.patch.object(brain.get_logger(), 'warning') as warning_mock,
        ):
            brain.loop()

        solve_joint_targets.assert_called_once()
        publish_mock.assert_called_once()
        warning_mock.assert_not_called()
    finally:
        brain.destroy_node()


def test_loop_warns_and_skips_publish_when_moveit_returns_no_solution():
    """Loop should leave the robot in its current safe state when MoveIt cannot solve IK."""
    brain = HexapodBrain()
    try:
        original_direction = brain.walker.current_direction.copy()
        original_rotation = brain.walker.current_rotation_direction
        original_phase = brain.walker.current_phase

        command = brain.current_movement
        command.stride_direction.x = 1.0
        command.rotation_speed = 0.5

        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain,
                'solve_joint_targets',
                create=True,
                return_value=None,
            ) as solve_joint_targets,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
            mock.patch.object(brain.get_logger(), 'warning') as warning_mock,
        ):
            brain.loop()

        solve_joint_targets.assert_called_once()
        warning_mock.assert_called_once()
        publish_mock.assert_not_called()
        assert brain.walker.current_direction == original_direction
        assert brain.walker.current_rotation_direction == original_rotation
        assert brain.walker.current_phase == original_phase
    finally:
        brain.destroy_node()


def test_loop_logs_error_and_skips_publish_when_moveit_service_fails():
    """Loop should log and stop cleanly when the MoveIt IK service is unavailable."""
    brain = HexapodBrain()
    try:
        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain,
                'solve_joint_targets',
                create=True,
                side_effect=RuntimeError('compute_ik unavailable'),
            ) as solve_joint_targets,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
            mock.patch.object(brain.get_logger(), 'error') as error_mock,
        ):
            brain.loop()

        solve_joint_targets.assert_called_once()
        error_mock.assert_called_once()
        publish_mock.assert_not_called()
    finally:
        brain.destroy_node()


def test_loop_skips_redundant_ik_when_feet_targets_do_not_change():
    """Identical stabilized foot targets should not re-enter MoveIt every timer tick."""
    brain = HexapodBrain()
    try:
        feet_targets = _make_feet_targets(brain)
        joint_targets = _make_joint_targets(brain)

        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain.walker,
                'next_step_targets',
                side_effect=[feet_targets, feet_targets],
            ),
            mock.patch.object(
                brain, 'solve_joint_targets', return_value=joint_targets
            ) as solve_mock,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
        ):
            brain.loop()
            brain.loop()

        solve_mock.assert_called_once_with(feet_targets)
        publish_mock.assert_called_once()
    finally:
        brain.destroy_node()


def test_loop_retries_redundant_targets_after_timeout():
    """A failed solve must not poison the redundant-target cache."""
    brain = HexapodBrain()
    try:
        feet_targets = _make_feet_targets(brain)
        joint_targets = _make_joint_targets(brain)

        with (
            _ik_ready_patch(brain),
            mock.patch.object(
                brain.walker,
                'next_step_targets',
                side_effect=[feet_targets, feet_targets],
            ),
            mock.patch.object(
                brain,
                'solve_joint_targets',
                side_effect=[RuntimeError('MoveIt IK request timed out'), joint_targets],
            ) as solve_mock,
            mock.patch.object(brain.joint_trajectory_pub, 'publish') as publish_mock,
            mock.patch.object(brain.get_logger(), 'error') as error_mock,
        ):
            brain.loop()
            brain.loop()

        assert solve_mock.call_count == 2
        error_mock.assert_called_once()
        publish_mock.assert_called_once()
    finally:
        brain.destroy_node()


def test_make_pose_stamped_converts_target_into_base_frame():
    """Verify IK targets are expressed in the declared BASE_FRAME."""
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
    """Startup readiness issues should be transient warnings rather than log spam every tick."""
    brain = HexapodBrain()
    try:
        with mock.patch.object(brain.get_logger(), 'warning') as warning_mock:
            brain.loop()
            brain.loop()

        warning_mock.assert_called_once_with('No joint state available to seed MoveIt IK requests')
    finally:
        brain.destroy_node()
