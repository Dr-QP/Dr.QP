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


def test_loop_uses_moveit_joint_targets_instead_of_leg_move_to():
    """Loop should consume MoveIt joint targets without falling back to custom leg IK."""
    brain = HexapodBrain()
    try:
        joint_targets = _make_joint_targets(brain)

        for leg in brain.hexapod.legs:
            leg.move_to = mock.Mock(side_effect=AssertionError('custom IK must not run'))

        with (
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
