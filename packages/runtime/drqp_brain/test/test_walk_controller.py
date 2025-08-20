# Copyright (c) 2017-2025 Anton Matosov
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

from unittest.mock import Mock, patch

from drqp_brain.geometry import Point3D
from drqp_brain.models import HexapodModel
from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.walk_controller import WalkController
import numpy as np
import pytest


class TestWalkController:
    """Test the WalkController class."""

    def assert_point3d_equal(self, p1, p2, tolerance=1e-6):
        """Helper method to compare Point3D objects with tolerance."""
        assert abs(p1.x - p2.x) < tolerance, f'X mismatch: {p1.x} != {p2.x}'
        assert abs(p1.y - p2.y) < tolerance, f'Y mismatch: {p1.y} != {p2.y}'
        assert abs(p1.z - p2.z) < tolerance, f'Z mismatch: {p1.z} != {p2.z}'

    @pytest.fixture
    def mock_hexapod(self):
        """Create a mock hexapod model for testing."""
        from drqp_brain.models import HexapodLeg

        hexapod = Mock(spec=HexapodModel)

        # Create mock legs with required attributes
        mock_legs = []
        leg_labels = [
            HexapodLeg.left_front,
            HexapodLeg.right_front,
            HexapodLeg.left_middle,
            HexapodLeg.right_middle,
            HexapodLeg.left_back,
            HexapodLeg.right_back,
        ]

        for i, label in enumerate(leg_labels):
            leg = Mock()
            leg.label = label
            leg.tibia_end = Point3D([0, 0, 0])
            leg.move_to = Mock()
            mock_legs.append(leg)

        hexapod.legs = mock_legs
        return hexapod

    @pytest.fixture
    def walk_controller(self, mock_hexapod):
        """Create a WalkController instance for testing."""
        return WalkController(
            hexapod=mock_hexapod,
            step_length=0.1,
            step_height=0.05,
            rotation_speed_degrees=20.0,
            phase_steps_per_cycle=30.0,
            gait=GaitType.tripod,
        )

    def test_initialization(self, walk_controller, mock_hexapod):
        """Test that WalkController initializes correctly."""
        assert walk_controller.hexapod == mock_hexapod
        assert walk_controller.step_length == 0.1
        assert walk_controller.step_height == 0.05
        assert walk_controller.rotation_speed_degrees == 20.0
        assert walk_controller.phase_step == 1.0 / 30.0
        assert walk_controller.current_gait == GaitType.tripod

        # Check initial state after reset
        self.assert_point3d_equal(walk_controller.current_direction, Point3D([1, 0, 0]))
        assert walk_controller.current_stride_ratio == 0
        assert walk_controller.current_rotation_ratio == 0
        assert walk_controller.current_phase == 0.0
        assert walk_controller.last_stop_phase == 0.0

    def test_gait_property(self, walk_controller):
        """Test gait property getter and setter."""
        # Test getter
        assert walk_controller.current_gait == GaitType.tripod

        # Test setter
        walk_controller.current_gait = GaitType.wave
        assert walk_controller.current_gait == GaitType.wave

        walk_controller.current_gait = GaitType.ripple
        assert walk_controller.current_gait == GaitType.ripple

    def test_reset(self, walk_controller):
        """Test reset functionality."""
        # Modify some values
        walk_controller.current_direction = Point3D([0, 1, 0])
        walk_controller.current_stride_ratio = 0.5
        walk_controller.current_rotation_ratio = 0.3
        walk_controller.current_phase = 0.7
        walk_controller.last_stop_phase = 0.2

        # Reset and verify
        walk_controller.reset()

        self.assert_point3d_equal(walk_controller.current_direction, Point3D([1, 0, 0]))
        assert walk_controller.current_stride_ratio == 0
        assert walk_controller.current_rotation_ratio == 0
        assert walk_controller.current_phase == 0.0
        assert walk_controller.last_stop_phase == 0.0

    def test_phase_progression(self, walk_controller):
        """Test phase progression during movement."""
        initial_phase = walk_controller.current_phase

        # Call next_step to advance phase
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.5, rotation_ratio=0.0
        )

        # Phase should have advanced by phase_step
        expected_phase = initial_phase + walk_controller.phase_step
        assert abs(walk_controller.current_phase - expected_phase) < 1e-6

    def test_phase_override(self, walk_controller):
        """Test phase override functionality."""
        override_phase = 0.75

        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=0.5,
            rotation_ratio=0.0,
            phase_override=override_phase,
        )

        assert walk_controller.current_phase == override_phase

    @patch('drqp_brain.walk_controller.WalkController._WalkController__move_feet')
    def test_next_step_calls_move_feet(self, mock_move_feet, walk_controller):
        """Test that next_step calls move_feet with correct targets."""
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.5, rotation_ratio=0.2
        )

        # Verify move_feet was called
        mock_move_feet.assert_called_once()

        # Get the arguments passed to move_feet
        call_args = mock_move_feet.call_args[0][0]

        # Should have 6 leg-target pairs (one for each leg)
        assert len(call_args) == 6

        # Each item should be a tuple of (leg, target_point)
        for leg, target in call_args:
            assert hasattr(leg, 'label')
            assert isinstance(target, Point3D)

    def test_stride_ratio_clipping(self, walk_controller):
        """Test that stride ratio is properly clipped to [0, 1]."""
        # Test with values outside valid range
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=1.5,  # Above max
            rotation_ratio=0.0,
        )

        # Should be clipped to 1.0 or interpolated towards it
        assert walk_controller.current_stride_ratio <= 1.0

        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=-0.5,  # Below min
            rotation_ratio=0.0,
        )

        # Should be clipped to 0.0 or interpolated towards it
        assert walk_controller.current_stride_ratio >= 0.0

    def test_rotation_ratio_clipping(self, walk_controller):
        """Test that rotation ratio is properly clipped to [-1, 1]."""
        # Test with values outside valid range
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=0.5,
            rotation_ratio=1.5,  # Above max
        )

        # Should be clipped to 1.0 or interpolated towards it
        assert walk_controller.current_rotation_ratio <= 1.0

        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=0.5,
            rotation_ratio=-1.5,  # Below min
        )

        # Should be clipped to -1.0 or interpolated towards it
        assert walk_controller.current_rotation_ratio >= -1.0

    def test_direction_interpolation(self, walk_controller):
        """Test that direction changes are interpolated smoothly."""
        initial_direction = walk_controller.current_direction
        new_direction = Point3D([0, 1, 0])  # 90 degree turn

        walk_controller.next_step(
            stride_direction=new_direction, stride_ratio=0.5, rotation_ratio=0.0
        )

        # Direction should have changed but not immediately to target
        current_direction = walk_controller.current_direction
        assert current_direction != initial_direction
        assert current_direction != new_direction

        # Should be somewhere between initial and target
        dot_initial = current_direction.dot(initial_direction)
        dot_target = current_direction.dot(new_direction)
        assert dot_initial > 0  # Still has component of initial direction
        assert dot_target > 0  # Has component of target direction

    def test_stopping_behavior(self, walk_controller):
        """Test behavior when transitioning from motion to stop."""
        # Start with motion
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.8, rotation_ratio=0.0
        )

        # Verify we have motion
        assert walk_controller.current_stride_ratio > 0.05

        # Stop motion
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.0, rotation_ratio=0.0
        )

        # Should record stop phase
        assert walk_controller.last_stop_phase > 0.0

    def test_starting_behavior(self, walk_controller):
        """Test behavior when transitioning from stop to motion."""
        # Ensure we start at rest
        walk_controller.reset()

        # Start motion
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.5, rotation_ratio=0.0
        )

        # Phase should reset to 0 when starting
        assert walk_controller.current_phase == 0.0

    def test_move_feet_calls_leg_move_to(self, walk_controller):
        """Test that move_feet calls move_to on each leg."""
        # Create test targets
        targets = []
        for i, leg in enumerate(walk_controller.hexapod.legs):
            target = Point3D([i, i, i])
            targets.append((leg, target))

        # Call the private method directly for testing
        walk_controller._WalkController__move_feet(targets)

        # Verify each leg's move_to was called with correct target
        for i, leg in enumerate(walk_controller.hexapod.legs):
            leg.move_to.assert_called_once()
            # Check the actual argument passed
            call_args = leg.move_to.call_args[0][0]
            expected_target = Point3D([i, i, i])
            self.assert_point3d_equal(call_args, expected_target)

    def test_direction_transform_forward(self, walk_controller):
        """Test direction transform for forward movement."""
        forward_direction = Point3D([1, 0, 0])
        transform = walk_controller._WalkController__make_direction_transform(forward_direction)

        # Forward direction should map to itself
        test_point = Point3D([1, 0, 0])
        result = transform.apply_point(test_point)

        # Should be approximately the same (allowing for floating point errors)
        assert abs(result.x - 1.0) < 1e-6
        assert abs(result.y - 0.0) < 1e-6
        assert abs(result.z - 0.0) < 1e-6

    def test_direction_transform_left(self, walk_controller):
        """Test direction transform for left movement."""
        left_direction = Point3D([0, 1, 0])
        transform = walk_controller._WalkController__make_direction_transform(left_direction)

        # Forward step should map to left
        test_point = Point3D([1, 0, 0])
        result = transform.apply_point(test_point)

        # Should map to left direction
        assert abs(result.x - 0.0) < 1e-6
        assert abs(result.y - 1.0) < 1e-6
        assert abs(result.z - 0.0) < 1e-6

    def test_direction_transform_diagonal(self, walk_controller):
        """Test direction transform for diagonal movement."""
        # 45-degree diagonal (normalized)
        diagonal_direction = Point3D([1, 1, 0]).normalized()
        transform = walk_controller._WalkController__make_direction_transform(diagonal_direction)

        # Forward step should map to diagonal
        test_point = Point3D([1, 0, 0])
        result = transform.apply_point(test_point)

        # Should be approximately diagonal
        expected_x = 1.0 / np.sqrt(2)
        expected_y = 1.0 / np.sqrt(2)
        assert abs(result.x - expected_x) < 1e-6
        assert abs(result.y - expected_y) < 1e-6
        assert abs(result.z - 0.0) < 1e-6

    def test_direction_transform_ignores_z(self, walk_controller):
        """Test that direction transform ignores z-component."""
        # Direction with z-component
        direction_with_z = Point3D([1, 0, 1])
        transform = walk_controller._WalkController__make_direction_transform(direction_with_z)

        # Forward step should only use x,y components
        test_point = Point3D([1, 0, 0])
        result = transform.apply_point(test_point)

        # Z should remain unchanged, x,y should be normalized to ignore z
        # The direction [1, 0, 1] normalized in x,y plane is [1, 0] / sqrt(1^2 + 0^2) = [1, 0]
        # So the transform should map [1, 0, 0] to [1, 0, 0] (normalized direction in x,y)
        assert abs(result.x - 1.0) < 1e-6
        assert abs(result.y - 0.0) < 1e-6
        assert abs(result.z - 0.0) < 1e-6

    @patch('drqp_brain.walk_controller.WalkController._WalkController__make_direction_transform')
    def test_gait_integration(self, mock_transform, walk_controller):
        """Test integration with gait generator."""
        # Mock the transform to return identity
        mock_transform_obj = Mock()
        mock_transform_obj.apply_point = Mock(side_effect=lambda p: p)
        mock_transform.return_value = mock_transform_obj

        # Mock gait generator to return predictable offsets
        with patch.object(walk_controller.gait_gen, 'get_offsets_at_phase_for_leg') as mock_gait:
            mock_gait.return_value = Point3D([0.5, 0.0, 0.2])

            walk_controller.next_step(
                stride_direction=Point3D([1, 0, 0]), stride_ratio=1.0, rotation_ratio=0.0
            )

            # Verify gait generator was called for each leg
            assert mock_gait.call_count == 6

            # Verify each call had the correct leg label and phase
            for i, call in enumerate(mock_gait.call_args_list):
                leg_label, phase = call[0]
                assert leg_label == walk_controller.hexapod.legs[i].label
                assert phase == walk_controller.current_phase

    def test_no_motion_threshold(self, walk_controller):
        """Test that small motion values are treated as no motion."""
        # Very small stride ratio (below threshold)
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]),
            stride_ratio=0.01,  # Below 0.05 threshold
            rotation_ratio=0.0,
        )

        # Should be treated as no motion
        # Phase should reset to 0 for stopped state
        assert walk_controller.current_phase == 0.0

    def test_verbose_output(self, walk_controller, capsys):
        """Test verbose output functionality."""
        walk_controller.next_step(
            stride_direction=Point3D([1, 0, 0]), stride_ratio=0.5, rotation_ratio=0.0, verbose=True
        )

        # Check that verbose output was generated
        captured = capsys.readouterr()
        assert 'current_phase' in captured.out or len(captured.out) > 0

    def test_leg_tips_on_ground_initialization(self, walk_controller):
        """Test that leg tips on ground are properly initialized."""
        assert len(walk_controller.leg_tips_on_ground) == 6

        for leg, tip in walk_controller.leg_tips_on_ground:
            assert hasattr(leg, 'label')
            assert isinstance(tip, Point3D)
            # Tip should be a copy of the leg's tibia_end
            self.assert_point3d_equal(tip, leg.tibia_end)
            assert tip is not leg.tibia_end  # Should be a copy, not reference
