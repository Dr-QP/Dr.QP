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

from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.stride_limits import DirectionalStrideLimits
from drqp_brain.walk_controller import WalkController
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodLeg, HexapodModel
import numpy as np
import pytest


class TestWalkController:
    """Test the WalkController class."""

    @pytest.fixture
    def hexapod(self):
        hexapod = HexapodModel()
        hexapod.forward_kinematics(0, -35, 130)
        return hexapod

    @pytest.fixture
    def walker(self, hexapod):
        return WalkController(hexapod)

    def test_initialization(self, walker):
        assert walker.hexapod is not None
        assert walker.step_length == 60.0
        assert walker.step_height == 40.0
        assert walker.rotation_speed_degrees == 10.0
        assert walker.gait_gen.current_gait == GaitType.wave
        assert walker.current_gait == GaitType.wave
        assert walker.phase_step == 1 / 30.0
        assert walker.current_direction == Point3D([0, 0, 0])
        assert walker.current_rotation_direction == 0.0
        assert walker.current_phase == 0.0

    def test_reset(self, walker):
        walker.current_direction = Point3D([0, 1, 0])
        walker.current_rotation_direction = 1.0
        walker.current_phase = 1.0

        walker.reset()

        assert walker.current_direction == Point3D([0, 0, 0])
        assert walker.current_rotation_direction == 0.0
        assert walker.current_phase == 0.0

    def test_current_phase(self, walker):
        walker.current_phase = 0.5

        walker.next_step(stride_direction=Point3D([1, 0, 0]), rotation_direction=0.0)
        assert walker.current_phase == 0, 'Starting resets phase to 0'

        walker.next_step(stride_direction=Point3D([1, 0, 0]), rotation_direction=0.0)
        assert walker.current_phase == 1 / 30.0

        walker.next_step(stride_direction=Point3D([1, 0, 0]), rotation_direction=0.0)
        assert walker.current_phase == 2 / 30.0

        # Phase out takes some steps
        for _ in range(10):
            walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=0.0)

        assert walker.current_phase == 0, 'Stopping resets phase to 0'

    def test_current_direction_ramping(self, walker, hexapod):
        walker.current_direction = Point3D([0, 0, 0])
        feet_at_rest = [leg.tibia_end.copy() for leg in hexapod.legs]

        walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=0.0)
        feet_at_zero_step = [leg.tibia_end.copy() for leg in hexapod.legs]
        for at_rest, after_step in zip(feet_at_rest, feet_at_zero_step):
            assert at_rest == after_step

        # Each entry drives one step with the given stride_direction and pins the
        # resulting current_direction as it ramps up towards, then down from, [1, 0, 0].
        ramping_steps = [
            (Point3D([1, 0, 0]), Point3D([0.3, 0, 0]), 'Direction is ramping up 1'),
            (Point3D([1, 0, 0]), Point3D([0.51, 0, 0]), 'Direction is ramping up 2'),
            (Point3D([1, 0, 0]), Point3D([0.657, 0, 0]), 'Direction is ramping up 3'),
            (Point3D([0, 0, 0]), Point3D([0.4599, 0, 0]), 'Direction is ramping down 1'),
            (Point3D([0, 0, 0]), Point3D([0.3219, 0, 0]), 'Direction is ramping down 2'),
            (Point3D([0, 0, 0]), Point3D([0.2253, 0, 0]), 'Direction is ramping down 3'),
        ]
        for stride_direction, expected_direction, message in ramping_steps:
            walker.next_step(stride_direction=stride_direction, rotation_direction=0.0)
            assert walker.current_direction == expected_direction, message

        for _ in range(10):
            walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=0.0)

        feet_at_end = [leg.tibia_end.copy() for leg in hexapod.legs]
        for at_rest, at_end in zip(feet_at_rest, feet_at_end):
            assert at_rest == at_end

    def test_stride_limits_allow_safe_lateral_stride(self, hexapod):
        walker = WalkController(
            hexapod,
            step_length=0.10,
            stride_limits=self._make_stride_limits(lateral_step_length=0.08),
            gait=GaitType.tripod,
        )

        for _ in range(20):
            walker.next_step(stride_direction=Point3D([0, 0.8, 0]), rotation_direction=0.0)

        assert walker.current_direction == Point3D([0, 0.8, 0])

    def test_stride_limits_clamp_full_lateral_stride(self, hexapod):
        walker = WalkController(
            hexapod,
            step_length=0.10,
            stride_limits=self._make_stride_limits(lateral_step_length=0.08),
            gait=GaitType.tripod,
        )

        for _ in range(20):
            walker.next_step(stride_direction=Point3D([0, 0.9, 0]), rotation_direction=0.0)

        assert walker.current_direction == Point3D([0, 0.8, 0])

    @staticmethod
    def _make_stride_limits(lateral_step_length):
        return DirectionalStrideLimits.from_dict(
            {
                'version': 1,
                'directions_count': 4,
                'joint_margin_degrees': 0.25,
                'gaits': {
                    gait.name: [
                        {'angle_degrees': 0.0, 'max_step_length_m': 0.10},
                        {'angle_degrees': 90.0, 'max_step_length_m': lateral_step_length},
                        {'angle_degrees': 180.0, 'max_step_length_m': 0.10},
                        {'angle_degrees': 270.0, 'max_step_length_m': lateral_step_length},
                    ]
                    for gait in GaitType
                },
            }
        )

    def test_current_rotation_ramping(self, walker, hexapod):
        walker.current_rotation_direction = 0.0
        feet_at_rest = [leg.tibia_end.copy() for leg in hexapod.legs]

        walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=0.0)
        feet_at_zero_step = [leg.tibia_end.copy() for leg in hexapod.legs]
        for at_rest, after_step in zip(feet_at_rest, feet_at_zero_step):
            assert at_rest == after_step

        # Each entry drives one step with the given rotation_direction and pins the
        # resulting current_rotation_direction as it ramps up towards, then down from, 1.0.
        ramping_steps = [
            (1.0, 0.3, 'Rotation ratio is ramping up 1'),
            (1.0, 0.51, 'Rotation ratio is ramping up 2'),
            (1.0, 0.657, 'Rotation ratio is ramping up 3'),
            (0.0, 0.4599, 'Rotation ratio is ramping down 1'),
            (0.0, 0.3219, 'Rotation ratio is ramping down 2'),
            (0.0, 0.2253, 'Rotation ratio is ramping down 3'),
        ]
        for rotation_direction, expected_rotation, message in ramping_steps:
            walker.next_step(
                stride_direction=Point3D([0, 0, 0]), rotation_direction=rotation_direction
            )
            assert walker.current_rotation_direction == pytest.approx(
                expected_rotation, rel=1e-3
            ), message

        for _ in range(10):
            walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=0.0)

        feet_at_end = [leg.tibia_end.copy() for leg in hexapod.legs]
        for at_rest, at_end in zip(feet_at_rest, feet_at_end):
            assert at_rest == at_end

    def test_step_stride(self, walker, hexapod):
        walker.current_gait = GaitType.tripod

        feet_at_rest = [leg.tibia_end.copy() for leg in hexapod.legs]

        # Ramp up walking
        for _ in range(100):
            walker.next_step(stride_direction=Point3D([1, 0, 0]), rotation_direction=0.0)

        assert walker.current_direction == Point3D([1, 0, 0]), 'Direction is at full stride'
        assert walker.current_rotation_direction == pytest.approx(0.0, abs=1e-3), (
            'Rotation is at rest'
        )

        walker.next_step(
            stride_direction=Point3D([1, 0, 0]),
            rotation_direction=0.0,
            phase_override=0.25,
            verbose=True,
        )
        feet_at_quarter_phase = [leg.tibia_end.copy() for leg in hexapod.legs]

        self._check_legs_lifted(feet_at_rest, feet_at_quarter_phase, walker.step_height)

        walker.next_step(
            stride_direction=Point3D([1, 0, 0]),
            rotation_direction=0.0,
            phase_override=0.5,
            verbose=True,
        )
        feet_at_half_phase = [leg.tibia_end.copy() for leg in hexapod.legs]

        for at_rest, after_step in zip(feet_at_rest, feet_at_half_phase):
            diff = after_step - at_rest

            assert diff.z == pytest.approx(0, abs=1e-3)
            assert abs(diff.x) == pytest.approx(walker.step_length / 2, abs=1e-3), (
                f'Leg {at_rest.label} is fully extended forward'
            )

    def _check_legs_lifted(self, feet_at_rest, feet_raised, step_height):
        lifted_legs_count = 0
        for at_rest, after_step in zip(feet_at_rest, feet_raised):
            diff = after_step - at_rest
            assert diff.x == pytest.approx(0, abs=1e-3)
            if diff.z > 0:
                lifted_legs_count += 1
                assert diff.z == pytest.approx(step_height, abs=1e-3), (
                    f'Leg {at_rest.label} is fully lifted'
                )
            else:
                assert diff.z == pytest.approx(0, abs=1e-3)

        assert lifted_legs_count == 3, 'Tripod gait lifts 3 legs at 0.25 phase'

    def test_step_rotation(self, walker, hexapod):
        walker.current_gait = GaitType.tripod
        feet_at_rest = [leg.tibia_end.copy() for leg in hexapod.legs]

        # Ramp up rotation
        for _ in range(20):
            walker.next_step(stride_direction=Point3D([0, 0, 0]), rotation_direction=1.0)

        assert walker.current_rotation_direction == pytest.approx(1.0, abs=1e-3), (
            'Rotation is at full speed'
        )
        assert walker.current_direction == Point3D([0, 0, 0]), 'Direction is at rest'

        walker.next_step(
            stride_direction=Point3D([0, 0, 0]), rotation_direction=1.0, phase_override=0.25
        )
        feet_at_quarter_phase = [leg.tibia_end.copy() for leg in hexapod.legs]
        self._check_legs_lifted(feet_at_rest, feet_at_quarter_phase, walker.step_height)

        walker.next_step(
            stride_direction=Point3D([0, 0, 0]),
            rotation_direction=1.0,
            phase_override=0.5,
            verbose=True,
        )
        feet_at_half_phase = [leg.tibia_end.copy() for leg in hexapod.legs]
        for at_rest, after_step in zip(feet_at_rest, feet_at_half_phase):
            diff = after_step - at_rest
            assert diff.z == pytest.approx(0, abs=1e-3)
            assert abs(diff.x) > 0
            assert abs(diff.y) > 0

            angular_distance = np.rad2deg(
                np.arctan2(after_step.y, after_step.x) - np.arctan2(at_rest.y, at_rest.x)
            )

            assert abs(angular_distance) == pytest.approx(
                walker.rotation_speed_degrees / 2, abs=1e-2
            )

    def test_step_rotation_and_stride(self, walker, hexapod):
        walker.current_gait = GaitType.tripod
        feet_at_rest = [leg.tibia_end.copy() for leg in hexapod.legs]

        # Ramp up rotation and stride
        for _ in range(20):
            walker.next_step(stride_direction=Point3D([1, 0, 0]), rotation_direction=1.0)

        assert walker.current_direction == Point3D([1, 0, 0]), 'Direction is at full stride'
        assert walker.current_rotation_direction == pytest.approx(1.0, abs=1e-3), (
            'Rotation is at full speed'
        )

        walker.next_step(
            stride_direction=Point3D([1, 0, 0]),
            rotation_direction=1.0,
            phase_override=0.25,
            verbose=True,
        )
        feet_at_quarter_phase = [leg.tibia_end.copy() for leg in hexapod.legs]

        self._check_legs_lifted(feet_at_rest, feet_at_quarter_phase, walker.step_height)

        walker.next_step(
            stride_direction=Point3D([0, 0, 0]),
            rotation_direction=1.0,
            phase_override=0.5,
            verbose=True,
        )
        feet_at_half_phase = [leg.tibia_end.copy() for leg in hexapod.legs]

        # Unlike test_step_rotation (pure rotation), the combined stride+rotation case
        # does not rotate every leg tip by the same angle. __next_feet_targets() blends a
        # translated "stride" target and a rotated "rotation" target via a per-leg weighted
        # average, so the net angular displacement is leg-position dependent. The tripod
        # gait's alternating gait_offsets.x flips the rotation sign between the two leg
        # groups, and the stride blend dilutes each angle below the pure-rotation value
        # (rotation_speed_degrees / 2 == 5.0, see test_step_rotation). The expected values
        # below were observed from the deterministic gait and pinned to guard the magnitude
        # and direction of rotation for every leg (see #397).
        expected_angular_distance_degrees = {
            HexapodLeg.left_front: -1.863,
            HexapodLeg.left_middle: 1.127,
            HexapodLeg.left_back: -1.792,
            HexapodLeg.right_front: 3.974,
            HexapodLeg.right_middle: -4.749,
            HexapodLeg.right_back: 4.132,
        }
        for leg, at_rest, after_step in zip(hexapod.legs, feet_at_rest, feet_at_half_phase):
            diff = after_step - at_rest
            assert diff.z == pytest.approx(0, abs=1e-3)
            assert abs(diff.x) > 0
            assert abs(diff.y) > 0

            angular_distance = np.rad2deg(
                np.arctan2(after_step.y, after_step.x) - np.arctan2(at_rest.y, at_rest.x)
            )

            assert abs(angular_distance) < walker.rotation_speed_degrees / 2, (
                f'Leg {leg.label} rotates less than pure rotation because stride dilutes it'
            )
            assert angular_distance == pytest.approx(
                expected_angular_distance_degrees[leg.label], abs=1e-2
            )

    def test_body_translation(self, walker, hexapod):
        walker.next_step(
            stride_direction=Point3D([0, 0, 0]),
            rotation_direction=0.0,
            body_direction=Point3D([0.1, 0.2, 0.3]),
        )
        assert hexapod.body_transform.translation == pytest.approx([0.1, 0.2, 0.3], abs=1e-3), (
            'Body is translated'
        )

    def test_body_rotation(self, walker, hexapod):
        walker.next_step(
            stride_direction=Point3D([0, 0, 0]),
            rotation_direction=0.0,
            body_rotation=Point3D([0.1, 0.2, 0.3]),
        )
        assert hexapod.body_transform.rotation == pytest.approx(
            AffineTransform.from_rotvec([0.1, 0.2, 0.3]).rotation, abs=1e-3
        ), 'Body is rotated'

    def test_body_translation_and_rotation(self, walker, hexapod):
        walker.next_step(
            stride_direction=Point3D([0, 0, 0]),
            rotation_direction=0.0,
            body_direction=Point3D([0.1, 0.2, 0.3]),
            body_rotation=Point3D([0.3, 0.4, 0.5]),
        )
        assert hexapod.body_transform.translation == pytest.approx(
            [0.131, 0.165, 0.309], abs=1e-3
        ), 'Body is translated'
        assert hexapod.body_transform.rotation == pytest.approx(
            AffineTransform.from_rotvec([0.3, 0.4, 0.5]).rotation, abs=1e-3
        ), 'Body is rotated'
