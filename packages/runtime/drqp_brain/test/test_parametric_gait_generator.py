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

from drqp_brain.parametric_gait_generator import GaitType, ParametricGaitGenerator
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodLeg
import numpy as np
import pytest


def is_swing_phase(offsets: Point3D):
    return offsets.label == 'swing'


class TestParametricGaitGenerator:
    """Test the ParametricGaitGenerator class."""

    @pytest.fixture
    def hexapod_legs(self):
        return [
            HexapodLeg.right_back,
            HexapodLeg.right_middle,
            HexapodLeg.right_front,
            HexapodLeg.left_back,
            HexapodLeg.left_middle,
            HexapodLeg.left_front,
        ]

    @pytest.fixture
    def gait_gen(self):
        return ParametricGaitGenerator(step_length=1.0, step_height=1.0)

    def test_tripod(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for tripod gait."""
        gait_gen.current_gait = GaitType.tripod

        self._test_gait(gait_gen, hexapod_legs, 0.5, 0, 3)

    def test_wave(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for wave gait."""
        gait_gen.current_gait = GaitType.wave

        self._test_gait(gait_gen, hexapod_legs, 1 / 6, 0, 1)

    def test_ripple(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for ripple gait."""
        gait_gen.current_gait = GaitType.ripple

        self._test_gait(gait_gen, hexapod_legs, 1 / 6, 1, 2)

    def _test_gait(
        self,
        gait_gen,
        hexapod_legs,
        stance_period,
        expected_on_ground_swing_count,
        expected_off_ground_swing_count,
    ):
        on_ground = []  # np.arange(0.0, 1.0, stance_period)
        leg_count = len(hexapod_legs)
        prev_offsets = {}
        for phase in np.arange(0.0, 1.0, 0.001):
            swing_legs, stance_legs, prev_offsets = self._gait_stages(
                gait_gen, hexapod_legs, phase, prev_offsets
            )
            swing_count = len(swing_legs)
            stance_count = len(stance_legs)
            if phase in on_ground:
                assert swing_count == expected_on_ground_swing_count, (
                    f'Phase: {phase}. {swing_legs=}'
                )
                assert stance_count == leg_count - expected_on_ground_swing_count, (
                    f'Phase: {phase}. {stance_legs=}'
                )
            else:
                assert swing_count == expected_off_ground_swing_count, (
                    f'Phase: {phase}. {swing_legs=}'
                )
                assert stance_count == leg_count - expected_off_ground_swing_count, (
                    f'Phase: {phase}. {stance_legs=}'
                )

    def test_world_tip_translation_adds_offset(self, gait_gen, hexapod_legs):
        """For a translation gait the world tip is the leg centre plus the raw offset."""
        gait_gen.current_gait = GaitType.tripod
        leg_center = Point3D([10.0, -5.0, 0.0])

        for leg in hexapod_legs:
            for phase in np.linspace(0.0, 1.0, 25):
                offset = gait_gen.get_offsets_at_phase_for_leg(leg, phase)
                tip = gait_gen.get_world_tip_at_phase_for_leg(
                    leg, phase, leg_center, rotation=False
                )
                assert tip == leg_center + offset

    def test_world_tip_rotation_rotates_center(self, gait_gen, hexapod_legs):
        """
        Apply offset.x as a yaw angle to the leg centre for a rotation gait.

        The generator must internalise the rotation transform so that visualisation and
        other consumers receive a world-frame point regardless of gait type.
        """
        gait_gen.current_gait = GaitType.tripod
        leg_center = Point3D([10.0, -5.0, 0.0])

        for leg in hexapod_legs:
            for phase in np.linspace(0.0, 1.0, 25):
                offset = gait_gen.get_offsets_at_phase_for_leg(leg, phase)
                expected = AffineTransform.from_rotvec(
                    [0, 0, offset.x], degrees=True
                ).apply_point(leg_center) + Point3D([0, 0, offset.z])
                tip = gait_gen.get_world_tip_at_phase_for_leg(
                    leg, phase, leg_center, rotation=True
                )
                assert tip == expected

    def _gait_stages(self, gait_gen, hexapod_legs, phase, prev_offsets):
        offsets = {}
        for leg in hexapod_legs:
            offsets[leg] = gait_gen.get_offsets_at_phase_for_leg(leg, phase)

        # Count how many legs are in swing phase and stance phase
        swing_legs = []
        stance_legs = []
        for leg in hexapod_legs:
            is_swing = is_swing_phase(offsets[leg])
            if is_swing:
                swing_legs += [leg]
            else:
                stance_legs += [leg]

            if leg in prev_offsets:
                was_swing = is_swing_phase(prev_offsets[leg])
                shift = offsets[leg] - prev_offsets[leg]
                all_swing = is_swing and was_swing
                all_stance = not is_swing and not was_swing
                if all_swing:
                    assert shift.x >= 0.0, (
                        f'Leg {leg} in swing phase is not moving forward. '
                        f'{prev_offsets[leg]=}, {offsets[leg]=}, {phase=}'
                    )
                elif all_stance:
                    assert shift.x <= 0.0, (
                        f'Leg {leg} in stance phase is not moving backward. '
                        f'{prev_offsets[leg]=}, {offsets[leg]=}, {phase=}'
                    )

        return swing_legs, stance_legs, offsets
