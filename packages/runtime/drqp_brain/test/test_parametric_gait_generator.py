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

from drqp_brain.models import HexapodLeg
from drqp_brain.parametric_gait_generator import GaitType, ParametricGaitGenerator
import numpy as np
import pytest


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

    def _gait_stages(self, gait_gen, hexapod_legs, phase):
        offsets = {}
        for leg in hexapod_legs:
            offsets[leg] = gait_gen.get_offsets_at_phase_for_leg(leg, phase)

        # Count how many legs are in swing phase and stance phase
        swing_count = 0
        stance_count = 0
        for leg in hexapod_legs:
            if offsets[leg].z > 0.0001:
                swing_count += 1
            else:
                stance_count += 1
        return swing_count, stance_count

    def test_tripod(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for tripod gait."""
        gait_gen.current_gait = GaitType.tripod

        on_ground = np.array([0.0, 0.5, 1.0])
        for phase in np.arange(0.0, 1.0, 0.01):
            swing_count, stance_count = self._gait_stages(gait_gen, hexapod_legs, phase)

            if phase in on_ground:
                assert swing_count == 0, f'Phase: {phase}'
                assert stance_count == 6, f'Phase: {phase}'
            else:
                assert swing_count == 3, f'Phase: {phase}'
                assert stance_count == 3, f'Phase: {phase}'

    def test_wave(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for wave gait."""
        gait_gen.current_gait = GaitType.wave

        on_ground = np.array([0.0, 1 / 6, 2 / 6, 3 / 6, 4 / 6, 5 / 6, 1.0])
        for phase in np.arange(0.0, 1.0, 0.01):
            swing_count, stance_count = self._gait_stages(gait_gen, hexapod_legs, phase)

            if phase in on_ground:
                assert swing_count == 0, f'Phase: {phase}'
                assert stance_count == 6, f'Phase: {phase}'
            else:
                assert swing_count == 1, f'Phase: {phase}'
                assert stance_count == 5, f'Phase: {phase}'

    def test_ripple(self, gait_gen, hexapod_legs):
        """Test that offsets are calculated correctly for ripple gait."""
        gait_gen.current_gait = GaitType.ripple
        on_ground = np.array([0.0, 1 / 6, 2 / 6, 3 / 6, 4 / 6, 5 / 6, 1.0])
        for phase in np.arange(0.0, 1.0, 0.01):
            swing_count, stance_count = self._gait_stages(gait_gen, hexapod_legs, phase)
            if phase in on_ground:
                assert swing_count == 1, f'Phase: {phase}'
                assert stance_count == 5, f'Phase: {phase}'
            else:
                assert swing_count == 2, f'Phase: {phase}'
                assert stance_count == 4, f'Phase: {phase}'
