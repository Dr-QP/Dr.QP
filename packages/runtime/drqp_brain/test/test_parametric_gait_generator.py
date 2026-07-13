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
from drqp_kinematics.geometry import Point3D
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

    def test_swing_profile_preserves_endpoints_and_apex(self, gait_gen):
        """Swing follows the specified cycloid endpoints and apex."""
        swing_duration = gait_gen.gaits[GaitType.wave].swing_duration
        half_step = gait_gen.step_length / 2

        liftoff = self._offset_at_swing_time(gait_gen, 0.0)
        apex = self._offset_at_swing_time(gait_gen, 0.5)
        touchdown = self._offset_at_swing_time(gait_gen, 1.0)

        assert liftoff.x == pytest.approx(-half_step)
        assert touchdown.x == pytest.approx(half_step)
        assert liftoff.z == pytest.approx(0.0)
        assert touchdown.z == pytest.approx(0.0)
        assert apex.z == pytest.approx(gait_gen.step_height)
        assert swing_duration == pytest.approx(1 / 6)

    def test_swing_profile_has_near_zero_boundary_velocity(self, gait_gen):
        """Cycloid feet begin and end swing much slower than mid-swing."""
        first = self._offset_at_swing_time(gait_gen, 0.0)
        near_first = self._offset_at_swing_time(gait_gen, 0.01)
        near_last = self._offset_at_swing_time(gait_gen, 0.99)
        last = self._offset_at_swing_time(gait_gen, 1.0)

        x_mid_before = self._offset_at_swing_time(gait_gen, 0.49)
        x_mid_after = self._offset_at_swing_time(gait_gen, 0.51)
        z_peak_before = self._offset_at_swing_time(gait_gen, 0.24)
        z_peak_after = self._offset_at_swing_time(gait_gen, 0.26)

        boundary_x_speeds = [
            abs((near_first.x - first.x) / 0.01),
            abs((last.x - near_last.x) / 0.01),
        ]
        boundary_z_speeds = [
            abs((near_first.z - first.z) / 0.01),
            abs((last.z - near_last.z) / 0.01),
        ]
        mid_x_speed = abs((x_mid_after.x - x_mid_before.x) / 0.02)
        peak_z_speed = abs((z_peak_after.z - z_peak_before.z) / 0.02)

        assert max(boundary_x_speeds) < mid_x_speed * 0.05
        assert max(boundary_z_speeds) < peak_z_speed * 0.05

    def test_swing_profile_is_monotonic_and_stance_stays_linear(self, gait_gen):
        """Cycloid swing moves forward while the stance regression stays linear."""
        swing_times = np.linspace(0.0, 1.0, 101)
        swing_offsets = [self._offset_at_swing_time(gait_gen, time) for time in swing_times]
        assert all(
            following.x > previous.x
            for previous, following in zip(swing_offsets, swing_offsets[1:])
        )

        gait = gait_gen.gaits[GaitType.wave]
        half_step = gait_gen.step_length / 2
        for phase in (gait.swing_duration, 0.25, 0.5, 0.75):
            offset = gait_gen.get_offsets_at_phase_for_leg(HexapodLeg.right_back, phase)
            expected_x = np.interp(
                phase,
                [gait.swing_duration, 1.0],
                [half_step, -half_step],
            )
            assert offset.label == 'stance'
            assert offset.x == pytest.approx(expected_x)
            assert offset.z == pytest.approx(0.0)

    def test_swing_and_stance_positions_converge_at_the_boundary(self, gait_gen):
        """Swing-to-stance position discontinuity approaches zero when sampled."""
        gait = gait_gen.gaits[GaitType.wave]
        distances = []
        for samples in (100, 1_000):
            last_swing_phase = gait.swing_duration * (1.0 - 1.0 / samples)
            first_stance_phase = gait.swing_duration
            last_swing = gait_gen.get_offsets_at_phase_for_leg(
                HexapodLeg.right_back, last_swing_phase
            )
            first_stance = gait_gen.get_offsets_at_phase_for_leg(
                HexapodLeg.right_back, first_stance_phase
            )
            distances.append(abs(first_stance.x - last_swing.x))

        assert distances[1] < distances[0]
        assert distances[1] < 0.001

    def test_phase_wraparound_uses_a_half_open_unit_interval(self, gait_gen):
        """Equivalent and negative phases wrap safely into the same gait state."""
        leg = HexapodLeg.right_back
        epsilon = 1e-9
        offsets = [
            gait_gen.get_offsets_at_phase_for_leg(leg, phase)
            for phase in (0.0, 1.0, 1.0 + epsilon)
        ]
        epsilon_offset = gait_gen.get_offsets_at_phase_for_leg(leg, epsilon)
        negative_phase = gait_gen.get_offsets_at_phase_for_leg(leg, -0.5)
        wrapped_phase = gait_gen.get_offsets_at_phase_for_leg(leg, 0.5)

        assert [offset.label for offset in offsets] == ['swing'] * 3
        assert (offsets[0].x, offsets[0].z) == pytest.approx((offsets[1].x, offsets[1].z))
        assert (offsets[2].x, offsets[2].z) == pytest.approx((epsilon_offset.x, epsilon_offset.z))
        assert negative_phase.label == wrapped_phase.label
        assert (negative_phase.x, negative_phase.z) == pytest.approx(
            (wrapped_phase.x, wrapped_phase.z)
        )

    @staticmethod
    def _offset_at_swing_time(gait_gen, time):
        gait = gait_gen.gaits[GaitType.wave]
        return gait_gen.get_offsets_at_phase_for_leg(
            HexapodLeg.right_back,
            time * gait.swing_duration,
        )

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
