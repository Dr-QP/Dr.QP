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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Tests for pure, time-based walking target generation."""

import math

from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.stride_limits import DirectionalStrideLimits
from drqp_brain.walk_controller import SteeringState, WalkController
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodModel
import pytest


@pytest.fixture
def hexapod():
    """Create a default walking-pose model."""
    model = HexapodModel()
    model.forward_kinematics(0, -35, 130)
    return model


@pytest.fixture
def walker(hexapod):
    """Create a tripod walker with the calibrated cycle time."""
    return WalkController(hexapod, gait=GaitType.tripod, cycle_time_sec=1.25)


def _target_values(targets):
    return [(float(target.x), float(target.y), float(target.z)) for _, target in targets]


def test_targets_at_is_pure(walker, hexapod):
    """Repeated target evaluation must not mutate the walker or hexapod."""
    walker.advance(0.125, Point3D([1, 0, 0]), 0.4)
    before_walker = dict(walker.__dict__)
    before_transform = hexapod.body_transform.matrix.copy()
    steering = walker.steering

    first = walker.targets_at(
        0.35,
        steering,
        body_direction=Point3D([0.1, 0.2, 0.3]),
        body_rotation=Point3D([0.1, 0.2, 0.3]),
    )
    second = walker.targets_at(
        0.35,
        steering,
        body_direction=Point3D([0.1, 0.2, 0.3]),
        body_rotation=Point3D([0.1, 0.2, 0.3]),
    )

    assert _target_values(first) == pytest.approx(_target_values(second))
    assert walker.__dict__ == before_walker
    assert hexapod.body_transform.matrix == pytest.approx(before_transform)


def test_advance_uses_elapsed_time_and_only_advances_while_moving(walker):
    """Phase should advance by dt/cycle_time and freeze after commands decay."""
    walker.advance(0.125, Point3D([1, 0, 0]), 0.0)
    assert walker.current_phase == pytest.approx(0.125 / 1.25)

    for _ in range(30):
        walker.advance(0.125, Point3D([0, 0, 0]), 0.0)
    frozen_phase = walker.current_phase
    assert walker.current_direction == Point3D([0, 0, 0])

    walker.advance(0.125, Point3D([0, 0, 0]), 0.0)
    assert walker.current_phase == pytest.approx(frozen_phase)

    walker.advance(0.125, Point3D([1, 0, 0]), 0.0)
    assert walker.current_phase == pytest.approx((frozen_phase + 0.125 / 1.25) % 1.0)


def test_smoothing_is_time_constant_based(hexapod):
    """Two equal time steps and one combined step produce the same steering."""
    split = WalkController(hexapod, steering_tau_sec=0.35)
    combined = WalkController(HexapodModel(), steering_tau_sec=0.35)
    target = Point3D([0.8, -0.2, 0.0])

    split.advance(0.04, target, 0.6)
    split.advance(0.04, target, 0.6)
    combined.advance(0.08, target, 0.6)

    expected_alpha = 1.0 - math.exp(-0.08 / 0.35)
    assert split.current_direction == combined.current_direction
    assert split.current_rotation_direction == pytest.approx(combined.current_rotation_direction)
    assert combined.current_direction.x == pytest.approx(target.x * expected_alpha)
    assert combined.current_rotation_direction == pytest.approx(0.6 * expected_alpha)


def test_ramp_down_snaps_to_rest_and_returns_feet(walker):
    """Commanding zero decays steering to exactly rest and settles feet back."""
    rest_targets = _target_values(
        [(leg, tip) for leg, tip in walker.leg_tips_on_ground]
    )

    for _ in range(5):
        walker.advance(0.125, Point3D([1, 0, 0]), 1.0)
    assert walker.current_direction.x > WalkController._NO_MOTION_EPSILON
    assert walker.current_rotation_direction > WalkController._NO_MOTION_EPSILON

    for _ in range(60):
        walker.advance(0.125, Point3D([0, 0, 0]), 0.0)

    # Below the epsilon the smoothed command must snap to an exact rest state,
    # not merely a small residual that keeps the feet drifting.
    assert walker.current_direction == Point3D([0, 0, 0])
    assert walker.current_rotation_direction == 0.0

    settled = walker.targets_at(walker.current_phase, walker.steering)
    assert _target_values(settled) == pytest.approx(rest_targets)


@pytest.mark.parametrize(
    ('gait', 'cycle_time_sec'),
    [
        (GaitType.tripod, 1.25),
        (GaitType.ripple, 1.5625),
        (GaitType.wave, 2.5),
    ],
)
def test_calibrated_cycle_returns_targets_to_same_offsets(hexapod, gait, cycle_time_sec):
    """The calibrated time for each gait completes exactly one gait cycle."""
    local_walker = WalkController(
        hexapod,
        gait=gait,
        cycle_time_sec=cycle_time_sec,
    )
    steering = SteeringState(Point3D([1, 0, 0]), 0.0)

    initial = local_walker.targets_at(0.0, steering)
    local_walker.advance(cycle_time_sec, Point3D([1, 0, 0]), 0.0)
    completed = local_walker.targets_at(local_walker.current_phase, steering)

    assert local_walker.current_phase == pytest.approx(0.0)
    assert _target_values(completed) == pytest.approx(_target_values(initial))


def test_stride_limits_clamp_smoothed_direction(hexapod):
    """Steering clamping remains applied after time-based smoothing."""
    limits = DirectionalStrideLimits.from_dict(
        {
            'version': 1,
            'directions_count': 4,
            'joint_margin_degrees': 0.25,
            'gaits': {
                gait.name: [
                    {'angle_degrees': 0.0, 'max_step_length_m': 0.10},
                    {'angle_degrees': 90.0, 'max_step_length_m': 0.08},
                    {'angle_degrees': 180.0, 'max_step_length_m': 0.10},
                    {'angle_degrees': 270.0, 'max_step_length_m': 0.08},
                ]
                for gait in GaitType
            },
        }
    )
    local_walker = WalkController(
        hexapod,
        step_length=0.10,
        stride_limits=limits,
        gait=GaitType.tripod,
    )

    local_walker.advance(10.0, Point3D([0, 0.9, 0]), 0.0)

    assert local_walker.current_direction == Point3D([0, 0.8, 0])


def test_body_transform_is_explicit_and_does_not_mutate_hexapod(walker, hexapod):
    """Body pose construction is separate from pure target evaluation."""
    transform = walker.body_transform(
        Point3D([0.1, 0.2, 0.3]),
        Point3D([0.1, 0.2, 0.3]),
    )

    assert transform.translation == pytest.approx([0.1, 0.2, 0.3], abs=1e-3)
    assert transform.rotation == pytest.approx(
        AffineTransform.from_rotvec([0.1, 0.2, 0.3]).rotation,
        abs=1e-3,
    )
    assert hexapod.body_transform.matrix == pytest.approx(AffineTransform.identity().matrix)
