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
from types import SimpleNamespace

from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.walk_controller import SteeringState, WalkController
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodModel
import numpy as np
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
    combined_hexapod = HexapodModel()
    combined_hexapod.forward_kinematics(0, -35, 130)
    combined = WalkController(combined_hexapod, steering_tau_sec=0.35)
    target = Point3D([0.8, -0.2, 0.0])

    split.advance(0.04, target, 0.6)
    split.advance(0.04, target, 0.6)
    combined.advance(0.08, target, 0.6)

    expected_alpha = 1.0 - math.exp(-0.08 / 0.35)
    assert split.current_direction == combined.current_direction
    assert split.current_rotation_direction == pytest.approx(combined.current_rotation_direction)
    expected_linear_speed = combined.step_length / combined.stance_duration_sec
    assert combined.current_direction.x == pytest.approx(
        target.x * expected_linear_speed * expected_alpha
    )
    assert combined.current_rotation_direction == pytest.approx(
        0.6 * combined.omega_max_rad_sec * expected_alpha
    )


def test_ramp_down_snaps_to_rest_and_returns_feet(walker):
    """Commanding zero decays steering to exactly rest and settles feet back."""
    rest_targets = _target_values([(leg, tip) for leg, tip in walker.leg_tips_on_ground])

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
    assert np.array(_target_values(settled)) == pytest.approx(np.array(rest_targets))


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


def _integrate_twist(linear_velocity, angular_velocity, duration, stride_offset, steps=10_000):
    """Numerically integrate the SE(2) body twist to an offset on the stride."""
    total_time = stride_offset * duration
    dt = total_time / steps
    heading = 0.0
    position = [0.0, 0.0]
    for _ in range(steps):
        position[0] += (
            linear_velocity.x * math.cos(heading) - linear_velocity.y * math.sin(heading)
        ) * dt
        position[1] += (
            linear_velocity.x * math.sin(heading) + linear_velocity.y * math.cos(heading)
        ) * dt
        heading += angular_velocity * dt
    return heading, position


def test_twist_defaults_preserve_legacy_yaw_per_stance(walker):
    """The default angular limit derives one legacy turn increment per stance."""
    expected = math.radians(walker.rotation_speed_degrees) / walker.stance_duration_sec

    assert walker.omega_max_rad_sec == pytest.approx(expected)


def test_pure_translation_follows_the_straight_line_stride_path(walker):
    """Zero yaw reduces the SE(2) target to the straight-line stride path."""
    neutral_foot = walker.leg_tips_on_ground[0][1]
    steering = SteeringState(Point3D([0.08, -0.02, 0.0]), 0.0)
    stride_offset = 0.37

    target = walker._foot_target_for_stride_offset(neutral_foot, stride_offset, steering)
    expected = Point3D(
        [
            neutral_foot.x
            - stride_offset * steering.linear_velocity.x * walker.stance_duration_sec,
            neutral_foot.y
            - stride_offset * steering.linear_velocity.y * walker.stance_duration_sec,
            neutral_foot.z,
        ]
    )

    assert target == expected


def test_pure_rotation_follows_the_legacy_yaw_per_stance(walker):
    """Zero translation rotates a neutral foot by the derived stance yaw."""
    neutral_foot = walker.leg_tips_on_ground[0][1]
    steering = SteeringState(Point3D([0.0, 0.0, 0.0]), walker.omega_max_rad_sec)
    stride_offset = -0.5
    theta = stride_offset * math.radians(walker.rotation_speed_degrees)

    target = walker._foot_target_for_stride_offset(neutral_foot, stride_offset, steering)

    assert (target.x, target.y, target.z) == pytest.approx(
        (
            math.cos(theta) * neutral_foot.x + math.sin(theta) * neutral_foot.y,
            -math.sin(theta) * neutral_foot.x + math.cos(theta) * neutral_foot.y,
            neutral_foot.z,
        )
    )


def test_combined_twist_keeps_stance_foot_world_fixed(walker):
    """Closed-form targets agree with an independently integrated body twist."""
    steering = SteeringState(Point3D([0.07, -0.03, 0.0]), 0.9)
    leg, neutral_foot = walker.leg_tips_on_ground[0]

    for stride_offset in (-0.5, -0.23, 0.0, 0.31, 0.5):
        target = walker._foot_target_for_stride_offset(neutral_foot, stride_offset, steering)
        heading, position = _integrate_twist(
            steering.linear_velocity,
            steering.angular_velocity,
            walker.stance_duration_sec,
            stride_offset,
        )
        world_x = math.cos(heading) * target.x - math.sin(heading) * target.y + position[0]
        world_y = math.sin(heading) * target.x + math.cos(heading) * target.y + position[1]

        assert (world_x, world_y, target.z) == pytest.approx(
            (neutral_foot.x, neutral_foot.y, neutral_foot.z), abs=2e-5
        )


def test_command_speed_is_isotropic_across_joystick_directions(hexapod):
    """Equal Euclidean joystick magnitudes map to equal planar twist speeds."""
    local_walker = WalkController(hexapod, gait=GaitType.tripod)
    speeds = []
    for index in range(16):
        angle = 2.0 * math.pi * index / 16
        twist = local_walker.command_to_twist(Point3D([math.cos(angle), math.sin(angle), 0]), 0.0)
        speeds.append(math.hypot(twist.linear_velocity.x, twist.linear_velocity.y))

    assert speeds == pytest.approx([speeds[0]] * len(speeds))


def test_saturation_scales_combined_twist_once_for_an_interior_path_limit(walker, monkeypatch):
    """One scalar makes every sampled path target reachable, including interior samples."""
    endpoint_solvers = []
    for leg, neutral_foot in walker.leg_tips_on_ground:

        def solve_ik(target, *, clamp, neutral_foot=neutral_foot):
            del clamp
            # Stance endpoints remain reachable. A swing-path-only workspace
            # cut exercises the interior samples that endpoint checks omit.
            if target.z <= neutral_foot.z + 1e-6:
                return SimpleNamespace(reachable=True, within_limits=True)
            displacement = math.hypot(
                float(target.x - neutral_foot.x),
                float(target.y - neutral_foot.y),
            )
            reachable = displacement <= 0.04
            return SimpleNamespace(reachable=reachable, within_limits=reachable)

        monkeypatch.setattr(leg, 'solve_ik', solve_ik)
        endpoint_solvers.append((neutral_foot, solve_ik))

    proposed = SteeringState(Point3D([0.14, 0.14, 0.0]), 2.5)
    for neutral_foot, solve_ik in endpoint_solvers:
        for stride_offset in (-0.5, 0.5):
            endpoint = walker._foot_target_for_stride_offset(
                neutral_foot,
                stride_offset,
                proposed,
            )
            assert solve_ik(endpoint, clamp=False).reachable

    assert not walker._twist_is_reachable(proposed)
    saturated = walker._saturate_twist(proposed)

    assert 0.0 < saturated.scale < 1.0
    assert saturated.linear_velocity.x / proposed.linear_velocity.x == pytest.approx(
        saturated.scale
    )
    assert saturated.angular_velocity / proposed.angular_velocity == pytest.approx(saturated.scale)
    assert walker._twist_is_reachable(saturated.twist)
