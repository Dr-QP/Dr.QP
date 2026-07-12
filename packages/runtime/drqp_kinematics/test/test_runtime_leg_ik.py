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

"""Tests for radians-native, limit-aware analytic leg IK."""

from drqp_kinematics.geometry import Point3D
from drqp_kinematics.models import HexapodLeg, LegModel
import numpy as np
import pytest

COXA = 5.0
FEMUR = 8.0
TIBIA = 10.0
JOINT_LIMITS_RAD = (
    (-np.pi / 2.0, np.pi / 2.0),
    (-np.pi / 2.0, np.pi / 2.0),
    (0.0, np.pi),
)


@pytest.fixture
def limited_leg():
    """Return an identity-frame leg with the physical solver branch limits."""
    return LegModel(
        coxa_length=COXA,
        femur_length=FEMUR,
        tibia_length=TIBIA,
        label=HexapodLeg.left_front,
        joint_limits_rad=JOINT_LIMITS_RAD,
    )


def _target_from_angles(leg: LegModel, angles_rad: tuple[float, float, float]) -> Point3D:
    return Point3D(leg.fk_foot_position(angles_rad))


@pytest.mark.parametrize(
    'angles_rad',
    [
        (-0.5, -0.5, 0.5),
        (0.0, -0.25, 1.0),
        (0.5, 0.25, 1.5),
    ],
)
def test_solve_ik_round_trips_in_limit_fk_targets(limited_leg, angles_rad):
    """IK returns the original reachable target for the supported elbow branch."""
    target = _target_from_angles(limited_leg, angles_rad)

    solution = limited_leg.solve_ik(target)

    assert solution.reachable
    assert solution.within_limits
    assert np.allclose(
        limited_leg.fk_foot_position(solution.angles_rad),
        target.numpy(),
        atol=1e-5,
    )
    assert solution.clamped_target == target


@pytest.mark.parametrize(
    'angles_rad',
    [
        (-0.5, -0.5, 0.5),
        (0.0, -0.25, 1.0),
        (0.5, 0.25, 1.5),
    ],
)
def test_solve_ik_round_trips_angles_on_the_supported_elbow_branch(limited_leg, angles_rad):
    """The positive-tibia-angle elbow branch maps FK angles back to themselves."""
    solution = limited_leg.solve_ik(_target_from_angles(limited_leg, angles_rad))

    assert solution.angles_rad == pytest.approx(angles_rad, abs=1e-5)


@pytest.mark.parametrize(
    'target, expected_span',
    [
        ([100.0, 0.0, 0.0], FEMUR + TIBIA),
        ([COXA, 0.0, 0.0], abs(FEMUR - TIBIA)),
    ],
)
def test_solve_ik_clamps_annulus_targets_to_fk_consistent_boundary(
    limited_leg,
    target,
    expected_span,
):
    """Too-far and too-close targets clamp to the planar annulus boundary."""
    solution = limited_leg.solve_ik(Point3D(target))
    clamped_local = limited_leg.to_local(solution.clamped_target)
    clamped_span = np.hypot(
        np.hypot(clamped_local.x, clamped_local.y) - COXA,
        -clamped_local.z,
    )

    assert not solution.reachable
    assert np.all(np.isfinite(solution.angles_rad))
    for angle, (lower, upper) in zip(solution.angles_rad, JOINT_LIMITS_RAD):
        assert lower <= angle <= upper
    assert clamped_span == pytest.approx(expected_span, abs=1e-5)
    assert np.allclose(
        limited_leg.fk_foot_position(solution.angles_rad),
        solution.clamped_target.numpy(),
        atol=1e-5,
    )


def test_solve_ik_clamps_out_of_range_coxa_yaw(limited_leg):
    """A target beyond the coxa yaw envelope rotates onto its yaw boundary."""
    limited_leg.joint_limits_rad = ((-0.2, 0.2), *JOINT_LIMITS_RAD[1:])

    solution = limited_leg.solve_ik(Point3D([10.0, 10.0, 0.0]))

    assert not solution.reachable
    assert not solution.within_limits
    assert solution.angles_rad[0] == pytest.approx(0.2)
    assert np.allclose(
        limited_leg.fk_foot_position(solution.angles_rad),
        solution.clamped_target.numpy(),
        atol=1e-5,
    )


def test_solve_ik_clamps_joint_limited_solution_to_fk_position(limited_leg):
    """An otherwise reachable target reports the limit violation and clamps it."""
    limited_leg.joint_limits_rad = (
        JOINT_LIMITS_RAD[0],
        JOINT_LIMITS_RAD[1],
        (0.0, 0.2),
    )
    target = _target_from_angles(limited_leg, (0.0, -0.25, 1.0))

    solution = limited_leg.solve_ik(target)

    assert solution.reachable
    assert not solution.within_limits
    assert solution.angles_rad[2] == pytest.approx(0.2)
    assert solution.limit_margin_rad == pytest.approx(0.0)
    assert np.allclose(
        limited_leg.fk_foot_position(solution.angles_rad),
        solution.clamped_target.numpy(),
        atol=1e-5,
    )
