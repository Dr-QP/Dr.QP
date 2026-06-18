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

"""
Tests for LegModel inverse kinematics and IK/FK round-trip accuracy.

The core property under test is: given a reachable foot target, IK computes
joint angles such that FK places the foot at (or very close to) that target.
This mirrors the 'drawing a circle' validation used in the getting-started
notebook (docs/source/notebooks/1_getting_started_with_robot_ik.ipynb).
"""

from drqp_kinematics.geometry import Point3D
from drqp_kinematics.models import HexapodLeg, LegModel
import numpy as np
import pytest

COXA = 5.0
FEMUR = 8.0
TIBIA = 10.0
# Max reach with all links fully extended:  COXA + FEMUR + TIBIA = 23
# Min reach beyond coxa (femur - tibia = -2):  COXA + |FEMUR - TIBIA| = 7


@pytest.fixture
def simple_leg():
    """Leg at body origin with identity rotation, pointing along +X."""
    return LegModel(
        coxa_length=COXA,
        femur_length=FEMUR,
        tibia_length=TIBIA,
        label=HexapodLeg.left_front,
        location_on_body=[0, 0, 0],
        rotation=[0, 0, 0],
    )


@pytest.mark.parametrize(
    'target',
    [
        [15.0, 0.0, 0.0],  # straight forward, on X axis
        [0.0, 15.0, 0.0],  # pure lateral (tests alpha=90° path)
        [10.0, 0.0, -5.0],  # forward and below ground plane
        [10.0, 8.0, -5.0],  # full 3D target
        [18.0, 0.0, 0.0],  # long reach (near maximum extension)
        [8.0, 6.0, -3.0],  # compact, bent position
    ],
)
def test_ik_fk_roundtrip(simple_leg, target):
    """IK followed by FK places the foot within 0.1 units of the target."""
    point = Point3D(target)
    reached = simple_leg.move_to(point)
    assert reached, f'Target {target} should be reachable'
    assert np.allclose(simple_leg.tibia_end.numpy(), target, atol=0.1)


def test_unreachable_target_returns_false(simple_leg):
    """A target far beyond maximum leg reach reports solvable=False."""
    far_target = Point3D([100.0, 0.0, 0.0])
    reached = simple_leg.move_to(far_target)
    assert not reached


def test_unreachable_target_below_minimum_reach_returns_false(simple_leg):
    """A target too close to the coxa joint (below min reach) is unsolvable."""
    # Place target so x_tick < |femur - tibia| = 2; e.g. directly above coxa end
    too_close = Point3D([COXA, 0.0, 0.1])
    reached = simple_leg.move_to(too_close)
    assert not reached


def test_ik_straight_leg_gives_zero_angles(simple_leg):
    """IK on a fully-extended target (coxa+femur+tibia, 0, 0) yields β=γ=0."""
    target = Point3D([COXA + FEMUR + TIBIA, 0.0, 0.0])
    reached, alpha, beta, gamma = simple_leg.inverse_kinematics(target)
    assert reached
    assert abs(alpha) < 1e-3
    assert abs(beta) < 1e-3
    assert abs(gamma) < 1e-3


def test_coxa_ik_angle_matches_arctan2(simple_leg):
    """Alpha from IK equals arctan2(y, x) of the foot target in local frame."""
    target = Point3D([10.0, 10.0, 0.0])
    reached, alpha, _beta, _gamma = simple_leg.inverse_kinematics(target)
    assert reached
    expected_alpha = np.degrees(np.arctan2(10.0, 10.0))  # 45°
    assert abs(alpha - expected_alpha) < 1e-3


def test_ik_traces_circle_in_xz_plane(simple_leg):
    """IK tracks a full circle in the XZ plane; max foot error stays below 0.1."""
    steps = 32
    x_center, z_center, radius = 15.0, -5.0, 2.0

    max_error = 0.0
    for i in range(steps):
        angle = 2 * np.pi * i / steps
        target = Point3D(
            [
                x_center + np.sin(angle) * radius,
                0.0,
                z_center - np.cos(angle) * radius,
            ]
        )
        reached = simple_leg.move_to(target)
        assert reached, f'Circle step {i} should be reachable'
        error = np.linalg.norm(simple_leg.tibia_end.numpy() - target.numpy())
        max_error = max(max_error, float(error))

    assert max_error < 0.1, f'Circle tracking error {max_error:.4f} exceeds tolerance'
