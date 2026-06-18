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

"""Tests for LegModel forward kinematics."""

from drqp_kinematics.models import HexapodLeg, LegModel
import numpy as np
import pytest

COXA = 5.0
FEMUR = 8.0
TIBIA = 10.0


@pytest.fixture
def simple_leg():
    """Leg placed at body origin with no rotation, pointing along +X."""
    return LegModel(
        coxa_length=COXA,
        femur_length=FEMUR,
        tibia_length=TIBIA,
        label=HexapodLeg.left_front,
        location_on_body=[0, 0, 0],
        rotation=[0, 0, 0],
    )


def test_neutral_foot_along_x_axis(simple_leg):
    """At all-zero angles the foot lands at (coxa+femur+tibia, 0, 0)."""
    simple_leg.forward_kinematics(0, 0, 0)
    expected = [COXA + FEMUR + TIBIA, 0.0, 0.0]
    assert np.allclose(simple_leg.tibia_end.numpy(), expected, atol=1e-3)


def test_neutral_coxa_end_position(simple_leg):
    """At alpha=0 the coxa end is at (coxa_length, 0, 0)."""
    simple_leg.forward_kinematics(0, 0, 0)
    assert np.allclose(simple_leg.coxa_end.numpy(), [COXA, 0.0, 0.0], atol=1e-3)


def test_neutral_femur_end_position(simple_leg):
    """At alpha=0, beta=0 the femur end is at (coxa+femur, 0, 0)."""
    simple_leg.forward_kinematics(0, 0, 0)
    assert np.allclose(simple_leg.femur_end.numpy(), [COXA + FEMUR, 0.0, 0.0], atol=1e-3)


def test_alpha_90_swings_foot_to_y_axis(simple_leg):
    """alpha=90° rotates entire leg around Z; foot lands at (0, total_length, 0)."""
    simple_leg.forward_kinematics(90, 0, 0)
    expected = [0.0, COXA + FEMUR + TIBIA, 0.0]
    assert np.allclose(simple_leg.tibia_end.numpy(), expected, atol=1e-2)


def test_alpha_minus_90_swings_foot_to_minus_y_axis(simple_leg):
    """alpha=-90° swings foot to the negative Y side."""
    simple_leg.forward_kinematics(-90, 0, 0)
    expected = [0.0, -(COXA + FEMUR + TIBIA), 0.0]
    assert np.allclose(simple_leg.tibia_end.numpy(), expected, atol=1e-2)


def test_beta_90_drops_femur_end_below_coxa(simple_leg):
    """beta=90° rotates femur about Y; femur_end drops to (coxa, 0, -femur)."""
    simple_leg.forward_kinematics(0, 90, 0)
    # Ry(+90°): +X axis tilts toward -Z
    assert np.allclose(simple_leg.femur_end.numpy(), [COXA, 0.0, -FEMUR], atol=1e-2)


def test_joint_angles_stored_after_forward_kinematics(simple_leg):
    """Applied joint angles are readable as instance attributes."""
    simple_leg.forward_kinematics(15, -30, 45)
    assert simple_leg.coxa_angle == 15
    assert simple_leg.femur_angle == -30
    assert simple_leg.tibia_angle == 45


def test_lines_property_returns_four_named_segments(simple_leg):
    """The leg exposes four line segments: Body, Coxa, Femur, Tibia."""
    simple_leg.forward_kinematics(0, 0, 0)
    lines = simple_leg.lines
    assert len(lines) == 4
    labels = [line.label for line in lines]
    assert labels == ['Body', 'Coxa', 'Femur', 'Tibia']


def test_body_start_equals_body_transform_origin(simple_leg):
    """body_start should be at the body transform origin (identity → origin)."""
    simple_leg.forward_kinematics(0, 0, 0)
    assert np.allclose(simple_leg.body_start.numpy(), [0.0, 0.0, 0.0], atol=1e-3)
