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

"""Tests for HexapodModel: structure, forward kinematics, and IK dispatch."""

from drqp_kinematics.geometry import AffineTransform
from drqp_kinematics.models import HexapodLeg, HexapodModel
import numpy as np
import pytest


@pytest.fixture
def model():
    """Return the default HexapodModel with standard link lengths."""
    return HexapodModel()


def test_hexapod_has_six_legs(model):
    """Assert that the model exposes exactly six legs."""
    assert len(list(model.legs)) == 6


def test_hexapod_named_legs_covers_all_six(model):
    """Assert that all six HexapodLeg enum values are present in named_legs."""
    expected = {
        HexapodLeg.left_front,
        HexapodLeg.left_middle,
        HexapodLeg.left_back,
        HexapodLeg.right_front,
        HexapodLeg.right_middle,
        HexapodLeg.right_back,
    }
    assert set(model.named_legs.keys()) == expected


def test_hexapod_leg_access_by_enum(model):
    """Individual legs are accessible via the HexapodLeg enum key."""
    lf = model.named_legs[HexapodLeg.left_front]
    assert lf.label == HexapodLeg.left_front


def test_forward_kinematics_updates_all_legs(model):
    """Calling forward_kinematics propagates joint angles to every leg."""
    model.forward_kinematics(10, 20, 30)
    for leg in model.legs:
        assert leg.coxa_angle == 10
        assert leg.femur_angle == 20
        assert leg.tibia_angle == 30


def test_move_legs_to_current_fk_positions(model):
    """
    Assert that every leg can reach its current FK-computed foot position.

    Uses a bent pose (beta=20, gamma=-40) rather than full extension to avoid
    float32 boundary errors where safe_arccos input numerically exceeds 1.
    """
    model.forward_kinematics(0, 20, -40)
    current_targets = [leg.tibia_end for leg in model.legs]
    results = model.move_legs_to(current_targets)
    assert all(results), 'All legs must reach their current FK positions'


def test_move_legs_to_returns_one_result_per_leg(model):
    """Assert that move_legs_to returns exactly one solvability flag per leg."""
    model.forward_kinematics(0, 20, -40)
    targets = [leg.tibia_end for leg in model.legs]
    results = model.move_legs_to(targets)
    assert len(results) == 6


def test_body_transform_setter_propagates_to_all_legs(model):
    """Setting body_transform updates every leg's transform."""
    new_tf = AffineTransform.from_translation([10.0, 0.0, 20.0])
    model.body_transform = new_tf
    for leg in model.legs:
        assert leg.body_transform is new_tf


def test_body_transform_getter_matches_left_front(model):
    """The body_transform getter mirrors the left_front leg's transform."""
    assert model.body_transform is model.left_front.body_transform


def test_head_updates_when_body_transform_changes(model):
    """Head line should be repositioned when body_transform is set."""
    original_head_start = np.array(model.head.start.numpy())
    model.body_transform = AffineTransform.from_translation([50.0, 0.0, 0.0])
    new_head_start = model.head.start.numpy()
    assert not np.allclose(original_head_start, new_head_start)


def test_left_right_symmetry_at_neutral():
    """Left and right legs should be mirror images in Y at neutral FK pose."""
    m = HexapodModel()
    m.forward_kinematics(0, 0, 0)
    lf = m.named_legs[HexapodLeg.left_front].tibia_end.numpy()
    rf = m.named_legs[HexapodLeg.right_front].tibia_end.numpy()
    # X and Z should match; Y should be negated
    assert np.allclose(lf[0], rf[0], atol=1e-2), 'X coordinate should match'
    assert np.allclose(lf[2], rf[2], atol=1e-2), 'Z coordinate should match'
    assert np.allclose(lf[1], -rf[1], atol=1e-2), 'Y should be negated'
