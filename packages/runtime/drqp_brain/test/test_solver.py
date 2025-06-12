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

import math

import pytest
from drqp_brain.geometry.point import Point3D
from drqp_brain.models import safe_arccos, LegModel


def test_safe_arccos():
    # Test valid inputs
    assert safe_arccos(1.0) == (True, 0.0)
    assert safe_arccos(-1.0) == (True, math.pi)
    assert safe_arccos(0.0) == (True, math.pi / 2)

    # Test safe zone extensions
    assert safe_arccos(1.01) == (True, 0.0)
    assert safe_arccos(-1.01) == (True, math.pi)

    # Test clamping
    assert safe_arccos(1.1) == (False, 0)
    assert safe_arccos(-1.1) == (False, 0)


class TestSolver:
    """Test the Solver class."""

    @pytest.fixture
    def leg_model(self):
        # Standard dimensions for a medium-sized leg
        return LegModel(coxa_length=20, femur_length=50, tibia_length=70)

    def test_initialization(self, leg_model):
        assert leg_model.coxa_length == 20
        assert leg_model.femur_length == 50
        assert leg_model.tibia_length == 70

    def test_forward_position(self, leg_model):
        # Test reaching forward
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([100, 0, -10]))

        assert success is True
        assert math.pi / 2 <= alpha <= math.pi
        assert math.pi / 4 <= beta <= math.pi / 2
        assert abs(gamma) < 0.001  # Should be close to 0 for y=0

    def test_side_position(self, leg_model):
        # Test reaching to the side
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([100, 20, -10]))
        assert success is True
        assert math.pi / 2 <= alpha <= math.pi
        assert math.pi / 4 <= beta <= math.pi / 2
        assert 0 <= gamma <= math.pi / 4

    def test_unreachable_position(self, leg_model):
        # Test position that's too far to reach
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([1000, 1000, -1000]))
        assert success is False
