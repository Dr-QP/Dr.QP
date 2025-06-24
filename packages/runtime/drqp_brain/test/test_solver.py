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

from drqp_brain.geometry.point import Point3D
from drqp_brain.models import HexapodLeg, LegModel, safe_arccos
import pytest


def test_safe_arccos():
    # Test valid inputs
    assert safe_arccos(1.0) == (True, 0.0)
    assert safe_arccos(-1.0) == (True, math.pi)
    assert safe_arccos(0.0) == (True, math.pi / 2)

    # Test clamping
    assert safe_arccos(1.1) == (False, 0)
    assert safe_arccos(-1.1) == (False, math.pi)


class TestSolver:
    """Test the Solver class."""

    @pytest.fixture
    def leg_model(self):
        # Standard dimensions for a medium-sized leg
        return LegModel(
            coxa_length=20, femur_length=50, tibia_length=70, label=HexapodLeg.left_front
        )

    def test_initialization(self, leg_model):
        assert leg_model.coxa_length == 20
        assert leg_model.femur_length == 50
        assert leg_model.tibia_length == 70

    def test_forward_position(self, leg_model):
        # Test reaching forward
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([100, 0, -10]))

        assert success is True
        assert math.isclose(alpha, 0, abs_tol=0.1)
        assert math.isclose(beta, -52.3, abs_tol=0.1)
        assert math.isclose(gamma, 97.3, abs_tol=0.1)

    def test_side_position(self, leg_model):
        # Test reaching to the side
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([100, 20, -10]))
        assert success is True
        assert math.isclose(alpha, 11.3, abs_tol=0.1)
        assert math.isclose(beta, -50.6, abs_tol=0.1)
        assert math.isclose(gamma, 94.7, abs_tol=0.1)

    def test_unreachable_position(self, leg_model):
        # Test position that's too far to reach
        success, alpha, beta, gamma = leg_model.inverse_kinematics(Point3D([1000, 1000, -1000]))
        assert success is False
        assert math.isclose(alpha, 45.0, abs_tol=0.1)
        assert math.isclose(beta, 35.6, abs_tol=0.1)
        assert math.isclose(gamma, 0.0, abs_tol=0.1)
