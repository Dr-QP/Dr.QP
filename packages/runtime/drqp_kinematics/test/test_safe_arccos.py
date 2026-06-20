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

"""Tests for the safe_arccos clamping utility."""

import math

from drqp_kinematics.models import safe_arccos
import pytest


def test_zero_returns_pi_halves():
    """arccos(0) = π/2, a valid in-domain value."""
    solvable, angle = safe_arccos(0.0)
    assert solvable is True
    assert math.isclose(angle, math.pi / 2)


def test_one_returns_zero():
    """arccos(1) = 0; the upper boundary of the domain is valid."""
    solvable, angle = safe_arccos(1.0)
    assert solvable is True
    assert math.isclose(angle, 0.0)


def test_minus_one_returns_pi():
    """arccos(-1) = π; the lower boundary of the domain is valid."""
    solvable, angle = safe_arccos(-1.0)
    assert solvable is True
    assert math.isclose(angle, math.pi)


def test_above_one_is_unsolvable():
    """Input > 1 is out of arccos domain; solvable is False and angle clamps to 0."""
    solvable, angle = safe_arccos(1.5)
    assert solvable is False
    assert math.isclose(angle, 0.0)


def test_below_minus_one_is_unsolvable():
    """Input < -1 is out of arccos domain; solvable is False and angle clamps to π."""
    solvable, angle = safe_arccos(-2.0)
    assert solvable is False
    assert math.isclose(angle, math.pi)


@pytest.mark.parametrize(
    'value,expected_deg',
    [
        (0.5, 60.0),
        (-0.5, 120.0),
        (math.sqrt(2) / 2, 45.0),
        (math.sqrt(3) / 2, 30.0),
    ],
)
def test_standard_values(value, expected_deg):
    """Common arccos values should be returned with high accuracy."""
    solvable, angle = safe_arccos(value)
    assert solvable is True
    assert math.isclose(math.degrees(angle), expected_deg, abs_tol=1e-4)
