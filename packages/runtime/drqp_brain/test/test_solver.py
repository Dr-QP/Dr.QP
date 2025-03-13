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

from drqp_brain.solver import safe_acos, Solver
import pytest


def test_safe_acos():
    # Test valid inputs
    assert safe_acos(1.0) == (True, 0.0)
    assert safe_acos(-1.0) == (True, math.pi)
    assert safe_acos(0.0) == (True, math.pi / 2)

    # Test safe zone extensions
    assert safe_acos(1.01) == (True, 0.0)
    assert safe_acos(-1.01) == (True, math.pi)

    # Test clamping
    assert safe_acos(1.1) == (False, 0)
    assert safe_acos(-1.1) == (False, 0)


class TestSolver:
    """Test the Solver class."""

    @pytest.fixture
    def solver(self):
        # Standard dimensions for a medium-sized leg
        return Solver(coxa=79, femur=128, tibia=167, logger=None)

    def test_initialization(self, solver):
        assert solver.coxa == 79
        assert solver.femur == 128
        assert solver.tibia == 167
        assert solver.logger is not None

    def test_forward_position(self, solver):
        # Test reaching forward
        success, alpha, beta, gamma = solver.solve(x=200, y=0, z=-100)
        assert success is True
        assert -math.pi / 2 <= alpha <= math.pi / 2
        assert 0 <= beta <= math.pi
        assert abs(gamma) < 0.001  # Should be close to 0 for y=0

    def test_side_position(self, solver):
        # Test reaching to the side
        success, alpha, beta, gamma = solver.solve(x=0, y=200, z=-100)
        assert success is True
        assert -math.pi / 2 <= alpha <= math.pi / 2
        assert 0 <= beta <= math.pi
        assert abs(gamma - math.pi / 2) < 0.001  # Should be close to 90 degrees

    def test_unreachable_position(self, solver):
        # Test position that's too far to reach
        success, alpha, beta, gamma = solver.solve(x=1000, y=1000, z=-1000)
        assert success is False

    @pytest.mark.parametrize(
        'x,y,z,expected_success',
        [
            (374, 0, -100, True),  # Maximum reach forward
            (0, 374, -100, True),  # Maximum reach sideways
            (264, 264, -100, True),  # Maximum reach at 45 degrees
            (400, 0, -100, False),  # Just beyond maximum reach
            (0, 0, 0, True),  # Straight up
            (79, 0, 0, True),  # Coxa length forward
        ],
    )
    def test_various_positions(self, solver, x, y, z, expected_success):
        success, alpha, beta, gamma = solver.solve(x, y, z)
        assert success == expected_success
        if success:
            assert -math.pi <= alpha <= math.pi
            assert 0 <= beta <= math.pi
            assert -math.pi <= gamma <= math.pi

    def test_zero_position(self, solver):
        # Test position at origin (should be possible but might be a special case)
        success, alpha, beta, gamma = solver.solve(x=0, y=0, z=-100)
        assert success is True
        assert -math.pi / 2 <= alpha <= math.pi / 2
        assert 0 <= beta <= math.pi
        assert abs(gamma) < 0.001  # Should be close to 0
