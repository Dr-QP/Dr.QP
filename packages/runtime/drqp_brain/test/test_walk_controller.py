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

import pytest
from drqp_brain.geometry import Point3D
from drqp_brain.walk_controller import WalkController
from drqp_brain.models import HexapodModel
from drqp_brain.parametric_gait_generator import GaitType


class TestWalkController:
    """Test the WalkController class."""

    @pytest.fixture
    def hexapod(self):
        return HexapodModel()

    @pytest.fixture
    def walker(self, hexapod):
        return WalkController(hexapod)

    def test_initialization(self, walker):
        assert walker.hexapod is not None
        assert walker.step_length == 60.0
        assert walker.step_height == 40.0
        assert walker.rotation_speed_degrees == 10.0
        assert walker.gait_gen.current_gait == GaitType.wave
        assert walker.current_gait == GaitType.wave
        assert walker.phase_step == 1 / 30.0
        assert walker.current_direction == Point3D([1, 0, 0])
        assert walker.current_stride_ratio == 0.0
        assert walker.current_rotation_ratio == 0.0
        assert walker.current_phase == 0.0
        assert walker.last_stop_phase == 0.0

    def test_reset(self, walker):
        walker.current_direction = Point3D([0, 1, 0])
        walker.current_stride_ratio = 1.0
        walker.current_rotation_ratio = 1.0
        walker.current_phase = 1.0
        walker.last_stop_phase = 1.0

        walker.reset()

        assert walker.current_direction == Point3D([1, 0, 0])
        assert walker.current_stride_ratio == 0.0
        assert walker.current_rotation_ratio == 0.0
        assert walker.current_phase == 0.0
        assert walker.last_stop_phase == 0.0

    def test_next_phase(self, walker):
        walker.next_step()
        assert walker.current_phase == 0

        walker.next_step()
        assert walker.current_phase == 1 / 30.0

        walker.next_step()
        assert walker.current_phase == 2 / 30.0
