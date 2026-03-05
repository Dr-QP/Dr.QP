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

from launch_testing import post_shutdown_test
import pytest
from robot_control_test_support import (
    create_simulation_launch_description,
    GazeboRobotControlBase,
)
from simulation_shutdown_test_case import (
    SimulationShutdownTestCase,
)


@pytest.mark.slow
@pytest.mark.launch_test
def generate_test_description():
    return create_simulation_launch_description()


@pytest.mark.slow
class TestGazeboRobotControlRotationMovement(GazeboRobotControlBase):
    """Verify rotation command changes robot yaw."""

    def test_movement_rotation(self):
        self.assert_rotation_movement()


@post_shutdown_test()
class TestSimulationShutdown(SimulationShutdownTestCase):
    """Verify processes exit cleanly after the launch test finishes."""
