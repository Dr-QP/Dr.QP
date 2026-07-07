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
Verify full-throttle motion and IMU balance mode do not permanently freeze MoveIt IK.

Regression tests for a robot "freeze" where MoveItPy IK repeatedly failed
("MoveItPy IK failed for <leg>" / "MoveIt IK rejected the current foot targets")
and the robot stopped responding to movement commands. Reported triggers:
  - reversing a full-throttle stride direction (e.g. forward -> backward), and
  - enabling balance mode and immediately commanding a full-throttle stride in any
    direction.

Both cases produce a lasting freeze rather than a transient blip: HexapodBrain.loop()
rolls back walker state on every IK failure, so a foot target that fails once from
the seeded pose keeps failing identically on every later tick.

``test_balance_mode_diagonal_stride_movement`` covers a third, specifically reported
trigger: a diagonal stride (not a pure fore-aft or lateral direction) combined with
balance mode.
"""

from drqp_launch_testing import assert_processes_exited_cleanly
import pytest
from robot_control_test_support import generate_test_description


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_direction_reversal_from_forward_to_backward(robot, generate_test_description):
    robot.assert_direction_reversal_from_forward_to_backward()
    # Function-scoped generator: the simulation tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_balance_mode_full_stride_movement_in_any_direction(robot, generate_test_description):
    robot.assert_balance_mode_full_stride_movement_in_any_direction()
    # Function-scoped generator: the simulation tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
def test_balance_mode_diagonal_stride_movement(robot, generate_test_description):
    robot.assert_balance_mode_diagonal_stride_movement(stride_x=0.66, stride_y=-0.77)
    # Function-scoped generator: the simulation tears down at the yield, then the
    # post-yield body verifies every non-simulator process exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
