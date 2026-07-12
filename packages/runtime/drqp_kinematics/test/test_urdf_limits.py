# Copyright (c) 2026 Anton Matosov
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

"""Tests for URDF joint-limit parsing and model-convention conversion."""

from drqp_kinematics.urdf_limits import (
    model_joint_limits_from_urdf,
    parse_joint_limits,
)
import numpy as np
import pytest


def test_parse_joint_limits_reads_only_revolute_limits():
    """URDF parser returns the lower and upper radians for limited joints."""
    robot_description = """
    <robot name="drqp">
      <joint name="drqp/left_front_coxa" type="revolute">
        <limit lower="-1.0" upper="1.0" effort="3" velocity="2"/>
      </joint>
      <joint name="fixed_joint" type="fixed"/>
    </robot>
    """

    assert parse_joint_limits(robot_description) == {
        'drqp/left_front_coxa': (-1.0, 1.0),
    }


def test_model_joint_limits_apply_controller_to_model_offsets():
    """Femur and tibia limits account for the controller zero offsets once."""
    urdf_limits = {
        'drqp/left_front_coxa': (-1.0, 1.0),
        'drqp/left_front_femur': (-2.0, 2.0),
        'drqp/left_front_tibia': (-3.0, 3.0),
    }

    model_limits = model_joint_limits_from_urdf(
        urdf_limits,
        (
            'drqp/left_front_coxa',
            'drqp/left_front_femur',
            'drqp/left_front_tibia',
        ),
    )

    assert model_limits[0] == pytest.approx((-1.0, 1.0))
    assert model_limits[1] == pytest.approx((-2.0 - np.radians(-13.11), 2.0 - np.radians(-13.11)))
    assert model_limits[2] == pytest.approx((-3.0 - np.radians(-32.9), 3.0 - np.radians(-32.9)))
