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

from pathlib import Path

from drqp_kinematics.urdf_limits import (
    FEMUR_MODEL_TO_URDF_OFFSET_DEG,
    model_joint_limits_from_urdf,
    model_to_urdf_angles,
    parse_joint_limits,
    TIBIA_MODEL_TO_URDF_OFFSET_DEG,
    urdf_to_model_angles,
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
    assert model_limits[1] == pytest.approx(
        (
            -2.0 - np.radians(FEMUR_MODEL_TO_URDF_OFFSET_DEG),
            2.0 - np.radians(FEMUR_MODEL_TO_URDF_OFFSET_DEG),
        )
    )
    assert model_limits[2] == pytest.approx(
        (
            -3.0 - np.radians(TIBIA_MODEL_TO_URDF_OFFSET_DEG),
            3.0 - np.radians(TIBIA_MODEL_TO_URDF_OFFSET_DEG),
        )
    )


@pytest.mark.parametrize(
    'model_angles',
    [
        (0.0, 0.0, 0.0),
        (0.3, -0.7, 1.2),
        (-1.0, 0.25, -0.5),
    ],
)
def test_model_urdf_conversion_round_trip(model_angles):
    """Model and URDF conversions are exact inverses within float precision."""
    assert urdf_to_model_angles(model_to_urdf_angles(model_angles)) == pytest.approx(
        model_angles
    )


def test_servo_offset_literals_are_owned_only_by_kinematics():
    """Prevent duplicated assembly offsets outside their owning module."""
    workspace_root = Path(__file__).parents[4]
    owning_module = Path('packages/runtime/drqp_kinematics/drqp_kinematics/urdf_limits.py')
    forbidden_literals = ('13.' + '11', '32.' + '9')
    duplicated_in = []
    for source_root in ('packages', 'docs'):
        for path in (workspace_root / source_root).rglob('*'):
            if path.suffix not in {'.py', '.md', '.yaml', '.xml'}:
                continue
            relative_path = path.relative_to(workspace_root)
            if relative_path == owning_module:
                continue
            contents = path.read_text(encoding='utf-8')
            if any(literal in contents for literal in forbidden_literals):
                duplicated_in.append(str(relative_path))

    assert duplicated_in == []
