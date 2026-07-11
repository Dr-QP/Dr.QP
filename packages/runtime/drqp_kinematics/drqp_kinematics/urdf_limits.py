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

"""Parse URDF joint limits and convert controller limits to model convention."""

import xml.etree.ElementTree as ElementTree

import numpy as np

MODEL_TO_URDF_OFFSETS_RAD = (
    0.0,
    float(np.radians(-13.11)),
    float(np.radians(-32.9)),
)


def parse_joint_limits(robot_description_xml: str) -> dict[str, tuple[float, float]]:
    """Return each URDF joint's finite positional limits in controller radians."""
    root = ElementTree.fromstring(robot_description_xml)
    joint_limits = {}
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is None:
            continue
        lower = limit.get('lower')
        upper = limit.get('upper')
        if lower is None or upper is None:
            continue
        joint_limits[joint.attrib['name']] = (float(lower), float(upper))
    return joint_limits


def model_joint_limits_from_urdf(
    urdf_joint_limits: dict[str, tuple[float, float]],
    joint_names: tuple[str, str, str],
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Convert coxa/femur/tibia URDF limits into the visual-model convention."""
    model_limits = []
    for joint_name, offset in zip(joint_names, MODEL_TO_URDF_OFFSETS_RAD):
        lower, upper = urdf_joint_limits[joint_name]
        model_limits.append((lower - offset, upper - offset))
    return tuple(model_limits)
