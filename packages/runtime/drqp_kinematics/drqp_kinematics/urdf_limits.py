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

"""Convert between analytic-model and URDF joint conventions."""

import xml.etree.ElementTree as ElementTree

import numpy as np

FEMUR_MODEL_TO_URDF_OFFSET_DEG = -13.11
TIBIA_MODEL_TO_URDF_OFFSET_DEG = -32.9

MODEL_TO_URDF_OFFSETS_DEG = (
    0.0,
    FEMUR_MODEL_TO_URDF_OFFSET_DEG,
    TIBIA_MODEL_TO_URDF_OFFSET_DEG,
)

MODEL_TO_URDF_OFFSETS_RAD = tuple(
    float(np.radians(offset)) for offset in MODEL_TO_URDF_OFFSETS_DEG
)


def model_to_urdf_angles(
    angles_rad: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Convert analytic coxa/femur/tibia radians to the physical URDF convention.

    ``LegModel`` deliberately represents the femur and tibia as straight links,
    so its zero pose differs from the physical brackets described by the URDF.
    The femur offset is the fixed femur-bracket angle and the tibia offset is the
    fixed tibia-bracket angle. MoveIt already solves against those physical URDF
    frames and must not use this conversion.

    These assembly-geometry offsets are shared by every robot. They are not
    per-servo trim calibration; unit-specific calibration belongs in the
    hardware/controller layer.
    """
    return tuple(
        float(np.radians(np.degrees(angle) + offset))
        for angle, offset in zip(angles_rad, MODEL_TO_URDF_OFFSETS_DEG)
    )


def model_degrees_to_urdf_angles(
    angles_deg: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert legacy ``LegModel`` degrees directly to URDF radians."""
    return tuple(
        float(np.radians(angle + offset))
        for angle, offset in zip(angles_deg, MODEL_TO_URDF_OFFSETS_DEG)
    )


def urdf_to_model_angles(
    angles_rad: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert physical URDF coxa/femur/tibia radians to analytic-model radians."""
    return tuple(
        float(np.radians(np.degrees(angle) - offset))
        for angle, offset in zip(angles_rad, MODEL_TO_URDF_OFFSETS_DEG)
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
    lower_limits, upper_limits = zip(
        *(urdf_joint_limits[joint_name] for joint_name in joint_names)
    )
    model_lower_limits = urdf_to_model_angles(lower_limits)
    model_upper_limits = urdf_to_model_angles(upper_limits)
    return tuple(zip(model_lower_limits, model_upper_limits))
