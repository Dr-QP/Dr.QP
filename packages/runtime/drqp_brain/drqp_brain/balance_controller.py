#!/usr/bin/env python3
#
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

from drqp_brain.geometry import Point3D
import numpy as np
from scipy.spatial.transform import Rotation as R

# Matches the fixed base_center_to_imu joint orientation in
# packages/runtime/drqp_control/urdf/body.urdf.xacro.
BASE_CENTER_TO_IMU_ROTATION = R.from_euler('xyz', [np.pi, 0.0, np.pi / 2.0])
_USE_DEFAULT_ROTATION = object()


def body_tilt_from_imu(
    orientation,
    *,
    base_center_to_imu_rotation=_USE_DEFAULT_ROTATION,
    imu_to_base_rotation=None,
) -> Point3D:
    """
    Return base_center_link roll and pitch in radians from an IMU quaternion.

    Parameters
    ----------
    orientation
        IMU quaternion in ROS message form.
    base_center_to_imu_rotation
        Optional base->IMU mount rotation. Omit to use the default hardware
        mount transform, or pass ``None`` to skip mount compensation.
    imu_to_base_rotation
        Optional IMU->base rotation, typically from TF. When provided, it takes
        precedence over ``base_center_to_imu_rotation`` and avoids an extra
        inversion on the hot path.

    A sentinel is used for the default because ``None`` is a public, meaningful input.
    """
    imu_in_world = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    if imu_to_base_rotation is not None:
        base_in_world = imu_in_world * imu_to_base_rotation
    else:
        if base_center_to_imu_rotation is _USE_DEFAULT_ROTATION:
            base_center_to_imu_rotation = BASE_CENTER_TO_IMU_ROTATION
        base_in_world = imu_in_world
        if base_center_to_imu_rotation is not None:
            base_in_world = imu_in_world * base_center_to_imu_rotation.inv()
    roll, pitch, _ = base_in_world.as_euler('xyz', degrees=False)
    return Point3D([roll, pitch, 0.0])


def apply_imu_balance(
    body_rotation: Point3D,
    measured_body_tilt: Point3D | None,
    *,
    target_body_tilt: Point3D | None = None,
    gain: float = 1.0,
    max_tilt_rad: float = np.pi / 4.0,
) -> Point3D:
    """Apply roll and pitch compensation while preserving yaw commands."""
    if measured_body_tilt is None:
        return body_rotation

    tilt_error = measured_body_tilt
    if target_body_tilt is not None:
        tilt_error = measured_body_tilt - target_body_tilt

    tilt_bounds = np.array([max_tilt_rad, max_tilt_rad, 0.0])
    clipped_correction = np.clip(tilt_error.numpy() * gain, -tilt_bounds, tilt_bounds)
    requested_rotation = R.from_rotvec(body_rotation.numpy())
    balance_correction = R.from_euler(
        'xyz',
        [-clipped_correction[0], -clipped_correction[1], 0.0],
        degrees=False,
    )
    return Point3D((balance_correction * requested_rotation).as_rotvec())
