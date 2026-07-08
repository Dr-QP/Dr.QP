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

from drqp_kinematics.geometry import Point3D
from geometry_msgs.msg import Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

# Fixed rotation of imu_link relative to base_center_link, matching the
# base_center_to_imu joint in packages/runtime/drqp_control/urdf/body.urdf.xacro.
# The IMU chip is mounted flipped (roll pi) and turned a quarter turn (yaw pi/2).
#
# Gazebo's IMU plugin reports the sensor's absolute orientation in the world frame.
# Because the IMU is rigidly mounted on the body, its world orientation is simply
# body_in_world composed with the fixed sensor mount: imu = body * mount.
# To recover the body orientation: body = imu * mount.inv().
# This uses the known static mount instead of a TF lookup or the message frame_id,
# which are unreliable for the simulated IMU.
BASE_CENTER_TO_IMU_ROTATION = R.from_euler('xyz', [np.pi, 0.0, np.pi / 2.0])


def body_tilt_from_imu(orientation: Quaternion) -> Point3D:
    """
    Return body roll and pitch in radians from a raw IMU quaternion.

    Gazebo's IMU plugin reports the sensor's absolute orientation in the world
    frame (imu = body * mount). Right-multiplying by the inverse mount rotation
    recovers the body orientation: body = imu * mount.inv().

    Parameters
    ----------
    orientation
        Raw IMU sensor-frame orientation quaternion in ROS message form.

    """
    imu_in_world = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    body_in_world = imu_in_world * BASE_CENTER_TO_IMU_ROTATION.inv()
    roll, pitch, _ = body_in_world.as_euler('xyz', degrees=False)
    return Point3D([roll, pitch, 0.0])


def imu_balance_stride_scale(
    measured_body_tilt: Point3D | None,
    *,
    target_body_tilt: Point3D | None = None,
    gain: float = 1.0,
    max_tilt_rad: float = np.pi / 4.0,
    floor: float = 0.3,
) -> float:
    """
    Return a stride-scaling factor in ``(0, 1]`` reflecting IMU balance saturation.

    ``apply_imu_balance()`` silently clips the requested roll/pitch correction
    (``tilt_error * gain``) to ``max_tilt_rad``. When the raw, unclipped
    correction exceeds that bound, the body is more tilted than the corrector
    can fully compensate for; continuing to command a full-throttle stride in
    that situation pushes foot targets past what MoveIt IK can solve (e.g. a
    diagonal stride combined with a saturating tilt correction). Scale the
    stride down as saturation grows so the walker backs off automatically
    instead of repeatedly failing IK on the same unreachable target.
    """
    if measured_body_tilt is None or max_tilt_rad <= 0.0:
        return 1.0

    tilt_error = measured_body_tilt
    if target_body_tilt is not None:
        tilt_error = measured_body_tilt - target_body_tilt

    raw_correction = np.abs(tilt_error.numpy()[:2] * gain)
    saturation = float(np.max(raw_correction)) / max_tilt_rad
    if saturation <= 1.0:
        return 1.0

    # Ramp linearly from 1.0 at the saturation boundary down to `floor` once
    # the raw correction reaches double the clamp, rather than stopping dead.
    return float(np.clip(1.0 - (saturation - 1.0), floor, 1.0))


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
