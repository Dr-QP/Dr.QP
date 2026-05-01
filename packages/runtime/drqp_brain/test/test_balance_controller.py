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

from drqp_brain.balance_controller import (
    apply_imu_balance,
    BASE_CENTER_TO_IMU_ROTATION,
    body_tilt_from_imu,
)
from drqp_brain.geometry import Point3D
from geometry_msgs.msg import Quaternion
import pytest
from scipy.spatial.transform import Rotation as R


def make_quaternion_msg(rotation: R) -> Quaternion:
    x, y, z, w = rotation.as_quat()
    return Quaternion(x=x, y=y, z=z, w=w)


def test_body_tilt_from_imu_returns_base_center_roll_and_pitch():
    base_in_world = R.from_euler('xyz', [0.18, -0.12, 0.41], degrees=False)
    imu_in_world = base_in_world * BASE_CENTER_TO_IMU_ROTATION

    tilt = body_tilt_from_imu(make_quaternion_msg(imu_in_world))

    assert tilt.x == pytest.approx(0.18)
    assert tilt.y == pytest.approx(-0.12)
    assert tilt.z == pytest.approx(0.0)


def test_body_tilt_from_imu_is_zero_for_level_body():
    tilt = body_tilt_from_imu(make_quaternion_msg(BASE_CENTER_TO_IMU_ROTATION))

    assert tilt == Point3D([0.0, 0.0, 0.0])


def test_body_tilt_from_imu_can_use_raw_orientation_without_mount_compensation():
    tilt = body_tilt_from_imu(
        make_quaternion_msg(R.from_euler('xyz', [0.07, -0.09, 0.2], degrees=False)),
        base_center_to_imu_rotation=R.identity(),
    )

    assert tilt.x == pytest.approx(0.07)
    assert tilt.y == pytest.approx(-0.09)
    assert tilt.z == pytest.approx(0.0)


def test_apply_imu_balance_cancels_tilt_and_preserves_yaw():
    balanced = apply_imu_balance(
        Point3D([0.0, 0.0, 0.20]),
        Point3D([0.10, -0.15, 0.30]),
    )
    balanced_rotation = R.from_rotvec(balanced.numpy())
    expected_rotation = R.from_euler('xyz', [-0.10, 0.15, 0.0], degrees=False) * R.from_rotvec(
        [0.0, 0.0, 0.20]
    )

    assert balanced_rotation.as_matrix() == pytest.approx(expected_rotation.as_matrix())


def test_apply_imu_balance_clamps_large_tilt():
    balanced = apply_imu_balance(
        Point3D([0.0, 0.0, 0.4]),
        Point3D([1.0, -1.0, 0.0]),
        gain=1.0,
        max_tilt_rad=0.25,
    )
    balanced_rotation = R.from_rotvec(balanced.numpy())
    expected_rotation = R.from_euler('xyz', [-0.25, 0.25, 0.0], degrees=False) * R.from_rotvec(
        [0.0, 0.0, 0.4]
    )

    assert balanced_rotation.as_matrix() == pytest.approx(expected_rotation.as_matrix())
