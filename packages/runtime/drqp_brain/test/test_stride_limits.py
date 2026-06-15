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

from drqp_brain.generate_stride_limits import find_max_step_length, generate_stride_limits
from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.stride_limits import DirectionalStrideLimits
from drqp_kinematics.geometry import Point3D
import pytest


def _limits_data(max_step_length_m=0.08):
    return {
        'version': 1,
        'directions_count': 4,
        'joint_margin_degrees': 0.25,
        'gaits': {
            gait.name: [
                {'angle_degrees': 0.0, 'max_step_length_m': 0.12},
                {'angle_degrees': 90.0, 'max_step_length_m': max_step_length_m},
                {'angle_degrees': 180.0, 'max_step_length_m': 0.12},
                {'angle_degrees': 270.0, 'max_step_length_m': max_step_length_m},
            ]
            for gait in GaitType
        },
    }


def test_stride_limits_load_and_interpolate_between_direction_samples():
    limits = DirectionalStrideLimits.from_dict(_limits_data())

    assert limits.max_step_length(GaitType.tripod, Point3D([1, 0, 0])) == pytest.approx(0.12)
    assert limits.max_step_length(GaitType.tripod, Point3D([0, 1, 0])) == pytest.approx(0.08)
    assert limits.max_step_length(GaitType.tripod, Point3D([1, 1, 0])) == pytest.approx(0.10)


def test_stride_limits_clamp_over_limit_vector_preserving_direction():
    limits = DirectionalStrideLimits.from_dict(_limits_data())

    clamped = limits.clamp_direction(
        GaitType.tripod,
        Point3D([0.0, 0.9, 0.0]),
        nominal_step_length=0.10,
    )

    assert clamped.x == pytest.approx(0.0)
    assert clamped.y == pytest.approx(0.8)
    assert clamped.z == pytest.approx(0.0)


def test_stride_limits_leave_under_limit_vector_unchanged():
    limits = DirectionalStrideLimits.from_dict(_limits_data())
    direction = Point3D([0.0, 0.8, 0.0])

    clamped = limits.clamp_direction(
        GaitType.tripod,
        direction,
        nominal_step_length=0.10,
    )

    assert clamped == direction


def test_find_max_step_length_binary_searches_highest_safe_candidate():
    result = find_max_step_length(
        is_safe=lambda step_length: step_length <= 0.125,
        upper_bound=0.2,
        iterations=8,
    )

    assert result == pytest.approx(0.125, abs=0.001)


def test_generate_stride_limits_emits_all_requested_directions_for_each_gait():
    config = generate_stride_limits(
        is_step_length_safe=lambda _gait, _direction, step_length: step_length <= 0.125,
        directions_count=4,
        max_step_length_m=0.2,
        search_iterations=8,
        joint_margin_degrees=0.25,
    )

    assert config['version'] == 1
    assert config['directions_count'] == 4
    assert set(config['gaits']) == {gait.name for gait in GaitType}
    for samples in config['gaits'].values():
        assert [sample['angle_degrees'] for sample in samples] == [0.0, 90.0, 180.0, 270.0]
        assert all(
            sample['max_step_length_m'] == pytest.approx(0.125, abs=0.001) for sample in samples
        )
