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

from dataclasses import dataclass
import math
from pathlib import Path

from drqp_brain.parametric_gait_generator import GaitType
from drqp_kinematics.geometry import Point3D
import yaml


@dataclass(frozen=True)
class DirectionalStrideSample:
    angle_degrees: float
    max_step_length_m: float


class DirectionalStrideLimits:
    """Per-gait polar stride limits for clamping planar walking commands."""

    def __init__(self, gait_limits: dict[GaitType, list[DirectionalStrideSample]]):
        if not gait_limits:
            raise ValueError('stride limits must include at least one gait')

        self._gait_limits = {
            gait: sorted(samples, key=lambda sample: sample.angle_degrees)
            for gait, samples in gait_limits.items()
        }
        for gait, samples in self._gait_limits.items():
            if len(samples) < 2:
                raise ValueError(f'{gait.name} stride limits must include at least two samples')

    @classmethod
    def from_file(cls, path: Path | str):
        with open(path) as file:
            return cls.from_dict(yaml.safe_load(file))

    @classmethod
    def from_dict(cls, data):
        if data.get('version') != 1:
            raise ValueError(f'unsupported stride limits version: {data.get("version")}')

        gait_limits = {}
        for gait_name, raw_samples in data.get('gaits', {}).items():
            gait = GaitType[gait_name]
            gait_limits[gait] = [
                DirectionalStrideSample(
                    angle_degrees=float(sample['angle_degrees']) % 360.0,
                    max_step_length_m=float(sample['max_step_length_m']),
                )
                for sample in raw_samples
            ]

        return cls(gait_limits)

    def max_step_length(self, gait: GaitType, direction: Point3D) -> float | None:
        samples = self._gait_limits.get(gait)
        if samples is None:
            return None

        planar_norm = math.hypot(float(direction.x), float(direction.y))
        if planar_norm == 0.0:
            return None

        angle = math.degrees(math.atan2(float(direction.y), float(direction.x))) % 360.0
        return self._interpolate_limit(samples, angle)

    def clamp_direction(
        self,
        gait: GaitType,
        direction: Point3D,
        nominal_step_length: float,
    ) -> Point3D:
        limit = self.max_step_length(gait, direction)
        if limit is None or nominal_step_length <= 0.0:
            return direction

        stride_ratio = abs(float(direction.x)) + abs(float(direction.y))
        if stride_ratio == 0.0:
            return direction

        max_stride_ratio = min(1.0, limit / nominal_step_length)
        if stride_ratio <= max_stride_ratio:
            return direction

        scale = max_stride_ratio / stride_ratio
        return Point3D(
            [
                float(direction.x) * scale,
                float(direction.y) * scale,
                float(direction.z),
            ],
            direction.label,
        )

    @staticmethod
    def _interpolate_limit(samples: list[DirectionalStrideSample], angle: float) -> float:
        extended_samples = samples + [
            DirectionalStrideSample(
                angle_degrees=samples[0].angle_degrees + 360.0,
                max_step_length_m=samples[0].max_step_length_m,
            )
        ]
        normalized_angle = angle
        if normalized_angle < samples[0].angle_degrees:
            normalized_angle += 360.0

        for start, end in zip(extended_samples, extended_samples[1:]):
            if start.angle_degrees <= normalized_angle <= end.angle_degrees:
                span = end.angle_degrees - start.angle_degrees
                if span == 0.0:
                    return min(start.max_step_length_m, end.max_step_length_m)
                ratio = (normalized_angle - start.angle_degrees) / span
                return (
                    start.max_step_length_m
                    + (end.max_step_length_m - start.max_step_length_m) * ratio
                )

        return samples[-1].max_step_length_m
