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

"""Pure gait-target generation and time-based walking state management."""

from dataclasses import dataclass
import math

from drqp_brain.parametric_gait_generator import GaitType, ParametricGaitGenerator
from drqp_kinematics.geometry import AffineTransform, Point3D
import numpy as np


@dataclass(frozen=True)
class SteeringState:
    """Smoothed stride and rotation commands used to evaluate a gait phase."""

    direction: Point3D
    rotation_direction: float


class WalkController:
    """Generate foot targets from an explicit gait phase and steering state."""

    _NO_MOTION_EPSILON = 1e-3

    def __init__(
        self,
        hexapod,
        step_length=60.0,
        step_height=40.0,
        rotation_speed_degrees=10.0,
        cycle_time_sec=2.5,
        gait=GaitType.wave,
        stride_limits=None,
        steering_tau_sec=0.35,
    ):
        if cycle_time_sec <= 0:
            raise ValueError('cycle_time_sec must be positive')
        if steering_tau_sec <= 0:
            raise ValueError('steering_tau_sec must be positive')

        self.hexapod = hexapod
        self.leg_tips_on_ground = [(leg, leg.tibia_end.copy()) for leg in hexapod.legs]
        self.step_length = step_length
        self.step_height = step_height
        self.rotation_speed_degrees = rotation_speed_degrees
        self.cycle_time_sec = float(cycle_time_sec)
        self.steering_tau_sec = float(steering_tau_sec)
        self.stride_limits = stride_limits
        self.gait_gen = ParametricGaitGenerator(step_length=1.0, step_height=1.0, gait=gait)
        self.reset()

    @property
    def current_gait(self):
        """Return the configured gait type."""
        return self.gait_gen.current_gait

    @current_gait.setter
    def current_gait(self, gait):
        self.gait_gen.current_gait = gait

    @property
    def current_direction(self) -> Point3D:
        """Return the current smoothed stride command."""
        return self.steering.direction

    @property
    def current_rotation_direction(self) -> float:
        """Return the current smoothed rotation command."""
        return self.steering.rotation_direction

    def reset(self):
        """Return the controller to a stationary initial state."""
        self.steering = SteeringState(Point3D([0, 0, 0]), 0.0)
        self.current_phase = 0.0

    def advance(
        self,
        dt: float,
        stride_direction: Point3D,
        rotation_direction: float,
    ) -> None:
        """Smooth commands and advance phase exactly once for this control tick."""
        if dt < 0:
            raise ValueError('dt must be non-negative')

        alpha = 1.0 - math.exp(-dt / self.steering_tau_sec)
        direction = self.steering.direction.interpolate(stride_direction, alpha)
        direction = self._clamp_direction(direction)
        rotation = float(
            np.interp(
                alpha,
                [0.0, 1.0],
                [self.steering.rotation_direction, np.clip(rotation_direction, -1.0, 1.0)],
            )
        )
        rotation = float(np.clip(rotation, -1.0, 1.0))

        if self._stride_ratio(direction) < self._NO_MOTION_EPSILON:
            direction = Point3D([0, 0, 0])
        if abs(rotation) < self._NO_MOTION_EPSILON:
            rotation = 0.0

        self.steering = SteeringState(direction, rotation)
        if self._has_motion(self.steering):
            self.current_phase = (self.current_phase + dt / self.cycle_time_sec) % 1.0

    def targets_at(
        self,
        phase: float,
        steering: SteeringState,
        body_direction: Point3D | None = None,
        body_rotation: Point3D | None = None,
        verbose: bool = False,
    ) -> list[tuple[object, Point3D]]:
        """Return foot targets for explicit inputs without mutating controller state."""
        # Body pose is deliberately an explicit input even though gait targets are
        # world-grounded. The caller applies body_transform before solving IK.
        del body_direction, body_rotation

        stride_ratio = self._stride_ratio(steering.direction)
        has_stride = stride_ratio >= self._NO_MOTION_EPSILON
        has_rotation = abs(steering.rotation_direction) >= self._NO_MOTION_EPSILON
        direction_transform = self._make_direction_transform(steering.direction)
        result = []

        for leg, leg_tip in self.leg_tips_on_ground:
            gait_offsets = self.gait_gen.get_offsets_at_phase_for_leg(leg.label, phase)
            stride_offsets = Point3D([0, 0, 0])
            direction_offsets = Point3D([0, 0, 0])
            stride_target = leg_tip
            if has_stride:
                stride_offsets = gait_offsets * Point3D(
                    [self.step_length * stride_ratio, 0.0, 0.0]
                )
                direction_offsets = direction_transform.apply_point(stride_offsets)
                stride_target = stride_target + direction_offsets

            rotation_target = leg_tip
            if has_rotation:
                rotation_degrees = (
                    self.rotation_speed_degrees * steering.rotation_direction * gait_offsets.x
                )
                rotation_transform = AffineTransform.from_rotvec(
                    [0, 0, rotation_degrees], degrees=True
                )
                rotation_target = rotation_transform.apply_point(rotation_target)

            if has_stride or has_rotation:
                mix_weights = np.array([stride_ratio, abs(steering.rotation_direction)])
                mix_weights /= mix_weights.sum()
                stride_weight, rotation_weight = mix_weights
                foot_target = stride_target * stride_weight + rotation_target * rotation_weight
                foot_target.z += gait_offsets.z * self.step_height
            else:
                foot_target = leg_tip.copy()

            if verbose:
                print(f'{leg.label} {phase=}')
                print(f'{leg_tip=}')
                print(f'{gait_offsets=}')
                print(f'{foot_target=}')
                print()
            result.append((leg, foot_target))

        return result

    @staticmethod
    def body_transform(
        body_direction: Point3D | None,
        body_rotation: Point3D | None,
    ) -> AffineTransform:
        """Build the body pose that the caller applies separately from targets."""
        transform = AffineTransform.identity()
        if body_direction is not None:
            transform = AffineTransform.from_translation(body_direction.numpy())
        if body_rotation is not None:
            transform = AffineTransform.from_rotvec(body_rotation.numpy()) @ transform
        return transform

    def apply_feet_targets(self, legs_and_targets):
        """Apply targets through the legacy visual/notebook kinematics path."""
        for leg, foot_target in legs_and_targets:
            leg.move_to(foot_target)

    def _clamp_direction(self, direction: Point3D) -> Point3D:
        if self.stride_limits is None:
            return direction
        return self.stride_limits.clamp_direction(
            self.current_gait,
            direction,
            self.step_length,
        )

    @staticmethod
    def _stride_ratio(direction: Point3D) -> float:
        return float(np.clip(abs(direction.x) + abs(direction.y) + abs(direction.z), 0.0, 1.0))

    def _has_motion(self, steering: SteeringState) -> bool:
        return (
            self._stride_ratio(steering.direction) >= self._NO_MOTION_EPSILON
            or abs(steering.rotation_direction) >= self._NO_MOTION_EPSILON
        )

    @staticmethod
    def _make_direction_transform(direction: Point3D) -> AffineTransform:
        norm_direction = direction.normalized().numpy()
        return AffineTransform.from_rotmatrix(
            [
                [norm_direction[0], -norm_direction[1], 0],
                [norm_direction[1], norm_direction[0], 0],
                [0, 0, 1],
            ]
        )
