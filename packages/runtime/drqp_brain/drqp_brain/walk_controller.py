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

"""Pure SE(2) gait-target generation and time-based walking state management."""

from dataclasses import dataclass
import math

from drqp_brain.parametric_gait_generator import GaitType, ParametricGaitGenerator
from drqp_kinematics.geometry import AffineTransform, Point3D
import numpy as np


@dataclass(frozen=True)
class SteeringState:
    """Smoothed body-frame planar twist used to evaluate a gait phase."""

    linear_velocity: Point3D
    angular_velocity: float

    @property
    def direction(self) -> Point3D:
        """Return the linear twist component for transitional callers."""
        return self.linear_velocity

    @property
    def rotation_direction(self) -> float:
        """Return the angular twist component for transitional callers."""
        return self.angular_velocity


@dataclass(frozen=True)
class TwistSaturation:
    """A reachability-limited twist and the scalar used to obtain it."""

    twist: SteeringState
    scale: float

    @property
    def linear_velocity(self) -> Point3D:
        """Expose the saturated linear velocity for concise callers."""
        return self.twist.linear_velocity

    @property
    def angular_velocity(self) -> float:
        """Expose the saturated angular velocity for concise callers."""
        return self.twist.angular_velocity


class WalkController:
    """Generate world-grounded foot targets from a planar body twist."""

    _NO_MOTION_EPSILON = 1e-3
    _PATH_SAMPLES = 49
    _SATURATION_STEPS = 20

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
        omega_max_rad_sec=None,
    ):
        if cycle_time_sec <= 0:
            raise ValueError('cycle_time_sec must be positive')
        if steering_tau_sec <= 0:
            raise ValueError('steering_tau_sec must be positive')
        if omega_max_rad_sec is not None and omega_max_rad_sec <= 0:
            raise ValueError('omega_max_rad_sec must be positive when set')

        self.hexapod = hexapod
        self.leg_tips_on_ground = [(leg, leg.tibia_end.copy()) for leg in hexapod.legs]
        self.step_length = float(step_length)
        self.step_height = float(step_height)
        # Retain this value only as a compatibility mapping.  When an explicit
        # omega limit is not supplied, it gives the historical yaw per stance.
        self.rotation_speed_degrees = float(rotation_speed_degrees)
        self._configured_omega_max_rad_sec = omega_max_rad_sec
        self.cycle_time_sec = float(cycle_time_sec)
        self.steering_tau_sec = float(steering_tau_sec)
        # The certified polar table remains an offline/CI artifact.  Runtime
        # saturation below evaluates every leg's actual combined twist path.
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
    def stance_duration_sec(self) -> float:
        """Return the current gait's duration with a foot on the ground."""
        swing_duration = self.gait_gen.gaits[self.current_gait].swing_duration
        return self.cycle_time_sec * (1.0 - swing_duration)

    @property
    def omega_max_rad_sec(self) -> float:
        """
        Return the full-scale yaw rate for the current gait.

        The legacy controller rotated a foot by ``rotation_speed_degrees * s``
        over the stride parameter ``s``.  Setting ``omega_max`` to
        ``radians(rotation_speed_degrees) / T_stance`` retains that yaw per
        stance while expressing the command in radians per second.
        """
        if self._configured_omega_max_rad_sec is not None:
            return float(self._configured_omega_max_rad_sec)
        return math.radians(self.rotation_speed_degrees) / self.stance_duration_sec

    @property
    def current_direction(self) -> Point3D:
        """Return the current smoothed linear body velocity."""
        return self.steering.linear_velocity

    @property
    def current_rotation_direction(self) -> float:
        """Return the current smoothed angular body velocity."""
        return self.steering.angular_velocity

    def reset(self):
        """Return the controller to a stationary initial state."""
        self.steering = SteeringState(Point3D([0, 0, 0]), 0.0)
        self.current_phase = 0.0

    def command_to_twist(
        self,
        stride_direction: Point3D,
        rotation_direction: float,
    ) -> SteeringState:
        """Map normalized movement input to a metric planar body twist."""
        planar_norm = math.hypot(float(stride_direction.x), float(stride_direction.y))
        planar_scale = 1.0 / planar_norm if planar_norm > 1.0 else 1.0
        max_linear_speed = self.step_length / self.stance_duration_sec
        return SteeringState(
            Point3D(
                [
                    float(stride_direction.x) * planar_scale * max_linear_speed,
                    float(stride_direction.y) * planar_scale * max_linear_speed,
                    0.0,
                ]
            ),
            float(np.clip(rotation_direction, -1.0, 1.0)) * self.omega_max_rad_sec,
        )

    def advance(
        self,
        dt: float,
        stride_direction: Point3D,
        rotation_direction: float,
    ) -> None:
        """Smooth, saturate, and commit a command before advancing gait phase."""
        if dt < 0:
            raise ValueError('dt must be non-negative')

        target = self.command_to_twist(stride_direction, rotation_direction)
        alpha = 1.0 - math.exp(-dt / self.steering_tau_sec)
        proposed = SteeringState(
            self.steering.linear_velocity.interpolate(target.linear_velocity, alpha),
            float(
                np.interp(
                    alpha,
                    [0.0, 1.0],
                    [self.steering.angular_velocity, target.angular_velocity],
                )
            ),
        )
        saturated = self._saturate_twist(proposed).twist
        if self._linear_speed(saturated) < self._NO_MOTION_EPSILON:
            saturated = SteeringState(Point3D([0, 0, 0]), saturated.angular_velocity)
        if abs(saturated.angular_velocity) < self._NO_MOTION_EPSILON:
            saturated = SteeringState(saturated.linear_velocity, 0.0)

        self.steering = saturated
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
        if not self._has_motion(steering):
            return [(leg, leg_tip.copy()) for leg, leg_tip in self.leg_tips_on_ground]
        result = []
        for leg, leg_tip in self.leg_tips_on_ground:
            gait_offsets = self.gait_gen.get_offsets_at_phase_for_leg(leg.label, phase)
            foot_target = self._foot_target_for_stride_offset(
                leg_tip,
                float(gait_offsets.x),
                steering,
            )
            foot_target.z += gait_offsets.z * self.step_height
            if verbose:
                print(f'{leg.label} {phase=}')
                print(f'{leg_tip=}')
                print(f'{gait_offsets=}')
                print(f'{foot_target=}')
                print()
            result.append((leg, foot_target))
        return result

    def _foot_target_for_stride_offset(
        self,
        neutral_foot: Point3D,
        stride_offset: float,
        steering: SteeringState,
    ) -> Point3D:
        """Return a stance-foot target from ``Exp(s * twist * T_stance)``."""
        delta_theta = steering.angular_velocity * self.stance_duration_sec
        theta = stride_offset * delta_theta
        delta_position = np.array(
            [
                float(steering.linear_velocity.x) * self.stance_duration_sec,
                float(steering.linear_velocity.y) * self.stance_duration_sec,
            ]
        )
        position = self._se2_translation(theta, stride_offset * delta_position)
        cosine = math.cos(theta)
        sine = math.sin(theta)
        relative_x = float(neutral_foot.x) - position[0]
        relative_y = float(neutral_foot.y) - position[1]
        return Point3D(
            [
                cosine * relative_x + sine * relative_y,
                -sine * relative_x + cosine * relative_y,
                float(neutral_foot.z),
            ]
        )

    @staticmethod
    def _se2_translation(theta: float, scaled_delta_position: np.ndarray) -> np.ndarray:
        """Apply the SE(2) V(theta) matrix, stably near zero yaw."""
        if abs(theta) < 1e-5:
            sine_over_theta = 1.0 - theta**2 / 6.0 + theta**4 / 120.0
            one_minus_cosine_over_theta = theta / 2.0 - theta**3 / 24.0 + theta**5 / 720.0
        else:
            sine_over_theta = math.sin(theta) / theta
            one_minus_cosine_over_theta = (1.0 - math.cos(theta)) / theta
        return np.array(
            [
                sine_over_theta * scaled_delta_position[0]
                - one_minus_cosine_over_theta * scaled_delta_position[1],
                one_minus_cosine_over_theta * scaled_delta_position[0]
                + sine_over_theta * scaled_delta_position[1],
            ]
        )

    def _saturate_twist(self, proposed: SteeringState) -> TwistSaturation:
        """Scale the complete twist by one scalar until every target is solvable."""
        if self._twist_is_reachable(proposed):
            return TwistSaturation(proposed, 1.0)

        stationary = SteeringState(Point3D([0, 0, 0]), 0.0)
        if not self._twist_is_reachable(stationary):
            return TwistSaturation(stationary, 0.0)

        lower, upper = 0.0, 1.0
        for _ in range(self._SATURATION_STEPS):
            midpoint = (lower + upper) / 2.0
            candidate = self._scale_twist(proposed, midpoint)
            if self._twist_is_reachable(candidate):
                lower = midpoint
            else:
                upper = midpoint
        return TwistSaturation(self._scale_twist(proposed, lower), lower)

    def _twist_is_reachable(self, steering: SteeringState) -> bool:
        """Check the full swing and stance path with analytic IK before commitment."""
        for phase in np.linspace(0.0, 1.0, self._PATH_SAMPLES, endpoint=False):
            for leg, neutral_foot in self.leg_tips_on_ground:
                gait_offsets = self.gait_gen.get_offsets_at_phase_for_leg(leg.label, float(phase))
                target = self._foot_target_for_stride_offset(
                    neutral_foot,
                    float(gait_offsets.x),
                    steering,
                )
                target.z += gait_offsets.z * self.step_height
                solution = leg.solve_ik(target, clamp=False)
                if not solution.reachable or not solution.within_limits:
                    return False
        return True

    @staticmethod
    def _scale_twist(steering: SteeringState, scale: float) -> SteeringState:
        """Return a twist with linear and angular components scaled together."""
        return SteeringState(steering.linear_velocity * scale, steering.angular_velocity * scale)

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

    def apply_feet_targets(self, legs_and_targets: list[tuple[object, Point3D]]) -> None:
        """Apply targets through the legacy visual/notebook kinematics path."""
        for leg, foot_target in legs_and_targets:
            leg.move_to(foot_target)

    @staticmethod
    def _linear_speed(steering: SteeringState) -> float:
        return math.hypot(
            float(steering.linear_velocity.x),
            float(steering.linear_velocity.y),
        )

    def _has_motion(self, steering: SteeringState) -> bool:
        return (
            self._linear_speed(steering) >= self._NO_MOTION_EPSILON
            or abs(steering.angular_velocity) >= self._NO_MOTION_EPSILON
        )
