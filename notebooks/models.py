# Copyright (c) 2017-2025 Anton Matosov
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

import math

from point import Line3D, Point3D
from transforms import Transform


def safe_acos(num):
    if num < -1:
        return False, math.pi  # math.acos(-1)
    elif num > 1:
        return False, 0.0  # math.acos(1)
    else:
        return True, math.acos(num)


class LegModel:
    """A leg model class."""

    def __init__(
        self,
        coxa_length: float,
        femur_length: float,
        tibia_length: float,
        location_on_body=[0, 0, 0],
        rotation=[0, 0, 0],
        body_transform=Transform.identity(),
        label='',
    ):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length

        self.label = label

        self.location_on_body = location_on_body
        self.rotation = rotation
        self._body_transform = body_transform
        self.update_base_transforms()

        self.coxa_link = None
        self.coxa_joint = None
        self.femur_link = None
        self.femur_joint = None
        self.tibia_link = None

        self.body_start = None
        self.body_end = None
        self.coxa_end = None
        self.femur_end = None
        self.tibia_end = None

    @property
    def body_transform(self):
        return self._body_transform

    @body_transform.setter
    def body_transform(self, value):
        self._body_transform = value
        self.update_base_transforms()

    def update_base_transforms(self):
        self.body_link = self.body_transform @ Transform.from_translation(self.location_on_body)
        self.body_joint = self.body_link @ Transform.from_rotvec(self.rotation, degrees=True)

    @property
    def lines(self):
        return [
            Line3D(self.body_start, self.body_end, 'Body'),
            Line3D(self.body_end, self.coxa_end, 'Coxa'),
            Line3D(self.coxa_end, self.femur_end, 'Femur'),
            Line3D(self.femur_end, self.tibia_end, 'Tibia'),
        ]

    @property
    def xy(self):
        return [line.xy for line in self.lines]

    @property
    def xz(self):
        return [line.xz for line in self.lines]

    @property
    def yz(self):
        return [line.yz for line in self.lines]

    def __iter__(self):
        return iter(self.lines)

    def __repr__(self):
        return f'LegModel(body_start={self.body_start}, body_end={self.body_end}, coxa_end={self.coxa_end}, femur_end={self.femur_end}, tibia_end={self.tibia_end})'

    def forward_kinematics(
        self,
        alpha,
        beta,
        gamma,
    ):
        self.update_base_transforms()

        self.coxa_joint = self.body_joint @ Transform.from_rotvec([0, 0, alpha], degrees=True)
        self.coxa_link = self.coxa_joint @ Transform.from_translation([self.coxa_length, 0, 0])

        self.femur_joint = self.coxa_link @ Transform.from_rotvec([0, beta, 0], degrees=True)
        self.femur_link = self.femur_joint @ Transform.from_translation([self.femur_length, 0, 0])

        self.tibia_joint = self.femur_link @ Transform.from_rotvec([0, gamma, 0], degrees=True)
        self.tibia_link = self.tibia_joint @ Transform.from_translation([self.tibia_length, 0, 0])

        # Calculate global positions using transformations
        identity_point = Point3D([0, 0, 0])

        self.body_start = self.body_transform.apply_point(identity_point)

        self.body_end = self.body_link.apply_point(identity_point)
        self.body_end.label = rf'$\alpha$={alpha}°'

        self.coxa_end = self.coxa_link.apply_point(identity_point)
        self.coxa_end.label = rf'$\beta$={beta}°'

        self.femur_end = self.femur_link.apply_point(identity_point)
        self.femur_end.label = rf'$\gamma$={gamma}°'

        self.tibia_end = self.tibia_link.apply_point(identity_point)
        self.tibia_end.label = 'Foot'

    def move_to(self, foot_target: Point3D, verbose=False):
        reached_target, alpha, beta, gamma = self.inverse_kinematics(foot_target, verbose)
        if verbose:
            print(
                f'{self.label} moving to {foot_target}. new angles: {alpha=:.4f}\t{beta=:.4f}\t{gamma=:.4f}'
            )
        self.forward_kinematics(alpha, beta, gamma)
        return reached_target

    def inverse_kinematics(self, foot_target: Point3D, verbose=False):
        localized_foot_target = self.to_local(foot_target)
        alpha, X_tick = self._inverse_kinematics_xy(localized_foot_target)
        solvable, beta, gamma = self._inverse_kinematics_xz(
            localized_foot_target.z,
            X_tick,
            verbose=verbose,
        )
        return solvable, alpha, beta, gamma

    def to_local(self, point):
        return self.body_joint.inverse().apply_point(point)

    @staticmethod
    def _inverse_kinematics_xy(localized_foot_target: Point3D):
        alpha = math.degrees(math.atan2(localized_foot_target.y, localized_foot_target.x))
        X_tick = math.hypot(localized_foot_target.x, localized_foot_target.y)
        return alpha, X_tick

    def _inverse_kinematics_xz(
        self,
        z_offset: float,
        X_tick: float,
        verbose=False,
    ):
        """
        XZ axis Inverse kinematics solver for 3DOF leg.

        Math as described above

        Parameters:
        -----------
        z_offset:
        z offset from the coxa joint of the foot target in meters

        X_tick:
            X distance from the coxa joint of the foot target in meters, returned by the inverse_kinematics_xy
        """
        D = -z_offset
        T = X_tick - self.coxa_length
        L = math.hypot(D, T)

        solvable_theta1, theta1_rad = safe_acos(
            (L**2 + self.femur_length**2 - self.tibia_length**2) / (2 * L * self.femur_length)
        )
        theta1 = math.degrees(theta1_rad)

        theta2 = math.degrees(math.atan2(T, D))
        solvable_phi, phi_rad = safe_acos(
            (self.tibia_length**2 + self.femur_length**2 - L**2)
            / (2 * self.tibia_length * self.femur_length)
        )
        phi = math.degrees(phi_rad)

        # The right hand coordinate system is used, so the angle offsets are inverted
        beta = 90 - (theta1 + theta2)
        gamma = 180 - phi
        if verbose:
            print(
                f'{theta1=} - solvable={solvable_theta1}\n{theta2=}\n{phi=} - solvable={solvable_phi}\n\n{beta=}\n{gamma=}'
            )
        return solvable_theta1 and solvable_phi, beta, gamma
