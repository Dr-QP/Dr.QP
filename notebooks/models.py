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

from transforms import Transform
from point import Line3D, Point3D


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
    ):
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length

        self.location_on_body = location_on_body
        self.rotation = rotation
        self.body_transform = body_transform
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
