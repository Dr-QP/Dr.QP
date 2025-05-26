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

from point import Line3D, Point3D
from models import LegModel
from plotting import plot_leg3d, plot_update_leg3d_lines

drqp_front_offset = 0.116924  # x offset for the front and back legs
drqp_side_offset = 0.063871  # y offset fo the front and back legs
drqp_middle_offset = 0.103  # x offset for the middle legs

drqp_coxa = 0.053
drqp_femur = 0.066225
drqp_tibia = 0.123


class DrQP:
    def __init__(self, coxa_len=drqp_coxa, femur_len=drqp_femur, tibia_len=drqp_tibia):
        self.left_front_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='left_front',
            rotation=[0, 0, 45],
            location_on_body=[drqp_front_offset, drqp_side_offset, 0.0],
        )
        self.left_middle_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='left_middle',
            rotation=[0, 0, 90],
            location_on_body=[0.0, drqp_middle_offset, 0.0],
        )
        self.left_back_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='left_back',
            rotation=[0, 0, 135],
            location_on_body=[-drqp_front_offset, drqp_side_offset, 0.0],
        )

        self.right_front_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='right_front',
            rotation=[0, 0, -45],
            location_on_body=[drqp_front_offset, -drqp_side_offset, 0.0],
        )
        self.right_middle_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='right_middle',
            rotation=[0, 0, -90],
            location_on_body=[0.0, -drqp_middle_offset, 0.0],
        )
        self.right_back_leg = LegModel(
            coxa_len,
            femur_len,
            tibia_len,
            label='right_back',
            rotation=[0, 0, -135],
            location_on_body=[-drqp_front_offset, -drqp_side_offset, 0.0],
        )
        self.update_head()

    def forward_kinematics(self, alpha, beta, gamma):
        for leg in self.legs:
            leg.forward_kinematics(alpha, beta, gamma)

    @property
    def legs(self):
        return [
            self.left_front_leg,
            self.left_middle_leg,
            self.left_back_leg,
            self.right_front_leg,
            self.right_middle_leg,
            self.right_back_leg,
        ]

    @property
    def body_transform(self):
        return self.left_front_leg.body_transform

    @body_transform.setter
    def body_transform(self, value):
        for leg in self.legs:
            leg.body_transform = value

        self.update_head()

    def update_head(self):
        # Head, x-forward
        self.head = Line3D(
            self.body_transform.apply_point(Point3D([0, 0, 0])),
            self.body_transform.apply_point(Point3D([drqp_front_offset, 0, 0])),
            'Head',
        )


def plot_drqp(drqp, targets=None):
    fig, ax = None, None

    class PlotData:
        def __init__(self):
            self.leg_plot_data = []
            self.head_line = None

    plot_data = PlotData()

    for leg in drqp.legs:
        fig, ax, leg_plot_data = plot_leg3d(
            leg,
            'Hexapod in 3D',
            link_labels='none',
            joint_labels='points',
            subplot=111,
            fig=fig,
            ax=ax,
        )
        plot_data.leg_plot_data.append(leg_plot_data)

    plot_data.head_line = ax.plot(*zip(drqp.head.start, drqp.head.end), 'c')[0]

    if targets:
        ax.scatter(
            *zip(*[target.numpy() for target in targets]),
            color='k',
            label='unreachable target',
        )

    ax.view_init(elev=44.0, azim=-160)
    return fig, ax, plot_data


def update_drqp_plot(drqp, plot_data):
    for leg, leg_plot_data in zip(drqp.legs, plot_data.leg_plot_data):
        plot_update_leg3d_lines(leg, leg_plot_data)

    if plot_data.head_line:
        plot_data.head_line.set_data_3d(*zip(drqp.head.start, drqp.head.end))
