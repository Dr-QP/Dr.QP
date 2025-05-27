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

from typing import Literal

import matplotlib.pyplot as plt
from models import HexapodModel
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
from point import Leg3D

from .leg import LegPlotData, plot_leg_links


class HexapodPlotData:
    def __init__(self):
        self.leg_plot_data = []
        self.head_line = None
        self.leg_tips = {}
        self.leg_tip_collections = {}
        self.feet_trails_frames = False


def plot_hexapod(hexapod: HexapodModel, targets=None, feet_trails_frames=0):
    fig, ax = None, None

    plot_data = HexapodPlotData()
    plot_data.feet_trails_frames = feet_trails_frames

    for leg in hexapod.legs:
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

        if feet_trails_frames > 0:
            plot_data.leg_tips[leg.label] = [leg.tibia_end.numpy()]
            segments = leg_tips_to_segments(plot_data.leg_tips[leg.label])

            lc = Line3DCollection(segments, cmap='plasma')

            colors = np.linspace(1, 0, plot_data.feet_trails_frames)
            lc.set_array(colors)
            lc.set_linewidth(2)

            plot_data.leg_tip_collections[leg.label] = ax.add_collection(lc)

    plot_data.head_line = ax.plot(*zip(hexapod.head.start, hexapod.head.end), 'c')[0]

    if targets:
        ax.scatter(
            *zip(*[target.numpy() for target in targets]), color='k', label='unreachable target'
        )

    ax.view_init(elev=44.0, azim=-160)
    return fig, ax, plot_data


def leg_tips_to_segments(leg_tips):
    points = np.array(leg_tips)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    segments = segments.reshape(-1, 2, 3)
    return segments


def update_hexapod_plot(hexapod: HexapodModel, plot_data: HexapodPlotData):
    for leg, leg_plot_data in zip(hexapod.legs, plot_data.leg_plot_data):
        plot_update_leg3d_lines(leg, leg_plot_data)

    if plot_data.head_line:
        plot_data.head_line.set_data_3d(*zip(hexapod.head.start, hexapod.head.end))

    if plot_data.feet_trails_frames > 0:
        for leg in hexapod.legs:
            if len(plot_data.leg_tips[leg.label]) > plot_data.feet_trails_frames:
                plot_data.leg_tips[leg.label].pop(0)
            plot_data.leg_tips[leg.label].append(leg.tibia_end.numpy())

            segments = leg_tips_to_segments(plot_data.leg_tips[leg.label])
            lc = plot_data.leg_tip_collections[leg.label]
            lc.set_segments(segments)
