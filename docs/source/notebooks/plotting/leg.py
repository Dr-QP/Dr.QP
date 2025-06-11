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

from drqp_brain.geometry import Leg3D, Line, Line3D, Point
from inline_labels import add_inline_labels
import matplotlib.pyplot as plt
import numpy as np

from .angle_annotation import AngleAnnotation

link_labels_type = Literal['inline', 'legend', 'label', 'none']
joint_labels_type = Literal['annotated', 'points', 'none']


class LegPlotData:
    def __init__(self):
        self.lines = []
        self.joints = []
        self.joint_labels = []
        self.link_labels = []


# Plot the leg links in 2D space
def plot_leg_links(
    axes: plt.Axes,
    model: list[Line] | list[Line3D],
    link_labels: link_labels_type,
    joint_labels: joint_labels_type,
):
    link_colors = ['c', 'r', 'g', 'b', 'm']
    joint_colors = link_colors[1:]

    plot_data = LegPlotData()

    line_i = 0
    for line, color in zip(model, link_colors):
        label = line.label if link_labels != 'none' else None
        plot_data.lines += axes.plot(*zip(line.start, line.end), color, label=label)
        if joint_labels == 'annotated' and not line_i == len(model) - 1:
            # Extend the line to make the joint angles easier to understand
            plot_data.lines += axes.plot(*zip(line.end, line.extended(3).end), color + ':')
        line_i += 1

    if joint_labels == 'annotated' or joint_labels == 'points':
        for line, joint_color in zip(model, joint_colors):
            joint = axes.scatter(*line.end.numpy(), color=joint_color)
            plot_data.joints.append(joint)

    # Add inline labels for leg links
    if link_labels == 'legend':
        axes.legend(bbox_to_anchor=(1.0, 0.97), loc='upper right')
    elif link_labels == 'inline':
        add_inline_labels(axes, with_overall_progress=False, fontsize='medium')

    if joint_labels == 'annotated':
        for i in range(len(model) - 1):
            line = model[i]
            next_line = model[i + 1]
            joint_color = joint_colors[i]

            # Sort the vectors by y coordinate to always display angle on the correct side
            vecs = [
                next_line.end.numpy(),
                line.extended().end.numpy(),
            ]
            vecs.sort(key=lambda v: v[1])
            if line.end.label:
                AngleAnnotation(
                    line.end.numpy(),
                    *vecs,
                    ax=axes,
                    size=50,
                    text=line.end.label,
                    color=joint_color,
                    linestyle='--',
                    textposition='outside',
                    text_kw={'fontsize': 10, 'color': joint_color},
                )

    return plot_data


def plot_leg_with_points(
    model: list[Line],
    title: str,
    link_labels: link_labels_type = 'inline',
    joint_labels: joint_labels_type = 'annotated',
    no_cartesian_ticks=False,
    x_label='X',
    y_label='Y',
    subplot=111,
    fig=None,
    ax=None,
):
    if fig is None:
        fig = plt.figure()

    if ax is None:
        ax = fig.add_subplot(subplot)
        ax.set_title(title)

    plot_data = plot_leg_links(ax, model, link_labels=link_labels, joint_labels=joint_labels)

    # Select length of axes and the space between tick labels
    x = np.array([[line.start.x, line.end.x] for line in model])
    y = np.array([[line.start.y, line.end.y] for line in model])
    plot_min = Point(np.min(x), np.min(y))
    plot_max = Point(np.max(x), np.max(y))

    def extra_space(v):
        return max(2, abs(v) * 0.2)

    plot_min.x -= extra_space(plot_min.x)
    plot_min.y -= extra_space(plot_min.y)

    plot_max.x += extra_space(plot_max.x)
    plot_max.y += extra_space(plot_max.y)

    plot_cartesian_plane(
        ax,
        plot_min,
        plot_max,
        ticks_frequency=5,
        no_ticks=no_cartesian_ticks,
        x_label=x_label,
        y_label=y_label,
    )

    return fig, ax, plot_data


def plot_cartesian_plane(
    ax: plt.Axes,
    plot_min: Point,
    plot_max: Point,
    ticks_frequency=1,
    no_ticks=False,
    x_label='X',
    y_label='Y',
):
    # Set identical scales for both axes
    ax.set(xlim=(plot_min.x, plot_max.x), ylim=(plot_min.y, plot_max.y), aspect='equal')

    # Set bottom and left spines as x and y axes of coordinate system
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')

    # Remove top and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Create 'x' and 'y' labels placed at the end of the axes
    ax.set_xlabel(x_label, size=14, labelpad=-24, x=1.03)
    ax.set_ylabel(y_label, size=14, labelpad=2, y=0.96, rotation=0)

    if no_ticks:
        ax.set_xticks([])
        ax.set_yticks([])
    else:
        # Create custom major ticks to determine position of tick labels
        x_ticks = np.arange(plot_min.x, plot_max.x, ticks_frequency, dtype=int)
        y_ticks = np.arange(plot_min.y, plot_max.y, ticks_frequency, dtype=int)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)

    # Create minor ticks placed at each integer to enable drawing of minor grid
    # lines: note that this has no effect in this example with ticks_frequency=1
    ax.set_xticks(np.arange(plot_min.x, plot_max.x, dtype=int), minor=True)
    ax.set_yticks(np.arange(plot_min.y, plot_max.y, dtype=int), minor=True)

    # Draw major and minor grid lines
    ax.grid(which='both', color='grey', linewidth=1, linestyle='-', alpha=0.2)

    # Draw arrows
    arrow_fmt = {'markersize': 4, 'color': 'black', 'clip_on': False}
    ax.plot((1), (0), marker='>', transform=ax.get_yaxis_transform(), **arrow_fmt)
    ax.plot((0), (1), marker='^', transform=ax.get_xaxis_transform(), **arrow_fmt)


def plot_leg_update_lines(model, plot_data: LegPlotData):
    for line, line_model in zip(plot_data.lines, model):
        line.set_data(*zip(line_model.start, line_model.end))

    for joint, line_model in zip(plot_data.joints, model):
        joint.set_offsets([line_model.end.x, line_model.end.y])


def plot_ik_lines(ax, femur, tibia):
    d_end = Point(femur.start.x, tibia.end.y)
    ax.plot(*zip(femur.start, tibia.end), 'm--', label='L')
    ax.plot(*zip(femur.start, d_end), 'm--', label='D')
    ax.plot(*zip(tibia.end, d_end), 'm--', label='T')

    AngleAnnotation(
        femur.start.numpy(),
        tibia.end.numpy(),
        femur.end.numpy(),
        ax=ax,
        size=50,
        text=r'$\theta$1',
        color='m',
        linestyle='--',
        textposition='outside',
        text_kw={'fontsize': 10, 'color': 'm'},
    )
    AngleAnnotation(
        femur.start.numpy(),
        d_end.numpy(),
        tibia.end.numpy(),
        ax=ax,
        size=50,
        text=r'$\theta$2',
        color='m',
        linestyle='--',
        textposition='outside',
        text_kw={'fontsize': 10, 'color': 'm'},
    )

    AngleAnnotation(
        femur.end.numpy(),
        femur.start.numpy(),
        tibia.end.numpy(),
        ax=ax,
        size=50,
        text=r'$\Phi$',
        color='m',
        linestyle='--',
        textposition='outside',
        text_kw={'fontsize': 10, 'color': 'm'},
    )

    add_inline_labels(ax, with_overall_progress=False, fontsize='medium')
    if hasattr(ax, '_legend'):
        ax._legend.remove()  # remove legend as labels are added inline


def plot_update_leg3d_lines(leg: Leg3D, leg_plot_data: LegPlotData):
    for line, line_model in zip(leg_plot_data.lines, leg.lines):
        line.set_data_3d(*zip(line_model.start, line_model.end))

    for joint, line_model in zip(leg_plot_data.joints, leg.lines):
        joint._offsets3d = ([line_model.end.x], [line_model.end.y], [line_model.end.z])


def plot_leg3d(
    model: Leg3D,
    title: str,
    link_labels: Literal['legend', 'label', 'none'] = 'legend',
    joint_labels: Literal['points', 'none'] = 'points',
    subplot=111,
    fig=None,
    ax=None,
    hide_grid=False,
):
    if fig is None:
        fig = plt.figure()

    if ax is None:
        ax = fig.add_subplot(subplot, projection='3d')
        ax.set_title(title)

    assert link_labels != 'inline', 'Inline labels not supported in 3D plots'
    assert joint_labels != 'annotated', 'Joint annotations not supported in 3D plots'

    plot_data = plot_leg_links(ax, model.lines, link_labels=link_labels, joint_labels=joint_labels)

    # Doesn't really add anything to the plot
    # plot_cartesian_plane(ax, Point(-10, -10), Point(10, 10), no_ticks=True)

    ax.set_aspect('equal')
    ax.set_facecolor('white')
    if hide_grid:
        ax.grid(False)

        # Hide axes ticks
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])

        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((0, 0.2, 0, 0.5))

        # Hide grid lines
        ax.zaxis.gridlines.set_visible(False)

    return fig, ax, plot_data
