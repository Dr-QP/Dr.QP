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

from inline_labels import add_inline_labels
import matplotlib.pyplot as plt
import numpy as np
from point import Point


# Plot the leg links in 2D space
def plot_leg_links(axes, points, no_labels=False):
    # Calculate the coordinates of the leg links
    assert len(points) == 4, 'points must be a list of 4 points'
    colors = ['r', 'g', 'b']
    joint_colors = ['r', 'g', 'b', 'm']

    if no_labels:
        labels = [''] * 3
    else:
        labels = ['Coxa', 'Femur', 'Tibia']

    result_lines = []
    result_joints = []

    def plot_joint(point, color):
        joint = axes.scatter(point.x, point.y, color=color)
        if not no_labels and point.label:
            axes.text(point.x, point.y + 0.2, point.label, color=color)
        result_joints.append(joint)

    last_point = points[0]
    plot_joint(last_point, joint_colors[0])
    for point, color, label, joint_color in zip(points[1:], colors, labels, joint_colors[1:]):
        # Plot the leg link
        result_lines += axes.plot(
            [last_point.x, point.x], [last_point.y, point.y], color, label=label
        )

        plot_joint(point, joint_color)

        last_point = point

    # Add inline labels for leg links
    if not no_labels:
        add_inline_labels(axes, with_overall_progress=False, fontsize='medium')

    return result_lines, result_joints


def plot_cartesian_plane(ax, min: Point, max: Point, ticks_frequency=1):
    # Set identical scales for both axes
    ax.set(xlim=(min.x - 1, max.y + 1), ylim=(min.x - 1, max.y + 1), aspect='equal')

    # Set bottom and left spines as x and y axes of coordinate system
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')

    # Remove top and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Create 'x' and 'y' labels placed at the end of the axes
    ax.set_xlabel('x', size=14, labelpad=-24, x=1.03)
    ax.set_ylabel('y', size=14, labelpad=-21, y=1.02, rotation=0)

    # Create custom major ticks to determine position of tick labels
    x_ticks = np.arange(min.x, max.x + 1, ticks_frequency)
    y_ticks = np.arange(min.y, max.y + 1, ticks_frequency)
    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)

    # Create minor ticks placed at each integer to enable drawing of minor grid
    # lines: note that this has no effect in this example with ticks_frequency=1
    ax.set_xticks(np.arange(min.x, max.x + 1), minor=True)
    ax.set_yticks(np.arange(min.y, max.y + 1), minor=True)

    # Draw major and minor grid lines
    ax.grid(which='both', color='grey', linewidth=1, linestyle='-', alpha=0.2)

    # Draw arrows
    arrow_fmt = dict(markersize=4, color='black', clip_on=False)
    ax.plot((1), (0), marker='>', transform=ax.get_yaxis_transform(), **arrow_fmt)
    ax.plot((0), (1), marker='^', transform=ax.get_xaxis_transform(), **arrow_fmt)


def plot_leg_with_points(points: list[Point], title: str, no_labels=False):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_title(title)

    result_lines, result_joints = plot_leg_links(ax, points, no_labels)

    # Select length of axes and the space between tick labels
    x, y = np.array([p.x for p in points]), np.array([p.y for p in points])
    min = Point(np.min(x), np.min(y))
    max = Point(np.max(x), np.max(y))

    plot_cartesian_plane(ax, min, max, ticks_frequency=5)

    return fig, result_lines, result_joints


def plot_leg_update_lines(points, lines, joints):
    last_point = points[0]
    joints[0].set_offsets([last_point.x, last_point.y])
    for line, point, joint in zip(lines, points[1:], joints[1:]):
        line.set_data([last_point.x, point.x], [last_point.y, point.y])
        joint.set_offsets([point.x, point.y])
        last_point = point

    return lines
