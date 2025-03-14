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

import numpy as np
import matplotlib.pyplot as plt
from inline_labels import add_inline_labels


# Plot the leg links in 2D space
def plot_leg_links(axes, points):
    # Calculate the coordinates of the leg links
    assert len(points) == 4, 'points must be a list of 4 points'
    colors = ['r', 'g', 'b']
    joint_colors = ['r', 'g', 'b', 'm']
    labels = ['Coxa', 'Femur', 'Tibia']

    def plot_joint(point, color):
        axes.scatter(point.x, point.y, color=color)
        if point.label:
            axes.text(point.x, point.y + 0.2, point.label, color=color)

    last_point = points[0]
    plot_joint(last_point, joint_colors[0])
    for point, color, label, joint_color in zip(points[1:], colors, labels, joint_colors[1:]):
        # Plot the leg link
        axes.plot([last_point.x, point.x], [last_point.y, point.y], color, label=label)

        plot_joint(point, joint_color)

        last_point = point

    # Add inline labels for leg links
    add_inline_labels(axes, with_overall_progress=False, fontsize='medium')
    # labelLines(axes.get_lines(), zorder=2.5, xvals=(1, 1), align=True, fontsize=12)


def plot_leg_links_at_angles(points, title):
    _, ax = plt.subplots(figsize=(10, 10))
    ax.set_title(title)

    plot_leg_links(ax, points)

    # Select length of axes and the space between tick labels
    x, y = np.array([p.x for p in points]), np.array([p.y for p in points])
    xmin, xmax = np.min(x), np.max(x)
    ymin, ymax = np.min(y), np.max(y)
    ticks_frequency = 5

    # Set identical scales for both axes
    ax.set(xlim=(xmin - 1, xmax + 1), ylim=(ymin - 1, ymax + 1), aspect='equal')

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
    x_ticks = np.arange(xmin, xmax + 1, ticks_frequency)
    y_ticks = np.arange(ymin, ymax + 1, ticks_frequency)
    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)

    # Create minor ticks placed at each integer to enable drawing of minor grid
    # lines: note that this has no effect in this example with ticks_frequency=1
    ax.set_xticks(np.arange(xmin, xmax + 1), minor=True)
    ax.set_yticks(np.arange(ymin, ymax + 1), minor=True)

    # Draw major and minor grid lines
    ax.grid(which='both', color='grey', linewidth=1, linestyle='-', alpha=0.2)

    # Draw arrows
    arrow_fmt = dict(markersize=4, color='black', clip_on=False)
    ax.plot((1), (0), marker='>', transform=ax.get_yaxis_transform(), **arrow_fmt)
    ax.plot((0), (1), marker='^', transform=ax.get_xaxis_transform(), **arrow_fmt)
