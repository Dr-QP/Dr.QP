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

from inline_labels import add_inline_labels
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox, IdentityTransform, TransformedBbox
from models import HexapodModel
import numpy as np
from point import Leg3D, Line, Line3D, Point


class AngleAnnotation(Arc):
    """
    Draws an arc between two vectors which appears circular in display space.

    From https://matplotlib.org/stable/gallery/text_labels_and_annotations/angle_annotation.html
    """

    def __init__(
        self,
        xy,
        p1,
        p2,
        size=75,
        unit='points',
        ax=None,
        text='',
        textposition='inside',
        text_kw=None,
        **kwargs,
    ):
        """
        Draws an arc between p1 and p2 which appears circular in display space.

        Parameters
        ----------
        xy, p1, p2 : tuple or array of two floats
            Center position and two points. Angle annotation is drawn between
            the two vectors connecting *p1* and *p2* with *xy*, respectively.
            Units are data coordinates.

        size : float
            Diameter of the angle annotation in units specified by *unit*.

        unit : str
            One of the following strings to specify the unit of *size*:

            * "pixels": pixels
            * "points": points, use points instead of pixels to not have a
              dependence on the DPI
            * "axes width", "axes height": relative units of Axes width, height
            * "axes min", "axes max": minimum or maximum of relative Axes
              width, height

        ax : `matplotlib.axes.Axes`
            The Axes to add the angle annotation to.

        text : str
            The text to mark the angle with.

        textposition : {"inside", "outside", "edge"}
            Whether to show the text in- or outside the arc. "edge" can be used
            for custom positions anchored at the arc's edge.

        text_kw : dict
            Dictionary of arguments passed to the Annotation.

        **kwargs
            Further parameters are passed to `matplotlib.patches.Arc`. Use this
            to specify, color, linewidth etc. of the arc.

        """
        self.ax = ax or plt.gca()
        self._xydata = xy  # in data coordinates
        self.vec1 = p1
        self.vec2 = p2
        self.size = size
        self.unit = unit
        self.textposition = textposition

        super().__init__(
            self._xydata, size, size, angle=0.0, theta1=self.theta1, theta2=self.theta2, **kwargs
        )

        self.set_transform(IdentityTransform())
        self.ax.add_patch(self)

        self.kw = {
            'ha': 'center',
            'va': 'center',
            'xycoords': IdentityTransform(),
            'xytext': (0, 0),
            'textcoords': 'offset points',
            'annotation_clip': True,
        }
        self.kw.update(text_kw or {})
        self.text = ax.annotate(text, xy=self._center, **self.kw)

    def get_size(self):
        factor = 1.0
        if self.unit == 'points':
            factor = self.ax.figure.dpi / 72.0
        elif self.unit[:4] == 'axes':
            b = TransformedBbox(Bbox.unit(), self.ax.transAxes)
            dic = {
                'max': max(b.width, b.height),
                'min': min(b.width, b.height),
                'width': b.width,
                'height': b.height,
            }
            factor = dic[self.unit[5:]]
        return self.size * factor

    def set_size(self, size):
        self.size = size

    def get_center_in_pixels(self):
        """Return center in pixels."""
        return self.ax.transData.transform(self._xydata)

    def set_center(self, xy):
        """Set center in data coordinates."""
        self._xydata = xy

    def get_theta(self, vec):
        vec_in_pixels = self.ax.transData.transform(vec) - self._center
        return np.rad2deg(np.arctan2(vec_in_pixels[1], vec_in_pixels[0]))

    def get_theta1(self):
        return self.get_theta(self.vec1)

    def get_theta2(self):
        return self.get_theta(self.vec2)

    def set_theta(self, angle):
        pass

    # Redefine attributes of the Arc to always give values in pixel space
    _center = property(get_center_in_pixels, set_center)
    theta1 = property(get_theta1, set_theta)
    theta2 = property(get_theta2, set_theta)
    width = property(get_size, set_size)
    height = property(get_size, set_size)

    # The following two methods are needed to update the text position.
    def draw(self, renderer):
        self.update_text()
        super().draw(renderer)

    def update_text(self):
        c = self._center
        s = self.get_size()
        angle_span = (self.theta2 - self.theta1) % 360
        # print(f'{angle_span=}')
        angle = np.deg2rad(self.theta1 + angle_span / 2)
        r = s / 2
        if self.textposition == 'inside':
            r = s / np.interp(angle_span, [60, 90, 135, 180], [3.3, 3.5, 3.8, 4])
        self.text.xy = c + r * np.array([np.cos(angle), np.sin(angle)])
        if self.textposition == 'outside':
            # The goal is to place text at an appropriate distance from the center
            # of an arc while avoiding overlap with the arc itself.
            def R90(a, r, w, h):
                """
                Calculate the distance needed to place text at angle a without overlapping the arc.

                It handles the case when the angle is in the first quadrant (0-90°)
                The function has two cases:
                 - For small angles: Uses a simpler calculation based on the tangent
                 - For larger angles: Uses a more complex geometric calculation involving the
                   diagonal of the text box

                Parameters
                ----------
                a:
                  angle in radians

                r:
                  radius of the arc

                w, h:
                  width and height of the text bounding box

                """
                if a < np.arctan(h / 2 / (r + w / 2)):
                    return np.sqrt((r + w / 2) ** 2 + (np.tan(a) * (r + w / 2)) ** 2)
                else:
                    c = np.sqrt((w / 2) ** 2 + (h / 2) ** 2)
                    T = np.arcsin(c * np.cos(np.pi / 2 - a + np.arcsin(h / 2 / c)) / r)
                    xy = r * np.array([np.cos(a + T), np.sin(a + T)])
                    xy += np.array([w / 2, h / 2])
                    return np.sqrt(np.sum(xy**2))

            def R(a, r, w, h):
                """
                Extend R90 to handle angles in all quadrants.

                Converting any angle to an equivalent angle in the first quadrant ( aa)
                Swapping width and height parameters depending on which quadrant the angle is in

                Parameters
                ----------
                a:
                  angle in radians

                r:
                  radius of the arc

                w, h:
                  width and height of the text bounding box

                """
                angle_mod_first_quadrant = a % (np.pi / 2)
                angle_is_under_45_degrees = angle_mod_first_quadrant <= np.pi / 4
                if angle_is_under_45_degrees:
                    aa = a % (np.pi / 4)
                else:
                    aa = np.pi / 4 - (a % (np.pi / 4))

                # swap w, h if a is in 2nd or 3rd quadrant
                return R90(aa, r, *[w, h][:: int(np.sign(np.cos(2 * a)))])

            # Gets the actual pixel dimensions of the text
            # Calculates the optimal distance X using the R function
            # Converts from pixel units to points (72 points = 1 inch)
            # Sets the text position using polar coordinates (distance and angle)
            bbox = self.text.get_window_extent()
            X = R(angle, r, bbox.width, bbox.height)
            trans = self.ax.figure.dpi_scale_trans.inverted()
            offs = trans.transform(((X - s / 2), 0))[0] * 72

            y_offs = 0
            if abs(angle_span) <= 15:
                y_offs = bbox.height / 2

            self.text.set_position([offs * np.cos(angle), offs * np.sin(angle) + y_offs])


link_labels_type = Literal['inline', 'legend', 'label', 'none']
joint_labels_type = Literal['annotated', 'points', 'none']


def plot_leg3d(
    model: Leg3D,
    title: str,
    link_labels: Literal['legend', 'label', 'none'] = 'legend',
    joint_labels: Literal['points', 'none'] = 'points',
    subplot=111,
    fig=None,
    ax=None,
):
    if fig is None:
        fig = plt.figure()

    if ax is None:
        ax = fig.add_subplot(subplot, projection='3d')
        ax.set_title(title)

    assert link_labels != 'inline', 'Inline labels not supported in 3D plots'
    assert joint_labels != 'annotated', 'Joint annotations not supported in 3D plots'

    result_lines, result_joints = plot_leg_links(
        ax, model.lines, link_labels=link_labels, joint_labels=joint_labels
    )

    # Doesn't really add anything to the plot
    # plot_cartesian_plane(ax, Point(-10, -10), Point(10, 10), no_ticks=True)

    ax.set(aspect='equal')
    # Hide grid lines
    ax.grid(False)
    # ax.grid(False, which='both', axis='z')

    # Hide axes ticks
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    ax.set_facecolor('white')
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((0, 0.2, 0, 0.5))
    ax.zaxis.line.set_visible(False)
    ax.zaxis.gridlines.set_visible(False)

    return fig, ax, result_lines, result_joints


# Plot the leg links in 2D space
def plot_leg_links(
    axes: plt.Axes,
    model: list[Line] | list[Line3D],
    link_labels: link_labels_type,
    joint_labels: joint_labels_type,
):
    link_colors = ['c', 'r', 'g', 'b', 'm']
    joint_colors = link_colors[1:]

    result_lines = []
    result_joints = []

    line_i = 0
    for line, color in zip(model, link_colors):
        label = line.label if link_labels != 'none' else None
        result_lines += axes.plot(*zip(line.start, line.end), color, label=label)
        if joint_labels == 'annotated' and not line_i == len(model) - 1:
            # Extend the line to make the joint angles easier to understand
            result_lines += axes.plot(*zip(line.end, line.extended(3).end), color + ':')
        line_i += 1

    if joint_labels == 'annotated' or joint_labels == 'points':
        for line, joint_color in zip(model, joint_colors):
            joint = axes.scatter(*line.end.numpy(), color=joint_color)
            result_joints.append(joint)

    # Add inline labels for leg links
    if link_labels == 'legend':
        axes.legend()
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

    return result_lines, result_joints


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

    result_lines, result_joints = plot_leg_links(
        ax, model, link_labels=link_labels, joint_labels=joint_labels
    )

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

    return fig, ax, result_lines, result_joints


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


def plot_leg_update_lines(model, lines, joints):
    for line, line_model in zip(lines, model):
        line.set_data(*zip(line_model.start, line_model.end))

    for joint, line_model in zip(joints, model):
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
    ax.legend().remove()  # remove legend as labels are added inline


class HexapodPlotData:
    def __init__(self):
        self.leg_lines = []
        self.leg_joints = []
        self.head_line = None


def plot_hexapod(hexapod: HexapodModel, targets=None):
    fig, ax = None, None

    plot_data = HexapodPlotData()

    for leg in hexapod.legs:
        fig, ax, lines, joints = plot_leg3d(
            leg,
            'Hexapod in 3D',
            link_labels='none',
            joint_labels='points',
            subplot=111,
            fig=fig,
            ax=ax,
        )
        plot_data.leg_lines.append(lines)
        plot_data.leg_joints.append(joints)

    plot_data.head_line = ax.plot(*zip(hexapod.head.start, hexapod.head.end), 'c')[0]

    if targets:
        ax.scatter(
            *zip(*[target.numpy() for target in targets]), color='k', label='unreachable target'
        )

    ax.view_init(elev=44.0, azim=-160)
    return fig, ax, plot_data


def update_hexapod_plot(hexapod: HexapodModel, plot_data: HexapodPlotData):
    for leg, lines, joints in zip(hexapod.legs, plot_data.leg_lines, plot_data.leg_joints):
        plot_update_leg3d_lines(leg, lines, joints)

    if plot_data.head_line:
        plot_data.head_line.set_data_3d(*zip(hexapod.head.start, hexapod.head.end))


def plot_update_leg3d_lines(leg: Leg3D, lines, joints):
    for line, line_model in zip(lines, leg.lines):
        line.set_data_3d(*zip(line_model.start, line_model.end))

    for joint, line_model in zip(joints, leg.lines):
        joint._offsets3d = ([line_model.end.x], [line_model.end.y], [line_model.end.z])
