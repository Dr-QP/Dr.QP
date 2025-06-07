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

from matplotlib import pyplot as plt
from matplotlib.patches import Arc
from matplotlib.transforms import Bbox, IdentityTransform, TransformedBbox
import numpy as np


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
