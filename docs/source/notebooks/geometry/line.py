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

from .point import Point, Point3D


class Line:
    """A simple 2D line class."""

    def __init__(self, start: Point, end: Point, label: str):
        self.start = start
        self.end = end
        self.label = label

    def extended(self, length: float = 1.0):
        return Line(
            self.start, self.end + (self.end - self.start).normalized() * length, self.label
        )

    def __repr__(self):
        return f'Line({self.start}, {self.end}, {self.label})'


class Line3D:
    """A simple 3D line class."""

    def __init__(self, start: Point3D, end: Point3D, label: str):
        self.start = start
        self.end = end
        self.label = label

    def extended(self, length: float = 1.0):
        return Line3D(
            self.start, self.end + (self.end - self.start).normalized() * length, self.label
        )

    @property
    def xy(self):
        return Line(self.start.xy, self.end.xy, self.label)

    @property
    def xz(self):
        return Line(self.start.xz, self.end.xz, self.label)

    @property
    def yz(self):
        return Line(self.start.yz, self.end.yz, self.label)
