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


class Point:
    """A simple 2D point class to make math less verbose"""

    def __init__(self, x, y, label=None):
        self.x = x
        self.y = y
        self.label = label

    def __repr__(self):
        return f'Point({self.x}, {self.y}, {self.label})'

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, other.label)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, other.label)

    def __mul__(self, other):
        return Point(self.x * other, self.y * other, other.label)

    # cast to numpy array
    def __array__(self):
        return self.numpy()

    def numpy(self):
        return np.array([self.x, self.y])

    def rotate(self, angle):
        """
        Rotate the point by the given angle

        https://en.wikipedia.org/wiki/Rotation_matrix
        """
        x = self.x * np.cos(angle) - self.y * np.sin(angle)
        y = self.x * np.sin(angle) + self.y * np.cos(angle)
        return Point(x, y, self.label)
