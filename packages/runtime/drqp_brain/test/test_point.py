# Copyright (c) 2017-present Anton Matosov
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


from drqp_brain.geometry.point import Point, Point3D, SimplePoint3D
import numpy as np


class TestPoint:
    """Test the Point class."""

    def test_initialization(self):
        point = Point(1, 2, 'Test')
        assert point.x == 1
        assert point.y == 2
        assert point.label == 'Test'

    def test_addition(self):
        point1 = Point(1, 2, 'Test')
        point2 = Point(3, 4, 'Test')
        assert point1 + point2 == Point(4, 6, 'Test')

    def test_subtraction(self):
        point1 = Point(1, 2, 'Test')
        point2 = Point(3, 4, 'Test')
        assert point1 - point2 == Point(-2, -2, 'Test')

    def test_multiplication(self):
        point1 = Point(1, 2, 'Test')
        point2 = Point(3, 4, 'Test')
        assert point1 * point2 == Point(3, 8, 'Test')

    def test_division(self):
        point1 = Point(1, 2, 'Test')
        point2 = Point(3, 4, 'Test')
        assert point1 / point2 == Point(1 / 3, 1 / 2, 'Test')

    def test_rotation(self):
        point = Point(1, 0, 'Test')
        assert point.rotate(np.pi / 2) == Point(0, 1, 'Test')

    def test_normalized(self):
        point = Point(1, 1, 'Test')
        assert point.normalized() == Point(1 / np.sqrt(2), 1 / np.sqrt(2), 'Test')

    def test_numpy(self):
        point = Point(1, 2, 'Test')
        assert np.allclose(point.numpy(), np.array([1, 2]))


class TestPoint3D:
    """Test the Point3D class."""

    def test_initialization(self):
        point = Point3D([1, 2, 3], 'Test')
        assert point.x == 1
        assert point.y == 2
        assert point.z == 3
        assert point.label == 'Test'

    def test_xyz_setters(self):
        point = Point3D([1, 2, 3], 'Test')
        point.x = 4
        point.y = 5
        point.z = 6
        assert point.x == 4
        assert point.y == 5
        assert point.z == 6

    def test_addition(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([4, 5, 6], 'Test')
        assert point1 + point2 == Point3D([5, 7, 9], 'Test')

    def test_subtraction(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([4, 5, 6], 'Test')
        assert point1 - point2 == Point3D([-3, -3, -3], 'Test')

    def test_multiplication(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([4, 5, 6], 'Test')
        assert point1 * point2 == Point3D([4, 10, 18], 'Test')

    def test_division(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([4, 5, 6], 'Test')
        assert point1 / point2 == Point3D([1 / 4, 2 / 5, 3 / 6], 'Test')

    def test_normalized(self):
        point = Point3D([1, 1, 1], 'Test')
        assert point.normalized() == Point3D(
            [1 / np.sqrt(3), 1 / np.sqrt(3), 1 / np.sqrt(3)], 'Test'
        )

    def test_numpy(self):
        point = Point3D([1, 2, 3], 'Test')
        assert np.allclose(point.numpy(), np.array([1, 2, 3]))

    def test_interpolation(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([4, 5, 6], 'Test')
        assert point1.interpolate(point2, 0.5) == Point3D([2.5, 3.5, 4.5], 'Test')

    def test_copy(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = point1.copy()
        assert point1 == point2
        assert point1 is not point2

    def test_xy(self):
        point = Point3D([1, 2, 3], 'Test')
        assert point.xy == Point(1, 2, 'Test')

    def test_xz(self):
        point = Point3D([1, 2, 3], 'Test')
        assert point.xz == Point(1, 3, 'Test')

    def test_yz(self):
        point = Point3D([1, 2, 3], 'Test')
        assert point.yz == Point(2, 3, 'Test')

    def test_eq(self):
        point1 = Point3D([1, 2, 3], 'Test')
        point2 = Point3D([1, 2, 3], 'Test')
        assert point1 == point2

    def test_to_vector3(self):
        """Test conversion to geometry_msgs/Vector3."""
        from geometry_msgs.msg import Vector3

        point = Point3D([1.5, 2.5, 3.5], 'Test')
        vec = point.to_vector3()
        assert isinstance(vec, Vector3)
        assert vec.x == 1.5
        assert vec.y == 2.5
        assert vec.z == 3.5


class TestSimplePoint3D:
    """Test the SimplePoint3D class."""

    def test_initialization(self):
        point = SimplePoint3D(1, 2, 3, 'Test')
        assert point.x == 1
        assert point.y == 2
        assert point.z == 3
        assert point.label == 'Test'

    def test_numpy(self):
        point = SimplePoint3D(1, 2, 3, 'Test')
        assert np.allclose(point.numpy(), np.array([1, 2, 3]))

    def test_xy(self):
        point = SimplePoint3D(1, 2, 3, 'Test')
        assert point.xy == Point(1, 2, 'Test')

    def test_xz(self):
        point = SimplePoint3D(1, 2, 3, 'Test')
        assert point.xz == Point(1, 3, 'Test')

    def test_yz(self):
        point = SimplePoint3D(1, 2, 3, 'Test')
        assert point.yz == Point(2, 3, 'Test')
