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

# https://pypi.org/project/pydualsense/ for additional libs install
#
# macOS
# brew install hidapi
#
# linux:
# # sudo apt install libhidapi-dev
import numpy as np

from drqp_brain.geometry import Point3D
from pydualsense import pydualsense

ds = pydualsense()  # open controller
ds.init()  # initialize controller

ds.light.setColorI(0, 0, 0)


class Controls:
    def __init__(self):
        self.direction = Point3D([0, 0, 0])
        self.rotation = 0.0
        self.walk_speed = 0.0
        self.rotation_speed = 0.0

        self.touchpad0 = False
        self.touchpad1 = False

        self.exit = False


def get_controls():
    inputs = Controls()
    stick_rang = [-128, 128]
    direction_y = float(np.interp(ds.state.LX, stick_rang, [1, -1]))
    direction_x = float(np.interp(ds.state.LY, stick_rang, [1, -1]))

    inputs.direction = Point3D([direction_x, direction_y, 0])
    inputs.walk_speed = abs(direction_x) + abs(direction_y)
    inputs.rotation_speed = float(np.interp(ds.state.RX, stick_rang, [1, -1]))

    inputs.touchpad0 = ds.state.trackPadTouch0
    inputs.touchpad1 = ds.state.trackPadTouch1

    inputs.exit = ds.state.circle

    return inputs


def close():
    ds.close()  # closing the controller


def main():
    loop = True
    while loop:
        inputs = get_controls()
        print(f'{inputs.touchpad0=}, {inputs.touchpad1=}')
        loop = not inputs.exit
    close()


if __name__ == '__main__':
    main()
