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

# https://pypi.org/project/pydualsense/ for additional libs install
#
# macOS
# brew install hidapi
#
# linux:
# # sudo apt install libhidapi-dev
import curses
import time

from drqp_brain.geometry import Point3D
import numpy as np
from pydualsense import pydualsense


def clear_console():
    # print('\033[4A\033[2K', end='')
    print('\033[2K', end='')


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


def print_trackpad_touch(stdscr, touch):
    # 'Dualsense Touchpad class. Contains X and Y position of touch and if the touch isActive'
    stdscr.addstr(f'id={touch.ID}, x={touch.X}, y={touch.Y}, isActive={touch.isActive}\n')


def main(stdscr):
    loop = True
    while loop:
        inputs = get_controls()

        stdscr.clear()
        print_trackpad_touch(stdscr, inputs.touchpad0)
        print_trackpad_touch(stdscr, inputs.touchpad1)
        stdscr.addstr(f'ps: {ds.state.ps}\n')
        stdscr.addstr(f'share: {ds.state.share}\n')
        stdscr.addstr(f'options: {ds.state.options}\n')
        stdscr.addstr(f'micBtn: {ds.state.micBtn}\n')
        stdscr.addstr(f'touch1: {ds.state.touch1}\n')
        stdscr.addstr(f'touch2: {ds.state.touch2}\n')
        stdscr.addstr(f'touchBtn: {ds.state.touchBtn}\n')
        stdscr.addstr(f'touchFinger1: {ds.state.touchFinger1}\n')
        stdscr.addstr(f'touchFinger2: {ds.state.touchFinger2}\n')
        stdscr.addstr(f'touchLeft: {ds.state.touchLeft}\n')
        stdscr.addstr(f'touchRight: {ds.state.touchRight}\n')
        stdscr.refresh()

        time.sleep(0.01)
        loop = not inputs.exit
    close()


if __name__ == '__main__':
    curses.wrapper(main)
