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
from point import Point3D
import pygame


class Controls:
    def __init__(self):
        self.direction = Point3D([0.0, 0.0, 0.0])

        self.walk_speed = 0
        self.rotation_speed = 0


pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()

joystick = None
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)  # Create a joystick object for the first joystick
    joystick.init()  # Initialize the joystick
    print('Joystick connected:', joystick.get_name())
    print('Number of axes:', joystick.get_numaxes())
    print('Number of buttons:', joystick.get_numbuttons())

else:
    print('No joystick connected')


inputs = Controls()


def map_float(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def get_controls():
    if joystick is None:
        return inputs

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return inputs

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 1:
                inputs.direction.x = -event.value
            elif event.axis == 0:
                inputs.direction.y = -event.value
            elif event.axis == 2:
                inputs.rotation_speed = -event.value
            elif event.axis == 4:
                interp = np.interp(event.value, [-1, 1], [0, 1])
                mapped = map_float(event.value, -1, 1, 0, 1)
                inputs.direction.z = interp
                print(f'{event.value=}, {interp=}, {mapped=} {inputs.direction.z=}')

            inputs.walk_speed = (
                abs(inputs.direction.x) + abs(inputs.direction.y) + abs(inputs.direction.z)
            )
    return inputs


def close():
    if joystick_count > 0:
        joystick.quit()
