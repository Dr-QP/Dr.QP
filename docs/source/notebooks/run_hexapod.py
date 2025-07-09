#!/usr/bin/env python3
#
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

import argparse

from drqp_brain.models import HexapodModel
from drqp_brain.walk_controller import GaitType, WalkController
from matplotlib import pyplot as plt
from plotting import plot_hexapod, update_hexapod_plot
import ps5_controller


def main():
    drqp_coxa = 0.053  # in meters
    drqp_femur = 0.066225  # in meters
    drqp_tibia = 0.123  # in meters

    drqp_front_offset = 0.116924  # x offset for the front and back legs in meters
    drqp_side_offset = 0.063871  # y offset fo the front and back legs
    drqp_middle_offset = 0.103  # x offset for the middle legs

    hexapod = HexapodModel(
        coxa_len=drqp_coxa,
        femur_len=drqp_femur,
        tibia_len=drqp_tibia,
        front_offset=drqp_front_offset,
        middle_offset=drqp_middle_offset,
        side_offset=drqp_side_offset,
        leg_rotation=[0, 0, 45],
    )

    hexapod.forward_kinematics(0, -25, 110)

    walker = WalkController(
        hexapod.legs,
        step_length=0.13,  # in meters
        step_height=0.06,  # in meters
        rotation_speed_degrees=25,
        gait=GaitType.ripple,
    )

    fig, ax, plot_data = plot_hexapod(hexapod, feet_trails_frames=30)

    def process_frame(delta_time=0.001):
        update_hexapod_plot(hexapod, plot_data)

        plt.show(block=False)
        plt.pause(delta_time)

    while plt.get_fignums():  # window(s) open
        inputs = ps5_controller.get_controls()
        walker.next_step(
            stride_direction=inputs.direction,
            stride_ratio=inputs.walk_speed,
            rotation_ratio=inputs.rotation_speed,
        )

        process_frame(0.001)


if __name__ == '__main__':
    try:
        main()
        parser = argparse.ArgumentParser('Dr.QP Robot controller')
        args = parser.parse_args()
        main(**vars(args))
    except KeyboardInterrupt:
        pass  # codeql[py/empty-except]
    except RuntimeError as e:
        print(e)
    finally:
        ps5_controller.close()
