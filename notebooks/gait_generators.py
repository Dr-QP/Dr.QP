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

from abc import abstractmethod

import matplotlib.pyplot as plt
import numpy as np

from point import Point3D
from models import HexapodLeg


class GaitGenerator:
    def __init__(
        self,
        all_legs=[
            HexapodLeg.left_front,
            HexapodLeg.left_middle,
            HexapodLeg.left_back,
            HexapodLeg.right_front,
            HexapodLeg.right_middle,
            HexapodLeg.right_back,
        ],
    ):
        self.all_legs = all_legs

    @abstractmethod
    def get_offsets_at_phase(self, phase) -> dict[str, Point3D]:
        return {leg: self.get_offsets_at_phase_for_leg(leg, phase) for leg in self.all_legs}

    @abstractmethod
    def get_offsets_at_phase_for_leg(self, leg, phase) -> Point3D:
        pass

    def _legend_for_leg(self, leg) -> str:
        return leg

    def _line_style_for_leg(self, leg) -> str:
        return '-'

    def visualize_continuous(self, steps=100, **gen_args):
        """Visualize the gait sequence as a continuous function."""
        phases = np.linspace(0, 1, steps, endpoint=True)

        # Create data structures to store values for plotting
        x_values = {leg: [] for leg in self.all_legs}
        y_values = {leg: [] for leg in self.all_legs}
        z_values = {leg: [] for leg in self.all_legs}

        # Generate data points
        for phase in phases:
            offsets = self.get_offsets_at_phase(phase, **gen_args)
            for leg, offset in offsets.items():
                x_values[leg].append(offset.x)
                y_values[leg].append(offset.y)
                z_values[leg].append(offset.z)

        # Plot the data
        fig, axs = plt.subplots(3, 1, figsize=(12, 10))

        # print out **gen_args into a string
        if len(gen_args) == 0:
            subtitle = ''
        else:
            subtitle = '\n' + ', '.join([f'{k}={v}' for k, v in gen_args.items()])

        # Plot x offsets
        for leg in self.all_legs:
            axs[0].plot(
                phases,
                x_values[leg],
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        axs[0].set_title('X offsets (forward/backward)' + subtitle)
        axs[0].set_ylabel('meters')
        axs[0].set_xlabel('phase')
        axs[0].legend()

        # Plot y offsets
        for leg in self.all_legs:
            axs[1].plot(
                phases,
                y_values[leg],
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        axs[1].set_title('Y offsets (left/right)' + subtitle)
        axs[1].set_ylabel('meters')
        axs[1].set_xlabel('phase')
        axs[1].legend()

        # Plot z offsets
        for leg in self.all_legs:
            axs[2].plot(
                phases,
                z_values[leg],
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        axs[2].set_title('Z offsets (up/down)' + subtitle)
        axs[2].set_ylabel('meters')
        axs[2].set_xlabel('phase')
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    def visualize_continuous_in_3d(
        self,
        steps=100,
        phase_start=0,
        phase_end=1,
        leg_centers=None,
        ax=None,
        plot_lines=None,
        step_length=50,
        **gen_args,
    ):
        """Visualize the gait sequence as a continuous function in 3D plot."""
        phases = np.linspace(phase_start, phase_end, steps, endpoint=True)

        if leg_centers is None:
            base_offset = step_length / 1.6
            side_offset = base_offset * 1.5
            leg_centers = {
                HexapodLeg.left_middle: Point3D([0.0, side_offset, 0.0]),
                HexapodLeg.left_front: Point3D([base_offset, base_offset, 0.0]),
                HexapodLeg.left_back: Point3D([-base_offset, base_offset, 0.0]),
                HexapodLeg.right_front: Point3D([base_offset, -base_offset, 0.0]),
                HexapodLeg.right_middle: Point3D([0.0, side_offset, 0.0]),
                HexapodLeg.right_back: Point3D([-base_offset, -base_offset, 0.0]),
            }

        # Create data structures to store values for plotting
        x_values = {leg: [] for leg in self.all_legs}
        y_values = {leg: [] for leg in self.all_legs}
        z_values = {leg: [] for leg in self.all_legs}

        # Generate data points
        for phase in phases:
            offsets = self.get_offsets_at_phase(phase, **gen_args)
            for leg, offset in offsets.items():
                offset = offset + leg_centers[leg]
                x_values[leg].append(offset.x)
                y_values[leg].append(offset.y)
                z_values[leg].append(offset.z)

        # print out **gen_args into a string
        if len(gen_args) == 0:
            subtitle = ''
        else:
            subtitle = '\n' + ', '.join([f'{k}={v}' for k, v in gen_args.items()])

        # Plot the data
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')

            ax.view_init(elev=47, azim=-160)
            # ax.view_init(elev=24, azim=24)

            ax.set_title('3D offsets' + subtitle)
            ax.set_xlabel('X (forward/backward)')
            ax.set_ylabel('Y (left/right)')
            ax.set_zlabel('Z (up/down)')

        # Plot x offsets
        for leg in self.all_legs:
            if plot_lines and leg in plot_lines and plot_lines[leg]:
                lines = plot_lines[leg]
                lines.set_data_3d(x_values[leg], y_values[leg], z_values[leg])
            else:
                lines = ax.plot(
                    x_values[leg],
                    y_values[leg],
                    z_values[leg],
                    label=self._legend_for_leg(leg),
                    linestyle=self._line_style_for_leg(leg),
                )
                if not plot_lines:
                    plot_lines = {}
                plot_lines[leg] = lines[0]

        return ax, plot_lines
