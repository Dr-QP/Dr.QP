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

from models import HexapodLeg
import numpy as np
from point import Point3D
from transforms import Transform


class GaitGenerator:
    def __init__(self):
        self.all_legs = [
            HexapodLeg.left_front,
            HexapodLeg.left_middle,
            HexapodLeg.left_back,
            HexapodLeg.right_front,
            HexapodLeg.right_middle,
            HexapodLeg.right_back,
        ]

    @abstractmethod
    def get_offsets_at_phase(self, phase) -> dict[str, Point3D]:
        return {leg: self.get_offsets_at_phase_for_leg(leg, phase) for leg in self.all_legs}

    @abstractmethod
    def get_offsets_at_phase_for_leg(self, leg, phase) -> Point3D:
        pass

    def _legend_for_leg(self, leg) -> str:
        return leg.name.replace('_', ' ').title()

    def _line_style_for_leg(self, leg) -> str:
        if leg.name.startswith('left'):
            return '-'
        return '--'

    def _color_for_leg(self, leg) -> str:
        if leg in [HexapodLeg.left_front, HexapodLeg.right_front]:
            return 'C2'
        elif leg in [HexapodLeg.left_middle, HexapodLeg.right_middle]:
            return 'C0'
        elif leg in [HexapodLeg.left_back, HexapodLeg.right_back]:
            return 'C3'
        raise ValueError(f'Unknown leg: {leg}')

    def visualize_continuous(self, _steps=100, _subtitle='', _num_phases=1, **gen_args):
        """Visualize the gait sequence as a continuous function."""
        import matplotlib.pyplot as plt

        phases = np.linspace(0, _num_phases, _steps, endpoint=True)

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
        fig, axs = plt.subplots(4, 1)
        fig.set_figheight(12, forward=True)
        # Adjust spacing between subplots to avoid title overlapping with ticks
        plt.subplots_adjust(hspace=0.8)  # Increased from 0.5 to 0.8 for more space between subplots

        # print out **gen_args into a string
        if len(gen_args) == 0:
            subtitle = _subtitle
        else:
            subtitle = _subtitle + '\n' + ', '.join([f'{k}={v}' for k, v in gen_args.items()])

        ax_index = 0
        ax = axs[ax_index]
        ax.set_title('Gait sequence' + subtitle)
        ax.set_ylabel('')
        ax.set_xlabel('phase')

        top_line_y = 6
        ax.set_ylim(-1, top_line_y + 1)
        leg_line = np.linspace(0, top_line_y, 6)
        for i, leg in enumerate(self.all_legs):
            y = leg_line[i]
            line_y = np.full_like(phases, y)
            ax.text(
                0.0,
                y + 0.2,
                self._legend_for_leg(leg),
                ha='left',
                va='bottom',
                color=self._color_for_leg(leg),
            )
            # Plot the base line for each leg
            ax.plot(
                phases,
                line_y,
                color=self._color_for_leg(leg),
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )

            # Plot black lines for swing phase
            is_swing = np.array(z_values[leg]) > 0
            # Create masked arrays for swing phases
            masked_phases = np.ma.masked_array(phases, mask=~is_swing)
            masked_y = np.ma.masked_array(line_y, mask=~is_swing)

            # Plot black lines for swing phases
            ax.plot(
                masked_phases,
                masked_y,
                color=self._color_for_leg(leg),
                label=self._legend_for_leg(leg),
                linestyle='-',
                linewidth=8,
                solid_capstyle='butt',  # The line ends exactly at the data point without any extension
            )

        # Plot x offsets
        ax_index += 1
        ax = axs[ax_index]
        ax.set_title('X offsets (forward/backward)' + subtitle)
        ax.set_ylabel('meters')
        ax.set_xlabel('phase')
        for leg in self.all_legs:
            ax.plot(
                phases,
                x_values[leg],
                color=self._color_for_leg(leg),
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        ax.legend()

        # Plot y offsets
        ax_index += 1
        ax = axs[ax_index]
        ax.set_title('Y offsets (left/right)' + subtitle)
        ax.set_ylabel('meters')
        ax.set_xlabel('phase')
        for leg in self.all_legs:
            ax.plot(
                phases,
                y_values[leg],
                color=self._color_for_leg(leg),
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        ax.legend()

        ax_index += 1
        ax = axs[ax_index]
        ax.set_title('Z offsets (up/down)' + subtitle)
        ax.set_ylabel('meters')
        ax.set_xlabel('phase')
        # Plot z offsets
        for leg in self.all_legs:
            ax.plot(
                phases,
                z_values[leg],
                color=self._color_for_leg(leg),
                label=self._legend_for_leg(leg),
                linestyle=self._line_style_for_leg(leg),
            )
        ax.legend()

        plt.tight_layout()
        plt.show()

    def visualize_continuous_in_3d(
        self,
        _steps=100,
        phase_start=0,
        phase_end=1,
        leg_centers=None,
        ax=None,
        plot_lines=None,
        step_length=50,
        rotation_gaits=False,
        **gen_args,
    ):
        """Visualize the gait sequence as a continuous function in 3D plot."""
        import matplotlib.pyplot as plt

        phases = np.linspace(phase_start, phase_end, _steps, endpoint=True)

        if leg_centers is None:
            base_offset = step_length * 1.3
            side_offset = base_offset * 1.6
            leg_centers = {
                HexapodLeg.left_middle: Point3D([0.0, side_offset, 0.0]),
                HexapodLeg.left_front: Point3D([base_offset, base_offset, 0.0]),
                HexapodLeg.left_back: Point3D([-base_offset, base_offset, 0.0]),
                HexapodLeg.right_front: Point3D([base_offset, -base_offset, 0.0]),
                HexapodLeg.right_middle: Point3D([0.0, -side_offset, 0.0]),
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
                leg_tip = leg_centers[leg]
                if rotation_gaits:
                    rotation_transform = Transform.from_rotvec([0, 0, offset.x], degrees=True)
                    leg_tip = rotation_transform.apply_point(leg_tip) + Point3D([0, 0, offset.z])
                else:
                    leg_tip = leg_tip + offset

                x_values[leg].append(leg_tip.x)
                y_values[leg].append(leg_tip.y)
                z_values[leg].append(leg_tip.z)

        # print out **gen_args into a string
        if len(gen_args) == 0:
            subtitle = ''
        else:
            subtitle = '\n' + ', '.join([f'{k}={v}' for k, v in gen_args.items()])

        own_fig = ax is None

        # Plot the data
        if ax is None:
            fig = plt.figure()
            fig.set_figheight(7, forward=True)
            ax = fig.add_subplot(111, projection='3d')

            # Adjust view angle for better visibility of all legs
            ax.view_init(elev=30, azim=-135)
            # ax.view_init(elev=24, azim=24)

            ax.set_title('3D offsets' + subtitle)
            ax.set_xlabel('X (forward/backward)')
            ax.set_ylabel('Y (left/right)')
            ax.set_zlabel('Z (up/down)')

            # Set axis limits to ensure all legs are visible
            # Calculate the maximum extent in each dimension
            max_x = max([max(x_values[leg]) for leg in self.all_legs])
            min_x = min([min(x_values[leg]) for leg in self.all_legs])
            max_y = max([max(y_values[leg]) for leg in self.all_legs])
            min_y = min([min(y_values[leg]) for leg in self.all_legs])
            max_z = max([max(z_values[leg]) for leg in self.all_legs])
            min_z = min([min(z_values[leg]) for leg in self.all_legs])

            # Add some padding
            padding = max(max_x - min_x, max_y - min_y, max_z - min_z) * 0.2
            ax.set_xlim(min_x - padding, max_x + padding)
            ax.set_ylim(min_y - padding, max_y + padding)
            ax.set_zlim(min_z, max_z + padding)
            ax.set_aspect('equal')

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
                    color=self._color_for_leg(leg),
                    label=self._legend_for_leg(leg),
                    linestyle=self._line_style_for_leg(leg),
                )
                if not plot_lines:
                    plot_lines = {}
                plot_lines[leg] = lines[0]

        if own_fig:
            ax.legend()
            plt.tight_layout()
            plt.show()
        return ax, plot_lines
