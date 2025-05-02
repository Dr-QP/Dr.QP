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

from IPython.display import display
from models import HexapodLeg
import numpy as np
import plotly.graph_objects as go
import plotly.subplots as sp
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
            return 'solid'
        return 'dash'

    def _color_for_leg(self, leg) -> str:
        if leg in [HexapodLeg.left_front, HexapodLeg.right_front]:
            return 'green'
        elif leg in [HexapodLeg.left_middle, HexapodLeg.right_middle]:
            return 'blue'
        elif leg in [HexapodLeg.left_back, HexapodLeg.right_back]:
            return 'orange'
        raise ValueError(f'Unknown leg: {leg}')

    def visualize_continuous(self, _steps=100, _subtitle='', _num_phases=1, **gen_args):
        """Visualize the gait sequence as a continuous function using Plotly."""
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

        # Create subplot figure with 4 rows
        fig = sp.make_subplots(
            rows=4,
            cols=1,
            subplot_titles=[
                'Gait sequence' + _subtitle,
                'X offsets (forward/backward)' + _subtitle,
                'Y offsets (left/right)' + _subtitle,
                'Z offsets (up/down)' + _subtitle,
            ],
            vertical_spacing=0.1,
        )

        # Plot gait sequence (top plot)
        top_line_y = 6
        leg_line = np.linspace(0, top_line_y, 6)

        for i, leg in enumerate(self.all_legs):
            y = leg_line[i]
            line_y = np.full_like(phases, y)

            # Add text annotation for leg name
            fig.add_annotation(
                x=0.0,
                y=y + 0.2,
                text=self._legend_for_leg(leg),
                showarrow=False,
                xref='x',
                yref='y',
                font={'color': self._color_for_leg(leg)},
                row=1,
                col=1,
            )

            # Plot base line for each leg
            fig.add_trace(
                go.Scatter(
                    x=phases,
                    y=line_y,
                    mode='lines',
                    line={'color': self._color_for_leg(leg), 'dash': self._line_style_for_leg(leg)},
                    name=self._legend_for_leg(leg),
                    showlegend=False,
                ),
                row=1,
                col=1,
            )

            # Plot swing phases (black lines)
            is_swing = np.array(z_values[leg]) > 0
            if any(is_swing):
                # Create segments for swing phases
                segments = []
                segment_start = None

                for j, (phase, swing) in enumerate(zip(phases, is_swing)):
                    if swing and segment_start is None:
                        segment_start = j
                    elif not swing and segment_start is not None:
                        segments.append((segment_start, j - 1))
                        segment_start = None

                # Handle case where last segment extends to the end
                if segment_start is not None:
                    segments.append((segment_start, len(phases) - 1))

                # Plot each swing segment
                for start, end in segments:
                    fig.add_trace(
                        go.Scatter(
                            x=phases[start : end + 1],
                            y=line_y[start : end + 1],
                            mode='lines',
                            line={'color': self._color_for_leg(leg), 'width': 8},
                            showlegend=False,
                        ),
                        row=1,
                        col=1,
                    )

        # Plot X offsets
        for leg in self.all_legs:
            fig.add_trace(
                go.Scatter(
                    x=phases,
                    y=x_values[leg],
                    mode='lines',
                    line={'color': self._color_for_leg(leg), 'dash': self._line_style_for_leg(leg)},
                    name=self._legend_for_leg(leg),
                ),
                row=2,
                col=1,
            )

        # Plot Y offsets
        for leg in self.all_legs:
            fig.add_trace(
                go.Scatter(
                    x=phases,
                    y=y_values[leg],
                    mode='lines',
                    line={'color': self._color_for_leg(leg), 'dash': self._line_style_for_leg(leg)},
                    name=self._legend_for_leg(leg),
                    showlegend=False,
                ),
                row=3,
                col=1,
            )

        # Plot Z offsets
        for leg in self.all_legs:
            fig.add_trace(
                go.Scatter(
                    x=phases,
                    y=z_values[leg],
                    mode='lines',
                    line={'color': self._color_for_leg(leg), 'dash': self._line_style_for_leg(leg)},
                    name=self._legend_for_leg(leg),
                    showlegend=False,
                ),
                row=4,
                col=1,
            )

        # Update layout
        fig.update_layout(
            height=800,
            width=1000,
            legend={'orientation': 'h', 'yanchor': 'bottom', 'y': 1.02, 'xanchor': 'right', 'x': 1},
        )

        # Update y-axis ranges
        fig.update_yaxes(range=[-1, top_line_y + 1], row=1, col=1)

        # Update axis labels
        fig.update_xaxes(title_text='phase', row=4, col=1)
        fig.update_yaxes(title_text='meters', row=2, col=1)
        fig.update_yaxes(title_text='meters', row=3, col=1)
        fig.update_yaxes(title_text='meters', row=4, col=1)

        display(fig)
        return fig

    def visualize_continuous_in_3d(
        self,
        _steps=100,
        phase_start=0,
        phase_end=1,
        leg_centers=None,
        fig=None,
        plot_lines=None,
        step_length=50,
        rotation_gaits=False,
        **gen_args,
    ):
        """Visualize the gait sequence as a continuous function in 3D plot using Plotly."""
        phases = np.linspace(phase_start, phase_end, _steps, endpoint=True)

        if leg_centers is None:
            base_offset = step_length / 1.6
            side_offset = base_offset * 1.5
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

        # Format subtitle from gen_args
        if len(gen_args) == 0:
            subtitle = ''
        else:
            subtitle = '<br>' + ', '.join([f'{k}={v}' for k, v in gen_args.items()])

        own_fig = fig is None

        # Create figure if not provided
        if fig is None:
            fig = go.Figure()
            fig.update_layout(
                title='3D offsets' + subtitle,
                scene={
                    'xaxis_title': 'X (forward/backward)',
                    'yaxis_title': 'Y (left/right)',
                    'zaxis_title': 'Z (up/down)',
                    'aspectmode': 'data',
                },
                width=800,
                height=700,
            )

        # Initialize plot_lines dictionary if needed
        if plot_lines is None:
            plot_lines = {}

        # Plot trajectories for each leg
        for leg in self.all_legs:
            if leg in plot_lines and plot_lines[leg] is not None:
                # Update existing trace
                plot_lines[leg].x = x_values[leg]
                plot_lines[leg].y = y_values[leg]
                plot_lines[leg].z = z_values[leg]
            else:
                # Create new trace
                trace = go.Scatter3d(
                    x=x_values[leg],
                    y=y_values[leg],
                    z=z_values[leg],
                    mode='lines',
                    line={
                        'color': self._color_for_leg(leg),
                        'dash': self._line_style_for_leg(leg),
                        'width': 4,
                    },
                    name=self._legend_for_leg(leg),
                )
                fig.add_trace(trace)
                plot_lines[leg] = trace

        # Set camera position for better visibility
        if own_fig:
            fig.update_layout(
                scene_camera={
                    'eye': {'x': 1.5, 'y': 1.5, 'z': 0.8},
                    'up': {'x': 0, 'y': 0, 'z': 1},
                }
            )

            # Calculate axis limits with padding
            max_x = max([max(x_values[leg]) for leg in self.all_legs])
            min_x = min([min(x_values[leg]) for leg in self.all_legs])
            max_y = max([max(y_values[leg]) for leg in self.all_legs])
            min_y = min([min(y_values[leg]) for leg in self.all_legs])
            max_z = max([max(z_values[leg]) for leg in self.all_legs])
            min_z = min([min(z_values[leg]) for leg in self.all_legs])

            padding = max(max_x - min_x, max_y - min_y, max_z - min_z) * 0.2
            fig.update_layout(
                scene={
                    'xaxis': {'range': [min_x - padding, max_x + padding]},
                    'yaxis': {'range': [min_y - padding, max_y + padding]},
                    'zaxis': {'range': [min_z, max_z + padding]},
                }
            )

            display(fig)

        return fig, plot_lines
