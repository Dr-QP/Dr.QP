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

import os
from typing import List, Literal

from inline_labels_plotly import add_inline_labels, AngleAnnotation
from IPython.display import display
from models import HexapodModel
import numpy as np
import plotly.graph_objects as go
from point import Leg3D, Line, Line3D, Point

# Define types for label options
link_labels_type = Literal['inline', 'legend', 'label', 'none']
joint_labels_type = Literal['annotated', 'points', 'none']


class LegPlotData:
    """Store plot data for a leg."""

    def __init__(self):
        self.lines = []
        self.joints = []
        self.joint_labels = []
        self.link_labels = []


class HexapodPlotData:
    """Store plot data for a hexapod."""

    def __init__(self):
        self.leg_plot_data = []
        self.head_line = None
        self.leg_tips = {}
        self.leg_tip_traces = {}
        self.feet_trails_frames = 0


def plot_leg3d(
    model: Leg3D,
    title: str,
    link_labels: Literal['legend', 'label', 'none'] = 'legend',
    joint_labels: Literal['points', 'none'] = 'points',
    fig=None,
):
    """Plot a 3D leg model using Plotly."""
    if fig is None:
        fig = go.Figure()
        fig.update_layout(
            title=title,
            scene={
                'aspectmode': 'data',
                'xaxis': {
                    'showticklabels': False,
                    'showgrid': False,
                    'zeroline': False,
                    'showline': False,
                },
                'yaxis': {
                    'showticklabels': False,
                    'showgrid': False,
                    'zeroline': False,
                    'showline': False,
                },
                'zaxis': {
                    'showticklabels': False,
                    'showgrid': False,
                    'zeroline': False,
                    'showline': False,
                },
            },
        )

    plot_data = plot_leg_links(fig, model.lines, link_labels=link_labels, joint_labels=joint_labels)

    return fig, plot_data


def plot_leg_links(
    fig: go.Figure,
    model: List[Line] | List[Line3D],
    link_labels: link_labels_type,
    joint_labels: joint_labels_type,
):
    """Plot leg links in 3D space using Plotly."""
    link_colors = ['cyan', 'red', 'green', 'blue', 'magenta']
    joint_colors = link_colors[1:]

    plot_data = LegPlotData()

    # Plot lines
    for i, (line, color) in enumerate(zip(model, link_colors)):
        label = line.label if link_labels != 'none' else None

        # Get start and end points
        if isinstance(line, Line3D):
            x_vals = [line.start.x, line.end.x]
            y_vals = [line.start.y, line.end.y]
            z_vals = [line.start.z, line.end.z]
        else:  # Line (2D)
            x_vals = [line.start.x, line.end.x]
            y_vals = [line.start.y, line.end.y]
            z_vals = [0, 0]

        # Create line trace
        line_trace = go.Scatter3d(
            x=x_vals,
            y=y_vals,
            z=z_vals,
            mode='lines',
            line={'color': color, 'width': 4},
            name=label if label else f'Line {i}',
            showlegend=(link_labels == 'legend'),
        )

        fig.add_trace(line_trace)
        plot_data.lines.append(line_trace)

    # Plot joints
    if joint_labels == 'points':
        for i, (line, joint_color) in enumerate(zip(model, joint_colors)):
            if isinstance(line, Line3D):
                joint_trace = go.Scatter3d(
                    x=[line.end.x],
                    y=[line.end.y],
                    z=[line.end.z],
                    mode='markers',
                    marker={'color': joint_color, 'size': 6},
                    name=f'Joint {i}',
                    showlegend=False,
                )
            else:  # Line (2D)
                joint_trace = go.Scatter3d(
                    x=[line.end.x],
                    y=[line.end.y],
                    z=[0],
                    mode='markers',
                    marker={'color': joint_color, 'size': 6},
                    name=f'Joint {i}',
                    showlegend=False,
                )

            fig.add_trace(joint_trace)
            plot_data.joints.append(joint_trace)

    return plot_data


def leg_tips_to_segments(leg_tips):
    """Convert leg tips to line segments for plotting trails."""
    points = np.array(leg_tips)
    if len(points) < 2:
        return points

    return points


def plot_hexapod(hexapod: HexapodModel, targets=None, feet_trails_frames=0):
    """Plot a hexapod model in 3D using Plotly."""
    fig = go.Figure()
    fig.update_layout(
        title='Hexapod in 3D',
        scene={
            'aspectmode': 'data',
            'xaxis': {
                'showticklabels': False,
                'showgrid': False,
                'zeroline': False,
                'showline': False,
            },
            'yaxis': {
                'showticklabels': False,
                'showgrid': False,
                'zeroline': False,
                'showline': False,
            },
            'zaxis': {
                'showticklabels': False,
                'showgrid': False,
                'zeroline': False,
                'showline': False,
            },
        },
        scene_camera={
            'eye': {'x': 1.5, 'y': 1.5, 'z': 0.8},
            'up': {'x': 0, 'y': 0, 'z': 1},
        },
    )

    plot_data = HexapodPlotData()
    plot_data.feet_trails_frames = feet_trails_frames

    # Plot each leg
    for leg in hexapod.legs:
        _, leg_plot_data = plot_leg3d(
            leg,
            'Hexapod in 3D',
            link_labels='none',
            joint_labels='points',
            fig=fig,
        )
        plot_data.leg_plot_data.append(leg_plot_data)

        # Add feet trails if requested
        if feet_trails_frames > 0:
            plot_data.leg_tips[leg.label] = [leg.tibia_end.numpy()]

            # Create empty trace for leg tip trails
            leg_tip_trace = go.Scatter3d(
                x=[],
                y=[],
                z=[],
                mode='lines',
                line={'color': 'orange', 'width': 2},
                name=f'{leg.label} trail',
                showlegend=False,
            )

            fig.add_trace(leg_tip_trace)
            plot_data.leg_tip_traces[leg.label] = leg_tip_trace

    # Plot head line
    head_trace = go.Scatter3d(
        x=[hexapod.head.start.x, hexapod.head.end.x],
        y=[hexapod.head.start.y, hexapod.head.end.y],
        z=[hexapod.head.start.z, hexapod.head.end.z],
        mode='lines',
        line={'color': 'cyan', 'width': 4},
        name='Head',
        showlegend=False,
    )

    fig.add_trace(head_trace)
    plot_data.head_line = head_trace

    # Plot targets if provided
    if targets:
        target_x = [target.x for target in targets]
        target_y = [target.y for target in targets]
        target_z = [target.z for target in targets]

        target_trace = go.Scatter3d(
            x=target_x,
            y=target_y,
            z=target_z,
            mode='markers',
            marker={'color': 'black', 'size': 6},
            name='Targets',
        )

        fig.add_trace(target_trace)

    # Set initial camera view
    fig.update_layout(
        scene_camera={
            'eye': {'x': 1.5, 'y': 1.5, 'z': 0.8},
            'up': {'x': 0, 'y': 0, 'z': 1},
        }
    )

    return fig, plot_data


def update_hexapod_plot(hexapod: HexapodModel, plot_data: HexapodPlotData, _=None):
    """Update the hexapod plot with new positions."""
    # Update leg positions
    for leg, leg_plot_data in zip(hexapod.legs, plot_data.leg_plot_data):
        update_leg3d_lines(leg, leg_plot_data)

    # Update head line
    if plot_data.head_line:
        plot_data.head_line.x = [hexapod.head.start.x, hexapod.head.end.x]
        plot_data.head_line.y = [hexapod.head.start.y, hexapod.head.end.y]
        plot_data.head_line.z = [hexapod.head.start.z, hexapod.head.end.z]

    # Update feet trails
    if plot_data.feet_trails_frames > 0:
        for leg in hexapod.legs:
            # Add new position to trail
            plot_data.leg_tips[leg.label].append(leg.tibia_end.numpy())

            # Remove oldest position if exceeding max frames
            if len(plot_data.leg_tips[leg.label]) > plot_data.feet_trails_frames:
                plot_data.leg_tips[leg.label].pop(0)

            # Update trail trace
            points = np.array(plot_data.leg_tips[leg.label])
            plot_data.leg_tip_traces[leg.label].x = points[:, 0]
            plot_data.leg_tip_traces[leg.label].y = points[:, 1]
            plot_data.leg_tip_traces[leg.label].z = points[:, 2]


def update_leg3d_lines(leg: Leg3D, leg_plot_data: LegPlotData):
    """Update the 3D leg lines with new positions."""
    for line_trace, line_model in zip(leg_plot_data.lines, leg.lines):
        line_trace.x = [line_model.start.x, line_model.end.x]
        line_trace.y = [line_model.start.y, line_model.end.y]
        line_trace.z = [line_model.start.z, line_model.end.z]

    for joint_trace, line_model in zip(leg_plot_data.joints, leg.lines):
        joint_trace.x = [line_model.end.x]
        joint_trace.y = [line_model.end.y]
        joint_trace.z = [line_model.end.z]


def is_sphinx_build():
    """Check if code is running in Sphinx build environment."""
    return os.getenv('SPHINX_BUILD') == '1'


class LegPlotData:
    """Store plot data for a leg."""

    def __init__(self):
        self.lines = []
        self.joints = []
        self.joint_labels = []
        self.link_labels = []


def plot_leg_with_points(
    model: list[Line],
    title: str,
    link_labels: link_labels_type = 'inline',
    joint_labels: joint_labels_type = 'annotated',
    no_cartesian_ticks=False,
    x_label='X',
    y_label='Y',
):
    """Plot a leg model with points using Plotly."""
    fig = go.Figure()
    fig.update_layout(title=title)

    plot_data = plot_leg_links(fig, model, link_labels=link_labels, joint_labels=joint_labels)

    # Select length of axes and the space between tick labels
    x = np.array([[line.start.x, line.end.x] for line in model])
    y = np.array([[line.start.y, line.end.y] for line in model])
    plot_min = Point(np.min(x), np.min(y))
    plot_max = Point(np.max(x), np.max(y))

    def extra_space(v):
        return max(2, abs(v) * 0.2)

    plot_min.x -= extra_space(plot_min.x)
    plot_min.y -= extra_space(plot_min.y)

    plot_max.x += extra_space(plot_max.x)
    plot_max.y += extra_space(plot_max.y)

    plot_cartesian_plane(
        fig,
        plot_min,
        plot_max,
        ticks_frequency=5,
        no_ticks=no_cartesian_ticks,
        x_label=x_label,
        y_label=y_label,
    )

    return fig, None, plot_data


def plot_leg_links(
    fig: go.Figure,
    model: list[Line] | list[Line3D],
    link_labels: link_labels_type,
    joint_labels: joint_labels_type,
):
    """Plot leg links in 2D space using Plotly."""
    link_colors = ['cyan', 'red', 'green', 'blue', 'magenta']
    joint_colors = link_colors[1:]

    plot_data = LegPlotData()

    # Plot lines
    for i, (line, color) in enumerate(zip(model, link_colors)):
        label = line.label if link_labels != 'none' else None

        # Plot main line
        line_trace = go.Scatter(
            x=[line.start.x, line.end.x],
            y=[line.start.y, line.end.y],
            mode='lines',
            line={'color': color, 'width': 2},
            name=label if label else f'Line {i}',
            showlegend=(link_labels == 'legend'),
        )
        fig.add_trace(line_trace)
        plot_data.lines.append(line_trace)

        # Plot extended line for joint angles if needed
        if joint_labels == 'annotated' and i < len(model) - 1:
            extended_line = line.extended(3)
            extended_trace = go.Scatter(
                x=[line.end.x, extended_line.end.x],
                y=[line.end.y, extended_line.end.y],
                mode='lines',
                line={'color': color, 'width': 1, 'dash': 'dot'},
                showlegend=False,
            )
            fig.add_trace(extended_trace)
            plot_data.lines.append(extended_trace)

    # Plot joints
    if joint_labels == 'annotated' or joint_labels == 'points':
        for i, (line, joint_color) in enumerate(zip(model, joint_colors)):
            joint_trace = go.Scatter(
                x=[line.end.x],
                y=[line.end.y],
                mode='markers',
                marker={'color': joint_color, 'size': 8},
                name=f'Joint {i}',
                showlegend=False,
            )
            fig.add_trace(joint_trace)
            plot_data.joints.append(joint_trace)

    # Add inline labels for leg links
    if link_labels == 'legend':
        pass  # Already handled with showlegend=True
    elif link_labels == 'inline':
        add_inline_labels(fig, with_overall_progress=False, fontsize='medium')

    # Add angle annotations
    if joint_labels == 'annotated':
        for i in range(len(model) - 1):
            line = model[i]
            next_line = model[i + 1]
            joint_color = joint_colors[i]

            # Sort the vectors by y coordinate to always display angle on the correct side
            vecs = [
                next_line.end.numpy(),
                line.extended().end.numpy(),
            ]
            vecs.sort(key=lambda v: v[1])

            if line.end.label:
                AngleAnnotation(
                    line.end.numpy(),
                    *vecs,
                    fig=fig,
                    size=50,
                    text=line.end.label,
                    color=joint_color,
                    linestyle='dash',
                    textposition='outside',
                    text_kw={'font_size': 10, 'font_color': joint_color},
                )

    return plot_data


def plot_cartesian_plane(
    fig: go.Figure,
    plot_min: Point,
    plot_max: Point,
    ticks_frequency=1,
    no_ticks=False,
    x_label='X',
    y_label='Y',
):
    """Plot a Cartesian coordinate system using Plotly."""
    # Set axis limits
    fig.update_layout(
        xaxis={
            'range': [plot_min.x, plot_max.x],
            'zeroline': True,
            'zerolinewidth': 2,
            'zerolinecolor': 'black',
            'showgrid': True,
            'gridwidth': 1,
            'gridcolor': 'lightgray',
        },
        yaxis={
            'range': [plot_min.y, plot_max.y],
            'zeroline': True,
            'zerolinewidth': 2,
            'zerolinecolor': 'black',
            'showgrid': True,
            'gridwidth': 1,
            'gridcolor': 'lightgray',
            'scaleanchor': 'x',
            'scaleratio': 1,
        },
    )

    # Set axis labels
    fig.update_layout(
        xaxis_title=x_label,
        yaxis_title=y_label,
    )

    # Hide ticks if requested
    if no_ticks:
        fig.update_layout(
            xaxis={'showticklabels': False},
            yaxis={'showticklabels': False},
        )
    else:
        # Create custom ticks
        x_ticks = np.arange(plot_min.x, plot_max.x, ticks_frequency, dtype=int)
        y_ticks = np.arange(plot_min.y, plot_max.y, ticks_frequency, dtype=int)

        fig.update_layout(
            xaxis={'tickvals': x_ticks},
            yaxis={'tickvals': y_ticks},
        )

    # Add arrows at the end of axes
    fig.add_annotation(
        x=plot_max.x,
        y=0,
        ax=plot_max.x - 0.5,
        ay=0,
        xref='x',
        yref='y',
        axref='x',
        ayref='y',
        showarrow=True,
        arrowhead=3,
        arrowsize=1.5,
        arrowwidth=2,
        arrowcolor='black',
    )

    fig.add_annotation(
        x=0,
        y=plot_max.y,
        ax=0,
        ay=plot_max.y - 0.5,
        xref='x',
        yref='y',
        axref='x',
        ayref='y',
        showarrow=True,
        arrowhead=3,
        arrowsize=1.5,
        arrowwidth=2,
        arrowcolor='black',
    )


def plot_leg_update_lines(model, plot_data: LegPlotData):
    """Update the leg lines with new positions."""
    line_index = 0
    for i, line_model in enumerate(model):
        # Update main line
        plot_data.lines[line_index].x = [line_model.start.x, line_model.end.x]
        plot_data.lines[line_index].y = [line_model.start.y, line_model.end.y]
        line_index += 1

        # Skip extended line for the last segment
        if i < len(model) - 1:
            extended = line_model.extended(3)
            plot_data.lines[line_index].x = [line_model.end.x, extended.end.x]
            plot_data.lines[line_index].y = [line_model.end.y, extended.end.y]
            line_index += 1

    for i, (joint, line_model) in enumerate(zip(plot_data.joints, model)):
        joint.x = [line_model.end.x]
        joint.y = [line_model.end.y]


def plot_ik_lines(fig, femur, tibia):
    """Plot IK lines for visualization using Plotly."""
    d_end = Point(femur.start.x, tibia.end.y)

    # Plot L line
    fig.add_trace(
        go.Scatter(
            x=[femur.start.x, tibia.end.x],
            y=[femur.start.y, tibia.end.y],
            mode='lines',
            line={'color': 'magenta', 'width': 2, 'dash': 'dash'},
            name='L',
        )
    )

    # Plot D line
    fig.add_trace(
        go.Scatter(
            x=[femur.start.x, d_end.x],
            y=[femur.start.y, d_end.y],
            mode='lines',
            line={'color': 'magenta', 'width': 2, 'dash': 'dash'},
            name='D',
        )
    )

    # Plot T line
    fig.add_trace(
        go.Scatter(
            x=[tibia.end.x, d_end.x],
            y=[tibia.end.y, d_end.y],
            mode='lines',
            line={'color': 'magenta', 'width': 2, 'dash': 'dash'},
            name='T',
        )
    )

    # Add angle annotations
    AngleAnnotation(
        femur.start.numpy(),
        tibia.end.numpy(),
        femur.end.numpy(),
        fig=fig,
        size=50,
        text=r'$\theta$1',
        color='magenta',
        linestyle='dash',
        textposition='outside',
        text_kw={'font_size': 10, 'font_color': 'magenta'},
    )

    AngleAnnotation(
        femur.start.numpy(),
        d_end.numpy(),
        tibia.end.numpy(),
        fig=fig,
        size=50,
        text=r'$\theta$2',
        color='magenta',
        linestyle='dash',
        textposition='outside',
        text_kw={'font_size': 10, 'font_color': 'magenta'},
    )

    AngleAnnotation(
        femur.end.numpy(),
        femur.start.numpy(),
        tibia.end.numpy(),
        fig=fig,
        size=50,
        text=r'$\Phi$',
        color='magenta',
        linestyle='dash',
        textposition='outside',
        text_kw={'font_size': 10, 'font_color': 'magenta'},
    )


def animate_plot(
    _fig,
    _update_func,
    _frames,
    _interval=16,  # 60 fps
    _interactive=False,
    **interact_kwargs,
):
    """Create an interactive animation using Plotly."""
    import ipywidgets as widgets

    if _interactive:
        # Create slider for interactive control
        slider = widgets.IntSlider(
            min=0,
            max=_frames - 1,
            step=1,
            value=0,
            description='Frame:',
            continuous_update=True,
            layout=widgets.Layout(width='500px'),
        )

        # Create output widget to display the figure
        output = widgets.Output()

        # Define update function for the slider
        def on_slider_change(change):
            with output:
                output.clear_output(wait=True)
                frame = change['new']
                _update_func(frame, **interact_kwargs)
                display(_fig)

        # Connect the slider to the update function
        slider.observe(on_slider_change, names='value')

        # Initial display
        with output:
            _update_func(0, **interact_kwargs)
            display(_fig)

        # Display the slider and output
        display(widgets.VBox([slider, output]))

        return slider
    else:
        # For non-interactive mode, create frames for animation
        frames_list = []

        for i in range(_frames):
            _update_func(i, **interact_kwargs)

            # Create a frame with the current state
            frame = go.Frame(data=_fig.data, name=f'frame_{i}')
            frames_list.append(frame)

        # Add frames to the figure
        _fig.frames = frames_list

        # Add animation controls
        _fig.update_layout(
            updatemenus=[
                {
                    'type': 'buttons',
                    'buttons': [
                        {
                            'label': 'Play',
                            'method': 'animate',
                            'args': [
                                None,
                                {
                                    'frame': {'duration': _interval, 'redraw': True},
                                    'fromcurrent': True,
                                },
                            ],
                        },
                        {
                            'label': 'Pause',
                            'method': 'animate',
                            'args': [
                                [None],
                                {'frame': {'duration': 0, 'redraw': True}, 'mode': 'immediate'},
                            ],
                        },
                    ],
                    'direction': 'left',
                    'pad': {'r': 10, 't': 10},
                    'showactive': False,
                    'x': 0.1,
                    'y': 0,
                    'xanchor': 'right',
                    'yanchor': 'top',
                }
            ],
            sliders=[
                {
                    'active': 0,
                    'yanchor': 'top',
                    'xanchor': 'left',
                    'currentvalue': {'prefix': 'Frame: ', 'visible': True, 'xanchor': 'right'},
                    'pad': {'b': 10, 't': 50},
                    'len': 0.9,
                    'x': 0.1,
                    'y': 0,
                    'steps': [
                        {
                            'args': [
                                [f'frame_{i}'],
                                {
                                    'frame': {'duration': _interval, 'redraw': True},
                                    'mode': 'immediate',
                                },
                            ],
                            'label': str(i),
                            'method': 'animate',
                        }
                        for i in range(_frames)
                    ],
                }
            ],
        )

        display(_fig)
        return _fig
