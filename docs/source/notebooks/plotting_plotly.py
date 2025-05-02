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
from pathlib import Path
from typing import Dict, List, Literal, Optional, Tuple, Union

import numpy as np
import plotly.graph_objects as go
from IPython.display import display
from models import HexapodModel
from point import Leg3D, Line, Line3D, Point, Point3D

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
            scene=dict(
                aspectmode='data',
                xaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
                yaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
                zaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
            ),
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
            line=dict(color=color, width=4),
            name=label if label else f"Line {i}",
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
                    marker=dict(color=joint_color, size=6),
                    name=f"Joint {i}",
                    showlegend=False,
                )
            else:  # Line (2D)
                joint_trace = go.Scatter3d(
                    x=[line.end.x],
                    y=[line.end.y],
                    z=[0],
                    mode='markers',
                    marker=dict(color=joint_color, size=6),
                    name=f"Joint {i}",
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
        scene=dict(
            aspectmode='data',
            xaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
            yaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
            zaxis=dict(showticklabels=False, showgrid=False, zeroline=False, showline=False),
        ),
        scene_camera=dict(
            eye=dict(x=1.5, y=1.5, z=0.8),
            up=dict(x=0, y=0, z=1),
        ),
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
                line=dict(color='orange', width=2),
                name=f"{leg.label} trail",
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
        line=dict(color='cyan', width=4),
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
            marker=dict(color='black', size=6),
            name='Targets',
        )
        
        fig.add_trace(target_trace)

    # Set initial camera view
    fig.update_layout(
        scene_camera=dict(
            eye=dict(x=1.5, y=1.5, z=0.8),
            up=dict(x=0, y=0, z=1),
        )
    )

    return fig, plot_data


def update_hexapod_plot(hexapod: HexapodModel, plot_data: HexapodPlotData, fig=None):
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


def animate_plot(
    fig,
    update_func,
    frames,
    interval=16,  # 60 fps
    interactive=False,
    **interact_kwargs
):
    """Create an interactive animation using Plotly."""
    import ipywidgets as widgets
    from IPython.display import display
    
    if interactive:
        # Create slider for interactive control
        slider = widgets.IntSlider(
            min=0,
            max=frames-1,
            step=1,
            value=0,
            description='Frame:',
            continuous_update=True,
            layout=widgets.Layout(width='500px')
        )
        
        # Create output widget to display the figure
        output = widgets.Output()
        
        # Define update function for the slider
        def on_slider_change(change):
            with output:
                output.clear_output(wait=True)
                frame = change['new']
                update_func(frame, **interact_kwargs)
                display(fig)
        
        # Connect the slider to the update function
        slider.observe(on_slider_change, names='value')
        
        # Initial display
        with output:
            update_func(0, **interact_kwargs)
            display(fig)
        
        # Display the slider and output
        display(widgets.VBox([slider, output]))
        
        return slider
    else:
        # For non-interactive mode, create frames for animation
        frames_list = []
        
        for i in range(frames):
            update_func(i, **interact_kwargs)
            
            # Create a frame with the current state
            frame = go.Frame(
                data=fig.data,
                name=f'frame_{i}'
            )
            frames_list.append(frame)
        
        # Add frames to the figure
        fig.frames = frames_list
        
        # Add animation controls
        fig.update_layout(
            updatemenus=[
                {
                    "type": "buttons",
                    "buttons": [
                        {
                            "label": "Play",
                            "method": "animate",
                            "args": [None, {"frame": {"duration": interval, "redraw": True}, "fromcurrent": True}]
                        },
                        {
                            "label": "Pause",
                            "method": "animate",
                            "args": [[None], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate"}]
                        }
                    ],
                    "direction": "left",
                    "pad": {"r": 10, "t": 10},
                    "showactive": False,
                    "x": 0.1,
                    "y": 0,
                    "xanchor": "right",
                    "yanchor": "top"
                }
            ],
            sliders=[{
                "active": 0,
                "yanchor": "top",
                "xanchor": "left",
                "currentvalue": {
                    "prefix": "Frame: ",
                    "visible": True,
                    "xanchor": "right"
                },
                "pad": {"b": 10, "t": 50},
                "len": 0.9,
                "x": 0.1,
                "y": 0,
                "steps": [
                    {
                        "args": [
                            [f"frame_{i}"],
                            {"frame": {"duration": interval, "redraw": True}, "mode": "immediate"}
                        ],
                        "label": str(i),
                        "method": "animate"
                    }
                    for i in range(frames)
                ]
            }]
        )
        
        display(fig)
        return fig


def is_sphinx_build():
    """Check if code is running in Sphinx build environment."""
    return os.getenv('SPHINX_BUILD') == '1'
