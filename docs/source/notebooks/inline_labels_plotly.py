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
import plotly.graph_objects as go


def add_inline_labels(fig, with_overall_progress=True, fontsize='medium'):
    """
    Add inline labels to a Plotly figure.

    This function adds text annotations to line traces in a Plotly figure,
    positioning them along the lines they describe.

    Parameters:
    -----------
    fig : plotly.graph_objects.Figure
        The Plotly figure to add labels to
    with_overall_progress : bool, optional
        Whether to add a progress indicator, by default True
    fontsize : str, optional
        Font size for the labels, by default 'medium'

    Returns:
    --------
    plotly.graph_objects.Figure
        The figure with added annotations

    """
    # Convert fontsize to pixel size
    font_size_map = {'small': 10, 'medium': 12, 'large': 14}
    font_size = font_size_map.get(fontsize, 12)

    # Process each trace in the figure
    for trace in fig.data:
        if trace.mode and 'lines' in trace.mode and trace.name:
            # Get line coordinates
            x_values = trace.x
            y_values = trace.y

            if len(x_values) < 2 or len(y_values) < 2:
                continue

            # Calculate the midpoint of the line
            mid_idx = len(x_values) // 2
            if len(x_values) > 2:
                # For lines with multiple segments, find a good position
                x_mid = x_values[mid_idx]
                y_mid = y_values[mid_idx]
            else:
                # For simple lines, use the midpoint
                x_mid = (x_values[0] + x_values[-1]) / 2
                y_mid = (y_values[0] + y_values[-1]) / 2

            # Calculate angle for text alignment
            if len(x_values) > 2:
                dx = x_values[mid_idx + 1] - x_values[mid_idx - 1]
                dy = y_values[mid_idx + 1] - y_values[mid_idx - 1]
            else:
                dx = x_values[-1] - x_values[0]
                dy = y_values[-1] - y_values[0]

            angle = np.degrees(np.arctan2(dy, dx))

            # Adjust angle for readability
            if -90 <= angle <= 90:
                text_angle = angle
            else:
                text_angle = angle - 180

            # Add annotation
            fig.add_annotation(
                x=x_mid,
                y=y_mid,
                text=trace.name,
                showarrow=False,
                font={
                    'family': 'Arial',
                    'size': font_size,
                    'color': trace.line.color if hasattr(trace.line, 'color') else 'black',
                },
                textangle=text_angle,
                xanchor='center',
                yanchor='middle',
                bgcolor='rgba(255, 255, 255, 0.7)',
                borderpad=2,
            )

    return fig


class AngleAnnotation:
    """
    Create an angle annotation for Plotly figures.

    This class creates an arc between two vectors and adds a text label.
    """

    def __init__(
        self,
        xy,
        p1,
        p2,
        fig,
        size=75,
        text='',
        color='black',
        textposition='inside',
        linestyle='solid',
        text_kw=None,
    ):
        """
        Initialize an angle annotation.

        Parameters:
        -----------
        xy : tuple or array of two floats
            Center position
        p1, p2 : tuple or array of two floats
            Two points defining the angle
        fig : plotly.graph_objects.Figure
            The figure to add the annotation to
        size : float, optional
            Size of the arc, by default 75
        text : str, optional
            Text to display, by default ''
        color : str, optional
            Color of the arc and text, by default 'black'
        textposition : str, optional
            Position of the text ('inside', 'outside'), by default 'inside'
        linestyle : str, optional
            Style of the arc line ('solid', 'dash', 'dot'), by default 'solid'
        text_kw : dict, optional
            Additional text styling options, by default None
        """
        self.fig = fig
        self.xy = np.array(xy)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.size = size
        self.text = text
        self.color = color
        self.textposition = textposition

        # Convert matplotlib linestyle to plotly dash
        dash_map = {
            'solid': 'solid',
            '-': 'solid',
            'dashed': 'dash',
            '--': 'dash',
            'dotted': 'dot',
            ':': 'dot',
            'dashdot': 'dashdot',
            '-.': 'dashdot',
        }
        self.dash = dash_map.get(linestyle, 'solid')

        # Default text styling
        self.text_kw = {'font_size': 10, 'font_color': color}
        if text_kw:
            self.text_kw.update(text_kw)

        # Draw the angle annotation
        self._draw()

    def _draw(self):
        """Draw the angle annotation on the figure."""
        # Calculate vectors from center to points
        v1 = self.p1 - self.xy
        v2 = self.p2 - self.xy

        # Calculate angles in degrees
        angle1 = np.degrees(np.arctan2(v1[1], v1[0]))
        angle2 = np.degrees(np.arctan2(v2[1], v2[0]))

        # Ensure angle2 > angle1 for proper arc drawing
        if angle2 < angle1:
            angle2 += 360

        # Create points for the arc
        theta = np.linspace(np.radians(angle1), np.radians(angle2), 50)
        radius = self.size / 100  # Scale size appropriately

        x = self.xy[0] + radius * np.cos(theta)
        y = self.xy[1] + radius * np.sin(theta)

        # Add the arc to the figure
        self.fig.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode='lines',
                line={'color': self.color, 'dash': self.dash, 'width': 1.5},
                showlegend=False,
            )
        )

        # Add the text label
        if self.text:
            # Calculate position for text
            angle_mid = np.radians((angle1 + angle2) / 2)

            if self.textposition == 'inside':
                text_radius = radius * 0.7
            else:  # 'outside'
                text_radius = radius * 1.4

            text_x = self.xy[0] + text_radius * np.cos(angle_mid)
            text_y = self.xy[1] + text_radius * np.sin(angle_mid)

            self.fig.add_annotation(
                x=text_x,
                y=text_y,
                text=self.text,
                showarrow=False,
                font={
                    'family': 'Arial',
                    'size': self.text_kw.get('font_size', 10),
                    'color': self.text_kw.get('font_color', self.color),
                },
                xanchor='center',
                yanchor='middle',
                bgcolor='rgba(255, 255, 255, 0.7)',
                borderpad=2,
            )
