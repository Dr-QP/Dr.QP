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

from drqp_brain.geometry import Point3D
from drqp_brain.joystick_button import ButtonAxis, ButtonIndex, JoystickButton
import numpy as np
import sensor_msgs.msg


class JoystickInputHandler:
    """
    Handles joystick input processing for the hexapod robot.

    Processes joystick axes and buttons, converting them to movement commands
    and triggering appropriate callbacks for button presses.
    """

    def __init__(self, button_callbacks=None):
        """
        Initialize the joystick input handler.

        Parameters
        ----------
        button_callbacks : dict, optional
            Dictionary mapping ButtonIndex to callback functions.
            Each callback should accept (button, event) parameters.

        """
        self.reset()

        # Set up button handlers
        self.joystick_buttons = []
        if button_callbacks:
            for button_index, callback in button_callbacks.items():
                self.joystick_buttons.append(JoystickButton(button_index, callback))

    def reset(self):
        """Reset all movement parameters to zero."""
        self.direction = Point3D([0, 0, 0])
        self.rotation_speed = 0

    def process_joy_message(self, joy: sensor_msgs.msg.Joy):
        """
        Process a ROS Joy message and update movement parameters.

        Parameters
        ----------
        joy : sensor_msgs.msg.Joy
            The joystick message to process

        """
        self._process_axes(joy.axes)
        self._process_buttons(joy.buttons)

    def _process_axes(self, axes):
        """Process joystick axes to extract movement commands."""
        left_x = axes[ButtonAxis.LeftX.value]
        left_y = axes[ButtonAxis.LeftY.value]
        right_x = axes[ButtonAxis.RightX.value]

        # On some platforms default value for trigger is -1 (robobook with ubuntu 24.04)
        # but on raspi with ubuntu 24.04 it is 0
        left_trigger = float(np.interp(axes[ButtonAxis.TriggerLeft.value], [-1, 0], [1, 0]))

        self.direction = Point3D([left_y, left_x, left_trigger])
        self.rotation_speed = right_x

    def _process_buttons(self, buttons):
        """Process joystick buttons and trigger callbacks."""
        for button in self.joystick_buttons:
            button.update(buttons)

    def add_button_handler(self, button_index: ButtonIndex, callback):
        """
        Add a button handler.

        Parameters
        ----------
        button_index : ButtonIndex
            The button to handle
        callback : callable
            Function to call when button is pressed. Should accept (button, event) parameters.

        """
        self.joystick_buttons.append(JoystickButton(button_index, callback))

    def remove_button_handler(self, button_index: ButtonIndex):
        """
        Remove a button handler.

        Parameters
        ----------
        button_index : ButtonIndex
            The button handler to remove

        """
        self.joystick_buttons = [
            button for button in self.joystick_buttons if button.button_index != button_index
        ]
