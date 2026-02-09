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

from enum import auto, Enum
from typing import Callable


class ButtonState(Enum):
    Released = 0  # match ROS joy states
    Pressed = 1  # match ROS joy states


class ButtonEvent(Enum):
    Tapped = auto()


# https://docs.ros.org/en/ros2_packages/jazzy/api/joy/
class ButtonIndex(Enum):
    Cross = 0
    Circle = 1
    Square = 2
    Triangle = 3
    Select = 4
    PS = 5
    Start = 6
    L3 = 7
    R3 = 8
    L1 = 9
    R1 = 10
    DpadUp = 11
    DpadDown = 12
    DpadLeft = 13
    DpadRight = 14
    TouchpadButton = 20


class ButtonAxis(Enum):
    LeftX = 0
    LeftY = 1
    RightX = 2
    RightY = 3
    TriggerLeft = 4
    TriggerRight = 5


class JoystickButton:
    """
    Helper class for processing joystick buttons.

    Parameters
    ----------
    button_index: ButtonIndex
        Index of the button to process
    event_handler: Callable
        Callback to call when button is pressed

    """

    def __init__(self, button_index: ButtonIndex, event_handler: Callable):
        self.button_index = button_index
        self.event_handler = event_handler
        assert event_handler is not None, 'event_handler must be provided'

        self.current_state = ButtonState.Released
        self.last_state = ButtonState.Released

    def update(self, joy_buttons_array):
        self.last_state = self.current_state
        self.current_state = ButtonState(joy_buttons_array[self.button_index.value])

        if self.last_state == ButtonState.Released and self.current_state == ButtonState.Pressed:
            self.event_handler(self, ButtonEvent.Tapped)
