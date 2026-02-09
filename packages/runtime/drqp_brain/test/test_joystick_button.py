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

from unittest.mock import Mock

from drqp_brain.joystick_button import ButtonEvent, ButtonIndex, ButtonState, JoystickButton
import pytest


class TestJoystickButton:
    """Test the JoystickButton class."""

    @pytest.fixture
    def mock_event_handler(self):
        """Create a mock event handler for testing."""
        return Mock()

    @pytest.fixture
    def joystick_button(self, mock_event_handler):
        """Create a JoystickButton instance for testing."""
        return JoystickButton(ButtonIndex.Cross, mock_event_handler)

    def test_initialization(self, joystick_button, mock_event_handler):
        """Test that JoystickButton initializes correctly."""
        assert joystick_button.button_index == ButtonIndex.Cross
        assert joystick_button.event_handler == mock_event_handler
        assert joystick_button.current_state == ButtonState.Released
        assert joystick_button.last_state == ButtonState.Released

    def test_update_button_released(self, joystick_button, mock_event_handler):
        """Test update when button remains released."""
        # Create a joy buttons array with all buttons released (15 buttons total)
        joy_buttons = [0] * 15

        joystick_button.update(joy_buttons)

        assert joystick_button.last_state == ButtonState.Released
        assert joystick_button.current_state == ButtonState.Released
        # No event should be triggered
        mock_event_handler.assert_not_called()

    def test_update_button_pressed_from_released(self, joystick_button, mock_event_handler):
        """Test update when button is pressed from released state (tap event)."""
        # Create a joy buttons array with Cross button (index 0) pressed
        joy_buttons = [1] + [0] * 14

        joystick_button.update(joy_buttons)

        assert joystick_button.last_state == ButtonState.Released
        assert joystick_button.current_state == ButtonState.Pressed
        # Tap event should be triggered
        mock_event_handler.assert_called_once_with(joystick_button, ButtonEvent.Tapped)

    def test_update_button_held_pressed(self, joystick_button, mock_event_handler):
        """Test update when button remains pressed (no new event)."""
        # First press the button
        joy_buttons = [1] + [0] * 14
        joystick_button.update(joy_buttons)
        mock_event_handler.reset_mock()

        # Keep button pressed
        joystick_button.update(joy_buttons)

        assert joystick_button.last_state == ButtonState.Pressed
        assert joystick_button.current_state == ButtonState.Pressed
        # No new event should be triggered
        mock_event_handler.assert_not_called()

    def test_update_button_released_from_pressed(self, joystick_button, mock_event_handler):
        """Test update when button is released from pressed state."""
        # First press the button
        joy_buttons_pressed = [1] + [0] * 14
        joystick_button.update(joy_buttons_pressed)
        mock_event_handler.reset_mock()

        # Then release the button
        joy_buttons_released = [0] * 15
        joystick_button.update(joy_buttons_released)

        assert joystick_button.last_state == ButtonState.Pressed
        assert joystick_button.current_state == ButtonState.Released
        # No event should be triggered on release
        mock_event_handler.assert_not_called()

    def test_different_button_indices(self, mock_event_handler):
        """Test that different button indices work correctly."""
        # Test Circle button (index 1)
        circle_button = JoystickButton(ButtonIndex.Circle, mock_event_handler)
        joy_buttons = [0, 1] + [0] * 13  # Circle button pressed

        circle_button.update(joy_buttons)

        assert circle_button.current_state == ButtonState.Pressed
        mock_event_handler.assert_called_once_with(circle_button, ButtonEvent.Tapped)

        mock_event_handler.reset_mock()

        # Test DpadRight button (index 14)
        dpad_button = JoystickButton(ButtonIndex.DpadRight, mock_event_handler)
        joy_buttons = [0] * 14 + [1]  # DpadRight button pressed

        dpad_button.update(joy_buttons)

        assert dpad_button.current_state == ButtonState.Pressed
        mock_event_handler.assert_called_once_with(dpad_button, ButtonEvent.Tapped)

    def test_multiple_press_release_cycles(self, joystick_button, mock_event_handler):
        """Test multiple press/release cycles generate multiple tap events."""
        joy_buttons_released = [0] * 15
        joy_buttons_pressed = [1] + [0] * 14

        # First tap
        joystick_button.update(joy_buttons_pressed)
        assert mock_event_handler.call_count == 1

        # Release
        joystick_button.update(joy_buttons_released)
        assert mock_event_handler.call_count == 1  # No new call on release

        # Second tap
        joystick_button.update(joy_buttons_pressed)
        assert mock_event_handler.call_count == 2

        # Release again
        joystick_button.update(joy_buttons_released)
        assert mock_event_handler.call_count == 2  # No new call on release

    def test_button_state_enum_values(self):
        """Test that ButtonState enum has correct values matching ROS joy states."""
        assert ButtonState.Released.value == 0
        assert ButtonState.Pressed.value == 1

    def test_button_index_enum_values(self):
        """Test that ButtonIndex enum has correct values."""
        assert ButtonIndex.Cross.value == 0
        assert ButtonIndex.Circle.value == 1
        assert ButtonIndex.Square.value == 2
        assert ButtonIndex.Triangle.value == 3
        assert ButtonIndex.Select.value == 4
        assert ButtonIndex.PS.value == 5
        assert ButtonIndex.Start.value == 6
        assert ButtonIndex.L3.value == 7
        assert ButtonIndex.R3.value == 8
        assert ButtonIndex.L1.value == 9
        assert ButtonIndex.R1.value == 10
        assert ButtonIndex.DpadUp.value == 11
        assert ButtonIndex.DpadDown.value == 12
        assert ButtonIndex.DpadLeft.value == 13
        assert ButtonIndex.DpadRight.value == 14
