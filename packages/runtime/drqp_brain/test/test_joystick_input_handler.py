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

from unittest.mock import Mock

from drqp_brain.geometry import Point3D
from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.joystick_input_handler import JoystickInputHandler
import pytest
import sensor_msgs.msg


class TestJoystickInputHandler:
    """Test the JoystickInputHandler class."""

    @pytest.fixture
    def input_handler(self):
        """Create a JoystickInputHandler instance for testing."""
        return JoystickInputHandler()

    @pytest.fixture
    def mock_callback(self):
        """Create a mock callback for button testing."""
        return Mock()

    @pytest.fixture
    def input_handler_with_callbacks(self, mock_callback):
        """Create a JoystickInputHandler with button callbacks."""
        callbacks = {
            ButtonIndex.Cross: mock_callback,
            ButtonIndex.PS: mock_callback,
        }
        return JoystickInputHandler(button_callbacks=callbacks)

    def test_initialization(self, input_handler):
        """Test that JoystickInputHandler initializes correctly."""
        assert input_handler.direction == Point3D([0, 0, 0])
        assert input_handler.walk_speed == pytest.approx(0.0)
        assert input_handler.rotation_speed == pytest.approx(0.0)
        assert len(input_handler.joystick_buttons) == 0

    def test_initialization_with_callbacks(self, input_handler_with_callbacks):
        """Test initialization with button callbacks."""
        assert len(input_handler_with_callbacks.joystick_buttons) == 2

        # Check that buttons were created with correct indices
        button_indices = [btn.button_index for btn in input_handler_with_callbacks.joystick_buttons]
        assert ButtonIndex.Cross in button_indices
        assert ButtonIndex.PS in button_indices

    def test_axes_processing_forward_movement(self, input_handler):
        """Test processing of joystick axes for forward movement."""
        # Create joy message with forward movement (left stick up)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0]  # left_x=0, left_y=0.8, right_x=0, ...
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        # Direction should be forward (left_y maps to x)
        assert input_handler.direction.x == pytest.approx(0.8)
        assert input_handler.direction.y == pytest.approx(0.0)
        assert input_handler.direction.z == pytest.approx(0.0)
        assert input_handler.walk_speed == pytest.approx(0.8)
        assert input_handler.rotation_speed == pytest.approx(0.0)

    def test_axes_processing_left_movement(self, input_handler):
        """Test processing of joystick axes for left movement."""
        # Create joy message with left movement (left stick left)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [-0.6, 0.0, 0.0, 0.0, 0.0, 0.0]  # left_x=-0.6, left_y=0, ...
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        # Direction should be left (left_x maps to y)
        assert input_handler.direction.x == pytest.approx(0.0)
        assert input_handler.direction.y == pytest.approx(-0.6)
        assert input_handler.direction.z == pytest.approx(0.0)
        assert input_handler.walk_speed == pytest.approx(0.6)
        assert input_handler.rotation_speed == pytest.approx(0.0)

    def test_axes_processing_rotation(self, input_handler):
        """Test processing of joystick axes for rotation."""
        # Create joy message with rotation (right stick)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # right_x=0.5
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        assert input_handler.rotation_speed == 0.5

    def test_axes_processing_trigger(self, input_handler):
        """Test processing of left trigger for vertical movement."""
        # Create joy message with left trigger pressed
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, -0.5, 0.0]  # left_trigger=-0.5
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        # Trigger should be processed and inverted
        expected_trigger = float(((-0.5) - (-1)) / (0 - (-1)))  # interp from [-1,0] to [1,0]
        assert abs(input_handler.direction.z - expected_trigger) < 1e-6

    def test_axes_processing_combined_movement(self, input_handler):
        """Test processing of combined joystick movements."""
        # Create joy message with combined movement
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.3, 0.4, 0.2, 0.0, -0.8, 0.0]  # left_x, left_y, right_x, ..., left_trigger
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        assert input_handler.direction.x == pytest.approx(0.4)  # left_y
        assert input_handler.direction.y == pytest.approx(0.3)  # left_x
        assert input_handler.rotation_speed == pytest.approx(0.2)  # right_x

        # Walk speed should be sum of absolute values
        expected_walk_speed = abs(0.3) + abs(0.4) + abs(input_handler.direction.z)
        assert input_handler.walk_speed == pytest.approx(expected_walk_speed)

    def test_button_processing(self, input_handler_with_callbacks, mock_callback):
        """Test processing of joystick buttons."""
        # Create joy message with Cross button pressed
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [1] + [0] * 14  # Cross button pressed

        input_handler_with_callbacks.process_joy_message(joy_msg)

        # Mock callback should have been called
        mock_callback.assert_called()

    def test_add_button_handler(self, input_handler, mock_callback):
        """Test adding a button handler."""
        initial_count = len(input_handler.joystick_buttons)

        input_handler.add_button_handler(ButtonIndex.Circle, mock_callback)

        assert len(input_handler.joystick_buttons) == initial_count + 1

        # Find the added button
        circle_button = None
        for button in input_handler.joystick_buttons:
            if button.button_index == ButtonIndex.Circle:
                circle_button = button
                break

        assert circle_button is not None
        assert circle_button.event_handler == mock_callback

    def test_remove_button_handler(self, input_handler_with_callbacks):
        """Test removing a button handler."""
        initial_count = len(input_handler_with_callbacks.joystick_buttons)

        input_handler_with_callbacks.remove_button_handler(ButtonIndex.Cross)

        assert len(input_handler_with_callbacks.joystick_buttons) == initial_count - 1

        # Verify Cross button was removed
        button_indices = [btn.button_index for btn in input_handler_with_callbacks.joystick_buttons]
        assert ButtonIndex.Cross not in button_indices
        assert ButtonIndex.PS in button_indices  # PS should still be there

    def test_reset(self, input_handler):
        """Test resetting the input handler."""
        # Set some values
        input_handler.direction = Point3D([0.5, -0.3, 0.2])
        input_handler.walk_speed = pytest.approx(0.8)
        input_handler.rotation_speed = pytest.approx(0.1)

        # Reset
        input_handler.reset()

        # Values should be back to defaults
        assert input_handler.direction == Point3D([0, 0, 0])
        assert input_handler.walk_speed == pytest.approx(0)
        assert input_handler.rotation_speed == pytest.approx(0)

    def test_trigger_interpolation_edge_cases(self, input_handler):
        """Test trigger interpolation with edge case values."""
        # Test with trigger at -1 (fully pressed on some platforms)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)
        assert input_handler.direction.z == pytest.approx(1.0)  # Should map to 1.0

        # Test with trigger at 0 (not pressed)
        joy_msg.axes[4] = 0.0
        input_handler.process_joy_message(joy_msg)
        assert input_handler.direction.z == pytest.approx(0.0)  # Should map to 0.0

    def test_walk_speed_calculation(self, input_handler):
        """Test walk speed calculation with various inputs."""
        # Test with all axes at maximum
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [1.0, 1.0, 0.0, 0.0, -1.0, 0.0]  # left_x=1, left_y=1, left_trigger=-1
        joy_msg.buttons = []

        input_handler.process_joy_message(joy_msg)

        # Walk speed should be sum of absolute values: |1| + |1| + |1| = 3
        assert input_handler.walk_speed == pytest.approx(3.0)

    def test_no_button_callbacks_initialization(self):
        """Test initialization with no button callbacks."""
        handler = JoystickInputHandler(button_callbacks=None)
        assert len(handler.joystick_buttons) == 0

    def test_empty_button_callbacks_initialization(self):
        """Test initialization with empty button callbacks."""
        handler = JoystickInputHandler(button_callbacks={})
        assert len(handler.joystick_buttons) == 0

    def test_multiple_button_handlers_same_button(self, input_handler):
        """Test adding multiple handlers for the same button."""
        mock_callback1 = Mock()
        mock_callback2 = Mock()

        input_handler.add_button_handler(ButtonIndex.Cross, mock_callback1)
        input_handler.add_button_handler(ButtonIndex.Cross, mock_callback2)

        # Should have two handlers for the same button
        cross_buttons = [
            btn for btn in input_handler.joystick_buttons if btn.button_index == ButtonIndex.Cross
        ]
        assert len(cross_buttons) == 2
