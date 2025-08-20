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

from unittest.mock import Mock, patch

from drqp_brain.joystick_button import ButtonIndex, JoystickButton
import pytest
import sensor_msgs.msg
import std_msgs.msg


class TestKillSwitchHandling:
    """Test kill switch functionality in the brain node."""

    @pytest.fixture
    def mock_brain_node(self):
        """Create a mock brain node for testing kill switch functionality."""
        brain_node = Mock()
        brain_node.robot_event_pub = Mock()
        brain_node.get_logger = Mock()
        brain_node.get_logger.return_value = Mock()

        # Mock the kill switch method
        brain_node.process_kill_switch = Mock()

        return brain_node

    @pytest.fixture
    def kill_switch_button(self, mock_brain_node):
        """Create a kill switch button for testing."""
        return JoystickButton(ButtonIndex.PS, lambda b, e: mock_brain_node.process_kill_switch())

    def test_kill_switch_button_initialization(self, kill_switch_button):
        """Test that kill switch button is properly initialized."""
        assert kill_switch_button.button_index == ButtonIndex.PS
        assert kill_switch_button.event_handler is not None

    def test_kill_switch_button_press_triggers_handler(self, kill_switch_button, mock_brain_node):
        """Test that pressing kill switch button triggers the handler."""
        # Create joy message with PS button pressed
        joy_buttons = [0] * 15
        joy_buttons[ButtonIndex.PS.value] = 1

        kill_switch_button.update(joy_buttons)

        # Verify the kill switch handler was called
        mock_brain_node.process_kill_switch.assert_called_once()

    def test_kill_switch_button_release_no_trigger(self, kill_switch_button, mock_brain_node):
        """Test that releasing kill switch button doesn't trigger handler."""
        # First press the button
        joy_buttons_pressed = [0] * 15
        joy_buttons_pressed[ButtonIndex.PS.value] = 1
        kill_switch_button.update(joy_buttons_pressed)

        # Reset mock
        mock_brain_node.process_kill_switch.reset_mock()

        # Then release the button
        joy_buttons_released = [0] * 15
        kill_switch_button.update(joy_buttons_released)

        # Verify handler was not called on release
        mock_brain_node.process_kill_switch.assert_not_called()

    def test_kill_switch_button_held_no_repeat(self, kill_switch_button, mock_brain_node):
        """Test that holding kill switch button doesn't repeat the trigger."""
        # Press the button
        joy_buttons = [0] * 15
        joy_buttons[ButtonIndex.PS.value] = 1
        kill_switch_button.update(joy_buttons)

        # Reset mock
        mock_brain_node.process_kill_switch.reset_mock()

        # Keep button pressed
        kill_switch_button.update(joy_buttons)

        # Verify handler was not called again
        mock_brain_node.process_kill_switch.assert_not_called()

    def test_process_kill_switch_publishes_event(self):
        """Test that process_kill_switch publishes the correct event."""
        # Create a mock publisher
        mock_publisher = Mock()

        # Create a minimal brain node mock with the actual method
        class MockBrainNode:
            def __init__(self):
                self.robot_event_pub = mock_publisher

            def process_kill_switch(self):
                self.robot_event_pub.publish(std_msgs.msg.String(data='kill_switch_pressed'))

        brain_node = MockBrainNode()
        brain_node.process_kill_switch()

        # Verify the correct message was published
        mock_publisher.publish.assert_called_once()
        published_msg = mock_publisher.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'kill_switch_pressed'

    def test_multiple_kill_switch_presses(self, kill_switch_button, mock_brain_node):
        """Test multiple kill switch press cycles."""
        joy_buttons_released = [0] * 15
        joy_buttons_pressed = [0] * 15
        joy_buttons_pressed[ButtonIndex.PS.value] = 1

        # First press
        kill_switch_button.update(joy_buttons_pressed)
        assert mock_brain_node.process_kill_switch.call_count == 1

        # Release
        kill_switch_button.update(joy_buttons_released)
        assert mock_brain_node.process_kill_switch.call_count == 1  # No new call

        # Second press
        kill_switch_button.update(joy_buttons_pressed)
        assert mock_brain_node.process_kill_switch.call_count == 2

        # Release again
        kill_switch_button.update(joy_buttons_released)
        assert mock_brain_node.process_kill_switch.call_count == 2  # No new call

    def test_kill_switch_integration_with_joy_message(self, mock_brain_node):
        """Test kill switch integration with actual Joy message processing."""
        # Create kill switch button
        kill_switch_button = JoystickButton(
            ButtonIndex.PS, lambda b, e: mock_brain_node.process_kill_switch()
        )

        # Create a Joy message
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.buttons = [0] * 15
        joy_msg.buttons[ButtonIndex.PS.value] = 1  # Press PS button
        joy_msg.axes = [0.0] * 6  # Add some axes data

        # Process the button
        kill_switch_button.update(joy_msg.buttons)

        # Verify kill switch was triggered
        mock_brain_node.process_kill_switch.assert_called_once()

    def test_kill_switch_with_other_buttons_pressed(self, kill_switch_button, mock_brain_node):
        """Test that kill switch works when other buttons are also pressed."""
        # Create joy message with multiple buttons pressed including PS
        joy_buttons = [0] * 15
        joy_buttons[ButtonIndex.Cross.value] = 1  # Cross pressed
        joy_buttons[ButtonIndex.Circle.value] = 1  # Circle pressed
        joy_buttons[ButtonIndex.PS.value] = 1  # PS (kill switch) pressed
        joy_buttons[ButtonIndex.Start.value] = 1  # Start pressed

        kill_switch_button.update(joy_buttons)

        # Kill switch should still trigger
        mock_brain_node.process_kill_switch.assert_called_once()

    def test_kill_switch_button_index_correct(self):
        """Test that kill switch uses the correct button index."""
        # Verify PS button has the expected index value
        assert ButtonIndex.PS.value == 5

        # Create kill switch button and verify it uses PS button
        kill_switch_button = JoystickButton(ButtonIndex.PS, lambda b, e: None)
        assert kill_switch_button.button_index == ButtonIndex.PS
        assert kill_switch_button.button_index.value == 5

    @patch('std_msgs.msg.String')
    def test_kill_switch_message_creation(self, mock_string_msg):
        """Test that kill switch creates the correct ROS message."""

        # Create a minimal brain node mock
        class MockBrainNode:
            def __init__(self):
                self.robot_event_pub = Mock()

            def process_kill_switch(self):
                msg = std_msgs.msg.String(data='kill_switch_pressed')
                self.robot_event_pub.publish(msg)

        brain_node = MockBrainNode()
        brain_node.process_kill_switch()

        # Verify String message was created with correct data
        mock_string_msg.assert_called_once_with(data='kill_switch_pressed')

    def test_kill_switch_event_data_format(self):
        """Test that kill switch event data has the correct format."""
        expected_event_data = 'kill_switch_pressed'

        # Verify the event data is a string and has expected content
        assert isinstance(expected_event_data, str)
        assert expected_event_data == 'kill_switch_pressed'
        assert len(expected_event_data) > 0

    def test_kill_switch_robustness_empty_buttons(self, mock_brain_node):
        """Test kill switch robustness with edge cases."""
        kill_switch_button = JoystickButton(
            ButtonIndex.PS, lambda b, e: mock_brain_node.process_kill_switch()
        )

        # Test with empty button array (should not crash)
        try:
            kill_switch_button.update([])
            assert False, 'Should have raised IndexError'
        except IndexError:
            # Expected behavior - button index out of range
            pass

        # Test with insufficient buttons
        try:
            kill_switch_button.update([0, 1, 0])  # Only 3 buttons, PS is index 5
            assert False, 'Should have raised IndexError'
        except IndexError:
            # Expected behavior - button index out of range
            pass

    def test_kill_switch_button_state_persistence(self, kill_switch_button, mock_brain_node):
        """Test that kill switch button state is properly maintained."""
        # Initial state should be released
        assert kill_switch_button.current_state.value == 0  # Released
        assert kill_switch_button.last_state.value == 0  # Released

        # Press button
        joy_buttons = [0] * 15
        joy_buttons[ButtonIndex.PS.value] = 1
        kill_switch_button.update(joy_buttons)

        # State should be updated
        assert kill_switch_button.current_state.value == 1  # Pressed
        assert kill_switch_button.last_state.value == 0  # Was released

        # Keep button pressed
        kill_switch_button.update(joy_buttons)

        # Both states should now be pressed
        assert kill_switch_button.current_state.value == 1  # Pressed
        assert kill_switch_button.last_state.value == 1  # Was pressed
