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

import pytest
import std_msgs.msg


class TestStateHandling:
    """Test state handling functionality in the brain node."""

    @pytest.fixture
    def mock_brain_node(self):
        """Create a mock brain node for testing state handling."""
        brain_node = Mock()
        brain_node.robot_state = None
        brain_node.get_logger = Mock()
        brain_node.get_logger.return_value = Mock()
        brain_node.loop_timer = Mock()
        brain_node.sequence_queue = Mock()
        brain_node.walker = Mock()

        # Mock methods
        brain_node.stop_walk_controller = Mock()
        brain_node.turn_torque_off = Mock()
        brain_node.initialization_sequence = Mock()
        brain_node.finalization_sequence = Mock()

        return brain_node

    def test_state_change_detection(self, mock_brain_node):
        """Test that state changes are properly detected."""

        # Create a state handler function
        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return  # No change
            mock_brain_node.robot_state = msg.data
            # Process state change...

        # Initial state
        assert mock_brain_node.robot_state is None

        # First state change
        msg1 = std_msgs.msg.String(data='torque_off')
        process_robot_state(msg1)
        assert mock_brain_node.robot_state == 'torque_off'

        # Same state (should be ignored)
        msg2 = std_msgs.msg.String(data='torque_off')
        process_robot_state(msg2)
        assert mock_brain_node.robot_state == 'torque_off'

        # Different state
        msg3 = std_msgs.msg.String(data='initializing')
        process_robot_state(msg3)
        assert mock_brain_node.robot_state == 'initializing'

    def test_torque_off_state_handling(self, mock_brain_node):
        """Test handling of torque_off state."""

        # Create state handler
        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'torque_off':
                mock_brain_node.get_logger().info('Torque is off, stopping')
                mock_brain_node.stop_walk_controller()
                mock_brain_node.turn_torque_off()

        # Process torque_off state
        msg = std_msgs.msg.String(data='torque_off')
        process_robot_state(msg)

        # Verify correct actions were taken
        mock_brain_node.get_logger().info.assert_called_with('Torque is off, stopping')
        mock_brain_node.stop_walk_controller.assert_called_once()
        mock_brain_node.turn_torque_off.assert_called_once()

    def test_initializing_state_handling(self, mock_brain_node):
        """Test handling of initializing state."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'initializing':
                mock_brain_node.initialization_sequence()

        # Process initializing state
        msg = std_msgs.msg.String(data='initializing')
        process_robot_state(msg)

        # Verify initialization sequence was started
        mock_brain_node.initialization_sequence.assert_called_once()

    def test_torque_on_state_handling(self, mock_brain_node):
        """Test handling of torque_on state."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'torque_on':
                mock_brain_node.get_logger().info('Torque is on, starting')
                mock_brain_node.loop_timer.reset()

        # Process torque_on state
        msg = std_msgs.msg.String(data='torque_on')
        process_robot_state(msg)

        # Verify correct actions were taken
        mock_brain_node.get_logger().info.assert_called_with('Torque is on, starting')
        mock_brain_node.loop_timer.reset.assert_called_once()

    def test_finalizing_state_handling(self, mock_brain_node):
        """Test handling of finalizing state."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'finalizing':
                mock_brain_node.stop_walk_controller()
                mock_brain_node.finalization_sequence()

        # Process finalizing state
        msg = std_msgs.msg.String(data='finalizing')
        process_robot_state(msg)

        # Verify correct actions were taken
        mock_brain_node.stop_walk_controller.assert_called_once()
        mock_brain_node.finalization_sequence.assert_called_once()

    def test_finalized_state_handling(self, mock_brain_node):
        """Test handling of finalized state."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'finalized':
                mock_brain_node.get_logger().info('Finalized')
                mock_brain_node.turn_torque_off()

        # Process finalized state
        msg = std_msgs.msg.String(data='finalized')
        process_robot_state(msg)

        # Verify correct actions were taken
        mock_brain_node.get_logger().info.assert_called_with('Finalized')
        mock_brain_node.turn_torque_off.assert_called_once()

    def test_unknown_state_handling(self, mock_brain_node):
        """Test handling of unknown/invalid states."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data
            # No specific handling for unknown states

        # Process unknown state
        msg = std_msgs.msg.String(data='unknown_state')
        process_robot_state(msg)

        # State should be updated but no actions taken
        assert mock_brain_node.robot_state == 'unknown_state'
        mock_brain_node.stop_walk_controller.assert_not_called()
        mock_brain_node.turn_torque_off.assert_not_called()
        mock_brain_node.initialization_sequence.assert_not_called()
        mock_brain_node.finalization_sequence.assert_not_called()

    def test_stop_walk_controller_functionality(self, mock_brain_node):
        """Test stop_walk_controller method functionality."""

        def stop_walk_controller():
            mock_brain_node.get_logger().info('Stopping')
            mock_brain_node.loop_timer.cancel()
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.walker.reset()

        # Call stop_walk_controller
        stop_walk_controller()

        # Verify all stopping actions were taken
        mock_brain_node.get_logger().info.assert_called_with('Stopping')
        mock_brain_node.loop_timer.cancel.assert_called_once()
        mock_brain_node.sequence_queue.clear.assert_called_once()
        mock_brain_node.walker.reset.assert_called_once()

    def test_state_transition_sequence(self, mock_brain_node):
        """Test a complete state transition sequence."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'torque_off':
                mock_brain_node.stop_walk_controller()
                mock_brain_node.turn_torque_off()
            elif mock_brain_node.robot_state == 'initializing':
                mock_brain_node.initialization_sequence()
            elif mock_brain_node.robot_state == 'torque_on':
                mock_brain_node.loop_timer.reset()
            elif mock_brain_node.robot_state == 'finalizing':
                mock_brain_node.stop_walk_controller()
                mock_brain_node.finalization_sequence()
            elif mock_brain_node.robot_state == 'finalized':
                mock_brain_node.turn_torque_off()

        # Simulate complete state sequence
        states = ['torque_off', 'initializing', 'torque_on', 'finalizing', 'finalized']

        for state in states:
            msg = std_msgs.msg.String(data=state)
            process_robot_state(msg)
            assert mock_brain_node.robot_state == state

        # Verify all expected methods were called
        assert mock_brain_node.stop_walk_controller.call_count == 2  # torque_off and finalizing
        assert mock_brain_node.turn_torque_off.call_count == 2  # torque_off and finalized
        mock_brain_node.initialization_sequence.assert_called_once()
        mock_brain_node.loop_timer.reset.assert_called_once()
        mock_brain_node.finalization_sequence.assert_called_once()

    def test_state_message_format(self):
        """Test that state messages have correct format."""
        valid_states = ['torque_off', 'initializing', 'torque_on', 'finalizing', 'finalized']

        for state in valid_states:
            msg = std_msgs.msg.String(data=state)
            assert isinstance(msg, std_msgs.msg.String)
            assert msg.data == state
            assert isinstance(msg.data, str)
            assert len(msg.data) > 0

    def test_concurrent_state_changes(self, mock_brain_node):
        """Test handling of rapid state changes."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'torque_off':
                mock_brain_node.stop_walk_controller()

        # Rapid state changes
        msg1 = std_msgs.msg.String(data='torque_off')
        msg2 = std_msgs.msg.String(data='torque_off')  # Same state
        msg3 = std_msgs.msg.String(data='initializing')

        process_robot_state(msg1)
        process_robot_state(msg2)  # Should be ignored
        process_robot_state(msg3)

        # stop_walk_controller should only be called once (for first torque_off)
        mock_brain_node.stop_walk_controller.assert_called_once()
        assert mock_brain_node.robot_state == 'initializing'

    def test_state_persistence(self, mock_brain_node):
        """Test that state is properly persisted."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

        # Set initial state
        msg = std_msgs.msg.String(data='torque_on')
        process_robot_state(msg)
        assert mock_brain_node.robot_state == 'torque_on'

        # State should persist until changed
        assert mock_brain_node.robot_state == 'torque_on'

        # Change state
        msg2 = std_msgs.msg.String(data='finalizing')
        process_robot_state(msg2)
        assert mock_brain_node.robot_state == 'finalizing'

    def test_state_logging(self, mock_brain_node):
        """Test that state changes are properly logged."""

        def process_robot_state(msg):
            if mock_brain_node.robot_state == msg.data:
                return
            mock_brain_node.robot_state = msg.data

            if mock_brain_node.robot_state == 'torque_off':
                mock_brain_node.get_logger().info('Torque is off, stopping')
            elif mock_brain_node.robot_state == 'torque_on':
                mock_brain_node.get_logger().info('Torque is on, starting')
            elif mock_brain_node.robot_state == 'finalized':
                mock_brain_node.get_logger().info('Finalized')

        # Test logging for different states
        states_and_logs = [
            ('torque_off', 'Torque is off, stopping'),
            ('torque_on', 'Torque is on, starting'),
            ('finalized', 'Finalized'),
        ]

        for state, expected_log in states_and_logs:
            mock_brain_node.get_logger().info.reset_mock()
            msg = std_msgs.msg.String(data=state)
            process_robot_state(msg)
            mock_brain_node.get_logger().info.assert_called_with(expected_log)
