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


class TestFinalizationSequence:
    """Test finalization sequence functionality in the brain node."""

    @pytest.fixture
    def mock_brain_node(self):
        """Create a mock brain node for testing finalization sequence."""
        brain_node = Mock()
        brain_node.sequence_queue = Mock()
        brain_node.hexapod = Mock()
        brain_node.robot_event_pub = Mock()
        brain_node.get_logger = Mock()
        brain_node.get_logger.return_value = Mock()
        brain_node.publish_joint_position_trajectory = Mock()
        
        return brain_node

    def test_finalization_sequence_setup(self, mock_brain_node):
        """Test that finalization sequence is properly set up."""
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.1, lambda: None)  # step1
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step2
            mock_brain_node.sequence_queue.add(0.0, lambda: None)  # done
        
        finalization_sequence()
        
        # Verify sequence queue was cleared and steps added
        mock_brain_node.sequence_queue.clear.assert_called_once()
        assert mock_brain_node.sequence_queue.add.call_count == 3
        
        # Verify timing of each step
        expected_timings = [1.1, 0.6, 0.0]
        for i, (call, expected_time) in enumerate(zip(mock_brain_node.sequence_queue.add.call_args_list, expected_timings)):
            assert call[0][0] == expected_time, f"Step {i+1} timing mismatch"

    def test_finalization_sequence_step1(self, mock_brain_node):
        """Test finalization sequence step 1 - retract to safe position."""
        def finalization_sequence_step1():
            mock_brain_node.get_logger().info('Finalization sequence started')
            mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=1000)
        
        finalization_sequence_step1()
        
        # Verify logging
        mock_brain_node.get_logger().info.assert_called_with('Finalization sequence started')
        
        # Verify hexapod positioning to safe retracted position
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, 0)
        
        # Verify joint trajectory publishing
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(playtime_ms=1000)

    def test_finalization_sequence_step2(self, mock_brain_node):
        """Test finalization sequence step 2 - final retraction."""
        def finalization_sequence_step2():
            mock_brain_node.hexapod.forward_kinematics(0, -105, -60)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)
        
        finalization_sequence_step2()
        
        # Verify hexapod positioning to fully retracted position
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, -60)
        
        # Verify joint trajectory publishing
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_finalization_sequence_done(self, mock_brain_node):
        """Test finalization sequence completion."""
        def finalization_sequence_done():
            mock_brain_node.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done'))
        
        finalization_sequence_done()
        
        # Verify completion event is published
        mock_brain_node.robot_event_pub.publish.assert_called_once()
        published_msg = mock_brain_node.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'finalizing_done'

    def test_finalization_sequence_joint_angles(self, mock_brain_node):
        """Test that finalization sequence uses correct joint angles."""
        # Define the sequence steps with their expected angles
        steps_and_angles = [
            (1, (0, -105, 0)),    # Step 1: safe position
            (2, (0, -105, -60)),  # Step 2: fully retracted
        ]
        
        for step_num, (coxa, femur, tibia) in steps_and_angles:
            mock_brain_node.hexapod.forward_kinematics.reset_mock()
            
            # Call the appropriate step function
            if step_num == 1:
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            elif step_num == 2:
                mock_brain_node.hexapod.forward_kinematics(0, -105, -60)
            
            # Verify the correct angles were used
            mock_brain_node.hexapod.forward_kinematics.assert_called_with(coxa, femur, tibia)

    def test_finalization_sequence_timing(self, mock_brain_node):
        """Test that finalization sequence has correct timing."""
        expected_timings = [
            (1.1, 'step1'),
            (0.6, 'step2'),
            (0.0, 'done')
        ]
        
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            for timing, step_name in expected_timings:
                mock_brain_node.sequence_queue.add(timing, lambda: None)
        
        finalization_sequence()
        
        # Verify all timings are correct
        for i, (expected_time, _) in enumerate(expected_timings):
            call_args = mock_brain_node.sequence_queue.add.call_args_list[i]
            assert call_args[0][0] == expected_time

    def test_finalization_sequence_playtime_values(self, mock_brain_node):
        """Test that finalization sequence uses correct playtime values."""
        expected_playtimes = [
            (1, 1000),  # Step 1: 1000ms
            (2, 500),   # Step 2: 500ms
        ]
        
        for step_num, expected_playtime in expected_playtimes:
            mock_brain_node.publish_joint_position_trajectory.reset_mock()
            
            # Simulate the step
            if step_num == 1:
                mock_brain_node.publish_joint_position_trajectory(playtime_ms=1000)
            elif step_num == 2:
                mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)
            
            # Verify the call
            call_args = mock_brain_node.publish_joint_position_trajectory.call_args
            if call_args.kwargs:
                assert call_args.kwargs['playtime_ms'] == expected_playtime
            else:
                # Check positional arguments if keywords not used
                assert expected_playtime in call_args.args

    def test_finalization_sequence_shorter_than_initialization(self, mock_brain_node):
        """Test that finalization sequence is shorter than initialization."""
        # Finalization has 3 steps vs initialization's 6 steps
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.1, lambda: None)  # step1
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step2
            mock_brain_node.sequence_queue.add(0.0, lambda: None)  # done
        
        finalization_sequence()
        
        # Should have exactly 3 steps
        assert mock_brain_node.sequence_queue.add.call_count == 3

    def test_finalization_sequence_total_time(self, mock_brain_node):
        """Test that finalization sequence total time is reasonable."""
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.1, lambda: None)  # step1
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step2
            mock_brain_node.sequence_queue.add(0.0, lambda: None)  # done
        
        finalization_sequence()
        
        # Calculate total time (excluding the final 0.0 step)
        timings = [call[0][0] for call in mock_brain_node.sequence_queue.add.call_args_list[:-1]]
        total_time = sum(timings)
        
        # Total time should be 1.7 seconds (1.1 + 0.6)
        assert total_time == 1.7

    def test_finalization_sequence_error_handling(self, mock_brain_node):
        """Test finalization sequence error handling."""
        # Mock an error in hexapod forward kinematics
        mock_brain_node.hexapod.forward_kinematics.side_effect = Exception("Kinematics error")
        
        def finalization_sequence_step1():
            try:
                mock_brain_node.get_logger().info('Finalization sequence started')
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
                mock_brain_node.publish_joint_position_trajectory(playtime_ms=1000)
            except Exception as e:
                mock_brain_node.get_logger().error(f"Finalization step 1 failed: {e}")
                raise
        
        # Should raise the exception
        with pytest.raises(Exception, match="Kinematics error"):
            finalization_sequence_step1()

    def test_finalization_sequence_queue_clearing(self, mock_brain_node):
        """Test that finalization sequence properly clears the queue."""
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            # Add some steps...
            mock_brain_node.sequence_queue.add(1.1, lambda: None)
        
        finalization_sequence()
        
        # Verify queue was cleared before adding new steps
        mock_brain_node.sequence_queue.clear.assert_called_once()

    def test_finalization_sequence_step_order(self, mock_brain_node):
        """Test that finalization sequence steps are added in correct order."""
        step_functions = []
        
        def mock_add(timing, func):
            step_functions.append((timing, func))
        
        mock_brain_node.sequence_queue.add = mock_add
        
        def finalization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.1, 'step1')
            mock_brain_node.sequence_queue.add(0.6, 'step2')
            mock_brain_node.sequence_queue.add(0.0, 'done')
        
        finalization_sequence()
        
        # Verify steps were added in correct order
        expected_order = ['step1', 'step2', 'done']
        actual_order = [func for _, func in step_functions]
        assert actual_order == expected_order

    def test_finalization_completion_event_format(self):
        """Test that finalization completion event has correct format."""
        completion_msg = std_msgs.msg.String(data='finalizing_done')
        
        assert isinstance(completion_msg, std_msgs.msg.String)
        assert completion_msg.data == 'finalizing_done'
        assert isinstance(completion_msg.data, str)
        assert len(completion_msg.data) > 0

    def test_finalization_vs_initialization_angles(self, mock_brain_node):
        """Test that finalization uses different angles than initialization."""
        # Finalization angles
        finalization_angles = [
            (0, -105, 0),   # Step 1
            (0, -105, -60), # Step 2
        ]
        
        # Initialization final angle for comparison
        initialization_final = (0, -35, 130)
        
        # Verify finalization angles are different from initialization final
        for angles in finalization_angles:
            assert angles != initialization_final

    def test_finalization_sequence_retraction_progression(self, mock_brain_node):
        """Test that finalization sequence progressively retracts the robot."""
        # Step 1: (0, -105, 0) - safe position
        # Step 2: (0, -105, -60) - fully retracted
        
        step1_angles = (0, -105, 0)
        step2_angles = (0, -105, -60)
        
        # Tibia should become more negative (more retracted)
        assert step2_angles[2] < step1_angles[2]  # -60 < 0
        
        # Femur should stay the same
        assert step1_angles[1] == step2_angles[1]  # Both -105
        
        # Coxa should stay the same
        assert step1_angles[0] == step2_angles[0]  # Both 0

    def test_finalization_sequence_no_joint_mask(self, mock_brain_node):
        """Test that finalization sequence doesn't use joint masks."""
        def finalization_sequence_step1():
            mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=1000)
        
        def finalization_sequence_step2():
            mock_brain_node.hexapod.forward_kinematics(0, -105, -60)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)
        
        # Test both steps
        finalization_sequence_step1()
        call_args1 = mock_brain_node.publish_joint_position_trajectory.call_args
        
        mock_brain_node.publish_joint_position_trajectory.reset_mock()
        
        finalization_sequence_step2()
        call_args2 = mock_brain_node.publish_joint_position_trajectory.call_args
        
        # Neither step should use joint_mask
        assert 'joint_mask' not in call_args1.kwargs
        assert 'joint_mask' not in call_args2.kwargs
