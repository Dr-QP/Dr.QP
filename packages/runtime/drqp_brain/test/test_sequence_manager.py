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

from drqp_brain.sequence_manager import SequenceManager
import pytest
import std_msgs.msg


class TestSequenceManager:
    """Test the SequenceManager class."""

    @pytest.fixture
    def mock_dependencies(self):
        """Create mock dependencies for SequenceManager."""
        node = Mock()
        hexapod = Mock()
        trajectory_publisher = Mock()
        event_publisher = Mock()
        logger = Mock()
        
        return {
            'node': node,
            'hexapod': hexapod,
            'trajectory_publisher': trajectory_publisher,
            'event_publisher': event_publisher,
            'logger': logger
        }

    @pytest.fixture
    def sequence_manager(self, mock_dependencies):
        """Create a SequenceManager instance for testing."""
        with patch('drqp_brain.sequence_manager.TimedQueue') as mock_timed_queue:
            manager = SequenceManager(
                node=mock_dependencies['node'],
                hexapod=mock_dependencies['hexapod'],
                trajectory_publisher=mock_dependencies['trajectory_publisher'],
                event_publisher=mock_dependencies['event_publisher'],
                logger=mock_dependencies['logger']
            )
            manager.sequence_queue = Mock()  # Replace with mock for easier testing
            return manager

    def test_initialization(self, sequence_manager, mock_dependencies):
        """Test that SequenceManager initializes correctly."""
        assert sequence_manager.hexapod == mock_dependencies['hexapod']
        assert sequence_manager.publish_joint_position_trajectory == mock_dependencies['trajectory_publisher']
        assert sequence_manager.robot_event_pub == mock_dependencies['event_publisher']
        assert sequence_manager.logger == mock_dependencies['logger']

    def test_start_initialization_sequence(self, sequence_manager):
        """Test starting the initialization sequence."""
        sequence_manager.start_initialization_sequence()
        
        # Verify sequence queue was cleared and steps added
        sequence_manager.sequence_queue.clear.assert_called_once()
        assert sequence_manager.sequence_queue.add.call_count == 6
        
        # Verify timing of each step
        expected_timings = [1.0, 0.6, 0.6, 0.6, 0.5, 0.0]
        for i, expected_time in enumerate(expected_timings):
            call_args = sequence_manager.sequence_queue.add.call_args_list[i]
            assert call_args[0][0] == expected_time

    def test_start_finalization_sequence(self, sequence_manager):
        """Test starting the finalization sequence."""
        sequence_manager.start_finalization_sequence()
        
        # Verify sequence queue was cleared and steps added
        sequence_manager.sequence_queue.clear.assert_called_once()
        assert sequence_manager.sequence_queue.add.call_count == 3
        
        # Verify timing of each step
        expected_timings = [1.1, 0.6, 0.0]
        for i, expected_time in enumerate(expected_timings):
            call_args = sequence_manager.sequence_queue.add.call_args_list[i]
            assert call_args[0][0] == expected_time

    def test_clear_sequence(self, sequence_manager):
        """Test clearing a running sequence."""
        sequence_manager.clear_sequence()
        sequence_manager.sequence_queue.clear.assert_called_once()

    def test_initialization_step1(self, sequence_manager):
        """Test initialization step 1."""
        sequence_manager._initialization_step1()
        
        # Verify logging
        sequence_manager.logger.info.assert_called_with('Initialization sequence started')
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, 0)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(
            playtime_ms=900, joint_mask=['femur']
        )

    def test_initialization_step2(self, sequence_manager):
        """Test initialization step 2."""
        sequence_manager._initialization_step2()
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, 0)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(
            playtime_ms=500, joint_mask=['femur', 'tibia']
        )

    def test_initialization_step3(self, sequence_manager):
        """Test initialization step 3."""
        sequence_manager._initialization_step3()
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, 0)
        
        # Verify trajectory publishing (no joint mask)
        sequence_manager.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_initialization_step4(self, sequence_manager):
        """Test initialization step 4."""
        sequence_manager._initialization_step4()
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, 95)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_initialization_step5(self, sequence_manager):
        """Test initialization step 5."""
        sequence_manager._initialization_step5()
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -35, 130)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(playtime_ms=400)

    def test_initialization_done(self, sequence_manager):
        """Test initialization completion."""
        sequence_manager._initialization_done()
        
        # Verify event publication
        sequence_manager.robot_event_pub.publish.assert_called_once()
        published_msg = sequence_manager.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'initializing_done'

    def test_finalization_step1(self, sequence_manager):
        """Test finalization step 1."""
        sequence_manager._finalization_step1()
        
        # Verify logging
        sequence_manager.logger.info.assert_called_with('Finalization sequence started')
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, 0)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(playtime_ms=1000)

    def test_finalization_step2(self, sequence_manager):
        """Test finalization step 2."""
        sequence_manager._finalization_step2()
        
        # Verify hexapod positioning
        sequence_manager.hexapod.forward_kinematics.assert_called_with(0, -105, -60)
        
        # Verify trajectory publishing
        sequence_manager.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_finalization_done(self, sequence_manager):
        """Test finalization completion."""
        sequence_manager._finalization_done()
        
        # Verify event publication
        sequence_manager.robot_event_pub.publish.assert_called_once()
        published_msg = sequence_manager.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'finalizing_done'

    def test_get_initialization_steps(self, sequence_manager):
        """Test getting initialization steps for testing."""
        steps = sequence_manager.get_initialization_steps()
        
        assert len(steps) == 6
        
        # Verify timing values
        expected_timings = [1.0, 0.6, 0.6, 0.6, 0.5, 0.0]
        for i, (timing, _) in enumerate(steps):
            assert timing == expected_timings[i]
        
        # Verify step functions
        assert steps[0][1] == sequence_manager._initialization_step1
        assert steps[1][1] == sequence_manager._initialization_step2
        assert steps[2][1] == sequence_manager._initialization_step3
        assert steps[3][1] == sequence_manager._initialization_step4
        assert steps[4][1] == sequence_manager._initialization_step5
        assert steps[5][1] == sequence_manager._initialization_done

    def test_get_finalization_steps(self, sequence_manager):
        """Test getting finalization steps for testing."""
        steps = sequence_manager.get_finalization_steps()
        
        assert len(steps) == 3
        
        # Verify timing values
        expected_timings = [1.1, 0.6, 0.0]
        for i, (timing, _) in enumerate(steps):
            assert timing == expected_timings[i]
        
        # Verify step functions
        assert steps[0][1] == sequence_manager._finalization_step1
        assert steps[1][1] == sequence_manager._finalization_step2
        assert steps[2][1] == sequence_manager._finalization_done

    def test_initialization_sequence_joint_angles_progression(self, sequence_manager):
        """Test that initialization sequence uses correct joint angle progression."""
        # Execute all initialization steps and verify angle progression
        steps = [
            (sequence_manager._initialization_step1, (0, -105, 0)),
            (sequence_manager._initialization_step2, (0, -105, 0)),
            (sequence_manager._initialization_step3, (0, -105, 0)),
            (sequence_manager._initialization_step4, (0, -105, 95)),
            (sequence_manager._initialization_step5, (0, -35, 130)),
        ]
        
        for step_func, expected_angles in steps:
            sequence_manager.hexapod.forward_kinematics.reset_mock()
            step_func()
            sequence_manager.hexapod.forward_kinematics.assert_called_with(*expected_angles)

    def test_finalization_sequence_joint_angles_progression(self, sequence_manager):
        """Test that finalization sequence uses correct joint angle progression."""
        # Execute all finalization steps and verify angle progression
        steps = [
            (sequence_manager._finalization_step1, (0, -105, 0)),
            (sequence_manager._finalization_step2, (0, -105, -60)),
        ]
        
        for step_func, expected_angles in steps:
            sequence_manager.hexapod.forward_kinematics.reset_mock()
            step_func()
            sequence_manager.hexapod.forward_kinematics.assert_called_with(*expected_angles)

    def test_initialization_sequence_playtime_progression(self, sequence_manager):
        """Test that initialization sequence uses correct playtime progression."""
        steps_and_playtimes = [
            (sequence_manager._initialization_step1, 900),
            (sequence_manager._initialization_step2, 500),
            (sequence_manager._initialization_step3, 500),
            (sequence_manager._initialization_step4, 500),
            (sequence_manager._initialization_step5, 400),
        ]
        
        for step_func, expected_playtime in steps_and_playtimes:
            sequence_manager.publish_joint_position_trajectory.reset_mock()
            step_func()
            
            # Check that playtime_ms was called with expected value
            call_args = sequence_manager.publish_joint_position_trajectory.call_args
            assert 'playtime_ms' in call_args.kwargs
            assert call_args.kwargs['playtime_ms'] == expected_playtime

    def test_finalization_sequence_playtime_progression(self, sequence_manager):
        """Test that finalization sequence uses correct playtime progression."""
        steps_and_playtimes = [
            (sequence_manager._finalization_step1, 1000),
            (sequence_manager._finalization_step2, 500),
        ]
        
        for step_func, expected_playtime in steps_and_playtimes:
            sequence_manager.publish_joint_position_trajectory.reset_mock()
            step_func()
            
            # Check that playtime_ms was called with expected value
            call_args = sequence_manager.publish_joint_position_trajectory.call_args
            assert 'playtime_ms' in call_args.kwargs
            assert call_args.kwargs['playtime_ms'] == expected_playtime

    def test_error_handling_in_steps(self, sequence_manager):
        """Test error handling in sequence steps."""
        # Mock hexapod to raise an exception
        sequence_manager.hexapod.forward_kinematics.side_effect = Exception("Kinematics error")
        
        # Steps should propagate the exception
        with pytest.raises(Exception, match="Kinematics error"):
            sequence_manager._initialization_step1()
        
        with pytest.raises(Exception, match="Kinematics error"):
            sequence_manager._finalization_step1()

    def test_sequence_manager_with_real_timed_queue(self, mock_dependencies):
        """Test SequenceManager with real TimedQueue (integration test)."""
        # Don't patch TimedQueue for this test
        manager = SequenceManager(
            node=mock_dependencies['node'],
            hexapod=mock_dependencies['hexapod'],
            trajectory_publisher=mock_dependencies['trajectory_publisher'],
            event_publisher=mock_dependencies['event_publisher'],
            logger=mock_dependencies['logger']
        )
        
        # Should have a real TimedQueue instance
        assert hasattr(manager.sequence_queue, 'add')
        assert hasattr(manager.sequence_queue, 'clear')
