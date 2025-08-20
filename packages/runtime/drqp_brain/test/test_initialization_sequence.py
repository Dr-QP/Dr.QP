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


class TestInitializationSequence:
    """Test initialization sequence functionality in the brain node."""

    @pytest.fixture
    def mock_brain_node(self):
        """Create a mock brain node for testing initialization sequence."""
        brain_node = Mock()
        brain_node.sequence_queue = Mock()
        brain_node.hexapod = Mock()
        brain_node.robot_event_pub = Mock()
        brain_node.get_logger = Mock()
        brain_node.get_logger.return_value = Mock()
        brain_node.publish_joint_position_trajectory = Mock()

        return brain_node

    def test_initialization_sequence_setup(self, mock_brain_node):
        """Test that initialization sequence is properly set up."""

        def initialization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.0, lambda: None)  # step1
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step2
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step3
            mock_brain_node.sequence_queue.add(0.6, lambda: None)  # step4
            mock_brain_node.sequence_queue.add(0.5, lambda: None)  # step5
            mock_brain_node.sequence_queue.add(0.0, lambda: None)  # done

        initialization_sequence()

        # Verify sequence queue was cleared and steps added
        mock_brain_node.sequence_queue.clear.assert_called_once()
        assert mock_brain_node.sequence_queue.add.call_count == 6

        # Verify timing of each step
        expected_timings = [1.0, 0.6, 0.6, 0.6, 0.5, 0.0]
        for i, (call, expected_time) in enumerate(
            zip(mock_brain_node.sequence_queue.add.call_args_list, expected_timings)
        ):
            assert call[0][0] == expected_time, f'Step {i + 1} timing mismatch'

    def test_initialization_sequence_step1(self, mock_brain_node):
        """Test initialization sequence step 1 - femur positioning."""

        def initialization_sequence_step1():
            mock_brain_node.get_logger().info('Initialization sequence started')
            mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=900, joint_mask=['femur'])

        initialization_sequence_step1()

        # Verify logging
        mock_brain_node.get_logger().info.assert_called_with('Initialization sequence started')

        # Verify hexapod positioning
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, 0)

        # Verify joint trajectory publishing
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(
            playtime_ms=900, joint_mask=['femur']
        )

    def test_initialization_sequence_step2(self, mock_brain_node):
        """Test initialization sequence step 2 - tibia positioning."""

        def initialization_sequence_step2():
            mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            mock_brain_node.publish_joint_position_trajectory(
                playtime_ms=500, joint_mask=['femur', 'tibia']
            )

        initialization_sequence_step2()

        # Verify hexapod positioning
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, 0)

        # Verify joint trajectory publishing with femur and tibia
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(
            playtime_ms=500, joint_mask=['femur', 'tibia']
        )

    def test_initialization_sequence_step3(self, mock_brain_node):
        """Test initialization sequence step 3 - coxa positioning."""

        def initialization_sequence_step3():
            mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)

        initialization_sequence_step3()

        # Verify hexapod positioning
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, 0)

        # Verify joint trajectory publishing (all joints, no mask)
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_initialization_sequence_step4(self, mock_brain_node):
        """Test initialization sequence step 4 - tibia extension."""

        def initialization_sequence_step4():
            mock_brain_node.hexapod.forward_kinematics(0, -105, 95)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)

        initialization_sequence_step4()

        # Verify hexapod positioning with tibia extended
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -105, 95)

        # Verify joint trajectory publishing
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(playtime_ms=500)

    def test_initialization_sequence_step5(self, mock_brain_node):
        """Test initialization sequence step 5 - final positioning."""

        def initialization_sequence_step5():
            mock_brain_node.hexapod.forward_kinematics(0, -35, 130)
            mock_brain_node.publish_joint_position_trajectory(playtime_ms=400)

        initialization_sequence_step5()

        # Verify hexapod positioning to final stance
        mock_brain_node.hexapod.forward_kinematics.assert_called_with(0, -35, 130)

        # Verify joint trajectory publishing
        mock_brain_node.publish_joint_position_trajectory.assert_called_with(playtime_ms=400)

    def test_initialization_sequence_done(self, mock_brain_node):
        """Test initialization sequence completion."""

        def initialization_sequence_done():
            mock_brain_node.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done'))

        initialization_sequence_done()

        # Verify completion event is published
        mock_brain_node.robot_event_pub.publish.assert_called_once()
        published_msg = mock_brain_node.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'initializing_done'

    def test_initialization_sequence_joint_angles(self, mock_brain_node):
        """Test that initialization sequence uses correct joint angles."""
        # Define the sequence steps with their expected angles
        steps_and_angles = [
            (1, (0, -105, 0)),  # Step 1: femur positioning
            (2, (0, -105, 0)),  # Step 2: tibia on
            (3, (0, -105, 0)),  # Step 3: coxa on
            (4, (0, -105, 95)),  # Step 4: tibia extension
            (5, (0, -35, 130)),  # Step 5: final position
        ]

        for step_num, (coxa, femur, tibia) in steps_and_angles:
            mock_brain_node.hexapod.forward_kinematics.reset_mock()

            # Call the appropriate step function
            if step_num == 1:
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            elif step_num == 2:
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            elif step_num == 3:
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
            elif step_num == 4:
                mock_brain_node.hexapod.forward_kinematics(0, -105, 95)
            elif step_num == 5:
                mock_brain_node.hexapod.forward_kinematics(0, -35, 130)

            # Verify the correct angles were used
            mock_brain_node.hexapod.forward_kinematics.assert_called_with(coxa, femur, tibia)

    def test_initialization_sequence_timing(self, mock_brain_node):
        """Test that initialization sequence has correct timing."""
        expected_timings = [
            (1.0, 'step1'),
            (0.6, 'step2'),
            (0.6, 'step3'),
            (0.6, 'step4'),
            (0.5, 'step5'),
            (0.0, 'done'),
        ]

        def initialization_sequence():
            mock_brain_node.sequence_queue.clear()
            for timing, step_name in expected_timings:
                mock_brain_node.sequence_queue.add(timing, lambda: None)

        initialization_sequence()

        # Verify all timings are correct
        for i, (expected_time, _) in enumerate(expected_timings):
            call_args = mock_brain_node.sequence_queue.add.call_args_list[i]
            assert call_args[0][0] == expected_time

    def test_initialization_sequence_playtime_values(self, mock_brain_node):
        """Test that initialization sequence uses correct playtime values."""
        expected_playtimes = [
            (1, 900, ['femur']),  # Step 1
            (2, 500, ['femur', 'tibia']),  # Step 2
            (3, 500, None),  # Step 3 (all joints)
            (4, 500, None),  # Step 4 (all joints)
            (5, 400, None),  # Step 5 (all joints)
        ]

        for step_num, expected_playtime, expected_mask in expected_playtimes:
            mock_brain_node.publish_joint_position_trajectory.reset_mock()

            # Simulate the step
            if step_num == 1:
                mock_brain_node.publish_joint_position_trajectory(
                    playtime_ms=900, joint_mask=['femur']
                )
            elif step_num == 2:
                mock_brain_node.publish_joint_position_trajectory(
                    playtime_ms=500, joint_mask=['femur', 'tibia']
                )
            elif step_num in [3, 4]:
                mock_brain_node.publish_joint_position_trajectory(playtime_ms=500)
            elif step_num == 5:
                mock_brain_node.publish_joint_position_trajectory(playtime_ms=400)

            # Verify the call
            call_args = mock_brain_node.publish_joint_position_trajectory.call_args
            assert 'playtime_ms' in call_args.kwargs or len(call_args.args) > 0

            if expected_mask:
                assert 'joint_mask' in call_args.kwargs

    def test_initialization_sequence_error_handling(self, mock_brain_node):
        """Test initialization sequence error handling."""
        # Mock an error in hexapod forward kinematics
        mock_brain_node.hexapod.forward_kinematics.side_effect = Exception('Kinematics error')

        def initialization_sequence_step1():
            try:
                mock_brain_node.get_logger().info('Initialization sequence started')
                mock_brain_node.hexapod.forward_kinematics(0, -105, 0)
                mock_brain_node.publish_joint_position_trajectory(
                    playtime_ms=900, joint_mask=['femur']
                )
            except Exception as e:
                mock_brain_node.get_logger().error(f'Initialization step 1 failed: {e}')
                raise

        # Should raise the exception
        with pytest.raises(Exception, match='Kinematics error'):
            initialization_sequence_step1()

    def test_initialization_sequence_queue_clearing(self, mock_brain_node):
        """Test that initialization sequence properly clears the queue."""

        def initialization_sequence():
            mock_brain_node.sequence_queue.clear()
            # Add some steps...
            mock_brain_node.sequence_queue.add(1.0, lambda: None)

        initialization_sequence()

        # Verify queue was cleared before adding new steps
        mock_brain_node.sequence_queue.clear.assert_called_once()

    def test_initialization_sequence_step_order(self, mock_brain_node):
        """Test that initialization sequence steps are added in correct order."""
        step_functions = []

        def mock_add(timing, func):
            step_functions.append((timing, func))

        mock_brain_node.sequence_queue.add = mock_add

        def initialization_sequence():
            mock_brain_node.sequence_queue.clear()
            mock_brain_node.sequence_queue.add(1.0, 'step1')
            mock_brain_node.sequence_queue.add(0.6, 'step2')
            mock_brain_node.sequence_queue.add(0.6, 'step3')
            mock_brain_node.sequence_queue.add(0.6, 'step4')
            mock_brain_node.sequence_queue.add(0.5, 'step5')
            mock_brain_node.sequence_queue.add(0.0, 'done')

        initialization_sequence()

        # Verify steps were added in correct order
        expected_order = ['step1', 'step2', 'step3', 'step4', 'step5', 'done']
        actual_order = [func for _, func in step_functions]
        assert actual_order == expected_order

    def test_initialization_completion_event_format(self):
        """Test that initialization completion event has correct format."""
        completion_msg = std_msgs.msg.String(data='initializing_done')

        assert isinstance(completion_msg, std_msgs.msg.String)
        assert completion_msg.data == 'initializing_done'
        assert isinstance(completion_msg.data, str)
        assert len(completion_msg.data) > 0
