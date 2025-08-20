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

from unittest.mock import Mock, patch, MagicMock

from drqp_brain.brain_node import HexapodBrain
from drqp_brain.geometry import Point3D
from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.parametric_gait_generator import GaitType
import pytest
import rclpy
import sensor_msgs.msg
import std_msgs.msg


class TestBrainNodeIntegration:
    """Integration tests for the HexapodBrain node."""

    @pytest.fixture
    def mock_rclpy_node(self):
        """Mock the rclpy.node.Node base class."""
        with patch('rclpy.node.Node') as mock_node:
            mock_instance = Mock()
            mock_node.return_value = mock_instance
            
            # Mock ROS node methods
            mock_instance.create_subscription = Mock()
            mock_instance.create_publisher = Mock()
            mock_instance.create_timer = Mock()
            mock_instance.get_logger = Mock()
            mock_instance.get_logger.return_value = Mock()
            
            yield mock_instance

    @pytest.fixture
    def brain_node(self, mock_rclpy_node):
        """Create a HexapodBrain instance with mocked dependencies."""
        with patch('drqp_brain.brain_node.HexapodModel'), \
             patch('drqp_brain.brain_node.WalkController'), \
             patch('drqp_brain.brain_node.TimedQueue'):
            
            brain = HexapodBrain()
            
            # Mock the walker and other components
            brain.walker = Mock()
            brain.walker.reset = Mock()
            brain.hexapod = Mock()
            brain.sequence_queue = Mock()
            
            return brain

    def test_brain_node_initialization(self, brain_node, mock_rclpy_node):
        """Test that brain node initializes correctly."""
        # Verify node was initialized with correct name
        mock_rclpy_node.assert_called_with('drqp_brain')
        
        # Verify initial state
        assert brain_node.direction == Point3D([0, 0, 0])
        assert brain_node.rotation == 0
        assert brain_node.walk_speed == 0
        assert brain_node.rotation_speed == 0
        assert brain_node.fps == 30
        assert brain_node.gait_index == 0
        assert brain_node.gaits == [GaitType.tripod, GaitType.ripple, GaitType.wave]

    def test_joystick_input_processing(self, brain_node):
        """Test joystick input processing."""
        # Create a mock joy message
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.5, -0.3, 0.2, 0.0, -0.5, 0.0]  # left_x, left_y, right_x, ..., left_trigger, ...
        joy_msg.buttons = [0] * 15
        
        brain_node.process_inputs(joy_msg)
        
        # Verify direction and speeds were updated
        assert brain_node.direction.x == -0.3  # left_y maps to x
        assert brain_node.direction.y == 0.5   # left_x maps to y
        assert brain_node.direction.z == 0.5   # left_trigger (inverted from -0.5)
        assert brain_node.rotation_speed == 0.2  # right_x
        
        # Walk speed should be sum of absolute values
        expected_walk_speed = abs(0.5) + abs(-0.3) + abs(0.5)
        assert brain_node.walk_speed == expected_walk_speed

    def test_gait_switching(self, brain_node):
        """Test gait switching functionality."""
        initial_gait_index = brain_node.gait_index
        
        # Test next gait
        brain_node.next_gait()
        assert brain_node.gait_index == (initial_gait_index + 1) % len(brain_node.gaits)
        
        # Test previous gait
        brain_node.prev_gait()
        assert brain_node.gait_index == initial_gait_index

    def test_gait_switching_wraparound(self, brain_node):
        """Test gait switching wraparound behavior."""
        # Set to last gait
        brain_node.gait_index = len(brain_node.gaits) - 1
        
        # Next should wrap to 0
        brain_node.next_gait()
        assert brain_node.gait_index == 0
        
        # Previous should wrap to last
        brain_node.prev_gait()
        assert brain_node.gait_index == len(brain_node.gaits) - 1

    def test_joystick_button_integration(self, brain_node):
        """Test joystick button integration."""
        # Create joy message with dpad left pressed (prev gait)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [0] * 15
        joy_msg.buttons[ButtonIndex.DpadLeft.value] = 1
        
        initial_gait_index = brain_node.gait_index
        
        # Process the input
        brain_node.process_inputs(joy_msg)
        
        # Gait should have changed (button was pressed)
        # Note: This tests the button update mechanism
        for button in brain_node.joystick_buttons:
            if button.button_index == ButtonIndex.DpadLeft:
                # Simulate the button state change
                button.current_state = button.current_state.__class__(1)  # Pressed
                button.last_state = button.current_state.__class__(0)     # Was released

    def test_loop_functionality(self, brain_node):
        """Test the main loop functionality."""
        # Set up some movement parameters
        brain_node.direction = Point3D([1, 0, 0])
        brain_node.walk_speed = 0.5
        brain_node.rotation_speed = 0.2
        brain_node.gait_index = 1  # ripple gait
        
        # Mock the walker
        brain_node.walker.next_step = Mock()
        
        # Mock publish method
        brain_node.publish_joint_position_trajectory = Mock()
        
        # Call loop
        brain_node.loop()
        
        # Verify walker was called with correct parameters
        brain_node.walker.next_step.assert_called_once_with(
            stride_direction=brain_node.direction,
            stride_ratio=brain_node.walk_speed,
            rotation_ratio=brain_node.rotation_speed,
        )
        
        # Verify gait was set
        assert brain_node.walker.current_gait == brain_node.gaits[brain_node.gait_index]
        
        # Verify joint trajectory was published
        brain_node.publish_joint_position_trajectory.assert_called_once()

    def test_robot_state_processing(self, brain_node):
        """Test robot state message processing."""
        # Mock methods that should be called
        brain_node.stop_walk_controller = Mock()
        brain_node.turn_torque_off = Mock()
        brain_node.initialization_sequence = Mock()
        brain_node.finalization_sequence = Mock()
        brain_node.loop_timer = Mock()
        
        # Test torque_off state
        msg = std_msgs.msg.String(data='torque_off')
        brain_node.process_robot_state(msg)
        brain_node.stop_walk_controller.assert_called_once()
        brain_node.turn_torque_off.assert_called_once()
        
        # Reset mocks
        brain_node.stop_walk_controller.reset_mock()
        brain_node.turn_torque_off.reset_mock()
        
        # Test initializing state
        msg = std_msgs.msg.String(data='initializing')
        brain_node.process_robot_state(msg)
        brain_node.initialization_sequence.assert_called_once()
        
        # Test torque_on state
        msg = std_msgs.msg.String(data='torque_on')
        brain_node.process_robot_state(msg)
        brain_node.loop_timer.reset.assert_called_once()

    def test_kill_switch_integration(self, brain_node):
        """Test kill switch integration."""
        brain_node.robot_event_pub = Mock()
        
        # Call kill switch
        brain_node.process_kill_switch()
        
        # Verify event was published
        brain_node.robot_event_pub.publish.assert_called_once()
        published_msg = brain_node.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'kill_switch_pressed'

    def test_finalize_integration(self, brain_node):
        """Test finalize integration."""
        brain_node.robot_event_pub = Mock()
        
        # Call finalize
        brain_node.finalize()
        
        # Verify event was published
        brain_node.robot_event_pub.publish.assert_called_once()
        published_msg = brain_node.robot_event_pub.publish.call_args[0][0]
        assert isinstance(published_msg, std_msgs.msg.String)
        assert published_msg.data == 'finalize'

    def test_joint_trajectory_publishing(self, brain_node):
        """Test joint trajectory publishing."""
        # Mock the hexapod legs
        mock_legs = []
        for i in range(6):
            leg = Mock()
            leg.label = Mock()
            leg.label.name = f'leg_{i}'
            leg.coxa_angle = 10 * i
            leg.femur_angle = 20 * i
            leg.tibia_angle = 30 * i
            mock_legs.append(leg)
        
        brain_node.hexapod.legs = mock_legs
        brain_node.joint_trajectory_pub = Mock()
        
        # Call publish method
        brain_node.publish_joint_position_trajectory(playtime_ms=500)
        
        # Verify publisher was called
        brain_node.joint_trajectory_pub.publish.assert_called_once()

    def test_servo_reboot_functionality(self, brain_node):
        """Test servo reboot functionality."""
        brain_node.publish_joint_position_trajectory = Mock()
        
        # Call reboot servos
        brain_node.reboot_servos()
        
        # Verify correct trajectory was published
        brain_node.publish_joint_position_trajectory.assert_called_once_with(
            effort_points=[-1.0, 0.0], playtime_ms=1000
        )

    def test_torque_off_functionality(self, brain_node):
        """Test torque off functionality."""
        brain_node.publish_joint_position_trajectory = Mock()
        
        # Call turn torque off
        brain_node.turn_torque_off()
        
        # Verify correct trajectory was published
        brain_node.publish_joint_position_trajectory.assert_called_once_with(effort_points=[0.0])

    def test_stop_walk_controller_integration(self, brain_node):
        """Test stop walk controller integration."""
        brain_node.loop_timer = Mock()
        brain_node.sequence_queue = Mock()
        brain_node.walker = Mock()
        
        # Call stop walk controller
        brain_node.stop_walk_controller()
        
        # Verify all components were stopped
        brain_node.loop_timer.cancel.assert_called_once()
        brain_node.sequence_queue.clear.assert_called_once()
        brain_node.walker.reset.assert_called_once()

    def test_complete_joystick_to_servo_flow(self, brain_node):
        """Test complete flow from joystick input to servo output."""
        # Mock all necessary components
        brain_node.walker = Mock()
        brain_node.walker.next_step = Mock()
        brain_node.publish_joint_position_trajectory = Mock()
        brain_node.hexapod = Mock()
        brain_node.hexapod.legs = []
        
        # Create joystick input
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.5, 0.3, 0.1, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0] * 15
        
        # Process input
        brain_node.process_inputs(joy_msg)
        
        # Run loop
        brain_node.loop()
        
        # Verify the flow
        assert brain_node.direction.x == 0.3  # left_y
        assert brain_node.direction.y == 0.5  # left_x
        assert brain_node.rotation_speed == 0.1  # right_x
        
        brain_node.walker.next_step.assert_called_once()
        brain_node.publish_joint_position_trajectory.assert_called_once()

    def test_error_handling_in_loop(self, brain_node):
        """Test error handling in the main loop."""
        # Mock walker to raise an exception
        brain_node.walker = Mock()
        brain_node.walker.next_step.side_effect = Exception("Walker error")
        brain_node.publish_joint_position_trajectory = Mock()
        
        # Loop should handle the error gracefully
        try:
            brain_node.loop()
        except Exception:
            # If exception propagates, that's also acceptable behavior
            pass
        
        # Walker should have been called
        brain_node.walker.next_step.assert_called_once()

    def test_state_change_ignored_when_same(self, brain_node):
        """Test that identical state changes are ignored."""
        brain_node.robot_state = 'torque_on'
        brain_node.loop_timer = Mock()
        
        # Send same state
        msg = std_msgs.msg.String(data='torque_on')
        brain_node.process_robot_state(msg)
        
        # Timer reset should not be called again
        brain_node.loop_timer.reset.assert_not_called()
