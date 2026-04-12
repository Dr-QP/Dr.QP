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

import unittest
from unittest.mock import Mock

from drqp_brain.joystick_translator_node import JoystickTranslatorNode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
import rclpy
import sensor_msgs.msg
import std_msgs.msg


class TestJoystickTranslatorNode(unittest.TestCase):
    """Test the joystick translator node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = JoystickTranslatorNode()
        self.test_node = rclpy.create_node('test_translator_consumer')

        # Store received messages
        self.movement_commands = []
        self.robot_events = []
        self.joy_feedback_messages = []

        # Subscribe to translator output
        self.movement_sub = self.test_node.create_subscription(
            MovementCommand,
            '/robot/movement_command',
            lambda msg: self.movement_commands.append(msg),
            10,
        )

        self.event_sub = self.test_node.create_subscription(
            std_msgs.msg.String,
            '/robot_event',
            lambda msg: self.robot_events.append(msg),
            10,
        )

        self.joy_feedback_sub = self.test_node.create_subscription(
            sensor_msgs.msg.JoyFeedback,
            '/joy/set_feedback',
            lambda msg: self.joy_feedback_messages.append(msg),
            10,
        )

    def tearDown(self):
        self.node.destroy_node()
        self.test_node.destroy_node()

    def test_joystick_to_movement_command(self):
        """Test that joystick messages are translated to movement commands."""
        # Create a joystick message with movement input
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]  # axes
        joy_msg.buttons = [0] * 21  # All buttons released

        # Process the message
        self.node._joy_callback(joy_msg)

        # Spin to process callbacks
        rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify movement command was published
        self.assertGreater(len(self.movement_commands), 0, 'Movement command should be published')

        cmd = self.movement_commands[-1]
        # Check that stride direction reflects joystick input
        self.assertIsNotNone(cmd.stride_direction)
        self.assertEqual(cmd.gait_type, MovementCommandConstants.GAIT_TRIPOD)  # Default gait

    def test_button_to_robot_event(self):
        """Test that button presses generate robot events for state machine."""
        # Wait for publisher/subscriber connection to be established
        # This is necessary because DDS discovery in ROS 2 is asynchronous
        max_wait_iterations = 10
        for _ in range(max_wait_iterations):
            if self.node.robot_event_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Create a joystick message with Select button pressed (finalize event)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [0] * 21
        joy_msg.buttons[4] = 1  # Select button (index 4)

        # Process the message
        self.node._joy_callback(joy_msg)

        # Spin multiple times to allow message propagation through DDS
        # ROS 2 publishing is asynchronous and may require multiple event loop iterations
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if len(self.robot_events) > 0:
                break

        # Verify robot event was published
        self.assertGreater(len(self.robot_events), 0, 'Robot event should be published')

        event = self.robot_events[-1]
        self.assertEqual(event.data, 'finalize')

    def test_dispatch_pending_feedback_publishes_without_subscriber_check(self):
        """Due haptic commands should be published without gating on discovery."""
        self.node.joy_feedback_pub.publish = Mock()
        self.node._pending_feedback_commands = [
            Mock(due_at=0.0, channel_id=0, intensity=0.8)
        ]
        self.node.haptic_feedback_scheduler.now = Mock(return_value=0.0)

        self.node._dispatch_pending_feedback()

        self.node.joy_feedback_pub.publish.assert_called_once()
        self.assertEqual(self.node._pending_feedback_commands, [])

    def test_haptic_feedback_is_published_on_joy_set_feedback(self):
        """Gait changes should publish JoyFeedback messages on /joy/set_feedback."""
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [0] * 21
        joy_msg.buttons[14] = 1

        self.node._joy_callback(joy_msg)

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if self.joy_feedback_messages:
                break

        self.assertGreater(
            len(self.joy_feedback_messages),
            0,
            'Joy feedback should be published',
        )
        feedback = self.joy_feedback_messages[0]
        self.assertEqual(
            feedback.type,
            sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE,
        )
        self.assertEqual(feedback.id, 0)
        self.assertGreater(feedback.intensity, 0.0)

    def test_control_mode_haptic_feedback_uses_working_rumble_channel(self):
        """Control-mode changes should publish on the same rumble channel."""
        self.node._publish_control_mode_change()

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if self.joy_feedback_messages:
                break

        self.assertGreater(
            len(self.joy_feedback_messages),
            0,
            'Control-mode joy feedback should be published',
        )
        feedback = self.joy_feedback_messages[0]
        self.assertEqual(
            feedback.type,
            sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE,
        )
        self.assertEqual(feedback.id, 0)
        self.assertGreater(feedback.intensity, 0.0)


if __name__ == '__main__':
    unittest.main()
