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

from drqp_brain.joystick_translator_node import JoystickTranslatorNode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants, RobotCommand
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
        self.robot_commands = []
        self.robot_events = []

        # Subscribe to translator output
        self.movement_sub = self.test_node.create_subscription(
            MovementCommand,
            '/robot/movement_command',
            lambda msg: self.movement_commands.append(msg),
            10,
        )

        self.command_sub = self.test_node.create_subscription(
            RobotCommand,
            '/robot/command',
            lambda msg: self.robot_commands.append(msg),
            10,
        )

        self.event_sub = self.test_node.create_subscription(
            std_msgs.msg.String,
            '/robot_event',
            lambda msg: self.robot_events.append(msg),
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
        # Create a joystick message with Select button pressed (finalize event)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.buttons = [0] * 21
        joy_msg.buttons[4] = 1  # Select button (index 4)

        # Process the message
        self.node._joy_callback(joy_msg)

        # Spin to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify robot event was published
        self.assertGreater(len(self.robot_events), 0, 'Robot event should be published')

        event = self.robot_events[-1]
        self.assertEqual(event.data, 'finalize')


if __name__ == '__main__':
    unittest.main()
