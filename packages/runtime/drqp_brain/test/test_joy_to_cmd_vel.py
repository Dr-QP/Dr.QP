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

from unittest.mock import MagicMock

from drqp_brain.joy_to_cmd_vel import JoyToCmdVel
from geometry_msgs.msg import Twist
import pytest
import rclpy
import sensor_msgs.msg


class TestJoyToCmdVel:
    """Test the JoyToCmdVel node."""

    @pytest.fixture
    def node(self):
        """Create a JoyToCmdVel node for testing."""
        rclpy.init()
        node = JoyToCmdVel()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self, node):
        """Test that JoyToCmdVel initializes correctly."""
        assert node.joy_sub is not None
        assert node.cmd_vel_pub is not None

    def test_joy_to_cmd_vel_forward(self, node):
        """Test conversion of forward joystick input to cmd_vel."""
        # Mock the publisher
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        node.cmd_vel_pub.publish = mock_publish

        # Create joy message with forward movement (left stick up)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0]  # left_x=0, left_y=0.8, ...
        joy_msg.buttons = []

        node.joy_callback(joy_msg)

        assert len(published_msgs) == 1
        twist = published_msgs[0]
        assert isinstance(twist, Twist)
        assert twist.linear.x == pytest.approx(0.8)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)

    def test_joy_to_cmd_vel_strafe(self, node):
        """Test conversion of strafe joystick input to cmd_vel."""
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        node.cmd_vel_pub.publish = mock_publish

        # Create joy message with left movement (left stick left)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [-0.6, 0.0, 0.0, 0.0, 0.0, 0.0]  # left_x=-0.6
        joy_msg.buttons = []

        node.joy_callback(joy_msg)

        assert len(published_msgs) == 1
        twist = published_msgs[0]
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.linear.y == pytest.approx(-0.6)
        assert twist.angular.z == pytest.approx(0.0)

    def test_joy_to_cmd_vel_rotation(self, node):
        """Test conversion of rotation joystick input to cmd_vel."""
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        node.cmd_vel_pub.publish = mock_publish

        # Create joy message with rotation (right stick)
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # right_x=0.5
        joy_msg.buttons = []

        node.joy_callback(joy_msg)

        assert len(published_msgs) == 1
        twist = published_msgs[0]
        assert twist.angular.z == pytest.approx(0.5)

    def test_joy_to_cmd_vel_trigger(self, node):
        """Test conversion of trigger input to cmd_vel vertical movement."""
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        node.cmd_vel_pub.publish = mock_publish

        # Create joy message with left trigger pressed
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, -0.5, 0.0]  # left_trigger=-0.5
        joy_msg.buttons = []

        node.joy_callback(joy_msg)

        assert len(published_msgs) == 1
        twist = published_msgs[0]
        # Trigger should be interpolated from [-1, 0] to [1, 0]
        # -0.5 is halfway between -1 and 0, so it maps to 0.5
        assert twist.linear.z == pytest.approx(0.5)

    def test_joy_to_cmd_vel_combined(self, node):
        """Test conversion of combined joystick movements to cmd_vel."""
        published_msgs = []

        def mock_publish(msg):
            published_msgs.append(msg)

        node.cmd_vel_pub.publish = mock_publish

        # Create joy message with combined movement
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [0.3, 0.4, 0.2, 0.0, -0.8, 0.0]  # left_x, left_y, right_x, ..., left_trigger
        joy_msg.buttons = []

        node.joy_callback(joy_msg)

        assert len(published_msgs) == 1
        twist = published_msgs[0]
        assert twist.linear.x == pytest.approx(0.4)
        assert twist.linear.y == pytest.approx(0.3)
        assert twist.angular.z == pytest.approx(0.2)
