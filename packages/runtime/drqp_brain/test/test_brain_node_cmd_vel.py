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

from drqp_brain.brain_node import HexapodBrain
from drqp_brain.geometry import Point3D
from drqp_brain.joystick_input_handler import ControlMode
from geometry_msgs.msg import Twist
import pytest
import rclpy


class TestBrainNodeCmdVel:
    """Test the brain_node's cmd_vel processing."""

    @pytest.fixture
    def node(self):
        """Create a HexapodBrain node for testing."""
        rclpy.init()
        node = HexapodBrain()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_cmd_vel_walk_mode(self, node):
        """Test processing cmd_vel in Walk mode."""
        node.control_mode = ControlMode.Walk

        twist = Twist()
        twist.linear.x = 0.5
        twist.linear.y = 0.3
        twist.linear.z = 0.2
        twist.angular.z = 0.4

        node.process_cmd_vel(twist)

        assert node.direction.x == pytest.approx(0.5)
        assert node.direction.y == pytest.approx(0.3)
        assert node.direction.z == pytest.approx(0.2)
        assert node.rotation_speed == pytest.approx(0.4)

    def test_cmd_vel_body_position_mode(self, node):
        """Test processing cmd_vel in BodyPosition mode."""
        node.control_mode = ControlMode.BodyPosition

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.2
        twist.linear.z = 0.3

        node.process_cmd_vel(twist)

        assert node.body_translation.x == pytest.approx(0.1)
        assert node.body_translation.y == pytest.approx(0.2)
        assert node.body_translation.z == pytest.approx(0.3)

    def test_cmd_vel_body_rotation_mode(self, node):
        """Test processing cmd_vel in BodyRotation mode."""
        node.control_mode = ControlMode.BodyRotation

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.2
        twist.angular.z = 0.3

        node.process_cmd_vel(twist)

        assert node.body_rotation.x == pytest.approx(0.1)
        assert node.body_rotation.y == pytest.approx(0.2)
        assert node.body_rotation.z == pytest.approx(0.3)

    def test_control_mode_switching(self, node):
        """Test switching between control modes."""
        # Start in Walk mode
        assert node.control_mode == ControlMode.Walk

        # Switch to BodyPosition
        node.next_control_mode()
        assert node.control_mode == ControlMode.BodyPosition

        # Switch to BodyRotation
        node.next_control_mode()
        assert node.control_mode == ControlMode.BodyRotation

        # Switch back to Walk
        node.next_control_mode()
        assert node.control_mode == ControlMode.Walk

    def test_initialization_state(self, node):
        """Test that movement state is initialized to zero."""
        assert node.direction == Point3D([0, 0, 0])
        assert node.rotation_speed == pytest.approx(0.0)
        assert node.body_translation == Point3D([0, 0, 0])
        assert node.body_rotation == Point3D([0, 0, 0])
        assert node.control_mode == ControlMode.Walk
