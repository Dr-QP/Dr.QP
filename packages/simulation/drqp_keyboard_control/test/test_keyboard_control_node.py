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

from drqp_brain.joystick_input_handler import ControlMode
from drqp_keyboard_control.keyboard_control_node import KeyboardControlNode
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import std_msgs.msg


@pytest.fixture
def ros_context():
    """Initialize ROS only for tests that construct a node."""
    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True

    yield

    if did_init and rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    """Provide a KeyboardControlNode and destroy it after the test."""
    node = KeyboardControlNode()
    yield node
    node.destroy_node()


def test_space_and_escape_publish_kill_switch_once_per_keydown(node):
    """Space and Esc should publish kill switch events as actions."""
    node.robot_event_pub.publish = Mock()

    assert node.handle_key_down('space') is True
    assert node.handle_key_down('esc') is True
    assert node.handle_key_up('space') is False
    assert node.handle_key_up('esc') is False

    event_names = [call.args[0].data for call in node.robot_event_pub.publish.call_args_list]
    assert event_names == ['kill_switch_pressed', 'kill_switch_pressed']


def test_delete_and_backspace_publish_robot_events(node):
    """Delete and Backspace should publish matching robot events."""
    node.robot_event_pub.publish = Mock()

    assert node.handle_key_down('delete') is True
    assert node.handle_key_down('backspace') is True

    event_names = [call.args[0].data for call in node.robot_event_pub.publish.call_args_list]
    assert event_names == ['reboot_servos', 'finalize']


def test_b_key_toggles_balance_mode_and_publishes(node):
    """B should toggle balance mode and publish the latched state each time."""
    node.balance_mode_pub.publish = Mock()

    assert node.handle_key_down('b') is True
    assert node.balance_mode_enabled is True
    assert node.handle_key_down('b') is True
    assert node.balance_mode_enabled is False

    published = [call.args[0].data for call in node.balance_mode_pub.publish.call_args_list]
    assert published == [True, False]


def test_motion_keys_are_delegated_to_state(node):
    """Plain motion keys should flow into the shared control state."""
    assert node.handle_key_down('w') is True
    assert node.state.axes().left_y == pytest.approx(0.5)

    assert node.handle_key_up('w') is True
    assert node.state.axes().left_y == pytest.approx(0.0)


def test_balance_mode_publishes_initial_false_for_late_joiners(ros_context):
    """Startup should publish a latched False so late subscribers see a defined state."""
    node = KeyboardControlNode()
    consumer = rclpy.create_node('balance_mode_consumer')
    received = []
    latched_qos = QoSProfile(depth=1)
    latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    consumer.create_subscription(
        std_msgs.msg.Bool,
        '/robot/balance_mode',
        lambda msg: received.append(msg.data),
        qos_profile=latched_qos,
    )
    try:
        for _ in range(10):
            rclpy.spin_once(consumer, timeout_sec=0.1)
            if received:
                break
    finally:
        node.destroy_node()
        consumer.destroy_node()

    assert received == [False]


def test_publish_stop_command_clears_motion_inputs_before_publishing(node):
    """GUI shutdown should send one final zero movement command."""
    node.movement_command_pub.publish = Mock()
    node.state.key_down('w')
    node.state.set_left_stick(0.5, 0.5)
    node.state.set_left_trigger(0.75)
    node.state.set_control_mode(ControlMode.BodyPosition)
    node.state.set_right_stick(-0.25, 0.6)
    node.state.release_right_stick()

    node.publish_stop_command()

    command = node.movement_command_pub.publish.call_args.args[0]
    assert command.stride_direction.x == pytest.approx(0.0)
    assert command.stride_direction.y == pytest.approx(0.0)
    assert command.stride_direction.z == pytest.approx(0.0)
    assert command.rotation_speed == pytest.approx(0.0)
    assert command.body_translation.x == pytest.approx(0.0)
    assert command.body_translation.z == pytest.approx(0.0)


def test_latched_body_pose_is_published_across_modes(node):
    """The published command must carry latched body pose in Walk mode."""
    node.state.set_control_mode(ControlMode.BodyPosition)
    node.state.set_left_stick(0.2, 0.4)
    node.state.release_left_stick()
    node.state.set_control_mode(ControlMode.Walk)

    command = node.state.movement_command()

    assert command.body_translation.x == pytest.approx(0.4)
    assert command.body_translation.y == pytest.approx(-0.2)
