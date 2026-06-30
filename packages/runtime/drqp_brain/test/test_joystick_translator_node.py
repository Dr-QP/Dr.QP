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

from dataclasses import dataclass, field
from unittest.mock import Mock

from drqp_brain.haptics import (
    HapticFeedbackScheduler,
    LEFT_RUMBLE_CHANNEL_ID,
    RIGHT_RUMBLE_CHANNEL_ID,
)
from drqp_brain.joystick_translator_node import JoystickTranslatorNode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
import pytest
import rclpy
import sensor_msgs.msg
import std_msgs.msg


@dataclass
class _TranslatorHarness:
    """Bundle the node under test, a consumer node, and captured messages."""

    node: JoystickTranslatorNode
    test_node: 'rclpy.node.Node'
    movement_commands: list = field(default_factory=list)
    robot_events: list = field(default_factory=list)
    joy_feedback_messages: list = field(default_factory=list)


@pytest.fixture
def translator(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Provide a joystick translator node wired to a consumer node."""
    node = JoystickTranslatorNode()
    test_node = rclpy.create_node('test_translator_consumer')
    harness = _TranslatorHarness(node=node, test_node=test_node)

    test_node.create_subscription(
        MovementCommand,
        '/robot/movement_command',
        lambda msg: harness.movement_commands.append(msg),
        10,
    )
    test_node.create_subscription(
        std_msgs.msg.String,
        '/robot_event',
        lambda msg: harness.robot_events.append(msg),
        10,
    )
    test_node.create_subscription(
        sensor_msgs.msg.JoyFeedback,
        '/joy/set_feedback',
        lambda msg: harness.joy_feedback_messages.append(msg),
        10,
    )

    try:
        yield harness
    finally:
        test_node.destroy_node()
        node.destroy_node()


def test_joystick_to_movement_command(translator):
    """Test that joystick messages are translated to movement commands."""
    # Create a joystick message with movement input
    joy_msg = sensor_msgs.msg.Joy()
    joy_msg.axes = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]  # axes
    joy_msg.buttons = [0] * 21  # All buttons released

    # Process the message
    translator.node._joy_callback(joy_msg)

    # Spin to process callbacks
    rclpy.spin_once(translator.test_node, timeout_sec=0.1)

    # Verify movement command was published
    assert len(translator.movement_commands) > 0, 'Movement command should be published'

    cmd = translator.movement_commands[-1]
    # Check that stride direction reflects joystick input
    assert cmd.stride_direction is not None
    assert cmd.gait_type == MovementCommandConstants.GAIT_TRIPOD  # Default gait


def test_button_to_robot_event(translator):
    """Test that button presses generate robot events for state machine."""
    # Wait for publisher/subscriber connection to be established
    # This is necessary because DDS discovery in ROS 2 is asynchronous
    max_wait_iterations = 10
    for _ in range(max_wait_iterations):
        if translator.node.robot_event_pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(translator.node, timeout_sec=0.01)
        rclpy.spin_once(translator.test_node, timeout_sec=0.01)

    # Create a joystick message with Select button pressed (finalize event)
    joy_msg = sensor_msgs.msg.Joy()
    joy_msg.axes = [0.0] * 6
    joy_msg.buttons = [0] * 21
    joy_msg.buttons[4] = 1  # Select button (index 4)

    # Process the message
    translator.node._joy_callback(joy_msg)

    # Spin multiple times to allow message propagation through DDS
    # ROS 2 publishing is asynchronous and may require multiple event loop iterations
    for _ in range(5):
        rclpy.spin_once(translator.node, timeout_sec=0.02)
        rclpy.spin_once(translator.test_node, timeout_sec=0.02)
        if len(translator.robot_events) > 0:
            break

    # Verify robot event was published
    assert len(translator.robot_events) > 0, 'Robot event should be published'

    event = translator.robot_events[-1]
    assert event.data == 'finalize'


def test_dispatch_pending_feedback_publishes_without_subscriber_check(translator):
    """Due haptic commands should be published without gating on discovery."""
    translator.node.joy_feedback_pub.publish = Mock()
    translator.node._pending_feedback_commands = [Mock(due_at=0.0, channel_id=0, intensity=0.8)]
    translator.node.haptic_feedback_scheduler.now = Mock(return_value=0.0)

    translator.node._dispatch_pending_feedback()

    translator.node.joy_feedback_pub.publish.assert_called_once()
    assert translator.node._pending_feedback_commands == []


def test_haptic_feedback_is_published_on_joy_set_feedback(translator):
    """Gait changes should publish JoyFeedback messages on /joy/set_feedback."""
    joy_msg = sensor_msgs.msg.Joy()
    joy_msg.axes = [0.0] * 6
    joy_msg.buttons = [0] * 21
    joy_msg.buttons[14] = 1

    translator.node._joy_callback(joy_msg)

    for _ in range(10):
        rclpy.spin_once(translator.node, timeout_sec=0.02)
        rclpy.spin_once(translator.test_node, timeout_sec=0.02)
        if translator.joy_feedback_messages:
            break

    assert len(translator.joy_feedback_messages) > 0, 'Joy feedback should be published'
    feedback = translator.joy_feedback_messages[0]
    assert feedback.type == sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE
    assert feedback.id == LEFT_RUMBLE_CHANNEL_ID
    assert feedback.intensity > 0.0


def test_replacing_pending_control_mode_feedback_starts_new_pattern_immediately(translator):
    """A newer mode change should interrupt stale queued feedback immediately."""
    current_time = [100.0]
    translator.node.haptic_feedback_scheduler = HapticFeedbackScheduler(
        clock=lambda: current_time[0]
    )

    translator.node._publish_control_mode_change()
    current_time[0] = 100.02
    translator.node._publish_control_mode_change()

    pending = translator.node._pending_feedback_commands
    active_due_ats = [command.due_at for command in pending if command.intensity > 0.0]

    assert len(pending) == 19
    assert pending[0].due_at == pytest.approx(100.02, abs=1e-7)
    assert pending[0].intensity == 0.0
    assert len(active_due_ats) == 9
    assert active_due_ats[0] == pytest.approx(100.02, abs=1e-7)
    assert active_due_ats[1] == pytest.approx(100.22, abs=1e-7)
    assert active_due_ats[2] == pytest.approx(100.42, abs=1e-7)
    assert active_due_ats[-1] == pytest.approx(102.02, abs=1e-7)


def test_dispatch_pending_feedback_publishes_interrupt_and_new_pulse_in_same_tick(translator):
    """Interrupting stale feedback should emit stop and replacement pulse together."""
    current_time = [200.0]
    translator.node.haptic_feedback_scheduler = HapticFeedbackScheduler(
        clock=lambda: current_time[0]
    )
    translator.node.joy_feedback_pub.publish = Mock()

    translator.node._publish_control_mode_change()
    current_time[0] = 200.02
    translator.node._publish_control_mode_change()

    translator.node._dispatch_pending_feedback()

    assert translator.node.joy_feedback_pub.publish.call_count == 2

    stop_feedback = translator.node.joy_feedback_pub.publish.call_args_list[0].args[0]
    start_feedback = translator.node.joy_feedback_pub.publish.call_args_list[1].args[0]

    assert stop_feedback.id == RIGHT_RUMBLE_CHANNEL_ID
    assert stop_feedback.intensity == 0.0
    assert start_feedback.id == RIGHT_RUMBLE_CHANNEL_ID
    assert start_feedback.intensity > 0.0
    assert translator.node._pending_feedback_commands[0].due_at == pytest.approx(200.17, abs=1e-7)


def test_control_mode_haptic_feedback_uses_working_rumble_channel(translator):
    """Control-mode changes should repeat the mapped pulse group 3 times."""
    translator.node._publish_control_mode_change()

    for _ in range(90):
        rclpy.spin_once(translator.node, timeout_sec=0.02)
        rclpy.spin_once(translator.test_node, timeout_sec=0.02)
        active_pulses = [
            feedback for feedback in translator.joy_feedback_messages if feedback.intensity > 0.0
        ]
        if len(active_pulses) >= 6:
            break

    assert len(translator.joy_feedback_messages) > 0, (
        'Control-mode joy feedback should be published'
    )
    active_pulses = [
        feedback for feedback in translator.joy_feedback_messages if feedback.intensity > 0.0
    ]
    assert len(active_pulses) == 6
    for feedback in active_pulses:
        assert feedback.type == sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE
        assert feedback.id == RIGHT_RUMBLE_CHANNEL_ID
        assert feedback.intensity > 0.0
