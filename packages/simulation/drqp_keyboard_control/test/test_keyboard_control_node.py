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
from drqp_interfaces.msg import MovementCommandConstants
from drqp_keyboard_control.gui_controls import (
    ButtonControl,
    CheckboxControl,
    clamp_vector,
    RectSpec,
    StickControl,
    TriggerControl,
)
from drqp_keyboard_control.keyboard_control_app import (
    KEYBOARD_HELP_LINES,
    PygameKeyboardControlApp,
)
from drqp_keyboard_control.keyboard_control_node import (
    GuiControlState,
    KeyboardControlNode,
)
from drqp_keyboard_control.sdl_window import (
    sdl_library_candidates,
    set_sdl_window_always_on_top,
)
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


def test_wasd_key_down_up_holds_until_release():
    """Keyboard axes should use key state instead of repeat timeouts."""
    state = GuiControlState(sensitivity=0.5)

    assert state.key_down('w') is True
    assert state.axes().left_y == pytest.approx(0.5)
    assert state.axes().left_y == pytest.approx(0.5)

    assert state.key_up('w') is True
    assert state.axes().left_y == pytest.approx(0.0)


def test_multiple_keyboard_keys_combine_across_sticks():
    """WASD and arrow keys should work while held together."""
    state = GuiControlState(sensitivity=0.75)

    for key in ('w', 'd', 'left', 'up'):
        state.key_down(key)

    axes = state.axes()
    assert axes.left_x == pytest.approx(0.75)
    assert axes.left_y == pytest.approx(0.75)
    assert axes.right_x == pytest.approx(-0.75)
    assert axes.right_y == pytest.approx(0.75)


def test_opposite_keyboard_keys_cancel_on_axis():
    """Opposite held keys should cancel each other."""
    state = GuiControlState(sensitivity=0.6)

    state.key_down('w')
    state.key_down('s')
    state.key_down('left')
    state.key_down('right')

    axes = state.axes()
    assert axes.left_y == pytest.approx(0.0)
    assert axes.right_x == pytest.approx(0.0)


def test_pointer_stick_takes_precedence_over_keyboard_until_released():
    """Dragged virtual sticks should override keyboard axes for that stick."""
    state = GuiControlState(sensitivity=0.5)
    state.key_down('w')

    state.set_left_stick(-0.25, -0.5)
    assert state.axes().left_x == pytest.approx(-0.25)
    assert state.axes().left_y == pytest.approx(-0.5)

    state.release_left_stick()
    assert state.axes().left_x == pytest.approx(0.0)
    assert state.axes().left_y == pytest.approx(0.5)


def test_stick_drag_clamps_and_release_springs_to_center():
    """Mobile-style stick drag should clamp to the unit circle and spring back."""
    state = GuiControlState()
    stick = StickControl(
        'Left',
        (100.0, 100.0),
        50.0,
        state.set_left_stick,
        state.release_left_stick,
    )

    stick.begin_drag((200.0, 0.0))
    axes = state.axes()
    assert axes.left_x == pytest.approx(0.707106, abs=1e-5)
    assert axes.left_y == pytest.approx(0.707106, abs=1e-5)

    stick.end_drag()
    assert state.axes().left_x == pytest.approx(0.0)
    assert state.axes().left_y == pytest.approx(0.0)


def test_trigger_slider_clamps():
    """Trigger sliders should produce stable 0..1 values."""
    state = GuiControlState()
    trigger = TriggerControl(
        'Left Trigger',
        RectSpec(10.0, 20.0, 100.0, 30.0),
        state.set_left_trigger,
    )

    trigger.begin_drag((110.0, 25.0))
    assert state.left_trigger == pytest.approx(1.0)

    trigger.update_drag((-10.0, 25.0))
    assert state.left_trigger == pytest.approx(0.0)


def test_vertical_trigger_slider_maps_top_to_pressed():
    """Vertical trigger sliders should put full trigger at the top."""
    state = GuiControlState()
    trigger = TriggerControl(
        'Right Trigger',
        RectSpec(20.0, 10.0, 30.0, 100.0),
        state.set_right_trigger,
    )

    trigger.begin_drag((35.0, 10.0))
    assert state.right_trigger == pytest.approx(1.0)

    trigger.update_drag((35.0, 110.0))
    assert state.right_trigger == pytest.approx(0.0)


def test_mode_and_gait_toggle_selection():
    """Visual toggle actions should update mode and gait state directly."""
    state = GuiControlState()

    mode_button = ButtonControl(
        'BodyRotation',
        RectSpec(0.0, 0.0, 100.0, 30.0),
        lambda: state.set_control_mode(ControlMode.BodyRotation),
        lambda: state.control_mode == ControlMode.BodyRotation,
    )
    gait_button = ButtonControl(
        'Wave',
        RectSpec(110.0, 0.0, 100.0, 30.0),
        lambda: state.set_gait_index(2),
        lambda: state.gait_index == 2,
    )

    assert mode_button.click((5.0, 5.0)) is True
    assert gait_button.click((115.0, 5.0)) is True

    assert state.control_mode == ControlMode.BodyRotation
    assert state.gait == MovementCommandConstants.GAIT_WAVE
    assert mode_button.selected() is True
    assert gait_button.selected() is True


def test_checkbox_click_invokes_action_and_tracks_pressed_state():
    """Checkbox controls should behave like click targets."""
    calls = []
    checkbox = CheckboxControl(
        'Stay on top',
        RectSpec(0.0, 0.0, 120.0, 28.0),
        lambda: calls.append('toggle'),
        lambda: bool(calls),
    )

    assert checkbox.click((10.0, 10.0)) is True
    assert calls == ['toggle']
    assert checkbox.selected() is True
    assert checkbox.pressed is True

    checkbox.release()
    assert checkbox.pressed is False


def test_stay_on_top_toggle_updates_state_and_applies_best_effort():
    """Stay-on-top checkbox should reflect clicks even if the platform call fails."""
    app = PygameKeyboardControlApp.__new__(PygameKeyboardControlApp)
    app.stay_on_top = False
    requested_states = []
    app._apply_stay_on_top = lambda enabled: requested_states.append(enabled) and False

    app._toggle_stay_on_top()
    assert app.stay_on_top is True
    assert requested_states == [True]

    app._toggle_stay_on_top()
    assert app.stay_on_top is False
    assert requested_states == [True, False]


def test_set_sdl_window_always_on_top_fails_without_window_id():
    """The platform topmost helper should safely no-op when unavailable."""
    assert set_sdl_window_always_on_top(None, True) is False


def test_sdl_library_candidates_are_unique():
    """SDL lookup should return a deduplicated best-effort candidate list."""
    candidates = sdl_library_candidates()

    assert candidates == list(dict.fromkeys(candidates))
    assert all(candidates)


def test_tab_cycles_control_modes():
    """TAB should cycle through the same control modes as the joystick node."""
    state = GuiControlState()

    state.key_down('tab')
    assert state.control_mode == ControlMode.BodyPosition

    state.key_down('tab')
    assert state.control_mode == ControlMode.BodyRotation

    state.key_down('tab')
    assert state.control_mode == ControlMode.Walk


def test_number_keys_select_gaits():
    """Number keys should directly select tripod, ripple, and wave gaits."""
    state = GuiControlState()

    state.key_down('2')
    assert state.gait == MovementCommandConstants.GAIT_RIPPLE

    state.key_down('3')
    assert state.gait == MovementCommandConstants.GAIT_WAVE

    state.key_down('1')
    assert state.gait == MovementCommandConstants.GAIT_TRIPOD


def test_sensitivity_adjustment_is_clamped():
    """Plus and minus keys should adjust the emulated stick magnitude."""
    state = GuiControlState(sensitivity=0.95, sensitivity_step=0.1)

    state.key_down('+')
    assert state.sensitivity == pytest.approx(1.0)

    for _ in range(20):
        state.key_down('-')

    assert state.sensitivity == pytest.approx(0.1)


def test_walk_command_maps_sticks_and_left_trigger():
    """Walk mode should map left stick, right stick X, and left trigger."""
    state = GuiControlState()
    state.set_left_stick(0.25, 0.5)
    state.set_right_stick(-0.75, 0.1)
    state.set_left_trigger(0.4)

    command = state.movement_command()

    assert command.stride_direction.x == pytest.approx(0.5)
    assert command.stride_direction.y == pytest.approx(-0.25)
    assert command.stride_direction.z == pytest.approx(0.4)
    assert command.rotation_speed == pytest.approx(0.75)


def test_body_position_mode_uses_sticks():
    """Body position mode should match existing joystick-style mapping."""
    state = GuiControlState()
    state.control_mode = ControlMode.BodyPosition
    state.set_left_stick(-0.4, 0.3)
    state.set_right_stick(0.0, 0.8)

    command = state.movement_command()

    assert command.body_translation.x == pytest.approx(0.3)
    assert command.body_translation.y == pytest.approx(0.4)
    assert command.body_translation.z == pytest.approx(0.8)
    assert command.stride_direction.x == pytest.approx(0.0)


def test_body_rotation_mode_uses_sticks():
    """Body rotation mode should match existing joystick-style mapping."""
    state = GuiControlState()
    state.control_mode = ControlMode.BodyRotation
    state.set_left_stick(0.6, -0.5)
    state.set_right_stick(0.7, 0.0)

    command = state.movement_command()

    assert command.body_rotation.x == pytest.approx(-0.6)
    assert command.body_rotation.y == pytest.approx(-0.5)
    assert command.body_rotation.z == pytest.approx(-0.7)
    assert command.rotation_speed == pytest.approx(0.0)


def test_space_and_escape_publish_kill_switch_once_per_keydown(ros_context):
    """Space and Esc should publish kill switch events as actions."""
    node = KeyboardControlNode()
    node.robot_event_pub.publish = Mock()
    try:
        assert node.handle_key_down('space') is True
        assert node.handle_key_down('esc') is True
        assert node.handle_key_up('space') is False
        assert node.handle_key_up('esc') is False
    finally:
        node.destroy_node()

    event_names = [call.args[0].data for call in node.robot_event_pub.publish.call_args_list]
    assert event_names == ['kill_switch_pressed', 'kill_switch_pressed']


def test_delete_and_backspace_publish_robot_events(ros_context):
    """Delete and Backspace should publish matching robot events."""
    node = KeyboardControlNode()
    node.robot_event_pub.publish = Mock()
    try:
        assert node.handle_key_down('delete') is True
        assert node.handle_key_down('backspace') is True
    finally:
        node.destroy_node()

    event_names = [call.args[0].data for call in node.robot_event_pub.publish.call_args_list]
    assert event_names == ['reboot_servos', 'finalize']


def test_b_key_toggles_balance_mode_and_publishes(ros_context):
    """B should toggle balance mode and publish the latched state each time."""
    node = KeyboardControlNode()
    node.balance_mode_pub.publish = Mock()
    try:
        assert node.handle_key_down('b') is True
        assert node.balance_mode_enabled is True
        assert node.handle_key_down('b') is True
        assert node.balance_mode_enabled is False
    finally:
        node.destroy_node()

    published = [call.args[0].data for call in node.balance_mode_pub.publish.call_args_list]
    assert published == [True, False]


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


def test_balance_mode_checkbox_toggles_node_directly():
    """Balance Mode checkbox should call into the node's toggle method directly."""
    app = PygameKeyboardControlApp.__new__(PygameKeyboardControlApp)
    app.node = Mock()
    app.node.balance_mode_enabled = False
    app.buttons = []
    app.checkboxes = []
    app.show_help = False
    app.stay_on_top = False

    app._build_buttons()

    balance_checkbox = next(c for c in app.checkboxes if c.label == 'Balance Mode')
    assert balance_checkbox.selected() is False

    pos = (balance_checkbox.rect.x + 1.0, balance_checkbox.rect.y + 1.0)
    assert balance_checkbox.click(pos) is True
    app.node._toggle_balance_mode.assert_called_once_with()


def test_publish_stop_command_clears_motion_inputs_before_publishing(ros_context):
    """GUI shutdown should send one final zero movement command."""
    node = KeyboardControlNode()
    node.movement_command_pub.publish = Mock()
    try:
        node.state.key_down('w')
        node.state.set_left_stick(0.5, 0.5)
        node.state.set_right_stick(-0.25, 0.0)
        node.state.set_left_trigger(0.75)
        node.state.set_right_trigger(0.5)

        node.publish_stop_command()
    finally:
        node.destroy_node()

    command = node.movement_command_pub.publish.call_args.args[0]
    assert command.stride_direction.x == pytest.approx(0.0)
    assert command.stride_direction.y == pytest.approx(0.0)
    assert command.stride_direction.z == pytest.approx(0.0)
    assert command.rotation_speed == pytest.approx(0.0)


def test_app_shutdown_requests_stop_command():
    """Closing the pygame window should ask the node to stop the robot."""
    app = PygameKeyboardControlApp.__new__(PygameKeyboardControlApp)
    app.running = True
    app.node = Mock()

    app._request_shutdown()

    app.node.publish_stop_command.assert_called_once_with()
    assert app.running is False


def test_keyboard_help_lines_cover_all_bindings():
    """Help content should list all keyboard controls."""
    help_text = '\n'.join(KEYBOARD_HELP_LINES)

    for expected in (
        'W/A/S/D',
        'Arrow keys',
        'Tab',
        '1/2/3',
        'B: toggle balance mode',
        '+/-',
        'Space or Esc',
        'Delete',
        'Backspace',
    ):
        assert expected in help_text


def test_clamp_vector_limits_to_unit_circle():
    """The vector helper should preserve direction while limiting magnitude."""
    x, y = clamp_vector(3.0, 4.0)

    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)
