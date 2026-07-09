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
from drqp_keyboard_control.control_state import GuiControlState
from drqp_keyboard_control.gui_controls import LabelControl, StickControl
from drqp_keyboard_control.ui import apply_layout, build_controls, KEYBOARD_HELP_LINES
import pytest


@pytest.fixture
def node():
    """Node double exposing the interfaces build_controls binds to."""
    node = Mock()
    node.state = GuiControlState()
    node.balance_mode_enabled = False
    return node


def make_controls(node, **overrides):
    """Build the full widget list with no-op app callbacks by default."""
    callbacks = {
        'toggle_help': lambda: None,
        'help_visible': lambda: False,
        'toggle_stay_on_top': lambda: None,
        'stay_on_top_enabled': lambda: False,
    }
    callbacks.update(overrides)
    return build_controls(node, **callbacks)


def rects_intersect(a, b) -> bool:
    """Return whether two RectSpec areas overlap."""
    return (
        a.x < b.x + b.width
        and b.x < a.x + a.width
        and a.y < b.y + b.height
        and b.y < a.y + a.height
    )


def test_every_control_gets_a_layout_slot(node):
    """The layout tree must provide a rect for every widget key."""
    controls = make_controls(node)

    apply_layout(controls, 980.0, 640.0)

    for control in controls:
        assert control.rect.width > 0.0, control.layout_key
        assert control.rect.height > 0.0, control.layout_key


@pytest.mark.parametrize('size', [(980.0, 640.0), (760.0, 520.0), (1400.0, 900.0)])
def test_layout_produces_no_overlapping_widgets(node, size):
    """Widgets must never overlap, regardless of the window size."""
    controls = make_controls(node)
    apply_layout(controls, *size)

    interactive = [c for c in controls if not isinstance(c, LabelControl)]
    for index, first in enumerate(interactive):
        for second in interactive[index + 1 :]:  # noqa: E203
            assert not rects_intersect(first.rect, second.rect), (
                f'{first.layout_key} overlaps {second.layout_key} at {size}'
            )


def test_layout_stays_inside_viewport(node):
    """All widgets must stay within the window bounds."""
    controls = make_controls(node)
    apply_layout(controls, 980.0, 640.0)

    for control in controls:
        assert control.rect.x >= 0.0, control.layout_key
        assert control.rect.y >= 0.0, control.layout_key
        assert control.rect.x + control.rect.width <= 980.0, control.layout_key
        assert control.rect.y + control.rect.height <= 640.0, control.layout_key


def test_relayout_moves_widgets_for_new_window_size(node):
    """Re-solving for a resized window should reposition widgets."""
    controls = make_controls(node)
    apply_layout(controls, 980.0, 640.0)
    left_stick = next(c for c in controls if c.layout_key == 'left_stick')
    small_rect = left_stick.rect

    apply_layout(controls, 1600.0, 1000.0)

    assert left_stick.rect != small_rect
    assert left_stick.rect.width > small_rect.width


def test_mode_and_gait_buttons_drive_state(node):
    """Mode and gait buttons should update state and reflect selection."""
    controls = make_controls(node)
    by_key = {c.layout_key: c for c in controls}

    by_key['mode.BodyRotation'].action()
    assert node.state.control_mode == ControlMode.BodyRotation
    assert by_key['mode.BodyRotation'].selected() is True
    assert by_key['mode.Walk'].selected() is False

    by_key['gait.2'].action()
    assert node.state.gait_index == 2
    assert by_key['gait.2'].selected() is True


def test_sticks_and_triggers_bind_to_state(node):
    """Stick and trigger widgets should read and write the shared state."""
    controls = make_controls(node)
    by_key = {c.layout_key: c for c in controls}

    by_key['left_stick'].on_move(0.25, -0.5)
    assert by_key['left_stick'].value() == (pytest.approx(0.25), pytest.approx(-0.5))

    by_key['right_trigger'].on_change(0.7)
    assert by_key['right_trigger'].value() == pytest.approx(0.7)


def test_stick_release_latches_in_body_mode_through_widget(node):
    """The full widget-to-state path must latch sticks in body modes."""
    controls = make_controls(node)
    apply_layout(controls, 980.0, 640.0)
    by_key = {c.layout_key: c for c in controls}
    node.state.set_control_mode(ControlMode.BodyPosition)

    stick: StickControl = by_key['left_stick']
    center_x, center_y = stick.center
    stick.press((center_x + stick.radius / 2.0, center_y))
    stick.release()

    assert stick.value() == (pytest.approx(0.5), pytest.approx(0.0))


def test_action_buttons_publish_robot_events(node):
    """Kill switch, finalize, and reboot buttons should publish events."""
    controls = make_controls(node)
    by_key = {c.layout_key: c for c in controls}

    by_key['action.kill_switch_pressed'].action()
    by_key['action.finalize'].action()
    by_key['action.reboot_servos'].action()

    published = [call.args[0] for call in node.publish_event.call_args_list]
    assert published == ['kill_switch_pressed', 'finalize', 'reboot_servos']


def test_balance_checkbox_binds_to_node(node):
    """The Balance Mode checkbox should call into the node toggle."""
    controls = make_controls(node)
    balance = next(c for c in controls if c.layout_key == 'balance')

    assert balance.selected() is False
    balance.action()
    node.toggle_balance_mode.assert_called_once_with()

    node.balance_mode_enabled = True
    assert balance.selected() is True


def test_help_and_stay_on_top_bind_to_app_callbacks(node):
    """Help button and stay-on-top checkbox use the app-provided hooks."""
    calls = []
    controls = make_controls(
        node,
        toggle_help=lambda: calls.append('help'),
        toggle_stay_on_top=lambda: calls.append('top'),
        help_visible=lambda: True,
    )
    by_key = {c.layout_key: c for c in controls}

    by_key['help'].action()
    by_key['stay_on_top'].action()

    assert calls == ['help', 'top']
    assert by_key['help'].selected() is True


def test_sensitivity_label_tracks_state(node):
    """The status label should render the live sensitivity value."""
    controls = make_controls(node)
    label = next(c for c in controls if c.layout_key == 'sensitivity')

    assert label.text() == 'Sensitivity 0.50'
    node.state.adjust_sensitivity(0.1)
    assert label.text() == 'Sensitivity 0.60'


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
