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

import curses

from drqp_brain.joystick_input_handler import ControlMode
from drqp_brain.keyboard_control_node import (
    KeyboardControlNode,
    KeyboardControlScreen,
    KeyboardControlState,
)
from drqp_interfaces.msg import MovementCommandConstants
import pytest
import rclpy


def test_wasd_maps_to_walk_stride_direction():
    """WASD should emulate the joystick left stick for walking."""
    state = KeyboardControlState(sensitivity=0.5)
    now = 10.0

    state.handle_key('w', now)
    state.handle_key('a', now)

    command = state.movement_command(now)

    assert command.stride_direction.x == pytest.approx(0.5)
    assert command.stride_direction.y == pytest.approx(-0.5)
    assert command.stride_direction.z == pytest.approx(0.0)
    assert command.rotation_speed == pytest.approx(0.0)


def test_arrows_map_to_walk_rotation():
    """Left and right arrow keys should emulate right-stick walk rotation."""
    state = KeyboardControlState(sensitivity=0.7)

    state.handle_key('right', 1.0)

    command = state.movement_command(1.0)

    assert command.rotation_speed == pytest.approx(0.7)


def test_body_position_mode_uses_wasd_and_arrow_z():
    """Body position mode should map like joystick body translation."""
    state = KeyboardControlState(sensitivity=0.4)
    state.control_mode = ControlMode.BodyPosition

    state.handle_key('d', 1.0)
    state.handle_key('up', 1.0)

    command = state.movement_command(1.0)

    assert command.body_translation.x == pytest.approx(0.0)
    assert command.body_translation.y == pytest.approx(0.4)
    assert command.body_translation.z == pytest.approx(0.4)
    assert command.stride_direction.x == pytest.approx(0.0)


def test_body_rotation_mode_uses_wasd_and_arrow_yaw():
    """Body rotation mode should map like joystick body rotation."""
    state = KeyboardControlState(sensitivity=0.6)
    state.control_mode = ControlMode.BodyRotation

    state.handle_key('a', 2.0)
    state.handle_key('s', 2.0)
    state.handle_key('left', 2.0)

    command = state.movement_command(2.0)

    assert command.body_rotation.x == pytest.approx(-0.6)
    assert command.body_rotation.y == pytest.approx(-0.6)
    assert command.body_rotation.z == pytest.approx(-0.6)
    assert command.rotation_speed == pytest.approx(0.0)


def test_keys_expire_after_timeout():
    """Motion should return to zero when terminal key repeat stops."""
    state = KeyboardControlState(sensitivity=0.5, key_timeout_sec=0.25)

    state.handle_key('w', 3.0)

    active_command = state.movement_command(3.2)
    expired_command = state.movement_command(3.26)

    assert active_command.stride_direction.x == pytest.approx(0.5)
    assert expired_command.stride_direction.x == pytest.approx(0.0)


def test_tab_cycles_control_modes():
    """TAB should cycle through the same control modes as the joystick node."""
    state = KeyboardControlState()

    state.handle_key('tab', 1.0)
    assert state.control_mode == ControlMode.BodyPosition

    state.handle_key('tab', 1.1)
    assert state.control_mode == ControlMode.BodyRotation

    state.handle_key('tab', 1.2)
    assert state.control_mode == ControlMode.Walk


def test_number_keys_select_gaits():
    """Number keys should directly select tripod, ripple, and wave gaits."""
    state = KeyboardControlState()

    state.handle_key('2', 1.0)
    assert state.gait == MovementCommandConstants.GAIT_RIPPLE

    state.handle_key('3', 1.1)
    assert state.gait == MovementCommandConstants.GAIT_WAVE

    state.handle_key('1', 1.2)
    assert state.gait == MovementCommandConstants.GAIT_TRIPOD


def test_sensitivity_adjustment_is_clamped():
    """Plus and minus keys should adjust the emulated stick magnitude."""
    state = KeyboardControlState(sensitivity=0.95, sensitivity_step=0.1)

    state.handle_key('+', 1.0)
    assert state.sensitivity == pytest.approx(1.0)

    for index in range(20):
        state.handle_key('-', 2.0 + index)

    assert state.sensitivity == pytest.approx(0.1)


def test_h_toggles_detailed_help():
    """H should toggle the detailed terminal help text."""
    state = KeyboardControlState()

    state.handle_key('h', 1.0)
    assert state.show_detailed_help is True

    state.handle_key('h', 1.1)
    assert state.show_detailed_help is False


class FakeCursesScreen:
    """Minimal screen stub for key normalization tests."""

    def __init__(self, key_code):
        self.key_code = key_code

    def getch(self):
        return self.key_code


class FakeRenderScreen:
    """Minimal screen stub for render behavior tests."""

    def __init__(self):
        self.calls = []

    def getmaxyx(self):
        return (8, 40)

    def bkgd(self, char, attr):
        self.calls.append(('bkgd', char, attr))

    def erase(self):
        self.calls.append(('erase',))

    def addnstr(self, row, column, text, limit, attr):
        self.calls.append(('addnstr', row, column, text, limit, attr))

    def noutrefresh(self):
        self.calls.append(('noutrefresh',))


class FakeUiNode:
    """Minimal node stub for render behavior tests."""

    def ui_lines(self, now):
        return ['Dr.QP Keyboard Control', '', 'Mode: Walk']


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


def test_ui_lines_do_not_emit_ansi_clear_sequences(ros_context):
    """Curses owns screen clearing; UI lines should remain plain text."""
    node = KeyboardControlNode()
    try:
        lines = node.ui_lines(1.0)
    finally:
        node.destroy_node()

    assert lines[0] == 'Dr.QP Keyboard Control'
    assert all('\033' not in line for line in lines)


def test_curses_screen_normalizes_arrow_and_tab_keys():
    """The curses input path should preserve the requested key bindings."""
    assert (
        KeyboardControlScreen(
            FakeCursesScreen(curses.KEY_DOWN),
            None,
            refresh_rate_hz=8.0,
        ).read_key()
        == 'down'
    )
    assert KeyboardControlScreen(FakeCursesScreen(9), None, refresh_rate_hz=8.0).read_key() == (
        'tab'
    )
    assert (
        KeyboardControlScreen(FakeCursesScreen(ord('+')), None, refresh_rate_hz=8.0).read_key()
        == '+'
    )


def test_curses_render_paints_full_black_background(monkeypatch):
    """Each frame should clear the whole screen with the configured background."""
    monkeypatch.setattr(curses, 'doupdate', lambda: None)
    fake_screen = FakeRenderScreen()
    screen = KeyboardControlScreen(fake_screen, FakeUiNode(), refresh_rate_hz=8.0)
    screen.screen_attr = 123

    screen.render(force=True)

    assert fake_screen.calls[0] == ('bkgd', ' ', 123)
    assert fake_screen.calls[1] == ('erase',)
    assert fake_screen.calls[-1] == ('noutrefresh',)
    drawn_text = [call[3] for call in fake_screen.calls if call[0] == 'addnstr']
    assert drawn_text == ['Dr.QP Keyboard Control', '', 'Mode: Walk']
