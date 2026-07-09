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

"""App-level tests running Pygame headless through the SDL dummy driver."""

import os
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

from drqp_keyboard_control.control_state import GuiControlState
import pytest


@pytest.fixture
def app():
    """Construct the app against a node double using the dummy SDL driver."""
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'

    from drqp_keyboard_control.keyboard_control_app import PygameKeyboardControlApp

    node = Mock()
    node.state = GuiControlState()
    node.balance_mode_enabled = False
    app = PygameKeyboardControlApp(node, width=980, height=640)
    yield app
    app.close()


def find_control(app, layout_key):
    """Return the widget bound to the given layout key."""
    return next(c for c in app.controls if c.layout_key == layout_key)


def test_key_events_are_normalized_and_forwarded(app):
    """Known keys should reach the node; unknown keys must be ignored."""
    pygame = app.pygame
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_w))
    pygame.event.post(pygame.event.Event(pygame.KEYUP, key=pygame.K_w))
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_F1))

    app._handle_events()

    app.node.handle_key_down.assert_called_once_with('w')
    app.node.handle_key_up.assert_called_once_with('w')


def test_mouse_drag_on_stick_drives_state(app):
    """A press-drag-release on the stick should drive the control state."""
    pygame = app.pygame
    stick = find_control(app, 'left_stick')
    center_x, center_y = stick.center
    grab = (int(center_x), int(center_y))
    target = (int(center_x + stick.radius), int(center_y))

    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=grab))
    pygame.event.post(pygame.event.Event(pygame.MOUSEMOTION, pos=target))
    app._handle_events()
    assert app.node.state.axes().left_x == pytest.approx(1.0)

    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=target))
    app._handle_events()
    assert app.node.state.axes().left_x == pytest.approx(0.0)


def test_button_click_fires_action(app):
    """Clicking inside a button rect should trigger its action."""
    pygame = app.pygame
    kill = find_control(app, 'action.kill_switch_pressed')
    pos = (int(kill.rect.center[0]), int(kill.rect.center[1]))

    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=pos))
    app._handle_events()

    app.node.publish_event.assert_called_once_with('kill_switch_pressed')


def test_window_resize_relayouts_widgets(app):
    """A resize event should re-solve the layout for the new size."""
    pygame = app.pygame
    stick = find_control(app, 'right_stick')
    old_rect = stick.rect

    pygame.event.post(pygame.event.Event(pygame.VIDEORESIZE, w=1600, h=1000))
    app._handle_events()

    assert stick.rect != old_rect


def test_quit_event_stops_the_robot_and_the_loop(app):
    """Closing the window should publish a stop command and end the loop."""
    pygame = app.pygame
    pygame.event.post(pygame.event.Event(pygame.QUIT))

    app._handle_events()

    app.node.publish_stop_command.assert_called_once_with()
    assert app.running is False


def test_render_smoke_headless(app):
    """A full frame render must succeed, including the help overlay."""
    app._render()
    app.show_help = True
    app._render()


def test_stay_on_top_toggle_updates_state_and_applies_best_effort(app):
    """Stay-on-top checkbox should reflect clicks even if the platform call fails."""
    requested_states = []
    app._apply_stay_on_top = lambda enabled: requested_states.append(enabled) and False

    checkbox = find_control(app, 'stay_on_top')
    checkbox.action()
    assert app.stay_on_top is True
    assert requested_states == [True]

    checkbox.action()
    assert app.stay_on_top is False
    assert requested_states == [True, False]


def test_stay_on_top_looks_up_window_id_lazily(app):
    """Window-id discovery should be deferred until topmost support is used."""
    app.window_id = None
    app._display_window_id = lambda: 42
    requested = []
    app._apply_stay_on_top = (
        lambda enabled: requested.append(enabled) or setattr(app, 'window_id', 42) or True
    )

    find_control(app, 'stay_on_top').action()

    assert app.window_id == 42
    assert requested == [True]


def test_display_window_id_uses_wm_info_window_handle(app):
    """Window-id discovery should prefer SDL's display window identifier."""
    app.pygame.display.get_wm_info = lambda: {'window': 99}
    fake_window_type = type(
        'FakeWindowType',
        (),
        {'from_display_module': staticmethod(lambda: type('FakeWindow', (), {'id': 42})())},
    )

    with patch.dict(sys.modules, {'pygame._sdl2.video': SimpleNamespace(Window=fake_window_type)}):
        assert app._display_window_id() == 42
