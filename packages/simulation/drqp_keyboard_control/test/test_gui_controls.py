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

from drqp_keyboard_control.gui_controls import (
    ButtonControl,
    CheckboxControl,
    LabelControl,
    PointerRouter,
    StickControl,
    TriggerControl,
)
from drqp_keyboard_control.layout import RectSpec
import pytest


def make_stick(moves, releases):
    """Create a stick with a 50px circle radius, recording its callbacks."""
    return StickControl(
        label='Left Stick',
        layout_key='left_stick',
        rect=RectSpec(50.0, 50.0, 100.0, 180.0),
        on_move=lambda x, y: moves.append((x, y)),
        on_release=lambda: releases.append(True),
        value=lambda: moves[-1] if moves else (0.0, 0.0),
    )


def test_button_fires_action_on_press_and_clears_on_release():
    """Buttons should act on press and track the pressed visual state."""
    presses = []
    button = ButtonControl(
        label='Kill',
        layout_key='kill',
        rect=RectSpec(0.0, 0.0, 100.0, 40.0),
        action=lambda: presses.append(True),
    )

    assert button.hit_test((50.0, 20.0)) is True
    button.press((50.0, 20.0))
    assert presses == [True]
    assert button.pressed is True

    button.release()
    assert button.pressed is False
    assert button.selected() is False


def test_checkbox_reports_selected_state():
    """Checkboxes should reflect their bound state getter."""
    enabled = []
    checkbox = CheckboxControl(
        label='Balance Mode',
        layout_key='balance',
        rect=RectSpec(0.0, 0.0, 120.0, 28.0),
        action=lambda: enabled.append(True),
        selected=lambda: bool(enabled),
    )

    assert checkbox.selected() is False
    checkbox.press((5.0, 5.0))
    assert checkbox.selected() is True


def test_label_ignores_pointer():
    """Labels must never take pointer input."""
    label = LabelControl(
        label='Sensitivity',
        layout_key='sensitivity',
        rect=RectSpec(0.0, 0.0, 200.0, 20.0),
        text=lambda: 'Sensitivity 0.50',
    )

    assert label.hit_test((10.0, 10.0)) is False
    assert label.text() == 'Sensitivity 0.50'


def test_stick_geometry_derives_from_rect():
    """Stick center/radius should come from the layout rect minus text margin."""
    stick = make_stick([], [])

    assert stick.center == (100.0, 140.0)
    assert stick.radius == pytest.approx(50.0)
    assert stick.hit_test((100.0, 140.0)) is True
    assert stick.hit_test((100.0, 191.0)) is False


def test_stick_drag_reports_normalized_axes_y_up():
    """Dragging should report normalized axes with y pointing up."""
    moves = []
    releases = []
    stick = make_stick(moves, releases)

    stick.press((125.0, 115.0))
    assert stick.dragging is True
    assert moves[-1] == (pytest.approx(0.5), pytest.approx(0.5))

    stick.drag((100.0, 190.0))
    assert moves[-1] == (pytest.approx(0.0), pytest.approx(-1.0))

    stick.release()
    assert stick.dragging is False
    assert releases == [True]


def test_horizontal_trigger_maps_left_to_zero_right_to_one():
    """Horizontal trigger sliders should map along the x axis, clamped."""
    values = []
    trigger = TriggerControl(
        label='Left Trigger',
        layout_key='left_trigger',
        rect=RectSpec(10.0, 20.0, 100.0, 30.0),
        on_change=values.append,
        value=lambda: values[-1] if values else 0.0,
    )

    trigger.press((110.0, 25.0))
    assert values[-1] == pytest.approx(1.0)

    trigger.drag((-10.0, 25.0))
    assert values[-1] == pytest.approx(0.0)


def test_vertical_trigger_maps_top_to_pressed():
    """Vertical trigger sliders should put full trigger at the top."""
    values = []
    trigger = TriggerControl(
        label='Right Trigger',
        layout_key='right_trigger',
        rect=RectSpec(20.0, 10.0, 30.0, 100.0),
        on_change=values.append,
        value=lambda: values[-1] if values else 0.0,
    )

    assert trigger.vertical is True
    trigger.press((35.0, 10.0))
    assert values[-1] == pytest.approx(1.0)

    trigger.drag((35.0, 110.0))
    assert values[-1] == pytest.approx(0.0)


def test_router_routes_press_to_first_hit_and_captures_drag():
    """The router should capture the pressed widget for the whole drag."""
    moves = []
    releases = []
    stick = make_stick(moves, releases)
    presses = []
    button = ButtonControl(
        label='Help',
        layout_key='help',
        rect=RectSpec(200.0, 0.0, 80.0, 30.0),
        action=lambda: presses.append(True),
    )
    router = PointerRouter([button, stick])

    assert router.press((100.0, 100.0)) is True
    assert router.active is stick

    # Motion over the button must keep driving the captured stick.
    router.move((240.0, 15.0))
    assert len(moves) == 2
    assert presses == []

    router.release()
    assert releases == [True]
    assert router.active is None


def test_router_ignores_misses_and_motion_without_capture():
    """Presses outside all widgets should not create a capture."""
    presses = []
    button = ButtonControl(
        label='Help',
        layout_key='help',
        rect=RectSpec(0.0, 0.0, 50.0, 20.0),
        action=lambda: presses.append(True),
    )
    router = PointerRouter([button])

    assert router.press((300.0, 300.0)) is False
    router.move((10.0, 10.0))
    router.release()

    assert presses == []
    assert button.pressed is False


def test_router_press_release_cycle_on_button():
    """A captured button should clear its pressed state on release."""
    button = ButtonControl(
        label='Reboot',
        layout_key='reboot',
        rect=RectSpec(0.0, 0.0, 50.0, 20.0),
        action=lambda: None,
    )
    router = PointerRouter([button])

    router.press((10.0, 10.0))
    assert button.pressed is True

    router.release()
    assert button.pressed is False
