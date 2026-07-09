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

"""
Declarative UI definition: which widgets exist and how they are laid out.

Pygame-free. ``build_controls`` wires widgets to the node/state callbacks,
``build_layout`` describes the screen as a layout tree, and ``apply_layout``
assigns solved rectangles to the widgets by their ``layout_key``. Every
position on screen comes from the layout engine — no hardcoded coordinates.
"""

from typing import Callable

from drqp_brain.joystick_input_handler import all_control_modes

from drqp_keyboard_control.gui_controls import (
    ButtonControl,
    CheckboxControl,
    Control,
    LabelControl,
    StickControl,
    TriggerControl,
)
from drqp_keyboard_control.layout import Align, Box, Column, Justify, Row, solve

KEYBOARD_HELP_LINES = [
    'W/A/S/D: left stick',
    'Arrow keys: right stick',
    'Hold multiple movement keys together',
    'Tab: cycle mode',
    '1/2/3: select Tripod/Ripple/Wave',
    'B: toggle balance mode',
    '+/-: adjust keyboard sensitivity',
    'Space or Esc: kill switch',
    'Delete: reboot servos',
    'Backspace: finalize',
]

GAIT_LABELS = ['Tripod', 'Ripple', 'Wave']

BUTTON_SIZE = (136.0, 38.0)
ACTION_BUTTON_SIZE = (118.0, 42.0)
CHECKBOX_SIZE = (150.0, 28.0)
TRIGGER_COLUMN_WIDTH = 90.0
STATUS_BAR_SIZE = (260.0, 20.0)


def build_controls(
    node,
    *,
    toggle_help: Callable[[], None],
    help_visible: Callable[[], bool],
    toggle_stay_on_top: Callable[[], None],
    stay_on_top_enabled: Callable[[], bool],
) -> list[Control]:
    """Create every widget, bound to the node's state and the app toggles."""
    state = node.state
    controls: list[Control] = []

    for mode in all_control_modes:
        controls.append(
            ButtonControl(
                label=mode.name,
                layout_key=f'mode.{mode.name}',
                action=lambda selected_mode=mode: state.set_control_mode(selected_mode),
                selected=lambda selected_mode=mode: state.control_mode == selected_mode,
            )
        )

    for index, label in enumerate(GAIT_LABELS):
        controls.append(
            ButtonControl(
                label=label,
                layout_key=f'gait.{index}',
                action=lambda selected_index=index: state.set_gait_index(selected_index),
                selected=lambda selected_index=index: state.gait_index == selected_index,
            )
        )

    controls.append(
        ButtonControl(
            label='Help',
            layout_key='help',
            action=toggle_help,
            selected=help_visible,
        )
    )
    controls.append(
        CheckboxControl(
            label='Stay on top',
            layout_key='stay_on_top',
            action=toggle_stay_on_top,
            selected=stay_on_top_enabled,
        )
    )
    controls.append(
        CheckboxControl(
            label='Balance Mode',
            layout_key='balance',
            action=node.toggle_balance_mode,
            selected=lambda: node.balance_mode_enabled,
        )
    )

    controls.append(
        TriggerControl(
            label='Left Trigger',
            layout_key='left_trigger',
            on_change=state.set_left_trigger,
            value=lambda: state.left_trigger,
        )
    )
    controls.append(
        StickControl(
            label='Left Stick',
            layout_key='left_stick',
            on_move=state.set_left_stick,
            on_release=state.release_left_stick,
            value=lambda: (state.axes().left_x, state.axes().left_y),
        )
    )
    controls.append(
        StickControl(
            label='Right Stick',
            layout_key='right_stick',
            on_move=state.set_right_stick,
            on_release=state.release_right_stick,
            value=lambda: (state.axes().right_x, state.axes().right_y),
        )
    )
    controls.append(
        TriggerControl(
            label='Right Trigger',
            layout_key='right_trigger',
            on_change=state.set_right_trigger,
            value=lambda: state.right_trigger,
        )
    )

    for event, label in (
        ('kill_switch_pressed', 'Kill Switch'),
        ('finalize', 'Finalize'),
        ('reboot_servos', 'Reboot'),
    ):
        controls.append(
            ButtonControl(
                label=label,
                layout_key=f'action.{event}',
                action=lambda event_name=event: node.publish_event(event_name),
            )
        )

    controls.append(
        LabelControl(
            label='Sensitivity',
            layout_key='sensitivity',
            text=lambda: f'Sensitivity {state.sensitivity:.2f}',
        )
    )
    return controls


def build_layout() -> Column:
    """Describe the whole window as a layout tree keyed by widget."""
    button_width, button_height = BUTTON_SIZE
    checkbox_width, checkbox_height = CHECKBOX_SIZE
    action_width, action_height = ACTION_BUTTON_SIZE

    def button_row(keys: list[str]) -> Row:
        return Row(
            children=[Box(key=key, width=button_width, height=button_height) for key in keys],
            spacing=14.0,
            justify=Justify.center,
        )

    return Column(
        padding=16.0,
        spacing=12.0,
        children=[
            button_row([f'mode.{mode.name}' for mode in all_control_modes]),
            button_row([f'gait.{index}' for index in range(len(GAIT_LABELS))]),
            Row(
                children=[
                    Box(key='help', width=button_width, height=button_height),
                    Box(key='stay_on_top', width=checkbox_width, height=checkbox_height),
                    Box(key='balance', width=checkbox_width, height=checkbox_height),
                ],
                spacing=20.0,
                justify=Justify.center,
                align=Align.center,
            ),
            Row(
                flex=1.0,
                spacing=24.0,
                children=[
                    Box(key='left_trigger', width=TRIGGER_COLUMN_WIDTH),
                    Box(key='left_stick', flex=1.0),
                    Box(key='right_stick', flex=1.0),
                    Box(key='right_trigger', width=TRIGGER_COLUMN_WIDTH),
                ],
            ),
            Row(
                children=[
                    Box(
                        key=f'action.{event}',
                        width=action_width,
                        height=action_height,
                    )
                    for event in ('kill_switch_pressed', 'finalize', 'reboot_servos')
                ],
                spacing=12.0,
                justify=Justify.center,
            ),
            Row(
                children=[
                    Box(key='sensitivity', width=STATUS_BAR_SIZE[0], height=STATUS_BAR_SIZE[1]),
                ],
            ),
        ],
    )


def apply_layout(controls: list[Control], width: float, height: float):
    """Solve the layout for the viewport and position every widget."""
    rects = solve(build_layout(), width, height)
    for control in controls:
        control.rect = rects[control.layout_key]
