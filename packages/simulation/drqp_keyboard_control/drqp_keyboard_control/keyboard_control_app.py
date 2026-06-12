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

from drqp_brain.joystick_input_handler import all_control_modes
import rclpy

from drqp_keyboard_control.gui_controls import (
    ButtonControl,
    CheckboxControl,
    RectSpec,
    StickControl,
    TriggerControl,
)
from drqp_keyboard_control.sdl_window import set_sdl_window_always_on_top


KEYBOARD_HELP_LINES = [
    'W/A/S/D: left stick',
    'Arrow keys: right stick',
    'Hold multiple movement keys together',
    'Tab: cycle mode',
    '1/2/3: select Tripod/Ripple/Wave',
    '+/-: adjust keyboard sensitivity',
    'Space or Esc: kill switch',
    'Delete: reboot servos',
    'Backspace: finalize',
]


class PygameKeyboardControlApp:
    """Pygame GUI wrapper around KeyboardControlNode."""

    def __init__(
        self,
        node,
        *,
        width: int = 980,
        height: int = 640,
        frame_rate_hz: float = 60.0,
    ):
        import pygame

        self.pygame = pygame
        self.node = node
        self.width = width
        self.height = height
        self.frame_rate_hz = frame_rate_hz
        self.running = True
        self.active_stick: StickControl | None = None
        self.active_trigger: TriggerControl | None = None
        self.buttons: list[ButtonControl] = []
        self.checkboxes: list[CheckboxControl] = []
        self.show_help = False
        self.stay_on_top = False

        pygame.init()
        pygame.display.set_caption('Dr.QP Keyboard Control')
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.window_id = self._display_window_id()
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.small_font = pygame.font.Font(None, 22)

        self.left_stick = StickControl(
            'Left Stick',
            (220.0, 385.0),
            105.0,
            self.node.state.set_left_stick,
            self.node.state.release_left_stick,
        )
        self.right_stick = StickControl(
            'Right Stick',
            (520.0, 385.0),
            105.0,
            self.node.state.set_right_stick,
            self.node.state.release_right_stick,
        )
        self.left_trigger = TriggerControl(
            'Left Trigger',
            RectSpec(55.0, 280.0, 34.0, 210.0),
            self.node.state.set_left_trigger,
        )
        self.right_trigger = TriggerControl(
            'Right Trigger',
            RectSpec(655.0, 280.0, 34.0, 210.0),
            self.node.state.set_right_trigger,
        )
        self._build_buttons()

    def run(self):
        """Run the GUI and ROS event loops."""
        while self.running and rclpy.ok():
            self._handle_events()
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._render()
            self.clock.tick(self.frame_rate_hz)

    def _build_buttons(self):
        mode_x = 150.0
        for index, mode in enumerate(all_control_modes):
            self.buttons.append(
                ButtonControl(
                    mode.name,
                    RectSpec(mode_x + index * 150.0, 35.0, 136.0, 38.0),
                    lambda selected_mode=mode: self.node.state.set_control_mode(selected_mode),
                    lambda selected_mode=mode: self.node.state.control_mode == selected_mode,
                )
            )

        gait_labels = ['Tripod', 'Ripple', 'Wave']
        for index, label in enumerate(gait_labels):
            self.buttons.append(
                ButtonControl(
                    label,
                    RectSpec(mode_x + index * 150.0, 88.0, 136.0, 38.0),
                    lambda selected_index=index: self.node.state.set_gait_index(selected_index),
                    lambda selected_index=index: self.node.state.gait_index == selected_index,
                )
            )

        actions = [
            ('Kill Switch', lambda: self.node._publish_event('kill_switch_pressed')),
            ('Finalize', lambda: self.node._publish_event('finalize')),
            ('Reboot', lambda: self.node._publish_event('reboot_servos')),
        ]
        for index, (label, action) in enumerate(actions):
            self.buttons.append(
                ButtonControl(
                    label,
                    RectSpec(185.0 + index * 130.0, 560.0, 118.0, 42.0),
                    action,
                )
            )
        self.buttons.append(
            ButtonControl(
                'Help',
                RectSpec(150.0, 141.0, 136.0, 38.0),
                self._toggle_help,
                lambda: self.show_help,
            )
        )
        self.checkboxes.append(
            CheckboxControl(
                'Stay on top',
                RectSpec(300.0, 146.0, 120.0, 28.0),
                self._toggle_stay_on_top,
                lambda: self.stay_on_top,
            )
        )

    def _toggle_help(self):
        self.show_help = not self.show_help

    def _toggle_stay_on_top(self):
        self.stay_on_top = not self.stay_on_top
        self._apply_stay_on_top(self.stay_on_top)

    def _apply_stay_on_top(self, enabled: bool) -> bool:
        return set_sdl_window_always_on_top(self.window_id, enabled)

    def _display_window_id(self) -> int | None:
        try:
            from pygame._sdl2.video import Window

            return int(Window.from_display_module().id)
        except (AttributeError, ImportError, TypeError, ValueError):
            return None

    def _handle_events(self):
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                self.running = False
            elif event.type == self.pygame.KEYDOWN:
                key = self._normalize_key(event.key)
                if key is not None:
                    self.node.handle_key_down(key)
            elif event.type == self.pygame.KEYUP:
                key = self._normalize_key(event.key)
                if key is not None:
                    self.node.handle_key_up(key)
            elif event.type == self.pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._begin_pointer(event.pos)
            elif event.type == self.pygame.MOUSEMOTION:
                self._move_pointer(event.pos)
            elif event.type == self.pygame.MOUSEBUTTONUP and event.button == 1:
                self._end_pointer()

    def _begin_pointer(self, pos: tuple[float, float]):
        for button in self.buttons:
            if button.click(pos):
                return
        for checkbox in self.checkboxes:
            if checkbox.click(pos):
                return

        if self.left_stick.hit_test(pos):
            self.active_stick = self.left_stick
            self.active_stick.begin_drag(pos)
            return
        if self.right_stick.hit_test(pos):
            self.active_stick = self.right_stick
            self.active_stick.begin_drag(pos)
            return

        for trigger in (self.left_trigger, self.right_trigger):
            if trigger.rect.contains(pos):
                self.active_trigger = trigger
                self.active_trigger.begin_drag(pos)
                return

    def _move_pointer(self, pos: tuple[float, float]):
        if self.active_stick is not None:
            self.active_stick.update_drag(pos)
        if self.active_trigger is not None:
            self.active_trigger.update_drag(pos)

    def _end_pointer(self):
        for button in self.buttons:
            button.release()
        for checkbox in self.checkboxes:
            checkbox.release()
        if self.active_stick is not None:
            self.active_stick.end_drag()
            self.active_stick = None
        if self.active_trigger is not None:
            self.active_trigger.end_drag()
            self.active_trigger = None

    def _normalize_key(self, key_code: int) -> str | None:
        pygame = self.pygame
        key_map = {
            pygame.K_w: 'w',
            pygame.K_a: 'a',
            pygame.K_s: 's',
            pygame.K_d: 'd',
            pygame.K_UP: 'up',
            pygame.K_DOWN: 'down',
            pygame.K_LEFT: 'left',
            pygame.K_RIGHT: 'right',
            pygame.K_TAB: 'tab',
            pygame.K_1: '1',
            pygame.K_2: '2',
            pygame.K_3: '3',
            pygame.K_PLUS: '+',
            pygame.K_EQUALS: '=',
            pygame.K_MINUS: '-',
            pygame.K_UNDERSCORE: '_',
            pygame.K_SPACE: 'space',
            pygame.K_ESCAPE: 'esc',
            pygame.K_DELETE: 'delete',
            pygame.K_BACKSPACE: 'backspace',
        }
        return key_map.get(key_code)

    def _render(self):
        self.screen.fill((21, 24, 28))
        self._draw_text(
            f'Sensitivity {self.node.state.sensitivity:.2f}',
            (775, 88),
            (178, 187, 197),
            self.small_font,
        )

        for button in self.buttons:
            self._draw_button(button)
        for checkbox in self.checkboxes:
            self._draw_checkbox(checkbox)

        axes = self.node.state.axes()
        self._draw_trigger(self.left_trigger, axes.left_trigger)
        self._draw_trigger(self.right_trigger, axes.right_trigger)
        self._draw_stick(self.left_stick, axes.left_x, axes.left_y)
        self._draw_stick(self.right_stick, axes.right_x, axes.right_y)
        if self.show_help:
            self._draw_help()
        self.pygame.display.flip()

    def _draw_button(self, button: ButtonControl):
        selected = button.selected()
        color = (63, 132, 103) if selected else (47, 54, 61)
        if button.pressed:
            color = (150, 80, 68)
        self._draw_rect(button.rect, color, border_radius=6)
        self._draw_rect_outline(button.rect, (111, 126, 140), border_radius=6)
        text_color = (245, 247, 250) if selected or button.pressed else (203, 211, 219)
        self._draw_centered_text(button.label, button.rect, text_color, self.small_font)

    def _draw_checkbox(self, checkbox: CheckboxControl):
        box = RectSpec(checkbox.rect.x, checkbox.rect.y + 4.0, 20.0, 20.0)
        text_pos = (checkbox.rect.x + 28.0, checkbox.rect.y + 5.0)
        border_color = (150, 163, 177) if not checkbox.pressed else (210, 218, 226)
        self._draw_rect(box, (47, 54, 61), border_radius=4)
        self._draw_rect_outline(box, border_color, border_radius=4)
        if checkbox.selected():
            inner = RectSpec(box.x + 5.0, box.y + 5.0, 10.0, 10.0)
            self._draw_rect(inner, (86, 160, 133), border_radius=2)
        self._draw_text(checkbox.label, text_pos, (203, 211, 219), self.small_font)

    def _draw_trigger(self, trigger: TriggerControl, value: float):
        self._draw_text(trigger.name, (trigger.rect.x - 26, trigger.rect.y - 28), (203, 211, 219))
        self._draw_rect(trigger.rect, (45, 51, 58), border_radius=6)
        if trigger.rect.height > trigger.rect.width:
            fill_height = trigger.rect.height * value
            fill = RectSpec(
                trigger.rect.x,
                trigger.rect.y + trigger.rect.height - fill_height,
                trigger.rect.width,
                fill_height,
            )
            knob = (
                round(trigger.rect.x + trigger.rect.width / 2.0),
                round(trigger.rect.y + trigger.rect.height * (1.0 - value)),
            )
        else:
            fill = RectSpec(
                trigger.rect.x,
                trigger.rect.y,
                trigger.rect.width * value,
                trigger.rect.height,
            )
            knob = (
                round(trigger.rect.x + trigger.rect.width * value),
                round(trigger.rect.y + trigger.rect.height / 2.0),
            )
        self._draw_rect(fill, (80, 130, 184), border_radius=6)
        self.pygame.draw.circle(
            self.screen,
            (235, 239, 244),
            knob,
            14,
        )

    def _draw_stick(self, stick: StickControl, x: float, y: float):
        pygame = self.pygame
        pygame.draw.circle(self.screen, (38, 44, 51), stick.center, stick.radius)
        pygame.draw.circle(self.screen, (91, 104, 118), stick.center, stick.radius, 2)
        pygame.draw.line(
            self.screen,
            (62, 70, 80),
            (stick.center[0] - stick.radius, stick.center[1]),
            (stick.center[0] + stick.radius, stick.center[1]),
            1,
        )
        pygame.draw.line(
            self.screen,
            (62, 70, 80),
            (stick.center[0], stick.center[1] - stick.radius),
            (stick.center[0], stick.center[1] + stick.radius),
            1,
        )
        knob = (stick.center[0] + x * stick.radius, stick.center[1] - y * stick.radius)
        pygame.draw.circle(self.screen, (86, 160, 133), knob, 34)
        pygame.draw.circle(self.screen, (235, 239, 244), knob, 34, 2)
        self._draw_text(stick.name, (stick.center[0] - 52, stick.center[1] - stick.radius - 35))
        self._draw_text(
            f'x={x:+.2f} y={y:+.2f}',
            (stick.center[0] - 58, stick.center[1] + stick.radius + 18),
            (178, 187, 197),
            self.small_font,
        )

    def _draw_help(self):
        panel = RectSpec(180.0, 150.0, 390.0, 285.0)
        self._draw_rect(panel, (34, 39, 45), border_radius=6)
        self._draw_rect_outline(panel, (118, 132, 146), border_radius=6)
        self._draw_text('Keyboard Controls', (panel.x + 18, panel.y + 16), (235, 239, 244))
        for index, line in enumerate(KEYBOARD_HELP_LINES):
            self._draw_text(
                line,
                (panel.x + 18, panel.y + 50 + index * 24),
                (203, 211, 219),
                self.small_font,
            )

    def _draw_text(
        self,
        text: str,
        pos: tuple[float, float],
        color: tuple[int, int, int] = (203, 211, 219),
        font=None,
    ):
        surface = (font or self.small_font).render(text, True, color)
        self.screen.blit(surface, pos)

    def _draw_centered_text(self, text: str, rect: RectSpec, color: tuple[int, int, int], font):
        surface = font.render(text, True, color)
        surface_rect = surface.get_rect(center=rect.center)
        self.screen.blit(surface, surface_rect)

    def _draw_rect(
        self,
        rect: RectSpec,
        color: tuple[int, int, int],
        *,
        border_radius: int = 0,
    ):
        self.pygame.draw.rect(
            self.screen,
            color,
            self.pygame.Rect(rect.x, rect.y, rect.width, rect.height),
            border_radius=border_radius,
        )

    def _draw_rect_outline(
        self,
        rect: RectSpec,
        color: tuple[int, int, int],
        *,
        border_radius: int = 0,
    ):
        self.pygame.draw.rect(
            self.screen,
            color,
            self.pygame.Rect(rect.x, rect.y, rect.width, rect.height),
            width=1,
            border_radius=border_radius,
        )
