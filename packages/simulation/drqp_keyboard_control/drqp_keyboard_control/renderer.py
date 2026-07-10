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

"""Pygame drawing of the widget models; the only module that paints pixels."""

from drqp_keyboard_control.gui_controls import (
    ButtonControl,
    CheckboxControl,
    Control,
    LabelControl,
    StickControl,
    TriggerControl,
)
from drqp_keyboard_control.layout import RectSpec
from drqp_keyboard_control.theme import DEFAULT_THEME, Theme

CHECKBOX_BOX_SIZE = 20.0
TEXT_GAP = 8.0

HELP_PANEL_WIDTH = 390.0
HELP_PANEL_PADDING = 18.0
HELP_TITLE_HEIGHT = 34.0
HELP_LINE_HEIGHT = 24.0


class PygameRenderer:
    """Draw widgets and overlays onto a Pygame surface using a theme."""

    def __init__(self, pygame_module, screen, theme: Theme = DEFAULT_THEME):
        self.pygame = pygame_module
        self.screen = screen
        self.theme = theme
        self.font = pygame_module.font.Font(None, theme.font_size)
        self.small_font = pygame_module.font.Font(None, theme.small_font_size)

    def render(self, controls: list[Control], *, help_lines: list[str] | None = None):
        """Draw a full frame: background, widgets, and optional help overlay."""
        self.screen.fill(self.theme.background)
        for control in controls:
            self._draw_control(control)
        if help_lines:
            self._draw_help(help_lines)
        self.pygame.display.flip()

    def _draw_control(self, control: Control):
        if isinstance(control, CheckboxControl):
            self._draw_checkbox(control)
        elif isinstance(control, ButtonControl):
            self._draw_button(control)
        elif isinstance(control, StickControl):
            self._draw_stick(control)
        elif isinstance(control, TriggerControl):
            self._draw_trigger(control)
        elif isinstance(control, LabelControl):
            self._draw_label(control)

    def _draw_button(self, button: ButtonControl):
        theme = self.theme
        selected = button.selected()
        color = theme.button_selected if selected else theme.button_fill
        if button.pressed:
            color = theme.button_pressed
        self._fill_rect(button.rect, color, radius=theme.button_radius)
        self._outline_rect(button.rect, theme.button_border, radius=theme.button_radius)
        text_color = theme.text_primary if selected or button.pressed else theme.text_normal
        self._text_centered(button.label, button.rect.center, self.small_font, text_color)

    def _draw_checkbox(self, checkbox: CheckboxControl):
        theme = self.theme
        center_x, center_y = checkbox.rect.center
        box = RectSpec(
            checkbox.rect.x,
            center_y - CHECKBOX_BOX_SIZE / 2.0,
            CHECKBOX_BOX_SIZE,
            CHECKBOX_BOX_SIZE,
        )
        border = theme.checkbox_border_pressed if checkbox.pressed else theme.checkbox_border
        self._fill_rect(box, theme.button_fill, radius=theme.checkbox_radius)
        self._outline_rect(box, border, radius=theme.checkbox_radius)
        if checkbox.selected():
            mark = RectSpec(box.x + 5.0, box.y + 5.0, box.width - 10.0, box.height - 10.0)
            self._fill_rect(mark, theme.checkbox_mark, radius=2)
        label_pos = (box.x + box.width + TEXT_GAP, center_y)
        self._text(checkbox.label, label_pos, self.small_font, theme.text_normal, anchor='midleft')

    def _draw_stick(self, stick: StickControl):
        pygame = self.pygame
        theme = self.theme
        center = stick.center
        radius = stick.radius

        pygame.draw.circle(self.screen, theme.stick_fill, center, radius)
        pygame.draw.circle(self.screen, theme.stick_border, center, radius, 2)
        for start, end in (
            ((center[0] - radius, center[1]), (center[0] + radius, center[1])),
            ((center[0], center[1] - radius), (center[0], center[1] + radius)),
        ):
            pygame.draw.line(self.screen, theme.stick_axis, start, end, 1)

        x, y = stick.value()
        knob = (center[0] + x * radius, center[1] - y * radius)
        pygame.draw.circle(self.screen, theme.stick_knob, knob, theme.stick_knob_radius)
        pygame.draw.circle(self.screen, theme.stick_knob_border, knob, theme.stick_knob_radius, 2)

        name_pos = (center[0], center[1] - radius - TEXT_GAP)
        self._text(stick.label, name_pos, self.font, theme.text_normal, anchor='midbottom')
        value_pos = (center[0], center[1] + radius + TEXT_GAP)
        readout = f'x={x:+.2f} y={y:+.2f}'
        self._text(readout, value_pos, self.small_font, theme.text_muted, anchor='midtop')

    def _draw_trigger(self, trigger: TriggerControl):
        pygame = self.pygame
        theme = self.theme
        track = trigger.track_rect
        value = trigger.value()

        label_pos = (trigger.rect.center[0], trigger.rect.y)
        self._text(trigger.label, label_pos, self.small_font, theme.text_normal, anchor='midtop')
        self._fill_rect(track, theme.trigger_track, radius=theme.button_radius)

        if trigger.vertical:
            fill_height = track.height * value
            fill_top = track.y + track.height - fill_height
            fill = RectSpec(track.x, fill_top, track.width, fill_height)
            knob = (track.center[0], track.y + track.height * (1.0 - value))
        else:
            fill = RectSpec(track.x, track.y, track.width * value, track.height)
            knob = (track.x + track.width * value, track.center[1])
        self._fill_rect(fill, theme.trigger_fill, radius=theme.button_radius)
        pygame.draw.circle(self.screen, theme.trigger_knob, knob, theme.trigger_knob_radius)

    def _draw_label(self, label: LabelControl):
        pos = (label.rect.x, label.rect.center[1])
        self._text(label.text(), pos, self.small_font, self.theme.text_muted, anchor='midleft')

    def _draw_help(self, lines: list[str]):
        theme = self.theme
        screen_width, screen_height = self.screen.get_size()
        panel_height = HELP_TITLE_HEIGHT + len(lines) * HELP_LINE_HEIGHT
        panel_height += 2.0 * HELP_PANEL_PADDING
        panel = RectSpec(
            (screen_width - HELP_PANEL_WIDTH) / 2.0,
            (screen_height - panel_height) / 2.0,
            HELP_PANEL_WIDTH,
            panel_height,
        )
        self._fill_rect(panel, theme.panel_fill, radius=theme.button_radius)
        self._outline_rect(panel, theme.panel_border, radius=theme.button_radius)

        text_x = panel.x + HELP_PANEL_PADDING
        title_pos = (text_x, panel.y + HELP_PANEL_PADDING)
        self._text('Keyboard Controls', title_pos, self.font, theme.text_primary)
        for index, line in enumerate(lines):
            line_y = panel.y + HELP_PANEL_PADDING + HELP_TITLE_HEIGHT
            line_y += index * HELP_LINE_HEIGHT
            self._text(line, (text_x, line_y), self.small_font, theme.text_normal)

    def _text(self, text, pos, font, color, *, anchor: str = 'topleft'):
        surface = font.render(text, True, color)
        self.screen.blit(surface, surface.get_rect(**{anchor: pos}))

    def _text_centered(self, text, center, font, color):
        self._text(text, center, font, color, anchor='center')

    def _fill_rect(self, rect: RectSpec, color, *, radius: int = 0):
        self.pygame.draw.rect(self.screen, color, self._to_pygame(rect), border_radius=radius)

    def _outline_rect(self, rect: RectSpec, color, *, radius: int = 0):
        self.pygame.draw.rect(
            self.screen, color, self._to_pygame(rect), width=1, border_radius=radius
        )

    def _to_pygame(self, rect: RectSpec):
        return self.pygame.Rect(rect.x, rect.y, rect.width, rect.height)
