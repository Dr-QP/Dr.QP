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

"""Composition root wiring the Pygame window, widgets, router, and renderer."""

from drqp_keyboard_control.gui_controls import PointerRouter
from drqp_keyboard_control.keymap import build_key_map
from drqp_keyboard_control.renderer import PygameRenderer
from drqp_keyboard_control.sdl_window import set_sdl_window_always_on_top
from drqp_keyboard_control.ui import apply_layout, build_controls, KEYBOARD_HELP_LINES
import rclpy


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
        self.frame_rate_hz = frame_rate_hz
        self.running = True
        self.show_help = False
        self.stay_on_top = False

        pygame.init()
        pygame.display.set_caption('Dr.QP Keyboard Control')
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        self.window_id = self._display_window_id()
        self.clock = pygame.time.Clock()
        self.key_map = build_key_map(pygame)
        self.renderer = PygameRenderer(pygame, self.screen)

        self.controls = build_controls(
            node,
            toggle_help=self._toggle_help,
            help_visible=lambda: self.show_help,
            toggle_stay_on_top=self._toggle_stay_on_top,
            stay_on_top_enabled=lambda: self.stay_on_top,
        )
        self.router = PointerRouter(self.controls)
        apply_layout(self.controls, width, height)

    def run(self):
        """Run the GUI and ROS event loops."""
        while self.running and rclpy.ok():
            self._handle_events()
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._render()
            self.clock.tick(self.frame_rate_hz)

    def close(self):
        """Shut Pygame down; safe to call after run() returns."""
        self.pygame.quit()

    def _handle_events(self):
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._request_shutdown()
            elif event.type == pygame.KEYDOWN:
                key = self.key_map.get(event.key)
                if key is not None:
                    self.node.handle_key_down(key)
            elif event.type == pygame.KEYUP:
                key = self.key_map.get(event.key)
                if key is not None:
                    self.node.handle_key_up(key)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self.router.press(event.pos)
            elif event.type == pygame.MOUSEMOTION:
                self.router.move(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.router.release()
            elif event.type == pygame.VIDEORESIZE:
                apply_layout(self.controls, event.w, event.h)

    def _render(self):
        help_lines = KEYBOARD_HELP_LINES if self.show_help else None
        self.renderer.render(self.controls, help_lines=help_lines)

    def _toggle_help(self):
        self.show_help = not self.show_help

    def _toggle_stay_on_top(self):
        self.stay_on_top = not self.stay_on_top
        self._apply_stay_on_top(self.stay_on_top)

    def _apply_stay_on_top(self, enabled: bool) -> bool:
        return set_sdl_window_always_on_top(self.window_id, enabled)

    def _request_shutdown(self):
        self.node.publish_stop_command()
        self.running = False

    def _display_window_id(self) -> int | None:
        try:
            window_id = self.pygame.display.get_wm_info().get('window')
            if window_id is None:
                return None
            return int(window_id)
        except (AttributeError, TypeError, ValueError):
            return None
