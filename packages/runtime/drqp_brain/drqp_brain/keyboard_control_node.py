#!/usr/bin/env python3
#
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

import argparse
import curses
from dataclasses import dataclass, field
import select
import sys
import termios
import time
import tty
from typing import TextIO

from drqp_brain.joystick_input_handler import all_control_modes, ControlMode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.utilities
import std_msgs.msg


LEFT_STICK_KEYS = frozenset({'w', 'a', 's', 'd'})
RIGHT_STICK_KEYS = frozenset({'up', 'down', 'left', 'right'})
MOTION_KEYS = LEFT_STICK_KEYS | RIGHT_STICK_KEYS
EVENT_KEYS = {
    'esc': 'kill_switch_pressed',
    'delete': 'reboot_servos',
    'backspace': 'finalize',
}


@dataclass
class KeyboardControlState:
    """Track terminal key input as joystick-like movement state."""

    sensitivity: float = 0.5
    sensitivity_step: float = 0.1
    key_timeout_sec: float = 0.25
    minimum_sensitivity: float = 0.1
    maximum_sensitivity: float = 1.0
    active_until: dict[str, float] = field(default_factory=dict)
    control_mode: ControlMode = ControlMode.Walk
    gait_index: int = 0
    show_detailed_help: bool = False

    gaits = [
        MovementCommandConstants.GAIT_TRIPOD,
        MovementCommandConstants.GAIT_RIPPLE,
        MovementCommandConstants.GAIT_WAVE,
    ]

    def handle_key(self, key: str, now: float | None = None) -> bool:
        """Apply a normalized key press and return whether state changed."""
        if now is None:
            now = time.monotonic()

        key = key.lower()
        if key in MOTION_KEYS:
            self.active_until[key] = now + self.key_timeout_sec
            return True
        if key == 'tab':
            self._next_control_mode()
            return True
        if key in {'+', '='}:
            self._adjust_sensitivity(self.sensitivity_step)
            return True
        if key in {'-', '_'}:
            self._adjust_sensitivity(-self.sensitivity_step)
            return True
        if key in {'1', '2', '3'}:
            self.gait_index = int(key) - 1
            return True
        if key == 'h':
            self.show_detailed_help = not self.show_detailed_help
            return True
        if key in {' ', 'space'}:
            self.active_until.clear()
            return True

        return False

    @property
    def gait(self) -> str:
        """Return the currently selected gait name."""
        return self.gaits[self.gait_index]

    def movement_command(self, now: float | None = None) -> MovementCommand:
        """Build the MovementCommand represented by the current keyboard state."""
        if now is None:
            now = time.monotonic()

        axes = self.axes(now)
        command = MovementCommand()
        command.gait_type = self.gait

        if self.control_mode == ControlMode.Walk:
            command.stride_direction = Vector3(x=axes.left_y, y=axes.left_x, z=0.0)
            command.rotation_speed = float(axes.right_x)
        elif self.control_mode == ControlMode.BodyPosition:
            command.body_translation = Vector3(
                x=axes.left_y,
                y=axes.left_x,
                z=axes.right_y,
            )
        elif self.control_mode == ControlMode.BodyRotation:
            command.body_rotation = Vector3(
                x=axes.left_x,
                y=axes.left_y,
                z=axes.right_x,
            )

        return command

    def axes(self, now: float | None = None) -> 'KeyboardAxes':
        """Return current left/right virtual stick axes."""
        if now is None:
            now = time.monotonic()

        self._expire_keys(now)
        return KeyboardAxes(
            left_x=self._axis('a', 'd'),
            left_y=self._axis('w', 's'),
            right_x=self._axis('left', 'right'),
            right_y=self._axis('up', 'down'),
        )

    def _axis(self, positive_key: str, negative_key: str) -> float:
        value = 0.0
        if positive_key in self.active_until:
            value += self.sensitivity
        if negative_key in self.active_until:
            value -= self.sensitivity
        return max(-1.0, min(1.0, value))

    def _adjust_sensitivity(self, delta: float):
        self.sensitivity = max(
            self.minimum_sensitivity,
            min(self.maximum_sensitivity, self.sensitivity + delta),
        )
        self.sensitivity = round(self.sensitivity, 2)

    def _expire_keys(self, now: float):
        expired_keys = [
            key for key, expires_at in self.active_until.items() if expires_at <= now
        ]
        for key in expired_keys:
            del self.active_until[key]

    def _next_control_mode(self):
        current_index = all_control_modes.index(self.control_mode)
        self.control_mode = all_control_modes[(current_index + 1) % len(all_control_modes)]


@dataclass(frozen=True)
class KeyboardAxes:
    """Virtual joystick axes generated from keyboard input."""

    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0


class TerminalKeyReader:
    """Read normalized key presses from a terminal in cbreak mode."""

    _ESCAPE_KEYS = {
        '\x1b[A': 'up',
        '\x1b[B': 'down',
        '\x1b[C': 'right',
        '\x1b[D': 'left',
        '\x1b[3~': 'delete',
    }

    def __init__(self, stdin: TextIO = sys.stdin):
        self.stdin = stdin
        self._settings = None

    def __enter__(self):
        if self.stdin.isatty():
            self._settings = termios.tcgetattr(self.stdin)
            tty.setcbreak(self.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._settings is not None:
            termios.tcsetattr(self.stdin, termios.TCSADRAIN, self._settings)

    def read_key(self, timeout_sec: float) -> str | None:
        """Return one normalized key press, or None when no key is ready."""
        ready, _, _ = select.select([self.stdin], [], [], timeout_sec)
        if not ready:
            return None

        char = self.stdin.read(1)
        if char == '\x03':
            raise KeyboardInterrupt
        if char == '\t':
            return 'tab'
        if char == '\x1b':
            sequence = char + self._read_available(3)
            return self._ESCAPE_KEYS.get(sequence, 'esc')
        if char == ' ':
            return 'space'
        if char in {'\x08', '\x7f'}:
            return 'backspace'
        return char

    def _read_available(self, limit: int) -> str:
        chars = []
        for _ in range(limit):
            ready, _, _ = select.select([self.stdin], [], [], 0.0)
            if not ready:
                break
            chars.append(self.stdin.read(1))
        return ''.join(chars)


class KeyboardControlNode(rclpy.node.Node):
    """Publish MovementCommand messages from terminal keyboard input."""

    def __init__(
        self,
        *,
        sensitivity: float = 0.5,
        sensitivity_step: float = 0.1,
        key_timeout_sec: float = 0.25,
        publish_rate_hz: float = 20.0,
    ):
        super().__init__('keyboard_control')
        self.state = KeyboardControlState(
            sensitivity=sensitivity,
            sensitivity_step=sensitivity_step,
            key_timeout_sec=key_timeout_sec,
        )

        self.movement_command_pub = self.create_publisher(
            MovementCommand,
            '/robot/movement_command',
            qos_profile=10,
        )
        self.robot_event_pub = self.create_publisher(
            std_msgs.msg.String,
            '/robot_event',
            qos_profile=10,
        )
        self._publish_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_command)
        self.get_logger().info('Keyboard control node initialized')

    def handle_key(self, key: str):
        """Handle a normalized key press from the terminal reader."""
        normalized_key = key.lower()
        if normalized_key in EVENT_KEYS:
            self._publish_event(EVENT_KEYS[normalized_key])
            return True
        return self.state.handle_key(normalized_key)

    def ui_lines(self, now: float | None = None) -> list[str]:
        """Return the current terminal UI as plain text lines."""
        if now is None:
            now = time.monotonic()
        axes = self.state.axes(now)
        lines = [
            'Dr.QP Keyboard Control',
            '',
            f'Mode: {self.state.control_mode.name}',
            f'Gait: {self.state.gait}   Sensitivity: {self.state.sensitivity:.2f}',
            (
                'Left stick WASD: '
                f'x={axes.left_x:+.2f} y={axes.left_y:+.2f}   '
                'Right stick arrows: '
                f'x={axes.right_x:+.2f} y={axes.right_y:+.2f}'
            ),
        ]

        if self.state.show_detailed_help:
            lines.extend(
                [
                    '',
                    'Key bindings:',
                    '  W/S forward/back, A/D left/right',
                    '  Arrow keys control the right stick',
                    '  +/- adjust sensitivity',
                    '  1 tripod, 2 ripple, 3 wave',
                    '  TAB toggles mode, H hides help',
                    '  Esc kill switch, Del reboot servos, Backspace finalize',
                    '  Space stops active keys, Ctrl-C exits',
                    '',
                    'Mode behavior:',
                    '  Walk: WASD controls stride, left/right arrows rotate',
                    '  BodyPosition: WASD moves body x/y, up/down arrows move body z',
                    '  BodyRotation: WASD controls roll/pitch, left/right arrows yaw',
                    '',
                    'Terminal notes:',
                    '  Holding a key relies on keyboard repeat.',
                    '  Motion returns to zero shortly after key repeat stops.',
                ]
            )
        else:
            lines.extend(['', 'Press H for help'])

        return lines

    def _publish_command(self):
        self.movement_command_pub.publish(self.state.movement_command())

    def _publish_event(self, event: str):
        msg = std_msgs.msg.String()
        msg.data = event
        self.robot_event_pub.publish(msg)
        self.get_logger().info(f'Published event: {event}')


class KeyboardControlScreen:
    """Curses terminal UI for keyboard control."""

    _SPECIAL_KEYS = {
        curses.KEY_UP: 'up',
        curses.KEY_DOWN: 'down',
        curses.KEY_LEFT: 'left',
        curses.KEY_RIGHT: 'right',
        curses.KEY_DC: 'delete',
        curses.KEY_BACKSPACE: 'backspace',
        9: 'tab',
        27: 'esc',
        32: 'space',
        127: 'backspace',
    }

    def __init__(
        self,
        stdscr,
        node: KeyboardControlNode,
        *,
        refresh_rate_hz: float,
    ):
        self.stdscr = stdscr
        self.node = node
        self.refresh_period = 1.0 / refresh_rate_hz
        self.last_render_at = 0.0
        self.screen_attr = curses.A_NORMAL

    def run(self):
        """Run the curses input and redraw loop."""
        curses.noecho()
        curses.cbreak()
        self._configure_screen()
        self.stdscr.keypad(True)
        self.stdscr.timeout(20)
        try:
            curses.curs_set(0)
        except curses.error:
            pass

        self.render(force=True)
        while rclpy.ok():
            key = self.read_key()
            if key is not None and self.node.handle_key(key):
                self.render(force=True)

            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.render()

    def read_key(self) -> str | None:
        """Read one normalized key from curses."""
        key_code = self.stdscr.getch()
        if key_code == -1:
            return None
        if key_code == 3:
            raise KeyboardInterrupt
        if key_code in self._SPECIAL_KEYS:
            return self._SPECIAL_KEYS[key_code]
        if 0 <= key_code <= 255:
            return chr(key_code)
        return None

    def _configure_screen(self):
        """Configure terminal colors for a full-screen black background."""
        try:
            curses.start_color()
            curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
            self.screen_attr = curses.color_pair(1)
        except curses.error:
            self.screen_attr = curses.A_NORMAL

    def render(self, *, force: bool = False):
        """Redraw the screen without producing scrollback."""
        now = time.monotonic()
        if not force and now - self.last_render_at < self.refresh_period:
            return

        self.last_render_at = now
        height, width = self.stdscr.getmaxyx()
        self.stdscr.bkgd(' ', self.screen_attr)
        self.stdscr.erase()
        for row, line in enumerate(self.node.ui_lines(now)):
            if row >= height:
                break
            self.stdscr.addnstr(row, 0, line, max(0, width - 1), self.screen_attr)
        self.stdscr.noutrefresh()
        curses.doupdate()


def _run_curses_keyboard_control(
    stdscr,
    node: KeyboardControlNode,
    refresh_rate_hz: float,
):
    KeyboardControlScreen(
        stdscr,
        node,
        refresh_rate_hz=refresh_rate_hz,
    ).run()


def _run_plain_keyboard_control(node: KeyboardControlNode):
    with TerminalKeyReader() as key_reader:
        while rclpy.ok():
            key = key_reader.read_key(timeout_sec=0.02)
            if key is not None:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)


def main():
    """Entry point for keyboard control node."""
    node = None
    try:
        parser = argparse.ArgumentParser('Terminal keyboard robot control ROS node')
        parser.add_argument('--sensitivity', type=float, default=0.5)
        parser.add_argument('--sensitivity-step', type=float, default=0.1)
        parser.add_argument('--key-timeout-sec', type=float, default=0.1)
        parser.add_argument('--publish-rate-hz', type=float, default=20.0)
        parser.add_argument('--ui-refresh-rate-hz', type=float, default=8.0)
        parser.add_argument('--no-ui', action='store_true')
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])

        rclpy.init()
        node = KeyboardControlNode(
            sensitivity=args.sensitivity,
            sensitivity_step=args.sensitivity_step,
            key_timeout_sec=args.key_timeout_sec,
            publish_rate_hz=args.publish_rate_hz,
        )

        if args.no_ui:
            _run_plain_keyboard_control(node)
        else:
            curses.wrapper(
                _run_curses_keyboard_control,
                node,
                args.ui_refresh_rate_hz,
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
