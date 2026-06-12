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
from dataclasses import dataclass, field
import math
from typing import Callable

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
    'space': 'kill_switch_pressed',
    'esc': 'kill_switch_pressed',
    'delete': 'reboot_servos',
    'backspace': 'finalize',
}


@dataclass(frozen=True)
class VirtualAxes:
    """Virtual controller axes used to build movement commands."""

    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0


@dataclass
class GuiControlState:
    """Track GUI and keyboard input as joystick-like movement state."""

    sensitivity: float = 0.5
    sensitivity_step: float = 0.1
    minimum_sensitivity: float = 0.1
    maximum_sensitivity: float = 1.0
    held_keys: set[str] = field(default_factory=set)
    control_mode: ControlMode = ControlMode.Walk
    gait_index: int = 0
    left_stick: tuple[float, float] = (0.0, 0.0)
    right_stick: tuple[float, float] = (0.0, 0.0)
    left_stick_active: bool = False
    right_stick_active: bool = False
    left_trigger: float = 0.0
    right_trigger: float = 0.0

    gaits = [
        MovementCommandConstants.GAIT_TRIPOD,
        MovementCommandConstants.GAIT_RIPPLE,
        MovementCommandConstants.GAIT_WAVE,
    ]

    def key_down(self, key: str) -> bool:
        """Apply a normalized key press and return whether state changed."""
        key = key.lower()
        if key in MOTION_KEYS:
            before = set(self.held_keys)
            self.held_keys.add(key)
            return before != self.held_keys
        if key == 'tab':
            self.next_control_mode()
            return True
        if key in {'+', '='}:
            self.adjust_sensitivity(self.sensitivity_step)
            return True
        if key in {'-', '_'}:
            self.adjust_sensitivity(-self.sensitivity_step)
            return True
        if key in {'1', '2', '3'}:
            self.set_gait_index(int(key) - 1)
            return True
        return False

    def key_up(self, key: str) -> bool:
        """Apply a normalized key release and return whether state changed."""
        key = key.lower()
        if key not in MOTION_KEYS or key not in self.held_keys:
            return False
        self.held_keys.remove(key)
        return True

    @property
    def gait(self) -> str:
        """Return the currently selected gait name."""
        return self.gaits[self.gait_index]

    def set_gait_index(self, gait_index: int):
        """Set selected gait by index."""
        self.gait_index = max(0, min(len(self.gaits) - 1, gait_index))

    def set_control_mode(self, control_mode: ControlMode):
        """Set the current semantic control mode."""
        self.control_mode = control_mode

    def next_control_mode(self):
        """Cycle through the same control modes as the joystick translator."""
        current_index = all_control_modes.index(self.control_mode)
        self.control_mode = all_control_modes[(current_index + 1) % len(all_control_modes)]

    def adjust_sensitivity(self, delta: float):
        """Adjust keyboard emulation magnitude."""
        self.sensitivity = clamp(
            self.sensitivity + delta,
            self.minimum_sensitivity,
            self.maximum_sensitivity,
        )
        self.sensitivity = round(self.sensitivity, 2)

    def set_left_stick(self, x: float, y: float, *, active: bool = True):
        """Set left virtual stick axes."""
        self.left_stick = clamp_vector(x, y)
        self.left_stick_active = active

    def release_left_stick(self):
        """Return left virtual stick to center."""
        self.left_stick = (0.0, 0.0)
        self.left_stick_active = False

    def set_right_stick(self, x: float, y: float, *, active: bool = True):
        """Set right virtual stick axes."""
        self.right_stick = clamp_vector(x, y)
        self.right_stick_active = active

    def release_right_stick(self):
        """Return right virtual stick to center."""
        self.right_stick = (0.0, 0.0)
        self.right_stick_active = False

    def set_left_trigger(self, value: float):
        """Set left trigger slider value."""
        self.left_trigger = clamp(value, 0.0, 1.0)

    def set_right_trigger(self, value: float):
        """Set right trigger slider value."""
        self.right_trigger = clamp(value, 0.0, 1.0)

    def axes(self) -> VirtualAxes:
        """Return merged keyboard and pointer controller axes."""
        keyboard_left = (
            self._axis('d', 'a') * self.sensitivity,
            self._axis('w', 's') * self.sensitivity,
        )
        keyboard_right = (
            self._axis('right', 'left') * self.sensitivity,
            self._axis('up', 'down') * self.sensitivity,
        )
        left_x, left_y = self.left_stick if self.left_stick_active else keyboard_left
        right_x, right_y = self.right_stick if self.right_stick_active else keyboard_right
        return VirtualAxes(
            left_x=left_x,
            left_y=left_y,
            right_x=right_x,
            right_y=right_y,
            left_trigger=self.left_trigger,
            right_trigger=self.right_trigger,
        )

    def movement_command(self) -> MovementCommand:
        """Build the MovementCommand represented by the current GUI state."""
        axes = self.axes()
        command = MovementCommand()
        command.gait_type = self.gait

        if self.control_mode == ControlMode.Walk:
            command.stride_direction = Vector3(
                x=axes.left_y,
                y=axes.left_x,
                z=axes.left_trigger,
            )
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

    def _axis(self, positive_key: str, negative_key: str) -> float:
        value = 0.0
        if positive_key in self.held_keys:
            value += 1.0
        if negative_key in self.held_keys:
            value -= 1.0
        return value


class KeyboardControlNode(rclpy.node.Node):
    """Publish MovementCommand messages from GUI keyboard input."""

    def __init__(
        self,
        *,
        sensitivity: float = 0.5,
        sensitivity_step: float = 0.1,
        publish_rate_hz: float = 20.0,
    ):
        super().__init__('keyboard_control')
        self.state = GuiControlState(
            sensitivity=sensitivity,
            sensitivity_step=sensitivity_step,
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
        self.get_logger().info('GUI keyboard control node initialized')

    def handle_key_down(self, key: str) -> bool:
        """Handle a normalized key press from the GUI."""
        normalized_key = key.lower()
        if normalized_key in EVENT_KEYS:
            self._publish_event(EVENT_KEYS[normalized_key])
            return True
        return self.state.key_down(normalized_key)

    def handle_key_up(self, key: str) -> bool:
        """Handle a normalized key release from the GUI."""
        return self.state.key_up(key)

    def _publish_command(self):
        self.movement_command_pub.publish(self.state.movement_command())

    def _publish_event(self, event: str):
        msg = std_msgs.msg.String()
        msg.data = event
        self.robot_event_pub.publish(msg)
        self.get_logger().info(f'Published event: {event}')


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into inclusive range."""
    return max(minimum, min(maximum, value))


def clamp_vector(x: float, y: float) -> tuple[float, float]:
    """Clamp a 2D vector into the unit circle."""
    magnitude = math.hypot(x, y)
    if magnitude <= 1.0:
        return (x, y)
    return (x / magnitude, y / magnitude)


@dataclass
class RectSpec:
    """Small rectangle helper that keeps most GUI math Pygame-free."""

    x: float
    y: float
    width: float
    height: float

    def contains(self, pos: tuple[float, float]) -> bool:
        px, py = pos
        return self.x <= px <= self.x + self.width and self.y <= py <= self.y + self.height

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


@dataclass
class StickControl:
    """Draggable circular virtual thumb stick."""

    name: str
    center: tuple[float, float]
    radius: float
    setter: Callable[[float, float], None]
    releaser: Callable[[], None]
    dragging: bool = False

    def hit_test(self, pos: tuple[float, float]) -> bool:
        return math.hypot(pos[0] - self.center[0], pos[1] - self.center[1]) <= self.radius

    def begin_drag(self, pos: tuple[float, float]):
        self.dragging = True
        self.update_drag(pos)

    def update_drag(self, pos: tuple[float, float]):
        x = (pos[0] - self.center[0]) / self.radius
        y = -(pos[1] - self.center[1]) / self.radius
        self.setter(*clamp_vector(x, y))

    def end_drag(self):
        self.dragging = False
        self.releaser()


@dataclass
class TriggerControl:
    """Horizontal trigger slider."""

    name: str
    rect: RectSpec
    setter: Callable[[float], None]
    dragging: bool = False

    def begin_drag(self, pos: tuple[float, float]):
        if not self.rect.contains(pos):
            return
        self.dragging = True
        self.update_drag(pos)

    def update_drag(self, pos: tuple[float, float]):
        if self.rect.height > self.rect.width:
            value = 1.0 - ((pos[1] - self.rect.y) / self.rect.height)
        else:
            value = (pos[0] - self.rect.x) / self.rect.width
        self.setter(clamp(value, 0.0, 1.0))

    def end_drag(self):
        self.dragging = False


@dataclass
class ButtonControl:
    """Clickable GUI button."""

    label: str
    rect: RectSpec
    action: Callable[[], None]
    selected: Callable[[], bool] = lambda: False
    pressed: bool = False

    def click(self, pos: tuple[float, float]) -> bool:
        if not self.rect.contains(pos):
            return False
        self.pressed = True
        self.action()
        return True

    def release(self):
        self.pressed = False


class PygameKeyboardControlApp:
    """Pygame GUI wrapper around KeyboardControlNode."""

    def __init__(
        self,
        node: KeyboardControlNode,
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

        pygame.init()
        pygame.display.set_caption('Dr.QP Keyboard Control')
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.small_font = pygame.font.Font(None, 22)

        self.left_stick = StickControl(
            'Left Stick',
            (240.0, 385.0),
            105.0,
            self.node.state.set_left_stick,
            self.node.state.release_left_stick,
        )
        self.right_stick = StickControl(
            'Right Stick',
            (740.0, 385.0),
            105.0,
            self.node.state.set_right_stick,
            self.node.state.release_right_stick,
        )
        self.left_trigger = TriggerControl(
            'Left Trigger',
            RectSpec(75.0, 280.0, 34.0, 210.0),
            self.node.state.set_left_trigger,
        )
        self.right_trigger = TriggerControl(
            'Right Trigger',
            RectSpec(870.0, 280.0, 34.0, 210.0),
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
        mode_x = 275.0
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
                    RectSpec(305.0 + index * 130.0, 560.0, 118.0, 42.0),
                    action,
                )
            )

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
        pygame = self.pygame
        self.screen.fill((21, 24, 28))
        self._draw_text(
            f'Sensitivity {self.node.state.sensitivity:.2f}',
            (775, 88),
            (178, 187, 197),
            self.small_font,
        )

        for button in self.buttons:
            self._draw_button(button)

        axes = self.node.state.axes()
        self._draw_trigger(self.left_trigger, axes.left_trigger)
        self._draw_trigger(self.right_trigger, axes.right_trigger)
        self._draw_stick(self.left_stick, axes.left_x, axes.left_y)
        self._draw_stick(self.right_stick, axes.right_x, axes.right_y)
        pygame.display.flip()

    def _draw_button(self, button: ButtonControl):
        selected = button.selected()
        color = (63, 132, 103) if selected else (47, 54, 61)
        if button.pressed:
            color = (150, 80, 68)
        self._draw_rect(button.rect, color, border_radius=6)
        self._draw_rect_outline(button.rect, (111, 126, 140), border_radius=6)
        text_color = (245, 247, 250) if selected or button.pressed else (203, 211, 219)
        self._draw_centered_text(button.label, button.rect, text_color, self.small_font)

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


def main():
    """Entry point for GUI keyboard control node."""
    node = None
    app = None
    try:
        parser = argparse.ArgumentParser('GUI keyboard robot control ROS node')
        parser.add_argument('--sensitivity', type=float, default=0.5)
        parser.add_argument('--sensitivity-step', type=float, default=0.1)
        parser.add_argument('--publish-rate-hz', type=float, default=20.0)
        parser.add_argument('--width', type=int, default=980)
        parser.add_argument('--height', type=int, default=640)
        parser.add_argument('--frame-rate-hz', type=float, default=60.0)
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])

        rclpy.init()
        node = KeyboardControlNode(
            sensitivity=args.sensitivity,
            sensitivity_step=args.sensitivity_step,
            publish_rate_hz=args.publish_rate_hz,
        )
        app = PygameKeyboardControlApp(
            node,
            width=args.width,
            height=args.height,
            frame_rate_hz=args.frame_rate_hz,
        )
        app.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if app is not None:
            app.pygame.quit()


if __name__ == '__main__':
    main()
