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

from drqp_brain.joystick_input_handler import all_control_modes, ControlMode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.utilities
import std_msgs.msg

from drqp_keyboard_control.gui_controls import clamp, clamp_vector
from drqp_keyboard_control.keyboard_control_app import PygameKeyboardControlApp

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

    def reset_motion_inputs(self):
        """Clear all latched motion inputs."""
        self.held_keys.clear()
        self.release_left_stick()
        self.release_right_stick()
        self.left_trigger = 0.0
        self.right_trigger = 0.0

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
                y=-axes.left_x,
                z=axes.left_trigger,
            )
            command.rotation_speed = float(-axes.right_x)
        elif self.control_mode == ControlMode.BodyPosition:
            command.body_translation = Vector3(
                x=axes.left_y,
                y=-axes.left_x,
                z=axes.right_y,
            )
        elif self.control_mode == ControlMode.BodyRotation:
            command.body_rotation = Vector3(
                x=-axes.left_x,
                y=axes.left_y,
                z=-axes.right_x,
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

    def publish_stop_command(self):
        """Publish a zero movement command before the GUI exits."""
        self.state.reset_motion_inputs()
        self._publish_command()

    def _publish_event(self, event: str):
        msg = std_msgs.msg.String()
        msg.data = event
        self.robot_event_pub.publish(msg)
        self.get_logger().info(f'Published event: {event}')


def main():
    """Entry point for GUI keyboard control node."""
    node = None
    app = None
    try:
        parser = argparse.ArgumentParser('GUI keyboard robot control ROS node')
        parser.add_argument('--sensitivity', type=float, default=0.5)
        parser.add_argument('--sensitivity-step', type=float, default=0.1)
        parser.add_argument('--publish-rate-hz', type=float, default=20.0)
        parser.add_argument('--width', type=int, default=760)
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
