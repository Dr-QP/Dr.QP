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

"""ROS node publishing MovementCommand messages built from GUI input."""

import argparse

from drqp_interfaces.msg import MovementCommand
from drqp_keyboard_control.control_state import GuiControlState
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.utilities
import std_msgs.msg

EVENT_KEYS = {
    'space': 'kill_switch_pressed',
    'esc': 'kill_switch_pressed',
    'delete': 'reboot_servos',
    'backspace': 'finalize',
}


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
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.balance_mode_pub = self.create_publisher(
            std_msgs.msg.Bool, '/robot/balance_mode', qos_profile=latched_qos
        )
        self.balance_mode_enabled = False
        self.balance_mode_pub.publish(std_msgs.msg.Bool(data=self.balance_mode_enabled))
        self._publish_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_command)
        self.get_logger().info('GUI keyboard control node initialized')

    def handle_key_down(self, key: str) -> bool:
        """Handle a normalized key press from the GUI."""
        normalized_key = key.lower()
        if normalized_key in EVENT_KEYS:
            self.publish_event(EVENT_KEYS[normalized_key])
            return True
        if normalized_key == 'b':
            self.toggle_balance_mode()
            return True
        return self.state.key_down(normalized_key)

    def handle_key_up(self, key: str) -> bool:
        """Handle a normalized key release from the GUI."""
        return self.state.key_up(key)

    def toggle_balance_mode(self):
        """Toggle dedicated balance mode without affecting walking input."""
        self.balance_mode_enabled = not self.balance_mode_enabled
        self.balance_mode_pub.publish(std_msgs.msg.Bool(data=self.balance_mode_enabled))

    def publish_stop_command(self):
        """Publish a zero movement command before the GUI exits."""
        self.state.reset_motion_inputs()
        self._publish_command()

    def publish_event(self, event: str):
        """Publish a named robot event (kill switch, reboot, finalize)."""
        msg = std_msgs.msg.String()
        msg.data = event
        self.robot_event_pub.publish(msg)
        self.get_logger().info(f'Published event: {event}')

    def _publish_command(self):
        self.movement_command_pub.publish(self.state.movement_command())


def main():
    """Entry point for GUI keyboard control node."""
    # Imported lazily so the node stays importable on headless systems.
    from drqp_keyboard_control.keyboard_control_app import PygameKeyboardControlApp

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
            app.close()


if __name__ == '__main__':
    main()
