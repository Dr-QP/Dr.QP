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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import std_msgs.msg


class RobotStateMachine(StateMachine):
    """A robot state machine."""

    torque_off = State(initial=True)
    initializing = State()
    torque_on = State()
    finalizing = State()
    finalized = State(final=True)

    initialize = torque_off.to(initializing)
    turn_on = initializing.to(torque_on)
    turn_off = torque_on.to(torque_off)
    finalize = torque_on.to(finalizing)
    done = finalizing.to(finalized)


class RobotState(Node):
    def __init__(self):
        super().__init__('drqp_robot_state')

        self.state_pub = self.create_publisher(std_msgs.msg.String, '/robot_state', qos_profile=1)
        self.robot_state_machine = RobotStateMachine()

        self.robot_state_machine.add_listener(self)
        # self.robot_state_machine.add_state_changed_listener(self.state_changed)

        self.robot_state_machine.activate_initial_state()

    def on_enter_state(self, target_state: State):
        msg = std_msgs.msg.String()
        msg.data = target_state.name
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotState()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
