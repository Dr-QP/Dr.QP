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
import sys
from models import HexapodModel
import numpy as np
from point import Point3D
import rclpy
import rclpy.node
import sensor_msgs.msg
from walk_controller import GaitType, WalkController
from enum import Enum, auto

kFemurOffsetAngle = -13.11
kTibiaOffsetAngle = -32.9


class ButtonState(Enum):
    Released = 0  # match ROS joy states
    Pressed = 1  # match ROS joy states


class ButtonEvent(Enum):
    Tapped = auto()


# https://docs.ros.org/en/ros2_packages/jazzy/api/joy/
class ButtonIndex(Enum):
    Cross = 0
    Circle = 1
    Square = 2
    Triangle = 3
    Select = 4
    PS = 5
    Start = 6
    LeftStick = 7
    RightStick = 8
    L1 = 9
    R1 = 10
    DpadUp = 11
    DpadDown = 12
    DpadLeft = 13
    DpadRight = 14
    TouchpadButton = 15


class ButtonAxis(Enum):
    LeftX = 0
    LeftY = 1
    RightX = 2
    RightY = 3
    TriggerLeft = 4
    TriggerRight = 5


class JoystickButton:
    def __init__(self, button_index: ButtonIndex, event_handler: callable):
        self.button_index = button_index
        self.event_handler = event_handler

        self.current_state = ButtonState.Released
        self.last_state = ButtonState.Released

    def update(self, joy_buttons_array):
        self.last_state = self.current_state
        self.current_state = ButtonState(joy_buttons_array[self.button_index.value])

        if not self.event_handler:
            return

        if self.last_state == ButtonState.Released and self.current_state == ButtonState.Pressed:
            self.event_handler(self, ButtonEvent.Tapped)


class HexapodController(rclpy.node.Node):
    def __init__(self):
        super().__init__('drqp_hexapod_joint_state_publisher')

        self.direction = Point3D([0, 0, 0])
        self.rotation = 0
        self.walk_speed = 0
        self.rotation_speed = 0

        self.fps = 30

        self.gait_index = 0
        self.gaits = [GaitType.tripod, GaitType.ripple, GaitType.wave]
        self.phase_steps_per_cycle = [20, 25, 40]  # per gait

        self.joystick_sub = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.process_inputs, qos_profile=10
        )
        self.joystick_buttons = [
            JoystickButton(ButtonIndex.L1, lambda _: self.prev_gait()),
            JoystickButton(ButtonIndex.R1, lambda _: self.next_gait()),
        ]

        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/joint_states', qos_profile=50
        )
        self.setup_hexapod()

        self.timer = self.create_timer(1 / self.fps, self.loop)

    def setup_hexapod(self):
        drqp_coxa = 0.053  # in meters
        drqp_femur = 0.066225  # in meters
        drqp_tibia = 0.123  # in meters

        drqp_front_offset = 0.116924  # x offset for the front and back legs in meters
        drqp_side_offset = 0.063871  # y offset fo the front and back legs
        drqp_middle_offset = 0.103  # x offset for the middle legs

        self.hexapod = HexapodModel(
            coxa_len=drqp_coxa,
            femur_len=drqp_femur,
            tibia_len=drqp_tibia,
            front_offset=drqp_front_offset,
            middle_offset=drqp_middle_offset,
            side_offset=drqp_side_offset,
            leg_rotation=[0, 0, 45],
        )

        # self.hexapod.forward_kinematics(0, 55, 50)  # bulldog
        # step_length = 0.10  # in meters
        # step_height = 0.06  # in meters

        # self.hexapod.forward_kinematics(0, -25, 110)  # default sim
        self.hexapod.forward_kinematics(
            0, -35, 130
        )  # reasonable hexa, servos out of reach for 0.06 height
        step_length = 0.14  # in meters
        step_height = 0.03  # in meters

        self.walker = WalkController(
            self.hexapod,
            step_length=step_length,
            step_height=step_height,
            rotation_speed_degrees=20,
            gait=self.gaits[self.gait_index],
            phase_steps_per_cycle=self.fps / 2.5,
        )

    def process_inputs(self, joy: sensor_msgs.msg.Joy):
        left_x = joy.axes[ButtonAxis.LeftX.value]
        left_y = joy.axes[ButtonAxis.LeftY.value]
        right_x = joy.axes[ButtonAxis.RightX.value]
        left_trigger = np.interp(joy.axes[ButtonAxis.TriggerLeft.value], [-1, 1], [1, 0])
        self.direction = Point3D([left_y, left_x, left_trigger])
        self.walk_speed = abs(left_x) + abs(left_y) + abs(left_trigger)
        self.rotation_speed = right_x

        for button in self.joystick_buttons:
            button.update(joy.buttons)

    def prev_gait(self):
        self.gait_index = (self.gait_index - 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def next_gait(self):
        self.gait_index = (self.gait_index + 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def loop(self):
        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.phase_step = 1 / self.phase_steps_per_cycle[self.gait_index]
        self.walker.next(
            stride_direction=self.direction,
            stride_ratio=self.walk_speed,
            rotation_ratio=self.rotation_speed,
        )
        self.publish_joint_states()

    def publish_joint_states(self):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for leg in self.hexapod.legs:
            for joint, angle in [
                ('coxa', leg.coxa_angle),
                ('femur', leg.femur_angle + kFemurOffsetAngle),
                ('tibia', leg.tibia_angle + kTibiaOffsetAngle),
            ]:
                msg.name.append(f'dr_qp/{leg.label.name}_{joint}')
                msg.position.append(np.radians(angle))

        self.joint_state_pub.publish(msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Dr.QP Robot controller ROS node')
    filtered_args = rclpy.utilities.remove_ros_args()
    args = parser.parse_args(args=filtered_args[1:])
    rclpy.init()
    node = HexapodController(**vars(args))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # codeql[py/empty-except]
    finally:
        node.destroy_node()
        rclpy.shutdown()
