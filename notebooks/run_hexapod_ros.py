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

from models import HexapodModel
from point import Point3D
from walk_controller import GaitType, WalkController

import sensor_msgs.msg
import rclpy
import rclpy.node
import numpy as np

kFemurOffsetAngle = -13.11
kTibiaOffsetAngle = -32.9


class HexapodController(rclpy.node.Node):
    def __init__(self):
        super().__init__('drqp/hexapod_joint_state_publisher')

        self.direction = Point3D([0, 0, 0])
        self.rotation = 0
        self.walk_speed = 0
        self.rotation_speed = 0

        self.fps = 30
        self.joystick_sub = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.process_inputs, qos_profile=10
        )

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

        self.hexapod.forward_kinematics(0, -25, 110)
        self.walker = WalkController(
            self.hexapod,
            step_length=0.13,  # in meters
            step_height=0.06,  # in meters
            rotation_speed_degrees=25,
            gait=GaitType.ripple,
        )

    def process_inputs(self, joy: sensor_msgs.msg.Joy):
        # Process the state of a joysticks axes and buttons.
        # Header header           # timestamp in the header is the time the data is received from the joystick
        # float32[] axes          # the axes measurements from a joystick
        # int32[] buttons         # the buttons measurements from a joystick
        #
        # Button and axis mappings:
        # https://docs.ros.org/en/ros2_packages/jazzy/api/joy/
        # Index	Button
        # 0	A (CROSS)
        # 1	B (CIRCLE)
        # 2	X (SQUARE)
        # 3	Y (TRIANGLE)
        # 4	BACK (SELECT)
        # 5	GUIDE (Middle/Manufacturer Button)
        # 6	START
        # 7	LEFTSTICK
        # 8	RIGHTSTICK
        # 9	LEFTSHOULDER
        # 10	RIGHTSHOULDER
        # 11	DPAD_UP
        # 12	DPAD_DOWN
        # 13	DPAD_LEFT
        # 14	DPAD_RIGHT
        # 15	MISC1 (Depends on the controller manufacturer, but is usually at a similar location on the controller as back/start)
        # 16	PADDLE1 (Upper left, facing the back of the controller if present)
        # 17	PADDLE2 (Upper right, facing the back of the controller if present)
        # 18	PADDLE3 (Lower left, facing the back of the controller if present)
        # 19	PADDLE4 (Lower right, facing the back of the controller if present)
        # 20	TOUCHPAD (If present. Button status only)
        # Index	Axis
        # 0	LEFTX
        # 1	LEFTY
        # 2	RIGHTX
        # 3	RIGHTY
        # 4	TRIGGERLEFT
        # 5	TRIGGERRIGHT

        left_x = joy.axes[0]
        left_y = joy.axes[1]
        right_x = joy.axes[2]
        self.direction = Point3D([left_x, left_y, 0])
        self.walk_speed = abs(left_x) + abs(left_y)
        self.rotation_speed = right_x

    def loop(self):
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
                msg.name.append(f'dr_qp/{leg.label}_{joint}')
                msg.position.append(np.radians(angle))

        self.joint_state_pub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = HexapodController()
    rclpy.spin(node)
