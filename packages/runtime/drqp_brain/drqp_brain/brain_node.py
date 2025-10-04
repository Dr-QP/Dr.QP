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

from control_msgs.action import FollowJointTrajectory
from drqp_brain.joint_trajectory_builder import JointTrajectoryBuilder
from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.joystick_input_handler import JoystickInputHandler
from drqp_brain.models import HexapodModel
from drqp_brain.walk_controller import GaitType, WalkController
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.utilities
import sensor_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg


class HexapodBrain(rclpy.node.Node):
    """
    ROS node for controlling Dr.QP hexapod robot.

    Subscribes to /joy topic for joystick input, processes it with WalkController,
    and publishes positions to /servo_goals topic.

    """

    def __init__(self):
        super().__init__('drqp_brain')

        self.fps = 30

        self.gait_index = 0
        self.gaits = [GaitType.tripod, GaitType.ripple, GaitType.wave]
        self.phase_steps_per_cycle = [20, 25, 40]  # per gait

        self.joystick_input_handler = JoystickInputHandler(
            button_callbacks={
                ButtonIndex.DpadLeft: lambda b, e: self.prev_gait(),
                ButtonIndex.DpadRight: lambda b, e: self.next_gait(),
                ButtonIndex.PS: lambda b, e: self.process_kill_switch(),
                ButtonIndex.TouchpadButton: lambda b, e: self.reboot_servos(),
                ButtonIndex.Select: lambda b, e: self.finalize(),
            }
        )
        self.joystick_sub = self.create_subscription(
            sensor_msgs.msg.Joy,
            '/joy',
            self.joystick_input_handler.process_joy_message,
            qos_profile=10,
        )

        self.robot_state = None

        qos_profile = QoSProfile(depth=1)
        # make state available to late joiners
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.robot_state_sub = self.create_subscription(
            std_msgs.msg.String, '/robot_state', self.process_robot_state, qos_profile=qos_profile
        )
        self.robot_event_pub = self.create_publisher(
            std_msgs.msg.String, '/robot_event', qos_profile=10
        )

        self.joint_trajectory_pub = self.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            qos_profile=10,
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

        self.setup_hexapod()

        self.loop_timer = self.create_timer(1 / self.fps, self.loop, autostart=False)

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

    def prev_gait(self):
        self.gait_index = (self.gait_index - 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def next_gait(self):
        self.gait_index = (self.gait_index + 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def loop(self):
        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.phase_step = 1 / self.phase_steps_per_cycle[self.gait_index]
        self.walker.next_step(
            stride_direction=self.joystick_input_handler.direction,
            rotation_direction=self.joystick_input_handler.rotation_speed,
        )

        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(seconds_from_start=self.walker.phase_step)
        trajectory.publish(self.joint_trajectory_pub)

    def initialization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        # - Turn torque on for femur
        # - Move all femur to -105
        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_hexapod(seconds_from_start=1.0, joint_mask=['femur'])

        # - Turn torque on for tibia
        # - Move all tibia to 0
        trajectory.add_point_from_hexapod(seconds_from_start=1.6, joint_mask=['femur', 'tibia'])

        # - Turn torque on for coxa
        # - Move all coxa to 0
        trajectory.add_point_from_hexapod(seconds_from_start=2.2)

        # - Move all tibia to 95
        self.hexapod.forward_kinematics(0, -105, 95)
        trajectory.add_point_from_hexapod(seconds_from_start=2.8)

        # Get into default stance for walk controller to take from here
        self.hexapod.forward_kinematics(0, -35, 130)
        trajectory.add_point_from_hexapod(seconds_from_start=3.2)

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done')),
        )

    def finalization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_hexapod(seconds_from_start=1.0)

        self.hexapod.forward_kinematics(0, -105, -60)
        trajectory.add_point_from_hexapod(seconds_from_start=1.5)

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done')),
        )

    def turn_torque_off(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(seconds_from_start=0.0, effort=0.0)
        trajectory.publish(self.joint_trajectory_pub)

    def reboot_servos(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(seconds_from_start=0.0, effort=-1.0)
        trajectory.add_point_from_hexapod(seconds_from_start=1.0, effort=0.0)
        trajectory.publish(self.joint_trajectory_pub)

    def process_robot_state(self, msg: std_msgs.msg.String):
        if self.robot_state == msg.data:
            return

        self.robot_state = msg.data

        if self.robot_state == 'torque_off':
            self.get_logger().info('Torque is off, stopping')
            self.stop_walk_controller()
            self.turn_torque_off()
        elif self.robot_state == 'initializing':
            self.initialization_sequence()
        elif self.robot_state == 'torque_on':
            self.get_logger().info('Torque is on, starting')
            self.loop_timer.reset()
        elif self.robot_state == 'finalizing':
            self.stop_walk_controller()
            self.finalization_sequence()
        elif self.robot_state == 'finalized':
            self.get_logger().info('Finalized')
            self.turn_torque_off()

    def stop_walk_controller(self):
        self.get_logger().info('Stopping')
        self.loop_timer.cancel()
        self.walker.reset()

    def process_kill_switch(self):
        self.robot_event_pub.publish(std_msgs.msg.String(data='kill_switch_pressed'))

    def finalize(self):
        self.robot_event_pub.publish(std_msgs.msg.String(data='finalize'))


def main():
    node = None
    try:
        parser = argparse.ArgumentParser('Dr.QP Robot controller ROS node')
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])
        rclpy.init()
        node = HexapodBrain(**vars(args))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        if node is not None:
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
