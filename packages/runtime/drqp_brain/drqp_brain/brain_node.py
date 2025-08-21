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
import math
from typing import Callable

from drqp_brain.joystick_button import ButtonIndex
from drqp_brain.joystick_input_handler import JoystickInputHandler
from drqp_brain.models import HexapodModel
from drqp_brain.timed_queue import TimedQueue
from drqp_brain.walk_controller import GaitType, WalkController
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import rclpy.node
import rclpy.publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.time
import rclpy.utilities
import sensor_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

kFemurOffsetAngle = -13.11
kTibiaOffsetAngle = -32.9


class JointTrajectoryBuilder:
    def __init__(self, hexapod: HexapodModel):
        self.hexapod = hexapod
        self.joint_names = self.make_joint_names()
        self.points = []

    def make_joint_names(self):
        joint_names = []
        for leg in self.hexapod.legs:
            for joint in ['coxa', 'femur', 'tibia']:
                joint_names.append(f'dr_qp/{leg.label.name}_{joint}')
        return joint_names

    def add_point_from_hexapod(self, seconds_from_start, effort=1.0, joint_mask=None):
        positions = []
        efforts = []
        for leg in self.hexapod.legs:
            for joint, angle in [
                ('coxa', leg.coxa_angle),
                ('femur', leg.femur_angle + kFemurOffsetAngle),
                ('tibia', leg.tibia_angle + kTibiaOffsetAngle),
            ]:
                if joint_mask is not None and joint not in joint_mask:
                    efforts.append(0.0)
                else:
                    efforts.append(effort)
                positions.append(float(np.radians(angle)))

        self.add_point(positions, efforts, seconds_from_start)

    def add_point(self, positions, effort, seconds_from_start):
        self.points.append(
            trajectory_msgs.msg.JointTrajectoryPoint(
                positions=positions,
                effort=effort,
                time_from_start=rclpy.time.Duration(seconds=seconds_from_start).to_msg(),
            )
        )

    def publish(self, pub: rclpy.publisher.Publisher):
        msg = trajectory_msgs.msg.JointTrajectory(joint_names=self.joint_names, points=self.points)
        pub.publish(msg)

    def publish_action(
        self,
        action_client: ActionClient,
        node: rclpy.node.Node,
        result_callback: Callable,
    ):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory_msgs.msg.JointTrajectory(
            joint_names=self.joint_names, points=self.points
        )

        goal_handle_future = action_client.send_goal_async(goal)

        def result_response_callback(future):
            node.get_logger().debug(f'Result received: {future.result().result}')
            result_callback()

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                node.get_logger().error('Goal rejected')
                return

            node.get_logger().debug('Goal accepted')

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_response_callback)

        goal_handle_future.add_done_callback(goal_response_callback)


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

        # Set up joystick input handler with button callbacks
        button_callbacks = {
            ButtonIndex.DpadLeft: lambda b, e: self.prev_gait(),
            ButtonIndex.DpadRight: lambda b, e: self.next_gait(),
            ButtonIndex.PS: lambda b, e: self.process_kill_switch(),
            ButtonIndex.Start: lambda b, e: self.reboot_servos(),
            ButtonIndex.Select: lambda b, e: self.finalize(),
        }
        self.joystick_input_handler = JoystickInputHandler(button_callbacks=button_callbacks)

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
        self.sequence_queue = TimedQueue(self)

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
            stride_ratio=self.joystick_input_handler.walk_speed,
            rotation_ratio=self.joystick_input_handler.rotation_speed,
        )
        self.publish_joint_position_trajectory(
            playtime_ms=math.floor(1000 * self.walker.phase_step)
        )

    def initialization_sequence(self):
        self.sequence_queue.clear()
        self.sequence_queue.add(1.0, self.initialization_sequence_step1)
        self.sequence_queue.add(0.6, self.initialization_sequence_step2)
        self.sequence_queue.add(0.6, self.initialization_sequence_step3)
        self.sequence_queue.add(0.6, self.initialization_sequence_step4)
        self.sequence_queue.add(0.5, self.initialization_sequence_step5)
        self.sequence_queue.add(0.0, self.initialization_sequence_done)

    # - Turn torque on for femur
    # - Move all femur to -105
    def initialization_sequence_step1(self):
        self.get_logger().info('Initialization sequence started')

        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=900, joint_mask=['femur'])

    # - Turn torque on for tibia
    # - Move all tibia to 0
    def initialization_sequence_step2(self):
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=500, joint_mask=['femur', 'tibia'])

    # - Turn torque on for coxa
    # - Move all coxa to 0
    def initialization_sequence_step3(self):
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=500)

    # - Move all tibia to 95
    def initialization_sequence_step4(self):
        self.hexapod.forward_kinematics(0, -105, 95)
        self.publish_joint_position_trajectory(playtime_ms=500)

    # - Use walk controller to move to default position slowly increasing body height
    def initialization_sequence_step5(self):
        self.hexapod.forward_kinematics(0, -35, 130)
        self.publish_joint_position_trajectory(playtime_ms=400)

    def initialization_sequence_done(self):
        self.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done'))

    def finalization_sequence(self):
        self.sequence_queue.clear()
        # self.sequence_queue.add(1.1, self.finalization_sequence_step1)
        # self.sequence_queue.add(0.6, self.finalization_sequence_step2)
        # self.sequence_queue.add(0.0, self.finalization_sequence_done)
        trajectory = JointTrajectoryBuilder(self.hexapod)

        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_hexapod(seconds_from_start=1.0)

        self.hexapod.forward_kinematics(0, -105, -60)
        trajectory.add_point_from_hexapod(seconds_from_start=1.5)

        trajectory.publish_action(self.trajectory_client, self, self.finalization_sequence_done)

    def finalization_sequence_step1(self):
        self.get_logger().info('Finalization sequence started')

        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=1000)

    def finalization_sequence_step2(self):
        self.hexapod.forward_kinematics(0, -105, -60)
        self.publish_joint_position_trajectory(playtime_ms=500)

    def finalization_sequence_done(self):
        self.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done'))

    def turn_torque_off(self):
        self.publish_joint_position_trajectory(effort_points=[0.0])

    def reboot_servos(self):
        self.publish_joint_position_trajectory(effort_points=[-1.0, 0.0], playtime_ms=1000)

    def publish_joint_position_trajectory(
        self, playtime_ms=0, joint_mask=None, effort_points=[1.0]
    ):
        points = []
        joint_names = []
        time_offset_seconds_step = playtime_ms / 1000.0 / len(effort_points)

        for i, effort_point in enumerate(effort_points, 1):
            joint_names = []
            positions = []
            effort = []
            for leg in self.hexapod.legs:
                for joint, angle in [
                    ('coxa', leg.coxa_angle),
                    ('femur', leg.femur_angle + kFemurOffsetAngle),
                    ('tibia', leg.tibia_angle + kTibiaOffsetAngle),
                ]:
                    if joint_mask is not None and joint not in joint_mask:
                        effort.append(0.0)
                    else:
                        effort.append(effort_point)

                    joint_names.append(f'dr_qp/{leg.label.name}_{joint}')
                    positions.append(float(np.radians(angle)))

            time_from_start = rclpy.time.Duration(seconds=time_offset_seconds_step * i).to_msg()
            points.append(
                trajectory_msgs.msg.JointTrajectoryPoint(
                    positions=positions,
                    effort=effort,
                    time_from_start=time_from_start,
                )
            )

        self.publish_joint_trajectory(joint_names, points)

    def publish_joint_trajectory(self, joint_names, points):
        msg = trajectory_msgs.msg.JointTrajectory(joint_names=joint_names, points=points)
        self.joint_trajectory_pub.publish(msg)

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
        self.sequence_queue.clear()
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
