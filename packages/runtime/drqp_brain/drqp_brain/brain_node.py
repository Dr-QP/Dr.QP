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
from drqp_brain.balance_controller import (
    apply_imu_balance,
    BASE_CENTER_TO_IMU_ROTATION,
    body_tilt_from_imu,
)
from drqp_brain.joint_trajectory_builder import JointTrajectoryBuilder
from drqp_brain.models import HexapodModel
from drqp_brain.walk_controller import GaitType, WalkController
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
import rclpy.utilities
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
import std_msgs.msg
import trajectory_msgs.msg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class HexapodBrain(rclpy.node.Node):
    """
    ROS node for controlling Dr.QP hexapod robot.

    Subscribes to semantic movement and robot command topics, processes them with WalkController,
    and publishes positions to /servo_goals topic.

    """

    def __init__(self):
        super().__init__('drqp_brain')

        self.fps = 30
        self.declare_parameter('enable_imu_balance', True)
        self.declare_parameter('transform_imu_to_base_frame', True)
        self.declare_parameter('imu_balance_gain', 1.0)
        self.declare_parameter('imu_balance_max_tilt_rad', 0.35)
        self.declare_parameter('imu_balance_timeout_sec', 1.0)
        self.enable_imu_balance = self.get_parameter('enable_imu_balance').value
        self.transform_imu_to_base_frame = self.get_parameter('transform_imu_to_base_frame').value
        self.imu_balance_gain = self.get_parameter('imu_balance_gain').value
        self.imu_balance_max_tilt_rad = self.get_parameter('imu_balance_max_tilt_rad').value
        self.imu_balance_timeout = Duration(
            seconds=self.get_parameter('imu_balance_timeout_sec').value
        )

        self.gait_index = 0
        self.gaits = [GaitType.tripod, GaitType.ripple, GaitType.wave]
        self.phase_steps_per_cycle = [20, 25, 40]  # per gait

        # Store current movement command state with defaults
        self.current_movement = MovementCommand()
        # Message defaults are automatically initialized to zero values
        # Set initial gait type
        self.current_movement.gait_type = MovementCommandConstants.GAIT_TRIPOD

        # Subscribe to semantic movement commands
        self.movement_command_sub = self.create_subscription(
            MovementCommand,
            '/robot/movement_command',
            self.process_movement_command,
            qos_profile=10,
        )
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.process_imu, qos_profile=10)
        self.tf_buffer = None
        self.tf_listener = None
        if self.transform_imu_to_base_frame:
            self.tf_buffer = Buffer()
            # Retain the listener on the instance so the TF subscriptions stay active.
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_state = None
        self.current_body_tilt = None
        self.target_body_tilt = None
        self.balance_mode_enabled = False
        self.last_imu_update = None

        qos_profile = QoSProfile(depth=1)
        # make state available to late joiners
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.robot_state_sub = self.create_subscription(
            std_msgs.msg.String, '/robot_state', self.process_robot_state, qos_profile=qos_profile
        )
        self.balance_mode_sub = self.create_subscription(
            std_msgs.msg.Bool,
            '/robot/balance_mode',
            self.process_balance_mode,
            qos_profile=qos_profile,
        )
        self.robot_event_pub = self.create_publisher(
            std_msgs.msg.String, '/robot_event', qos_profile=10
        )

        self.joint_trajectory_pub = self.create_publisher(
            trajectory_msgs.msg.JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            qos_profile=10,
        )
        self.__trajectory_client = None

        self.setup_hexapod()

        self.loop_timer = self.create_timer(1 / self.fps, self.loop, autostart=False)

    @property
    def trajectory_client(self):
        """Create the action client only when a trajectory action is needed."""
        if self.__trajectory_client is None:
            self.__trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/joint_trajectory_controller/follow_joint_trajectory',
            )
        return self.__trajectory_client

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
            rotation_speed_degrees=45,
            gait=self.gaits[self.gait_index],
            phase_steps_per_cycle=self.fps / 2.5,
        )

    def prev_gait(self):
        self.gait_index = (self.gait_index - 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def next_gait(self):
        self.gait_index = (self.gait_index + 1) % len(self.gaits)
        self.get_logger().info(f'Switching gait: {self.gaits[self.gait_index].name}')

    def process_movement_command(self, msg: MovementCommand):
        """
        Process semantic movement command.

        Parameters
        ----------
        msg : MovementCommand
            Semantic movement command with stride, rotation, body position/orientation

        """
        self.current_movement = msg

        # Update gait index based on gait type from message
        gait_names = [g.name for g in self.gaits]
        if msg.gait_type in gait_names:
            self.gait_index = gait_names.index(msg.gait_type)

    def _lookup_base_center_to_imu_rotation(self, imu_frame: str):
        """Return the TF-derived IMU mount rotation when TF-based compensation is enabled."""
        if self.tf_buffer is None:
            return None

        transform = self.tf_buffer.lookup_transform(
            'drqp/base_center_link',
            imu_frame,
            # Zero time requests the latest transform available in the TF buffer.
            Time(seconds=0),
        )

        rotation = transform.transform.rotation
        return R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])

    def process_imu(self, msg: Imu):
        """Store the latest body tilt estimate from the IMU orientation."""
        quaternion = np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ]
        )
        # Per sensor_msgs/Imu, a negative covariance means orientation is unavailable.
        if msg.orientation_covariance[0] < 0 or np.allclose(quaternion, 0.0):
            self.current_body_tilt = None
            self.last_imu_update = None
            return

        imu_frame = msg.header.frame_id or 'drqp/imu_link'
        try:
            base_center_to_imu_rotation = self._lookup_base_center_to_imu_rotation(imu_frame)
        except TransformException as exc:
            self.get_logger().warning(
                f'Failed to lookup transform from drqp/base_center_link to {imu_frame}: {exc}'
            )
            self.current_body_tilt = None
            self.last_imu_update = None
            return

        self.current_body_tilt = body_tilt_from_imu(
            msg.orientation,
            base_center_to_imu_rotation=base_center_to_imu_rotation,
        )
        self.last_imu_update = self.get_clock().now()
        if self.balance_mode_enabled and self.target_body_tilt is None:
            self.target_body_tilt = self.current_body_tilt

    def next_control_mode(self):
        """Log control mode changes (control mode is now handled by translator)."""
        self.get_logger().info('Control mode changed in translator node')

    def process_balance_mode(self, msg: std_msgs.msg.Bool):
        """Enable or disable IMU balancing around the captured body tilt."""
        self.balance_mode_enabled = self.enable_imu_balance and msg.data
        if not self.balance_mode_enabled:
            self.target_body_tilt = None
            return

        self.target_body_tilt = self.get_imu_body_tilt()

    def get_imu_body_tilt(self):
        """Return the latest IMU-derived tilt when balancing data is fresh enough."""
        if (
            not self.enable_imu_balance
            or not self.balance_mode_enabled
            or self.current_body_tilt is None
            or self.last_imu_update is None
            or self.get_clock().now() - self.last_imu_update > self.imu_balance_timeout
        ):
            return None
        return self.current_body_tilt

    def loop(self):
        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.phase_step = 1 / self.phase_steps_per_cycle[self.gait_index]

        # Convert Vector3 messages to Point3D (or numpy arrays) for walker
        from drqp_brain.geometry import Point3D

        stride_direction = Point3D(
            [
                self.current_movement.stride_direction.x,
                self.current_movement.stride_direction.y,
                self.current_movement.stride_direction.z,
            ]
        )

        body_translation = Point3D(
            [
                self.current_movement.body_translation.x,
                self.current_movement.body_translation.y,
                self.current_movement.body_translation.z,
            ]
        )

        body_rotation = Point3D(
            [
                self.current_movement.body_rotation.x,
                self.current_movement.body_rotation.y,
                self.current_movement.body_rotation.z,
            ]
        )
        body_rotation = apply_imu_balance(
            body_rotation,
            self.get_imu_body_tilt(),
            target_body_tilt=self.target_body_tilt,
            gain=self.imu_balance_gain,
            max_tilt_rad=self.imu_balance_max_tilt_rad,
        )

        self.walker.next_step(
            stride_direction=stride_direction,
            rotation_direction=self.current_movement.rotation_speed,
            body_direction=body_translation / 8.0,
            body_rotation=body_rotation,
        )

        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=self.walker.phase_step)
        trajectory.publish(self.joint_trajectory_pub)

    def initialization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        # - Turn torque on for femur
        # - Move all femur to -105
        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=1.0, joint_mask=['femur'])

        # - Turn torque on for tibia
        # - Move all tibia to 0
        trajectory.add_point_from_hexapod(
            reach_in_seconds_from_start=1.6, joint_mask=['femur', 'tibia']
        )

        # - Turn torque on for coxa
        # - Move all coxa to 0
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=2.2)

        # - Move all tibia to 95
        self.hexapod.forward_kinematics(0, -105, 95)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=2.8)

        # Get into default stance for walk controller to take from here
        self.hexapod.forward_kinematics(0, -35, 130)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=3.2)

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done')),
        )

    def finalization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=1.0)

        self.hexapod.forward_kinematics(0, -105, -60)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=1.5)

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done')),
        )

    def turn_torque_off(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=0.0, effort=0.0)
        trajectory.publish(self.joint_trajectory_pub)

    def reboot_servos(self):
        """Execute servo reboot sequence and publish completion event."""
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=0.0, effort=-1.0)
        trajectory.add_point_from_hexapod(reach_in_seconds_from_start=1.0, effort=0.0)
        trajectory.publish_action(self.trajectory_client, self, self.publish_servos_rebooting_done)

    def publish_servos_rebooting_done(self):
        self.robot_event_pub.publish(std_msgs.msg.String(data='servos_rebooting_done'))

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
        elif self.robot_state == 'servos_rebooting':
            self.get_logger().info('Rebooting servos')
            self.stop_walk_controller()
            self.reboot_servos()

    def stop_walk_controller(self):
        self.get_logger().info('Stopping')
        self.loop_timer.cancel()
        self.walker.reset()

    def destroy_node(self):
        """Clean up node resources before shutting down."""
        if self.__trajectory_client is not None:
            self.__trajectory_client.destroy()
            self.__trajectory_client = None
        return super().destroy_node()


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
