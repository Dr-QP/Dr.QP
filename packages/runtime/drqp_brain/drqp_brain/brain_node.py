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
from drqp_brain.geometry import AffineTransform
from drqp_brain.joint_trajectory_builder import (
    JointTrajectoryBuilder,
    kFemurOffsetAngle,
    kTibiaOffsetAngle,
)
from drqp_brain.models import HexapodModel
from drqp_brain.walk_controller import GaitType, WalkController
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.utilities
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
import std_msgs.msg
import trajectory_msgs.msg


MOVEIT_IK_SERVICE = '/compute_ik'
MOVEIT_IK_TIMEOUT_SEC = 1.0
BASE_FRAME = 'drqp/base_center_link'


class HexapodBrain(rclpy.node.Node):
    """
    ROS node for controlling Dr.QP hexapod robot.

    Subscribes to semantic movement and robot command topics, processes them with WalkController,
    and publishes positions to /servo_goals topic.

    """

    def __init__(self):
        super().__init__('drqp_brain')

        self.fps = 30

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
        self.__trajectory_client = None
        self.__ik_client = None
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.process_joint_state,
            qos_profile=10,
        )

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

    @property
    def ik_client(self):
        """Create the MoveIt IK client only when a leg solve is needed."""
        if self.__ik_client is None:
            self.__ik_client = self.create_client(GetPositionIK, MOVEIT_IK_SERVICE)
        return self.__ik_client

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

    def next_control_mode(self):
        """Log control mode changes (control mode is now handled by translator)."""
        self.get_logger().info('Control mode changed in translator node')

    def loop(self):
        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.phase_step = 1 / self.phase_steps_per_cycle[self.gait_index]
        previous_state = self._snapshot_motion_state()

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

        feet_targets = self.walker.next_step_targets(
            stride_direction=stride_direction,
            rotation_direction=self.current_movement.rotation_speed,
            body_direction=body_translation / 8.0,
            body_rotation=body_rotation,
        )

        try:
            joint_targets = self.solve_joint_targets(feet_targets)
        except RuntimeError as exc:
            self._restore_motion_state(previous_state)
            self.get_logger().error(str(exc))
            return

        if joint_targets is None:
            self._restore_motion_state(previous_state)
            self.get_logger().warning(
                'MoveIt IK rejected the current foot targets; skipping trajectory publish'
            )
            return

        self.apply_joint_targets(joint_targets)

        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_joint_targets(
            joint_targets,
            reach_in_seconds_from_start=self.walker.phase_step,
        )
        trajectory.publish(self.joint_trajectory_pub)

    def process_joint_state(self, msg: JointState):
        self.latest_joint_state = msg

    def solve_joint_targets(self, legs_and_targets):
        if self.latest_joint_state is None:
            raise RuntimeError('No joint state available to seed MoveIt IK requests')

        if not self.ik_client.wait_for_service(timeout_sec=0.0):
            raise RuntimeError(f'MoveIt IK service {MOVEIT_IK_SERVICE} is unavailable')

        robot_state = self._current_robot_state()
        joint_targets = {}
        for leg, foot_target in legs_and_targets:
            response = self._call_ik(leg, foot_target, robot_state)
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().warning(
                    f'MoveIt IK failed for {leg.label.name} with code {response.error_code.val}'
                )
                return None

            leg_joint_targets = self._extract_leg_joint_targets(leg, response.solution)
            joint_targets.update(leg_joint_targets)
            robot_state = response.solution

        return joint_targets

    def _call_ik(self, leg, foot_target, robot_state: RobotState):
        request = self._build_ik_request(leg, foot_target, robot_state)
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=MOVEIT_IK_TIMEOUT_SEC)

        if not future.done():
            raise RuntimeError(
                f'MoveIt IK request timed out for {leg.label.name} after '
                f'{MOVEIT_IK_TIMEOUT_SEC:.1f}s'
            )
        if future.exception() is not None:
            raise RuntimeError(
                f'MoveIt IK request failed for {leg.label.name}: '
                f'{future.exception()}'
            )

        response = future.result()
        if response is None:
            raise RuntimeError(f'MoveIt IK returned no response for {leg.label.name}')
        return response

    def _current_robot_state(self) -> RobotState:
        robot_state = RobotState()
        latest = self.latest_joint_state
        assert latest is not None
        robot_state.joint_state = JointState(
            header=latest.header,
            name=list(latest.name),
            position=list(latest.position),
            velocity=list(latest.velocity),
            effort=list(latest.effort),
        )
        return robot_state

    def _build_ik_request(self, leg, foot_target, robot_state: RobotState):
        request = GetPositionIK.Request()
        request.ik_request.group_name = f'{leg.label.name}_leg'
        request.ik_request.robot_state = robot_state
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = f'drqp/{leg.label.name}_foot_link'
        request.ik_request.pose_stamped = self._make_pose_stamped(leg, foot_target)
        request.ik_request.timeout = Duration(seconds=MOVEIT_IK_TIMEOUT_SEC).to_msg()
        return request

    def _make_pose_stamped(self, leg, foot_target):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(foot_target.x)
        pose.pose.position.y = float(foot_target.y)
        pose.pose.position.z = float(foot_target.z)

        orientation = Rotation.from_matrix(leg.tibia_link.rotation).as_quat()
        pose.pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )
        return pose

    def _extract_leg_joint_targets(self, leg, robot_state: RobotState):
        joint_map = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        joint_targets = {}
        for joint_name in self._controller_joint_names(leg):
            if joint_name not in joint_map:
                raise RuntimeError(f'MoveIt IK response missing joint target for {joint_name}')
            joint_targets[joint_name] = joint_map[joint_name]
        return joint_targets

    def _controller_joint_names(self, leg):
        return [f'drqp/{leg.label.name}_{joint_name}' for joint_name in ('coxa', 'femur', 'tibia')]

    def apply_joint_targets(self, joint_targets: dict[str, float]):
        for leg in self.hexapod.legs:
            coxa, femur, tibia = (
                joint_targets[joint_name] for joint_name in self._controller_joint_names(leg)
            )
            leg.forward_kinematics(
                float(np.degrees(coxa)),
                float(np.degrees(femur)) - kFemurOffsetAngle,
                float(np.degrees(tibia)) - kTibiaOffsetAngle,
            )

    def _snapshot_motion_state(self):
        return {
            'current_direction': self.walker.current_direction.copy(),
            'current_rotation_direction': self.walker.current_rotation_direction,
            'current_phase': self.walker.current_phase,
            'body_transform': AffineTransform(self.hexapod.body_transform.matrix.copy()),
            'leg_angles': {
                leg.label: (leg.coxa_angle, leg.femur_angle, leg.tibia_angle)
                for leg in self.hexapod.legs
            },
        }

    def _restore_motion_state(self, previous_state):
        self.walker.current_direction = previous_state['current_direction']
        self.walker.current_rotation_direction = previous_state['current_rotation_direction']
        self.walker.current_phase = previous_state['current_phase']
        self.hexapod.body_transform = previous_state['body_transform']
        for leg in self.hexapod.legs:
            leg.forward_kinematics(*previous_state['leg_angles'][leg.label])

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
        if self.__trajectory_client is not None:
            self.__trajectory_client.destroy()
            self.__trajectory_client = None
        if self.__ik_client is not None:
            self.__ik_client.destroy()
            self.__ik_client = None
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
