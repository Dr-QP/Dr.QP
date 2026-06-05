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
from concurrent.futures import CancelledError
import threading
import time
import traceback

from control_msgs.action import FollowJointTrajectory
from drqp_brain.joint_trajectory_builder import (
    JointTrajectoryBuilder,
    kFemurOffsetAngle,
    kTibiaOffsetAngle,
)
from drqp_brain.walk_controller import GaitType, WalkController
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodModel
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import NotInitializedException, TimerCancelledError
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.utilities
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
import std_msgs.msg
import trajectory_msgs.msg

MOVEIT_IK_SERVICE = '/compute_ik'
MOVEIT_IK_TIMEOUT_SEC = 2.0
BASE_FRAME = 'drqp/base_center_link'

RCLPY_SHUTDOWN_ERRORS = (InvalidHandle, NotInitializedException, RCLError, RuntimeError)


class HexapodBrain(rclpy.node.Node):
    """
    ROS node for controlling Dr.QP hexapod robot.

    Subscribes to semantic movement and robot command topics, processes them with WalkController,
    and publishes positions to /servo_goals topic.

    """

    def __init__(self):
        super().__init__('drqp_brain')
        self._shutdown_lock = threading.Lock()
        self._is_shutting_down = False
        self._pending_futures = set()

        # MoveIt IK is invoked once per leg, so the walking loop must stay well
        # below the service throughput seen in simulation.
        self.fps = 8
        self.loop_callback_group = MutuallyExclusiveCallbackGroup()
        self.ik_callback_group = ReentrantCallbackGroup()
        self.state_callback_group = ReentrantCallbackGroup()

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
        self._last_published_foot_targets = None
        self._joint_state_warning_logged = False
        self._ik_service_warning_logged = False
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.process_joint_state,
            qos_profile=10,
            callback_group=self.state_callback_group,
        )

        self.setup_hexapod()

        self.loop_timer = self.create_timer(
            1 / self.fps,
            self.loop,
            callback_group=self.loop_callback_group,
            autostart=False,
        )

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
            self.__ik_client = self.create_client(
                GetPositionIK,
                MOVEIT_IK_SERVICE,
                callback_group=self.ik_callback_group,
            )
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
        step_length = 0.10  # in meters
        step_height = 0.01  # in meters

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
        if self._is_shutting_down:
            return

        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.phase_step = 1 / self.phase_steps_per_cycle[self.gait_index]
        previous_state = self._snapshot_motion_state()

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

        if not self._ik_ready():
            self._restore_motion_state(previous_state)
            return

        foot_targets_key = self._foot_targets_key(feet_targets)
        if foot_targets_key == self._last_published_foot_targets:
            self._restore_motion_state(previous_state)
            return

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
            reach_in_seconds_from_start=1 / self.fps,
        )
        trajectory.publish(self.joint_trajectory_pub)
        self._last_published_foot_targets = foot_targets_key

    def process_joint_state(self, msg: JointState):
        self.latest_joint_state = msg
        self._joint_state_warning_logged = False

    def solve_joint_targets(self, legs_and_targets):
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
        if self._is_shutting_down:
            raise RuntimeError('MoveIt IK request aborted because drqp_brain is shutting down')

        request = self._build_ik_request(leg, foot_target, robot_state)
        future = self.ik_client.call_async(request)
        self._track_future(future)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=MOVEIT_IK_TIMEOUT_SEC):
            self._safe_cancel_future(future, f'MoveIt IK request for {leg.label.name}')
            if self._is_shutting_down:
                raise RuntimeError(
                    f'MoveIt IK request aborted for {leg.label.name} during shutdown'
                )
            raise RuntimeError(
                f'MoveIt IK request timed out for {leg.label.name} after '
                f'{MOVEIT_IK_TIMEOUT_SEC:.1f}s'
            )

        if not future.done():
            self._safe_cancel_future(future, f'MoveIt IK request for {leg.label.name}')
            raise RuntimeError(
                f'MoveIt IK request timed out for {leg.label.name} after '
                f'{MOVEIT_IK_TIMEOUT_SEC:.1f}s'
            )

        future_exception = future.exception()
        if future_exception is not None:
            raise RuntimeError(
                f'MoveIt IK request for {leg.label.name} raised an exception: {future_exception}'
            ) from future_exception
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
        request.ik_request.avoid_collisions = False
        request.ik_request.ik_link_name = f'drqp/{leg.label.name}_foot_link'
        request.ik_request.pose_stamped = self._make_pose_stamped(leg, foot_target)
        request.ik_request.timeout = Duration(seconds=MOVEIT_IK_TIMEOUT_SEC).to_msg()
        return request

    def _make_pose_stamped(self, leg, foot_target):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        base_frame_target = self.hexapod.body_transform.inverse.apply_point(foot_target)
        pose.pose.position.x = float(base_frame_target.x)
        pose.pose.position.y = float(base_frame_target.y)
        pose.pose.position.z = float(base_frame_target.z)

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

    def _foot_targets_key(self, legs_and_targets):
        return tuple(
            (
                leg.label.name,
                round(float(foot_target.x), 6),
                round(float(foot_target.y), 6),
                round(float(foot_target.z), 6),
            )
            for leg, foot_target in legs_and_targets
        )

    def _ik_ready(self) -> bool:
        if self._is_shutting_down:
            return False

        if self.latest_joint_state is None:
            if not self._joint_state_warning_logged:
                self.get_logger().warning('No joint state available to seed MoveIt IK requests')
                self._joint_state_warning_logged = True
            return False

        try:
            service_available = self.ik_client.wait_for_service(timeout_sec=0.0)
        except RCLPY_SHUTDOWN_ERRORS as exc:
            if not self._is_shutting_down:
                self.get_logger().warning(
                    f'MoveIt IK service {MOVEIT_IK_SERVICE} readiness check failed: {exc}'
                )
            return False

        if not service_available:
            if not self._ik_service_warning_logged:
                self.get_logger().warning(f'MoveIt IK service {MOVEIT_IK_SERVICE} is unavailable')
                self._ik_service_warning_logged = True
            return False

        self._ik_service_warning_logged = False
        return True

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
        if self._is_shutting_down:
            return

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
        if not self._is_shutting_down:
            self.get_logger().info('Stopping')
        try:
            self.loop_timer.cancel()
        except (InvalidHandle, RCLError, RuntimeError, TimerCancelledError):
            # Timer teardown is best-effort; the node may already be destroying it.
            pass
        self.walker.reset()
        self._last_published_foot_targets = None

    def _track_future(self, future):
        self._pending_futures.add(future)
        try:
            future.add_done_callback(self._discard_future)
        except AttributeError:
            pass
        return future

    def _discard_future(self, future):
        # Remove from tracking set first
        self._pending_futures.discard(future)
        # Ensure exception is retrieved so asyncio does not print "exception was never retrieved"
        try:
            # If the future completed normally, this will return the result.
            # If it raised, result() will re-raise and the except block will capture traceback.
            future.result()
        except CancelledError:
            # Pending ROS futures are routinely cancelled during shutdown.
            return
        except Exception:
            # Log the full traceback where possible; during shutdown rosout may be unavailable.
            try:
                self._log_shutdown_warning(
                    f'Pending future finished with exception: {traceback.format_exc()}'
                )
            except Exception:
                # If logging fails, swallow to avoid raising during teardown.
                pass
        # No return value needed

    def _safe_cancel_future(self, future, description: str):
        if future.done():
            return
        future.cancel()

    def _cancel_pending_futures(self):
        # Cancel pending futures and attempt to retrieve their exceptions to avoid
        # "exception was never retrieved" warnings from asyncio during teardown.
        for future in list(self._pending_futures):
            try:
                if not future.done():
                    future.cancel()
                # Wait briefly for the future to finish, but don't block long.
                for _ in range(20):
                    if future.done():
                        break
                    time.sleep(0.01)
                if future.done():
                    try:
                        future.result()
                    except CancelledError:
                        # Expected during shutdown.
                        pass
                    except Exception:
                        try:
                            self._log_shutdown_warning(
                                'Pending future finished with exception '
                                f'during cancel: {traceback.format_exc()}'
                            )
                        except Exception:
                            pass
            except Exception:
                # Defensive: ensure shutdown continues even if future handling fails.
                try:
                    self._log_shutdown_warning(
                        f'Error while cancelling pending future: {traceback.format_exc()}'
                    )
                except Exception:
                    pass
            finally:
                self._pending_futures.discard(future)
        # All tracked futures cleared
        self._pending_futures.clear()

    def _safe_destroy_client(self, client, description: str):
        try:
            client.destroy()
        except RCLPY_SHUTDOWN_ERRORS as exc:
            self._log_shutdown_warning(f'Failed to destroy {description}: {exc}')
        except Exception:
            # Log unexpected exceptions with traceback; during shutdown logging may be limited.
            try:
                self._log_shutdown_warning(
                    f'Unexpected exception destroying {description}: {traceback.format_exc()}'
                )
            except Exception:
                pass

    def _log_shutdown_warning(self, message: str):
        try:
            self.get_logger().warning(message)
        except RCLPY_SHUTDOWN_ERRORS:
            # rosout may already be unavailable during node teardown.
            pass

    def destroy_node(self) -> None:
        with self._shutdown_lock:
            if self._is_shutting_down:
                return
            self._is_shutting_down = True

        self.stop_walk_controller()

        self._cancel_pending_futures()

        if self.__trajectory_client is not None:
            self._safe_destroy_client(self.__trajectory_client, 'trajectory action client')
            self.__trajectory_client = None
        if self.__ik_client is not None:
            self._safe_destroy_client(self.__ik_client, 'MoveIt IK client')
            self.__ik_client = None

        try:
            super().destroy_node()
        except RCLPY_SHUTDOWN_ERRORS as exc:
            self._log_shutdown_warning(f'Failed to destroy drqp_brain node: {exc}')


def main():
    node = None
    executor = None
    try:
        parser = argparse.ArgumentParser('Dr.QP Robot controller ROS node')
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])
        rclpy.init()
        node = HexapodBrain(**vars(args))
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
