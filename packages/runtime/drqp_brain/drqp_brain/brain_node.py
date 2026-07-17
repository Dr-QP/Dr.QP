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
from collections import deque
from concurrent.futures import CancelledError
from dataclasses import dataclass
import os
import sys
import threading
from time import perf_counter
import traceback

from control_msgs.action import FollowJointTrajectory
from drqp_brain.balance_controller import (
    apply_imu_balance,
    body_tilt_from_imu,
    imu_balance_stride_scale,
)
from drqp_brain.instance_guard import domain_instance_guard, InstanceAlreadyRunningError
from drqp_brain.joint_trajectory_builder import JointTrajectoryBuilder
from drqp_brain.locomotion_kinematics import (
    AnalyticLocomotionKinematics,
    CLAMPING_EVENT_TICKS,
    DEFAULT_CONTROL_RATE_HZ,
    LocomotionKinematics,
    MoveItPyLocomotionKinematics,
    MoveItPyStateValidator,
    RCLPY_SHUTDOWN_ERRORS,
    WALKING_TRAJECTORY_POINTS,
)
from drqp_brain.walk_controller import GaitType, WalkController
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from drqp_kinematics.geometry import AffineTransform, Point3D
from drqp_kinematics.models import HexapodModel
from drqp_kinematics.urdf_limits import (
    model_degrees_to_urdf_angles,
    urdf_to_model_angles,
)
import numpy as np
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import TimerCancelledError
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.utilities
from sensor_msgs.msg import Imu, JointState
import std_msgs.msg
import trajectory_msgs.msg

CONTROL_RATE_MIN_HZ = 5.0
CONTROL_RATE_MAX_HZ = 100.0
TICK_DURATION_WINDOW_SEC = 5.0
BODY_POSE_TRANSLATION_SCALE = 8.0


@dataclass(frozen=True)
class TickDurationStatistics:
    """Summary of recent walking-control callback durations."""

    sample_count: int
    minimum_sec: float
    mean_sec: float
    maximum_sec: float


class TickDurationWindow:
    """Maintain bounded tick-duration measurements without per-tick allocation growth."""

    def __init__(self, capacity: int):
        if capacity <= 0:
            raise ValueError('tick-duration window capacity must be positive')
        self._durations: deque[float] = deque(maxlen=capacity)

    def record(self, duration_sec: float) -> None:
        self._durations.append(duration_sec)

    @property
    def capacity(self) -> int:
        """Return the maximum number of retained duration samples."""
        return self._durations.maxlen

    @property
    def statistics(self) -> TickDurationStatistics:
        if not self._durations:
            return TickDurationStatistics(0, 0.0, 0.0, 0.0)
        return TickDurationStatistics(
            sample_count=len(self._durations),
            minimum_sec=min(self._durations),
            mean_sec=sum(self._durations) / len(self._durations),
            maximum_sec=max(self._durations),
        )


def _assert_no_existing_brain_node(node: rclpy.node.Node) -> None:
    for node_name, namespace in node.get_node_names_and_namespaces():
        if node_name == 'drqp_brain':
            qualified_name = f'{namespace.rstrip("/")}/{node_name}'
            raise InstanceAlreadyRunningError(
                f'Another drqp_brain ROS node is already running as {qualified_name}.'
            )


class HexapodBrain(rclpy.node.Node):
    """
    ROS node for controlling Dr.QP hexapod robot.

    Subscribes to semantic movement and robot command topics, processes them with WalkController,
    and publishes positions to /servo_goals topic.

    """

    def __init__(self, **node_kwargs):
        super().__init__('drqp_brain', **node_kwargs)
        self._shutdown_lock = threading.Lock()
        self._is_shutting_down = False
        self._pending_futures = set()

        # Analytic IK is the runtime default. MoveIt remains in-process for
        # planning-scene validation and as a selectable comparison backend.
        self.loop_callback_group = MutuallyExclusiveCallbackGroup()
        self.state_callback_group = ReentrantCallbackGroup()
        self.declare_parameter(
            'control_rate_hz',
            DEFAULT_CONTROL_RATE_HZ,
            ParameterDescriptor(
                description='Walking control loop rate in Hz',
                floating_point_range=[
                    FloatingPointRange(
                        from_value=CONTROL_RATE_MIN_HZ,
                        to_value=CONTROL_RATE_MAX_HZ,
                        step=0.0,
                    )
                ],
            ),
        )
        self.declare_parameter('enable_imu_balance', True)
        self.declare_parameter('imu_balance_gain', 2.0)
        self.declare_parameter('imu_balance_max_tilt_rad', 0.15)
        self.declare_parameter('imu_balance_timeout_sec', 1.0)
        self.declare_parameter('kinematics_backend', 'analytic')
        self.declare_parameter('omega_max_rad_sec', 0.0)
        self.declare_parameter('rotation_speed_degrees', 45.0)
        self.enable_imu_balance = self.get_parameter('enable_imu_balance').value
        self.imu_balance_gain = self.get_parameter('imu_balance_gain').value
        self.imu_balance_max_tilt_rad = self.get_parameter('imu_balance_max_tilt_rad').value
        self.imu_balance_timeout = Duration(
            seconds=self.get_parameter('imu_balance_timeout_sec').value
        )
        self.kinematics_backend = self.get_parameter('kinematics_backend').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        if not CONTROL_RATE_MIN_HZ <= self.control_rate_hz <= CONTROL_RATE_MAX_HZ:
            raise ValueError(
                'control_rate_hz must be within '
                f'[{CONTROL_RATE_MIN_HZ:g}, {CONTROL_RATE_MAX_HZ:g}], '
                f'got {self.control_rate_hz:g}'
            )
        parameter_overrides = getattr(self, '_parameter_overrides', {})
        omega_max_overridden = 'omega_max_rad_sec' in parameter_overrides
        rotation_speed_overridden = 'rotation_speed_degrees' in parameter_overrides
        configured_omega_max = self.get_parameter('omega_max_rad_sec').value
        if omega_max_overridden:
            if configured_omega_max < 0.0:
                raise ValueError('omega_max_rad_sec must be non-negative when configured')
            self.omega_max_rad_sec = (
                float(configured_omega_max) if configured_omega_max > 0.0 else None
            )
        else:
            self.omega_max_rad_sec = None
        self.rotation_speed_degrees = float(self.get_parameter('rotation_speed_degrees').value)
        if rotation_speed_overridden:
            self.get_logger().warning(
                'rotation_speed_degrees is deprecated; configure omega_max_rad_sec instead'
            )
        if self.kinematics_backend not in {'analytic', 'moveit'}:
            raise ValueError(
                'kinematics_backend must be one of: analytic, moveit; '
                f'got {self.kinematics_backend!r}'
            )

        self.gait_index = 0
        self.gaits = [GaitType.tripod, GaitType.ripple, GaitType.wave]
        # These preserve the observed speed of the old double-advance loop at
        # 8 Hz. They are deliberately time-based rather than frame-based.
        self.cycle_time_sec = {
            GaitType.tripod: 1.25,
            GaitType.ripple: 1.5625,
            GaitType.wave: 2.5,
        }

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
        self._last_published_motion_state = None
        self._last_loop_time = None
        self.tick_duration_window = TickDurationWindow(
            max(1, round(self.control_rate_hz * TICK_DURATION_WINDOW_SEC))
        )
        self._ticks_since_duration_log = 0
        self._joint_state_warning_logged = False
        self._latest_clamped_legs = ()
        self._clamping_ticks = {}
        self.walking_trajectory_points = WALKING_TRAJECTORY_POINTS
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.process_joint_state,
            qos_profile=10,
            callback_group=self.state_callback_group,
        )

        self.setup_hexapod()
        self._last_published_motion_state = self._motion_state_key(self.hexapod.body_transform)
        self.kinematics: LocomotionKinematics
        if self.kinematics_backend == 'moveit':
            self.kinematics = MoveItPyLocomotionKinematics(
                node=self,
                hexapod=self.hexapod,
                is_shutting_down=lambda: self._is_shutting_down,
                control_rate_hz=self.control_rate_hz,
            )
        else:
            state_validator = MoveItPyStateValidator(
                node=self,
                hexapod=self.hexapod,
                is_shutting_down=lambda: self._is_shutting_down,
            )
            self.kinematics = AnalyticLocomotionKinematics(
                node=self,
                hexapod=self.hexapod,
                state_validator=state_validator,
            )

        self.loop_timer = self.create_timer(
            1 / self.control_rate_hz,
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
            rotation_speed_degrees=self.rotation_speed_degrees,
            omega_max_rad_sec=self.omega_max_rad_sec,
            gait=self.gaits[self.gait_index],
            cycle_time_sec=self.cycle_time_sec[self.gaits[self.gait_index]],
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

        self.current_body_tilt = body_tilt_from_imu(msg.orientation)
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
        """Run one control tick and retain a bounded runtime-duration measurement."""
        tick_start = perf_counter()
        try:
            self._run_loop()
        finally:
            self._record_tick_duration(perf_counter() - tick_start)

    def _run_loop(self):
        """Apply the latest semantic command and publish viable joint targets."""
        if self._is_shutting_down:
            return

        self.walker.current_gait = self.gaits[self.gait_index]
        self.walker.cycle_time_sec = self.cycle_time_sec[self.walker.current_gait]
        dt = self._next_loop_dt()

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
        imu_body_tilt = self.get_imu_body_tilt()
        stride_scale = imu_balance_stride_scale(
            imu_body_tilt,
            target_body_tilt=self.target_body_tilt,
            gain=self.imu_balance_gain,
            max_tilt_rad=self.imu_balance_max_tilt_rad,
        )
        stride_direction = stride_direction * stride_scale
        body_rotation = apply_imu_balance(
            body_rotation,
            imu_body_tilt,
            target_body_tilt=self.target_body_tilt,
            gain=self.imu_balance_gain,
            max_tilt_rad=self.imu_balance_max_tilt_rad,
        )

        body_transform = self.walker.body_transform(
            body_translation / BODY_POSE_TRANSLATION_SCALE,
            body_rotation,
        )
        self.walker.advance(
            dt,
            stride_direction=stride_direction,
            rotation_direction=self.current_movement.rotation_speed,
        )
        self.hexapod.body_transform = body_transform

        if not self._ik_ready():
            return

        feet_target_window = self._build_walking_feet_target_window(
            body_translation=body_translation / BODY_POSE_TRANSLATION_SCALE,
            body_rotation=body_rotation,
        )
        motion_state_key = self._motion_state_key(body_transform)
        if motion_state_key == self._last_published_motion_state:
            return

        try:
            trajectory_targets = self._solve_walking_trajectory_targets(feet_target_window)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        if trajectory_targets is None:
            self.get_logger().warning(
                'Kinematics rejected the current foot targets; skipping trajectory publish'
            )
            return

        joint_targets = trajectory_targets[-1][1]
        self.apply_joint_targets(joint_targets)

        trajectory = JointTrajectoryBuilder(self.hexapod)
        for point_index, (_, point_joint_targets) in enumerate(trajectory_targets, start=1):
            trajectory.add_point_from_joint_targets(
                point_joint_targets,
                reach_in_seconds_from_start=point_index / self.control_rate_hz,
            )
        trajectory.publish(self.joint_trajectory_pub)
        self._last_published_motion_state = motion_state_key

    def process_joint_state(self, msg: JointState):
        self.latest_joint_state = msg
        self._joint_state_warning_logged = False

    def solve_joint_targets(self, legs_and_targets):
        result = self.kinematics.solve(legs_and_targets, self.latest_joint_state)
        self._latest_clamped_legs = result.clamped_legs
        if not result.succeeded:
            self.get_logger().warning(result.failure_reason)
            return None
        if result.clamped_legs:
            self.get_logger().warning(
                f'Analytic IK clamped legs: {", ".join(result.clamped_legs)}',
                throttle_duration_sec=5.0,
            )
        return result.joint_targets

    def _build_walking_feet_target_window(
        self,
        body_translation: Point3D,
        body_rotation: Point3D,
    ):
        phase_increment = 1.0 / (self.control_rate_hz * self.walker.cycle_time_sec)
        return [
            self.walker.targets_at(
                self.walker.current_phase + point_index * phase_increment,
                self.walker.steering,
                body_direction=body_translation,
                body_rotation=body_rotation,
            )
            for point_index in range(max(1, self.walking_trajectory_points))
        ]

    def _solve_walking_trajectory_targets(self, feet_target_window):
        trajectory_targets = []
        clamped_legs = set()
        for feet_targets in feet_target_window:
            joint_targets = self.solve_joint_targets(feet_targets)
            if joint_targets is None:
                return None

            clamped_legs.update(self._latest_clamped_legs)
            trajectory_targets.append((feet_targets, joint_targets))

        self._update_clamping_status(clamped_legs)
        return trajectory_targets

    def _update_clamping_status(self, clamped_legs: set[str]) -> None:
        persistent_legs = []
        for leg in self.hexapod.legs:
            leg_name = leg.label.name
            if leg_name in clamped_legs:
                self._clamping_ticks[leg_name] = self._clamping_ticks.get(leg_name, 0) + 1
                if self._clamping_ticks[leg_name] == CLAMPING_EVENT_TICKS:
                    persistent_legs.append(leg_name)
            else:
                self._clamping_ticks[leg_name] = 0

        if persistent_legs:
            self.robot_event_pub.publish(
                std_msgs.msg.String(
                    data='locomotion_clamping_persistent:' + ','.join(persistent_legs)
                )
            )

    def _make_pose_stamped(self, leg, foot_target):
        return self.kinematics.make_pose_stamped(leg, foot_target)

    def _extract_leg_joint_targets(self, leg, robot_state):
        return self.kinematics.extract_leg_joint_targets(leg, robot_state)

    def _motion_state_key(
        self, body_transform: AffineTransform
    ) -> tuple[str, float, float, float, float, float, tuple[float, ...], int]:
        """Return the committed gait inputs that make a publish necessary."""
        direction = self.walker.steering.direction
        return (
            self.walker.current_gait.name,
            round(self.walker.current_phase, 9),
            round(float(direction.x), 9),
            round(float(direction.y), 9),
            round(float(direction.z), 9),
            round(self.walker.steering.rotation_direction, 9),
            tuple(round(float(value), 9) for row in body_transform.matrix for value in row),
            self.walking_trajectory_points,
        )

    def _next_loop_dt(self) -> float:
        """Measure and bound the elapsed time between control-loop ticks."""
        now = self.get_clock().now()
        if self._last_loop_time is None:
            dt = 1.0 / self.control_rate_hz
        else:
            dt = (now - self._last_loop_time).nanoseconds / 1_000_000_000.0
            dt = float(np.clip(dt, 0.0, 2.0 / self.control_rate_hz))
        self._last_loop_time = now
        return dt

    def _record_tick_duration(self, duration_sec: float) -> None:
        """Log min/mean/max control-tick duration once per five-second window."""
        self.tick_duration_window.record(duration_sec)
        self._ticks_since_duration_log += 1
        if self._ticks_since_duration_log < self.tick_duration_window.capacity:
            return

        statistics = self.tick_duration_window.statistics
        self.get_logger().debug(
            'Control tick duration over '
            f'{statistics.sample_count} samples: '
            f'min={statistics.minimum_sec * 1_000.0:.3f} ms '
            f'mean={statistics.mean_sec * 1_000.0:.3f} ms '
            f'max={statistics.maximum_sec * 1_000.0:.3f} ms'
        )
        self._ticks_since_duration_log = 0

    def _ik_ready(self) -> bool:
        if self._is_shutting_down:
            return False

        if self.latest_joint_state is None:
            if not self._joint_state_warning_logged:
                self.get_logger().warning('No joint state available from trajectory controller')
                self._joint_state_warning_logged = True
            return False

        return self.kinematics.ready()

    def _controller_joint_names(self, leg):
        return self.kinematics.controller_joint_names(leg)

    def apply_joint_targets(self, joint_targets: dict[str, float]):
        for leg in self.hexapod.legs:
            urdf_angles = tuple(
                joint_targets[joint_name] for joint_name in self._controller_joint_names(leg)
            )
            leg.forward_kinematics(*np.degrees(urdf_to_model_angles(urdf_angles)))

    def _model_joint_targets(self) -> dict[str, float]:
        """Return the current analytic model pose in URDF/controller radians."""
        joint_targets = {}
        for leg in self.hexapod.legs:
            model_angles_deg = (
                leg.coxa_angle,
                leg.femur_angle,
                leg.tibia_angle,
            )
            for joint_name, angle in zip(
                self._controller_joint_names(leg),
                model_degrees_to_urdf_angles(model_angles_deg),
            ):
                joint_targets[joint_name] = angle
        return joint_targets

    def initialization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        # - Turn torque on for femur
        # - Move all femur to -105
        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(),
            reach_in_seconds_from_start=1.0,
            joint_mask=['femur'],
        )

        # - Turn torque on for tibia
        # - Move all tibia to 0
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(),
            reach_in_seconds_from_start=1.6,
            joint_mask=['femur', 'tibia'],
        )

        # - Turn torque on for coxa
        # - Move all coxa to 0
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=2.2
        )

        # - Move all tibia to 95
        self.hexapod.forward_kinematics(0, -105, 95)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=2.8
        )

        # Get into default stance for walk controller to take from here
        self.hexapod.forward_kinematics(0, -35, 130)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=3.2
        )

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done')),
        )

    def finalization_sequence(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)

        self.hexapod.forward_kinematics(0, -105, 0)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=1.0
        )

        self.hexapod.forward_kinematics(0, -105, -60)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=1.5
        )

        trajectory.publish_action(
            self.trajectory_client,
            self,
            lambda: self.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done')),
        )

    def turn_torque_off(self):
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=0.0, effort=0.0
        )
        trajectory.publish(self.joint_trajectory_pub)

    def reboot_servos(self):
        """Execute servo reboot sequence and publish completion event."""
        trajectory = JointTrajectoryBuilder(self.hexapod)
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=0.0, effort=-1.0
        )
        trajectory.add_point_from_joint_targets(
            self._model_joint_targets(), reach_in_seconds_from_start=1.0, effort=0.0
        )
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
            self.stop_walk_controller()
            self.initialization_sequence()
        elif self.robot_state == 'torque_on':
            self.get_logger().info('Torque is on, starting')
            self.start_walk_controller()
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

    def start_walk_controller(self):
        if not self._is_shutting_down:
            self.get_logger().info('Starting')
        self.loop_timer.reset()

    def stop_walk_controller(self):
        if not self._is_shutting_down:
            self.get_logger().info('Stopping')
        try:
            self.loop_timer.cancel()
        except (InvalidHandle, RCLError, RuntimeError, TimerCancelledError):
            # Timer teardown is best-effort; the node may already be destroying it.
            pass
        self.walker.reset()
        self._last_loop_time = None
        self._last_published_motion_state = self._motion_state_key(self.hexapod.body_transform)

    def _track_future(self, future):
        self._pending_futures.add(future)
        add_done_callback = getattr(future, 'add_done_callback', None)
        if callable(add_done_callback):
            add_done_callback(self._discard_future)
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
        except Exception:  # noqa: BLE001 — shutdown teardown must swallow any future exception
            # Log the full traceback where possible; during shutdown rosout may be unavailable.
            try:
                self._log_shutdown_warning(
                    f'Pending future finished with exception: {traceback.format_exc()}'
                )
            except Exception:  # noqa: BLE001 — logging itself may fail during shutdown
                # If logging fails, emit a stderr fallback and continue teardown.
                print(
                    'Pending future finished with exception during shutdown.',
                    file=sys.stderr,
                )
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
                if future.done():
                    try:
                        future.result()
                    except CancelledError:
                        # Expected during shutdown.
                        pass
                    except Exception:  # noqa: BLE001 — swallow any future exception in shutdown
                        try:
                            self._log_shutdown_warning(
                                'Pending future finished with exception '
                                f'during cancel: {traceback.format_exc()}'
                            )
                        except Exception:  # noqa: BLE001 — logging itself may fail during shutdown
                            print(
                                'Pending future finished with exception during cancel.',
                                file=sys.stderr,
                            )
            except Exception:  # noqa: BLE001 — shutdown teardown must swallow any future exception
                # Defensive: ensure shutdown continues even if future handling fails.
                try:
                    self._log_shutdown_warning(
                        f'Error while cancelling pending future: {traceback.format_exc()}'
                    )
                except Exception:  # noqa: BLE001 — logging itself may fail during shutdown
                    print(
                        'Error while cancelling pending future during shutdown.',
                        file=sys.stderr,
                    )
            finally:
                self._pending_futures.discard(future)
        # All tracked futures cleared
        self._pending_futures.clear()

    def _safe_destroy_client(self, client, description: str):
        try:
            client.destroy()
        except RCLPY_SHUTDOWN_ERRORS as exc:
            self._log_shutdown_warning(f'Failed to destroy {description}: {exc}')
        except Exception:  # noqa: BLE001 — swallow unexpected destroy errors in shutdown
            # Log unexpected exceptions with traceback; during shutdown logging may be limited.
            try:
                self._log_shutdown_warning(
                    f'Unexpected exception destroying {description}: {traceback.format_exc()}'
                )
            except Exception:  # noqa: BLE001 — logging itself may fail during shutdown
                print(
                    f'Unexpected exception destroying {description} during shutdown.',
                    file=sys.stderr,
                )

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

        try:
            super().destroy_node()
        except RCLPY_SHUTDOWN_ERRORS as exc:
            self._log_shutdown_warning(f'Failed to destroy drqp_brain node: {exc}')


def main():
    node = None
    executor = None
    guard_node = None
    clean_process_exit = False
    try:
        parser = argparse.ArgumentParser('Dr.QP Robot controller ROS node')
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])
        with domain_instance_guard('drqp_brain'):
            rclpy.init()
            guard_node = rclpy.create_node('drqp_brain_startup_guard')
            _assert_no_existing_brain_node(guard_node)
            guard_node.destroy_node()
            guard_node = None

            node = HexapodBrain(**vars(args))
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        clean_process_exit = True
        return 0
    except InstanceAlreadyRunningError as exc:
        print(f'drqp_brain startup refused: {exc}', file=sys.stderr)
        return 1
    finally:
        if guard_node is not None:
            guard_node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()
        if clean_process_exit:
            os._exit(0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
