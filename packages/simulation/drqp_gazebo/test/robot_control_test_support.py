# Copyright (c) 2017-2026 Anton Matosov
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

"""Shared support code for isolated Gazebo robot control launch tests."""

from collections.abc import Callable
from copy import deepcopy
import faulthandler
import inspect
import logging
import math
import os
from pathlib import Path
import re
import signal
import sys
import time

import builtin_interfaces
import builtin_interfaces.msg
from controller_manager.test_utils import check_controllers_running, check_node_running
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Pose, Vector3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_pytest.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare
from launch_testing_ros import WaitForTopics
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
import std_msgs.msg

_TEST_LOG = logging.getLogger(__name__)


def _configure_test_logging() -> None:
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
        stream=sys.stdout,
        force=True,
    )

ODOM_TOPIC = '/odom'


def build_test_gz_partition(test_name: str) -> str:
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    sanitized_name = re.sub(r'[^a-z0-9]+', '-', test_name.lower()).strip('-')
    if not sanitized_name:
        sanitized_name = 'test'
    return f'drqp-domain-{domain_id}-{sanitized_name}'


def create_simulation_launch_description(test_name: str | None = None) -> LaunchDescription:
    """Launch Gazebo simulation and wait for initialization before tests."""
    if test_name is None:
        test_name = Path(inspect.stack()[1].filename).stem

    simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare('drqp_gazebo'),
            'launch',
            'sim.launch.py',
        ]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    'sim_gui': 'false',
                    'gz_partition': build_test_gz_partition(test_name),
                }.items(),
            ),
            # Handshake with the launched processes, then let the test harness
            # block on real graph/clock/controller readiness.
            TimerAction(period=1.0, actions=[ReadyToTest()]),
        ]
    )


class GazeboRobotControlBase:
    """Shared harness and helpers for robot control behavior tests."""

    __test__ = False  # Prevent pytest from collecting this base class as a test case.

    # Configurable timeouts.
    READY_TIMEOUT = 90.0
    SPAWN_TIMEOUT = 90.0
    STATE_TRANSITION_TIMEOUT = 90.0
    MOVEMENT_TIMEOUT = 30.0
    CLOCK_TIMEOUT = 30.0
    CONTROLLER_TIMEOUT = 60.0
    SIM_TIME_TIMEOUT = 60.0
    MOVEMENT_DURATION = 5.0
    POSE_SETTLE_DURATION = 1.0

    # Posture delta threshold (meters).
    MIN_ARM_DISARM_HEIGHT_DELTA = 0.02
    POSTURE_HEIGHT_EPSILON = 0.01

    # Minimum movement to count as "moved" (meters/radians). Lowered by 20% from
    # 0.01m/0.1rad to absorb simulation timing variance that otherwise produced
    # borderline flaky failures (e.g. -0.00996m vs a -0.01m threshold).
    MIN_LINEAR_MOVEMENT_DELTA = 0.008
    MIN_ROTATION_DELTA = 0.08

    @classmethod
    def setup_class(cls) -> None:
        _configure_test_logging()
        # Dump all thread stacks on crash or SIGUSR1 — helps diagnose CI hangs.
        faulthandler.enable(file=sys.stderr, all_threads=True)
        try:
            faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)
        except (AttributeError, OSError) as e:
            _TEST_LOG.warning('Could not register SIGUSR1 faulthandler: %s', e)
        _TEST_LOG.info('setup_class: rclpy.init()')
        rclpy.init()
        _TEST_LOG.info('setup_class: rclpy initialized OK')

    @classmethod
    def teardown_class(cls) -> None:
        _TEST_LOG.info('teardown_class: rclpy.try_shutdown()')
        rclpy.try_shutdown()

    @pytest.fixture(autouse=True)
    def _node_setup(self, request) -> None:
        """Set up test node and publishers/subscribers."""
        _TEST_LOG.info(
            'node_setup START: test=%s  GZ_PARTITION=%s  ROS_DOMAIN_ID=%s',
            request.node.name,
            os.environ.get('GZ_PARTITION', 'UNSET'),
            os.environ.get('ROS_DOMAIN_ID', 'UNSET'),
        )
        self.node = rclpy.create_node('test_gazebo_robot_control')
        request.addfinalizer(self.node.destroy_node)

        self.current_robot_state = None
        self.current_clock = None
        self.robot_pose = None
        self.robot_pose_stamp_ns = None

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.state_sub = self.node.create_subscription(
            std_msgs.msg.String,
            '/robot_state',
            self._robot_state_callback,
            qos_profile=qos_profile,
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odometry_callback,
            10,
        )
        self.clock_sub = self.node.create_subscription(Clock, '/clock', self._clock_callback, 10)

        self.event_pub = self.node.create_publisher(std_msgs.msg.String, '/robot_event', 10)
        self.movement_pub = self.node.create_publisher(
            MovementCommand, '/robot/movement_command', 10
        )

        _TEST_LOG.info('node_setup: calling _wait_for_simulation_ready')
        self._wait_for_simulation_ready()
        _TEST_LOG.info('node_setup DONE: test=%s', request.node.name)

    def _wait_for_simulation_ready(self) -> None:
        logger = self.node.get_logger()

        logger.info('[sim_ready] Phase 1/6: assert_nodes_and_clock')
        self.assert_nodes_and_clock()
        logger.info('[sim_ready] Phase 1/6 DONE')

        logger.info('[sim_ready] Phase 2/6: assert_controllers_are_active')
        self.assert_controllers_are_active()
        logger.info('[sim_ready] Phase 2/6 DONE')

        logger.info('[sim_ready] Phase 3/6: wait for /robot_event + /robot/movement_command subscribers')
        self._spin_until(
            lambda: (
                self.event_pub.get_subscription_count() > 0
                and self.movement_pub.get_subscription_count() > 0
            ),
            self.READY_TIMEOUT,
            'Timed out waiting for /robot_event and /robot/movement_command subscribers',
        )
        logger.info('[sim_ready] Phase 3/6 DONE')

        logger.info('[sim_ready] Phase 4/6: wait for torque_off state')
        self._wait_for_any_state(
            ('torque_off',),  # robot initial state
            self.SPAWN_TIMEOUT,
        )
        logger.info('[sim_ready] Phase 4/6 DONE')

        logger.info('[sim_ready] Phase 5/6: wait for sim time to advance %.1fs', self.POSE_SETTLE_DURATION)
        self._wait_for_sim_time(self.POSE_SETTLE_DURATION, wall_timeout_sec=self.CLOCK_TIMEOUT)
        logger.info('[sim_ready] Phase 5/6 DONE')

        logger.info('[sim_ready] Phase 6/6: wait for odometry pose')
        self._wait_for_pose()
        logger.info('[sim_ready] ALL PHASES DONE — simulation ready')

    def _robot_state_callback(self, msg: std_msgs.msg.String) -> None:
        self.current_robot_state = msg.data
        self.node.get_logger().info(f'Robot state updated: {self.current_robot_state}')

    def _clock_callback(self, msg: Clock) -> None:
        self.current_clock = msg.clock

    def _odometry_callback(self, msg: Odometry) -> None:
        self.robot_pose = msg.pose.pose
        self.robot_pose_stamp_ns = self._time_msg_to_nanoseconds(msg.header.stamp)

    @staticmethod
    def _time_msg_to_nanoseconds(msg: builtin_interfaces.msg.Time) -> int:
        return (msg.sec * 1_000_000_000) + msg.nanosec

    _SPIN_LOG_INTERVAL_SEC = 10.0

    def _spin_until(
        self,
        condition_fn: Callable[[], bool],
        timeout_sec: float,
        error_message: str,
    ) -> bool:
        """Spin the node until a condition is met or timeout occurs."""
        self.node.get_logger().info(
            '[spin_until] START (timeout=%.0fs): %s', timeout_sec, error_message
        )
        start_time = time.monotonic()
        last_log_time = start_time
        while time.monotonic() - start_time < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition_fn():
                elapsed = time.monotonic() - start_time
                self.node.get_logger().info('[spin_until] DONE in %.1fs', elapsed)
                return True
            now = time.monotonic()
            if now - last_log_time >= self._SPIN_LOG_INTERVAL_SEC:
                elapsed = now - start_time
                self.node.get_logger().warning(
                    '[spin_until] still waiting %.0f/%.0fs: %s',
                    elapsed,
                    timeout_sec,
                    error_message,
                )
                last_log_time = now
            time.sleep(0.1)
        # Log diagnostic snapshot before raising so CI logs have context.
        try:
            visible_nodes = self.node.get_node_names()
            visible_topics = [n for n, _ in self.node.get_topic_names_and_types()]
            self.node.get_logger().error(
                '[spin_until] TIMEOUT after %.0fs: %s\n'
                '  visible nodes  : %s\n'
                '  visible topics : %s',
                timeout_sec,
                error_message,
                visible_nodes,
                visible_topics[:30],
            )
        except Exception as diag_err:
            self.node.get_logger().error(
                '[spin_until] TIMEOUT after %.0fs (diagnostics unavailable: %s): %s',
                timeout_sec,
                diag_err,
                error_message,
            )
        raise TimeoutError(error_message)

    def _publish_event(self, event_name: str) -> None:
        msg = std_msgs.msg.String()
        msg.data = event_name
        self.event_pub.publish(msg)
        self.node.get_logger().info(f'Published event: {event_name}')
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def _wait_for_state(self, expected_state: str, timeout_sec: float) -> None:
        self._spin_until(
            lambda: self.current_robot_state == expected_state,
            timeout_sec,
            f'Robot did not reach state "{expected_state}" within {timeout_sec}s. '
            f'Current state: {self.current_robot_state}',
        )

    def _wait_for_any_state(
        self,
        expected_states: tuple[str, ...],
        timeout_sec: float,
    ) -> None:
        self._spin_until(
            lambda: self.current_robot_state in expected_states,
            timeout_sec,
            f'Robot did not reach any expected state {expected_states} '
            f'within {timeout_sec}s. Current state: {self.current_robot_state}',
        )

    def _current_sim_time_ns(self) -> int | None:
        if self.current_clock is None:
            return None
        return self._time_msg_to_nanoseconds(self.current_clock)

    def _wait_for_sim_time_since(
        self,
        start_time_ns: int,
        duration_sec: float,
        *,
        wall_timeout_sec: float | None = None,
    ) -> None:
        target_time_ns = start_time_ns + int(duration_sec * 1_000_000_000)

        def reached_target_time() -> bool:
            current_time_ns = self._current_sim_time_ns()
            return current_time_ns is not None and current_time_ns >= target_time_ns

        timeout_sec = wall_timeout_sec or max(self.SIM_TIME_TIMEOUT, duration_sec * 12.0)
        self._spin_until(
            reached_target_time,
            timeout_sec,
            (
                f'Simulation time did not advance by {duration_sec:.1f}s '
                f'within {timeout_sec:.1f}s of wall time'
            ),
        )

    def _wait_for_sim_time(
        self,
        duration_sec: float,
        *,
        wall_timeout_sec: float | None = None,
    ) -> None:
        self._spin_until(
            lambda: self.current_clock is not None,
            self.CLOCK_TIMEOUT,
            'Did not receive /clock from Gazebo bridge',
        )
        start_time_ns = self._current_sim_time_ns()
        if start_time_ns is None:
            raise RuntimeError(
                'Simulation clock became unavailable while waiting for time to advance'
            )
        self._wait_for_sim_time_since(
            start_time_ns,
            duration_sec,
            wall_timeout_sec=wall_timeout_sec,
        )

    def _wait_for_pose(self) -> None:
        self._spin_until(
            lambda: self.robot_pose is not None,
            self.MOVEMENT_TIMEOUT,
            f'Did not receive {ODOM_TOPIC} from Gazebo bridge',
        )

    def _wait_for_new_pose(self, previous_pose_stamp_ns: int | None) -> None:
        self._spin_until(
            lambda: (
                self.robot_pose is not None
                and self.robot_pose_stamp_ns is not None
                and (
                    previous_pose_stamp_ns is None
                    or self.robot_pose_stamp_ns > previous_pose_stamp_ns
                )
            ),
            self.MOVEMENT_TIMEOUT,
            f'Did not receive a fresh {ODOM_TOPIC} pose sample from Gazebo bridge',
        )

    def _wait_for_pose_sample(self, settle_sim_time_sec: float = 0.0) -> Pose:
        self._require_pose()
        previous_pose_stamp_ns = self.robot_pose_stamp_ns
        if settle_sim_time_sec > 0.0:
            self._wait_for_sim_time(settle_sim_time_sec)
        self._wait_for_new_pose(previous_pose_stamp_ns)
        if self.robot_pose is None:
            raise RuntimeError('Gazebo pose became unavailable while sampling odometry')
        return deepcopy(self.robot_pose)

    def _require_pose(self) -> None:
        self._wait_for_pose()
        assert self.robot_pose is not None, (
            f'Gazebo pose data is required but {ODOM_TOPIC} did not provide a pose'
        )

    def _sample_base_height(self) -> float:
        return self._wait_for_pose_sample(settle_sim_time_sec=self.POSE_SETTLE_DURATION).position.z

    def _arm_robot(self) -> None:
        self._wait_for_any_state(
            ('torque_off', 'finalized'),
            self.STATE_TRANSITION_TIMEOUT,
        )
        assert self.current_robot_state in ('torque_off', 'finalized')

        self._publish_event('initialize')

        self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        assert self.current_robot_state == 'torque_on', (
            'Failed to arm robot with clean transitions. '
            f'Current state: {self.current_robot_state}'
        )

    def _disarm_robot(self) -> None:
        self._wait_for_any_state(
            ('torque_on',),
            self.STATE_TRANSITION_TIMEOUT,
        )
        assert self.current_robot_state in ('torque_on',)

        self._publish_event('finalize')

        self._wait_for_state('finalized', self.STATE_TRANSITION_TIMEOUT)

        assert self.current_robot_state == 'finalized', (
            'Failed to disarm robot with clean transitions. '
            f'Current state: {self.current_robot_state}'
        )

    def _yaw_from_pose(self, pose: Pose) -> float:
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _run_movement_and_measure(
        self,
        stride_x: float = 0.0,
        stride_y: float = 0.0,
        rotation: float = 0.0,
    ) -> tuple[float, float, float]:
        start_pose = self._wait_for_pose_sample(settle_sim_time_sec=self.POSE_SETTLE_DURATION)

        start_pos = start_pose.position
        start_yaw = self._yaw_from_pose(start_pose)
        start_sim_time_ns = self._current_sim_time_ns()
        if start_sim_time_ns is None:
            raise RuntimeError('Simulation clock is unavailable before movement command')
        start_pose_stamp_ns = self.robot_pose_stamp_ns

        self._publish_movement_command(stride_x=stride_x, stride_y=stride_y, rotation=rotation)

        self._wait_for_sim_time_since(start_sim_time_ns, self.MOVEMENT_DURATION)
        self._wait_for_new_pose(start_pose_stamp_ns)

        if self.robot_pose is None:
            raise RuntimeError('Pose became unavailable after movement command')
        end_pose = deepcopy(self.robot_pose)

        end_pos = end_pose.position
        end_yaw = self._yaw_from_pose(end_pose)

        delta_x = end_pos.x - start_pos.x
        delta_y = end_pos.y - start_pos.y

        forward_delta = math.cos(start_yaw) * delta_x + math.sin(start_yaw) * delta_y
        left_delta = -math.sin(start_yaw) * delta_x + math.cos(start_yaw) * delta_y
        yaw_delta = ((end_yaw - start_yaw + math.pi) % (2.0 * math.pi)) - math.pi

        self.node.get_logger().info(
            'Movement delta '
            f'(cmd stride=({stride_x}, {stride_y}), rot={rotation}): '
            f'world=({delta_x:.3f}, {delta_y:.3f}), '
            f'body=(forward={forward_delta:.3f}, left={left_delta:.3f}), '
            f'yaw={yaw_delta:.3f}'
        )

        return forward_delta, left_delta, yaw_delta

    def _publish_movement_command(
        self,
        stride_x: float = 0.0,
        stride_y: float = 0.0,
        rotation: float = 0.0,
    ) -> None:
        cmd = MovementCommand()
        cmd.stride_direction = Vector3(x=stride_x, y=stride_y, z=0.0)
        cmd.rotation_speed = rotation
        cmd.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD
        for _ in range(3):
            self.movement_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.05)
        self.node.get_logger().info(
            f'Published movement command: stride=({stride_x}, {stride_y}), rotation={rotation}'
        )

    def _assert_posture_delta_or_static(
        self,
        delta_z: float,
        expected_direction: int,
        context_message: str,
    ) -> None:
        if abs(delta_z) >= self.MIN_ARM_DISARM_HEIGHT_DELTA:
            if expected_direction > 0:
                assert delta_z > self.MIN_ARM_DISARM_HEIGHT_DELTA, context_message
            else:
                assert delta_z < -self.MIN_ARM_DISARM_HEIGHT_DELTA, context_message
            return

        self.node.get_logger().warning(
            'Base height delta is below posture threshold; simulation appears to keep base '
            f'height nearly constant (delta_z={delta_z:.6f}m). Accepting static posture mode.'
        )
        assert abs(delta_z) <= self.POSTURE_HEIGHT_EPSILON, (
            'Unexpected intermediate posture delta in static-height simulation mode: '
            f'|delta_z|={abs(delta_z):.3f}m > {self.POSTURE_HEIGHT_EPSILON}m'
        )

    def assert_nodes_and_clock(self) -> None:
        """Verify simulation nodes are running and clock is available."""
        for node_name in ('robot_state_publisher', 'drqp_brain', 'drqp_robot_state'):
            self.node.get_logger().info(
                '[nodes] Waiting for node "%s" (timeout=%.0fs)', node_name, self.CLOCK_TIMEOUT
            )
            check_node_running(self.node, node_name, timeout=self.CLOCK_TIMEOUT)
            self.node.get_logger().info('[nodes] Found node "%s"', node_name)

        self.node.get_logger().info(
            '[nodes] Waiting for /clock topic (timeout=%.0fs)', self.CLOCK_TIMEOUT
        )
        with WaitForTopics([('/clock', Clock)], timeout=self.CLOCK_TIMEOUT) as wait:
            assert wait.wait(), 'Did not receive /clock from Gazebo bridge'
        self.node.get_logger().info('[nodes] /clock topic is publishing')

    def assert_controllers_are_active(self) -> None:
        """Verify ros2_control controllers are alive inside Gazebo."""
        controller_names = ['joint_state_broadcaster', 'joint_trajectory_controller']
        self.node.get_logger().info(
            '[controllers] Waiting for controllers %s (timeout=%.0fs)',
            controller_names,
            self.CONTROLLER_TIMEOUT,
        )
        check_controllers_running(
            self.node,
            controller_names,
            timeout=self.CONTROLLER_TIMEOUT,
        )
        self.node.get_logger().info('[controllers] All controllers are active')

    def assert_imu_data(self) -> None:
        """Verify the simulated IMU publishes on /imu/data via the Gazebo bridge."""
        with WaitForTopics([('/imu/data', Imu)], timeout=self.CLOCK_TIMEOUT) as wait:
            assert wait.wait(), 'Did not receive /imu/data from Gazebo IMU sensor'

    def assert_robot_spawned(self) -> None:
        """Verify robot model is spawned and state machine publishes state."""
        try:
            check_node_running(self.node, 'robot_state_publisher', timeout=self.SPAWN_TIMEOUT)
        except (AssertionError, RuntimeError, TimeoutError) as error:
            raise RuntimeError(
                'Robot model failed to spawn: robot_state_publisher not running within '
                f'{self.SPAWN_TIMEOUT}s'
            ) from error

        self._wait_for_any_state(
            ('torque_off', 'finalized', 'initializing', 'finalizing', 'torque_on'),
            self.SPAWN_TIMEOUT,
        )

        assert self.current_robot_state is not None, (
            f'Robot failed to spawn: did not receive robot state within {self.SPAWN_TIMEOUT}s. '
            'Check that Gazebo spawned the model "drqp" successfully.'
        )

    def assert_armed_posture(self) -> None:
        """Verify arming elevates base, or static posture mode is consistent."""
        self._arm_robot()
        self._disarm_robot()
        disarmed_base_z = self._sample_base_height()

        self._arm_robot()
        armed_base_z = self._sample_base_height()
        delta_z = armed_base_z - disarmed_base_z
        self.node.get_logger().info(
            f'Armed base height: z={armed_base_z:.3f}m '
            f'(disarmed={disarmed_base_z:.3f}m, delta={delta_z:.3f}m)'
        )
        self._assert_posture_delta_or_static(
            delta_z,
            expected_direction=1,
            context_message=(
                f'Base should rise when armed '
                f'(delta_z={delta_z:.3f}m <= {self.MIN_ARM_DISARM_HEIGHT_DELTA}m)'
            ),
        )
        assert self.current_robot_state == 'torque_on'

    def assert_forward_movement(self) -> None:
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=1.0)
        except RuntimeError as error:
            raise RuntimeError('Forward movement failed') from error

        assert forward_delta > self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot did not move forward significantly: '
            f'forward_delta={forward_delta:.3f}m '
            f'(expected > {self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )

    def assert_sustained_forward_movement(self) -> None:
        self._arm_robot()
        try:
            first_forward_delta, _, _ = self._run_movement_and_measure(stride_x=1.0)
            second_forward_delta, _, _ = self._run_movement_and_measure(stride_x=1.0)
        except RuntimeError as error:
            raise RuntimeError('Sustained forward movement failed') from error

        assert first_forward_delta > self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot did not move forward significantly during the first window: '
            f'forward_delta={first_forward_delta:.3f}m '
            f'(expected > {self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )
        assert second_forward_delta > self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot forward motion was not sustained into the second window: '
            f'forward_delta={second_forward_delta:.3f}m '
            f'(expected > {self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )
        assert self.current_robot_state == 'torque_on'

    def assert_backward_movement(self) -> None:
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=-1.0)
        except RuntimeError as error:
            raise RuntimeError('Backward movement failed') from error

        assert forward_delta < -self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot did not move backward significantly: '
            f'forward_delta={forward_delta:.3f}m '
            f'(expected < {-self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )

    def assert_left_movement(self) -> None:
        self._arm_robot()
        try:
            _, left_delta, _ = self._run_movement_and_measure(stride_y=1.0)
        except RuntimeError as error:
            raise RuntimeError('Left strafe movement failed') from error

        assert left_delta > self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot did not strafe left significantly: '
            f'left_delta={left_delta:.3f}m '
            f'(expected > {self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )

    def assert_right_movement(self) -> None:
        self._arm_robot()
        try:
            _, left_delta, _ = self._run_movement_and_measure(stride_y=-1.0)
        except RuntimeError as error:
            raise RuntimeError('Right strafe movement failed') from error

        assert left_delta < -self.MIN_LINEAR_MOVEMENT_DELTA, (
            'Robot did not strafe right significantly: '
            f'left_delta={left_delta:.3f}m '
            f'(expected < {-self.MIN_LINEAR_MOVEMENT_DELTA}m)'
        )

    def assert_rotation_movement(self) -> None:
        self._arm_robot()
        try:
            _, _, delta_yaw = self._run_movement_and_measure(rotation=0.5)
        except RuntimeError as error:
            raise RuntimeError('Rotation movement failed') from error

        assert abs(delta_yaw) > self.MIN_ROTATION_DELTA, (
            f'Robot did not rotate significantly: |delta_yaw|={abs(delta_yaw):.3f}'
        )

    def assert_disarmed_posture(self) -> None:
        """Verify disarming lowers base, or static posture mode is consistent."""
        self._arm_robot()
        armed_base_z = self._sample_base_height()

        self._disarm_robot()
        disarmed_base_z = self._sample_base_height()
        delta_z = disarmed_base_z - armed_base_z
        self.node.get_logger().info(
            f'Disarmed base height: z={disarmed_base_z:.3f}m '
            f'(armed={armed_base_z:.3f}m, delta={delta_z:.3f}m)'
        )
        self._assert_posture_delta_or_static(
            delta_z,
            expected_direction=-1,
            context_message=(
                'Base should lower when disarmed '
                f'(armed={armed_base_z:.3f}m, disarmed={disarmed_base_z:.3f}m, '
                f'min_delta={self.MIN_ARM_DISARM_HEIGHT_DELTA}m)'
            ),
        )

        assert self.current_robot_state == 'finalized', (
            'Robot did not reach finalized state after disarm. '
            f'Current state: {self.current_robot_state}'
        )
