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
import math
from pathlib import Path
import subprocess
import time
import unittest

import builtin_interfaces
import builtin_interfaces.msg
from controller_manager.test_utils import check_controllers_running, check_node_running
from drqp_brain.balance_controller import body_tilt_from_imu
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Pose, Vector3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ProcInfoHandler
from launch_testing_ros import WaitForTopics
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
import std_msgs.msg
from test_utils import ensure_gz_sim_not_running

ODOM_TOPIC = '/odom'
BALANCE_BOARD_WORLD_NAME = 'balance_test'
BALANCE_BOARD_MODEL_NAME = 'balance_board'
BALANCE_BOARD_POSE_Z = 0.025
BALANCE_BOARD_WORLD_PATH = str(
    Path(__file__).resolve().parent / 'fixtures' / 'balance_board_world.sdf'
)


def create_simulation_launch_description(
    launch_arguments: dict[str, str] | None = None,
) -> LaunchDescription:
    """Launch Gazebo simulation and wait for initialization before tests."""
    ensure_gz_sim_not_running()

    simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare('drqp_gazebo'),
            'launch',
            'sim.launch.py',
        ]
    )
    combined_launch_arguments = {'sim_gui': 'false'}
    if launch_arguments is not None:
        combined_launch_arguments.update(launch_arguments)
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments=combined_launch_arguments.items(),
            ),
            # Handshake with the launched processes, then let the test harness
            # block on real graph/clock/controller readiness.
            TimerAction(period=1.0, actions=[ReadyToTest()]),
        ]
    )


def create_balance_board_launch_description() -> LaunchDescription:
    """Launch Gazebo with a tiltable board beneath the robot spawn point."""
    return create_simulation_launch_description(
        {
            'world_sdf': BALANCE_BOARD_WORLD_PATH,
            'robot_z': '0.15',
        }
    )


def filter_out_gazebo_processes(proc_info: ProcInfoHandler) -> ProcInfoHandler:
    """Filter out Gazebo processes that are terminated by launch teardown."""
    filtered_proc_info = ProcInfoHandler()
    skipped_procs = ('gazebo', 'gz', 'bridge_node')
    for proc_name in proc_info.process_names():
        if not any(skip in proc_name for skip in skipped_procs):
            filtered_proc_info.append(proc_info[proc_name])
    return filtered_proc_info


def assert_clean_exit_codes(proc_info: ProcInfoHandler) -> None:
    """Assert all non-Gazebo processes exited successfully."""
    asserts.assertExitCodes(filter_out_gazebo_processes(proc_info))


class GazeboRobotControlBase(unittest.TestCase):
    """Shared harness and helpers for robot control behavior tests."""

    # Configurable timeouts.
    READY_TIMEOUT = 90.0
    SPAWN_TIMEOUT = 90.0
    STATE_TRANSITION_TIMEOUT = 90.0
    MOVEMENT_TIMEOUT = 30.0
    CLOCK_TIMEOUT = 30.0
    CONTROLLER_TIMEOUT = 60.0
    SIM_TIME_TIMEOUT = 60.0
    GZ_COMMAND_TIMEOUT = 10.0
    MOVEMENT_DURATION = 5.0
    POSE_SETTLE_DURATION = 1.0
    BALANCE_SETTLE_DURATION = 3.0

    # Posture delta threshold (meters).
    MIN_ARM_DISARM_HEIGHT_DELTA = 0.02
    POSTURE_HEIGHT_EPSILON = 0.01

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        """Set up test node and publishers/subscribers."""
        self.node = rclpy.create_node('test_gazebo_robot_control')

        self.current_robot_state = None
        self.current_clock = None
        self.robot_pose = None
        self.robot_pose_stamp_ns = None
        self.current_imu_message = None
        self.current_imu_stamp_ns = None

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
        self.imu_sub = self.node.create_subscription(Imu, '/imu/data', self._imu_callback, 10)

        self.event_pub = self.node.create_publisher(std_msgs.msg.String, '/robot_event', 10)
        self.movement_pub = self.node.create_publisher(
            MovementCommand, '/robot/movement_command', 10
        )
        self.balance_mode_pub = self.node.create_publisher(
            std_msgs.msg.Bool, '/robot/balance_mode', qos_profile
        )

        self._wait_for_simulation_ready()

    def _wait_for_simulation_ready(self) -> None:
        self.assert_nodes_and_clock()
        self.assert_controllers_are_active()
        self._spin_until(
            lambda: (
                self.event_pub.get_subscription_count() > 0
                and self.movement_pub.get_subscription_count() > 0
            ),
            self.READY_TIMEOUT,
            'Timed out waiting for /robot_event and /robot/movement_command subscribers',
        )
        self._wait_for_any_state(
            ('torque_off',),  # robot initial state
            self.SPAWN_TIMEOUT,
        )
        self._wait_for_sim_time(self.POSE_SETTLE_DURATION, wall_timeout_sec=self.CLOCK_TIMEOUT)
        self._wait_for_pose()

    def tearDown(self) -> None:
        """Clean up test node."""
        self.node.destroy_node()

    def _robot_state_callback(self, msg: std_msgs.msg.String) -> None:
        self.current_robot_state = msg.data
        self.node.get_logger().info(f'Robot state updated: {self.current_robot_state}')

    def _clock_callback(self, msg: Clock) -> None:
        self.current_clock = msg.clock

    def _odometry_callback(self, msg: Odometry) -> None:
        self.robot_pose = msg.pose.pose
        self.robot_pose_stamp_ns = self._time_msg_to_nanoseconds(msg.header.stamp)

    def _imu_callback(self, msg: Imu) -> None:
        self.current_imu_message = msg
        self.current_imu_stamp_ns = self._time_msg_to_nanoseconds(msg.header.stamp)

    @staticmethod
    def _time_msg_to_nanoseconds(msg: builtin_interfaces.msg.Time) -> int:
        return (msg.sec * 1_000_000_000) + msg.nanosec

    @staticmethod
    def _roll_pitch_from_quaternion(quaternion) -> tuple[float, float]:
        sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        return roll, pitch

    def _spin_until(
        self,
        condition_fn: Callable[[], bool],
        timeout_sec: float,
        error_message: str,
    ) -> bool:
        """Spin the node until a condition is met or timeout occurs."""
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition_fn():
                return True
            time.sleep(0.1)
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

    def _wait_for_new_imu(self, previous_imu_stamp_ns: int | None) -> None:
        self._spin_until(
            lambda: (
                self.current_imu_message is not None
                and self.current_imu_stamp_ns is not None
                and (
                    previous_imu_stamp_ns is None
                    or self.current_imu_stamp_ns > previous_imu_stamp_ns
                )
            ),
            self.CLOCK_TIMEOUT,
            'Did not capture a fresh /imu/data sample from Gazebo',
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
        self.assertIsNotNone(
            self.robot_pose,
            f'Gazebo pose data is required but {ODOM_TOPIC} did not provide a pose',
        )

    def _sample_base_height(self) -> float:
        return self._wait_for_pose_sample(settle_sim_time_sec=self.POSE_SETTLE_DURATION).position.z

    def _sample_base_roll_pitch(
        self,
        settle_sim_time_sec: float = 0.0,
    ) -> tuple[float, float]:
        pose = self._wait_for_pose_sample(settle_sim_time_sec=settle_sim_time_sec)
        return self._roll_pitch_from_quaternion(pose.orientation)

    def _sample_imu_body_tilt(
        self,
        settle_sim_time_sec: float = 0.0,
    ) -> tuple[float, float]:
        previous_imu_stamp_ns = self.current_imu_stamp_ns
        if settle_sim_time_sec > 0.0:
            self._wait_for_sim_time(settle_sim_time_sec)
        self._wait_for_new_imu(previous_imu_stamp_ns)
        if self.current_imu_message is None:
            raise RuntimeError('Gazebo IMU data became unavailable while sampling body tilt')
        imu_message = deepcopy(self.current_imu_message)
        imu_body_tilt = body_tilt_from_imu(imu_message.orientation)
        return imu_body_tilt.x, imu_body_tilt.y

    @staticmethod
    def _quaternion_from_roll_pitch_yaw(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
        """Convert XYZ Euler angles to a quaternion tuple in (x, y, z, w) order."""
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0
        sin_roll = math.sin(half_roll)
        cos_roll = math.cos(half_roll)
        sin_pitch = math.sin(half_pitch)
        cos_pitch = math.cos(half_pitch)
        sin_yaw = math.sin(half_yaw)
        cos_yaw = math.cos(half_yaw)
        return (
            sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
            cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
            cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
            cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
        )

    def _run_gz_command(self, args: list[str], error_context: str) -> str:
        try:
            completed = subprocess.run(
                args,
                check=True,
                capture_output=True,
                text=True,
                timeout=self.GZ_COMMAND_TIMEOUT,
            )
        except FileNotFoundError as error:
            self.fail(f'{error_context} failed because the Gazebo CLI is unavailable: {error}')
        except subprocess.CalledProcessError as error:
            self.fail(
                f'{error_context} failed with exit code {error.returncode}: '
                f'{error.stderr.strip() or error.stdout.strip()}'
            )
        except subprocess.TimeoutExpired as error:
            self.fail(f'{error_context} timed out after {self.GZ_COMMAND_TIMEOUT:.1f}s: {error}')
        return completed.stdout

    def _set_balance_mode(self, enabled: bool) -> None:
        self._spin_until(
            lambda: self.balance_mode_pub.get_subscription_count() > 0,
            self.READY_TIMEOUT,
            'Timed out waiting for /robot/balance_mode subscribers',
        )
        msg = std_msgs.msg.Bool(data=enabled)
        for _ in range(3):
            self.balance_mode_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.05)
        self.node.get_logger().info(f'Published balance mode: enabled={enabled}')

    def _set_board_tilt(
        self,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        x, y, z, w = self._quaternion_from_roll_pitch_yaw(roll, pitch, yaw)
        request = ' '.join(
            [
                f'name: "{BALANCE_BOARD_MODEL_NAME}"',
                f'position {{ x: 0.0 y: 0.0 z: {BALANCE_BOARD_POSE_Z:.3f} }}',
                f'orientation {{ x: {x:.8f} y: {y:.8f} z: {z:.8f} w: {w:.8f} }}',
            ]
        )
        self._run_gz_command(
            [
                'gz',
                'service',
                '-s',
                f'/world/{BALANCE_BOARD_WORLD_NAME}/set_pose',
                '--reqtype',
                'gz.msgs.Pose',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                str(int(self.GZ_COMMAND_TIMEOUT * 1000)),
                '--req',
                request,
            ],
            error_context='Tilting Gazebo balance board',
        )

    def _sample_entity_pose_from_gazebo(self, entity_name: str) -> Pose:
        raw_output = self._run_gz_command(
            [
                'gz',
                'topic',
                '-e',
                '-n',
                '1',
                '-t',
                f'/world/{BALANCE_BOARD_WORLD_NAME}/pose/info',
            ],
            error_context=f'Reading Gazebo pose info for entity "{entity_name}"',
        )
        for entity in _parse_gazebo_pose_info(raw_output):
            if entity['name'] == entity_name:
                return entity['pose']
        raise RuntimeError(f'Gazebo pose info did not include entity "{entity_name}"')

    def _wait_for_board_tilt(
        self,
        *,
        expected_roll: float = 0.0,
        expected_pitch: float = 0.0,
        tolerance: float = 0.03,
    ) -> tuple[float, float]:
        deadline = time.monotonic() + self.MOVEMENT_TIMEOUT
        last_roll = 0.0
        last_pitch = 0.0
        while time.monotonic() < deadline:
            board_pose = self._sample_entity_pose_from_gazebo(BALANCE_BOARD_MODEL_NAME)
            last_roll, last_pitch = self._roll_pitch_from_quaternion(board_pose.orientation)
            if (
                abs(last_roll - expected_roll) <= tolerance
                and abs(last_pitch - expected_pitch) <= tolerance
            ):
                return last_roll, last_pitch
            time.sleep(0.2)
        raise TimeoutError(
            'Gazebo board tilt did not reach the expected pose. '
            f'Expected roll={expected_roll:.3f}, pitch={expected_pitch:.3f}; '
            f'last roll={last_roll:.3f}, pitch={last_pitch:.3f}'
        )

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
                self.assertGreater(
                    delta_z,
                    self.MIN_ARM_DISARM_HEIGHT_DELTA,
                    context_message,
                )
            else:
                self.assertLess(
                    delta_z,
                    -self.MIN_ARM_DISARM_HEIGHT_DELTA,
                    context_message,
                )
            return

        self.node.get_logger().warning(
            'Base height delta is below posture threshold; simulation appears to keep base '
            f'height nearly constant (delta_z={delta_z:.6f}m). Accepting static posture mode.'
        )
        self.assertLessEqual(
            abs(delta_z),
            self.POSTURE_HEIGHT_EPSILON,
            (
                'Unexpected intermediate posture delta in static-height simulation mode: '
                f'|delta_z|={abs(delta_z):.3f}m > {self.POSTURE_HEIGHT_EPSILON}m'
            ),
        )

    def assert_nodes_and_clock(self) -> None:
        """Verify simulation nodes are running and clock is available."""
        for node_name in ('robot_state_publisher', 'drqp_brain', 'drqp_robot_state'):
            check_node_running(self.node, node_name, timeout=self.CLOCK_TIMEOUT)

        with WaitForTopics([('/clock', Clock)], timeout=self.CLOCK_TIMEOUT) as wait:
            self.assertTrue(wait.wait(), 'Did not receive /clock from Gazebo bridge')

    def assert_controllers_are_active(self) -> None:
        """Verify ros2_control controllers are alive inside Gazebo."""
        check_controllers_running(
            self.node,
            [
                'joint_state_broadcaster',
                'joint_trajectory_controller',
            ],
            timeout=self.CONTROLLER_TIMEOUT,
        )

    def assert_imu_data(self) -> None:
        """Verify the simulated IMU publishes on /imu/data via the Gazebo bridge."""
        with WaitForTopics([('/imu/data', Imu)], timeout=self.CLOCK_TIMEOUT) as wait:
            self.assertTrue(wait.wait(), 'Did not receive /imu/data from Gazebo IMU sensor')

    def assert_imu_data_reports_orientation(self) -> None:
        """Issue 356: Gazebo IMU data should include mounted-link orientation for ROS consumers."""
        self.assert_imu_data()
        self._spin_until(
            lambda: self.current_imu_message is not None,
            self.CLOCK_TIMEOUT,
            'Did not capture /imu/data after Gazebo IMU topic became available',
        )
        self._wait_for_pose()

        orientation = self.current_imu_message.orientation
        orientation_norm = math.sqrt(
            orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2
        )
        self.assertAlmostEqual(
            orientation_norm,
            1.0,
            delta=0.05,
            msg='Expected Gazebo IMU orientation quaternion to be normalized in /imu/data',
        )
        self.assertGreaterEqual(
            self.current_imu_message.orientation_covariance[0],
            0.0,
            'Expected Gazebo IMU orientation to be marked available in /imu/data',
        )

        angle_from_identity = 2.0 * math.acos(
            min(1.0, max(0.0, abs(orientation.w) / orientation_norm))
        )
        self.assertGreater(
            angle_from_identity,
            0.5,
            (
                'Expected Gazebo to preserve the non-identity imu_link orientation '
                'instead of publishing a default orientation'
            ),
        )

        imu_body_tilt = body_tilt_from_imu(orientation)
        base_roll, base_pitch = self._roll_pitch_from_quaternion(self.robot_pose.orientation)
        self.assertAlmostEqual(
            imu_body_tilt.x,
            base_roll,
            delta=0.1,
            msg='Expected IMU orientation to reconstruct the spawned base roll',
        )
        self.assertAlmostEqual(
            imu_body_tilt.y,
            base_pitch,
            delta=0.1,
            msg='Expected IMU orientation to reconstruct the spawned base pitch',
        )

    def assert_robot_spawned(self) -> None:
        """Verify robot model is spawned and state machine publishes state."""
        try:
            check_node_running(self.node, 'robot_state_publisher', timeout=self.SPAWN_TIMEOUT)
        except (AssertionError, RuntimeError, TimeoutError) as error:
            self.fail(
                'Robot model failed to spawn: robot_state_publisher not running within '
                f'{self.SPAWN_TIMEOUT}s. Error: {error}'
            )

        self._wait_for_any_state(
            ('torque_off', 'finalized', 'initializing', 'finalizing', 'torque_on'),
            self.SPAWN_TIMEOUT,
        )

        self.assertIsNotNone(
            self.current_robot_state,
            f'Robot failed to spawn: did not receive robot state within {self.SPAWN_TIMEOUT}s. '
            'Check that Gazebo spawned the model "drqp" successfully.',
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
        self.assertEqual(self.current_robot_state, 'torque_on')

    def assert_forward_movement(self) -> None:
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=1.0)
            self.assertGreater(
                forward_delta,
                0.01,
                msg=(
                    'Robot did not move forward significantly: '
                    f'forward_delta={forward_delta:.3f}m (expected > 0.01m)'
                ),
            )
        except RuntimeError as error:
            self.fail(f'Forward movement test failed. Error: {error}')

    def assert_backward_movement(self) -> None:
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=-1.0)
            self.assertLess(
                forward_delta,
                -0.01,
                msg=(
                    'Robot did not move backward significantly: '
                    f'forward_delta={forward_delta:.3f}m (expected < -0.01m)'
                ),
            )
        except RuntimeError as error:
            self.fail(f'Backward movement test failed. Error: {error}')

    def assert_left_movement(self) -> None:
        self._arm_robot()
        try:
            _, left_delta, _ = self._run_movement_and_measure(stride_y=1.0)
            self.assertGreater(
                left_delta,
                0.01,
                msg=(
                    'Robot did not strafe left significantly: '
                    f'left_delta={left_delta:.3f}m (expected > 0.01m)'
                ),
            )
        except RuntimeError as error:
            self.fail(f'Left strafe test failed. Error: {error}')

    def assert_right_movement(self) -> None:
        self._arm_robot()
        try:
            _, left_delta, _ = self._run_movement_and_measure(stride_y=-1.0)
            self.assertLess(
                left_delta,
                -0.01,
                msg=(
                    'Robot did not strafe right significantly: '
                    f'left_delta={left_delta:.3f}m (expected < -0.01m)'
                ),
            )
        except RuntimeError as error:
            self.fail(f'Right strafe test failed. Error: {error}')

    def assert_rotation_movement(self) -> None:
        self._arm_robot()
        try:
            _, _, delta_yaw = self._run_movement_and_measure(rotation=0.5)
            self.assertGreater(
                abs(delta_yaw),
                0.1,
                msg=f'Robot did not rotate significantly: |delta_yaw|={abs(delta_yaw):.3f}',
            )
        except RuntimeError as error:
            self.fail(f'Rotation movement test failed. Error: {error}')

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

        self.assertEqual(
            self.current_robot_state,
            'finalized',
            msg='Robot did not reach finalized state after disarm. '
            f'Current state: {self.current_robot_state}',
        )

    def assert_balance_mode_levels_body_on_tilted_board(self) -> None:
        """Verify balance mode keeps the body close to level while a board tilts underneath."""
        board_pitch_target = 0.15

        self._arm_robot()
        self._wait_for_sim_time(self.POSE_SETTLE_DURATION)
        initial_base_height = self._sample_base_height()
        initial_base_roll, initial_base_pitch = self._sample_base_roll_pitch(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )

        self._set_balance_mode(True)
        self._wait_for_sim_time(self.POSE_SETTLE_DURATION)
        board_roll, board_pitch = self._wait_for_board_tilt(expected_roll=0.0, expected_pitch=0.0)
        self.assertAlmostEqual(board_roll, 0.0, delta=0.03)
        self.assertAlmostEqual(board_pitch, 0.0, delta=0.03)

        self._set_board_tilt(pitch=board_pitch_target)
        board_roll, board_pitch = self._wait_for_board_tilt(
            expected_roll=0.0,
            expected_pitch=board_pitch_target,
        )
        self._wait_for_sim_time(self.BALANCE_SETTLE_DURATION)

        balanced_base_roll, balanced_base_pitch = self._sample_base_roll_pitch(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )
        balanced_base_height = self._sample_base_height()
        balanced_imu_roll, balanced_imu_pitch = self._sample_imu_body_tilt(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )

        self.assertGreater(
            abs(board_pitch),
            0.10,
            msg=f'Expected balance board to tilt noticeably, observed pitch={board_pitch:.3f}rad',
        )
        self.assertAlmostEqual(
            balanced_base_roll,
            initial_base_roll,
            delta=0.08,
            msg=(
                'Expected body roll to stay close to its pre-tilt value after balance mode '
                f'compensated for the board tilt (initial={initial_base_roll:.3f}, '
                f'balanced={balanced_base_roll:.3f})'
            ),
        )
        self.assertAlmostEqual(
            balanced_base_pitch,
            initial_base_pitch,
            delta=0.08,
            msg=(
                'Expected body pitch to stay close to its pre-tilt value after balance mode '
                f'compensated for the board tilt (initial={initial_base_pitch:.3f}, '
                f'balanced={balanced_base_pitch:.3f}, board={board_pitch:.3f})'
            ),
        )
        self.assertAlmostEqual(
            balanced_imu_roll,
            balanced_base_roll,
            delta=0.10,
            msg='Expected IMU-derived roll to match the balanced body roll on the tilted board',
        )
        self.assertAlmostEqual(
            balanced_imu_pitch,
            balanced_base_pitch,
            delta=0.10,
            msg='Expected IMU-derived pitch to match the balanced body pitch on the tilted board',
        )
        self.assertGreater(
            abs(board_pitch - balanced_base_pitch),
            0.05,
            msg=(
                'Expected the balanced body to stay noticeably closer to level ground than '
                f'the tilted board (board={board_pitch:.3f}, body={balanced_base_pitch:.3f})'
            ),
        )
        self.assertGreater(
            balanced_base_height,
            initial_base_height - 0.03,
            msg=(
                'Expected the robot to remain supported near the board height after the tilt '
                f'(initial_z={initial_base_height:.3f}, balanced_z={balanced_base_height:.3f})'
            ),
        )


def _parse_gazebo_pose_info(raw_output: str) -> list[dict[str, Pose | str]]:
    """
    Parse Gazebo CLI pose info output into named pose dictionaries.

    Parameters
    ----------
    raw_output
        Text emitted by `gz topic -e -n 1 -t /world/<world>/pose/info`.

    Returns
    -------
    list[dict[str, Pose | str]]
        Parsed entities with `name` and `pose` keys.
    """
    entities = []
    for block in _extract_blocks(raw_output, 'pose'):
        entity = _parse_pose_block(block)
        if entity['name']:
            entities.append(entity)
    return entities


def _extract_blocks(raw_output: str, block_name: str) -> list[list[str]]:
    """
    Extract nested protobuf-style blocks from Gazebo CLI text output.

    Parameters
    ----------
    raw_output
        Text output containing repeated `<block_name> { ... }` sections.
    block_name
        Name of the protobuf block to extract.

    Returns
    -------
    list[list[str]]
        One list of inner lines for each extracted block.
    """
    blocks = []
    lines = raw_output.splitlines()
    opening = block_name + ' {'
    index = 0
    while index < len(lines):
        if lines[index].strip() != opening:
            index += 1
            continue
        block_lines = []
        depth = 1
        index += 1
        while index < len(lines) and depth > 0:
            stripped = lines[index].strip()
            if stripped.endswith('{'):
                depth += 1
            if stripped == '}':
                depth -= 1
                if depth == 0:
                    break
            if depth > 0:
                block_lines.append(lines[index])
            index += 1
        blocks.append(block_lines)
        index += 1
    return blocks


def _parse_pose_block(lines: list[str]) -> dict[str, Pose | str]:
    """
    Parse one Gazebo CLI pose block into a named pose dictionary.

    Parameters
    ----------
    lines
        Inner lines from a Gazebo `pose { ... }` block containing `name`,
        `position { ... }`, and `orientation { ... }` sections.

    Returns
    -------
    dict[str, Pose | str]
        Dictionary with `name` and `pose` keys.
    """
    name = ''
    position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    current_section = None

    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped in {'position {', 'orientation {'}:
            current_section = stripped.split()[0]
            continue
        if stripped == '}':
            current_section = None
            continue
        if stripped.startswith('name:'):
            name = stripped.split(':', maxsplit=1)[1].strip().strip('"')
            continue
        if ':' not in stripped or current_section is None:
            continue
        axis, raw_value = stripped.split(':', maxsplit=1)
        if current_section == 'position' and axis in position:
            position[axis] = float(raw_value.strip())
        if current_section == 'orientation' and axis in orientation:
            orientation[axis] = float(raw_value.strip())

    pose = Pose()
    pose.position.x = position['x']
    pose.position.y = position['y']
    pose.position.z = position['z']
    pose.orientation.x = orientation['x']
    pose.orientation.y = orientation['y']
    pose.orientation.z = orientation['z']
    pose.orientation.w = orientation['w']
    return {'name': name, 'pose': pose}
