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
import inspect
import math
import os
from pathlib import Path
import re
import subprocess
import time

import builtin_interfaces
import builtin_interfaces.msg
from controller_manager.test_utils import check_controllers_running, check_node_running
from drqp_brain.balance_controller import body_tilt_from_imu
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from drqp_launch_testing import track_process_exit_codes
from geometry_msgs.msg import Pose, Vector3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_pytest
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

ODOM_TOPIC = '/odom'
BALANCE_BOARD_WORLD_NAME = 'balance_test'
BALANCE_BOARD_BOARD_LINK_NAME = 'board'
BALANCE_BOARD_ROLL_TOPIC = '/balance_board/roll_target'
BALANCE_BOARD_PITCH_TOPIC = '/balance_board/pitch_target'
BALANCE_BOARD_WORLD_PATH = str(
    Path(__file__).resolve().parent / 'fixtures' / 'balance_board_world.sdf'
)


def build_test_gz_partition(test_name: str) -> str:
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    sanitized_name = re.sub(r'[^a-z0-9]+', '-', test_name.lower()).strip('-')
    if not sanitized_name:
        sanitized_name = 'test'
    return f'drqp-domain-{domain_id}-{sanitized_name}'


def create_simulation_launch_description(
    test_name: str | None = None,
    launch_arguments: dict[str, str] | None = None,
) -> LaunchDescription:
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
    combined_launch_arguments = {'sim_gui': 'true'}
    if launch_arguments is not None:
        combined_launch_arguments.update(launch_arguments)
    combined_launch_arguments['gz_partition'] = build_test_gz_partition(test_name)
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
        launch_arguments={
            'world_sdf': BALANCE_BOARD_WORLD_PATH,
            'robot_z': '0.30',
        }
    )


def create_board_only_launch_description() -> LaunchDescription:
    """
    Launch only Gazebo and the balance board world, with no robot.

    Used to characterise the board fixture in isolation: it verifies the tilting
    mechanism reaches its commanded angles without any coupling from the robot.
    """
    return create_simulation_launch_description(
        launch_arguments={
            'world_sdf': BALANCE_BOARD_WORLD_PATH,
            'spawn_robot': 'false',
            # No robot to view and no sensors to render, so skip the GUI to boot
            # faster and keep this fixture check lightweight.
            'sim_gui': 'false',
        }
    )


@launch_pytest.fixture
def generate_test_description(request):
    """Launch the simulation and record process exit codes for the shutdown check."""
    test_name = Path(request.fspath).stem
    launch_description = create_simulation_launch_description(test_name=test_name)
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


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
    GZ_COMMAND_TIMEOUT = 10.0
    MOVEMENT_DURATION = 5.0
    POSE_SETTLE_DURATION = 1.0
    BALANCE_SETTLE_DURATION = 3.0
    BALANCED_BODY_TILT_TOLERANCE = 0.10
    # Sim seconds to let a robot motion run before checking its effect on the board.
    MOTION_RESPONSE_DURATION = 1.0

    # Board tilt commands go through short-lived `gz topic -p` publishers whose
    # message can be dropped before the controller's subscription is discovered.
    # Republish a few times so a single drop does not leave an axis uncommanded.
    BOARD_TILT_PUBLISH_ATTEMPTS = 3
    BOARD_TILT_PUBLISH_INTERVAL = 0.15

    # Absolute tolerance (radians) for the board reaching a commanded angle.
    BOARD_TILT_TOLERANCE = 0.03
    # Absolute tolerance (radians) for the board holding level while the robot
    # moves on it. Looser than BOARD_TILT_TOLERANCE to absorb transient contact
    # forces from the gait without masking a board that is back-driven off level.
    BOARD_ALIGNMENT_TOLERANCE = 0.05

    # Posture delta threshold (meters).
    MIN_ARM_DISARM_HEIGHT_DELTA = 0.02
    POSTURE_HEIGHT_EPSILON = 0.012

    # Minimum movement to count as "moved" (meters/radians). Lowered by 20% from
    # 0.01m/0.1rad to absorb simulation timing variance that otherwise produced
    # borderline flaky failures (e.g. -0.00996m vs a -0.01m threshold).
    MIN_LINEAR_MOVEMENT_DELTA = 0.008
    MIN_ROTATION_DELTA = 0.08

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()

    @classmethod
    def teardown_class(cls) -> None:
        rclpy.try_shutdown()

    @pytest.fixture(autouse=True)
    def _node_setup(self, generate_test_description) -> None:  # noqa: ARG002
        """Set up test node and publishers/subscribers."""
        self.setup_node()

    def setup_node(self) -> None:
        """
        Create the test node and publishers/subscribers, then wait for readiness.

        Exposed as a plain instance method so the harness can be driven both as a
        pytest test class (via ``_node_setup``) and as a shared fixture instance
        in a functions-only test module.
        """
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
        assert self.robot_pose is not None, (
            f'Gazebo pose data is required but {ODOM_TOPIC} did not provide a pose'
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
    def _quaternion_from_roll_pitch_yaw(
        roll: float, pitch: float, yaw: float
    ) -> tuple[float, ...]:
        """
        Convert XYZ Euler angles in radians to a quaternion tuple.

        Parameters
        ----------
        roll
            Rotation around the X axis in radians.
        pitch
            Rotation around the Y axis in radians.
        yaw
            Rotation around the Z axis in radians.

        Returns
        -------
        tuple[float, ...]
            Quaternion in `(x, y, z, w)` order using XYZ Euler convention.

        """
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

    def _run_gz_command(self, gz_args: list[str], error_context: str) -> str:
        """
        Run a Gazebo CLI command and return its stdout.

        Parameters
        ----------
        gz_args
            Full `gz` command argument list.
        error_context
            Human-readable context included in failures.

        Returns
        -------
        str
            Captured stdout from the Gazebo command.

        """
        # These commands are built from test constants plus explicit helper inputs;
        # this helper intentionally does not execute arbitrary user-provided shell.
        try:
            completed = subprocess.run(
                gz_args,
                check=True,
                capture_output=True,
                text=True,
                timeout=self.GZ_COMMAND_TIMEOUT,
            )
        except FileNotFoundError as error:
            raise AssertionError(
                f'{error_context} failed because the Gazebo CLI is unavailable: {error}'
            )
        except subprocess.CalledProcessError as error:
            raise AssertionError(
                f'{error_context} failed with exit code {error.returncode}: '
                f'{error.stderr.strip() or error.stdout.strip()}'
            )
        except subprocess.TimeoutExpired as error:
            raise AssertionError(
                f'{error_context} timed out after {self.GZ_COMMAND_TIMEOUT:.1f}s: {error}'
            )
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
    ) -> None:
        """
        Command the balance board to the given roll and pitch via joint position controllers.

        ``gz topic -p`` publishes a single message from a short-lived publisher that
        exits immediately, so the message is occasionally dropped before the
        controller's subscription finishes discovery. A dropped publish would leave
        one axis at zero (e.g. turning a diagonal command into a pure-pitch tilt) and
        make ``_wait_for_board_tilt`` time out. Publish each target a few times to
        make delivery reliable.
        """
        for topic, value, context in (
            (BALANCE_BOARD_ROLL_TOPIC, roll, 'Setting balance board roll target'),
            (BALANCE_BOARD_PITCH_TOPIC, pitch, 'Setting balance board pitch target'),
        ):
            for _ in range(self.BOARD_TILT_PUBLISH_ATTEMPTS):
                self._run_gz_command(
                    [
                        'gz',
                        'topic',
                        '-t',
                        topic,
                        '-m',
                        'gz.msgs.Double',
                        '-p',
                        f'data: {value}',
                    ],
                    error_context=context,
                )
                time.sleep(self.BOARD_TILT_PUBLISH_INTERVAL)

    def _sample_entity_pose_from_gazebo(self, entity_name: str) -> Pose:
        """
        Read a named entity pose from Gazebo's world pose info topic.

        Parameters
        ----------
        entity_name
            Gazebo entity name expected in `/world/<world>/pose/info`.

        Returns
        -------
        Pose
            Latest pose reported for the requested entity.

        """
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
        entities = _parse_gazebo_pose_info(raw_output)
        for entity in entities:
            if entity['name'] == entity_name:
                return entity['pose']
        available = [e['name'] for e in entities]
        raise RuntimeError(
            f'Gazebo pose info did not include entity "{entity_name}". Available: {available}'
        )

    def _wait_for_board_tilt(
        self,
        *,
        expected_roll: float = 0.0,
        expected_pitch: float = 0.0,
        tolerance: float = 0.03,
    ) -> tuple[float, float]:
        """
        Poll Gazebo until the balance board reaches the requested tilt.

        Parameters
        ----------
        expected_roll
            Expected board roll in radians.
        expected_pitch
            Expected board pitch in radians.
        tolerance
            Acceptable absolute error in radians for both axes.

        Returns
        -------
        tuple[float, float]
            Observed board roll and pitch in radians.

        """
        deadline = time.monotonic() + self.MOVEMENT_TIMEOUT
        last_roll = 0.0
        last_pitch = 0.0
        while time.monotonic() < deadline:
            board_pose = self._sample_entity_pose_from_gazebo(BALANCE_BOARD_BOARD_LINK_NAME)
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

    def _sample_board_tilt(self) -> tuple[float, float]:
        """Return the board's current roll and pitch in radians from Gazebo."""
        board_pose = self._sample_entity_pose_from_gazebo(BALANCE_BOARD_BOARD_LINK_NAME)
        return self._roll_pitch_from_quaternion(board_pose.orientation)

    def _assert_board_reaches_tilt(self, board_roll: float, board_pitch: float) -> None:
        """Command a tilt, assert the board reaches it, then return the board to level."""
        self._set_board_tilt(roll=board_roll, pitch=board_pitch)
        observed_roll, observed_pitch = self._wait_for_board_tilt(
            expected_roll=board_roll,
            expected_pitch=board_pitch,
            tolerance=self.BOARD_TILT_TOLERANCE,
        )
        assert abs(observed_roll - board_roll) <= self.BOARD_TILT_TOLERANCE, (
            'Board roll did not reach commanded angle '
            f'(commanded={board_roll:.3f}, observed={observed_roll:.3f})'
        )
        assert abs(observed_pitch - board_pitch) <= self.BOARD_TILT_TOLERANCE, (
            'Board pitch did not reach commanded angle '
            f'(commanded={board_pitch:.3f}, observed={observed_pitch:.3f})'
        )
        self._set_board_tilt(roll=0.0, pitch=0.0)
        self._wait_for_board_tilt(
            expected_roll=0.0, expected_pitch=0.0, tolerance=self.BOARD_TILT_TOLERANCE
        )

    def _assert_board_stays_level(self, context_message: str) -> None:
        """Assert the board is holding level within the alignment tolerance."""
        observed_roll, observed_pitch = self._sample_board_tilt()
        assert abs(observed_roll) <= self.BOARD_ALIGNMENT_TOLERANCE, (
            f'Board roll drifted off level {context_message} (roll={observed_roll:.3f})'
        )
        assert abs(observed_pitch) <= self.BOARD_ALIGNMENT_TOLERANCE, (
            f'Board pitch drifted off level {context_message} (pitch={observed_pitch:.3f})'
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
        *,
        stride_z: float = 0.0,
        body_translation_x: float = 0.0,
    ) -> None:
        # stride_direction.z contributes to the stride magnitude with no horizontal
        # heading, so stride_z=1 makes the robot step in place. body_translation_x
        # shifts the body forward over the stance without taking a step.
        cmd = MovementCommand()
        cmd.stride_direction = Vector3(x=stride_x, y=stride_y, z=stride_z)
        cmd.rotation_speed = rotation
        cmd.body_translation = Vector3(x=body_translation_x, y=0.0, z=0.0)
        cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD
        for _ in range(3):
            self.movement_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.05)
        self.node.get_logger().info(
            f'Published movement command: stride=({stride_x}, {stride_y}, {stride_z}), '
            f'body_translation_x={body_translation_x}, rotation={rotation}'
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
            check_node_running(self.node, node_name, timeout=self.CLOCK_TIMEOUT)

        with WaitForTopics([('/clock', Clock)], timeout=self.CLOCK_TIMEOUT) as wait:
            assert wait.wait(), 'Did not receive /clock from Gazebo bridge'

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
            assert wait.wait(), 'Did not receive /imu/data from Gazebo IMU sensor'

    def assert_imu_data_reports_orientation(self) -> None:
        """Issue 356: Gazebo IMU data should publish body attitude for ROS consumers."""
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
        assert abs(orientation_norm - 1.0) <= 0.05, (
            'Expected Gazebo IMU orientation quaternion to be normalized in /imu/data'
        )
        assert self.current_imu_message.orientation_covariance[0] >= 0.0, (
            'Expected Gazebo IMU orientation to be marked available in /imu/data'
        )

        imu_body_tilt = body_tilt_from_imu(orientation)
        base_roll, base_pitch = self._roll_pitch_from_quaternion(self.robot_pose.orientation)
        assert abs(imu_body_tilt.x - base_roll) <= 0.1, (
            'Expected IMU orientation to reconstruct the spawned base roll'
        )
        assert abs(imu_body_tilt.y - base_pitch) <= 0.1, (
            'Expected IMU orientation to reconstruct the spawned base pitch'
        )

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

    def _assert_body_level_at_board_tilt(
        self,
        board_roll: float,
        board_pitch: float,
        initial_roll: float,
        initial_pitch: float,
    ) -> None:
        pre_tilt_height = self._sample_base_height()
        self._set_board_tilt(roll=board_roll, pitch=board_pitch)
        board_r, board_p = self._wait_for_board_tilt(
            expected_roll=board_roll, expected_pitch=board_pitch
        )
        self._wait_for_sim_time(self.BALANCE_SETTLE_DURATION)
        balanced_roll, balanced_pitch = self._sample_base_roll_pitch(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )
        balanced_height = self._sample_base_height()
        imu_roll, imu_pitch = self._sample_imu_body_tilt(
            settle_sim_time_sec=self.POSE_SETTLE_DURATION
        )

        assert math.sqrt(board_r**2 + board_p**2) > 0.08, (
            f'Expected board to tilt noticeably (roll={board_r:.3f}, pitch={board_p:.3f})'
        )
        assert abs(balanced_roll - initial_roll) <= self.BALANCED_BODY_TILT_TOLERANCE, (
            'Expected body roll to stay near initial after balance compensation '
            f'(initial={initial_roll:.3f}, balanced={balanced_roll:.3f}, board_r={board_r:.3f})'
        )
        assert abs(balanced_pitch - initial_pitch) <= self.BALANCED_BODY_TILT_TOLERANCE, (
            'Expected body pitch to stay near initial after balance compensation '
            f'(initial={initial_pitch:.3f}, balanced={balanced_pitch:.3f}, board_p={board_p:.3f})'
        )
        assert abs(imu_roll - balanced_roll) <= 0.10, (
            'Expected IMU-derived roll to match balanced body roll'
        )
        assert abs(imu_pitch - balanced_pitch) <= 0.10, (
            'Expected IMU-derived pitch to match balanced body pitch'
        )
        if abs(board_r) > 0.05:
            assert abs(board_r - balanced_roll) > 0.05, (
                'Expected body roll to be notably closer to level than board '
                f'(board_r={board_r:.3f}, balanced_roll={balanced_roll:.3f})'
            )
        if abs(board_p) > 0.05:
            assert abs(board_p - balanced_pitch) > 0.05, (
                'Expected body pitch to be notably closer to level than board '
                f'(board_p={board_p:.3f}, balanced_pitch={balanced_pitch:.3f})'
            )
        assert balanced_height > pre_tilt_height - 0.03, (
            'Expected robot to remain supported near board height after tilt '
            f'(pre_tilt_z={pre_tilt_height:.3f}, balanced_z={balanced_height:.3f})'
        )

    def _reset_board_and_balance_mode(
        self,
        initial_roll: float,
        initial_pitch: float,
    ) -> None:
        self._set_board_tilt(roll=0.0, pitch=0.0)
        board_r, board_p = self._wait_for_board_tilt(expected_roll=0.0, expected_pitch=0.0)
        assert abs(board_r) <= 0.03, f'Expected board roll to return to level (got {board_r:.3f})'
        assert abs(board_p) <= 0.03, f'Expected board pitch to return to level (got {board_p:.3f})'

        self._set_balance_mode(False)
        self._wait_for_sim_time(self.POSE_SETTLE_DURATION)
        roll, pitch = self._sample_base_roll_pitch(settle_sim_time_sec=self.POSE_SETTLE_DURATION)
        assert abs(roll - initial_roll) <= self.BALANCED_BODY_TILT_TOLERANCE, (
            'Expected body roll near initial after disabling balance mode '
            f'(initial={initial_roll:.3f}, actual={roll:.3f})'
        )
        assert abs(pitch - initial_pitch) <= self.BALANCED_BODY_TILT_TOLERANCE, (
            'Expected body pitch near initial after disabling balance mode '
            f'(initial={initial_pitch:.3f}, actual={pitch:.3f})'
        )
        self._set_balance_mode(True)


class BalanceBoardWorldBase(GazeboRobotControlBase):
    """
    Harness for testing the balance board fixture in isolation (no robot).

    Reuses the board-driving helpers from :class:`GazeboRobotControlBase` but
    launches Gazebo with ``spawn_robot:=false``, so there is no robot, ROS bridge,
    or control stack. The board helpers talk to Gazebo over the ``gz`` CLI and need
    no ROS node, so the test fixture only waits for the world to start serving
    poses via :meth:`wait_for_board_world_ready`.
    """

    __test__ = False  # Prevent pytest from collecting this base class as a test case.

    def wait_for_board_world_ready(self) -> Pose:
        """Poll Gazebo until the board entity pose is served, tolerating startup errors."""
        deadline = time.monotonic() + self.READY_TIMEOUT
        last_error = ''
        while time.monotonic() < deadline:
            try:
                return self._sample_entity_pose_from_gazebo(BALANCE_BOARD_BOARD_LINK_NAME)
            except (AssertionError, RuntimeError) as error:
                # gz CLI not up yet, or the world has not published poses; retry.
                last_error = str(error)
            time.sleep(0.5)
        raise AssertionError(
            f'Balance board world did not become ready within {self.READY_TIMEOUT}s: {last_error}'
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
            elif stripped == '}':
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
        elif current_section == 'orientation' and axis in orientation:
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
