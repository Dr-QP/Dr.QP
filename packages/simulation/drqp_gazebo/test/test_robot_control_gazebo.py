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

import math
import time
import unittest

from controller_manager.test_utils import check_node_running
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Pose, Vector3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ProcInfoHandler
from launch_testing_ros import WaitForTopics
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
import std_msgs.msg
from test_utils import ensure_gz_sim_not_running


@pytest.mark.slow
@pytest.mark.launch_test
def generate_test_description():
    """Launch Gazebo simulation for testing."""
    ensure_gz_sim_not_running()

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
                }.items(),
            ),
            # Wait for simulation to fully initialize before starting tests
            TimerAction(period=10.0, actions=[ReadyToTest()]),
        ]
    )


@pytest.mark.slow
class TestGazeboRobotControl(unittest.TestCase):
    """Test robot control behaviors in Gazebo simulation."""

    # Configurable timeouts
    SPAWN_TIMEOUT = 30.0
    STATE_TRANSITION_TIMEOUT = 10.0
    MOVEMENT_TIMEOUT = 5.0
    CLOCK_TIMEOUT = 10.0
    MOVEMENT_DURATION = 5.0

    # Posture delta threshold (meters)
    MIN_ARM_DISARM_HEIGHT_DELTA = 0.02

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        """Set up test node and publishers/subscribers."""
        self.node = rclpy.create_node('test_gazebo_robot_control')

        # State tracking
        self.current_robot_state = None
        self.robot_pose = None

        # QoS profile for state subscriptions
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Subscribe to robot state
        self.state_sub = self.node.create_subscription(
            std_msgs.msg.String,
            '/robot_state',
            self._robot_state_callback,
            qos_profile=qos_profile,
        )

        # Subscribe to robot odometry from Gazebo bridge
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/model/drqp/odometry',
            self._odometry_callback,
            10,
        )

        # Publisher for robot events (to trigger state transitions)
        self.event_pub = self.node.create_publisher(std_msgs.msg.String, '/robot_event', 10)

        # Publisher for movement commands
        self.movement_pub = self.node.create_publisher(
            MovementCommand, '/robot/movement_command', 10
        )

        self._spin_until(
            lambda: self.event_pub.get_subscription_count() > 0,
            self.CLOCK_TIMEOUT,
            'No subscribers for /robot_event publisher',
        )

    def tearDown(self):
        """Clean up test node."""
        self.node.destroy_node()

    def _robot_state_callback(self, msg: std_msgs.msg.String):
        """Track current robot state."""
        self.current_robot_state = msg.data
        self.node.get_logger().info(f'Robot state updated: {self.current_robot_state}')

    def _odometry_callback(self, msg: Odometry):
        """Track robot pose from robot-scoped odometry."""
        self.robot_pose = msg.pose.pose

    def _spin_until(self, condition_fn, timeout_sec, error_message):
        """Spin the node until a condition is met or timeout occurs."""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition_fn():
                return True
            time.sleep(0.1)
        raise TimeoutError(error_message)

    def _publish_event(self, event_name: str):
        """Publish a robot event."""
        msg = std_msgs.msg.String()
        msg.data = event_name
        for _ in range(3):
            self.event_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.05)
        self.node.get_logger().info(f'Published event: {event_name}')
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def _wait_for_state(self, expected_state: str, timeout_sec: float):
        """Wait for robot to reach a specific state."""
        self._spin_until(
            lambda: self.current_robot_state == expected_state,
            timeout_sec,
            f'Robot did not reach state "{expected_state}" within {timeout_sec}s. '
            f'Current state: {self.current_robot_state}',
        )

    def _wait_for_any_state(self, expected_states: tuple[str, ...], timeout_sec: float):
        """Wait until robot state matches any one of the expected states."""
        self._spin_until(
            lambda: self.current_robot_state in expected_states,
            timeout_sec,
            f'Robot did not reach any expected state {expected_states} '
            f'within {timeout_sec}s. Current state: {self.current_robot_state}',
        )

    def _wait_for_pose(self):
        """Wait until robot pose is available from Gazebo odometry."""
        self._spin_until(
            lambda: self.robot_pose is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive /model/drqp/odometry from Gazebo bridge',
        )

    def _require_pose(self):
        """Require Gazebo pose bridge data for pose-dependent tests."""
        self._wait_for_pose()
        self.assertIsNotNone(
            self.robot_pose,
            'Gazebo pose data is required but /model/drqp/odometry did not provide a pose',
        )

    def _arm_robot(self):
        """Arm the robot and wait for torque_on state."""
        if self.current_robot_state == 'torque_on':
            return
        if self.current_robot_state is None:
            self._wait_for_any_state(
                ('torque_off', 'finalized', 'initializing', 'finalizing', 'torque_on'),
                self.STATE_TRANSITION_TIMEOUT,
            )

        initialize_error = None
        for _ in range(2):
            self._publish_event('initialize')
            try:
                self._wait_for_any_state(
                    ('initializing', 'torque_on'),
                    self.STATE_TRANSITION_TIMEOUT,
                )
                initialize_error = None
                break
            except TimeoutError as error:
                initialize_error = error

        if initialize_error is not None:
            raise initialize_error

        if self.current_robot_state != 'torque_on':
            self._publish_event('initializing_done')
            try:
                self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)
            except TimeoutError:
                self._publish_event('initializing_done')
                self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

    def _disarm_robot(self):
        """Disarm the robot and wait for finalized state."""
        if self.current_robot_state == 'finalized':
            return
        if self.current_robot_state != 'torque_on':
            self._arm_robot()

        finalize_error = None
        for _ in range(2):
            self._publish_event('finalize')
            try:
                self._wait_for_any_state(
                    ('finalizing', 'finalized'),
                    self.STATE_TRANSITION_TIMEOUT,
                )
                finalize_error = None
                break
            except TimeoutError as error:
                finalize_error = error

        if finalize_error is not None:
            raise finalize_error

        if self.current_robot_state != 'finalized':
            self._publish_event('finalizing_done')
            try:
                self._wait_for_state('finalized', self.STATE_TRANSITION_TIMEOUT)
            except TimeoutError:
                self._publish_event('finalizing_done')
                self._wait_for_state('finalized', self.STATE_TRANSITION_TIMEOUT)

    def _yaw_from_pose(self, pose: Pose) -> float:
        """Extract yaw angle from pose quaternion."""
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _run_movement_and_measure(
        self, stride_x: float = 0.0, stride_y: float = 0.0, rotation: float = 0.0
    ) -> tuple[float, float, float]:
        """Publish movement command and return body-frame deltas."""
        self._require_pose()
        start_pose = self.robot_pose
        if start_pose is None:
            raise RuntimeError('No starting pose available before movement command')

        start_pos = start_pose.position
        start_yaw = self._yaw_from_pose(start_pose)

        self._publish_movement_command(stride_x=stride_x, stride_y=stride_y, rotation=rotation)

        end_time = time.time() + self.MOVEMENT_DURATION
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        end_pose = self.robot_pose
        if end_pose is None:
            raise RuntimeError('Pose became unavailable after movement command')

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

    def _publish_movement_command(self, stride_x=0.0, stride_y=0.0, rotation=0.0):
        """Publish a movement command."""
        cmd = MovementCommand()
        cmd.stride_direction = Vector3(x=stride_x, y=stride_y, z=0.0)
        cmd.rotation_speed = rotation
        cmd.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
        cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD
        self.movement_pub.publish(cmd)
        self.node.get_logger().info(
            f'Published movement command: stride=({stride_x}, {stride_y}), rotation={rotation}'
        )

    def test_nodes_and_clock(self):
        """Verify simulation nodes are running and clock is available."""
        # Check required nodes are running
        for node_name in ('robot_state_publisher', 'drqp_brain', 'drqp_robot_state'):
            check_node_running(self.node, node_name, timeout=self.CLOCK_TIMEOUT)

        # Wait for clock to ensure Gazebo is running
        with WaitForTopics([('/clock', Clock)], timeout=self.CLOCK_TIMEOUT) as wait:
            self.assertTrue(wait.wait(), 'Did not receive /clock from Gazebo bridge')

    def test_robot_spawn(self):
        """Verify robot model spawns in Gazebo within timeout."""
        # Verify robot_state_publisher is running (indicates robot description is loaded)
        try:
            check_node_running(self.node, 'robot_state_publisher', timeout=self.SPAWN_TIMEOUT)
        except (AssertionError, RuntimeError, TimeoutError) as e:
            self.fail(
                f'Robot model failed to spawn: robot_state_publisher not running within '
                f'{self.SPAWN_TIMEOUT}s. Error: {e}'
            )

        # Spin to receive initial state (confirms robot state machine is active)
        start_time = time.time()
        while self.current_robot_state is None and time.time() - start_time < self.SPAWN_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertIsNotNone(
            self.current_robot_state,
            f'Robot failed to spawn: did not receive robot state within {self.SPAWN_TIMEOUT}s. '
            f'Check that Gazebo spawned the model "drqp" successfully.',
        )

    def test_arm_robot(self):
        """Test arming the robot and verify state transition."""
        try:
            if self.current_robot_state is None:
                rclpy.spin_once(self.node, timeout_sec=1.0)
            self._arm_robot()
        except TimeoutError as e:
            self.fail(
                f'Arm command failed: robot did not reach "torque_on" (armed) state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

    def test_verify_armed_posture(self):
        """Verify robot posture after arming (base off ground)."""
        self._require_pose()
        self._disarm_robot()
        time.sleep(1.0)
        self._require_pose()

        self.assertIsNotNone(self.robot_pose, 'Failed to read disarmed pose before arming')
        disarmed_base_z = self.robot_pose.position.z

        # Ensure robot is armed
        self._arm_robot()

        # Give robot time to reach standing posture
        time.sleep(3.0)

        # Wait for pose to verify robot is in stable position
        self._require_pose()

        # Verify base is elevated relative to disarmed posture
        self.assertIsNotNone(self.robot_pose, 'Failed to read pose after arming')
        armed_base_z = self.robot_pose.position.z
        delta_z = armed_base_z - disarmed_base_z
        self.node.get_logger().info(
            f'Armed base height: z={armed_base_z:.3f}m '
            f'(disarmed={disarmed_base_z:.3f}m, delta={delta_z:.3f}m)'
        )
        self.assertGreater(
            delta_z,
            self.MIN_ARM_DISARM_HEIGHT_DELTA,
            f'Base should rise when armed '
            f'(delta_z={delta_z:.3f}m <= {self.MIN_ARM_DISARM_HEIGHT_DELTA}m)',
        )

        # Verify robot is in armed state
        self.assertEqual(self.current_robot_state, 'torque_on')

    def test_movement_forward(self):
        """Test forward movement command and verify motion."""
        self._require_pose()
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=1.0)
            self.assertGreater(
                forward_delta,
                0.01,
                msg=f'Robot did not move forward significantly: '
                f'forward_delta={forward_delta:.3f}m (expected > 0.01m)',
            )
        except RuntimeError as e:
            self.fail(f'Forward movement test failed. Error: {e}')

    def test_movement_backward(self):
        """Test backward movement command and verify motion."""
        self._require_pose()
        self._arm_robot()
        try:
            forward_delta, _, _ = self._run_movement_and_measure(stride_x=-1.0)
            self.assertLess(
                forward_delta,
                -0.01,
                msg=f'Robot did not move backward significantly: '
                f'forward_delta={forward_delta:.3f}m (expected < -0.01m)',
            )
        except RuntimeError as e:
            self.fail(f'Backward movement test failed. Error: {e}')

    def test_movement_left(self):
        """Test left strafe movement command and verify motion."""
        self._require_pose()
        self._arm_robot()
        try:
            _, left_delta, _ = self._run_movement_and_measure(stride_y=1.0)
            self.assertGreater(
                left_delta,
                0.01,
                msg=f'Robot did not strafe left significantly: '
                f'left_delta={left_delta:.3f}m (expected > 0.01m)',
            )
        except RuntimeError as e:
            self.fail(f'Left strafe test failed. Error: {e}')

    def test_movement_right(self):
        """Test right strafe movement command and verify motion."""
        self._require_pose()
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
        except RuntimeError as e:
            self.fail(f'Right strafe test failed. Error: {e}')

    def test_movement_rotation(self):
        """Test rotation movement command and verify yaw rotation."""
        self._require_pose()
        self._arm_robot()
        try:
            _, _, delta_yaw = self._run_movement_and_measure(rotation=0.5)
            self.assertGreater(
                abs(delta_yaw),
                0.1,
                msg=f'Robot did not rotate significantly: |delta_yaw|={abs(delta_yaw):.3f}',
            )
        except RuntimeError as e:
            self.fail(f'Rotation movement test failed. Error: {e}')

    def test_disarm_robot(self):
        """Test disarming the robot and verify state transition."""
        try:
            self._disarm_robot()
        except TimeoutError as e:
            self.fail(
                f'Disarm command failed: robot did not reach "finalized" (disarmed) state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

    def test_verify_disarmed_posture(self):
        """Verify robot posture after disarming (base on ground)."""
        self._require_pose()
        self._arm_robot()
        time.sleep(1.0)
        self._require_pose()

        self.assertIsNotNone(self.robot_pose, 'Failed to read armed pose before disarming')
        armed_base_z = self.robot_pose.position.z

        self._disarm_robot()

        # Give robot time to settle to ground
        time.sleep(3.0)

        # Wait for pose
        self._require_pose()

        # Verify base goes down relative to armed posture
        if self.robot_pose is not None:
            disarmed_base_z = self.robot_pose.position.z
            delta_z = armed_base_z - disarmed_base_z
            self.node.get_logger().info(
                f'Disarmed base height: z={disarmed_base_z:.3f}m '
                f'(armed={armed_base_z:.3f}m, delta={delta_z:.3f}m)'
            )
            self.assertLess(
                disarmed_base_z,
                armed_base_z - self.MIN_ARM_DISARM_HEIGHT_DELTA,
                msg=(
                    f'Base should lower when disarmed '
                    f'(armed={armed_base_z:.3f}m, disarmed={disarmed_base_z:.3f}m, '
                    f'min_delta={self.MIN_ARM_DISARM_HEIGHT_DELTA}m)'
                ),
            )
        else:
            self.fail('Failed to read pose after disarm')

        # Verify robot completed disarm cycle
        self.assertEqual(
            self.current_robot_state,
            'finalized',
            msg=f'Robot did not reach finalized state after disarm. '
            f'Current state: {self.current_robot_state}',
        )


@post_shutdown_test()
class TestGazeboShutdown(unittest.TestCase):
    """Verify processes exit cleanly after the launch test finishes."""

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()
        ensure_gz_sim_not_running()

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally (except Gazebo)."""
        # Gazebo is SIGTERMed by the launch file, so we need to filter it out
        proc_info = self._filter_out_gazebo(proc_info)
        from launch_testing import asserts

        asserts.assertExitCodes(proc_info)

    def _filter_out_gazebo(self, proc_info):
        """Filter out Gazebo processes from the list."""
        filtered_proc_info = ProcInfoHandler()
        skipped_procs = ('gazebo', 'gz', 'bridge_node')
        for proc_name in proc_info.process_names():
            if not any(skip in proc_name for skip in skipped_procs):
                filtered_proc_info.append(proc_info[proc_name])
        return filtered_proc_info
