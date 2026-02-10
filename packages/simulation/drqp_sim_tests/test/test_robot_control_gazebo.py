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
from geometry_msgs.msg import Vector3
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
                launch_arguments={'gui': 'false'}.items(),
            ),
            # Wait for simulation to fully initialize before starting tests
            TimerAction(period=10.0, actions=[ReadyToTest()]),
        ]
    )


class TestGazeboRobotControl(unittest.TestCase):
    """Test robot control behaviors in Gazebo simulation."""

    # Configurable timeouts
    SPAWN_TIMEOUT = 30.0
    STATE_TRANSITION_TIMEOUT = 20.0
    MOVEMENT_TIMEOUT = 15.0
    CLOCK_TIMEOUT = 20.0

    # Model and link names
    ROBOT_MODEL_NAME = 'drqp'
    BASE_LINK_NAME = 'drqp/base_link'
    FOOT_LINKS = [
        'drqp/left_front_foot_link',
        'drqp/right_front_foot_link',
        'drqp/left_middle_foot_link',
        'drqp/right_middle_foot_link',
        'drqp/left_back_foot_link',
        'drqp/right_back_foot_link',
    ]

    # Ground clearance thresholds (meters)
    BASE_ON_GROUND_THRESHOLD = 0.015  # Base should be below this when on ground
    BASE_OFF_GROUND_THRESHOLD = 0.05  # Base should be above this when armed
    FOOT_ON_GROUND_THRESHOLD = 0.01  # Feet should be below this when on ground
    FOOT_OFF_GROUND_THRESHOLD = 0.02  # Feet should be above this when off ground

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
        self.robot_odometry = None
        self.initial_pose = None

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

        # Subscribe to robot odometry from Gazebo
        self.odom_sub = self.node.create_subscription(
            Odometry, '/model/drqp/odometry', self._odometry_callback, 10
        )

        # Publisher for robot events (to trigger state transitions)
        self.event_pub = self.node.create_publisher(std_msgs.msg.String, '/robot_event', 10)

        # Publisher for movement commands
        self.movement_pub = self.node.create_publisher(
            MovementCommand, '/robot/movement_command', 10
        )

    def tearDown(self):
        """Clean up test node."""
        self.node.destroy_node()

    def _robot_state_callback(self, msg: std_msgs.msg.String):
        """Track current robot state."""
        self.current_robot_state = msg.data
        self.node.get_logger().info(f'Robot state updated: {self.current_robot_state}')

    def _odometry_callback(self, msg: Odometry):
        """Track robot odometry from Gazebo."""
        self.robot_odometry = msg
        # Store initial pose for movement delta calculations
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

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
        self.event_pub.publish(msg)
        self.node.get_logger().info(f'Published event: {event_name}')
        # Give time for message to be sent
        time.sleep(0.1)
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def _wait_for_state(self, expected_state: str, timeout_sec: float):
        """Wait for robot to reach a specific state."""
        self._spin_until(
            lambda: self.current_robot_state == expected_state,
            timeout_sec,
            f'Robot did not reach state "{expected_state}" within {timeout_sec}s. '
            f'Current state: {self.current_robot_state}',
        )

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
        except Exception as e:
            self.fail(
                f'Robot model failed to spawn: robot_state_publisher not running within '
                f'{self.SPAWN_TIMEOUT}s. Error: {e}'
            )
            raise

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
        # Ensure we have initial state
        if self.current_robot_state is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if self.current_robot_state is None:
                self.fail('Cannot arm robot: no initial state received')

        initial_state = self.current_robot_state
        self.node.get_logger().info(f'Initial state before arming: {initial_state}')

        # Send initialize event to arm the robot
        self._publish_event('initialize')

        # Wait for initializing state (intermediate)
        try:
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
        except TimeoutError as e:
            self.fail(
                f'Arm command failed: robot did not transition to "initializing" state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

        # Send initializing_done to complete arming
        self._publish_event('initializing_done')

        # Wait for torque_on (armed) state
        try:
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)
        except TimeoutError as e:
            self.fail(
                f'Arm command failed: robot did not reach "torque_on" (armed) state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

    def test_verify_armed_posture(self):
        """Verify robot posture after arming (base off ground, feet on ground)."""
        # Ensure robot is armed first
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Give robot time to reach standing posture
        time.sleep(3.0)

        # Wait for odometry to verify robot is in stable position
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )

        # Verify base is elevated (z position should be > threshold)
        if self.robot_odometry is not None:
            base_z = self.robot_odometry.pose.pose.position.z
            self.node.get_logger().info(f'Armed base height: z={base_z:.3f}m')
            self.assertGreater(
                base_z,
                self.BASE_OFF_GROUND_THRESHOLD,
                f'Base should be elevated when armed '
                f'(z={base_z:.3f}m < {self.BASE_OFF_GROUND_THRESHOLD}m)',
            )

        # Verify robot is in armed state
        self.assertEqual(self.current_robot_state, 'torque_on')

    def test_movement_forward(self):
        """Test forward movement command and verify motion."""
        # Ensure robot is armed
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Wait for odometry and store starting position
        try:
            self._spin_until(
                lambda: self.robot_odometry is not None,
                self.MOVEMENT_TIMEOUT,
                'Did not receive odometry from Gazebo',
            )
        except TimeoutError as e:
            self.fail(
                f'Forward movement test failed: no odometry received from Gazebo. '
                f'Check bridge configuration. Error: {e}'
            )

        start_x = self.robot_odometry.pose.pose.position.x

        # Publish forward movement command
        self._publish_movement_command(stride_x=1.0, stride_y=0.0, rotation=0.0)

        # Allow time for movement
        time.sleep(5.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify forward motion occurred (x position should have changed)
        if self.robot_odometry is not None:
            end_x = self.robot_odometry.pose.pose.position.x
            delta_x = end_x - start_x
            self.node.get_logger().info(
                f'Forward movement: start_x={start_x:.3f}, end_x={end_x:.3f}, delta={delta_x:.3f}'
            )
            # Assert that forward motion occurred (x increased)
            self.assertGreater(
                delta_x,
                0.01,
                msg=f'Robot did not move forward significantly: '
                f'delta_x={delta_x:.3f}m (expected > 0.01m)',
            )
        else:
            self.fail('Forward movement test failed: lost odometry after movement command')

    def test_movement_backward(self):
        """Test backward movement command and verify motion."""
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Wait for odometry and store starting position
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )
        start_x = self.robot_odometry.pose.pose.position.x

        self._publish_movement_command(stride_x=-1.0, stride_y=0.0, rotation=0.0)
        time.sleep(5.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify backward motion occurred (x decreased)
        if self.robot_odometry is not None:
            end_x = self.robot_odometry.pose.pose.position.x
            delta_x = end_x - start_x
            self.node.get_logger().info(
                f'Backward movement: start_x={start_x:.3f}, end_x={end_x:.3f}, delta={delta_x:.3f}'
            )
            # Assert that backward motion occurred (x decreased)
            self.assertLess(
                end_x,
                start_x - 0.01,
                msg=f'Robot did not move backward significantly: '
                f'start_x={start_x:.3f}, end_x={end_x:.3f} '
                f'(expected end_x < start_x - 0.01m)',
            )
        else:
            self.fail('Backward movement test failed: lost odometry after movement command')

    def test_movement_left(self):
        """Test left strafe movement command and verify motion."""
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Wait for odometry and store starting position
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )
        start_y = self.robot_odometry.pose.pose.position.y

        self._publish_movement_command(stride_x=0.0, stride_y=1.0, rotation=0.0)
        time.sleep(5.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Verify left strafe motion occurred (Y position should have changed)
        if self.robot_odometry is not None:
            end_y = self.robot_odometry.pose.pose.position.y
            delta_y = end_y - start_y
            self.node.get_logger().info(
                f'Left movement: start_y={start_y:.3f}, end_y={end_y:.3f}, delta={delta_y:.3f}'
            )
            # Assert that left strafe motion occurred (Y changed)
            self.assertGreater(
                abs(delta_y),
                0.01,
                msg=f'Robot did not strafe left significantly: '
                f'|delta_y|={abs(delta_y):.3f}m (expected > 0.01m)',
            )
        else:
            self.fail('Left strafe test failed: lost odometry after movement command')

    def test_movement_right(self):
        """Test right strafe movement command and verify motion."""
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Wait for odometry and store starting position
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )
        start_y = self.robot_odometry.pose.pose.position.y

        self._publish_movement_command(stride_x=0.0, stride_y=-1.0, rotation=0.0)
        time.sleep(5.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Log movement and verify that some motion occurred along Y
        if self.robot_odometry is not None:
            end_y = self.robot_odometry.pose.pose.position.y
            self.node.get_logger().info(
                f'Right movement: start_y={start_y:.3f}, end_y={end_y:.3f}'
            )
            # Require a minimal change in Y to consider the movement successful
            self.assertGreater(
                abs(end_y - start_y),
                0.01,
                msg='Robot did not move significantly in Y during right strafe command',
            )
        else:
            self.fail('Odometry became unavailable during right strafe movement test')

    def test_movement_rotation(self):
        """Test rotation movement command and verify rotation."""
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

        # Wait for odometry and store starting orientation
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )

        # Get starting orientation (yaw from quaternion)
        if self.robot_odometry is not None:
            q = self.robot_odometry.pose.pose.orientation
            start_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        else:
            self.fail('Failed to read initial odometry before rotation command')

        self._publish_movement_command(stride_x=0.0, stride_y=0.0, rotation=0.5)
        time.sleep(5.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        # Log rotation and verify that a significant rotation occurred
        if self.robot_odometry is not None:
            q = self.robot_odometry.pose.pose.orientation
            end_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.node.get_logger().info(
                f'Rotation: start_yaw={start_yaw:.3f}, end_yaw={end_yaw:.3f}'
            )

            # Compute smallest angular difference, handling wraparound at ±pi
            delta_yaw = ((end_yaw - start_yaw + math.pi) % (2.0 * math.pi)) - math.pi
            self.assertGreater(
                abs(delta_yaw),
                0.1,
                msg=(
                    f'Robot did not rotate significantly: '
                    f'start_yaw={start_yaw:.3f}, end_yaw={end_yaw:.3f}, '
                    f'|delta_yaw|={abs(delta_yaw):.3f}'
                ),
            )
        else:
            self.fail('Failed to read final odometry after rotation command')

    def test_disarm_robot(self):
        """Test disarming the robot and verify state transition."""
        # Ensure robot is armed first
        if self.current_robot_state != 'torque_on':
            self._publish_event('initialize')
            try:
                self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            except TimeoutError:
                self.fail('Failed to arm robot before disarm test')
            self._publish_event('initializing_done')
            try:
                self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)
            except TimeoutError:
                self.fail('Failed to reach armed state before disarm test')

        # Send finalize event to disarm
        self._publish_event('finalize')

        # Wait for finalizing state
        try:
            self._wait_for_state('finalizing', self.STATE_TRANSITION_TIMEOUT)
        except TimeoutError as e:
            self.fail(
                f'Disarm command failed: robot did not transition to "finalizing" state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

        # Send finalizing_done
        self._publish_event('finalizing_done')

        # Wait for finalized (disarmed) state
        try:
            self._wait_for_state('finalized', self.STATE_TRANSITION_TIMEOUT)
        except TimeoutError as e:
            self.fail(
                f'Disarm command failed: robot did not reach "finalized" (disarmed) state. '
                f'Current state: {self.current_robot_state}. Error: {e}'
            )

    def test_verify_disarmed_posture(self):
        """Verify robot completed disarm state transition successfully."""
        # Ensure robot goes through full cycle
        if self.current_robot_state != 'finalized':
            # Arm
            self._publish_event('initialize')
            self._wait_for_state('initializing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('initializing_done')
            self._wait_for_state('torque_on', self.STATE_TRANSITION_TIMEOUT)

            # Disarm
            self._publish_event('finalize')
            self._wait_for_state('finalizing', self.STATE_TRANSITION_TIMEOUT)
            self._publish_event('finalizing_done')
            self._wait_for_state('finalized', self.STATE_TRANSITION_TIMEOUT)

        # Give robot time to settle to ground
        time.sleep(3.0)

        # Wait for odometry
        self._spin_until(
            lambda: self.robot_odometry is not None,
            self.MOVEMENT_TIMEOUT,
            'Did not receive odometry from Gazebo',
        )

        # Log base height after disarm
        if self.robot_odometry is not None:
            base_z = self.robot_odometry.pose.pose.position.z
            self.node.get_logger().info(f'Disarmed base height: z={base_z:.3f}m')
        else:
            self.fail('Failed to read odometry after disarm')

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
