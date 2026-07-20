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

"""Launch and unit tests for the drqp_brain node."""

import math
from unittest import mock

from control_msgs.action import FollowJointTrajectory
from drqp_brain.balance_controller import BASE_CENTER_TO_IMU_ROTATION
from drqp_brain.brain_node import _assert_no_existing_brain_node, HexapodBrain
from drqp_brain.instance_guard import InstanceAlreadyRunningError
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from drqp_kinematics.geometry import Point3D
from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes
from geometry_msgs.msg import Quaternion, Vector3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
import pytest
import rclpy
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
import std_msgs.msg


@pytest.mark.parametrize(
    ('sequence_name', 'expected_points'),
    [
        (
            'initialization_sequence',
            [
                (0.0, -2.0614083795305027, -0.5742133239061344),
                (0.0, -2.0614083795305027, -0.5742133239061344),
                (0.0, -2.0614083795305027, -0.5742133239061344),
                (0.0, -2.0614083795305027, 1.0838494654884787),
                (0.0, -0.8396779031344719, 1.6947147036864938),
            ],
        ),
        (
            'finalization_sequence',
            [
                (0.0, -2.0614083795305027, -0.5742133239061344),
                (0.0, -2.0614083795305027, -1.6214108751027323),
            ],
        ),
    ],
)
def test_model_pose_sequences_preserve_golden_controller_positions(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
    sequence_name,
    expected_points,
):
    """Initialization and finalization retain their exact controller positions."""
    with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
        action_client = action_client_cls.return_value
        action_client.wait_for_server.return_value = True

        brain = HexapodBrain()
        try:
            getattr(brain, sequence_name)()
            goal = action_client.send_goal_async.call_args.args[0]

            assert [
                tuple(point.positions[:3]) for point in goal.trajectory.points
            ] == expected_points
            assert all(
                tuple(point.positions[slice(index, index + 3)]) == expected_point
                for point, expected_point in zip(goal.trajectory.points, expected_points)
                for index in range(0, 18, 3)
            )
        finally:
            brain.destroy_node()


def test_existing_brain_node_detection_rejects_duplicate_ros_node():
    """Refuse startup when the ROS graph already contains drqp_brain."""
    node = mock.Mock()
    node.get_node_names_and_namespaces.return_value = [
        ('robot_state_publisher', '/'),
        ('drqp_brain', '/'),
    ]

    with pytest.raises(InstanceAlreadyRunningError, match='/drqp_brain'):
        _assert_no_existing_brain_node(node)


def test_trajectory_action_client_is_created_lazily(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Only create the action client when an action sequence is requested."""
    with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
        action_client = action_client_cls.return_value
        action_client.send_goal_async.return_value.add_done_callback = mock.Mock()

        brain = HexapodBrain()
        try:
            action_client_cls.assert_not_called()

            brain.reboot_servos()

            action_client_cls.assert_called_once_with(
                brain,
                FollowJointTrajectory,
                '/joint_trajectory_controller/follow_joint_trajectory',
            )
        finally:
            brain.destroy_node()


def test_destroy_node_destroys_action_client(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Destroy the action client explicitly during node shutdown."""
    with mock.patch('drqp_brain.brain_node.ActionClient') as action_client_cls:
        action_client = action_client_cls.return_value
        action_client.send_goal_async.return_value.add_done_callback = mock.Mock()

        brain = HexapodBrain()
        brain.reboot_servos()
        brain.destroy_node()

        action_client.destroy.assert_called_once_with()


def test_omega_max_parameter_replaces_legacy_rotation_speed(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """A supplied metric yaw-rate limit reaches the walk controller unchanged."""
    brain = HexapodBrain(
        parameter_overrides=[Parameter('omega_max_rad_sec', Parameter.Type.DOUBLE, 1.75)]
    )
    try:
        assert brain.walker.omega_max_rad_sec == pytest.approx(1.75)
    finally:
        brain.destroy_node()


def test_legacy_rotation_speed_override_warns_with_omega_max_override(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """Warn whenever the deprecated rotation-speed parameter is configured."""
    logger = mock.Mock()
    with mock.patch.object(HexapodBrain, 'get_logger', return_value=logger):
        brain = HexapodBrain(
            parameter_overrides=[
                Parameter('omega_max_rad_sec', Parameter.Type.DOUBLE, 1.75),
                Parameter('rotation_speed_degrees', Parameter.Type.DOUBLE, 30.0),
            ]
        )
    try:
        logger.warning.assert_any_call(
            'rotation_speed_degrees is deprecated; configure omega_max_rad_sec instead'
        )
    finally:
        brain.destroy_node()


def test_zero_omega_max_parameter_falls_back_to_legacy_rotation_speed(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """A zero omega override preserves the legacy yaw rate."""
    brain = HexapodBrain(
        parameter_overrides=[Parameter('omega_max_rad_sec', Parameter.Type.DOUBLE, 0.0)]
    )
    try:
        expected_omega_max = (
            math.radians(brain.rotation_speed_degrees) / brain.walker.stance_duration_sec
        )

        assert brain.walker.omega_max_rad_sec == pytest.approx(expected_omega_max)
    finally:
        brain.destroy_node()


def make_imu_msg_from_base_tilt(
    roll: float,
    pitch: float,
    yaw: float = 0.0,
    *,
    frame_id: str = 'drqp/imu_link',
) -> Imu:
    """Build a raw IMU message whose sensor-frame orientation maps to the given body tilt."""
    body_in_world = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    qx, qy, qz, qw = (body_in_world * BASE_CENTER_TO_IMU_ROTATION).as_quat()
    return Imu(
        header=std_msgs.msg.Header(frame_id=frame_id),
        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        orientation_covariance=[0.0] * 9,
    )


def make_movement_command() -> MovementCommand:
    """Build a command with every operator-controlled motion component active."""
    return MovementCommand(
        stride_direction=Vector3(x=0.7, y=-0.4, z=0.2),
        rotation_speed=0.6,
        body_translation=Vector3(x=0.03, y=-0.02, z=0.01),
        body_rotation=Vector3(x=0.1, y=-0.2, z=0.3),
        gait_type=MovementCommandConstants.GAIT_RIPPLE,
    )


def arm_brain_with_fresh_imu(brain: HexapodBrain) -> None:
    """Put a directly constructed brain in the balance activation precondition."""
    brain.robot_state = 'torque_on'
    brain.process_imu(make_imu_msg_from_base_tilt(0.05, -0.04, 0.2))


def test_process_imu_compensates_mount_rotation(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Recover body attitude by de-rotating the fixed IMU sensor mount."""
    brain = HexapodBrain()
    try:
        brain.process_imu(make_imu_msg_from_base_tilt(0.11, -0.07, 0.2))

        assert brain.current_body_tilt.x == pytest.approx(0.11)
        assert brain.current_body_tilt.y == pytest.approx(-0.07)
    finally:
        brain.destroy_node()


def test_process_imu_does_not_require_known_tf_frame(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Gazebo IMU frames are accepted because mount compensation uses a static rotation."""
    brain = HexapodBrain()
    try:
        brain.process_imu(
            make_imu_msg_from_base_tilt(0.03, -0.02, 0.1, frame_id='drqp/ground/imu_sensor')
        )

        assert brain.current_body_tilt.x == pytest.approx(0.03)
        assert brain.current_body_tilt.y == pytest.approx(-0.02)
    finally:
        brain.destroy_node()


def test_process_imu_rejects_unavailable_orientation(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Ignore IMU messages that mark orientation unavailable."""
    brain = HexapodBrain()
    try:
        msg = make_imu_msg_from_base_tilt(0.03, -0.02, 0.1)
        msg.orientation_covariance[0] = -1.0

        brain.process_imu(msg)

        assert brain.current_body_tilt is None
        assert brain.last_imu_update is None
    finally:
        brain.destroy_node()


def test_balance_mode_captures_target_orientation_until_disabled_issue356(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """Issue 356: keep the toggle-captured target tilt until balance mode is disabled."""
    brain = HexapodBrain()
    try:
        arm_brain_with_fresh_imu(brain)

        assert brain.balance_mode_enabled is False
        assert brain.target_body_tilt is None
        assert brain.get_imu_body_tilt() is None

        brain.process_balance_mode(std_msgs.msg.Bool(data=True))

        assert brain.balance_mode_enabled is True
        assert brain.target_body_tilt.x == pytest.approx(0.05)
        assert brain.target_body_tilt.y == pytest.approx(-0.04)

        brain.process_imu(make_imu_msg_from_base_tilt(0.09, -0.02, 0.2))

        assert brain.target_body_tilt.x == pytest.approx(0.05)
        assert brain.target_body_tilt.y == pytest.approx(-0.04)
        assert brain.get_imu_body_tilt().x == pytest.approx(0.09)
        assert brain.get_imu_body_tilt().y == pytest.approx(-0.02)

        brain.process_balance_mode(std_msgs.msg.Bool(data=True))

        assert brain.target_body_tilt.x == pytest.approx(0.05)
        assert brain.target_body_tilt.y == pytest.approx(-0.04)

        brain.process_balance_mode(std_msgs.msg.Bool(data=False))

        assert brain.balance_mode_enabled is False
        assert brain.target_body_tilt is None
        assert brain.get_imu_body_tilt() is None
    finally:
        brain.destroy_node()


def test_entering_balance_mode_zeros_semantic_motion_and_stops_gait(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """Stationary posture mode discards the active command and gait progress."""
    brain = HexapodBrain()
    try:
        brain.process_movement_command(make_movement_command())
        brain.walker.current_phase = 0.42
        arm_brain_with_fresh_imu(brain)

        brain.process_balance_mode(std_msgs.msg.Bool(data=True))

        assert brain.balance_mode_enabled
        assert brain.current_movement.stride_direction == Vector3()
        assert brain.current_movement.rotation_speed == 0.0
        assert brain.current_movement.body_translation == Vector3()
        assert brain.current_movement.body_rotation == Vector3()
        assert brain.walker.current_phase == 0.0
        assert brain.walker.current_direction.numpy() == pytest.approx([0.0, 0.0, 0.0])
        assert brain.walker.current_rotation_direction == 0.0
    finally:
        brain.destroy_node()


def test_balance_mode_ignores_concurrent_movement_commands(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """A held joystick cannot re-arm gait or user body motion while balancing."""
    brain = HexapodBrain()
    try:
        arm_brain_with_fresh_imu(brain)
        brain.process_balance_mode(std_msgs.msg.Bool(data=True))

        brain.process_movement_command(make_movement_command())

        assert brain.current_movement.stride_direction == Vector3()
        assert brain.current_movement.rotation_speed == 0.0
        assert brain.current_movement.body_translation == Vector3()
        assert brain.current_movement.body_rotation == Vector3()
        assert brain.gait_index == 0
    finally:
        brain.destroy_node()


def test_disabling_balance_requires_a_fresh_command_to_resume_motion(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """Neither the pre-balance command nor an ignored command is replayed."""
    brain = HexapodBrain()
    try:
        brain.process_movement_command(make_movement_command())
        arm_brain_with_fresh_imu(brain)
        brain.process_balance_mode(std_msgs.msg.Bool(data=True))
        brain.process_movement_command(make_movement_command())

        brain.process_balance_mode(std_msgs.msg.Bool(data=False))

        assert brain.current_movement.stride_direction == Vector3()
        assert brain.current_movement.rotation_speed == 0.0
        assert brain.walker.current_phase == 0.0

        fresh_command = make_movement_command()
        brain.process_movement_command(fresh_command)
        assert brain.current_movement is fresh_command
    finally:
        brain.destroy_node()


@pytest.mark.parametrize('robot_state', [None, 'torque_off', 'finalized'])
def test_balance_mode_requires_armed_robot_and_fresh_imu(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
    robot_state,
):
    """Reject balance activation outside the armed, fresh-IMU safety envelope."""
    brain = HexapodBrain()
    try:
        brain.robot_state = robot_state
        brain.process_imu(make_imu_msg_from_base_tilt(0.05, -0.04, 0.2))
        if robot_state == 'finalized':
            brain.last_imu_update = None

        brain.process_balance_mode(std_msgs.msg.Bool(data=True))

        assert not brain.balance_mode_enabled
        assert brain.target_body_tilt is None
    finally:
        brain.destroy_node()


def test_stale_imu_disables_balance_without_replaying_motion(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """A stale sensor exits posture hold into a stationary fail-safe state."""
    brain = HexapodBrain()
    try:
        arm_brain_with_fresh_imu(brain)
        brain.process_balance_mode(std_msgs.msg.Bool(data=True))
        brain.process_movement_command(make_movement_command())
        brain.last_imu_update = None
        brain._ik_ready = mock.Mock(return_value=False)

        brain.loop()

        assert not brain.balance_mode_enabled
        assert brain.current_movement.stride_direction == Vector3()
        assert brain.current_movement.rotation_speed == 0.0
        assert brain.walker.current_phase == 0.0
    finally:
        brain.destroy_node()


def test_balance_correction_is_scaled_by_six_leg_unclamped_reachability(
    rclpy_context,  # noqa: ARG001 (needs rclpy)
):
    """Bound the complete posture correction before the runtime solve can clamp."""
    brain = HexapodBrain()
    try:
        targets = [(leg, leg.tibia_end.copy()) for leg in brain.hexapod.legs]
        reachability = mock.Mock()

        def constrained_legs(legs_and_targets, body_transform):
            assert len(legs_and_targets) == 6
            correction_angle = R.from_matrix(body_transform.rotation).magnitude()
            return ('left_front', 'right_back') if correction_angle > 0.08 else ()

        reachability.unreachable_legs.side_effect = constrained_legs
        brain.balance_reachability = reachability
        logger = mock.Mock()
        brain.get_logger = mock.Mock(return_value=logger)

        bounded_rotation = brain._constrain_balance_correction(
            Point3D([0.20, 0.0, 0.0]),
            targets,
        )

        bounded_magnitude = math.sqrt(sum(value**2 for value in bounded_rotation.numpy()))
        assert bounded_magnitude == pytest.approx(0.08, abs=1e-4)
        assert reachability.unreachable_legs.call_count > 2
        logger.warning.assert_called_once_with(
            'stationary_balance_correction_saturated:left_front,right_back scale=0.400',
            throttle_duration_sec=5.0,
        )
    finally:
        brain.destroy_node()


def test_loop_uses_imu_balance_correction(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Apply only stationary roll/pitch compensation while balance mode is active."""
    with mock.patch('drqp_brain.brain_node.JointTrajectoryBuilder') as trajectory_builder_cls:
        brain = HexapodBrain()
        try:
            # Neutralize the tuning parameters so this test only exercises the
            # target-relative tilt error computation, not the gain/clamp math
            # covered separately in test_balance_controller.py.
            brain.imu_balance_gain = 1.0
            brain.imu_balance_max_tilt_rad = 1.0
            brain.walker.targets_at = mock.Mock(return_value=[])
            brain._ik_ready = mock.Mock(return_value=False)
            brain.current_movement.stride_direction = Vector3(x=0.0, y=0.0, z=0.0)
            brain.current_movement.rotation_speed = 0.0
            brain.current_movement.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
            brain.current_movement.body_rotation = Vector3(x=0.0, y=0.0, z=0.4)
            brain.current_movement.gait_type = MovementCommandConstants.GAIT_TRIPOD
            brain.robot_state = 'torque_on'
            brain.process_imu(make_imu_msg_from_base_tilt(0.0, 0.0, 0.35))
            brain.process_balance_mode(std_msgs.msg.Bool(data=True))
            brain.process_imu(make_imu_msg_from_base_tilt(0.12, -0.08, 0.35))

            brain.loop()

            expected_rotation = R.from_euler('xyz', [-0.12, 0.08, 0.0], degrees=False)
            assert brain.hexapod.body_transform.rotation == pytest.approx(
                expected_rotation.as_matrix(), abs=1e-8
            )
            trajectory_builder_cls.assert_not_called()
        finally:
            brain.destroy_node()


@launch_pytest.fixture
def generate_test_description():
    """Launch the drqp_brain node and record process exit codes."""
    launch_description = LaunchDescription(
        [
            Node(
                executable=FindExecutable(name='python3'),
                arguments=[
                    '-m',
                    'coverage',
                    'run',
                    ExecutableInPackage(package='drqp_brain', executable='drqp_brain'),
                ],
                output='screen',
            ),
            # Launch tests 3s later
            TimerAction(period=3.0, actions=[ReadyToTest()]),
        ]
    )
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info


@pytest.fixture
def consumer(generate_test_description):  # noqa: ARG001 (drives the launch)
    """Own rclpy init/shutdown and provide a consumer node for the launched node."""
    rclpy.init()
    node = rclpy.create_node('test_brain_consumer')
    yield node
    rclpy.try_shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_movement_command_processing(consumer, generate_test_description):
    """Process a movement command, then verify the launched node exits cleanly."""
    cmd = MovementCommand()
    cmd.stride_direction = Vector3(x=1.0, y=0.0, z=0.0)
    cmd.rotation_speed = 0.5
    cmd.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
    cmd.body_rotation = Vector3(x=0.0, y=0.0, z=0.0)
    cmd.gait_type = MovementCommandConstants.GAIT_TRIPOD

    movement_pub = consumer.create_publisher(MovementCommand, '/robot/movement_command', 10)
    movement_pub.publish(cmd)

    # Spin to allow processing; if we get here without errors the brain node
    # processed the command without crashing.
    rclpy.spin_once(consumer, timeout_sec=0.1)

    # Function-scoped generator: the launched node is torn down at the yield,
    # then the post-yield body verifies it exited cleanly.
    yield
    _launch_description, proc_info = generate_test_description
    assert_processes_exited_cleanly(proc_info)
