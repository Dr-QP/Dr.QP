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

from unittest import mock

from control_msgs.action import FollowJointTrajectory
from drqp_brain.balance_controller import BASE_CENTER_TO_IMU_ROTATION
from drqp_brain.brain_node import _assert_no_existing_brain_node, HexapodBrain
from drqp_brain.instance_guard import InstanceAlreadyRunningError
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
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
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
import std_msgs.msg


def test_existing_brain_node_detection_rejects_duplicate_ros_node():
    """Refuse startup when the ROS graph already contains drqp_brain."""
    node = mock.Mock()
    node.get_node_names_and_namespaces.return_value = [
        ('robot_state_publisher', '/'),
        ('drqp_brain', '/'),
    ]

    with pytest.raises(InstanceAlreadyRunningError, match='/drqp_brain'):
        _assert_no_existing_brain_node(node)


@pytest.fixture
def rclpy_context():
    """Initialize rclpy for unit tests that construct a HexapodBrain directly."""
    rclpy.init()
    yield
    rclpy.try_shutdown()


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


def test_process_imu_compensates_static_mount_rotation(rclpy_context):  # noqa: ARG001 (needs rclpy)
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
        brain.process_imu(make_imu_msg_from_base_tilt(0.05, -0.04, 0.2))

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

        brain.process_balance_mode(std_msgs.msg.Bool(data=False))

        assert brain.balance_mode_enabled is False
        assert brain.target_body_tilt is None
        assert brain.get_imu_body_tilt() is None
    finally:
        brain.destroy_node()


def test_loop_uses_imu_balance_correction(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Apply IMU roll and pitch compensation relative to the current measured tilt."""
    with mock.patch('drqp_brain.brain_node.JointTrajectoryBuilder') as trajectory_builder_cls:
        brain = HexapodBrain()
        try:
            brain.walker.next_step_targets = mock.Mock(return_value=[])
            brain._ik_ready = mock.Mock(return_value=False)
            brain.current_movement.stride_direction = Vector3(x=0.0, y=0.0, z=0.0)
            brain.current_movement.rotation_speed = 0.0
            brain.current_movement.body_translation = Vector3(x=0.0, y=0.0, z=0.0)
            brain.current_movement.body_rotation = Vector3(x=0.0, y=0.0, z=0.4)
            brain.current_movement.gait_type = MovementCommandConstants.GAIT_TRIPOD
            brain.process_imu(make_imu_msg_from_base_tilt(0.0, 0.0, 0.35))
            brain.process_balance_mode(std_msgs.msg.Bool(data=True))
            brain.process_imu(make_imu_msg_from_base_tilt(0.12, -0.08, 0.35))

            brain.loop()

            body_rotation = brain.walker.next_step_targets.call_args.kwargs['body_rotation']
            expected_rotation = R.from_euler(
                'xyz', [-0.12, 0.08, 0.0], degrees=False
            ) * R.from_rotvec([0.0, 0.0, 0.4])
            assert R.from_rotvec(body_rotation.numpy()).as_matrix() == pytest.approx(
                expected_rotation.as_matrix()
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
