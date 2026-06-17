# Copyright (c) 2026 Anton Matosov
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

from control_msgs.action import FollowJointTrajectory
from controller_manager.test_utils import check_controllers_running, check_node_running
from drqp_brain.joint_trajectory_builder import kFemurOffsetAngle, kTibiaOffsetAngle
from drqp_kinematics.models import HexapodModel
from geometry_msgs.msg import PoseStamped, Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_pytest
from launch_pytest.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare
from moveit_launch_smoke_test_support import build_test_gz_partition
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
import pytest
import rclpy
from rclpy.action import ActionClient
import rclpy.time
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

BASE_FRAME = 'drqp/base_center_link'
GROUP_NAME = 'left_front_leg'
LEFT_FRONT_JOINTS = [
    'drqp/left_front_coxa',
    'drqp/left_front_femur',
    'drqp/left_front_tibia',
]
TARGET_OBSTACLE_ID = 'issue43_left_front_target_blocker'


@pytest.mark.slow
@launch_pytest.fixture
def generate_test_description():
    demo_gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('drqp_moveit'), 'launch', 'demo_gazebo.launch.py']
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_gazebo_launch),
                launch_arguments={
                    'show_rviz': 'false',
                    'sim_gui': 'false',
                    'gz_partition': build_test_gz_partition('moveit_runtime_launch'),
                }.items(),
            ),
            TimerAction(period=1.0, actions=[ReadyToTest()]),
        ]
    )


@pytest.mark.slow
@pytest.mark.launch(fixture=generate_test_description)
class TestMoveItRuntimeIssue43:
    READY_TIMEOUT = 90.0
    JOINT_TOLERANCE = 0.08

    FRONT_OFFSET = 0.11692
    SIDE_OFFSET = 0.06387
    MIDDLE_OFFSET = 0.103
    COXA_LENGTH = 0.0265 * 2.0
    FEMUR_LENGTH = math.hypot(0.036 + 0.0055 + 0.023, 0.015)
    TIBIA_LENGTH = math.hypot(0.0325 + 0.052797, 0.088395)

    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.try_shutdown()

    def setup_method(self, method):
        self.node = rclpy.create_node('test_moveit_runtime_issue43')
        self.latest_joint_state = None
        self.latest_clock = None
        self._active_obstacle_ids: set[str] = set()

        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        self.clock_sub = self.node.create_subscription(Clock, '/clock', self._clock_callback, 10)

        self.motion_plan_client = self.node.create_client(
            GetMotionPlan,
            '/plan_kinematic_path',
        )
        self.state_validity_client = self.node.create_client(
            GetStateValidity,
            '/check_state_validity',
        )
        self.apply_planning_scene_client = self.node.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )
        self.follow_joint_trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

        self._wait_for_runtime_ready()

    def teardown_method(self, method):
        self._clear_obstacles()
        self.follow_joint_trajectory_client.destroy()
        self.apply_planning_scene_client.destroy()
        self.state_validity_client.destroy()
        self.motion_plan_client.destroy()
        self.clock_sub.destroy()
        self.joint_state_sub.destroy()
        self.node.destroy_node()

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _clock_callback(self, msg: Clock) -> None:
        self.latest_clock = msg.clock

    def _spin_until(self, predicate, timeout_sec: float, error_message: str) -> None:
        deadline = self.node.get_clock().now() + rclpy.time.Duration(seconds=timeout_sec)
        while self.node.get_clock().now() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        pytest.fail(error_message)

    def _wait_for_runtime_ready(self) -> None:
        self._spin_until(
            lambda: self.latest_clock is not None and self.latest_joint_state is not None,
            self.READY_TIMEOUT,
            'Timed out waiting for /clock and /joint_states',
        )

        check_node_running(self.node, 'robot_state_publisher', timeout=self.READY_TIMEOUT)
        check_node_running(self.node, 'move_group', timeout=self.READY_TIMEOUT)
        self._wait_for_active_controllers(
            ['joint_state_broadcaster', 'joint_trajectory_controller']
        )

        for client, name in [
            (self.motion_plan_client, '/plan_kinematic_path'),
            (self.state_validity_client, '/check_state_validity'),
            (self.apply_planning_scene_client, '/apply_planning_scene'),
        ]:
            assert client.wait_for_service(timeout_sec=self.READY_TIMEOUT), \
                f'{name} service is not available'

        assert self.follow_joint_trajectory_client.wait_for_server(
            timeout_sec=self.READY_TIMEOUT
        ), 'FollowJointTrajectory action is not available'
        self._assert_single_move_group_node()

    def _assert_single_move_group_node(self) -> None:
        move_group_nodes = [name for name in self.node.get_node_names() if name == 'move_group']
        assert len(move_group_nodes) == 1, (
            f'Expected one move_group node, found {len(move_group_nodes)}'
        )

    def _wait_for_active_controllers(self, controller_names: list[str]) -> None:
        """Retry controller_manager checks until startup races settle."""

        def controllers_are_active() -> bool:
            try:
                check_controllers_running(
                    self.node,
                    controller_names,
                    timeout=1.0,
                )
            except AssertionError:
                return False
            return True

        self._spin_until(
            controllers_are_active,
            self.READY_TIMEOUT,
            (f'Timed out waiting for active controllers: {sorted(controller_names)}'),
        )

    def _current_joint_map(self) -> dict[str, float]:
        assert self.latest_joint_state is not None
        return dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))

    def _current_robot_state(self) -> RobotState:
        assert self.latest_joint_state is not None
        return RobotState(joint_state=self.latest_joint_state, is_diff=False)

    def _make_hexapod_model(self) -> HexapodModel:
        return HexapodModel(
            front_offset=self.FRONT_OFFSET,
            side_offset=self.SIDE_OFFSET,
            middle_offset=self.MIDDLE_OFFSET,
            coxa_len=self.COXA_LENGTH,
            femur_len=self.FEMUR_LENGTH,
            tibia_len=self.TIBIA_LENGTH,
        )

    def _reachable_target(self) -> tuple[PoseStamped, dict[str, float]]:
        model = self._make_hexapod_model()
        leg = model.left_front
        analytical_angles_deg = (10.0, -40.0, 70.0)
        leg.forward_kinematics(*analytical_angles_deg)

        target_pose = PoseStamped()
        target_pose.header.frame_id = BASE_FRAME
        target_pose.pose.position.x = float(leg.tibia_end.x)
        target_pose.pose.position.y = float(leg.tibia_end.y)
        target_pose.pose.position.z = float(leg.tibia_end.z)

        quat = Rotation.from_matrix(leg.tibia_link.rotation).as_quat()
        target_pose.pose.orientation = Quaternion(
            x=float(quat[0]),
            y=float(quat[1]),
            z=float(quat[2]),
            w=float(quat[3]),
        )

        expected_joints = {
            LEFT_FRONT_JOINTS[0]: math.radians(analytical_angles_deg[0]),
            LEFT_FRONT_JOINTS[1]: math.radians(analytical_angles_deg[1] + kFemurOffsetAngle),
            LEFT_FRONT_JOINTS[2]: math.radians(analytical_angles_deg[2] + kTibiaOffsetAngle),
        }
        return target_pose, expected_joints

    def _call_service(self, client, request, timeout_sec: float = 30.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        assert future.done(), f'{client.srv_name} request did not complete'
        assert future.exception() is None, future.exception()
        response = future.result()
        assert response is not None
        return response

    def _goal_constraints_for(self, target_positions: dict[str, float]) -> Constraints:
        constraints = Constraints()
        for joint_name, joint_position in target_positions.items():
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint_name,
                    position=joint_position,
                    tolerance_above=0.01,
                    tolerance_below=0.01,
                    weight=1.0,
                )
            )
        return constraints

    def _plan_to_joint_target(self, target_positions: dict[str, float]):
        request = GetMotionPlan.Request()
        motion_plan_request = request.motion_plan_request
        motion_plan_request.group_name = GROUP_NAME
        motion_plan_request.start_state = self._current_robot_state()
        motion_plan_request.goal_constraints = [self._goal_constraints_for(target_positions)]
        motion_plan_request.num_planning_attempts = 2
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.2
        motion_plan_request.max_acceleration_scaling_factor = 0.2
        return self._call_service(self.motion_plan_client, request)

    def _robot_state_with_joint_targets(self, target_positions: dict[str, float]) -> RobotState:
        robot_state = self._current_robot_state()
        joint_map = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        joint_map.update(target_positions)
        robot_state.joint_state.position = [
            joint_map[joint_name] for joint_name in robot_state.joint_state.name
        ]
        return robot_state

    def _assert_joint_map_close(
        self,
        actual: dict[str, float],
        expected: dict[str, float],
        tolerance: float,
    ) -> None:
        for joint_name, expected_position in expected.items():
            assert joint_name in actual
            assert abs(actual[joint_name] - expected_position) <= tolerance, (
                f'{joint_name} mismatch: expected {expected_position:.4f}, '
                f'got {actual[joint_name]:.4f}'
            )

    def _execute_trajectory(self, robot_trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = robot_trajectory.joint_trajectory

        goal_future = self.follow_joint_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=self.READY_TIMEOUT)
        goal_handle = goal_future.result()
        assert goal_handle is not None
        assert goal_handle.accepted, 'FollowJointTrajectory goal was rejected'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.READY_TIMEOUT,
        )
        result = result_future.result()
        assert result is not None
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return result.result

    def _wait_for_joint_positions(
        self,
        expected_positions: dict[str, float],
        tolerance: float,
        timeout_sec: float = 30.0,
    ) -> None:
        def reached_target() -> bool:
            if self.latest_joint_state is None:
                return False
            current_positions = self._current_joint_map()
            return all(
                abs(current_positions[joint_name] - expected_position) <= tolerance
                for joint_name, expected_position in expected_positions.items()
            )

        self._spin_until(
            reached_target,
            timeout_sec,
            'Timed out waiting for joint states to match the executed trajectory',
        )

    def _apply_target_obstacle(
        self,
        target_pose: PoseStamped,
        blocked_state: RobotState | None = None,
    ) -> None:
        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_FRAME
        collision_object.id = TARGET_OBSTACLE_ID
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.08, 0.08, 0.08])
        ]
        collision_object.primitive_poses = [target_pose.pose]

        planning_scene = PlanningScene(is_diff=True)
        planning_scene.world.collision_objects = [collision_object]

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        response = self._call_service(self.apply_planning_scene_client, request)
        assert response.success, 'Failed to apply obstacle to the planning scene'
        self._active_obstacle_ids.add(TARGET_OBSTACLE_ID)

        if blocked_state is not None:
            self._spin_until(
                lambda: not self._state_validity(blocked_state).valid,
                timeout_sec=5.0,
                error_message='Obstacle did not invalidate the target state in time',
            )

    def _remove_obstacle(self, obstacle_id: str) -> None:
        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_FRAME
        collision_object.id = obstacle_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene(is_diff=True)
        planning_scene.world.collision_objects = [collision_object]

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        response = self._call_service(self.apply_planning_scene_client, request)
        assert response.success, f'Failed to remove obstacle {obstacle_id}'

    def _clear_obstacles(self) -> None:
        for obstacle_id in list(self._active_obstacle_ids):
            self._remove_obstacle(obstacle_id)
            self._active_obstacle_ids.remove(obstacle_id)

    def _state_validity(self, robot_state: RobotState):
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = GROUP_NAME
        return self._call_service(self.state_validity_client, request)

    def test_issue43_left_front_leg_analytical_target_get_motion_plan_succeeds(self):
        _, expected_joint_positions = self._reachable_target()

        plan_response = self._plan_to_joint_target(expected_joint_positions)
        assert plan_response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS, (
            f'Planning failed with code {plan_response.motion_plan_response.error_code.val}'
        )
        assert plan_response.motion_plan_response.trajectory.joint_trajectory.points, \
            'Expected a non-empty planned trajectory'

    def test_issue43_execute_trajectory_reaches_planned_goal_via_joint_trajectory_controller(self):
        _, target_joint_positions = self._reachable_target()
        plan_response = self._plan_to_joint_target(target_joint_positions)
        assert plan_response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS

        robot_trajectory = plan_response.motion_plan_response.trajectory
        trajectory_joint_names = robot_trajectory.joint_trajectory.joint_names
        final_point = robot_trajectory.joint_trajectory.points[-1]
        final_joint_map = dict(zip(trajectory_joint_names, final_point.positions))
        self._execute_trajectory(robot_trajectory)
        self._wait_for_joint_positions(
            {joint_name: final_joint_map[joint_name] for joint_name in LEFT_FRONT_JOINTS},
            tolerance=self.JOINT_TOLERANCE,
        )

    def test_issue43_collision_object_blocks_goal_state_and_plan_is_rejected(self):
        target_pose, target_joint_positions = self._reachable_target()
        blocked_state = self._robot_state_with_joint_targets(target_joint_positions)
        self._apply_target_obstacle(target_pose, blocked_state=blocked_state)

        validity_response = self._state_validity(blocked_state)
        assert not validity_response.valid, 'Expected the blocked target state to be invalid'

        plan_response = self._plan_to_joint_target(target_joint_positions)
        assert plan_response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS, \
            'Expected planning to fail for a blocked target state'


@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_moveit_runtime_launch_shutdown():
    pass
