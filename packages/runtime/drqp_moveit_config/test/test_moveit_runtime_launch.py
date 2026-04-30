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
import subprocess
import unittest

from control_msgs.action import FollowJointTrajectory
from controller_manager.test_utils import check_controllers_running, check_node_running
from drqp_brain.joint_trajectory_builder import kFemurOffsetAngle, kTibiaOffsetAngle
from drqp_brain.models import HexapodModel
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing import asserts, post_shutdown_test
from launch_testing.actions import ReadyToTest
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity
import pytest
import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

BASE_FRAME = 'drqp/base_center_link'
GROUP_NAME = 'left_front_leg'
IK_LINK_NAME = 'drqp/left_front_foot_link'
LEFT_FRONT_JOINTS = [
    'drqp/left_front_coxa',
    'drqp/left_front_femur',
    'drqp/left_front_tibia',
]
TARGET_OBSTACLE_ID = 'issue43_left_front_target_blocker'


def _ensure_gz_sim_not_running() -> None:
    subprocess.run(['pkill', '-9', '-f', '^gz sim'], check=False)


@pytest.mark.slow
@pytest.mark.launch_test
def generate_test_description():
    _ensure_gz_sim_not_running()
    sim_launch = PathJoinSubstitution([FindPackageShare('drqp_gazebo'), 'launch', 'sim.launch.py'])
    move_group_launch = PathJoinSubstitution(
        [FindPackageShare('drqp_moveit_config'), 'launch', 'move_group.launch.py']
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={'sim_gui': 'false'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),
            TimerAction(period=1.0, actions=[ReadyToTest()]),
        ]
    )


@pytest.mark.slow
@pytest.mark.launch_test
class TestMoveItRuntimeIssue43(unittest.TestCase):
    READY_TIMEOUT = 90.0
    JOINT_TOLERANCE = 0.08

    FRONT_OFFSET = 0.11692
    SIDE_OFFSET = 0.06387
    MIDDLE_OFFSET = 0.103
    COXA_LENGTH = 0.0265 * 2.0
    FEMUR_LENGTH = math.hypot(0.036 + 0.0055 + 0.023, 0.015)
    TIBIA_LENGTH = math.hypot(0.0325 + 0.052797, 0.088395)

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
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

        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
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
        self.execute_trajectory_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            '/execute_trajectory',
        )
        self.follow_joint_trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

        self._wait_for_runtime_ready()

    def tearDown(self):
        self._clear_obstacles()
        self.follow_joint_trajectory_client.destroy()
        self.execute_trajectory_client.destroy()
        self.apply_planning_scene_client.destroy()
        self.state_validity_client.destroy()
        self.motion_plan_client.destroy()
        self.ik_client.destroy()
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
        self.fail(error_message)

    def _wait_for_runtime_ready(self) -> None:
        self._spin_until(
            lambda: self.latest_clock is not None and self.latest_joint_state is not None,
            self.READY_TIMEOUT,
            'Timed out waiting for /clock and /joint_states',
        )

        check_node_running(self.node, 'robot_state_publisher', timeout=self.READY_TIMEOUT)
        check_node_running(self.node, 'move_group', timeout=self.READY_TIMEOUT)
        check_controllers_running(
            self.node,
            ['joint_state_broadcaster', 'joint_trajectory_controller'],
            timeout=self.READY_TIMEOUT,
        )

        for client, name in [
            (self.ik_client, '/compute_ik'),
            (self.motion_plan_client, '/plan_kinematic_path'),
            (self.state_validity_client, '/check_state_validity'),
            (self.apply_planning_scene_client, '/apply_planning_scene'),
        ]:
            self.assertTrue(
                client.wait_for_service(timeout_sec=self.READY_TIMEOUT),
                f'{name} service is not available',
            )

        self.assertTrue(
            self.execute_trajectory_client.wait_for_server(timeout_sec=self.READY_TIMEOUT),
            'ExecuteTrajectory action is not available',
        )
        self.assertTrue(
            self.follow_joint_trajectory_client.wait_for_server(timeout_sec=self.READY_TIMEOUT),
            'FollowJointTrajectory action is not available',
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

    def _unreachable_target(self) -> PoseStamped:
        reachable_pose, _ = self._reachable_target()
        unreachable_pose = PoseStamped()
        unreachable_pose.header.frame_id = BASE_FRAME
        unreachable_pose.pose = Pose(
            position=reachable_pose.pose.position,
            orientation=reachable_pose.pose.orientation,
        )
        unreachable_pose.pose.position.x += 0.30
        unreachable_pose.pose.position.y += 0.30
        unreachable_pose.pose.position.z += 0.20
        return unreachable_pose

    def _call_service(self, client, request, timeout_sec: float = 30.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), f'{client.srv_name} request did not complete')
        self.assertIsNone(future.exception(), future.exception())
        response = future.result()
        self.assertIsNotNone(response)
        return response

    def _solve_ik(self, target_pose: PoseStamped):
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.robot_state = self._current_robot_state()
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = IK_LINK_NAME
        request.ik_request.pose_stamped = target_pose
        request.ik_request.timeout.sec = 1
        return self._call_service(self.ik_client, request)

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

    def _joint_positions_from_robot_state(
        self,
        robot_state: RobotState,
        joint_names: list[str],
    ) -> dict[str, float]:
        joint_map = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        return {joint_name: joint_map[joint_name] for joint_name in joint_names}

    def _assert_joint_map_close(
        self,
        actual: dict[str, float],
        expected: dict[str, float],
        tolerance: float,
    ) -> None:
        for joint_name, expected_position in expected.items():
            self.assertIn(joint_name, actual)
            self.assertAlmostEqual(
                actual[joint_name],
                expected_position,
                delta=tolerance,
                msg=(
                    f'{joint_name} mismatch: expected {expected_position:.4f}, '
                    f'got {actual[joint_name]:.4f}'
                ),
            )

    def _execute_trajectory(self, robot_trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        goal.controller_names = ['joint_trajectory_controller']

        goal_future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=self.READY_TIMEOUT)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted, 'ExecuteTrajectory goal was rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=self.READY_TIMEOUT,
        )
        result = result_future.result()
        self.assertIsNotNone(result)
        self.assertEqual(result.result.error_code.val, MoveItErrorCodes.SUCCESS)
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

    def _apply_target_obstacle(self, target_pose: PoseStamped) -> None:
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
        self.assertTrue(response.success, 'Failed to apply obstacle to the planning scene')
        self._active_obstacle_ids.add(TARGET_OBSTACLE_ID)

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
        self.assertTrue(response.success, f'Failed to remove obstacle {obstacle_id}')

    def _clear_obstacles(self) -> None:
        for obstacle_id in list(self._active_obstacle_ids):
            self._remove_obstacle(obstacle_id)
            self._active_obstacle_ids.remove(obstacle_id)

    def _state_validity(self, robot_state: RobotState):
        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = GROUP_NAME
        return self._call_service(self.state_validity_client, request)

    def test_issue43_left_front_leg_get_position_ik_matches_leg_model_and_get_motion_plan_succeeds(
        self,
    ):
        target_pose, expected_joint_positions = self._reachable_target()

        ik_response = self._solve_ik(target_pose)
        self.assertEqual(
            ik_response.error_code.val,
            MoveItErrorCodes.SUCCESS,
            f'IK failed with code {ik_response.error_code.val}',
        )

        actual_joint_positions = self._joint_positions_from_robot_state(
            ik_response.solution,
            LEFT_FRONT_JOINTS,
        )
        self._assert_joint_map_close(
            actual_joint_positions,
            expected_joint_positions,
            tolerance=self.JOINT_TOLERANCE,
        )

        plan_response = self._plan_to_joint_target(actual_joint_positions)
        self.assertEqual(
            plan_response.motion_plan_response.error_code.val,
            MoveItErrorCodes.SUCCESS,
            f'Planning failed with code {plan_response.motion_plan_response.error_code.val}',
        )
        self.assertTrue(
            plan_response.motion_plan_response.trajectory.joint_trajectory.points,
            'Expected a non-empty planned trajectory',
        )

    def test_issue43_execute_trajectory_reaches_planned_goal_via_joint_trajectory_controller(self):
        target_pose, _ = self._reachable_target()
        ik_response = self._solve_ik(target_pose)
        self.assertEqual(ik_response.error_code.val, MoveItErrorCodes.SUCCESS)

        target_joint_positions = self._joint_positions_from_robot_state(
            ik_response.solution,
            LEFT_FRONT_JOINTS,
        )
        plan_response = self._plan_to_joint_target(target_joint_positions)
        self.assertEqual(
            plan_response.motion_plan_response.error_code.val,
            MoveItErrorCodes.SUCCESS,
        )

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
        target_pose, _ = self._reachable_target()
        ik_response = self._solve_ik(target_pose)
        self.assertEqual(ik_response.error_code.val, MoveItErrorCodes.SUCCESS)

        target_joint_positions = self._joint_positions_from_robot_state(
            ik_response.solution,
            LEFT_FRONT_JOINTS,
        )
        self._apply_target_obstacle(target_pose)

        validity_response = self._state_validity(ik_response.solution)
        self.assertFalse(
            validity_response.valid, 'Expected the blocked target state to be invalid'
        )

        plan_response = self._plan_to_joint_target(target_joint_positions)
        self.assertNotEqual(
            plan_response.motion_plan_response.error_code.val,
            MoveItErrorCodes.SUCCESS,
            'Expected planning to fail for a blocked target state',
        )

    def test_issue43_unreachable_left_front_target_fails_without_joint_motion(self):
        before_joint_positions = self._current_joint_map()
        unreachable_pose = self._unreachable_target()

        ik_response = self._solve_ik(unreachable_pose)
        self.assertNotEqual(
            ik_response.error_code.val,
            MoveItErrorCodes.SUCCESS,
            'Expected MoveIt IK to reject an unreachable target',
        )

        self._spin_until(
            lambda: self.latest_joint_state is not None,
            timeout_sec=2.0,
            error_message='Joint states stopped updating after unreachable IK request',
        )
        after_joint_positions = self._current_joint_map()
        for joint_name in LEFT_FRONT_JOINTS:
            self.assertAlmostEqual(
                before_joint_positions[joint_name],
                after_joint_positions[joint_name],
                delta=0.01,
                msg=f'{joint_name} moved after an unreachable target request',
            )


@post_shutdown_test()
class TestMoveItRuntimeIssue43Shutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        filtered_proc_info = type(proc_info)()
        skipped_procs = ('gazebo', 'gz', 'bridge_node')
        for proc_name in proc_info.process_names():
            if not any(skip in proc_name for skip in skipped_procs):
                filtered_proc_info.append(proc_info[proc_name])
        asserts.assertExitCodes(filtered_proc_info)
