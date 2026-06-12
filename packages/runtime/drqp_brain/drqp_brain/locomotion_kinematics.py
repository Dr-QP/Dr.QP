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

from dataclasses import dataclass
import importlib
import math
import threading
from typing import Callable

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
import numpy as np
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.duration import Duration
from rclpy.exceptions import NotInitializedException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

MOVEIT_IK_SERVICE = '/compute_ik'
MOVEIT_IK_TIMEOUT_SEC = 2.0
BASE_FRAME = 'drqp/base_center_link'

RCLPY_SHUTDOWN_ERRORS = (InvalidHandle, NotInitializedException, RCLError, RuntimeError)


@dataclass(frozen=True)
class LocomotionKinematicsResult:
    """Result of solving and validating one complete locomotion state."""

    joint_targets: dict[str, float]
    robot_state: RobotState | None = None
    failure_reason: str | None = None
    error_code: int | None = None

    @property
    def succeeded(self) -> bool:
        return self.failure_reason is None


class MoveItServiceLocomotionKinematics:
    """MoveIt-backed locomotion helper for leg IK and whole-state validation."""

    def __init__(
        self,
        node,
        hexapod,
        callback_group,
        track_future: Callable,
        is_shutting_down: Callable[[], bool],
    ):
        self._node = node
        self._hexapod = hexapod
        self._callback_group = callback_group
        self._track_future = track_future
        self._is_shutting_down = is_shutting_down
        self._ik_client = None
        self._service_warning_logged = False

    @property
    def ik_client(self):
        """Create the persistent MoveIt IK service client on first use."""
        if self._ik_client is None:
            self._ik_client = self._node.create_client(
                GetPositionIK,
                MOVEIT_IK_SERVICE,
                callback_group=self._callback_group,
            )
        return self._ik_client

    def ready(self) -> bool:
        """Return whether the persistent MoveIt IK service client is usable."""
        if self._is_shutting_down():
            return False

        try:
            service_available = self.ik_client.wait_for_service(timeout_sec=0.0)
        except RCLPY_SHUTDOWN_ERRORS as exc:
            if not self._is_shutting_down():
                self._node.get_logger().warning(
                    f'MoveIt IK service {MOVEIT_IK_SERVICE} readiness check failed: {exc}'
                )
            return False

        if not service_available:
            if not self._service_warning_logged:
                self._node.get_logger().warning(
                    f'MoveIt IK service {MOVEIT_IK_SERVICE} is unavailable'
                )
                self._service_warning_logged = True
            return False

        self._service_warning_logged = False
        return True

    def solve(
        self,
        legs_and_targets,
        latest_joint_state: JointState,
    ) -> LocomotionKinematicsResult:
        """Solve all requested leg targets and validate the merged robot state."""
        robot_state = self.current_robot_state(latest_joint_state)
        joint_targets = {}

        for leg, foot_target in legs_and_targets:
            response = self.call_ik(leg, foot_target, robot_state)
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                return LocomotionKinematicsResult(
                    joint_targets={},
                    robot_state=robot_state,
                    failure_reason=(
                        f'MoveIt IK failed for {leg.label.name} with code '
                        f'{response.error_code.val}'
                    ),
                    error_code=response.error_code.val,
                )

            robot_state = self.merge_leg_solution(robot_state, response.solution, leg)
            joint_targets.update(self.extract_leg_joint_targets(leg, robot_state))

        validation_failure = self.validate_complete_state(robot_state)
        if validation_failure is not None:
            return LocomotionKinematicsResult(
                joint_targets={},
                robot_state=robot_state,
                failure_reason=validation_failure,
            )

        return LocomotionKinematicsResult(joint_targets=joint_targets, robot_state=robot_state)

    def call_ik(self, leg, foot_target, robot_state: RobotState):
        if self._is_shutting_down():
            raise RuntimeError('MoveIt IK request aborted because drqp_brain is shutting down')

        request = self.build_ik_request(leg, foot_target, robot_state)
        future = self.ik_client.call_async(request)
        self._track_future(future)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=MOVEIT_IK_TIMEOUT_SEC):
            self._safe_cancel_future(future)
            if self._is_shutting_down():
                raise RuntimeError(
                    f'MoveIt IK request aborted for {leg.label.name} during shutdown'
                )
            raise RuntimeError(
                f'MoveIt IK request timed out for {leg.label.name} after '
                f'{MOVEIT_IK_TIMEOUT_SEC:.1f}s'
            )

        if not future.done():
            self._safe_cancel_future(future)
            raise RuntimeError(
                f'MoveIt IK request timed out for {leg.label.name} after '
                f'{MOVEIT_IK_TIMEOUT_SEC:.1f}s'
            )

        future_exception = future.exception()
        if future_exception is not None:
            raise RuntimeError(
                f'MoveIt IK request for {leg.label.name} raised an exception: {future_exception}'
            ) from future_exception
        response = future.result()

        if response is None:
            raise RuntimeError(f'MoveIt IK returned no response for {leg.label.name}')
        return response

    @staticmethod
    def current_robot_state(latest_joint_state: JointState) -> RobotState:
        robot_state = RobotState()
        robot_state.joint_state = JointState(
            header=latest_joint_state.header,
            name=list(latest_joint_state.name),
            position=list(latest_joint_state.position),
            velocity=list(latest_joint_state.velocity),
            effort=list(latest_joint_state.effort),
        )
        return robot_state

    def build_ik_request(self, leg, foot_target, robot_state: RobotState):
        request = GetPositionIK.Request()
        request.ik_request.group_name = f'{leg.label.name}_leg'
        request.ik_request.robot_state = robot_state
        request.ik_request.avoid_collisions = False
        request.ik_request.ik_link_name = f'drqp/{leg.label.name}_foot_link'
        request.ik_request.pose_stamped = self.make_pose_stamped(leg, foot_target)
        request.ik_request.timeout = Duration(seconds=MOVEIT_IK_TIMEOUT_SEC).to_msg()
        return request

    def make_pose_stamped(self, leg, foot_target):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        base_frame_target = self._hexapod.body_transform.inverse.apply_point(foot_target)
        pose.pose.position.x = float(base_frame_target.x)
        pose.pose.position.y = float(base_frame_target.y)
        pose.pose.position.z = float(base_frame_target.z)

        orientation = Rotation.from_matrix(leg.tibia_link.rotation).as_quat()
        pose.pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )
        return pose

    def merge_leg_solution(
        self,
        robot_state: RobotState,
        solution_state: RobotState,
        leg,
    ) -> RobotState:
        merged_state = self.copy_robot_state(robot_state)
        merged_positions = dict(
            zip(merged_state.joint_state.name, merged_state.joint_state.position)
        )
        solution_positions = dict(
            zip(solution_state.joint_state.name, solution_state.joint_state.position)
        )

        for joint_name in self.controller_joint_names(leg):
            if joint_name not in solution_positions:
                raise RuntimeError(f'MoveIt IK response missing joint target for {joint_name}')
            merged_positions[joint_name] = solution_positions[joint_name]

        merged_state.joint_state.position = [
            merged_positions[name] for name in merged_state.joint_state.name
        ]
        return merged_state

    @staticmethod
    def copy_robot_state(robot_state: RobotState) -> RobotState:
        copied = RobotState()
        copied.joint_state = JointState(
            header=robot_state.joint_state.header,
            name=list(robot_state.joint_state.name),
            position=list(robot_state.joint_state.position),
            velocity=list(robot_state.joint_state.velocity),
            effort=list(robot_state.joint_state.effort),
        )
        copied.multi_dof_joint_state = robot_state.multi_dof_joint_state
        copied.attached_collision_objects = list(robot_state.attached_collision_objects)
        copied.is_diff = robot_state.is_diff
        return copied

    def extract_leg_joint_targets(self, leg, robot_state: RobotState):
        joint_map = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        joint_targets = {}
        for joint_name in self.controller_joint_names(leg):
            if joint_name not in joint_map:
                raise RuntimeError(f'MoveIt IK response missing joint target for {joint_name}')
            joint_targets[joint_name] = joint_map[joint_name]
        return joint_targets

    def validate_complete_state(self, robot_state: RobotState) -> str | None:
        joint_map = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        expected_names = [
            joint_name
            for leg in self._hexapod.legs
            for joint_name in self.controller_joint_names(leg)
        ]

        missing_names = [
            joint_name for joint_name in expected_names if joint_name not in joint_map
        ]
        if missing_names:
            return f'RobotState is missing joint targets: {", ".join(missing_names)}'

        invalid_names = [
            joint_name for joint_name in expected_names if not math.isfinite(joint_map[joint_name])
        ]
        if invalid_names:
            return f'RobotState contains non-finite joint targets: {", ".join(invalid_names)}'

        return None

    @staticmethod
    def controller_joint_names(leg):
        return [f'drqp/{leg.label.name}_{joint_name}' for joint_name in ('coxa', 'femur', 'tibia')]

    @staticmethod
    def _safe_cancel_future(future):
        if future.done():
            return
        future.cancel()

    def destroy(self):
        if self._ik_client is None:
            return

        try:
            self._ik_client.destroy()
        except RCLPY_SHUTDOWN_ERRORS as exc:
            try:
                self._node.get_logger().warning(f'Failed to destroy MoveIt IK client: {exc}')
            except RCLPY_SHUTDOWN_ERRORS:
                pass
        self._ik_client = None


class MoveItPyLocomotionKinematics:
    """In-process MoveIt kinematics helper backed by moveit_py bindings."""

    def __init__(
        self,
        node,
        hexapod,
        is_shutting_down: Callable[[], bool],
        moveit_py_factory=None,
        robot_state_cls=None,
    ):
        self._node = node
        self._hexapod = hexapod
        self._is_shutting_down = is_shutting_down
        self._moveit_py_factory = moveit_py_factory
        self._robot_state_cls = robot_state_cls
        self._moveit_py = None
        self._robot_model = None
        self._planning_scene_monitor = None
        self._initialization_error_logged = False

    def ready(self) -> bool:
        if self._is_shutting_down():
            return False

        try:
            self._ensure_moveit_py()
        except RuntimeError as exc:
            if not self._initialization_error_logged:
                self._node.get_logger().warning(
                    f'MoveItPy locomotion helper is unavailable: {exc}'
                )
                self._initialization_error_logged = True
            return False
        return True

    def solve(
        self,
        legs_and_targets,
        latest_joint_state: JointState,
    ) -> LocomotionKinematicsResult:
        self._ensure_moveit_py()
        robot_state = self._seed_robot_state(latest_joint_state)
        joint_targets = {}

        for leg, foot_target in legs_and_targets:
            group_name = self.group_name(leg)
            tip_name = self.tip_link_name(leg)
            pose = self.make_pose_stamped(leg, foot_target).pose
            solved = robot_state.set_from_ik(
                group_name,
                pose,
                tip_name,
                MOVEIT_IK_TIMEOUT_SEC,
            )
            if not solved:
                return LocomotionKinematicsResult(
                    joint_targets={},
                    robot_state=None,
                    failure_reason=f'MoveItPy IK failed for {leg.label.name}',
                )

            robot_state.update()
            joint_targets.update(self.extract_leg_joint_targets(leg, robot_state))

        validation_failure = self.validate_complete_state(robot_state)
        if validation_failure is not None:
            return LocomotionKinematicsResult(
                joint_targets={},
                robot_state=None,
                failure_reason=validation_failure,
            )

        return LocomotionKinematicsResult(joint_targets=joint_targets, robot_state=None)

    def _ensure_moveit_py(self):
        if self._moveit_py is not None:
            return

        moveit_py_factory = self._moveit_py_factory
        robot_state_cls = self._robot_state_cls
        if moveit_py_factory is None or robot_state_cls is None:
            try:
                planning = importlib.import_module('moveit.planning')
                robot_state_module = importlib.import_module('moveit.core.robot_state')
            except ImportError as exc:
                raise RuntimeError('moveit_py Python bindings are not installed') from exc

            moveit_py_factory = planning.MoveItPy
            robot_state_cls = robot_state_module.RobotState

        try:
            self._moveit_py = moveit_py_factory(node_name=self._node.get_name())
            self._robot_model = self._moveit_py.get_robot_model()
            self._planning_scene_monitor = self._moveit_py.get_planning_scene_monitor()
            self._robot_state_cls = robot_state_cls
        except Exception as exc:
            self._moveit_py = None
            self._robot_model = None
            self._planning_scene_monitor = None
            raise RuntimeError(str(exc)) from exc

    def _seed_robot_state(self, latest_joint_state: JointState):
        robot_state = self._robot_state_cls(self._robot_model)
        latest_positions = dict(zip(latest_joint_state.name, latest_joint_state.position))

        for leg in self._hexapod.legs:
            group_name = self.group_name(leg)
            joint_names = self.controller_joint_names(leg)
            missing_names = [
                joint_name for joint_name in joint_names if joint_name not in latest_positions
            ]
            if missing_names:
                raise RuntimeError(
                    f'Current joint state is missing joint targets: {", ".join(missing_names)}'
                )

            robot_state.set_joint_group_positions(
                group_name,
                np.asarray([latest_positions[joint_name] for joint_name in joint_names]),
            )

        robot_state.update()
        return robot_state

    def make_pose_stamped(self, leg, foot_target):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        base_frame_target = self._hexapod.body_transform.inverse.apply_point(foot_target)
        pose.pose.position.x = float(base_frame_target.x)
        pose.pose.position.y = float(base_frame_target.y)
        pose.pose.position.z = float(base_frame_target.z)

        orientation = Rotation.from_matrix(leg.tibia_link.rotation).as_quat()
        pose.pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )
        return pose

    def extract_leg_joint_targets(self, leg, robot_state):
        positions = robot_state.get_joint_group_positions(self.group_name(leg))
        return {
            joint_name: float(position)
            for joint_name, position in zip(self.controller_joint_names(leg), positions)
        }

    def validate_complete_state(self, robot_state) -> str | None:
        joint_targets = {}
        for leg in self._hexapod.legs:
            joint_targets.update(self.extract_leg_joint_targets(leg, robot_state))

        missing_names = [
            joint_name
            for leg in self._hexapod.legs
            for joint_name in self.controller_joint_names(leg)
            if joint_name not in joint_targets
        ]
        if missing_names:
            return f'RobotState is missing joint targets: {", ".join(missing_names)}'

        invalid_names = [
            joint_name
            for joint_name, position in joint_targets.items()
            if not math.isfinite(position)
        ]
        if invalid_names:
            return f'RobotState contains non-finite joint targets: {", ".join(invalid_names)}'

        collision_failure = self._collision_failure(robot_state)
        if collision_failure is not None:
            return collision_failure

        return None

    def _collision_failure(self, robot_state) -> str | None:
        if self._planning_scene_monitor is None:
            return None

        try:
            with self._planning_scene_monitor.read_only() as planning_scene:
                if planning_scene.is_state_colliding(robot_state, '', False):
                    return 'RobotState is in self-collision'
        except Exception as exc:
            return f'Unable to validate RobotState collision status: {exc}'

        return None

    @staticmethod
    def group_name(leg) -> str:
        return f'{leg.label.name}_leg'

    @staticmethod
    def tip_link_name(leg) -> str:
        return f'drqp/{leg.label.name}_foot_link'

    @staticmethod
    def controller_joint_names(leg):
        return [f'drqp/{leg.label.name}_{joint_name}' for joint_name in ('coxa', 'femur', 'tibia')]

    def destroy(self):
        if self._moveit_py is None:
            return

        try:
            self._moveit_py.shutdown()
        except Exception as exc:
            try:
                self._node.get_logger().warning(f'Failed to shut down MoveItPy helper: {exc}')
            except RCLPY_SHUTDOWN_ERRORS:
                pass
        self._moveit_py = None
        self._robot_model = None
        self._planning_scene_monitor = None


class FallbackLocomotionKinematics:
    """Prefer in-process MoveItPy, falling back to the service helper when needed."""

    def __init__(self, primary, fallback):
        self._primary = primary
        self._fallback = fallback
        self._using_fallback = False
        self._fallback_warning_logged = False

    @property
    def ik_client(self):
        return self._fallback.ik_client

    def ready(self) -> bool:
        if self._using_fallback:
            return self._fallback.ready()

        if self._primary.ready():
            return True

        self._using_fallback = True
        self._log_fallback()
        return self._fallback.ready()

    def solve(
        self,
        legs_and_targets,
        latest_joint_state: JointState,
    ) -> LocomotionKinematicsResult:
        if self._using_fallback:
            return self._fallback.solve(legs_and_targets, latest_joint_state)

        try:
            return self._primary.solve(legs_and_targets, latest_joint_state)
        except RuntimeError:
            self._using_fallback = True
            self._log_fallback()
            return self._fallback.solve(legs_and_targets, latest_joint_state)

    def make_pose_stamped(self, leg, foot_target):
        if self._using_fallback:
            return self._fallback.make_pose_stamped(leg, foot_target)
        return self._primary.make_pose_stamped(leg, foot_target)

    def current_robot_state(self, latest_joint_state: JointState) -> RobotState:
        return self._fallback.current_robot_state(latest_joint_state)

    def build_ik_request(self, leg, foot_target, robot_state: RobotState):
        return self._fallback.build_ik_request(leg, foot_target, robot_state)

    def call_ik(self, leg, foot_target, robot_state: RobotState):
        return self._fallback.call_ik(leg, foot_target, robot_state)

    def extract_leg_joint_targets(self, leg, robot_state):
        if isinstance(robot_state, RobotState):
            return self._fallback.extract_leg_joint_targets(leg, robot_state)
        return self._primary.extract_leg_joint_targets(leg, robot_state)

    def controller_joint_names(self, leg):
        return self._fallback.controller_joint_names(leg)

    def destroy(self):
        self._primary.destroy()
        self._fallback.destroy()

    def _log_fallback(self):
        if self._fallback_warning_logged:
            return

        self._fallback._node.get_logger().warning(
            'Falling back to MoveIt /compute_ik service for locomotion IK'
        )
        self._fallback_warning_logged = True
