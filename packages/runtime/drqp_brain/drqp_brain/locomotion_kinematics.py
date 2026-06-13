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
from typing import Callable

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.exceptions import NotInitializedException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

MOVEIT_IK_TIMEOUT_SEC = 2.0
BASE_FRAME = 'drqp/base_center_link'

RCLPY_SHUTDOWN_ERRORS = (InvalidHandle, NotInitializedException, RCLError, RuntimeError)


@dataclass(frozen=True)
class LocomotionKinematicsResult:
    """Result of solving and validating one complete locomotion state."""

    joint_targets: dict[str, float]
    robot_state: object | None = None
    failure_reason: str | None = None
    error_code: int | None = None
    backend_name: str = 'moveit_py'
    validated: bool = False

    @property
    def succeeded(self) -> bool:
        return self.failure_reason is None


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
        self._initialization_failed = False
        self._initialization_error_logged = False

    def ready(self) -> bool:
        if self._is_shutting_down() or self._initialization_failed:
            return False

        try:
            self._ensure_moveit_py()
        except RuntimeError as exc:
            self._initialization_failed = True
            if not self._initialization_error_logged:
                self._node.get_logger().error(
                    f'MoveItPy locomotion helper failed permanently: {exc}'
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
            pose = self._model_frame_pose(robot_state, leg, foot_target)
            solved = robot_state.set_from_ik(
                group_name,
                pose,
                tip_name,
                MOVEIT_IK_TIMEOUT_SEC,
            )
            if not solved:
                return LocomotionKinematicsResult(
                    joint_targets={},
                    robot_state=robot_state,
                    failure_reason=f'MoveItPy IK failed for {leg.label.name}',
                )

            robot_state.update()
            joint_targets.update(self.extract_leg_joint_targets(leg, robot_state))

        validation_failure = self.validate_complete_state(robot_state)
        if validation_failure is not None:
            return LocomotionKinematicsResult(
                joint_targets={},
                robot_state=robot_state,
                failure_reason=validation_failure,
            )

        return LocomotionKinematicsResult(
            joint_targets=joint_targets,
            robot_state=robot_state,
            validated=True,
        )

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
            self._moveit_py = moveit_py_factory(
                node_name='drqp_brain_moveit_py',
                provide_planning_service=False,
            )
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

    def _model_frame_pose(self, robot_state, leg, foot_target):
        base_pose = self.make_pose_stamped(leg, foot_target).pose
        model_to_base = np.asarray(robot_state.get_frame_transform(BASE_FRAME))
        if model_to_base.shape != (4, 4):
            raise RuntimeError(
                f'MoveItPy returned invalid transform for {BASE_FRAME}: '
                f'expected 4x4, got {model_to_base.shape}'
            )

        base_position = np.asarray(
            [
                base_pose.position.x,
                base_pose.position.y,
                base_pose.position.z,
                1.0,
            ]
        )
        model_position = model_to_base @ base_position
        model_orientation = Rotation.from_matrix(model_to_base[:3, :3]) * Rotation.from_quat(
            [
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w,
            ]
        )
        orientation = model_orientation.as_quat()

        pose = Pose()
        pose.position.x = float(model_position[0])
        pose.position.y = float(model_position[1])
        pose.position.z = float(model_position[2])
        pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )
        return pose

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

        bounds_failure = self._bounds_failure(robot_state)
        if bounds_failure is not None:
            return bounds_failure

        collision_failure = self._collision_failure(robot_state)
        if collision_failure is not None:
            return collision_failure

        return None

    def _bounds_failure(self, robot_state) -> str | None:
        if self._robot_model is None:
            return 'Unable to validate RobotState joint bounds: robot model is unavailable'

        for leg in self._hexapod.legs:
            group_name = self.group_name(leg)
            try:
                joint_model_group = self._robot_model.get_joint_model_group(group_name)
                positions = robot_state.get_joint_group_positions(group_name)
                satisfies_bounds = joint_model_group.satisfies_position_bounds(
                    np.asarray(positions)
                )
            except Exception as exc:
                return f'Unable to validate RobotState joint bounds: {exc}'

            if not satisfies_bounds:
                return f'RobotState violates joint bounds for {group_name}'

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
            except RCLPY_SHUTDOWN_ERRORS as log_exc:
                if not self._is_shutting_down():
                    raise log_exc from exc
        self._moveit_py = None
        self._robot_model = None
        self._planning_scene_monitor = None
