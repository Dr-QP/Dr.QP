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
import sys
from typing import Callable, Protocol
import xml.etree.ElementTree as ElementTree

from drqp_kinematics.urdf_limits import (
    model_joint_limits_from_urdf,
    model_to_urdf_angles,
    parse_joint_limits,
)
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.exceptions import NotInitializedException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

WALKING_TRAJECTORY_POINTS = 2
CLAMPING_EVENT_TICKS = 8
LOCOMOTION_LEG_COUNT = 6
MOVEIT_IK_ATTEMPTS_PER_TARGET = 2
MOVEIT_IK_CALLS_PER_TICK = (
    LOCOMOTION_LEG_COUNT * WALKING_TRAJECTORY_POINTS * MOVEIT_IK_ATTEMPTS_PER_TARGET
)
DEFAULT_CONTROL_RATE_HZ = 25.0
MIN_VIABLE_IK_TIMEOUT_SEC = 0.001


def _moveit_ik_timeout_for_control_rate(control_rate_hz: float) -> float:
    """
    Return a per-call MoveIt timeout that remains viable at supported rates.

    Divide half the loop period evenly across calls, but never make an individual
    MoveIt call so short that its solver cannot produce a viable result.
    """
    budget_timeout_sec = (0.5 / control_rate_hz) / MOVEIT_IK_CALLS_PER_TICK
    return max(MIN_VIABLE_IK_TIMEOUT_SEC, budget_timeout_sec)


MOVEIT_IK_TIMEOUT_SEC = _moveit_ik_timeout_for_control_rate(DEFAULT_CONTROL_RATE_HZ)
BASE_FRAME = 'drqp/base_center_link'

RCLPY_SHUTDOWN_ERRORS = (InvalidHandle, NotInitializedException, RCLError, RuntimeError)

MOVEIT_CONFIG_ROOTS = (
    'allow_trajectory_execution',
    'capabilities',
    'default_planning_pipeline',
    'disable_capabilities',
    'moveit_controller_manager',
    'moveit_simple_controller_manager',
    'ompl',
    'planning_pipelines',
    'planning_scene_monitor_options',
    'publish_geometry_updates',
    'publish_planning_scene',
    'publish_robot_description',
    'publish_robot_description_semantic',
    'publish_state_updates',
    'publish_transforms_updates',
    'robot_description',
    'robot_description_kinematics',
    'robot_description_planning',
    'robot_description_semantic',
)

REQUIRED_MOVEIT_CONFIG_ROOTS = (
    'ompl',
    'planning_pipelines',
    'robot_description',
    'robot_description_kinematics',
    'robot_description_planning',
    'robot_description_semantic',
)


@dataclass(frozen=True)
class LocomotionKinematicsResult:
    """Result of solving and validating one complete locomotion state."""

    joint_targets: dict[str, float]
    robot_state: object | None = None
    failure_reason: str | None = None
    error_code: int | None = None
    backend_name: str = 'moveit_py'
    validated: bool = False
    clamped_legs: tuple[str, ...] = ()

    @property
    def succeeded(self) -> bool:
        return self.failure_reason is None


class LocomotionKinematics(Protocol):
    """Backend contract for one complete locomotion-state solve."""

    def ready(self) -> bool:
        """Return whether the backend can solve and validate targets."""
        raise NotImplementedError

    def solve(
        self,
        legs_and_targets,
        latest_joint_state: JointState | None,
    ) -> LocomotionKinematicsResult:
        """Return controller-convention joint targets for the requested legs."""
        raise NotImplementedError

    @staticmethod
    def controller_joint_names(leg):
        """Return controller joint names in backend output order."""
        raise NotImplementedError


class MoveItPyLocomotionKinematics:
    """In-process MoveIt kinematics helper backed by moveit_py bindings."""

    def __init__(
        self,
        node,
        hexapod,
        is_shutting_down: Callable[[], bool],
        control_rate_hz: float = DEFAULT_CONTROL_RATE_HZ,
        moveit_py_factory=None,
        robot_state_cls=None,
    ):
        self._node = node
        self._hexapod = hexapod
        self._is_shutting_down = is_shutting_down
        self._ik_timeout_sec = _moveit_ik_timeout_for_control_rate(control_rate_hz)
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
        latest_joint_state: JointState | None,
    ) -> LocomotionKinematicsResult:
        if latest_joint_state is None:
            return LocomotionKinematicsResult(
                joint_targets={},
                failure_reason='MoveItPy requires a current joint state seed',
            )
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
                self._ik_timeout_sec,
            )
            if not solved:
                # The warm seed (this leg's actual current joint state) can
                # occasionally be a worse starting point for the numerical IK
                # solver than the neutral home pose for an extreme target -- e.g.
                # a diagonal stride combined with the IMU balance tilt correction
                # can push a foot target well outside this leg's everyday range
                # while the leg happens to be mid-swing. Retry once from the SRDF
                # home pose for just this leg, without disturbing any other leg's
                # already-solved joint state, so one bad seed does not turn into
                # a lasting failure that repeats identically every tick.
                robot_state.joint_positions = dict.fromkeys(self.controller_joint_names(leg), 0.0)
                robot_state.update()
                solved = robot_state.set_from_ik(
                    group_name,
                    pose,
                    tip_name,
                    self._ik_timeout_sec,
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
            config_dict = self._moveit_config_dict()
            self._moveit_py = self._create_moveit_py(moveit_py_factory, config_dict)
            self._robot_model = self._moveit_py.get_robot_model()
            self._planning_scene_monitor = self._moveit_py.get_planning_scene_monitor()
            self._robot_state_cls = robot_state_cls
        except Exception as exc:
            self._moveit_py = None
            self._robot_model = None
            self._planning_scene_monitor = None
            raise RuntimeError(str(exc)) from exc

    @staticmethod
    def _create_moveit_py(moveit_py_factory, config_dict):
        original_argv = sys.argv
        try:
            sys.argv = original_argv[:1]
            moveit_py = moveit_py_factory(
                node_name='drqp_brain_moveit_py',
                config_dict=config_dict,
                provide_planning_service=False,
            )
            return moveit_py
        finally:
            sys.argv = original_argv

    def _moveit_config_dict(self):
        flat_parameters = self._moveit_parameter_values()
        config = {}

        for root in MOVEIT_CONFIG_ROOTS:
            nested_prefix = f'{root}.'
            nested_values = {
                name.removeprefix(nested_prefix): value
                for name, value in flat_parameters.items()
                if name.startswith(nested_prefix)
            }
            if nested_values:
                config[root] = self._nested_parameter_dict(nested_values)
            elif root in flat_parameters:
                config[root] = self._plain_parameter_value(flat_parameters[root])

        self._normalize_planning_pipelines(config)
        self._prepare_planning_scene_monitor_options(config)
        config['allow_trajectory_execution'] = False

        missing_roots = [root for root in REQUIRED_MOVEIT_CONFIG_ROOTS if root not in config]
        if missing_roots:
            raise RuntimeError(
                'MoveItPy locomotion helper is missing required MoveIt parameters: '
                f'{", ".join(missing_roots)}'
            )

        return config

    def _moveit_parameter_values(self):
        values = {}
        get_parameters_by_prefix = getattr(self._node, 'get_parameters_by_prefix', None)

        if callable(get_parameters_by_prefix):
            values.update(get_parameters_by_prefix(''))

        for name, parameter in getattr(self._node, '_parameter_overrides', {}).items():
            values[name] = getattr(parameter, 'value', parameter)

        return values

    @classmethod
    def _nested_parameter_dict(cls, flat_values):
        nested = {}
        for name, value in flat_values.items():
            target = nested
            parts = name.split('.')
            for part in parts[:-1]:
                target = target.setdefault(part, {})
            target[parts[-1]] = cls._plain_parameter_value(value)
        return nested

    @classmethod
    def _plain_parameter_value(cls, value):
        if isinstance(value, tuple):
            return [cls._plain_parameter_value(item) for item in value]
        if isinstance(value, list):
            return [cls._plain_parameter_value(item) for item in value]
        if isinstance(value, dict):
            return {key: cls._plain_parameter_value(item) for key, item in value.items()}
        return value

    @staticmethod
    def _normalize_planning_pipelines(config):
        pipelines = config.get('planning_pipelines')
        if isinstance(pipelines, (list, tuple)):
            config['planning_pipelines'] = {
                'pipeline_names': list(pipelines),
                'namespace': '',
            }
        elif isinstance(pipelines, dict):
            pipelines.setdefault('namespace', '')
            if 'pipeline_names' not in pipelines and 'default_planning_pipeline' in config:
                pipelines['pipeline_names'] = [config['default_planning_pipeline']]
        elif 'default_planning_pipeline' in config:
            config['planning_pipelines'] = {
                'pipeline_names': [config['default_planning_pipeline']],
                'namespace': '',
            }

    @staticmethod
    def _prepare_planning_scene_monitor_options(config):
        options = config.setdefault('planning_scene_monitor_options', {})
        options['wait_for_initial_state_timeout'] = 0.0

    def _seed_robot_state(self, latest_joint_state: JointState):
        robot_state = self._robot_state_cls(self._robot_model)
        latest_positions = dict(zip(latest_joint_state.name, latest_joint_state.position))

        robot_state.set_to_default_values()

        seed_positions = {}
        for leg in self._hexapod.legs:
            joint_names = self.controller_joint_names(leg)
            missing_names = [
                joint_name for joint_name in joint_names if joint_name not in latest_positions
            ]
            if missing_names:
                raise RuntimeError(
                    f'Current joint state is missing joint targets: {", ".join(missing_names)}'
                )

            for joint_name in joint_names:
                seed_positions[joint_name] = latest_positions[joint_name]

        robot_state.joint_positions = seed_positions
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
                satisfies_bounds = self._satisfies_position_bounds(
                    joint_model_group,
                    positions,
                )
            except Exception as exc:  # noqa: BLE001 — MoveIt bindings raise unpredictable types
                return f'Unable to validate RobotState joint bounds: {exc}'

            if not satisfies_bounds:
                return f'RobotState violates joint bounds for {group_name}'

        return None

    @staticmethod
    def _satisfies_position_bounds(joint_model_group, positions) -> bool:
        bounds_by_joint = getattr(joint_model_group, 'active_joint_model_bounds', None)
        if bounds_by_joint is None:
            return joint_model_group.satisfies_position_bounds(np.asarray(positions))

        if len(bounds_by_joint) != len(positions):
            raise RuntimeError(
                f'expected {len(positions)} joint bounds, got {len(bounds_by_joint)}'
            )

        tolerance = 1e-9
        for position, variable_bounds in zip(positions, bounds_by_joint):
            for bounds in variable_bounds:
                if not getattr(bounds, 'position_bounded', False):
                    continue
                if position < bounds.min_position - tolerance:
                    return False
                if position > bounds.max_position + tolerance:
                    return False
        return True

    def _collision_failure(self, robot_state) -> str | None:
        if self._planning_scene_monitor is None:
            return None

        try:
            with self._planning_scene_monitor.read_only() as planning_scene:
                if planning_scene.is_state_colliding(robot_state, '', False):
                    return 'RobotState is in self-collision'
        except Exception as exc:  # noqa: BLE001 — MoveIt C++ bindings raise unpredictable types
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


class MoveItPyStateValidator(MoveItPyLocomotionKinematics):
    """Validate analytic joint targets against the MoveIt planning scene."""

    def validate_joint_targets(
        self,
        joint_targets: dict[str, float],
    ) -> tuple[object | None, str | None]:
        """Return a MoveIt RobotState and any self-collision validation failure."""
        expected_names = [
            joint_name
            for leg in self._hexapod.legs
            for joint_name in self.controller_joint_names(leg)
        ]
        missing_names = [name for name in expected_names if name not in joint_targets]
        if missing_names:
            return None, f'RobotState is missing joint targets: {", ".join(missing_names)}'

        invalid_names = [
            name for name, position in joint_targets.items() if not math.isfinite(position)
        ]
        if invalid_names:
            return None, (
                f'RobotState contains non-finite joint targets: {", ".join(invalid_names)}'
            )

        self._ensure_moveit_py()
        robot_state = self._robot_state_cls(self._robot_model)
        robot_state.set_to_default_values()
        robot_state.joint_positions = joint_targets
        robot_state.update()
        return robot_state, self._collision_failure(robot_state)


class AnalyticLocomotionKinematics:
    """Closed-form runtime kinematics backed by ``LegModel.solve_ik``."""

    def __init__(self, node, hexapod, state_validator=None):
        self._node = node
        self._hexapod = hexapod
        self._state_validator = state_validator
        self._limits_installed = all(
            leg.joint_limits_rad is not None for leg in self._hexapod.legs
        )
        self._limits_error_logged = False

    def ready(self) -> bool:
        """Return whether URDF limits and the collision validator are ready."""
        if not self._limits_installed:
            self._limits_installed = self._install_joint_limits()
        if not self._limits_installed:
            return False
        return self._state_validator is None or self._state_validator.ready()

    def solve(
        self,
        legs_and_targets,
        latest_joint_state: JointState | None,
    ) -> LocomotionKinematicsResult:
        """Solve each leg independently and report workspace/limit clamping."""
        del latest_joint_state
        if not self.ready():
            return LocomotionKinematicsResult(
                joint_targets={},
                failure_reason='Analytic kinematics is waiting for URDF joint limits',
                backend_name='analytic',
            )

        joint_targets = {}
        clamped_legs = []
        for leg, foot_target in legs_and_targets:
            solution = leg.solve_ik(foot_target, clamp=True)
            if not solution.reachable or not solution.within_limits:
                if leg.label.name not in clamped_legs:
                    clamped_legs.append(leg.label.name)
            urdf_angles = model_to_urdf_angles(solution.angles_rad)
            joint_targets.update(zip(self.controller_joint_names(leg), urdf_angles))

        assert all(math.isfinite(position) for position in joint_targets.values()), (
            'Analytic IK produced non-finite joint targets'
        )

        robot_state = None
        validation_failure = None
        validated = False
        if self._state_validator is not None:
            robot_state, validation_failure = self._state_validator.validate_joint_targets(
                joint_targets
            )
            validated = validation_failure is None

        return LocomotionKinematicsResult(
            joint_targets={} if validation_failure else joint_targets,
            robot_state=robot_state,
            failure_reason=validation_failure,
            backend_name='analytic',
            validated=validated,
            clamped_legs=tuple(clamped_legs),
        )

    def _install_joint_limits(self) -> bool:
        robot_description = self._robot_description()
        if robot_description is None:
            self._log_limits_error_once('robot_description is unavailable')
            return False

        try:
            urdf_limits = parse_joint_limits(robot_description)
            for leg in self._hexapod.legs:
                leg.joint_limits_rad = model_joint_limits_from_urdf(
                    urdf_limits,
                    tuple(self.controller_joint_names(leg)),
                )
        except (ElementTree.ParseError, KeyError, ValueError) as exc:
            self._log_limits_error_once(str(exc))
            return False
        return True

    def make_pose_stamped(self, leg, foot_target):
        """Build the legacy base-frame pose used by comparison tests."""
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

    def _robot_description(self) -> str | None:
        override = getattr(self._node, '_parameter_overrides', {}).get('robot_description')
        value = getattr(override, 'value', override)
        if isinstance(value, str):
            return value

        try:
            value = self._node.get_parameter('robot_description').value
        except Exception:  # noqa: BLE001 - node doubles and undeclared parameters vary
            return None
        return value if isinstance(value, str) else None

    def _log_limits_error_once(self, reason: str) -> None:
        if self._limits_error_logged:
            return
        self._node.get_logger().error(
            f'Unable to install analytic joint limits from URDF: {reason}'
        )
        self._limits_error_logged = True

    @staticmethod
    def controller_joint_names(leg):
        """Return controller joint names in coxa/femur/tibia order."""
        return [f'drqp/{leg.label.name}_{joint_name}' for joint_name in ('coxa', 'femur', 'tibia')]
