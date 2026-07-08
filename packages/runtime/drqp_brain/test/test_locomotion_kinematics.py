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

from unittest.mock import Mock

from drqp_brain.locomotion_kinematics import MoveItPyLocomotionKinematics
from drqp_kinematics.models import HexapodModel
import numpy as np
import pytest
from sensor_msgs.msg import JointState


@pytest.fixture
def hexapod():
    hexapod = HexapodModel()
    hexapod.forward_kinematics(0, -35, 130)
    return hexapod


def _all_joint_names(hexapod):
    return [
        f'drqp/{leg.label.name}_{joint_name}'
        for leg in hexapod.legs
        for joint_name in ('coxa', 'femur', 'tibia')
    ]


class FakeJointModelGroup:
    """Fake MoveIt joint model group with configurable bounds validation."""

    def __init__(self, group_name, valid=True):
        self.group_name = group_name
        self.valid = valid
        self.positions_seen = []

    def satisfies_position_bounds(self, positions):
        self.positions_seen.append(list(positions))
        return self.valid

    @property
    def active_joint_model_bounds(self):
        return [[FakeVariableBounds(valid=self.valid)] for _ in range(3)]


class FakeVariableBounds:
    """Fake MoveIt variable bounds entry."""

    def __init__(self, valid=True):
        self.position_bounded = True
        self.min_position = -1.0
        self.max_position = 1.0 if valid else 0.0


class FakeRobotModel:
    """Fake MoveIt robot model exposing joint model groups."""

    def __init__(self, invalid_group_name=None, model_to_base=None):
        self.invalid_group_name = invalid_group_name
        self.model_to_base = model_to_base
        self.groups = {}

    def get_joint_model_group(self, group_name):
        if group_name not in self.groups:
            self.groups[group_name] = FakeJointModelGroup(
                group_name,
                valid=group_name != self.invalid_group_name,
            )
        return self.groups[group_name]


class FakePlanningSceneMonitor:
    """Fake planning scene monitor with read-only scene access."""

    def __init__(self, colliding=False):
        self.colliding = colliding

    def read_only(self):
        return self

    def __enter__(self):
        return self

    def __exit__(self, *_exc_info):
        return False

    def is_state_colliding(self, _robot_state, _group_name, _verbose):
        return self.colliding


class FakeMoveItPy:
    """Fake MoveItPy facade for unit tests."""

    def __init__(
        self,
        node_name,
        config_dict=None,
        provide_planning_service=True,
        invalid_group_name=None,
        colliding=False,
        model_to_base=None,
    ):
        self.node_name = node_name
        self.config_dict = config_dict
        self.provide_planning_service = provide_planning_service
        self.robot_model = FakeRobotModel(
            invalid_group_name=invalid_group_name,
            model_to_base=model_to_base,
        )
        self.planning_scene_monitor = FakePlanningSceneMonitor(colliding=colliding)

    def get_robot_model(self):
        return self.robot_model

    def get_planning_scene_monitor(self):
        return self.planning_scene_monitor


class FakeMoveItRobotState:
    """Fake MoveIt robot state with group-position and IK hooks."""

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.group_positions = {}
        self.ik_calls = []
        self.update_count = 0
        self._joint_positions = {}

    @property
    def joint_positions(self):
        return dict(self._joint_positions)

    @joint_positions.setter
    def joint_positions(self, positions):
        self._joint_positions.update(positions)

    def set_joint_group_positions(self, group_name, positions):
        self.group_positions[group_name] = list(positions)

    def set_to_default_values(self):
        for group_name in self.group_positions:
            self.group_positions[group_name] = [0.0, 0.0, 0.0]

    def set_from_ik(self, group_name, pose, tip_name, timeout):
        self.ik_calls.append((group_name, pose, tip_name, timeout))
        self.group_positions[group_name] = [0.1, 0.2, 0.3]
        return True

    def update(self):
        self.update_count += 1

    def get_joint_group_positions(self, group_name):
        return self.group_positions.get(group_name, [0.0, 0.0, 0.0])

    def get_frame_transform(self, _frame_id):
        if self.robot_model.model_to_base is None:
            return np.eye(4)
        return self.robot_model.model_to_base


class FakeParameter:
    """Minimal launch parameter override value wrapper."""

    def __init__(self, value):
        self.value = value


def _node_with_moveit_params():
    node = Mock()
    node.get_name.return_value = 'drqp_brain'
    node.get_logger.return_value = Mock()
    node.get_parameters_by_prefix.return_value = {}
    node._parameter_overrides = {
        'robot_description': FakeParameter('<robot name="drqp"/>'),
        'robot_description_semantic': FakeParameter('<robot name="drqp"/>'),
        'robot_description_kinematics.left_front_leg.kinematics_solver': FakeParameter(
            'kdl_kinematics_plugin/KDLKinematicsPlugin'
        ),
        'robot_description_planning.joint_limits.drqp/left_front_coxa.max_velocity': (
            FakeParameter(3.14)
        ),
        'ompl.planning_plugin': FakeParameter('ompl_interface/OMPLPlanner'),
        'planning_pipelines': FakeParameter(('ompl',)),
        'default_planning_pipeline': FakeParameter('ompl'),
        'planning_scene_monitor_options.joint_state_topic': FakeParameter('/joint_states'),
        'moveit_controller_manager': FakeParameter(
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        ),
        'moveit_simple_controller_manager.controller_names': FakeParameter(
            ('joint_trajectory_controller',)
        ),
        'moveit_simple_controller_manager.joint_trajectory_controller.type': (
            FakeParameter('FollowJointTrajectory')
        ),
        'moveit_simple_controller_manager.joint_trajectory_controller.action_ns': (
            FakeParameter('follow_joint_trajectory')
        ),
        'moveit_simple_controller_manager.joint_trajectory_controller.default': (
            FakeParameter(True)
        ),
        'moveit_simple_controller_manager.joint_trajectory_controller.joints': (
            FakeParameter(('drqp/left_front_coxa', 'drqp/left_front_femur'))
        ),
        'trajectory_execution.allowed_goal_duration_margin': FakeParameter(0.5),
        'allow_trajectory_execution': FakeParameter(True),
        'use_sim_time': FakeParameter(True),
    }
    return node


def test_moveit_config_reads_overrides_without_prefix_api(hexapod):
    """Support launch-test fakes that only expose parameter overrides."""

    class OverrideOnlyNode:
        """Minimal node double without get_parameters_by_prefix."""

        def __init__(self):
            source = _node_with_moveit_params()
            self._parameter_overrides = source._parameter_overrides

        def get_name(self):
            return 'drqp_brain'

        def get_logger(self):
            return Mock()

    helper = MoveItPyLocomotionKinematics(
        node=OverrideOnlyNode(),
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=FakeMoveItPy,
        robot_state_cls=FakeMoveItRobotState,
    )
    config = helper._moveit_config_dict()

    assert config['robot_description'] == '<robot name="drqp"/>'
    assert config['allow_trajectory_execution'] is False


def test_moveit_py_solver_uses_in_process_robot_state_ik(hexapod):
    """Verify MoveItPy helper solves leg IK without calling the service backend."""
    node = _node_with_moveit_params()
    created_moveit_py = []
    created_robot_states = []

    def moveit_py_factory(**kwargs):
        instance = FakeMoveItPy(**kwargs)
        created_moveit_py.append(instance)
        return instance

    class CapturingRobotState(FakeMoveItRobotState):
        """Capture created fake robot states."""

        def __init__(self, robot_model):
            super().__init__(robot_model)
            created_robot_states.append(self)

    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=moveit_py_factory,
        robot_state_cls=CapturingRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    leg = next(iter(hexapod.legs))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert result.succeeded
    assert created_moveit_py[0].node_name == 'drqp_brain_moveit_py'
    assert created_moveit_py[0].config_dict['planning_pipelines'] == {
        'pipeline_names': ['ompl'],
        'namespace': '',
    }
    assert (
        created_moveit_py[0].config_dict['robot_description_kinematics']['left_front_leg'][
            'kinematics_solver'
        ]
        == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    )
    assert created_moveit_py[0].config_dict['allow_trajectory_execution'] is False
    assert (
        created_moveit_py[0].config_dict['moveit_controller_manager']
        == 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    )
    assert created_moveit_py[0].config_dict['moveit_simple_controller_manager'] == {
        'controller_names': ['joint_trajectory_controller'],
        'joint_trajectory_controller': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': True,
            'joints': ['drqp/left_front_coxa', 'drqp/left_front_femur'],
        },
    }
    assert 'trajectory_execution' not in created_moveit_py[0].config_dict
    assert 'use_sim_time' not in created_moveit_py[0].config_dict
    assert (
        created_moveit_py[0].config_dict['planning_scene_monitor_options'][
            'wait_for_initial_state_timeout'
        ]
        == 0.0
    )
    assert created_moveit_py[0].provide_planning_service is False
    robot_state = created_robot_states[0]
    assert robot_state.ik_calls[0][0] == f'{leg.label.name}_leg'
    assert robot_state.ik_calls[0][2] == f'drqp/{leg.label.name}_foot_link'
    assert result.joint_targets == {
        f'drqp/{leg.label.name}_coxa': pytest.approx(0.1),
        f'drqp/{leg.label.name}_femur': pytest.approx(0.2),
        f'drqp/{leg.label.name}_tibia': pytest.approx(0.3),
    }
    assert result.backend_name == 'moveit_py'
    assert result.validated


def test_moveit_py_solver_seeds_robot_state_from_latest_joint_state(hexapod):
    """
    IK solves must warm-start from the robot's actual pose, not the SRDF default.

    Regression test: cold-seeding every solve from the default pose (instead of
    latest_joint_state) makes IK convergence near the workspace boundary depend only
    on the target and not on continuity with the previous solution. Combined with
    brain_node's rollback-on-failure, that turns a single failed solve near the
    boundary (e.g. full-throttle backward walking with IMU balance correction) into
    a persistent IK "freeze", since every retry is cold-seeded identically.
    """
    node = _node_with_moveit_params()
    created_robot_states = []

    class CapturingRobotState(FakeMoveItRobotState):
        """Capture created fake robot states."""

        def __init__(self, robot_model):
            super().__init__(robot_model)
            created_robot_states.append(self)

    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(**kwargs),
        robot_state_cls=CapturingRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    non_default_positions = [0.05 * (index + 1) for index in range(len(joint_names))]
    latest_joint_state = JointState(name=joint_names, position=non_default_positions)
    leg = next(iter(hexapod.legs))
    joint_name_to_position = dict(zip(joint_names, non_default_positions))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert result.succeeded
    seeded_positions = created_robot_states[0].joint_positions
    for joint_name in MoveItPyLocomotionKinematics.controller_joint_names(leg):
        assert seeded_positions[joint_name] == pytest.approx(joint_name_to_position[joint_name])


def test_moveit_py_solver_retries_failed_ik_from_home_pose_for_that_leg_only(hexapod):
    """
    A failed IK solve retries once from the SRDF home pose for that leg only.

    Regression test: warm-seeding from the leg's actual current joint state (see
    test_moveit_py_solver_seeds_robot_state_from_latest_joint_state) can
    occasionally be a worse starting point for the numerical solver than the
    neutral home pose for an extreme target (e.g. a diagonal stride combined
    with IMU balance tilt correction). Without a fallback, that single bad seed
    repeats identically forever because brain_node rolls back walker state on
    every failure. The retry must reseed only the failing leg's joints, leaving
    any other already-solved leg's joint_positions untouched.
    """
    node = _node_with_moveit_params()
    created_robot_states = []

    class RetryRescueRobotState(FakeMoveItRobotState):
        """Fake robot state whose first IK attempt fails, second succeeds."""

        def __init__(self, robot_model):
            super().__init__(robot_model)
            created_robot_states.append(self)
            self._attempts = 0

        def set_from_ik(self, group_name, pose, tip_name, timeout):
            self._attempts += 1
            self.ik_calls.append((group_name, pose, tip_name, timeout, dict(self.joint_positions)))
            if self._attempts == 1:
                return False
            self.group_positions[group_name] = [0.1, 0.2, 0.3]
            return True

    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(**kwargs),
        robot_state_cls=RetryRescueRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    non_default_positions = [0.05 * (index + 1) for index in range(len(joint_names))]
    latest_joint_state = JointState(name=joint_names, position=non_default_positions)
    leg = next(iter(hexapod.legs))
    leg_joint_names = MoveItPyLocomotionKinematics.controller_joint_names(leg)
    joint_name_to_position = dict(zip(joint_names, non_default_positions))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert result.succeeded
    robot_state = created_robot_states[0]
    assert len(robot_state.ik_calls) == 2
    first_attempt_seed = robot_state.ik_calls[0][4]
    second_attempt_seed = robot_state.ik_calls[1][4]
    for joint_name in leg_joint_names:
        assert first_attempt_seed[joint_name] == pytest.approx(joint_name_to_position[joint_name])
        assert second_attempt_seed[joint_name] == pytest.approx(0.0)


def test_moveit_py_solver_transforms_base_frame_target_to_model_frame(hexapod):
    """Robot IK receives poses in the model frame, not the helper base frame."""
    node = _node_with_moveit_params()
    model_to_base = np.eye(4)
    model_to_base[:3, 3] = [0.1, -0.2, 0.3]
    created_robot_states = []

    class CapturingRobotState(FakeMoveItRobotState):
        """Capture created fake robot states."""

        def __init__(self, robot_model):
            super().__init__(robot_model)
            created_robot_states.append(self)

    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(
            model_to_base=model_to_base,
            **kwargs,
        ),
        robot_state_cls=CapturingRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    leg = next(iter(hexapod.legs))
    expected_base_pose = helper.make_pose_stamped(leg, leg.tibia_end.copy()).pose

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert result.succeeded
    pose_seen_by_ik = created_robot_states[0].ik_calls[0][1]
    assert pose_seen_by_ik.position.x == pytest.approx(
        expected_base_pose.position.x + model_to_base[0, 3]
    )
    assert pose_seen_by_ik.position.y == pytest.approx(
        expected_base_pose.position.y + model_to_base[1, 3]
    )
    assert pose_seen_by_ik.position.z == pytest.approx(
        expected_base_pose.position.z + model_to_base[2, 3]
    )


def test_moveit_py_solver_returns_complete_six_leg_joint_targets(hexapod):
    """A normal walking tick should solve and validate every controller joint."""
    node = _node_with_moveit_params()
    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(**kwargs),
        robot_state_cls=FakeMoveItRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    foot_targets = [(leg, leg.tibia_end.copy()) for leg in hexapod.legs]

    result = helper.solve(foot_targets, latest_joint_state)

    assert result.succeeded
    assert result.validated
    assert set(result.joint_targets) == set(joint_names)


def test_moveit_py_validation_reports_non_finite_controller_joints(hexapod):
    """Validation should reject NaN or infinite controller targets."""
    node = _node_with_moveit_params()

    class NonFiniteRobotState(FakeMoveItRobotState):
        """Fake robot state that injects an invalid joint target."""

        def set_from_ik(self, group_name, pose, tip_name, timeout):
            solved = super().set_from_ik(group_name, pose, tip_name, timeout)
            self.group_positions[group_name][1] = float('nan')
            return solved

    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(**kwargs),
        robot_state_cls=NonFiniteRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    leg = next(iter(hexapod.legs))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert not result.succeeded
    assert not result.validated
    assert 'RobotState contains non-finite joint targets' in result.failure_reason


def test_moveit_py_validation_reports_joint_bounds_failure(hexapod):
    """Validation should reject states outside MoveIt joint bounds."""
    node = _node_with_moveit_params()
    invalid_group_name = f'{next(iter(hexapod.legs)).label.name}_leg'
    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(
            invalid_group_name=invalid_group_name,
            **kwargs,
        ),
        robot_state_cls=FakeMoveItRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    leg = next(iter(hexapod.legs))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert not result.succeeded
    assert not result.validated
    assert result.failure_reason == f'RobotState violates joint bounds for {invalid_group_name}'


def test_moveit_py_validation_reports_collision_failure(hexapod):
    """Validation should reject self-colliding complete robot states."""
    node = _node_with_moveit_params()
    helper = MoveItPyLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        is_shutting_down=lambda: False,
        moveit_py_factory=lambda **kwargs: FakeMoveItPy(colliding=True, **kwargs),
        robot_state_cls=FakeMoveItRobotState,
    )
    joint_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=joint_names, position=[0.0] * len(joint_names))
    leg = next(iter(hexapod.legs))

    result = helper.solve([(leg, leg.tibia_end.copy())], latest_joint_state)

    assert not result.succeeded
    assert not result.validated
    assert result.failure_reason == 'RobotState is in self-collision'
