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
        self.shutdown_called = False

    def get_robot_model(self):
        return self.robot_model

    def get_planning_scene_monitor(self):
        return self.planning_scene_monitor

    def shutdown(self):
        self.shutdown_called = True


class FakeMoveItRobotState:
    """Fake MoveIt robot state with group-position and IK hooks."""

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.group_positions = {}
        self.ik_calls = []
        self.update_count = 0

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
        'moveit_simple_controller_manager.controller_names': FakeParameter(
            ('joint_trajectory_controller',)
        ),
        'trajectory_execution.allowed_goal_duration_margin': FakeParameter(0.5),
        'allow_trajectory_execution': FakeParameter(True),
        'use_sim_time': FakeParameter(True),
    }
    return node


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
    assert created_moveit_py[0].config_dict['robot_description_kinematics'][
        'left_front_leg'
    ]['kinematics_solver'] == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    assert created_moveit_py[0].config_dict['allow_trajectory_execution'] is False
    assert 'moveit_simple_controller_manager' not in created_moveit_py[0].config_dict
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
