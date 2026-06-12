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

from types import SimpleNamespace
from unittest.mock import Mock

from drqp_brain.locomotion_kinematics import MoveItServiceLocomotionKinematics
from drqp_kinematics.models import HexapodModel
from moveit_msgs.msg import MoveItErrorCodes, RobotState
import pytest
from sensor_msgs.msg import JointState


@pytest.fixture
def hexapod():
    hexapod = HexapodModel()
    hexapod.forward_kinematics(0, -35, 130)
    return hexapod


@pytest.fixture
def kinematics(hexapod):
    node = Mock()
    node.get_logger.return_value = Mock()
    return MoveItServiceLocomotionKinematics(
        node=node,
        hexapod=hexapod,
        callback_group=Mock(),
        track_future=Mock(),
        is_shutting_down=lambda: False,
    )


def _all_joint_names(hexapod):
    return [
        f'drqp/{leg.label.name}_{joint_name}'
        for leg in hexapod.legs
        for joint_name in ('coxa', 'femur', 'tibia')
    ]


def _robot_state(joint_values):
    state = RobotState()
    state.joint_state = JointState(
        name=list(joint_values.keys()),
        position=list(joint_values.values()),
    )
    return state


def _success_response(solution_state):
    return SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=solution_state,
    )


def test_solve_merges_leg_solutions_into_complete_robot_state(kinematics, hexapod):
    """Sequential leg IK results should build one complete RobotState."""
    all_names = _all_joint_names(hexapod)
    latest_joint_state = JointState(name=all_names, position=[0.0] * len(all_names))
    selected_legs = list(hexapod.legs)[:2]
    foot_targets = [(leg, leg.tibia_end.copy()) for leg in selected_legs]
    states_seen_by_ik = []

    def fake_call_ik(leg, _foot_target, robot_state):
        states_seen_by_ik.append(
            dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        )
        joint_values = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        for index, joint_name in enumerate(kinematics.controller_joint_names(leg), start=1):
            joint_values[joint_name] = float(index + len(states_seen_by_ik) * 10)
        return _success_response(_robot_state(joint_values))

    kinematics.call_ik = fake_call_ik

    result = kinematics.solve(foot_targets, latest_joint_state)

    assert result.succeeded
    assert set(result.joint_targets) == {
        joint_name
        for leg in selected_legs
        for joint_name in kinematics.controller_joint_names(leg)
    }
    first_leg_targets = kinematics.controller_joint_names(selected_legs[0])
    assert states_seen_by_ik[1][first_leg_targets[0]] == pytest.approx(11.0)
    assert dict(zip(result.robot_state.joint_state.name, result.robot_state.joint_state.position))[
        first_leg_targets[0]
    ] == pytest.approx(11.0)


def test_validate_complete_state_reports_missing_controller_joints(kinematics, hexapod):
    """Validation should fail before publishing incomplete whole-robot states."""
    incomplete_names = _all_joint_names(hexapod)[:-1]
    state = RobotState()
    state.joint_state = JointState(name=incomplete_names, position=[0.0] * len(incomplete_names))

    failure = kinematics.validate_complete_state(state)

    assert failure is not None
    assert 'RobotState is missing joint targets' in failure


def test_validate_complete_state_reports_non_finite_controller_joints(kinematics, hexapod):
    """Validation should reject NaN or infinite controller targets."""
    all_names = _all_joint_names(hexapod)
    values = [0.0] * len(all_names)
    values[3] = float('nan')
    state = RobotState()
    state.joint_state = JointState(name=all_names, position=values)

    failure = kinematics.validate_complete_state(state)

    assert failure is not None
    assert 'RobotState contains non-finite joint targets' in failure
