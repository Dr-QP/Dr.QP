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

import argparse
from pathlib import Path
import sys
from unittest.mock import Mock

from ament_index_python.packages import get_package_share_path
from drqp_brain.locomotion_kinematics import MoveItPyLocomotionKinematics
from drqp_brain.parametric_gait_generator import GaitType
from drqp_brain.walk_controller import WalkController
from drqp_kinematics.geometry import Point3D
from drqp_kinematics.models import HexapodModel
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
import xacro
import yaml

DEFAULT_DIRECTIONS_COUNT = 16
DEFAULT_MAX_STEP_LENGTH_M = 0.25
DEFAULT_PHASE_SAMPLES = 32
DEFAULT_SEARCH_ITERATIONS = 12
DEFAULT_JOINT_MARGIN_DEGREES = 0.25

JOINT_LIMITS_DEGREES = {
    'coxa': (-90.0, 90.0),
    'femur': (-98.0, 90.0),
    'tibia': (-80.0, 110.0),
}


class ParameterValue:
    """Minimal launch parameter wrapper compatible with MoveItPy config loading."""

    def __init__(self, value):
        self.value = value


def create_hexapod():
    hexapod = HexapodModel(
        coxa_len=0.053,
        femur_len=0.066225,
        tibia_len=0.123,
        front_offset=0.116924,
        middle_offset=0.103,
        side_offset=0.063871,
        leg_rotation=[0, 0, 45],
    )
    hexapod.forward_kinematics(0, -35, 130)
    return hexapod


def find_max_step_length(is_safe, upper_bound: float, iterations: int) -> float:
    low = 0.0
    high = upper_bound
    for _ in range(iterations):
        candidate = (low + high) / 2.0
        if is_safe(candidate):
            low = candidate
        else:
            high = candidate
    return low


def generate_stride_limits(
    is_step_length_safe,
    directions_count: int = DEFAULT_DIRECTIONS_COUNT,
    max_step_length_m: float = DEFAULT_MAX_STEP_LENGTH_M,
    search_iterations: int = DEFAULT_SEARCH_ITERATIONS,
    joint_margin_degrees: float = DEFAULT_JOINT_MARGIN_DEGREES,
):
    result = {
        'version': 1,
        'directions_count': directions_count,
        'joint_margin_degrees': joint_margin_degrees,
        'gaits': {},
    }
    for gait in GaitType:
        samples = []
        for direction_index in range(directions_count):
            angle_degrees = direction_index * 360.0 / directions_count
            direction = Point3D(
                [
                    float(np.cos(np.radians(angle_degrees))),
                    float(np.sin(np.radians(angle_degrees))),
                    0.0,
                ]
            )
            max_safe_step_length = find_max_step_length(
                lambda step_length: is_step_length_safe(gait, direction, step_length),
                max_step_length_m,
                search_iterations,
            )
            samples.append(
                {
                    'angle_degrees': round(angle_degrees, 6),
                    'max_step_length_m': round(max_safe_step_length, 6),
                }
            )
        result['gaits'][gait.name] = samples
    return result


def make_moveit_step_length_checker(
    phase_samples: int,
    joint_margin_degrees: float,
):
    hexapod = create_hexapod()
    helper = MoveItPyLocomotionKinematics(
        node=_make_moveit_parameter_node(),
        hexapod=hexapod,
        is_shutting_down=lambda: False,
    )
    latest_joint_state = JointState(
        name=[
            f'drqp/{leg.label.name}_{joint_name}'
            for leg in hexapod.legs
            for joint_name in ('coxa', 'femur', 'tibia')
        ],
        position=[0.0] * 18,
    )

    def is_safe(gait: GaitType, direction: Point3D, step_length: float) -> bool:
        walker = WalkController(
            create_hexapod(),
            step_length=step_length,
            step_height=0.01,
            rotation_speed_degrees=45,
            gait=gait,
            phase_steps_per_cycle=phase_samples,
        )
        for phase_index in range(phase_samples):
            phase = phase_index / phase_samples
            walker.current_direction = direction.copy()
            targets = walker.next_step_targets(
                stride_direction=direction,
                rotation_direction=0.0,
                phase_override=phase,
            )
            result = helper.solve(targets, latest_joint_state)
            if not result.succeeded:
                return False
            if not _joint_targets_have_margin(result.joint_targets, joint_margin_degrees):
                return False
        return True

    return is_safe, helper


def _joint_targets_have_margin(joint_targets: dict[str, float], margin_degrees: float) -> bool:
    for joint_name, position in joint_targets.items():
        joint_kind = joint_name.rsplit('_', 1)[1]
        lower, upper = JOINT_LIMITS_DEGREES[joint_kind]
        position_degrees = float(np.degrees(position))
        if position_degrees < lower + margin_degrees:
            return False
        if position_degrees > upper - margin_degrees:
            return False
    return True


def _make_moveit_parameter_node():
    moveit_pkg = Path(get_package_share_path('drqp_moveit'))
    control_pkg = Path(get_package_share_path('drqp_control'))
    robot_description = xacro.process_file(
        str(control_pkg / 'urdf' / 'drqp.urdf.xacro'),
        mappings={'use_gazebo': 'false', 'hardware_device_address': 'mock_servo'},
    ).toxml()
    parameter_overrides = {
        'robot_description': ParameterValue(robot_description),
        'robot_description_semantic': ParameterValue(
            (moveit_pkg / 'config' / 'drqp.srdf').read_text()
        ),
        'publish_robot_description': ParameterValue(True),
        'publish_robot_description_semantic': ParameterValue(True),
        'use_sim_time': ParameterValue(False),
    }
    for config in [
        {'robot_description_kinematics': _load_yaml(moveit_pkg / 'config' / 'kinematics.yaml')},
        {'robot_description_planning': _load_yaml(moveit_pkg / 'config' / 'joint_limits.yaml')},
        _load_yaml(moveit_pkg / 'config' / 'ompl_planning.yaml'),
        _load_yaml(moveit_pkg / 'config' / 'moveit_controllers.yaml'),
        _load_yaml(moveit_pkg / 'config' / 'move_group.yaml'),
    ]:
        flattened = {}
        _flatten_parameters('', config, flattened)
        parameter_overrides.update(flattened)

    node = Mock()
    node.get_name.return_value = 'drqp_stride_limit_generator'
    node.get_logger.return_value = Mock()
    node.get_parameters_by_prefix.return_value = {}
    node._parameter_overrides = parameter_overrides
    return node


def _load_yaml(path: Path):
    with open(path) as file:
        return yaml.safe_load(file)


def _flatten_parameters(prefix: str, value, output: dict):
    if isinstance(value, dict):
        for key, item in value.items():
            item_prefix = f'{prefix}.{key}' if prefix else str(key)
            _flatten_parameters(item_prefix, item, output)
    else:
        output[prefix] = ParameterValue(value)


def default_output_path():
    return Path(get_package_share_path('drqp_brain')) / 'config' / 'stride_limits.yaml'


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Generate directional walking stride limits.')
    parser.add_argument('--output', type=Path, default=default_output_path())
    parser.add_argument('--directions', type=int, default=DEFAULT_DIRECTIONS_COUNT)
    parser.add_argument('--max-step-length', type=float, default=DEFAULT_MAX_STEP_LENGTH_M)
    parser.add_argument('--phase-samples', type=int, default=DEFAULT_PHASE_SAMPLES)
    parser.add_argument('--search-iterations', type=int, default=DEFAULT_SEARCH_ITERATIONS)
    parser.add_argument('--joint-margin-degrees', type=float, default=DEFAULT_JOINT_MARGIN_DEGREES)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    try:
        is_safe, _ = make_moveit_step_length_checker(
            phase_samples=args.phase_samples,
            joint_margin_degrees=args.joint_margin_degrees,
        )
        config = generate_stride_limits(
            is_safe,
            directions_count=args.directions,
            max_step_length_m=args.max_step_length,
            search_iterations=args.search_iterations,
            joint_margin_degrees=args.joint_margin_degrees,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, 'w') as file:
            yaml.safe_dump(config, file, sort_keys=False)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
