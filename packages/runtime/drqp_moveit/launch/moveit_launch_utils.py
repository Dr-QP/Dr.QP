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

from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path):
    with open(path) as file:
        return yaml.safe_load(file)


def get_moveit_params(pkg_path, use_gazebo):
    drqp_control_pkg = get_package_share_path('drqp_control')
    robot_description_content = ParameterValue(
        Command(
            [
                'xacro ',
                str(drqp_control_pkg / 'urdf' / 'drqp.urdf.xacro'),
                ' use_gazebo:=',
                use_gazebo,
            ]
        ),
        value_type=str,
    )
    srdf_content = (pkg_path / 'config' / 'drqp.srdf').read_text()
    kinematics = load_yaml(pkg_path / 'config' / 'kinematics.yaml')
    joint_limits = load_yaml(pkg_path / 'config' / 'joint_limits.yaml')
    ompl = load_yaml(pkg_path / 'config' / 'ompl_planning.yaml')
    controllers = load_yaml(pkg_path / 'config' / 'moveit_controllers.yaml')
    move_group = load_yaml(pkg_path / 'config' / 'move_group.yaml')

    return [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': srdf_content},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        ompl,
        controllers,
        move_group,
    ]
