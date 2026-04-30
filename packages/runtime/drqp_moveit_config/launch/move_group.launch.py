# Copyright (c) 2017-2025 Anton Matosov
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

"""
MoveIt 2 move_group launch file for the Dr.QP hexapod robot.

Starts the move_group node with robot_description, SRDF, kinematics,
OMPL planning pipeline and controller configuration loaded from this
package's config/ directory.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _get_moveit_params(pkg_path):
    """Return a list of parameter dicts for the move_group node."""
    drqp_control_pkg = get_package_share_path('drqp_control')
    robot_description_content = ParameterValue(
        Command(
            [
                'xacro ',
                str(drqp_control_pkg / 'urdf' / 'drqp.urdf.xacro'),
            ]
        ),
        value_type=str,
    )
    srdf_content = (pkg_path / 'config' / 'drqp.srdf').read_text()
    kinematics = _load_yaml(pkg_path / 'config' / 'kinematics.yaml')
    joint_limits = _load_yaml(pkg_path / 'config' / 'joint_limits.yaml')
    ompl = _load_yaml(pkg_path / 'config' / 'ompl_planning.yaml')
    controllers = _load_yaml(pkg_path / 'config' / 'moveit_controllers.yaml')
    move_group = _load_yaml(pkg_path / 'config' / 'move_group.yaml')

    return [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': srdf_content},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        ompl,
        controllers,
        move_group,
    ]


def generate_launch_description():
    pkg = get_package_share_path('drqp_moveit_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=_get_moveit_params(pkg) + [{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use simulation time',
            ),
            move_group_node,
        ]
    )
