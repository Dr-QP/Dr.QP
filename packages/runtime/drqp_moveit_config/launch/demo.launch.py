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
Demo launch: Dr.QP + MoveIt 2 in RViz2 (no hardware, no simulation).

Starts:
  1. robot_state_publisher  – publishes robot_description from URDF/xacro
  2. joint_state_publisher  – publishes a fake /joint_states for RViz
  3. move_group             – MoveIt 2 planning node
  4. rviz2                  – RViz2 with the MoveIt Motion Planning plugin

Run with::

    ros2 launch drqp_moveit_config demo.launch.py
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    drqp_control_pkg = get_package_share_path('drqp_control')
    moveit_pkg = get_package_share_path('drqp_moveit_config')

    show_rviz = LaunchConfiguration('show_rviz')
    gui = LaunchConfiguration('gui')

    robot_description_content = ParameterValue(
        Command(
            [
                'xacro ',
                str(drqp_control_pkg / 'urdf' / 'drqp.urdf.xacro'),
            ]
        ),
        value_type=str,
    )

    srdf_content = (moveit_pkg / 'config' / 'drqp.srdf').read_text()
    kinematics = _load_yaml(moveit_pkg / 'config' / 'kinematics.yaml')
    joint_limits = _load_yaml(moveit_pkg / 'config' / 'joint_limits.yaml')
    ompl = _load_yaml(moveit_pkg / 'config' / 'ompl_planning.yaml')
    controllers = _load_yaml(moveit_pkg / 'config' / 'moveit_controllers.yaml')
    move_group_params = _load_yaml(moveit_pkg / 'config' / 'move_group.yaml')

    moveit_params = [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': srdf_content},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        ompl,
        controllers,
        move_group_params,
        {'use_sim_time': False},
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}],
    )

    # Fake joint states so the robot visualises in RViz without hardware.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=moveit_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', str(moveit_pkg / 'config' / 'moveit.rviz')],
        parameters=moveit_params,
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='show_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz2 with MoveIt Motion Planning plugin',
            ),
            DeclareLaunchArgument(
                name='gui',
                default_value='false',
                choices=['true', 'false'],
                description='Start joint_state_publisher_gui instead of joint_state_publisher',
            ),
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            move_group_node,
            rviz_node,
        ]
    )
