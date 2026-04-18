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
Demo launch: Dr.QP + MoveIt 2 with Gazebo simulation.

Starts:
  1. drqp_gazebo sim.launch.py     – Gazebo + ros2_control + brain
  2. move_group                    – MoveIt 2 planning node (sim time)
  3. rviz2                         – RViz2 with the MoveIt Motion Planning plugin

Run with::

    ros2 launch drqp_moveit_config demo_gazebo.launch.py

For CI / headless runs::

    ros2 launch drqp_moveit_config demo_gazebo.launch.py show_rviz:=false sim_gui:=false
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg = get_package_share_path('drqp_moveit_config')

    show_rviz = LaunchConfiguration('show_rviz')
    sim_gui = LaunchConfiguration('sim_gui')

    srdf_content = (moveit_pkg / 'config' / 'dr_qp.srdf').read_text()
    kinematics = _load_yaml(moveit_pkg / 'config' / 'kinematics.yaml')
    joint_limits = _load_yaml(moveit_pkg / 'config' / 'joint_limits.yaml')
    ompl = _load_yaml(moveit_pkg / 'config' / 'ompl_planning.yaml')
    controllers = _load_yaml(moveit_pkg / 'config' / 'moveit_controllers.yaml')
    move_group_params = _load_yaml(moveit_pkg / 'config' / 'move_group.yaml')

    moveit_params = [
        {'robot_description_semantic': srdf_content},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        ompl,
        controllers,
        move_group_params,
        {'use_sim_time': True},
    ]

    # Bring up the full Gazebo simulation stack (Gazebo + ros2_control + brain).
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('drqp_gazebo'),
                    'launch',
                    'sim.launch.py',
                ]
            )
        ),
        launch_arguments={
            'sim_gui': sim_gui,
        }.items(),
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
                name='sim_gui',
                default_value='false',
                choices=['true', 'false'],
                description='Start Gazebo GUI (false for headless CI runs)',
            ),
            GroupAction(
                [
                    SetParameter('use_sim_time', value=True),
                    gazebo_sim,
                    move_group_node,
                    rviz_node,
                ]
            ),
        ]
    )
