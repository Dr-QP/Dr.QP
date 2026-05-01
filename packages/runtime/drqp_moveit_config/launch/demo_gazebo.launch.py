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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_pkg = get_package_share_path('drqp_moveit_config')
    move_group_launch_path = moveit_pkg / 'launch' / 'move_group.launch.py'
    moveit_rviz_launch_path = moveit_pkg / 'launch' / 'moveit_rviz.launch.py'

    show_rviz = LaunchConfiguration('show_rviz')
    sim_gui = LaunchConfiguration('sim_gui')

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

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(move_group_launch_path)),
        launch_arguments={
            'use_sim_time': 'true',
            'use_gazebo': 'true',
            'publish_fake_joint_states': 'false',
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_rviz_launch_path)),
        launch_arguments={
            'use_rviz': show_rviz,
            'use_sim_time': 'true',
            'use_gazebo': 'true',
        }.items(),
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
                    move_group_launch,
                    moveit_rviz_launch,
                ]
            ),
        ]
    )
