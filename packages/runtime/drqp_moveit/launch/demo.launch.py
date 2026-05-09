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

    ros2 launch drqp_moveit demo.launch.py
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_pkg = get_package_share_path('drqp_moveit')
    move_group_launch_path = moveit_pkg / 'launch' / 'move_group.launch.py'
    moveit_rviz_launch_path = moveit_pkg / 'launch' / 'moveit_rviz.launch.py'

    show_rviz = LaunchConfiguration('show_rviz')
    gui = LaunchConfiguration('gui')

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(move_group_launch_path)),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_fake_joint_states': 'false',
            'gui': gui,
            'hardware_device_address': 'mock_servo',
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_rviz_launch_path)),
        launch_arguments={
            'use_rviz': show_rviz,
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
                name='gui',
                default_value='false',
                choices=['true', 'false'],
                description='Start joint_state_publisher_gui instead of joint_state_publisher',
            ),
            move_group_launch,
            moveit_rviz_launch,
        ]
    )
