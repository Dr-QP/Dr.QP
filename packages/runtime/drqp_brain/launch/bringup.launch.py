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

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    load_drivers = LaunchConfiguration('load_drivers')
    load_joystick = LaunchConfiguration('load_joystick')
    show_rviz = LaunchConfiguration('show_rviz')

    description_launch_path = get_package_share_path('drqp_description') / 'launch'

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(description_launch_path / 'rsp.launch.py'))
            ),
            DeclareLaunchArgument(
                name='show_rviz',
                default_value='false',
                choices=['true', 'false'],
                description='Show rviz',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_launch_path / 'rviz.launch.py'),
                ),
                condition=IfCondition(show_rviz),
            ),
            DeclareLaunchArgument(
                name='load_drivers',
                default_value='true',
                choices=['true', 'false'],
                description='Load drqp_a1_16_driver pose_setter',
            ),
            Node(
                package='drqp_a1_16_driver',
                executable='pose_setter',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(load_drivers),
            ),
            DeclareLaunchArgument(
                name='load_joystick',
                default_value='false',
                choices=['true', 'false'],
                description='Load joy game_controller_node',
            ),
            Node(
                package='joy',
                executable='game_controller_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(load_joystick),
            ),
            Node(
                package='drqp_control',
                executable='joint_state_to_pose',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='drqp_brain',
                executable='run_hexapod',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
