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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    show_rviz = LaunchConfiguration('show_rviz')

    description_launch_path = get_package_share_path('drqp_description') / 'launch'

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('drqp_control'),
            'config',
            'drqp_controllers.yml',
        ]
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {
                'use_sim_time': use_sim_time,
            },
            robot_controllers,
        ],
        output='both',
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )
    battery_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['battery_state_broadcaster'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--param-file', robot_controllers],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='show_rviz',
                default_value='false',
                choices=['true', 'false'],
                description='Show rviz',
            ),
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use sim time if true',
            ),
            control_node,
            joint_state_broadcaster_spawner,
            battery_state_broadcaster_spawner,
            robot_controller_spawner,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(description_launch_path / 'rsp.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_launch_path / 'rviz.launch.py'),
                ),
                condition=IfCondition(show_rviz),
            ),
        ]
    )
