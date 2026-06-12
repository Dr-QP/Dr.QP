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


import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _moveit_params(use_gazebo, hardware_device_address):
    from ament_index_python.packages import get_package_share_path

    launch_dir = str(get_package_share_path('drqp_moveit') / 'launch')
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)

    from moveit_launch_utils import get_moveit_params

    return get_moveit_params(
        get_package_share_path('drqp_moveit'),
        use_gazebo=use_gazebo,
        hardware_device_address=hardware_device_address,
    )


def generate_launch_description():
    use_gazebo = LaunchConfiguration('use_gazebo')
    load_joystick = LaunchConfiguration('load_joystick')
    load_controllers = LaunchConfiguration('load_controllers')
    load_imu = LaunchConfiguration('load_imu')
    load_moveit = LaunchConfiguration('load_moveit')
    hardware_device_address = LaunchConfiguration('hardware_device_address')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_gazebo',
                default_value='false',
                choices=['true', 'false'],
                description='Use gazebo if true',
            ),
            DeclareLaunchArgument(
                name='load_joystick',
                default_value='false',
                choices=['true', 'false'],
                description='Load joy game_controller_node',
            ),
            DeclareLaunchArgument(
                name='load_controllers',
                default_value='true',
                choices=['true', 'false'],
                description='Load controllers',
            ),
            DeclareLaunchArgument(
                name='load_imu',
                default_value='true',
                choices=['true', 'false'],
                description='Load the BNO055 IMU node',
            ),
            DeclareLaunchArgument(
                name='load_moveit',
                default_value='true',
                choices=['true', 'false'],
                description='Load the MoveIt move_group node',
            ),
            DeclareLaunchArgument(
                name='hardware_device_address',
                default_value='/dev/ttySC0',
                description='Hardware device address passed through to MoveIt robot description',
            ),
            GroupAction(
                [
                    SetParameter('use_sim_time', value=use_gazebo),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare('drqp_control'),
                                    'launch',
                                    'ros2_controller.launch.py',
                                ]
                            ),
                        ),
                        condition=IfCondition(load_controllers),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare('drqp_moveit'),
                                    'launch',
                                    'move_group.launch.py',
                                ]
                            )
                        ),
                        condition=IfCondition(load_moveit),
                        launch_arguments={
                            'use_gazebo': use_gazebo,
                            'hardware_device_address': hardware_device_address,
                        }.items(),
                    ),
                    DeclareLaunchArgument(
                        name='load_joystick',
                        default_value='false',
                        choices=['true', 'false'],
                        description='Load drqp_joy game_controller_node',
                    ),
                    Node(
                        package='drqp_joy',
                        executable='game_controller_node',
                        output='screen',
                        condition=IfCondition(load_joystick),
                    ),
                    Node(
                        package='drqp_brain',
                        executable='drqp_joystick_translator',
                        output='screen',
                        condition=IfCondition(load_joystick),
                    ),
                    GroupAction(
                        condition=UnlessCondition(use_gazebo),
                        actions=[
                            Node(
                                package='drqp_brain',
                                executable='drqp_imu',
                                output='screen',
                                condition=IfCondition(load_imu),
                            )
                        ],
                    ),
                    Node(
                        package='drqp_brain',
                        executable='drqp_brain',
                        output='screen',
                        parameters=_moveit_params(use_gazebo, hardware_device_address)
                        + [{'use_sim_time': use_gazebo}],
                    ),
                    Node(
                        package='drqp_brain',
                        executable='drqp_robot_state',
                        output='screen',
                    ),
                ]
            ),
        ]
    )
