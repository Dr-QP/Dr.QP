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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_sdf',
            default_value='empty.sdf',
            description='Gazebo world SDF file to load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_gui',
            default_value='true',
            description='Start Gazebo GUI Client automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_sigterm_timeout',
            default_value='20.0',
            description='Seconds to wait after SIGINT before sending SIGTERM to Gazebo.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_sigkill_timeout',
            default_value='40.0',
            description='Seconds to wait after SIGTERM before sending SIGKILL to Gazebo.',
        )
    )
    for argument_name in ('robot_x', 'robot_y', 'robot_z', 'robot_roll', 'robot_pitch', 'robot_yaw'):
        declared_arguments.append(
            DeclareLaunchArgument(
                argument_name,
                default_value='0.0',
                description=f'Robot spawn {argument_name.removeprefix("robot_")} pose component.',
            )
        )

    # Initialize Arguments
    sim_gui = LaunchConfiguration('sim_gui')
    world_sdf = LaunchConfiguration('world_sdf')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_roll = LaunchConfiguration('robot_roll')
    robot_pitch = LaunchConfiguration('robot_pitch')
    robot_yaw = LaunchConfiguration('robot_yaw')
    container_name = 'drqp_gazebo_container'
    gz_args = ['-r -v 3 ', world_sdf]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ]
            )
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                sim_gui,
                gz_args,
                ['-r -v 3 ', world_sdf, ' --headless-rendering -s'],
            ),
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gazebo_bridge = RosGzBridge(
        bridge_name='drqp_gazebo_bridge',
        config_file=PathJoinSubstitution(
            [
                FindPackageShare('drqp_gazebo'),
                'config',
                'drqp_gazebo_bridge.yml',
            ]
        ),
        use_composition='false',  # Composition throws an exception
        create_own_container='true',
        container_name=container_name,
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'drqp',
            '-allow_renaming',
            'false',
            '-x',
            robot_x,
            '-y',
            robot_y,
            '-z',
            robot_z,
            '-R',
            robot_roll,
            '-P',
            robot_pitch,
            '-Y',
            robot_yaw,
        ],
    )

    drqp_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('drqp_brain'),
                    'launch',
                    'bringup.launch.py',
                ]
            )
        ),
        launch_arguments={
            'use_gazebo': 'true',
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            GroupAction(
                [
                    SetParameter('use_sim_time', value=True),
                    drqp_system,
                    gazebo,
                    gazebo_bridge,
                    gz_spawn_entity,
                ]
            )
        ]
    )
