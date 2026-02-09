# Copyright (c) 2017-present Anton Matosov
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
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
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

    # Initialize Arguments
    gui = LaunchConfiguration('gui')

    gazebo_sigterm_timeout = LaunchConfiguration('gazebo_sigterm_timeout')
    gazebo_sigkill_timeout = LaunchConfiguration('gazebo_sigkill_timeout')
    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            '-v',
            '3',
            'empty.sdf',
            IfElseSubstitution(gui, '', ' --headless-rendering -s'),
        ],
        shell=True,
        output='screen',
        sigterm_timeout=gazebo_sigterm_timeout,
        sigkill_timeout=gazebo_sigkill_timeout,
    )
    on_gazebo_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[Shutdown(reason='Gazebo exited')],
        )
    )

    container_name = 'drqp_gazebo_container'
    gz_args = '-r -v 3 empty.sdf'
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
                gui,
                gz_args,
                gz_args + ' --headless-rendering -s',
            ),
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'ros_gz_sim.launch.py',
                ]
            )
        ),
        launch_arguments={
            'bridge_name': 'drqp_gazebo_bridge',
            'config_file': PathJoinSubstitution(
                [
                    FindPackageShare('drqp_gazebo'),
                    'config',
                    'drqp_gazebo_bridge.yml',
                ]
            ),
            'use_composition': 'false',  # Composition throws an exception
            'create_own_container': 'true',
            'container_name': container_name,
        }.items(),
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
            'true',
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
            'show_rviz': gui,
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
                    on_gazebo_shutdown,
                    gazebo_bridge,
                    gz_spawn_entity,
                ]
            )
        ]
    )
