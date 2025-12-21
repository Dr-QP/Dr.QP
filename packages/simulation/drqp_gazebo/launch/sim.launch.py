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
    ExecuteProcess,
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
from launch_ros.actions import Node
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

    gz_args = 'gz_args:=-r -v 3 empty.sdf'
    gazebo = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'ros_gz_sim',
            'gz_sim.launch.py',
            IfElseSubstitution(
                gui,
                gz_args,
                gz_args + ' --headless-rendering -s',
            ),
        ],
        output='screen',
        sigterm_timeout=gazebo_sigterm_timeout,
        sigkill_timeout=gazebo_sigkill_timeout,
    )

    # Gazebo bridge
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
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

    # Shutdown all nodes when Gazebo exits
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=Shutdown(reason='Gazebo has exited'),
        )
    )

    # Launch them all!
    return LaunchDescription(
        declared_arguments
        + [
            drqp_system,
            gazebo,
            gazebo_bridge,
            gz_spawn_entity,
            shutdown_handler,
        ]
    )
