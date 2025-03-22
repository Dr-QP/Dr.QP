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

import os

from ament_index_python.packages import get_package_share_directory as get_pkg_dir
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_pkg_dir('drqp_description'), 'launch', 'rsp.launch.py')]
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'use_ros2_control': 'True',
        }.items(),
    )

    gazebo_params_file = os.path.join(get_pkg_dir('drqp_gazebo'), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_pkg_dir('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'gui_required': 'True',  # Set "true" to shut down launch script when GUI is terminated
            # 'world': os.path.join(get_pkg_dir('drqp_gazebo'), 'worlds', 'drqp.world'),
            # 'world': os.path.join(get_pkg_dir('gazebo_ros'), 'worlds', 'empty.world'),
        }.items(),
    )

    entity_name = 'dr_qp'
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really
    # matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            entity_name,
            '-package_to_model',
            '-z',
            '.15',  # initial Z possition
        ],
        output='screen',
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'joint_state_broadcaster',
        ],
        output='screen',
    )

    position_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'position_trajectory_controller',
        ],
        output='screen',
    )
    # velocity_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller',
    #          '--set-state', 'active', 'velocity_controller'],
    #     output='screen'
    # )
    # effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller',
    #          '--set-state', 'active', 'effort_controller'],
    #     output='screen'
    # )
    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[position_trajectory_controller],
                )
            ),
        ]
    )
