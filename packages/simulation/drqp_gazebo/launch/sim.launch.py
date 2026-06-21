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

from ament_index_python.packages import get_package_share_directory
from drqp_brain.instance_guard import make_launch_instance_guard
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AllSubstitution,
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def _resolve_world_sdf(world_sdf):
    if '/' in world_sdf:
        return world_sdf

    world_file = world_sdf if world_sdf.endswith('.sdf') else f'{world_sdf}.sdf'
    package_world_path = get_package_share_directory('drqp_gazebo') + f'/worlds/{world_file}'
    try:
        with open(package_world_path, encoding='utf-8'):
            return package_world_path
    except FileNotFoundError:
        return world_sdf


def _start_gazebo(context, sim_gui, world_sdf):
    resolved_world_sdf = _resolve_world_sdf(context.perform_substitution(world_sdf))
    resolved_world_sdf_substitution = TextSubstitution(text=resolved_world_sdf)
    gz_args = [TextSubstitution(text='-r -v 3 '), resolved_world_sdf_substitution]
    # Keep server/headless flags before the world path so `gz sim` reliably
    # parses them as options instead of trailing arguments after the SDF file.
    headless_gz_args = [
        TextSubstitution(text='-r -v 3 --headless-rendering -s '),
        resolved_world_sdf_substitution,
    ]
    return [
        IncludeLaunchDescription(
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
                    headless_gz_args,
                ),
                'on_exit_shutdown': 'true',
            }.items(),
        )
    ]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_sdf',
            default_value='empty.sdf',
            description=(
                'Gazebo world SDF file to load. Accepts a path, a Gazebo built-in '
                'world, or a world name from drqp_gazebo/worlds.'
            ),
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
    for argument_name, default_value in (
        ('robot_x', '0.0'),
        ('robot_y', '0.0'),
        ('robot_z', '0.03'),
        ('robot_roll', '0.0'),
        ('robot_pitch', '0.0'),
        ('robot_yaw', '0.0'),
    ):
        declared_arguments.append(
            DeclareLaunchArgument(
                argument_name,
                default_value=default_value,
                description=f'Robot spawn {argument_name.removeprefix("robot_")} pose component.',
            )
        )
    declared_arguments.append(
        DeclareLaunchArgument(
            'follow_camera',
            default_value='true',
            choices=['true', 'false'],
            description='Ask Gazebo GUI to follow the spawned drqp model.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'follow_camera_delay',
            default_value='5.0',
            description='Seconds to wait after robot spawn before sending the Gazebo GUI follow command.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_keyboard_control',
            default_value='false',
            choices=['true', 'false'],
            description='Load the simulation GUI keyboard control node.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_robot',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Spawn the robot and bring up its control stack. Set to false to '
                'launch only Gazebo and the world (e.g. to test the world fixture '
                'in isolation).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_partition',
            default_value=[
                EnvironmentVariable('HOSTNAME', default_value='unknown-host'),
                ':',
                EnvironmentVariable('USER', default_value='unknown-user'),
                '-domain-',
                EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
            ],
            description=(
                'Gazebo transport partition. Defaults to '
                '<HOSTNAME>:<USERNAME>-domain-<ROS_DOMAIN_ID>.'
            ),
        )
    )

    # Initialize Arguments
    sim_gui = LaunchConfiguration('sim_gui')
    world_sdf = LaunchConfiguration('world_sdf')
    follow_camera = LaunchConfiguration('follow_camera')
    follow_camera_delay = LaunchConfiguration('follow_camera_delay')
    load_keyboard_control = LaunchConfiguration('load_keyboard_control')
    spawn_robot = LaunchConfiguration('spawn_robot')
    gz_partition = LaunchConfiguration('gz_partition')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_roll = LaunchConfiguration('robot_roll')
    robot_pitch = LaunchConfiguration('robot_pitch')
    robot_yaw = LaunchConfiguration('robot_yaw')
    container_name = 'drqp_gazebo_container'
    robot_entity_name = 'drqp'
    gazebo = OpaqueFunction(function=_start_gazebo, args=[sim_gui, world_sdf])
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
            robot_entity_name,
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
    keyboard_control = Node(
        package='drqp_keyboard_control',
        executable='drqp_keyboard_control',
        output='screen',
        condition=IfCondition(load_keyboard_control),
    )

    def on_spawn_exit(event, _context):
        # gz_spawn_entity exits 0 even when the spawn itself failed inside Gazebo,
        # but a non-zero returncode means the process never ran to begin with
        # (e.g. malformed URDF). Skip the follow command in that case instead of
        # letting `gz service` time out against a nonexistent entity.
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        'Skipping camera follow: entity spawn process exited with '
                        f'code {event.returncode}.'
                    )
                )
            ]
        return [
            TimerAction(
                period=follow_camera_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'gz',
                            'service',
                            '-s',
                            '/gui/follow',
                            '--reqtype',
                            'gz.msgs.StringMsg',
                            '--reptype',
                            'gz.msgs.Boolean',
                            '--timeout',
                            '5000',
                            '--req',
                            f'data: "{robot_entity_name}"',
                        ],
                        output='screen',
                    ),
                ],
            ),
        ]

    follow_camera_command = GroupAction(
        condition=IfCondition(AllSubstitution(sim_gui, follow_camera)),
        actions=[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=on_spawn_exit,
                )
            )
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

    balance_challenge_disturbance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('drqp_gazebo'),
                    'launch',
                    'balance_challenge_disturbance.launch.py',
                ]
            )
        ),
        condition=IfCondition(PythonExpression(["'balance_challenge' in '", world_sdf, "'"])),
    )

    return LaunchDescription(
        [make_launch_instance_guard('drqp_gazebo_sim')]
        + declared_arguments
        + [
            SetEnvironmentVariable('GZ_PARTITION', gz_partition),
            gazebo,
            GroupAction(
                condition=IfCondition(spawn_robot),
                actions=[
                    SetParameter('use_sim_time', value=True),
                    drqp_system,
                    gazebo_bridge,
                    gz_spawn_entity,
                    follow_camera_command,
                    keyboard_control,
                    balance_challenge_disturbance,
                ],
            ),
        ]
    )
