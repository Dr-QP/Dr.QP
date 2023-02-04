import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                'drqp_description'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': True,
            'use_ros2_control': True,
        }.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(
        'drqp_gazebo'), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'gui_required': True  # Set "true" to shut down launch script when GUI is terminated
        }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'dr_qp'
                                   '-package_to_model',  # convert mesh paths to work as gazebo models. description package.xml should have proper <export><gazebo_ros gazebo_model_path=""/> tags
                                   '-wait',  # Wait for entity to exist
                                   '-z', '.5',  # initial Z possition
                                   ],
                        output='screen')

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
