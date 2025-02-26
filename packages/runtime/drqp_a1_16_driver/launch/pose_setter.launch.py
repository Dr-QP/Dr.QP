import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pose_setter_node = Node(
        package='drqp_a1_16_driver',
        executable='pose_setter',

        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "device_address": "192.168.0.190"
        }],
    )

    ld = LaunchDescription()
    ld.add_action(pose_setter_node)

    return ld
