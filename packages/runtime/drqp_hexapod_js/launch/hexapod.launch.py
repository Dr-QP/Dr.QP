import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    drqp_hexapod_js_dir = get_package_share_directory("drqp_hexapod_js")

    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drqp_hexapod_js_dir, "launch/web.launch.py")
        )
    )

    drqp_a1_16_driver_dir = get_package_share_directory("drqp_a1_16_driver")
    pose_setter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drqp_a1_16_driver_dir, "launch/pose_setter.launch.py")
        )
    )

    ld = LaunchDescription([web_launch, pose_setter_launch])

    return ld
