import os

from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    share_directory = get_package_share_directory("drqp_hexapod_js")

    website = Node(
        executable="yarn",
        output="screen",
        arguments=["start-client-prod"],
        cwd=os.path.join(share_directory, "dist", "packages", "hexapod"),
    )

    server = Node(
        executable="yarn",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["start-kill-server"],
        cwd=os.path.join(share_directory, "dist", "packages", "hexapod"),
    )
    ld = LaunchDescription()
    ld.add_action(server)
    ld.add_action(website)

    return ld
