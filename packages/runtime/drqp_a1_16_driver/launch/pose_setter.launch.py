from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pose_setter_node = Node(
        package="drqp_a1_16_driver",
        executable="pose_setter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "device_address": "192.168.0.190"}],
    )

    ld = LaunchDescription()
    ld.add_action(pose_setter_node)

    return ld
