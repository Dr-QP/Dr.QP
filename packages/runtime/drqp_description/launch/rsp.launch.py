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

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share_path = get_package_share_path('drqp_description')

    use_gazebo = LaunchConfiguration('use_gazebo')
    hardware_device_address = LaunchConfiguration('hardware_device_address')

    # Process the URDF file

    # robot_description_config
    robot_description_config = ParameterValue(
        Command(
            [
                'xacro ',
                str(pkg_share_path / 'urdf' / 'dr_qp.urdf.xacro'),
                ' hardware_device_address:=',
                hardware_device_address,
                ' use_gazebo:=',
                use_gazebo,
            ]
        ),
        value_type=str,
    )

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_gazebo}],
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_gazebo', default_value='false', description='Use gazebo if true'
            ),
            DeclareLaunchArgument(
                'hardware_device_address',
                default_value='/dev/ttySC0',
                description='Hardware device address for ros2_control',
            ),
            node_robot_state_publisher,
        ]
    )
