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

"""
MoveIt 2 move_group launch file for the Dr.QP hexapod robot.

Starts the move_group node with robot_description, SRDF, kinematics,
OMPL planning pipeline and controller configuration loaded from this
package's config/ directory.
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = str(Path(__file__).resolve().parent)
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)

    from moveit_launch_utils import get_moveit_params

    pkg = get_package_share_path('drqp_moveit')
    use_gazebo = LaunchConfiguration('use_gazebo')
    hardware_device_address = LaunchConfiguration('hardware_device_address')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=get_moveit_params(
            pkg, use_gazebo=use_gazebo, hardware_device_address=hardware_device_address
        )
        + [{'use_sim_time': use_gazebo}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_gazebo',
                default_value='false',
                choices=['true', 'false'],
                description='Build robot_description with Gazebo ros2_control settings',
            ),
            DeclareLaunchArgument(
                'hardware_device_address',
                default_value='/dev/ttySC0',
                description=(
                    'Hardware device address for ros2_control in URDF. '
                    'Use "mock_servo" for fake hardware.'
                ),
            ),
            move_group_node,
        ]
    )
