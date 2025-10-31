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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_path = get_package_share_path('drqp_description')

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            LaunchConfiguration('rviz_config'),
            '-f',
            LaunchConfiguration('rviz_frame'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time', default_value='true', description='Use sim time if true'
            ),
            DeclareLaunchArgument(
                name='rviz_frame', default_value='ground', description='Base model frame in rviz'
            ),
            DeclareLaunchArgument(
                name='rviz_config',
                default_value=str(pkg_share_path / 'rviz' / 'drqp_description.rviz'),
                description='Absolute path to rviz config file',
            ),
            rviz_node,
        ]
    )
