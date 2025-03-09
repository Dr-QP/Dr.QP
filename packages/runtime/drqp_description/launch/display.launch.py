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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_path = get_package_share_path('drqp_description')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', LaunchConfiguration('rviz_config'),
            '-f', LaunchConfiguration('rviz_frame')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui',
                              default_value='true',
                              choices=['true', 'false'],
                              description='Enable joint_state_publisher_gui'),

        DeclareLaunchArgument(name='external_urdf_loc',
                              default_value='',
                              description='Absolute path additional urdf file to include in the model'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(name='rviz',
                              default_value='true',
                              choices=['true', 'false'],
                              description='Enable rviz'),
        DeclareLaunchArgument(name='rviz_frame',
                              default_value='dr_qp/base_link',
                              description='Base model frame in rviz'),
        DeclareLaunchArgument(name='rviz_config',
                              default_value=str(
                                  pkg_share_path / 'rviz' / 'drqp_description.rviz'),
                              description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
