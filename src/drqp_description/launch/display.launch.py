from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    drqp_description_path = get_package_share_path('drqp_description')
    default_rviz_config_path = drqp_description_path / 'rviz/drqp_description.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    external_urdf_loc_arg = DeclareLaunchArgument(name='external_urdf_loc', default_value="",
                                                  description='Absolute path additional urdf file to include in the model')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true')
    rviz_frame_arg = DeclareLaunchArgument(name='rviz_frame', default_value='dr_qp/base_link',
                                           description='Base model frame in rviz')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
                                     description='Flag to enable rviz')
    rvizconfig_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                           description='Absolute path to rviz config file')

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', LaunchConfiguration('rvizconfig'), '-f', LaunchConfiguration('rviz_frame')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        rvizconfig_arg,
        rviz_frame_arg,
        external_urdf_loc_arg,
        use_sim_time_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
