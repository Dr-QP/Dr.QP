from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path):
    with open(path) as file:
        return yaml.safe_load(file)


def get_moveit_params(pkg_path, use_gazebo):
    drqp_control_pkg = get_package_share_path('drqp_control')
    robot_description_content = ParameterValue(
        Command(
            [
                'xacro ',
                str(drqp_control_pkg / 'urdf' / 'drqp.urdf.xacro'),
                ' use_gazebo:=',
                use_gazebo,
            ]
        ),
        value_type=str,
    )
    srdf_content = (pkg_path / 'config' / 'drqp.srdf').read_text()
    kinematics = load_yaml(pkg_path / 'config' / 'kinematics.yaml')
    joint_limits = load_yaml(pkg_path / 'config' / 'joint_limits.yaml')
    ompl = load_yaml(pkg_path / 'config' / 'ompl_planning.yaml')
    controllers = load_yaml(pkg_path / 'config' / 'moveit_controllers.yaml')
    move_group = load_yaml(pkg_path / 'config' / 'move_group.yaml')

    return [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': srdf_content},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        ompl,
        controllers,
        move_group,
    ]
