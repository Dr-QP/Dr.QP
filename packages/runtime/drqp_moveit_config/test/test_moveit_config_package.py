from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import ModuleType
from typing import Any, Iterable
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / 'config'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'
DRQP_CONTROL_ROOT = PACKAGE_ROOT.parent / 'drqp_control'
LEG_GROUPS = [
    'left_front_leg',
    'right_front_leg',
    'left_middle_leg',
    'right_middle_leg',
    'left_back_leg',
    'right_back_leg',
]
EXPECTED_GROUPS = LEG_GROUPS + ['whole_body']


def _load_yaml(path: Path) -> dict[str, Any]:
    with open(path, encoding='utf-8') as file_obj:
        return yaml.safe_load(file_obj)


def _load_launch_module(module_name: str, file_name: str) -> ModuleType:
    spec = spec_from_file_location(module_name, LAUNCH_DIR / file_name)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _generate_launch_description(
    monkeypatch: pytest.MonkeyPatch, module_name: str, file_name: str
):
    module = _load_launch_module(module_name, file_name)

    def fake_get_package_share_path(package_name: str) -> Path:
        if package_name == 'drqp_moveit_config':
            return PACKAGE_ROOT
        if package_name == 'drqp_control':
            return DRQP_CONTROL_ROOT
        raise AssertionError(f'unexpected package lookup: {package_name}')

    monkeypatch.setattr(module, 'get_package_share_path', fake_get_package_share_path)
    return module.generate_launch_description()


def _iter_entities(entities: Iterable[Any]) -> Iterable[Any]:
    for entity in entities:
        yield entity
        if isinstance(entity, GroupAction):
            yield from _iter_entities(vars(entity)['_GroupAction__actions'])


def _argument_map(launch_description) -> dict[str, DeclareLaunchArgument]:
    return {
        entity.name: entity
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _node_map(launch_description) -> dict[tuple[str, str], Node]:
    return {
        (entity.node_package, entity.node_executable): entity
        for entity in _iter_entities(launch_description.entities)
        if isinstance(entity, Node)
    }


def _normalize_substitution(value: Any) -> str:
    if isinstance(value, (list, tuple)):
        return ''.join(_normalize_substitution(part) for part in value)
    text = getattr(value, 'text', None)
    if text is not None:
        return text
    return str(value)


def _normalize_parameter_value(value: Any) -> Any:
    if isinstance(value, (bool, int, float, dict, ParameterValue)):
        return value
    if isinstance(value, (list, tuple)):
        return _normalize_substitution(value)
    return value


def _normalized_xml_text(value: str) -> str:
    return value.replace('\\"', '"').strip('"\n')


def _node_parameters(node: Node) -> dict[str, Any]:
    normalized: dict[str, Any] = {}
    for parameter_dict in vars(node)['_Node__parameters']:
        for key, value in parameter_dict.items():
            normalized[_normalize_substitution(key)] = _normalize_parameter_value(value)
    return normalized


def _has_parameter_prefix(parameters: dict[str, Any], prefix: str) -> bool:
    return any(key.startswith(prefix) for key in parameters)


def _node_arguments(node: Node) -> list[str]:
    return list(vars(node)['_Node__arguments'])


def _set_parameter_dict(action: SetParameter) -> dict[str, Any]:
    return {
        _normalize_substitution(key): value
        for key, value in vars(action)['_SetParameter__param_dict'].items()
    }


def _command_text(parameter: ParameterValue) -> str:
    value = parameter.value
    if isinstance(value, list) and len(value) == 1:
        value = value[0]
    assert isinstance(value, Command)
    return _normalize_substitution(value.command)


def test_package_manifest_declares_required_dependencies():
    package_xml = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()

    depends = {element.text for element in package_xml.findall('depend') if element.text}
    exec_depends = {element.text for element in package_xml.findall('exec_depend') if element.text}
    test_depends = {element.text for element in package_xml.findall('test_depend') if element.text}

    assert {
        'drqp_control',
        'moveit_ros_move_group',
        'moveit_kinematics',
        'moveit_planners_ompl',
    }.issubset(depends)
    assert {
        'drqp_brain',
        'drqp_gazebo',
        'joint_state_publisher',
        'joint_state_publisher_gui',
        'robot_state_publisher',
        'rviz2',
        'xacro',
    }.issubset(exec_depends)
    assert {'ament_cmake_pytest', 'ament_lint_auto', 'drqp_lint_common'} == test_depends


def test_moveit_config_yaml_files_parse_with_expected_shape():
    kinematics = _load_yaml(CONFIG_DIR / 'kinematics.yaml')
    joint_limits = _load_yaml(CONFIG_DIR / 'joint_limits.yaml')
    move_group = _load_yaml(CONFIG_DIR / 'move_group.yaml')
    controllers = _load_yaml(CONFIG_DIR / 'moveit_controllers.yaml')
    ompl = _load_yaml(CONFIG_DIR / 'ompl_planning.yaml')

    assert set(kinematics) == set(EXPECTED_GROUPS)
    assert all(
        config['kinematics_solver'] == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
        for config in kinematics.values()
    )

    assert len(joint_limits['joint_limits']) == 18
    assert all(
        limit['has_velocity_limits'] and limit['max_velocity'] > 0
        for limit in joint_limits['joint_limits'].values()
    )

    planning_scene_monitor = move_group['move_group']['planning_scene_monitor_options']
    assert planning_scene_monitor['robot_description'] == 'robot_description'
    assert planning_scene_monitor['joint_state_topic'] == '/joint_states'
    assert move_group['move_group']['sensors'] == []

    controller_names = controllers['moveit_simple_controller_manager']['controller_names']
    assert controller_names == ['joint_trajectory_controller']
    assert len(controllers['joint_trajectory_controller']['joints']) == 18

    assert ompl['ompl']['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert ompl['ompl']['whole_body']['default_planner_config'] == 'RRTConnectkConfigDefault'
    assert 'RRTConnectkConfigDefault' in ompl['ompl']['planner_configs']


def test_srdf_is_well_formed_and_describes_robot_groups():
    root = ET.parse(CONFIG_DIR / 'drqp.srdf').getroot()

    groups = {group.attrib['name']: group for group in root.findall('group')}
    end_effectors = root.findall('end_effector')
    disable_collisions = root.findall('disable_collisions')

    assert root.attrib['name'] == 'drqp'
    assert set(groups) == set(EXPECTED_GROUPS)
    assert len(end_effectors) == 6

    whole_body_children = [child.attrib['name'] for child in groups['whole_body'].findall('group')]
    assert whole_body_children == LEG_GROUPS

    whole_body_home = next(
        group_state
        for group_state in root.findall('group_state')
        if group_state.attrib['group'] == 'whole_body'
    )
    assert len(whole_body_home.findall('joint')) == 18

    assert any(
        collision.attrib['reason'] == 'Never'
        and 'left_middle' in collision.attrib['link1']
        and 'right_middle' in collision.attrib['link2']
        for collision in disable_collisions
    )


def test_demo_launch_description_contains_expected_nodes(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_description = _generate_launch_description(
        monkeypatch,
        'drqp_moveit_demo_launch',
        'demo.launch.py',
    )

    arguments = _argument_map(launch_description)
    assert set(arguments) == {'show_rviz', 'gui'}
    assert arguments['show_rviz'].choices == ['true', 'false']
    assert arguments['gui'].choices == ['true', 'false']

    nodes = _node_map(launch_description)
    robot_state_publisher = nodes[('robot_state_publisher', 'robot_state_publisher')]
    joint_state_publisher = nodes[('joint_state_publisher', 'joint_state_publisher')]
    joint_state_publisher_gui = nodes[('joint_state_publisher_gui', 'joint_state_publisher_gui')]
    move_group = nodes[('moveit_ros_move_group', 'move_group')]
    rviz = nodes[('rviz2', 'rviz2')]

    assert isinstance(joint_state_publisher.condition, UnlessCondition)
    assert isinstance(joint_state_publisher_gui.condition, IfCondition)
    assert isinstance(rviz.condition, IfCondition)

    robot_parameters = _node_parameters(robot_state_publisher)
    assert robot_parameters['use_sim_time'] is False
    assert isinstance(robot_parameters['robot_description'], ParameterValue)
    assert 'drqp.urdf.xacro' in _command_text(robot_parameters['robot_description'])

    move_group_parameters = _node_parameters(move_group)
    assert move_group_parameters['use_sim_time'] is False
    assert isinstance(move_group_parameters['robot_description'], ParameterValue)
    assert '<robot name="drqp">' in _normalized_xml_text(
        move_group_parameters['robot_description_semantic']
    )
    assert _has_parameter_prefix(
        move_group_parameters,
        'robot_description_kinematics.whole_body',
    )
    assert _has_parameter_prefix(
        move_group_parameters,
        'robot_description_planning.joint_limits.drqp/left_front_coxa',
    )

    assert _node_arguments(rviz)[-1].endswith('config/moveit.rviz')
    rviz_parameters = _node_parameters(rviz)
    assert rviz_parameters['use_sim_time'] is False
    assert isinstance(rviz_parameters['robot_description'], ParameterValue)


def test_move_group_launch_description_exposes_move_group_node(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_description = _generate_launch_description(
        monkeypatch,
        'drqp_moveit_move_group_launch',
        'move_group.launch.py',
    )

    arguments = _argument_map(launch_description)
    assert set(arguments) == {'use_sim_time'}

    nodes = _node_map(launch_description)
    move_group = nodes[('moveit_ros_move_group', 'move_group')]
    parameters = _node_parameters(move_group)

    assert isinstance(parameters['robot_description'], ParameterValue)
    assert 'drqp.urdf.xacro' in _command_text(parameters['robot_description'])
    assert '<robot name="drqp">' in _normalized_xml_text(parameters['robot_description_semantic'])
    assert _has_parameter_prefix(parameters, 'robot_description_kinematics.whole_body')
    assert _has_parameter_prefix(
        parameters,
        'robot_description_planning.joint_limits.drqp/left_front_coxa',
    )
    assert 'use_sim_time' in parameters


def test_moveit_rviz_launch_description_exposes_rviz_node(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_description = _generate_launch_description(
        monkeypatch,
        'drqp_moveit_rviz_launch',
        'moveit_rviz.launch.py',
    )

    arguments = _argument_map(launch_description)
    assert set(arguments) == {'use_rviz'}

    nodes = _node_map(launch_description)
    rviz = nodes[('rviz2', 'rviz2')]
    parameters = _node_parameters(rviz)

    assert isinstance(rviz.condition, IfCondition)
    assert _node_arguments(rviz) == ['-d', str(CONFIG_DIR / 'moveit.rviz')]
    assert isinstance(parameters['robot_description'], ParameterValue)
    assert '<robot name="drqp">' in _normalized_xml_text(parameters['robot_description_semantic'])
    assert _has_parameter_prefix(parameters, 'robot_description_kinematics.whole_body')
    assert _has_parameter_prefix(
        parameters,
        'robot_description_planning.joint_limits.drqp/left_front_coxa',
    )


def test_demo_gazebo_launch_description_groups_simulation_actions(
    monkeypatch: pytest.MonkeyPatch,
):
    launch_description = _generate_launch_description(
        monkeypatch,
        'drqp_moveit_demo_gazebo_launch',
        'demo_gazebo.launch.py',
    )

    arguments = _argument_map(launch_description)
    assert set(arguments) == {'show_rviz', 'sim_gui'}

    group_action = next(
        entity for entity in launch_description.entities if isinstance(entity, GroupAction)
    )
    group_entities = list(_iter_entities([group_action]))[1:]

    set_parameter = next(entity for entity in group_entities if isinstance(entity, SetParameter))
    include_launch = next(
        entity for entity in group_entities if isinstance(entity, IncludeLaunchDescription)
    )
    move_group = next(
        entity
        for entity in group_entities
        if isinstance(entity, Node) and entity.node_package == 'moveit_ros_move_group'
    )
    rviz = next(
        entity
        for entity in group_entities
        if isinstance(entity, Node) and entity.node_package == 'rviz2'
    )

    assert _set_parameter_dict(set_parameter) == {'use_sim_time': True}
    include_launch_arguments = dict(include_launch.launch_arguments)
    assert set(include_launch_arguments) == {'sim_gui'}
    assert isinstance(include_launch_arguments['sim_gui'], LaunchConfiguration)
    assert _normalize_substitution(include_launch_arguments['sim_gui'].variable_name) == 'sim_gui'

    move_group_parameters = _node_parameters(move_group)
    assert move_group_parameters['use_sim_time'] is True
    assert isinstance(move_group_parameters['robot_description'], ParameterValue)
    assert 'use_gazebo:=true' in _command_text(move_group_parameters['robot_description'])

    rviz_parameters = _node_parameters(rviz)
    assert isinstance(rviz.condition, IfCondition)
    assert rviz_parameters['use_sim_time'] is True
    assert isinstance(rviz_parameters['robot_description'], ParameterValue)
