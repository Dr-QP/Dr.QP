from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read_package_file(*parts: str) -> str:
    return PACKAGE_ROOT.joinpath(*parts).read_text(encoding='utf-8')


def test_demo_launch_uses_split_joint_state_publisher_pattern_pr349():
    """PR 349: demo launch should use the jsp/jsp_gui split pattern."""
    source = _read_package_file('launch', 'demo.launch.py')

    has_split_pattern = (
        'joint_state_publisher_gui' in source
        and 'IfCondition(' in source
        and 'UnlessCondition(' in source
    )

    assert "{'use_gui':" not in source
    assert has_split_pattern or 'jsp.launch.py' in source


def test_move_group_launch_passes_robot_description_pr349():
    """PR 349: move_group launch should provide robot_description."""
    source = _read_package_file('launch', 'move_group.launch.py')

    assert "{'robot_description':" in source


def test_moveit_rviz_launch_passes_moveit_robot_descriptions_pr349():
    """PR 349: RViz launch should provide MoveIt robot descriptions."""
    source = _read_package_file('launch', 'moveit_rviz.launch.py')

    assert 'parameters=' in source
    assert "{'robot_description':" in source
    assert "{'robot_description_semantic':" in source


def test_demo_gazebo_launch_includes_robot_description_pr349():
    """PR 349: Gazebo demo moveit params should include robot_description."""
    source = _read_package_file('launch', 'demo_gazebo.launch.py')

    assert "{'robot_description':" in source


def test_package_xml_declares_joint_state_publisher_exec_depend_pr349():
    """PR 349: package.xml should declare joint_state_publisher."""
    package_xml = ET.fromstring(_read_package_file('package.xml'))
    exec_depends = {
        element.text
        for element in package_xml.findall('exec_depend')
        if element.text is not None
    }

    assert 'joint_state_publisher' in exec_depends