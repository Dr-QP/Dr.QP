"""Unit tests for local ROS and Gazebo runtime helpers."""

from __future__ import annotations

# ruff: noqa: E402
import pytest

pytest.skip(
    (
        'Legacy tests under py_packages/drqp_robot_mcp are superseded by '
        'packages/simulation/drqp_robot_mcp/test.'
    ),
    allow_module_level=True,
)

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

from drqp_robot_mcp import runtime


def test_parse_gazebo_pose_info_extracts_entity_poses() -> None:
    """Gazebo protobuf text output is converted into structured entity poses."""
    raw_output = """
header {
  stamp {
    sec: 12
    nsec: 34
  }
}
pose {
  name: "ground_plane"
  id: 1
  position {
    x: 0
    y: 0
    z: 0
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
pose {
  name: "drqp"
  id: 2
  position {
    x: 1.5
    y: -0.25
    z: 0.4
  }
  orientation {
    x: 0
    y: 0
    z: 0.707
    w: 0.707
  }
}
"""

    parsed = runtime.parse_gazebo_pose_info(raw_output)

    assert len(parsed) == 2
    assert parsed[1]['name'] == 'drqp'
    assert parsed[1]['entity_id'] == 2
    assert parsed[1]['pose']['position']['x'] == 1.5
    assert parsed[1]['pose']['orientation']['w'] == 0.707


def test_start_simulation_returns_unavailable_when_gazebo_launch_is_missing(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Start simulation returns a structured response when Gazebo is absent."""
    pid_path = tmp_path / 'sim.pid'
    log_path = tmp_path / 'sim.log'

    monkeypatch.setattr(runtime, '_gazebo_launch_is_available', lambda: False)

    result = runtime.start_simulation(
        workspace_root=tmp_path,
        pid_path=pid_path,
        log_path=log_path,
    )

    assert result['started'] is False
    assert result['available'] is False
    assert 'Gazebo launch is not available' in result['message']


def test_start_simulation_uses_direct_ros2_launch_without_shell(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Simulation launch uses a direct local argv command."""
    captured: dict[str, object] = {}
    pid_path = tmp_path / 'sim.pid'
    log_path = tmp_path / 'sim.log'

    class FakeProcess:
        pid = 4321

    def fake_popen(command, **kwargs):
        captured['command'] = command
        captured['kwargs'] = kwargs
        return FakeProcess()

    monkeypatch.setattr(runtime, '_gazebo_launch_is_available', lambda: True)
    monkeypatch.setattr(runtime.subprocess, 'Popen', fake_popen)

    result = runtime.start_simulation(
        workspace_root=tmp_path,
        pid_path=pid_path,
        log_path=log_path,
    )

    assert result['started'] is True
    assert captured['command'] == [
        'ros2',
        'launch',
        'drqp_gazebo',
        'sim.launch.py',
        'sim_gui:=false',
    ]
    assert captured['kwargs']['cwd'] == tmp_path
    assert captured['kwargs']['start_new_session'] is True
    assert 'shell' not in captured['kwargs']


def test_pid_is_running_returns_false_for_zombie_process(monkeypatch) -> None:
    """Zombie launch processes must be treated as stale, not running."""

    def fake_run(command, **kwargs):
        assert command == ['ps', '-o', 'stat=', '-p', '4321']
        return SimpleNamespace(stdout='Z\n', returncode=0)

    monkeypatch.setattr(runtime.os, 'kill', lambda pid, sig: None)
    monkeypatch.setattr(runtime.subprocess, 'run', fake_run)

    assert runtime._pid_is_running(4321) is False


def test_publish_movement_command_publishes_expected_message(monkeypatch) -> None:
    """Movement commands are published on the robot control topic."""
    published: list[object] = []

    class FakePublisher:
        def get_subscription_count(self) -> int:
            return 1

        def publish(self, message: object) -> None:
            published.append(message)

    class FakeNode:
        def create_publisher(self, msg_type, topic: str, qos_depth: int) -> FakePublisher:
            assert topic == '/robot/movement_command'
            assert qos_depth == 10
            self.msg_type = msg_type
            return FakePublisher()

        def destroy_node(self) -> None:
            return None

    class FakeVector3:
        def __init__(self, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z

    class FakeMovementCommand:
        def __init__(self) -> None:
            self.stride_direction = None
            self.rotation_speed = 0.0
            self.body_translation = None
            self.body_rotation = None
            self.gait_type = ''

    fake_rclpy = ModuleType('rclpy')
    fake_rclpy.spin_once = lambda node, timeout_sec=0.1: None
    fake_node_module = ModuleType('rclpy.node')
    fake_node_module.Node = lambda name: FakeNode()
    fake_geometry_package = ModuleType('geometry_msgs')
    fake_geometry_module = ModuleType('geometry_msgs.msg')
    fake_geometry_module.Vector3 = FakeVector3
    fake_geometry_package.msg = fake_geometry_module
    fake_interfaces_package = ModuleType('drqp_interfaces')
    fake_interfaces_module = ModuleType('drqp_interfaces.msg')
    fake_interfaces_module.MovementCommand = FakeMovementCommand
    fake_interfaces_package.msg = fake_interfaces_module

    monkeypatch.setattr(runtime, '_with_rclpy', lambda operation: operation())
    monkeypatch.setitem(sys.modules, 'rclpy', fake_rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.node', fake_node_module)
    monkeypatch.setitem(sys.modules, 'geometry_msgs', fake_geometry_package)
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', fake_geometry_module)
    monkeypatch.setitem(sys.modules, 'drqp_interfaces', fake_interfaces_package)
    monkeypatch.setitem(sys.modules, 'drqp_interfaces.msg', fake_interfaces_module)

    result = runtime.publish_movement_command(
        stride_direction={'x': 1.0, 'y': -0.5, 'z': 0.0},
        rotation_speed=0.25,
        body_translation={'x': 0.0, 'y': 0.0, 'z': 0.1},
        body_rotation={'x': 0.0, 'y': -0.2, 'z': 0.3},
        gait_type='wave',
    )

    assert result['published'] is True
    assert result['topic'] == '/robot/movement_command'
    assert result['gait_type'] == 'wave'
    assert len(published) == 1
    command = published[0]
    assert command.stride_direction.x == 1.0
    assert command.stride_direction.y == -0.5
    assert command.rotation_speed == 0.25
    assert command.body_translation.z == 0.1
    assert command.body_rotation.y == -0.2
    assert command.body_rotation.z == 0.3
    assert command.gait_type == 'wave'
