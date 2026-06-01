"""Unit tests for local ROS and Gazebo runtime helpers."""

from __future__ import annotations

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from drqp_robot_mcp import runtime


@pytest.fixture(autouse=True)
def shutdown_runtime_session() -> None:
    """Reset the shared runtime singleton between tests."""
    runtime.shutdown_default_ros_runtime()
    yield
    runtime.shutdown_default_ros_runtime()


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
    assert captured['kwargs']['start_new_session'] is True
    assert 'shell' not in captured['kwargs']


def test_get_runtime_directory_prefers_ros_home(monkeypatch) -> None:
    """Runtime files should prefer the ROS_HOME directory when available."""
    monkeypatch.setenv('ROS_HOME', '/tmp/ros-home')

    result = runtime.get_runtime_directory()

    assert result == Path('/tmp/ros-home/drqp_robot_mcp')


def test_get_runtime_directory_falls_back_to_home(monkeypatch) -> None:
    """HOME is used when ROS_HOME is unavailable."""
    monkeypatch.delenv('ROS_HOME', raising=False)
    monkeypatch.setenv('HOME', '/tmp/home')

    result = runtime.get_runtime_directory()

    assert result == Path('/tmp/home/.ros/drqp_robot_mcp')


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

    class FakeString:
        def __init__(self, data: str = '') -> None:
            self.data = data

    class FakeQoSProfile:
        def __init__(self, depth: int) -> None:
            self.depth = depth
            self.durability = None

    class FakeDurabilityPolicy:
        TRANSIENT_LOCAL = 'transient_local'

    class FakePublisher:
        def get_subscription_count(self) -> int:
            return 1

        def publish(self, message: object) -> None:
            published.append(message)

    class FakeNode:
        def __init__(self) -> None:
            self.publishers: dict[str, FakePublisher] = {}
            self.subscriptions: list[tuple[str, object]] = []

        def create_publisher(self, msg_type, topic: str, qos_depth: int) -> FakePublisher:
            assert qos_depth == 10
            self.publishers[topic] = FakePublisher()
            return FakePublisher()

        def create_subscription(self, msg_type, topic: str, callback, qos):
            del msg_type, qos
            self.subscriptions.append((topic, callback))
            return object()

        def destroy_node(self) -> None:
            return None

    class FakeExecutor:
        def add_node(self, node: object) -> None:
            self.node = node

        def remove_node(self, node: object) -> None:
            assert node is self.node

        def spin_once(self, timeout_sec: float = 0.1) -> None:
            return None

        def shutdown(self) -> None:
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

    fake_rclpy = SimpleNamespace(
        ok=lambda: True,
        init=lambda: None,
        shutdown=lambda: None,
        create_node=lambda name: FakeNode(),
    )

    monkeypatch.setattr(
        runtime,
        '_load_ros_dependencies',
        lambda: runtime._RosDependencies(
            rclpy=fake_rclpy,
            executor_factory=FakeExecutor,
            string_message_type=FakeString,
            vector3_message_type=FakeVector3,
            movement_command_type=FakeMovementCommand,
            qos_profile_type=FakeQoSProfile,
            durability_policy=FakeDurabilityPolicy,
        ),
    )
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', ModuleType('sensor_msgs.msg'))
    monkeypatch.setitem(sys.modules, 'rosgraph_msgs.msg', ModuleType('rosgraph_msgs.msg'))
    sys.modules['sensor_msgs.msg'].JointState = object
    sys.modules['rosgraph_msgs.msg'].Clock = object

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
