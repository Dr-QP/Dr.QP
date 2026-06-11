"""Unit tests for local ROS and Gazebo runtime helpers."""

from __future__ import annotations

import builtins
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

from drqp_robot_mcp import runtime
import pytest


@pytest.fixture(autouse=True)
def shutdown_runtime_session() -> None:
    """Reset the shared runtime singleton between tests."""
    runtime.shutdown_default_ros_runtime()
    yield
    runtime.shutdown_default_ros_runtime()


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


def test_get_robot_state_uses_bridged_odometry_instead_of_gz_cli(monkeypatch) -> None:
    """Robot pose should come from the ROS bridge, not a Gazebo CLI subprocess."""

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
            del message

    class FakeNode:
        def __init__(self) -> None:
            self.subscriptions: dict[str, object] = {}

        def create_publisher(self, msg_type, topic: str, qos_depth: int) -> FakePublisher:
            del msg_type, topic
            assert qos_depth == 10
            return FakePublisher()

        def create_subscription(self, msg_type, topic: str, callback, qos):
            del msg_type, qos
            self.subscriptions[topic] = callback
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

    fake_node = FakeNode()
    fake_rclpy = SimpleNamespace(
        ok=lambda: True,
        init=lambda: None,
        shutdown=lambda: None,
        create_node=lambda name: fake_node,
    )

    monkeypatch.setattr(
        runtime,
        '_load_ros_dependencies',
        lambda: runtime._RosDependencies(
            rclpy=fake_rclpy,
            executor_factory=FakeExecutor,
            odometry_message_type=object,
            string_message_type=FakeString,
            vector3_message_type=FakeVector3,
            movement_command_type=FakeMovementCommand,
            qos_profile_type=FakeQoSProfile,
            durability_policy=FakeDurabilityPolicy,
        ),
    )
    monkeypatch.setattr(
        runtime.subprocess,
        'run',
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError('get_robot_state should not shell out to gz topic')
        ),
    )
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', ModuleType('sensor_msgs.msg'))
    monkeypatch.setitem(sys.modules, 'rosgraph_msgs.msg', ModuleType('rosgraph_msgs.msg'))
    sys.modules['sensor_msgs.msg'].JointState = object
    sys.modules['rosgraph_msgs.msg'].Clock = object

    runtime_session = runtime.RosRuntimeSession()
    runtime_session._ensure_started()

    fake_node.subscriptions['/robot_state'](SimpleNamespace(data='torque_on'))
    fake_node.subscriptions['/clock'](
        SimpleNamespace(clock=SimpleNamespace(sec=12, nanosec=500_000_000))
    )
    fake_node.subscriptions['/odom'](
        SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.25, y=-0.5, z=0.3),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.707, w=0.707),
                )
            )
        )
    )

    result = runtime_session.get_robot_state(
        world_name='empty',
        robot_name='drqp',
        timeout_sec=1.0,
    )

    assert result['available'] is True
    assert result['simulation_running'] is True
    assert result['world_name'] == 'empty'
    assert result['simulation_time_sec'] == pytest.approx(12.5)
    assert result['robot_pose']['position']['x'] == pytest.approx(1.25)
    assert result['robot_pose']['position']['y'] == pytest.approx(-0.5)
    assert result['robot_pose']['orientation']['z'] == pytest.approx(0.707)
    assert result['robot_pose']['orientation']['w'] == pytest.approx(0.707)


def test_get_world_state_uses_gazebo_transport_instead_of_gz_cli(monkeypatch) -> None:
    """World state should come from Gazebo Transport, not a CLI subprocess."""

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
            del message

    class FakeNode:
        def __init__(self) -> None:
            self.subscriptions: dict[str, object] = {}

        def create_publisher(self, msg_type, topic: str, qos_depth: int) -> FakePublisher:
            del msg_type, topic
            assert qos_depth == 10
            return FakePublisher()

        def create_subscription(self, msg_type, topic: str, callback, qos):
            del msg_type, qos
            self.subscriptions[topic] = callback
            return object()

        def destroy_node(self) -> None:
            return None

    class FakeGazeboNode:
        def __init__(self) -> None:
            self.subscriptions: dict[str, object] = {}

        def subscribe(self, msg_type, topic: str, callback) -> bool:
            del msg_type
            self.subscriptions[topic] = callback
            return True

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

    fake_node = FakeNode()
    fake_gazebo_node = FakeGazeboNode()
    fake_rclpy = SimpleNamespace(
        ok=lambda: True,
        init=lambda: None,
        shutdown=lambda: None,
        create_node=lambda name: fake_node,
    )

    monkeypatch.setattr(
        runtime,
        '_load_ros_dependencies',
        lambda: runtime._RosDependencies(
            rclpy=fake_rclpy,
            executor_factory=FakeExecutor,
            odometry_message_type=object,
            string_message_type=FakeString,
            vector3_message_type=FakeVector3,
            movement_command_type=FakeMovementCommand,
            qos_profile_type=FakeQoSProfile,
            durability_policy=FakeDurabilityPolicy,
        ),
    )
    monkeypatch.setattr(
        runtime,
        '_load_gazebo_transport_dependencies',
        lambda: runtime._GazeboTransportDependencies(
            node_factory=lambda: fake_gazebo_node,
            pose_v_message_type=object,
        ),
    )
    monkeypatch.setattr(
        runtime.subprocess,
        'run',
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError('get_world_state should not shell out to gz topic')
        ),
    )
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', ModuleType('sensor_msgs.msg'))
    monkeypatch.setitem(sys.modules, 'rosgraph_msgs.msg', ModuleType('rosgraph_msgs.msg'))
    sys.modules['sensor_msgs.msg'].JointState = object
    sys.modules['rosgraph_msgs.msg'].Clock = object

    runtime_session = runtime.RosRuntimeSession()
    runtime_session._ensure_started()
    runtime_session._ensure_world_state_subscription('empty')

    fake_gazebo_node.subscriptions['/world/empty/pose/info'](
        SimpleNamespace(
            HasField=lambda field: field == 'header',
            header=SimpleNamespace(stamp=SimpleNamespace(sec=8, nsec=250_000_000)),
            pose=[
                SimpleNamespace(
                    name='drqp',
                    id=2,
                    position=SimpleNamespace(x=1.0, y=-0.5, z=0.25),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ],
        )
    )

    result = runtime_session.get_world_state(world_name='empty', timeout_sec=1.0)

    assert result == {
        'available': True,
        'world_name': 'empty',
        'simulation_time_sec': pytest.approx(8.25),
        'entity_count': 1,
        'entities': [
            {
                'name': 'drqp',
                'entity_id': 2,
                'pose': {
                    'position': {'x': 1.0, 'y': -0.5, 'z': 0.25},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
            }
        ],
        'source': 'gazebo',
        'note': None,
    }


def test_get_world_state_handles_missing_gazebo_transport_bindings(monkeypatch) -> None:
    """Missing Gazebo transport Python bindings should return a degraded snapshot."""
    runtime_session = runtime.RosRuntimeSession()
    monkeypatch.setattr(runtime_session, '_ensure_started', lambda: None)

    monkeypatch.setattr(
        runtime,
        '_load_gazebo_transport_dependencies',
        lambda: (_ for _ in ()).throw(ImportError('No module named gz.transport13')),
    )

    result = runtime_session.get_world_state(world_name='empty', timeout_sec=0.1)

    assert result['available'] is False
    assert result['world_name'] == 'empty'
    assert result['entity_count'] == 0
    assert result['entities'] == []
    assert result['source'] == 'gazebo'
    assert result['note'] is not None
    assert 'Gazebo transport bindings are unavailable' in result['note']


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
            odometry_message_type=object,
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


def test_close_reports_teardown_failures_and_continues_cleanup(
    monkeypatch,
) -> None:
    """Runtime close should log teardown failures while finishing cleanup."""
    warning_messages: list[str] = []

    class FakeNode:
        def destroy_node(self) -> None:
            raise RuntimeError('destroy failed')

    class FakeExecutor:
        def remove_node(self, node: object) -> None:
            del node
            raise RuntimeError('remove failed')

        def shutdown(self) -> None:
            raise RuntimeError('shutdown failed')

    class FakeRclpy:
        def __init__(self) -> None:
            self.shutdown_called = False

        def ok(self) -> bool:
            return True

        def shutdown(self) -> None:
            self.shutdown_called = True

    fake_rclpy = FakeRclpy()
    runtime_session = runtime.RosRuntimeSession()
    runtime_session._started = runtime._StartedRosRuntime(
        dependencies=runtime._RosDependencies(
            rclpy=fake_rclpy,
            executor_factory=None,
            odometry_message_type=None,
            string_message_type=None,
            vector3_message_type=None,
            movement_command_type=None,
            qos_profile_type=None,
            durability_policy=None,
        ),
        node=FakeNode(),
        executor=FakeExecutor(),
        event_publisher=None,
        movement_command_publisher=None,
        stop_event=SimpleNamespace(set=lambda: None),
        spin_thread=SimpleNamespace(join=lambda timeout=0.0: None),
        did_init=True,
    )

    monkeypatch.setattr(
        runtime._LOGGER,
        'warning',
        lambda message, **kwargs: warning_messages.append(message),
    )

    runtime_session.close()

    assert runtime_session._started is None
    assert fake_rclpy.shutdown_called is True
    assert any('remove node from executor' in message for message in warning_messages)
    assert any('destroy ROS node' in message for message in warning_messages)
    assert any('shut down executor' in message for message in warning_messages)


def test_gazebo_launch_is_available_returns_false_when_ament_index_import_fails(
    monkeypatch,
) -> None:
    """Import errors in ament_index_python should report Gazebo as unavailable."""
    original_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == 'ament_index_python.packages':
            raise ImportError('ament index unavailable')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', fake_import)

    assert runtime._gazebo_launch_is_available() is False


def test_gazebo_launch_is_available_returns_false_when_package_is_missing(
    monkeypatch,
) -> None:
    """Missing drqp_gazebo package prefixes should report Gazebo as unavailable."""

    class FakePackageNotFoundError(Exception):
        """Test double for missing ament package lookups."""

    fake_ament_module = ModuleType('ament_index_python')
    fake_packages_module = ModuleType('ament_index_python.packages')

    def fake_get_package_prefix(package_name: str) -> str:
        assert package_name == 'drqp_gazebo'
        raise FakePackageNotFoundError('missing package')

    fake_packages_module.get_package_prefix = fake_get_package_prefix
    fake_packages_module.PackageNotFoundError = FakePackageNotFoundError
    fake_ament_module.packages = fake_packages_module
    monkeypatch.setitem(sys.modules, 'ament_index_python', fake_ament_module)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', fake_packages_module)

    assert runtime._gazebo_launch_is_available() is False
