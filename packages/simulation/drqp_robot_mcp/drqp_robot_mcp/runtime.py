"""Local ROS 2 and Gazebo runtime helpers for the Dr.QP robot MCP server."""

from __future__ import annotations

import atexit
from dataclasses import dataclass
from datetime import UTC, datetime
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any


@dataclass(slots=True)
class _RosDependencies:
    """Loaded ROS dependencies required by the runtime session."""

    rclpy: Any
    executor_factory: Any
    string_message_type: Any
    vector3_message_type: Any
    movement_command_type: Any
    qos_profile_type: Any
    durability_policy: Any


@dataclass(slots=True)
class _StartedRosRuntime:
    """State for a running ROS runtime session."""

    dependencies: _RosDependencies
    node: Any
    executor: Any
    event_publisher: Any
    movement_command_publisher: Any
    stop_event: threading.Event
    spin_thread: threading.Thread
    did_init: bool


class RosRuntimeSession:
    """Own a single ROS node and its long-lived publishers/subscribers."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state_changed = threading.Condition(self._lock)
        self._started: _StartedRosRuntime | None = None
        self._spin_error: BaseException | None = None
        self._lifecycle_state: str | None = None
        self._joint_states: dict[str, dict[str, float | None]] = {}
        self._simulation_time_sec: float | None = None

    def publish_event(self, event: str) -> dict[str, Any]:
        """Publish a robot lifecycle event onto `/robot_event`."""
        started = self._ensure_started()
        self._wait_for_subscribers(started.event_publisher, timeout_sec=2.0)
        started.event_publisher.publish(
            started.dependencies.string_message_type(data=event)
        )
        self._wait_for_delivery(duration_sec=0.25)
        return {
            'published': True,
            'event': event,
            'subscription_count': started.event_publisher.get_subscription_count(),
        }

    def publish_movement_command(
        self,
        stride_direction: dict[str, float],
        rotation_speed: float,
        body_translation: dict[str, float],
        body_rotation: dict[str, float],
        gait_type: str,
    ) -> dict[str, Any]:
        """Publish a movement command onto `/robot/movement_command`."""
        started = self._ensure_started()
        self._wait_for_subscribers(started.movement_command_publisher, timeout_sec=2.0)

        message = started.dependencies.movement_command_type()
        message.stride_direction = started.dependencies.vector3_message_type(
            **stride_direction
        )
        message.rotation_speed = rotation_speed
        message.body_translation = started.dependencies.vector3_message_type(
            **body_translation
        )
        message.body_rotation = started.dependencies.vector3_message_type(
            **body_rotation
        )
        message.gait_type = gait_type

        started.movement_command_publisher.publish(message)
        self._wait_for_delivery(duration_sec=0.25)
        return {
            'published': True,
            'topic': '/robot/movement_command',
            'subscription_count': (
                started.movement_command_publisher.get_subscription_count()
            ),
            'stride_direction': stride_direction,
            'rotation_speed': rotation_speed,
            'body_translation': body_translation,
            'body_rotation': body_rotation,
            'gait_type': gait_type,
            'message': 'Published motion command.',
        }

    def wait_for_state(self, target_state: str, timeout_sec: float) -> dict[str, Any]:
        """Wait until `/robot_state` reaches the requested state."""
        deadline = time.monotonic() + timeout_sec
        self._ensure_started()
        with self._state_changed:
            while True:
                self._raise_spin_error_locked()
                current_state = self._lifecycle_state
                if current_state == target_state:
                    return {
                        'reached': True,
                        'state': current_state,
                        'target_state': target_state,
                    }

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return {
                        'reached': False,
                        'state': current_state,
                        'target_state': target_state,
                    }

                self._state_changed.wait(timeout=min(0.1, remaining))

    def get_robot_state(
        self,
        world_name: str,
        robot_name: str,
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Read the latest robot lifecycle state, joints, and pose."""
        deadline = time.monotonic() + timeout_sec
        self._ensure_started()
        self._wait_for_runtime_data(timeout_sec)

        with self._state_changed:
            self._raise_spin_error_locked()
            lifecycle_state = self._lifecycle_state
            joint_states = {
                name: values.copy()
                for name, values in self._joint_states.items()
            }
            simulation_time_sec = self._simulation_time_sec

        world_state = _get_world_state_snapshot(
            world_name=world_name,
            timeout_sec=_time_left(deadline),
            simulation_time_sec=simulation_time_sec,
        )
        robot_pose = _find_robot_pose(world_state.get('entities', []), robot_name)

        available = (
            lifecycle_state is not None
            or bool(joint_states)
            or robot_pose is not None
            or bool(world_state.get('available'))
        )
        simulation_running = bool(world_state.get('available'))

        return {
            'timestamp': datetime.now(UTC).isoformat(),
            'available': available,
            'simulation_running': simulation_running,
            'lifecycle_state': lifecycle_state,
            'world_name': (
                world_state.get('world_name') if world_state.get('available') else None
            ),
            'simulation_time_sec': world_state.get('simulation_time_sec'),
            'robot_pose': robot_pose,
            'joint_states': joint_states,
            'note': None if available else 'Robot topics are not yet available.',
        }

    def get_world_state(self, world_name: str, timeout_sec: float) -> dict[str, Any]:
        """Read the latest Gazebo world entity poses."""
        self._ensure_started()
        self._wait_for_runtime_data(min(timeout_sec, 0.5))

        with self._state_changed:
            self._raise_spin_error_locked()
            simulation_time_sec = self._simulation_time_sec

        return _get_world_state_snapshot(
            world_name=world_name,
            timeout_sec=timeout_sec,
            simulation_time_sec=simulation_time_sec,
        )

    def close(self) -> None:
        """Stop the executor thread and release the ROS node."""
        with self._state_changed:
            started = self._started
            self._started = None

        if started is None:
            return

        started.stop_event.set()
        started.spin_thread.join(timeout=2.0)

        try:
            started.executor.remove_node(started.node)
        except Exception:
            pass

        try:
            started.node.destroy_node()
        except Exception:
            pass

        try:
            shutdown = getattr(started.executor, 'shutdown', None)
            if callable(shutdown):
                shutdown()
        except Exception:
            pass

        if started.did_init and started.dependencies.rclpy.ok():
            started.dependencies.rclpy.shutdown()

    def _ensure_started(self) -> _StartedRosRuntime:
        """Initialize the shared ROS node on first use."""
        with self._state_changed:
            self._raise_spin_error_locked()
            if self._started is not None:
                return self._started

            dependencies = _load_ros_dependencies()
            did_init = False
            if not dependencies.rclpy.ok():
                dependencies.rclpy.init()
                did_init = True

            node = dependencies.rclpy.create_node('drqp_robot_mcp_runtime')
            executor = dependencies.executor_factory()
            executor.add_node(node)

            event_publisher = node.create_publisher(
                dependencies.string_message_type,
                '/robot_event',
                10,
            )
            movement_command_publisher = node.create_publisher(
                dependencies.movement_command_type,
                '/robot/movement_command',
                10,
            )

            state_qos = dependencies.qos_profile_type(depth=1)
            state_qos.durability = dependencies.durability_policy.TRANSIENT_LOCAL

            node.create_subscription(
                dependencies.string_message_type,
                '/robot_state',
                self._handle_state_message,
                state_qos,
            )
            from sensor_msgs.msg import JointState
            from rosgraph_msgs.msg import Clock

            node.create_subscription(
                JointState,
                '/joint_states',
                self._handle_joint_state_message,
                10,
            )
            node.create_subscription(
                Clock,
                '/clock',
                self._handle_clock_message,
                10,
            )

            stop_event = threading.Event()
            spin_thread = threading.Thread(
                target=self._spin_executor,
                args=(executor, stop_event),
                name='drqp-robot-mcp-runtime',
                daemon=True,
            )
            spin_thread.start()

            self._started = _StartedRosRuntime(
                dependencies=dependencies,
                node=node,
                executor=executor,
                event_publisher=event_publisher,
                movement_command_publisher=movement_command_publisher,
                stop_event=stop_event,
                spin_thread=spin_thread,
                did_init=did_init,
            )
            return self._started

    def _spin_executor(self, executor: Any, stop_event: threading.Event) -> None:
        """Continuously spin the shared ROS executor in a background thread."""
        while not stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                with self._state_changed:
                    self._spin_error = exc
                    self._state_changed.notify_all()
                return

    def _handle_state_message(self, message: Any) -> None:
        """Cache the latest lifecycle state message."""
        with self._state_changed:
            self._lifecycle_state = message.data
            self._state_changed.notify_all()

    def _handle_joint_state_message(self, message: Any) -> None:
        """Cache the latest joint state message."""
        joint_states: dict[str, dict[str, float | None]] = {}
        for index, name in enumerate(message.name):
            joint_states[name] = {
                'position': _value_at(message.position, index),
                'velocity': _value_at(message.velocity, index),
                'effort': _value_at(message.effort, index),
            }

        with self._state_changed:
            self._joint_states = joint_states
            self._state_changed.notify_all()

    def _handle_clock_message(self, message: Any) -> None:
        """Cache the latest simulation clock message."""
        simulation_time_sec = float(message.clock.sec) + (
            message.clock.nanosec / 1_000_000_000
        )
        with self._state_changed:
            self._simulation_time_sec = simulation_time_sec
            self._state_changed.notify_all()

    def _wait_for_runtime_data(self, timeout_sec: float) -> None:
        """Wait briefly for any subscribed runtime data to arrive."""
        deadline = time.monotonic() + timeout_sec
        with self._state_changed:
            while True:
                self._raise_spin_error_locked()
                if (
                    self._lifecycle_state is not None
                    or bool(self._joint_states)
                    or self._simulation_time_sec is not None
                ):
                    return

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return

                self._state_changed.wait(timeout=min(0.1, remaining))

    def _wait_for_subscribers(self, publisher: Any, timeout_sec: float) -> None:
        """Give the ROS graph a brief chance to attach subscribers."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._state_changed:
                self._raise_spin_error_locked()
            if publisher.get_subscription_count() > 0:
                return
            time.sleep(0.05)

    def _wait_for_delivery(self, duration_sec: float) -> None:
        """Leave a short window for DDS delivery and graph callbacks."""
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            with self._state_changed:
                self._raise_spin_error_locked()
            time.sleep(0.05)

    def _raise_spin_error_locked(self) -> None:
        """Raise any executor failure captured by the spin thread."""
        if self._spin_error is None:
            return

        raise RuntimeError('drqp_robot_mcp ROS runtime stopped unexpectedly.') from (
            self._spin_error
        )


def _load_ros_dependencies() -> _RosDependencies:
    """Import ROS modules lazily so module import stays cheap."""
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    import drqp_interfaces.msg

    return _RosDependencies(
        rclpy=rclpy,
        executor_factory=SingleThreadedExecutor,
        string_message_type=String,
        vector3_message_type=Vector3,
        movement_command_type=drqp_interfaces.msg.MovementCommand,
        qos_profile_type=QoSProfile,
        durability_policy=QoSDurabilityPolicy,
    )


_DEFAULT_ROS_RUNTIME: RosRuntimeSession | None = None
_DEFAULT_ROS_RUNTIME_LOCK = threading.Lock()


def get_default_ros_runtime() -> RosRuntimeSession:
    """Return the process-wide ROS runtime session."""
    global _DEFAULT_ROS_RUNTIME
    with _DEFAULT_ROS_RUNTIME_LOCK:
        if _DEFAULT_ROS_RUNTIME is None:
            _DEFAULT_ROS_RUNTIME = RosRuntimeSession()
        return _DEFAULT_ROS_RUNTIME


def shutdown_default_ros_runtime() -> None:
    """Tear down the process-wide ROS runtime session."""
    global _DEFAULT_ROS_RUNTIME
    with _DEFAULT_ROS_RUNTIME_LOCK:
        runtime_session = _DEFAULT_ROS_RUNTIME
        _DEFAULT_ROS_RUNTIME = None

    if runtime_session is not None:
        runtime_session.close()


def start_simulation(
    pid_path: Path,
    log_path: Path,
    gui: bool = False,
) -> dict[str, Any]:
    """Start the Gazebo simulation locally when the launch file is available."""
    pid_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    current_pid = _read_pid(pid_path)
    if current_pid is not None and _pid_is_running(current_pid):
        return {
            'started': False,
            'available': True,
            'pid': current_pid,
            'message': 'Simulation launch is already running.',
            'log_path': str(log_path),
        }

    if not _gazebo_launch_is_available():
        return {
            'started': False,
            'available': False,
            'pid': None,
            'message': 'Gazebo launch is not available in the current ROS 2 environment.',
            'log_path': str(log_path),
        }

    with log_path.open('a', encoding='utf-8') as log_file:
        process = subprocess.Popen(  # noqa: S603
            [
                'ros2',
                'launch',
                'drqp_gazebo',
                'sim.launch.py',
                f'sim_gui:={str(gui).lower()}',
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    pid_path.write_text(f'{process.pid}\n', encoding='utf-8')
    return {
        'started': True,
        'available': True,
        'pid': process.pid,
        'message': 'Started Gazebo simulation launch.',
        'log_path': str(log_path),
    }


def publish_event(event: str) -> dict[str, Any]:
    """Publish a robot event onto /robot_event."""
    return get_default_ros_runtime().publish_event(event)


def publish_movement_command(
    stride_direction: dict[str, float],
    rotation_speed: float,
    body_translation: dict[str, float],
    body_rotation: dict[str, float],
    gait_type: str,
) -> dict[str, Any]:
    """Publish a movement command onto /robot/movement_command."""
    return get_default_ros_runtime().publish_movement_command(
        stride_direction=stride_direction,
        rotation_speed=rotation_speed,
        body_translation=body_translation,
        body_rotation=body_rotation,
        gait_type=gait_type,
    )


def wait_for_state(target_state: str, timeout_sec: float) -> dict[str, Any]:
    """Wait until /robot_state reaches the requested state."""
    return get_default_ros_runtime().wait_for_state(target_state, timeout_sec)


def get_world_state(world_name: str, timeout_sec: float) -> dict[str, Any]:
    """Read the latest Gazebo world entity poses."""
    return get_default_ros_runtime().get_world_state(world_name, timeout_sec)


def get_robot_state(
    world_name: str,
    robot_name: str,
    timeout_sec: float,
) -> dict[str, Any]:
    """Read the latest robot lifecycle state, joints, and pose."""
    return get_default_ros_runtime().get_robot_state(
        world_name=world_name,
        robot_name=robot_name,
        timeout_sec=timeout_sec,
    )


def _get_world_state_snapshot(
    world_name: str,
    timeout_sec: float,
    simulation_time_sec: float | None = None,
) -> dict[str, Any]:
    """Read Gazebo entity poses from pose info and ROS clock."""
    entities: list[dict[str, Any]] = []
    note: str | None = None

    try:
        raw_output = subprocess.run(  # noqa: S603
            ['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world_name}/pose/info'],
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        entities = parse_gazebo_pose_info(raw_output.stdout)
    except FileNotFoundError:
        note = 'Gazebo CLI is not installed in the current environment.'
    except subprocess.CalledProcessError as exc:
        note = exc.stderr.strip() or exc.stdout.strip() or 'Gazebo pose topic is unavailable.'
    except subprocess.TimeoutExpired:
        note = 'Timed out waiting for Gazebo world pose information.'

    available = bool(entities) or simulation_time_sec is not None

    return {
        'available': available,
        'world_name': world_name,
        'simulation_time_sec': simulation_time_sec,
        'entity_count': len(entities),
        'entities': entities,
        'source': 'gazebo',
        'note': note,
    }


def _find_robot_pose(
    entities: list[dict[str, Any]],
    robot_name: str,
) -> dict[str, Any] | None:
    """Return the best matching robot pose from world entities."""
    for entity in entities:
        name = entity.get('name', '')
        if name == robot_name or name.startswith(f'{robot_name}_'):
            return entity.get('pose')
    return None


def parse_gazebo_pose_info(raw_output: str) -> list[dict[str, Any]]:
    """Parse `gz topic -e /world/<world>/pose/info` output into entity poses."""
    entities: list[dict[str, Any]] = []
    for block in _extract_blocks(raw_output, 'pose'):
        entity = _parse_pose_block(block)
        if entity['name']:
            entities.append(entity)
    return entities


def _extract_blocks(raw_output: str, block_name: str) -> list[list[str]]:
    """Extract nested protobuf-style blocks from Gazebo CLI output."""
    blocks: list[list[str]] = []
    lines = raw_output.splitlines()
    index = 0
    opening = f'{block_name} {{'

    while index < len(lines):
        if lines[index].strip() != opening:
            index += 1
            continue

        depth = 1
        block_lines: list[str] = []
        index += 1
        while index < len(lines) and depth > 0:
            stripped = lines[index].strip()
            if stripped.endswith('{'):
                depth += 1
            if stripped == '}':
                depth -= 1
                if depth == 0:
                    break
            if depth > 0:
                block_lines.append(lines[index])
            index += 1

        blocks.append(block_lines)
        index += 1

    return blocks


def _parse_pose_block(lines: list[str]) -> dict[str, Any]:
    """Parse a single Gazebo entity pose block."""
    name = ''
    entity_id: int | None = None
    position: dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    orientation: dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    current_section: str | None = None

    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped in {'position {', 'orientation {'}:
            current_section = stripped.split()[0]
            continue
        if stripped == '}':
            current_section = None
            continue
        if ':' not in stripped:
            continue

        key, value = stripped.split(':', maxsplit=1)
        text = value.strip().strip('"')
        if current_section is None:
            if key == 'name':
                name = text
            elif key == 'id':
                entity_id = int(text)
            continue

        numeric = float(text)
        if current_section == 'position':
            position[key] = numeric
        elif current_section == 'orientation':
            orientation[key] = numeric

    return {
        'name': name,
        'entity_id': entity_id,
        'pose': {
            'position': position,
            'orientation': orientation,
        },
    }


def _gazebo_launch_is_available() -> bool:
    """Return whether Gazebo launch support is present in this environment."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix

        get_package_prefix('drqp_gazebo')
    except (ImportError, PackageNotFoundError):
        return False
    return True


def get_runtime_directory(app_name: str = 'drqp_robot_mcp') -> Path:
    """Return the writable runtime directory for this package.

    Prefer ROS_HOME so runtime files align with ROS conventions instead of the
    repository checkout location.
    """
    ros_home = os.environ.get('ROS_HOME')
    if ros_home:
        return Path(ros_home).expanduser() / app_name

    return Path.home() / '.ros' / app_name


def _read_pid(pid_path: Path) -> int | None:
    """Read a PID file if present."""
    if not pid_path.exists():
        return None
    try:
        return int(pid_path.read_text(encoding='utf-8').strip())
    except ValueError:
        return None


def _pid_is_running(pid: int) -> bool:
    """Check whether a process is still alive."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False

    try:
        result = subprocess.run(  # noqa: S603
            ['ps', '-o', 'stat=', '-p', str(pid)],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return True

    if result.returncode != 0:
        return False

    status = result.stdout.strip()
    if not status:
        return False

    return not status.startswith('Z')


def _value_at(values: Any, index: int) -> float | None:
    """Safely read a sequence value by index."""
    if index >= len(values):
        return None
    return float(values[index])


def _time_left(deadline: float) -> float:
    """Return the remaining time budget for a composite operation."""
    return max(0.1, deadline - time.monotonic())


atexit.register(shutdown_default_ros_runtime)
