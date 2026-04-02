"""Local ROS 2 and Gazebo runtime helpers for the Dr.QP robot MCP server."""

from __future__ import annotations

from datetime import datetime, UTC
import os
from pathlib import Path
import shutil
import subprocess
import time
from typing import Any, Callable


def start_simulation(
    workspace_root: Path,
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
                f'gui:={str(gui).lower()}',
            ],
            cwd=workspace_root,
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

    def _operation() -> dict[str, Any]:
        import rclpy
        from rclpy.node import Node
        import std_msgs.msg

        node = Node('drqp_robot_mcp_event_publisher')
        publisher = node.create_publisher(std_msgs.msg.String, '/robot_event', 10)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        publisher.publish(std_msgs.msg.String(data=event))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)

        subscriptions = publisher.get_subscription_count()
        node.destroy_node()
        return {
            'published': True,
            'event': event,
            'subscription_count': subscriptions,
        }

    return _with_rclpy(_operation)


def wait_for_state(target_state: str, timeout_sec: float) -> dict[str, Any]:
    """Wait until /robot_state reaches the requested state."""
    state = _read_robot_lifecycle_state(timeout_sec=timeout_sec, expected=target_state)
    return {
        'reached': state == target_state,
        'state': state,
        'target_state': target_state,
    }


def get_world_state(world_name: str, timeout_sec: float) -> dict[str, Any]:
    """Read the latest Gazebo world entity poses."""
    return _get_world_state_snapshot(world_name=world_name, timeout_sec=timeout_sec)


def get_robot_state(
    world_name: str,
    robot_name: str,
    timeout_sec: float,
) -> dict[str, Any]:
    """Read the latest robot lifecycle state, joints, and pose."""
    deadline = time.monotonic() + timeout_sec

    lifecycle_state = _read_robot_lifecycle_state(timeout_sec=_time_left(deadline))
    joint_states = _read_joint_states(timeout_sec=_time_left(deadline))
    world_state = _get_world_state_snapshot(
        world_name=world_name,
        timeout_sec=_time_left(deadline),
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
        'world_name': world_state.get('world_name') if world_state.get('available') else None,
        'simulation_time_sec': world_state.get('simulation_time_sec'),
        'robot_pose': robot_pose,
        'joint_states': joint_states,
        'note': None if available else 'Robot topics are not yet available.',
    }


def _get_world_state_snapshot(world_name: str, timeout_sec: float) -> dict[str, Any]:
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

    simulation_time_sec = _read_simulation_time(timeout_sec=min(timeout_sec, 0.5))
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


def _read_robot_lifecycle_state(
    timeout_sec: float,
    expected: str | None = None,
) -> str | None:
    """Read the latest lifecycle state from /robot_state."""

    def _operation() -> str | None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        import std_msgs.msg

        node = Node('drqp_robot_mcp_state_reader')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        state: dict[str, str | None] = {'value': None}

        def callback(message: std_msgs.msg.String) -> None:
            state['value'] = message.data

        node.create_subscription(std_msgs.msg.String, '/robot_state', callback, qos)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state['value'] is None:
                continue
            if expected is None or state['value'] == expected:
                break

        value = state['value']
        node.destroy_node()
        return value

    return _with_rclpy(_operation)


def _read_joint_states(timeout_sec: float) -> dict[str, dict[str, float | None]]:
    """Read the latest joint state message from /joint_states."""

    def _operation() -> dict[str, dict[str, float | None]]:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        node = Node('drqp_robot_mcp_joint_state_reader')
        joint_states: dict[str, dict[str, float | None]] = {}

        def callback(message: JointState) -> None:
            for index, name in enumerate(message.name):
                joint_states[name] = {
                    'position': _value_at(message.position, index),
                    'velocity': _value_at(message.velocity, index),
                    'effort': _value_at(message.effort, index),
                }

        node.create_subscription(JointState, '/joint_states', callback, 10)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not joint_states:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()
        return joint_states

    return _with_rclpy(_operation)


def _read_simulation_time(timeout_sec: float) -> float | None:
    """Read the latest simulation time from /clock."""

    def _operation() -> float | None:
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock

        node = Node('drqp_robot_mcp_clock_reader')
        result: dict[str, float | None] = {'value': None}

        def callback(message: Clock) -> None:
            result['value'] = float(message.clock.sec) + (message.clock.nanosec / 1_000_000_000)

        node.create_subscription(Clock, '/clock', callback, 10)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and result['value'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        value = result['value']
        node.destroy_node()
        return value

    return _with_rclpy(_operation)


def _with_rclpy(operation: Callable[[], Any]) -> Any:
    """Run a short-lived rclpy operation with clean init/shutdown."""
    import rclpy

    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True

    try:
        return operation()
    finally:
        if did_init and rclpy.ok():
            rclpy.shutdown()


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
    if shutil.which('ros2') is None:
        return False
    try:
        subprocess.run(  # noqa: S603
            ['ros2', 'pkg', 'prefix', 'drqp_gazebo'],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False
    return True


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
    return True


def _value_at(values: Any, index: int) -> float | None:
    """Safely read a sequence value by index."""
    if index >= len(values):
        return None
    return float(values[index])


def _time_left(deadline: float) -> float:
    """Return the remaining time budget for a composite operation."""
    return max(0.1, deadline - time.monotonic())
