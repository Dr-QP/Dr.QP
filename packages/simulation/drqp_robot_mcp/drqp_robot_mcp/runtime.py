"""Local ROS 2 and Gazebo runtime helpers for the Dr.QP robot MCP server."""

from __future__ import annotations

import atexit
from dataclasses import dataclass
from datetime import datetime, UTC
import logging
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any

_LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class _RosDependencies:
    """Loaded ROS dependencies required by the runtime session."""

    rclpy: Any
    executor_factory: Any
    odometry_message_type: Any
    string_message_type: Any
    vector3_message_type: Any
    movement_command_type: Any
    qos_profile_type: Any
    durability_policy: Any


@dataclass(slots=True)
class _GazeboTransportDependencies:
    """Loaded Gazebo Transport dependencies required by world-state reads."""

    node_factory: Any
    pose_v_message_type: Any


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
        self._gazebo_transport_dependencies: _GazeboTransportDependencies | None = None
        self._gazebo_transport_node: Any | None = None
        self._lifecycle_state: str | None = None
        self._robot_pose: dict[str, Any] | None = None
        self._world_states: dict[str, dict[str, Any]] = {}
        self._world_state_subscriptions: dict[str, Any] = {}
        self._joint_states: dict[str, dict[str, float | None]] = {}
        self._simulation_time_sec: float | None = None

    def publish_event(self, event: str) -> dict[str, Any]:
        """Publish a robot lifecycle event onto `/robot_event`."""
        started = self._ensure_started()
        self._wait_for_subscribers(started.event_publisher, timeout_sec=2.0)
        started.event_publisher.publish(started.dependencies.string_message_type(data=event))
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
        message.stride_direction = started.dependencies.vector3_message_type(**stride_direction)
        message.rotation_speed = rotation_speed
        message.body_translation = started.dependencies.vector3_message_type(**body_translation)
        message.body_rotation = started.dependencies.vector3_message_type(**body_rotation)
        message.gait_type = gait_type

        started.movement_command_publisher.publish(message)
        self._wait_for_delivery(duration_sec=0.25)
        return {
            'published': True,
            'topic': '/robot/movement_command',
            'subscription_count': (started.movement_command_publisher.get_subscription_count()),
            'stride_direction': stride_direction,
            'rotation_speed': rotation_speed,
            'body_translation': body_translation,
            'body_rotation': body_rotation,
            'gait_type': gait_type,
            'message': 'Published motion command.',
        }

    def wait_for_trajectory_action_server(self, timeout_sec: float) -> bool:
        """Wait for the joint trajectory controller action server."""
        started = self._ensure_started()

        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        action_client = ActionClient(
            started.node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        try:
            return bool(action_client.wait_for_server(timeout_sec=timeout_sec))
        finally:
            destroy = getattr(action_client, 'destroy', None)
            if callable(destroy):
                destroy()

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
        self._ensure_started()
        self._wait_for_runtime_data(timeout_sec)

        with self._state_changed:
            self._raise_spin_error_locked()
            lifecycle_state = self._lifecycle_state
            robot_pose = None if self._robot_pose is None else self._robot_pose.copy()
            joint_states = {name: values.copy() for name, values in self._joint_states.items()}
            simulation_time_sec = self._simulation_time_sec

        available = (
            lifecycle_state is not None
            or bool(joint_states)
            or robot_pose is not None
            or simulation_time_sec is not None
        )
        simulation_running = simulation_time_sec is not None

        return {
            'timestamp': datetime.now(UTC).isoformat(),
            'available': available,
            'simulation_running': simulation_running,
            'lifecycle_state': lifecycle_state,
            'world_name': world_name if simulation_running else None,
            'simulation_time_sec': simulation_time_sec,
            'robot_pose': robot_pose,
            'joint_states': joint_states,
            'note': None if available else 'Robot topics are not yet available.',
        }

    def get_world_state(self, world_name: str, timeout_sec: float) -> dict[str, Any]:
        """Read the latest Gazebo world entity poses via Transport."""
        self._ensure_started()
        note: str | None = None
        try:
            self._ensure_world_state_subscription(world_name)
            self._wait_for_world_state(world_name, timeout_sec)
        except RuntimeError as exc:
            note = str(exc)

        with self._state_changed:
            self._raise_spin_error_locked()
            cached_world_state = self._world_states.get(world_name)
            simulation_time_sec = self._simulation_time_sec

        if cached_world_state is None:
            return {
                'available': simulation_time_sec is not None,
                'world_name': world_name,
                'simulation_time_sec': simulation_time_sec,
                'entity_count': 0,
                'entities': [],
                'source': 'gazebo',
                'note': (
                    note
                    or (
                        None
                        if simulation_time_sec is not None
                        else 'World state transport is not yet available.'
                    )
                ),
            }

        world_state = {
            'available': True,
            'world_name': cached_world_state['world_name'],
            'simulation_time_sec': cached_world_state['simulation_time_sec'],
            'entity_count': cached_world_state['entity_count'],
            'entities': [entity.copy() for entity in cached_world_state['entities']],
            'source': cached_world_state['source'],
            'note': None,
        }
        if world_state['simulation_time_sec'] is None:
            world_state['simulation_time_sec'] = simulation_time_sec

        return world_state

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
        except Exception:  # noqa: BLE001
            _LOGGER.warning(
                'Failed to remove node from executor during runtime close.',
                exc_info=True,
            )

        try:
            started.node.destroy_node()
        except Exception:  # noqa: BLE001
            _LOGGER.warning(
                'Failed to destroy ROS node during runtime close.',
                exc_info=True,
            )

        try:
            shutdown = getattr(started.executor, 'shutdown', None)
            if callable(shutdown):
                shutdown()
        except Exception:  # noqa: BLE001
            _LOGGER.warning(
                'Failed to shut down executor during runtime close.',
                exc_info=True,
            )

        with self._state_changed:
            self._gazebo_transport_node = None
            self._world_state_subscriptions = {}
            self._world_states = {}

        if started.did_init and started.dependencies.rclpy.ok():
            started.dependencies.rclpy.shutdown()

    def reset_simulation_state(self) -> None:
        """Clear cached simulation-only state after simulation shutdown."""
        with self._state_changed:
            self._robot_pose = None
            self._world_states = {}
            self._simulation_time_sec = None
            self._state_changed.notify_all()

    def get_system_state(
        self,
        world_name: str,
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Return the current ROS and transport integration health snapshot."""
        started = self._ensure_started()
        self._wait_for_runtime_data(timeout_sec)

        with self._state_changed:
            self._raise_spin_error_locked()
            lifecycle_state = self._lifecycle_state
            joint_state_available = bool(self._joint_states)
            simulation_channel_available = self._simulation_time_sec is not None
            world_state_channel_available = world_name in self._world_states

        motion_command_channel_available = (
            started.movement_command_publisher.get_subscription_count() > 0
        )
        robot_lifecycle_channel_available = lifecycle_state is not None
        deployment_mode = 'simulation' if simulation_channel_available else 'real_robot'
        degraded_subsystems = []
        if not robot_lifecycle_channel_available:
            degraded_subsystems.append('robot_lifecycle')
        if not joint_state_available:
            degraded_subsystems.append('joint_states')
        if not motion_command_channel_available:
            degraded_subsystems.append('motion_command')
        if deployment_mode == 'simulation' and not world_state_channel_available:
            degraded_subsystems.append('world_state')

        return {
            'timestamp': datetime.now(UTC).isoformat(),
            'available': True,
            'ros_runtime_available': True,
            'robot_lifecycle_channel_available': (robot_lifecycle_channel_available),
            'joint_state_available': joint_state_available,
            'motion_command_channel_available': (motion_command_channel_available),
            'simulation_channel_available': simulation_channel_available,
            'world_state_channel_available': world_state_channel_available,
            'deployment_mode': deployment_mode,
            'degraded_subsystems': degraded_subsystems,
            'note': None if not degraded_subsystems else 'One or more subsystems are degraded.',
        }

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
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState

            node.create_subscription(
                JointState,
                '/joint_states',
                self._handle_joint_state_message,
                10,
            )
            node.create_subscription(
                dependencies.odometry_message_type,
                '/odom',
                self._handle_odometry_message,
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

    def _ensure_world_state_subscription(self, world_name: str) -> None:
        """Create a Gazebo Transport subscription on first use for a world."""
        with self._state_changed:
            if world_name in self._world_state_subscriptions:
                return

            dependencies = self._ensure_gazebo_transport_dependencies_locked()
            if self._gazebo_transport_node is None:
                _ensure_gazebo_partition_environment()
                self._gazebo_transport_node = dependencies.node_factory()

            topic = f'/world/{world_name}/pose/info'

            def callback(message: Any, subscribed_world: str = world_name) -> None:
                self._handle_world_state_message(
                    subscribed_world,
                    message,
                )

            subscribed = self._gazebo_transport_node.subscribe(
                dependencies.pose_v_message_type,
                topic,
                callback,
            )
            if not subscribed:
                raise RuntimeError(f'Failed to subscribe to Gazebo world pose topic: {topic}')

            self._world_state_subscriptions[world_name] = callback

    def _ensure_gazebo_transport_dependencies_locked(
        self,
    ) -> _GazeboTransportDependencies:
        """Load Gazebo Transport dependencies once for world-state reads."""
        if self._gazebo_transport_dependencies is None:
            self._gazebo_transport_dependencies = _load_gazebo_transport_dependencies()
        return self._gazebo_transport_dependencies

    def _spin_executor(self, executor: Any, stop_event: threading.Event) -> None:
        """Continuously spin the shared ROS executor in a background thread."""
        while not stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001
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

    def _handle_odometry_message(self, message: Any) -> None:
        """Cache the latest robot pose from the bridged odometry topic."""
        with self._state_changed:
            self._robot_pose = _pose_to_mapping(message.pose.pose)
            self._state_changed.notify_all()

    def _handle_world_state_message(self, world_name: str, message: Any) -> None:
        """Cache the latest Gazebo world-state message for a world."""
        with self._state_changed:
            self._world_states[world_name] = _world_state_to_mapping(world_name, message)
            self._state_changed.notify_all()

    def _handle_clock_message(self, message: Any) -> None:
        """Cache the latest simulation clock message."""
        simulation_time_sec = float(message.clock.sec) + (message.clock.nanosec / 1_000_000_000)
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
                    or self._robot_pose is not None
                    or bool(self._joint_states)
                    or self._simulation_time_sec is not None
                ):
                    return

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return

                self._state_changed.wait(timeout=min(0.1, remaining))

    def _wait_for_world_state(self, world_name: str, timeout_sec: float) -> None:
        """Wait briefly for a bridged world-state message to arrive."""
        deadline = time.monotonic() + timeout_sec
        with self._state_changed:
            while True:
                self._raise_spin_error_locked()
                if world_name in self._world_states:
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
    import drqp_interfaces.msg
    from geometry_msgs.msg import Vector3
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    return _RosDependencies(
        rclpy=rclpy,
        executor_factory=SingleThreadedExecutor,
        odometry_message_type=Odometry,
        string_message_type=String,
        vector3_message_type=Vector3,
        movement_command_type=drqp_interfaces.msg.MovementCommand,
        qos_profile_type=QoSProfile,
        durability_policy=QoSDurabilityPolicy,
    )


def _load_gazebo_transport_dependencies() -> _GazeboTransportDependencies:
    """Import Gazebo Transport modules lazily for world-state subscriptions."""
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    return _GazeboTransportDependencies(
        node_factory=Node,
        pose_v_message_type=Pose_V,
    )


def _ensure_gazebo_partition_environment() -> None:
    """Match the launch-file default Gazebo partition when none is set."""
    if os.environ.get('GZ_PARTITION'):
        return

    hostname = os.environ.get('HOSTNAME') or 'unknown-host'
    user = os.environ.get('USER') or 'unknown-user'
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID') or '0'
    os.environ['GZ_PARTITION'] = f'{hostname}:{user}-domain-{ros_domain_id}'


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


def stop_simulation(
    pid_path: Path,
    log_path: Path,
    timeout_sec: float = 120.0,
) -> dict[str, Any]:
    """Stop the Gazebo simulation launch process tracked by the runtime PID file."""
    current_pid = _read_pid(pid_path)
    if current_pid is None or not _pid_is_running(current_pid):
        if pid_path.exists():
            pid_path.unlink(missing_ok=True)
        return {
            'stopped': False,
            'available': False,
            'pid': current_pid,
            'message': 'Simulation launch is already stopped.',
            'log_path': str(log_path),
        }

    try:
        os.killpg(current_pid, signal.SIGTERM)
    except ProcessLookupError:
        pid_path.unlink(missing_ok=True)
        return {
            'stopped': True,
            'available': False,
            'pid': current_pid,
            'message': 'Simulation launch was already exiting.',
            'log_path': str(log_path),
        }

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not _pid_is_running(current_pid):
            pid_path.unlink(missing_ok=True)
            return {
                'stopped': True,
                'available': False,
                'pid': current_pid,
                'message': 'Stopped Gazebo simulation launch.',
                'log_path': str(log_path),
            }
        time.sleep(0.1)

    return {
        'stopped': False,
        'available': True,
        'pid': current_pid,
        'message': 'Timed out waiting for the Gazebo simulation launch to stop.',
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


def get_system_state(world_name: str, timeout_sec: float) -> dict[str, Any]:
    """Return the current ROS and transport integration health snapshot."""
    return get_default_ros_runtime().get_system_state(world_name, timeout_sec)


def wait_for_trajectory_action_server(timeout_sec: float) -> bool:
    """Wait for the joint trajectory controller action server."""
    return get_default_ros_runtime().wait_for_trajectory_action_server(timeout_sec)


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


def _pose_to_mapping(message: Any) -> dict[str, Any]:
    """Convert a ROS pose-like object into the MCP pose payload shape."""
    return {
        'position': {
            'x': float(message.position.x),
            'y': float(message.position.y),
            'z': float(message.position.z),
        },
        'orientation': {
            'x': float(message.orientation.x),
            'y': float(message.orientation.y),
            'z': float(message.orientation.z),
            'w': float(message.orientation.w),
        },
    }


def _world_state_to_mapping(world_name: str, message: Any) -> dict[str, Any]:
    """Convert a Gazebo Pose_V message into the MCP payload shape."""
    entities = [_world_entity_to_mapping(entity) for entity in message.pose if entity.name]
    return {
        'world_name': world_name,
        'simulation_time_sec': _simulation_time_sec_from_pose_v(message),
        'entity_count': len(entities),
        'entities': entities,
        'source': 'gazebo',
    }


def _world_entity_to_mapping(message: Any) -> dict[str, Any]:
    """Convert a Gazebo pose message into the MCP payload shape."""
    return {
        'name': str(message.name),
        'entity_id': int(message.id),
        'pose': {
            'position': {
                'x': float(message.position.x),
                'y': float(message.position.y),
                'z': float(message.position.z),
            },
            'orientation': {
                'x': float(message.orientation.x),
                'y': float(message.orientation.y),
                'z': float(message.orientation.z),
                'w': float(message.orientation.w),
            },
        },
    }


def _simulation_time_sec_from_pose_v(message: Any) -> float | None:
    """Extract simulation time from a Gazebo Pose_V message header."""
    if not message.HasField('header'):
        return None

    stamp = message.header.stamp
    return float(stamp.sec) + (float(stamp.nsec) / 1_000_000_000.0)


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
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    except ImportError:
        return False

    try:
        get_package_prefix('drqp_gazebo')
    except PackageNotFoundError:
        return False
    return True


def get_runtime_directory(app_name: str = 'drqp_robot_mcp') -> Path:
    """
    Return the writable runtime directory for this package.

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
