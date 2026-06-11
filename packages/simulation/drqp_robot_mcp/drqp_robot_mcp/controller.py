"""Host-side controller for the Dr.QP robot MCP server."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, UTC
import math
import threading
import time
from typing import Any, Callable

from . import runtime
from .models import (
    LifecycleActionResult,
    MotionCommandResult,
    MotionSequenceResult,
    RecordedRobotStates,
    RecordingStatus,
    RobotStateSnapshot,
    WorldStateSnapshot,
)


class ControllerError(RuntimeError):
    """Raised when the robot MCP controller cannot complete an action."""


@dataclass(slots=True)
class _RecordingSession:
    """State for an active recording session."""

    sample_interval_sec: float
    started_at: str
    samples: list[RobotStateSnapshot] = field(default_factory=list)
    stop_event: threading.Event = field(default_factory=threading.Event)
    worker: threading.Thread | None = None


class RobotMcpController:
    """Coordinate local ROS 2, optional Gazebo, and robot lifecycle operations."""

    _VALID_GAITS = {'tripod', 'ripple', 'wave'}

    def __init__(
        self,
        world_name: str = 'empty',
        robot_name: str = 'drqp',
        runtime_session: runtime.RosRuntimeSession | None = None,
    ) -> None:
        self.world_name = world_name
        self.robot_name = robot_name
        self.runtime = runtime_session or runtime.get_default_ros_runtime()
        self.runtime_dir = runtime.get_runtime_directory()
        self.launch_pid_path = self.runtime_dir / 'sim.launch.pid'
        self.launch_log_path = self.runtime_dir / 'sim.launch.log'
        self._recording_lock = threading.Lock()
        self._recording: _RecordingSession | None = None
        self._latest_motion_command: dict[str, Any] | None = None

    def close(self) -> None:
        """Release long-lived ROS resources owned by this controller."""
        self.runtime.close()

    def boot_up(self, timeout_sec: float = 120.0) -> LifecycleActionResult:
        """Boot the robot to the `torque_on` state."""
        simulation_was_started = False
        state_before_snapshot = self.get_robot_state()

        if state_before_snapshot.lifecycle_state is None:
            simulation = self._start_simulation()
            simulation_was_started = bool(simulation.get('started', False))
            try:
                state_before_snapshot = self._poll_state(
                    predicate=lambda snapshot: (
                        snapshot.lifecycle_state is not None
                        or bool(snapshot.joint_states)
                        or snapshot.simulation_running
                    ),
                    timeout_sec=min(timeout_sec, 60.0),
                    error_message=(
                        'Timed out waiting for the robot state topic after starting simulation.'
                    ),
                )
            except ControllerError as exc:
                if not bool(simulation.get('available', True)):
                    raise ControllerError(str(simulation.get('message'))) from exc
                raise

        state_before = state_before_snapshot.lifecycle_state
        if state_before is None and (
            bool(state_before_snapshot.joint_states) or state_before_snapshot.simulation_running
        ):
            state_before = 'torque_off'

        if state_before == 'torque_on':
            return LifecycleActionResult(
                action='boot_up',
                state_before=state_before,
                state_after=state_before,
                simulation_was_started=simulation_was_started,
                simulation_running=True,
                message='Robot is already booted.',
                log_path=str(self.launch_log_path),
            )

        if state_before in {'torque_off', 'finalized'}:
            if simulation_was_started or self._simulation_process_running():
                self._wait_for_boot_ready(timeout_sec=min(timeout_sec, 30.0))
            self._publish_event('initialize')
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'initializing':
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'finalizing':
            self._wait_for_state('finalized', timeout_sec)
            self._publish_event('initialize')
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'servos_rebooting':
            self._wait_for_state('torque_off', timeout_sec)
            self._publish_event('initialize')
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        else:
            raise ControllerError('Robot lifecycle state is unavailable; cannot complete boot up.')

        return LifecycleActionResult(
            action='boot_up',
            state_before=state_before,
            state_after=state_after_snapshot.lifecycle_state,
            simulation_was_started=simulation_was_started,
            simulation_running=state_after_snapshot.simulation_running,
            message='Robot reached torque_on.',
            log_path=str(self.launch_log_path),
        )

    def shut_down(self, timeout_sec: float = 120.0) -> LifecycleActionResult:
        """Shut the robot down through its lifecycle state machine."""
        state_before_snapshot = self.get_robot_state()
        state_before = state_before_snapshot.lifecycle_state

        if state_before is None:
            return LifecycleActionResult(
                action='shut_down',
                state_before=state_before,
                state_after=state_before,
                simulation_was_started=False,
                simulation_running=state_before_snapshot.simulation_running,
                message='Robot lifecycle state is unavailable.',
                log_path=str(self.launch_log_path),
            )

        if state_before == 'torque_on':
            self._publish_event('finalize')
            state_after_snapshot = self._wait_for_state('finalized', timeout_sec)
        elif state_before == 'initializing':
            self._publish_event('turn_off')
            state_after_snapshot = self._wait_for_state('torque_off', timeout_sec)
        elif state_before == 'finalizing':
            state_after_snapshot = self._wait_for_state('finalized', timeout_sec)
        elif state_before in {'finalized', 'torque_off'}:
            state_after_snapshot = state_before_snapshot
        elif state_before == 'servos_rebooting':
            state_after_snapshot = self._wait_for_state('torque_off', timeout_sec)
        else:
            raise ControllerError(f'Unsupported lifecycle state for shutdown: {state_before}')

        return LifecycleActionResult(
            action='shut_down',
            state_before=state_before,
            state_after=state_after_snapshot.lifecycle_state,
            simulation_was_started=False,
            simulation_running=state_after_snapshot.simulation_running,
            message=f'Robot shutdown completed in state {state_after_snapshot.lifecycle_state}.',
            log_path=str(self.launch_log_path),
        )

    def get_robot_state(self, timeout_sec: float = 10.0) -> RobotStateSnapshot:
        """Return the latest robot snapshot."""
        return RobotStateSnapshot.from_mapping(
            self.runtime.get_robot_state(
                self.world_name,
                self.robot_name,
                timeout_sec,
            )
        )

    def get_world_state(self, timeout_sec: float = 10.0) -> WorldStateSnapshot:
        """Return the latest Gazebo world snapshot."""
        return WorldStateSnapshot.from_mapping(
            self.runtime.get_world_state(self.world_name, timeout_sec)
        )

    def start_simulation(self, timeout_sec: float = 120.0) -> dict[str, Any]:
        """Ensure the simulation process is running and observable."""
        state_before = self.get_simulation_state(timeout_sec=min(timeout_sec, 1.0))
        result = self._start_simulation()
        state_after = self._poll_simulation_state(
            predicate=lambda snapshot: bool(snapshot['available']),
            timeout_sec=min(timeout_sec, 30.0),
            error_message='Timed out waiting for the simulation to become observable.',
        )
        return {
            'action': 'start',
            'state_before': state_before,
            'state_after': state_after,
            'simulation_was_started': bool(result.get('started', False)),
            'running': bool(state_after['running']),
            'message': str(result.get('message', 'Started Gazebo simulation launch.')),
            'log_path': str(result.get('log_path', self.launch_log_path)),
        }

    def stop_simulation(self, timeout_sec: float = 120.0) -> dict[str, Any]:
        """Stop the simulation process tracked by the runtime PID file."""
        state_before = self.get_simulation_state(timeout_sec=min(timeout_sec, 1.0))
        result = runtime.stop_simulation(
            self.launch_pid_path,
            self.launch_log_path,
            timeout_sec=timeout_sec,
        )
        if bool(result.get('stopped', False)):
            self.runtime.reset_simulation_state()
        state_after = self.get_simulation_state(timeout_sec=0.1)
        if not bool(result.get('stopped', False)) and bool(state_before['running']):
            raise ControllerError(str(result.get('message')))
        return {
            'action': 'stop',
            'state_before': state_before,
            'state_after': state_after,
            'simulation_was_started': False,
            'running': bool(state_after['running']),
            'message': str(result.get('message', 'Stopped Gazebo simulation launch.')),
            'log_path': str(result.get('log_path', self.launch_log_path)),
        }

    def get_simulation_state(self, timeout_sec: float = 10.0) -> dict[str, Any]:
        """Return the simulation runtime surface required by the spec."""
        robot_state = self.get_robot_state(timeout_sec=timeout_sec)
        process_running = self._simulation_process_running()
        available = bool(robot_state.simulation_running or process_running)
        note = robot_state.note
        if process_running and not robot_state.simulation_running:
            note = 'Simulation process is running but runtime signals are not yet available.'
        return {
            'timestamp': robot_state.timestamp,
            'available': available,
            'running': bool(robot_state.simulation_running or process_running),
            'world_name': self.world_name if available else None,
            'simulation_time_sec': robot_state.simulation_time_sec,
            'note': note,
            'status_details': None,
        }

    def get_simulation_robot_state(self, timeout_sec: float = 10.0) -> dict[str, Any]:
        """Return the simulated robot pose surface required by the spec."""
        robot_state = self.get_robot_state(timeout_sec=timeout_sec)
        available = robot_state.robot_pose is not None or robot_state.simulation_running
        return {
            'timestamp': robot_state.timestamp,
            'available': available,
            'world_name': self.world_name if available else None,
            'simulation_time_sec': robot_state.simulation_time_sec,
            'robot_entity_name': self.robot_name,
            'robot_pose': self._pose_to_payload(robot_state.robot_pose),
            'note': None if robot_state.robot_pose is not None else robot_state.note,
        }

    def get_simulation_world_state(self, timeout_sec: float = 10.0) -> dict[str, Any]:
        """Return the simulated world-state surface required by the spec."""
        world_state = self.get_world_state(timeout_sec=timeout_sec)
        return {
            'timestamp': _utc_now(),
            'available': world_state.available,
            'world_name': world_state.world_name,
            'simulation_time_sec': world_state.simulation_time_sec,
            'entity_count': world_state.entity_count,
            'entities': [
                {
                    'name': entity.name,
                    'entity_id': entity.entity_id,
                    'pose': self._pose_to_payload(entity.pose),
                }
                for entity in world_state.entities
            ],
            'source': world_state.source,
            'note': world_state.note,
        }

    def get_robot_namespace_state(self, timeout_sec: float = 10.0) -> dict[str, Any]:
        """Return the robot-local state surface required by the spec."""
        robot_state = self.get_robot_state(timeout_sec=timeout_sec)
        latest_motion_command = None
        if self._latest_motion_command is not None:
            latest_motion_command = dict(self._latest_motion_command)
            body_rotation = latest_motion_command.get('body_rotation')
            if isinstance(body_rotation, dict) and {'x', 'y', 'z'} <= body_rotation.keys():
                latest_motion_command['body_rotation'] = {
                    'roll': float(body_rotation['x']),
                    'pitch': float(body_rotation['y']),
                    'yaw': float(body_rotation['z']),
                }
        return {
            'timestamp': robot_state.timestamp,
            'available': robot_state.available,
            'lifecycle_state': robot_state.lifecycle_state,
            'active': robot_state.lifecycle_state == 'torque_on',
            'joint_states': {
                name: {
                    'position': value.position,
                    'velocity': value.velocity,
                    'effort': value.effort,
                }
                for name, value in robot_state.joint_states.items()
            },
            'latest_motion_command': latest_motion_command,
            'note': robot_state.note,
        }

    def get_system_state(self, timeout_sec: float = 10.0) -> dict[str, Any]:
        """Return the system runtime health surface required by the spec."""
        system_state = self.runtime.get_system_state(self.world_name, timeout_sec)
        if self._simulation_process_running():
            system_state['deployment_mode'] = 'simulation'
            if not system_state['simulation_channel_available']:
                degraded_subsystems = list(system_state['degraded_subsystems'])
                if 'simulation_runtime' not in degraded_subsystems:
                    degraded_subsystems.append('simulation_runtime')
                system_state['degraded_subsystems'] = degraded_subsystems
                system_state['note'] = 'Simulation is running but runtime signals are degraded.'
        return system_state

    def send_motion_command(
        self,
        *,
        stride_x: float = 0.0,
        stride_y: float = 0.0,
        stride_z: float = 0.0,
        rotation_speed: float = 0.0,
        body_x: float = 0.0,
        body_y: float = 0.0,
        body_z: float = 0.0,
        body_roll: float = 0.0,
        body_pitch: float = 0.0,
        body_yaw: float = 0.0,
        gait_type: str = 'tripod',
    ) -> MotionCommandResult:
        """Publish a normalized motion command for joystick-like control."""
        stride_direction = {
            'x': self._validate_normalized('stride_x', stride_x),
            'y': self._validate_normalized('stride_y', stride_y),
            'z': self._validate_normalized('stride_z', stride_z),
        }
        normalized_rotation_speed = self._validate_normalized(
            'rotation_speed',
            rotation_speed,
        )
        body_translation = {
            'x': self._validate_normalized('body_x', body_x),
            'y': self._validate_normalized('body_y', body_y),
            'z': self._validate_normalized('body_z', body_z),
        }
        body_rotation = {
            'x': self._validate_normalized('body_roll', body_roll),
            'y': self._validate_normalized('body_pitch', body_pitch),
            'z': self._validate_normalized('body_yaw', body_yaw),
        }
        normalized_gait = self._validate_gait_type(gait_type)
        result = self._publish_movement_command(
            stride_direction=stride_direction,
            rotation_speed=normalized_rotation_speed,
            body_translation=body_translation,
            body_rotation=body_rotation,
            gait_type=normalized_gait,
        )
        command_result = MotionCommandResult.from_mapping(result)
        self._latest_motion_command = {
            'stride_direction': dict(stride_direction),
            'rotation_speed': normalized_rotation_speed,
            'body_translation': dict(body_translation),
            'body_rotation': {
                'roll': body_rotation['x'],
                'pitch': body_rotation['y'],
                'yaw': body_rotation['z'],
            },
            'gait_type': normalized_gait,
            'timestamp': _utc_now(),
            'note': result.get('message'),
        }
        return command_result

    def stop_motion(self) -> MotionCommandResult:
        """Publish a zeroed movement command to stop robot motion."""
        return self.send_motion_command()

    def walk_for_duration(
        self,
        *,
        duration_sec: float,
        publish_hz: float = 5.0,
        stop_after: bool = True,
        stride_x: float = 0.0,
        stride_y: float = 0.0,
        stride_z: float = 0.0,
        rotation_speed: float = 0.0,
        body_x: float = 0.0,
        body_y: float = 0.0,
        body_z: float = 0.0,
        body_roll: float = 0.0,
        body_pitch: float = 0.0,
        body_yaw: float = 0.0,
        gait_type: str = 'tripod',
    ) -> MotionSequenceResult:
        """Publish the same walking command repeatedly for a fixed duration."""
        normalized_duration_sec = float(duration_sec)
        normalized_publish_hz = float(publish_hz)
        if normalized_duration_sec <= 0.0:
            raise ValueError('duration_sec must be positive.')
        if normalized_publish_hz <= 0.0:
            raise ValueError('publish_hz must be positive.')

        publish_count = max(1, math.ceil(normalized_duration_sec * normalized_publish_hz))
        interval_sec = 1.0 / normalized_publish_hz
        command_result: MotionCommandResult | None = None
        for publish_index in range(publish_count):
            command_result = self.send_motion_command(
                stride_x=stride_x,
                stride_y=stride_y,
                stride_z=stride_z,
                rotation_speed=rotation_speed,
                body_x=body_x,
                body_y=body_y,
                body_z=body_z,
                body_roll=body_roll,
                body_pitch=body_pitch,
                body_yaw=body_yaw,
                gait_type=gait_type,
            )
            if publish_index < publish_count - 1:
                time.sleep(interval_sec)

        if command_result is None:
            raise RuntimeError('walk_for_duration did not publish any motion commands.')

        stop_command_sent = False
        if stop_after:
            self.stop_motion()
            stop_command_sent = True

        return MotionSequenceResult(
            duration_sec=normalized_duration_sec,
            publish_hz=normalized_publish_hz,
            publish_count=publish_count,
            stop_command_sent=stop_command_sent,
            command=command_result,
            message='Published walking sequence.',
        )

    def start_recording(self, sample_interval_sec: float = 0.5) -> RecordingStatus:
        """Start recording robot snapshots."""
        if sample_interval_sec <= 0:
            raise ControllerError('sample_interval_sec must be positive.')

        with self._recording_lock:
            if self._recording is not None:
                raise ControllerError('Robot state recording is already active.')

            session = _RecordingSession(
                sample_interval_sec=sample_interval_sec,
                started_at=_utc_now(),
            )
            worker = threading.Thread(
                target=self._record_worker,
                args=(session,),
                name='drqp-robot-state-recorder',
                daemon=True,
            )
            session.worker = worker
            self._recording = session
            worker.start()

        return RecordingStatus(
            active=True,
            sample_interval_sec=sample_interval_sec,
            sample_count=0,
            started_at=session.started_at,
            message='Started recording robot state snapshots.',
        )

    def stop_recording(self) -> RecordedRobotStates:
        """Stop recording and return captured snapshots."""
        with self._recording_lock:
            session = self._recording
            if session is None:
                raise ControllerError('Robot state recording is not active.')
            self._recording = None

        session.stop_event.set()
        if session.worker is not None:
            session.worker.join(timeout=session.sample_interval_sec + 5.0)

        return RecordedRobotStates(
            started_at=session.started_at,
            stopped_at=_utc_now(),
            sample_interval_sec=session.sample_interval_sec,
            sample_count=len(session.samples),
            samples=session.samples,
        )

    def get_recording_status(self) -> RecordingStatus:
        """Return current recording status."""
        with self._recording_lock:
            session = self._recording
            if session is None:
                return RecordingStatus(
                    active=False,
                    sample_interval_sec=None,
                    sample_count=0,
                    started_at=None,
                    message='Robot state recording is idle.',
                )

            return RecordingStatus(
                active=True,
                sample_interval_sec=session.sample_interval_sec,
                sample_count=len(session.samples),
                started_at=session.started_at,
                message='Robot state recording is active.',
            )

    def stream_simulation_state(self) -> dict[str, Any]:
        """Return the current simulation state stream event payload."""
        return self._stream_event(
            'simulation.state.stream',
            self.get_simulation_state(),
        )

    def stream_simulation_robot_state(self) -> dict[str, Any]:
        """Return the current simulated robot state stream event payload."""
        return self._stream_event(
            'simulation.robot_state.stream',
            self.get_simulation_robot_state(),
        )

    def stream_simulation_world_state(self) -> dict[str, Any]:
        """Return the current world state stream event payload."""
        return self._stream_event(
            'simulation.world_state.stream',
            self.get_simulation_world_state(),
        )

    def stream_robot_state(self) -> dict[str, Any]:
        """Return the current robot state stream event payload."""
        return self._stream_event(
            'robot.state.stream',
            self.get_robot_namespace_state(),
        )

    def stream_system_state(self) -> dict[str, Any]:
        """Return the current system state stream event payload."""
        return self._stream_event(
            'system.state.stream',
            self.get_system_state(),
        )

    def _record_worker(self, session: _RecordingSession) -> None:
        """Record robot snapshots until stopped."""
        while not session.stop_event.is_set():
            session.samples.append(self.get_robot_state())
            if session.stop_event.wait(session.sample_interval_sec):
                break

    def _wait_for_state(
        self,
        target_state: str,
        timeout_sec: float,
    ) -> RobotStateSnapshot:
        """Wait for the robot to reach the requested lifecycle state."""
        result = self.runtime.wait_for_state(target_state, timeout_sec)
        if not bool(result.get('reached', False)):
            raise ControllerError(
                f"Timed out waiting for robot state '{target_state}'. "
                f'Latest observed state: {result.get("state")!r}.'
            )
        return self.get_robot_state()

    def _poll_state(
        self,
        predicate: Callable[[RobotStateSnapshot], bool],
        timeout_sec: float,
        error_message: str,
    ) -> RobotStateSnapshot:
        """Poll robot state until a predicate succeeds or timeout expires."""
        deadline = time.monotonic() + timeout_sec
        latest = self.get_robot_state()
        while time.monotonic() < deadline:
            if predicate(latest):
                return latest
            time.sleep(0.5)
            latest = self.get_robot_state()
        raise ControllerError(error_message)

    def _start_simulation(self) -> dict[str, Any]:
        """Start the background Gazebo launch process when available."""
        return runtime.start_simulation(
            self.launch_pid_path,
            self.launch_log_path,
            False,
        )

    def _publish_event(self, event: str) -> dict[str, Any]:
        """Publish a lifecycle event onto the local ROS graph."""
        return self.runtime.publish_event(event)

    def _wait_for_trajectory_action_server(self, timeout_sec: float) -> bool:
        """Wait for the joint trajectory controller action server."""
        return self.runtime.wait_for_trajectory_action_server(timeout_sec)

    def _publish_movement_command(
        self,
        stride_direction: dict[str, float],
        rotation_speed: float,
        body_translation: dict[str, float],
        body_rotation: dict[str, float],
        gait_type: str,
    ) -> dict[str, Any]:
        """Publish a motion command onto the local ROS graph."""
        return self.runtime.publish_movement_command(
            stride_direction=stride_direction,
            rotation_speed=rotation_speed,
            body_translation=body_translation,
            body_rotation=body_rotation,
            gait_type=gait_type,
        )

    def _validate_normalized(self, name: str, value: float) -> float:
        """Validate normalized joystick input in the inclusive range [-1, 1]."""
        normalized_value = float(value)
        if normalized_value < -1.0 or normalized_value > 1.0:
            raise ValueError(f'{name} must be between -1.0 and 1.0.')
        return normalized_value

    def _validate_gait_type(self, gait_type: str) -> str:
        """Validate gait names against the supported movement command contract."""
        normalized_gait_type = gait_type.strip().lower()
        if normalized_gait_type not in self._VALID_GAITS:
            supported = ', '.join(sorted(self._VALID_GAITS))
            raise ValueError(f'gait_type must be one of {supported}; got {gait_type!r}.')
        return normalized_gait_type

    def _poll_simulation_state(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        timeout_sec: float,
        error_message: str,
    ) -> dict[str, Any]:
        """Poll simulation state until a predicate succeeds or timeout expires."""
        deadline = time.monotonic() + timeout_sec
        latest = self.get_simulation_state(timeout_sec=0.1)
        while time.monotonic() < deadline:
            if predicate(latest):
                return latest
            time.sleep(0.5)
            latest = self.get_simulation_state(timeout_sec=0.1)
        raise ControllerError(error_message)

    def _wait_for_boot_ready(self, timeout_sec: float) -> dict[str, Any]:
        """Wait for simulation-side control surfaces before booting the lifecycle."""
        ready_state = self._poll_system_state(
            predicate=lambda snapshot: (
                bool(snapshot.get('robot_lifecycle_channel_available'))
                and bool(snapshot.get('joint_state_available'))
                and bool(snapshot.get('motion_command_channel_available'))
                and (
                    snapshot.get('deployment_mode') != 'simulation'
                    or bool(snapshot.get('simulation_channel_available'))
                )
            ),
            timeout_sec=timeout_sec,
            error_message='Timed out waiting for simulation control surfaces to become ready.',
        )
        if not self._wait_for_trajectory_action_server(timeout_sec=min(timeout_sec, 5.0)):
            raise ControllerError('Timed out waiting for the joint trajectory action server.')
        return ready_state

    def _poll_system_state(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        timeout_sec: float,
        error_message: str,
    ) -> dict[str, Any]:
        """Poll system state until a predicate succeeds or timeout expires."""
        deadline = time.monotonic() + timeout_sec
        latest = self.get_system_state(timeout_sec=0.1)
        while time.monotonic() < deadline:
            if predicate(latest):
                return latest
            time.sleep(0.5)
            latest = self.get_system_state(timeout_sec=0.1)
        raise ControllerError(error_message)

    def _simulation_process_running(self) -> bool:
        """Return whether the tracked simulation launch process is still alive."""
        current_pid = runtime._read_pid(self.launch_pid_path)
        return current_pid is not None and runtime._pid_is_running(current_pid)

    def _pose_to_payload(self, pose: Any) -> dict[str, Any] | None:
        """Convert a pose model into a plain JSON-serializable payload."""
        if pose is None:
            return None
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w,
            },
        }

    def _stream_event(
        self,
        stream_name: str,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        """Wrap a snapshot payload in the stream event envelope from the spec."""
        return {
            'timestamp': payload.get('timestamp', _utc_now()),
            'stream_name': stream_name,
            'event_type': 'snapshot',
            'payload': payload,
        }


def _utc_now() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(UTC).isoformat()
