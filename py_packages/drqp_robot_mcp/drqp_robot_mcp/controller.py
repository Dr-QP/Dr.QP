"""Host-side controller for the Dr.QP robot MCP server."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, UTC
from pathlib import Path
import threading
import time
from typing import Any, Callable

from . import runtime
from .models import (
    LifecycleActionResult,
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

    def __init__(
        self,
        workspace_root: str | Path | None = None,
        world_name: str = 'empty',
        robot_name: str = 'drqp',
    ) -> None:
        self.workspace_root = Path(workspace_root or Path.cwd()).resolve()
        self.world_name = world_name
        self.robot_name = robot_name
        self.launch_pid_path = self.workspace_root / '.tmp' / 'drqp_robot_mcp' / 'sim.launch.pid'
        self.launch_log_path = self.workspace_root / '.tmp' / 'drqp_robot_mcp' / 'sim.launch.log'
        self._recording_lock = threading.Lock()
        self._recording: _RecordingSession | None = None

    def boot_up(self, timeout_sec: float = 120.0) -> LifecycleActionResult:
        """Boot the robot to the `torque_on` state."""
        simulation_was_started = False
        state_before_snapshot = self.get_robot_state()

        if state_before_snapshot.lifecycle_state is None:
            simulation = self._start_simulation()
            simulation_was_started = bool(simulation.get('started', False))
            try:
                state_before_snapshot = self._poll_state(
                    predicate=lambda snapshot: snapshot.lifecycle_state is not None,
                    timeout_sec=min(timeout_sec, 60.0),
                    error_message='Timed out waiting for the robot state topic after starting simulation.',
                )
            except ControllerError as exc:
                if not bool(simulation.get('available', True)):
                    raise ControllerError(str(simulation.get('message'))) from exc
                raise

        state_before = state_before_snapshot.lifecycle_state
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
            runtime.get_robot_state(self.world_name, self.robot_name, timeout_sec)
        )

    def get_world_state(self, timeout_sec: float = 10.0) -> WorldStateSnapshot:
        """Return the latest Gazebo world snapshot."""
        return WorldStateSnapshot.from_mapping(
            runtime.get_world_state(self.world_name, timeout_sec)
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
        result = runtime.wait_for_state(target_state, timeout_sec)
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
            self.workspace_root,
            self.launch_pid_path,
            self.launch_log_path,
            False,
        )

    def _publish_event(self, event: str) -> dict[str, Any]:
        """Publish a lifecycle event onto the local ROS graph."""
        return runtime.publish_event(event)


def _utc_now() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(UTC).isoformat()
