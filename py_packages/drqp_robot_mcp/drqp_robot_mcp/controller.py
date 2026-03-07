"""Host-side controller for the Dr.QP robot MCP server."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, UTC
import json
from pathlib import Path
import shlex
import subprocess
import threading
import time
from typing import Any, Callable, Sequence

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
class CommandResult:
    """Captured subprocess result."""

    stdout: str
    stderr: str
    returncode: int


RunCommand = Callable[[Sequence[str], float | None], CommandResult]


@dataclass(slots=True)
class _RecordingSession:
    """State for an active recording session."""

    sample_interval_sec: float
    started_at: str
    samples: list[RobotStateSnapshot] = field(default_factory=list)
    stop_event: threading.Event = field(default_factory=threading.Event)
    worker: threading.Thread | None = None


class RobotMcpController:
    """Coordinate Gazebo, ROS 2, and robot lifecycle operations."""

    def __init__(
        self,
        workspace_root: str | Path | None = None,
        run_command: RunCommand | None = None,
        world_name: str = 'empty',
        robot_name: str = 'drqp',
    ) -> None:
        self.workspace_root = Path(workspace_root or Path.cwd()).resolve()
        self.world_name = world_name
        self.robot_name = robot_name
        self.compose_file = self.workspace_root / '.devcontainer' / 'docker-compose.yml'
        self.compose_env_file = self.workspace_root / '.devcontainer' / '.env'
        self.pythonpath_root = self.workspace_root / 'py_packages' / 'drqp_robot_mcp'
        self.launch_pid_path = self.workspace_root / '.tmp' / 'drqp_robot_mcp' / 'sim.launch.pid'
        self.launch_log_path = self.workspace_root / '.tmp' / 'drqp_robot_mcp' / 'sim.launch.log'
        self._run_command_impl = run_command or self._default_run_command
        self._recording_lock = threading.Lock()
        self._recording: _RecordingSession | None = None

    def boot_up(self, timeout_sec: float = 120.0) -> LifecycleActionResult:
        """Boot the robot to the `torque_on` state."""
        simulation_was_started = False
        state_before_snapshot = self.get_robot_state()

        if not state_before_snapshot.simulation_running:
            simulation = self._container_start_simulation()
            simulation_was_started = bool(simulation.get('started', False))
            state_before_snapshot = self._poll_state(
                predicate=lambda snapshot: snapshot.lifecycle_state is not None,
                timeout_sec=min(timeout_sec, 60.0),
                error_message='Timed out waiting for the robot state topic after starting simulation.',
            )

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
            self._container_publish_event('initialize')
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'initializing':
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'finalizing':
            self._wait_for_state('finalized', timeout_sec)
            self._container_publish_event('initialize')
            state_after_snapshot = self._wait_for_state('torque_on', timeout_sec)
        elif state_before == 'servos_rebooting':
            self._wait_for_state('torque_off', timeout_sec)
            self._container_publish_event('initialize')
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

        if not state_before_snapshot.simulation_running or state_before is None:
            return LifecycleActionResult(
                action='shut_down',
                state_before=state_before,
                state_after=state_before,
                simulation_was_started=False,
                simulation_running=False,
                message='Simulation is not running.',
                log_path=str(self.launch_log_path),
            )

        if state_before == 'torque_on':
            self._container_publish_event('finalize')
            state_after_snapshot = self._wait_for_state('finalized', timeout_sec)
        elif state_before == 'initializing':
            self._container_publish_event('turn_off')
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
        return RobotStateSnapshot.from_mapping(self._container_get_robot_state(timeout_sec))

    def get_world_state(self, timeout_sec: float = 10.0) -> WorldStateSnapshot:
        """Return the latest Gazebo world snapshot."""
        return WorldStateSnapshot.from_mapping(self._container_get_world_state(timeout_sec))

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
        result = self._container_wait_state(target_state, timeout_sec)
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

    def _container_start_simulation(self) -> dict[str, Any]:
        """Start the background Gazebo launch process."""
        return self._run_container_json(
            'start-simulation',
            '--workspace-root',
            str(self.workspace_root),
            '--pid-path',
            str(self.launch_pid_path),
            '--log-path',
            str(self.launch_log_path),
            timeout=60.0,
        )

    def _container_publish_event(self, event: str) -> dict[str, Any]:
        """Publish a robot lifecycle event."""
        return self._run_container_json(
            'publish-event',
            '--event',
            event,
            timeout=20.0,
        )

    def _container_wait_state(
        self,
        target_state: str,
        timeout_sec: float,
    ) -> dict[str, Any]:
        """Wait for a target lifecycle state inside the devcontainer."""
        return self._run_container_json(
            'wait-state',
            '--target-state',
            target_state,
            '--timeout-sec',
            str(timeout_sec),
            timeout=timeout_sec + 10.0,
        )

    def _container_get_robot_state(self, timeout_sec: float) -> dict[str, Any]:
        """Fetch the latest robot state from the devcontainer."""
        return self._run_container_json(
            'get-robot-state',
            '--world-name',
            self.world_name,
            '--robot-name',
            self.robot_name,
            '--timeout-sec',
            str(timeout_sec),
            timeout=timeout_sec + 10.0,
        )

    def _container_get_world_state(self, timeout_sec: float) -> dict[str, Any]:
        """Fetch the latest world state from the devcontainer."""
        return self._run_container_json(
            'get-world-state',
            '--world-name',
            self.world_name,
            '--timeout-sec',
            str(timeout_sec),
            timeout=timeout_sec + 10.0,
        )

    def _run_container_json(self, *args: str, timeout: float) -> dict[str, Any]:
        """Run the devcontainer bridge and decode its JSON output."""
        self._ensure_devcontainer_running()

        python_command = shlex.join(['python3', '-m', 'drqp_robot_mcp.container_cli', *args])
        shell_command = (
            f'cd {shlex.quote(str(self.workspace_root))} && '
            f'export PYTHONPATH={shlex.quote(str(self.pythonpath_root))}'
            '${PYTHONPATH:+:$PYTHONPATH} && '
            'export ROS_DISTRO=jazzy CC=clang CXX=clang++ '
            'CMAKE_EXPORT_COMPILE_COMMANDS=1 && '
            'source scripts/setup.bash && '
            f'{python_command}'
        )
        command = [
            *self._docker_compose_base_command(),
            'exec',
            '-T',
            'devcontainer',
            'bash',
            '-lc',
            shell_command,
        ]
        result = self._run_command(command, timeout=timeout)
        try:
            return json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise ControllerError(
                'Container bridge returned invalid JSON output.\n'
                f'stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}'
            ) from exc

    def _ensure_devcontainer_running(self) -> None:
        """Create the compose env file and ensure the devcontainer is up."""
        self.launch_pid_path.parent.mkdir(parents=True, exist_ok=True)
        self._run_command(
            ['bash', str(self.workspace_root / '.devcontainer' / 'devcontainer-init.sh')],
            timeout=10.0,
        )
        self._run_command(
            [*self._docker_compose_base_command(), 'up', '-d', 'devcontainer'],
            timeout=120.0,
        )

    def _docker_compose_base_command(self) -> list[str]:
        """Build the base docker compose command for the devcontainer."""
        return [
            'docker',
            'compose',
            '-f',
            str(self.compose_file),
            '--env-file',
            str(self.compose_env_file),
        ]

    def _run_command(self, command: Sequence[str], timeout: float | None) -> CommandResult:
        """Run a command and raise a helpful error on failure."""
        result = self._run_command_impl(command, timeout)
        if result.returncode != 0:
            joined = shlex.join(command)
            raise ControllerError(
                f'Command failed: {joined}\nstdout:\n{result.stdout}\n\nstderr:\n{result.stderr}'
            )
        return result

    @staticmethod
    def _default_run_command(
        command: Sequence[str],
        timeout: float | None,
    ) -> CommandResult:
        """Run a subprocess and capture text output."""
        completed = subprocess.run(  # noqa: S603
            list(command),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return CommandResult(
            stdout=completed.stdout,
            stderr=completed.stderr,
            returncode=completed.returncode,
        )


def _utc_now() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(UTC).isoformat()
