"""Unit tests for the Dr.QP robot MCP controller."""

from __future__ import annotations

import time

from drqp_robot_mcp.controller import RobotMcpController
from drqp_robot_mcp.models import Pose, Quaternion, RobotStateSnapshot, Vector3


def make_snapshot(
    lifecycle_state: str | None,
    *,
    available: bool = True,
    simulation_running: bool = True,
) -> RobotStateSnapshot:
    """Create a minimal robot snapshot for controller tests."""
    return RobotStateSnapshot(
        timestamp='2026-03-07T00:00:00+00:00',
        available=available,
        simulation_running=simulation_running,
        lifecycle_state=lifecycle_state,
        world_name='empty' if simulation_running else None,
        simulation_time_sec=1.0 if simulation_running else None,
        robot_pose=Pose(
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        if simulation_running
        else None,
        joint_states={},
        note=None,
    )


class FakeController(RobotMcpController):
    """Controller test double that avoids ROS middleware dependencies."""

    def __init__(self, states: list[RobotStateSnapshot]):
        super().__init__(workspace_root='/workspace')
        self.states = list(states)
        self.published_events: list[str] = []
        self.waited_states: list[str] = []
        self.start_calls = 0
        self.current_state = self.states[-1]

    def get_robot_state(self, timeout_sec: float = 10.0) -> RobotStateSnapshot:
        del timeout_sec
        if len(self.states) > 1:
            snapshot = self.states.pop(0)
            self.current_state = snapshot
            return snapshot
        return self.current_state

    def _start_simulation(self) -> dict[str, object]:
        self.start_calls += 1
        self.current_state = make_snapshot('torque_off')
        self.states = [self.current_state]
        return {'started': True, 'available': True}

    def _publish_event(self, event: str) -> dict[str, object]:
        self.published_events.append(event)
        return {'published': True, 'event': event}

    def _wait_for_state(
        self,
        target_state: str,
        timeout_sec: float,
    ) -> RobotStateSnapshot:
        del timeout_sec
        self.waited_states.append(target_state)
        self.current_state = make_snapshot(target_state)
        self.states = [self.current_state]
        return self.current_state


def test_boot_up_starts_simulation_and_initializes_robot() -> None:
    """Boot-up starts the sim and initializes the lifecycle state machine."""
    controller = FakeController([make_snapshot(None, available=False, simulation_running=False)])

    result = controller.boot_up(timeout_sec=5.0)

    assert controller.start_calls == 1
    assert controller.published_events == ['initialize']
    assert controller.waited_states == ['torque_on']
    assert result.state_before == 'torque_off'
    assert result.state_after == 'torque_on'
    assert result.simulation_was_started is True


def test_boot_up_initializes_without_starting_simulation_when_state_exists() -> None:
    """Boot-up uses the existing ROS lifecycle state even without Gazebo."""
    controller = FakeController([make_snapshot('torque_off', simulation_running=False)])

    result = controller.boot_up(timeout_sec=5.0)

    assert controller.start_calls == 0
    assert controller.published_events == ['initialize']
    assert controller.waited_states == ['torque_on']
    assert result.state_before == 'torque_off'
    assert result.state_after == 'torque_on'
    assert result.simulation_was_started is False


def test_shut_down_finalizes_robot_from_torque_on() -> None:
    """Shutdown sends finalize when the robot is currently on."""
    controller = FakeController([make_snapshot('torque_on')])

    result = controller.shut_down(timeout_sec=5.0)

    assert controller.published_events == ['finalize']
    assert controller.waited_states == ['finalized']
    assert result.state_before == 'torque_on'
    assert result.state_after == 'finalized'


def test_shut_down_uses_lifecycle_state_without_simulation_running() -> None:
    """Shutdown uses lifecycle state when ROS is available without Gazebo."""
    controller = FakeController([make_snapshot('torque_on', simulation_running=False)])

    result = controller.shut_down(timeout_sec=5.0)

    assert controller.published_events == ['finalize']
    assert controller.waited_states == ['finalized']
    assert result.state_before == 'torque_on'
    assert result.state_after == 'finalized'


def test_recording_collects_multiple_samples() -> None:
    """Recording returns sampled robot state snapshots."""
    controller = FakeController([make_snapshot('torque_on')])

    status = controller.start_recording(sample_interval_sec=0.01)
    deadline = time.time() + 1.0
    while getattr(status, 'sample_count', 0) < 2 and time.time() < deadline:
        time.sleep(0.005)
    recorded = controller.stop_recording()

    assert status.active is True
    assert recorded.sample_count >= 2
    assert all(sample.lifecycle_state == 'torque_on' for sample in recorded.samples)
