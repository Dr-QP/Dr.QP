"""Unit tests for the Dr.QP robot MCP controller."""

from __future__ import annotations

import math
import time

import pytest

import drqp_robot_mcp.controller as controller_module
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
        self.movement_commands: list[dict[str, object]] = []

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

    def _publish_movement_command(
        self,
        stride_direction: dict[str, float],
        rotation_speed: float,
        body_translation: dict[str, float],
        body_rotation: dict[str, float],
        gait_type: str,
    ) -> dict[str, object]:
        command = {
            'stride_direction': stride_direction,
            'rotation_speed': rotation_speed,
            'body_translation': body_translation,
            'body_rotation': body_rotation,
            'gait_type': gait_type,
        }
        self.movement_commands.append(command)
        return {'published': True, **command}

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


def test_send_motion_command_publishes_normalized_command() -> None:
    """Motion commands are delegated to the runtime publishing seam."""
    controller = FakeController([make_snapshot('torque_on')])

    result = controller.send_motion_command(
        stride_x=1.0,
        stride_y=-0.25,
        rotation_speed=0.5,
        body_z=0.2,
        body_pitch=-0.1,
        gait_type='ripple',
    )

    assert controller.movement_commands == [
        {
            'stride_direction': {'x': 1.0, 'y': -0.25, 'z': 0.0},
            'rotation_speed': 0.5,
            'body_translation': {'x': 0.0, 'y': 0.0, 'z': 0.2},
            'body_rotation': {'x': 0.0, 'y': -0.1, 'z': 0.0},
            'gait_type': 'ripple',
        }
    ]
    assert result.gait_type == 'ripple'
    assert result.rotation_speed == 0.5
    assert result.stride_direction.x == 1.0
    assert result.body_translation.z == 0.2
    assert result.body_rotation.y == -0.1


def test_send_motion_command_rejects_out_of_range_input() -> None:
    """Motion inputs must stay within the normalized message contract."""
    controller = FakeController([make_snapshot('torque_on')])

    try:
        controller.send_motion_command(stride_x=1.5)
    except ValueError as exc:
        assert 'stride_x' in str(exc)
    else:
        raise AssertionError('Expected ValueError for out-of-range stride_x')


def test_stop_motion_publishes_zeroed_tripod_command() -> None:
    """Stop motion resets all joystick axes to zero."""
    controller = FakeController([make_snapshot('torque_on')])

    result = controller.stop_motion()

    assert controller.movement_commands == [
        {
            'stride_direction': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rotation_speed': 0.0,
            'body_translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'body_rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gait_type': 'tripod',
        }
    ]
    assert result.gait_type == 'tripod'


def test_walk_for_duration_republishes_motion_command_and_stops(monkeypatch) -> None:
    """Walk sequences publish the same command repeatedly before stopping."""
    controller = FakeController([make_snapshot('torque_on')])
    sleep_calls: list[float] = []
    monkeypatch.setattr(controller_module.time, 'sleep', sleep_calls.append)

    result = controller.walk_for_duration(
        stride_x=1.0,
        duration_sec=0.5,
        publish_hz=4.0,
    )

    assert result.publish_count == math.ceil(0.5 * 4.0)
    assert result.stop_command_sent is True
    assert len(controller.movement_commands) == result.publish_count + 1
    assert controller.movement_commands[0]['stride_direction']['x'] == 1.0
    assert controller.movement_commands[-1]['stride_direction']['x'] == 0.0
    assert sleep_calls == [0.25]


def test_walk_for_duration_rejects_non_positive_duration() -> None:
    """Walk sequencing requires a positive duration."""
    controller = FakeController([make_snapshot('torque_on')])

    with pytest.raises(ValueError, match='duration_sec'):
        controller.walk_for_duration(duration_sec=0.0)
