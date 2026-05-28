"""Unit tests for MCP tool exposure in the Dr.QP robot server."""

from __future__ import annotations

from drqp_robot_mcp.models import MotionCommandResult
from drqp_robot_mcp import server
from drqp_robot_mcp.models import Vector3


def test_drqp_robot_send_motion_command_delegates_to_controller(monkeypatch) -> None:
    """The MCP tool delegates motion commands to the shared controller."""
    captured: dict[str, object] = {}

    def fake_send_motion_command(**kwargs) -> MotionCommandResult:
        captured.update(kwargs)
        return MotionCommandResult(
            stride_direction=Vector3(1.0, 0.0, 0.0),
            rotation_speed=0.0,
            body_translation=Vector3(0.0, 0.0, 0.0),
            body_rotation=Vector3(0.0, 0.0, 0.0),
            gait_type='tripod',
            message='Published motion command.',
        )

    monkeypatch.setattr(server.controller, 'send_motion_command', fake_send_motion_command)

    result = server.drqp_robot_send_motion_command(stride_x=1.0, gait_type='tripod')

    assert captured['stride_x'] == 1.0
    assert captured['gait_type'] == 'tripod'
    assert result.gait_type == 'tripod'


def test_drqp_robot_stop_motion_delegates_to_controller(monkeypatch) -> None:
    """The MCP tool exposes a dedicated stop motion operation."""

    def fake_stop_motion() -> MotionCommandResult:
        return MotionCommandResult(
            stride_direction=Vector3(0.0, 0.0, 0.0),
            rotation_speed=0.0,
            body_translation=Vector3(0.0, 0.0, 0.0),
            body_rotation=Vector3(0.0, 0.0, 0.0),
            gait_type='tripod',
            message='Published stop command.',
        )

    monkeypatch.setattr(server.controller, 'stop_motion', fake_stop_motion)

    result = server.drqp_robot_stop_motion()

    assert result.message == 'Published stop command.'