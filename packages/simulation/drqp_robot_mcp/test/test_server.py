"""Unit tests for MCP tool exposure in the Dr.QP robot server."""

from __future__ import annotations

import importlib
import sys
from types import ModuleType

import pytest

from drqp_robot_mcp.models import MotionCommandResult, MotionSequenceResult, Vector3


class _FakeFastMCP:
    """Small FastMCP test double that records registered tools."""

    def __init__(self, name: str, instructions: str) -> None:
        self.name = name
        self.instructions = instructions
        self.tools: dict[str, object] = {}

    def tool(self):
        def decorator(function):
            self.tools[function.__name__] = function
            return function

        return decorator

    def run(self) -> None:
        return None


@pytest.fixture
def server(monkeypatch):
    """Import the server module with a stub FastMCP implementation."""
    fastmcp_module = ModuleType('mcp.server.fastmcp')
    fastmcp_module.FastMCP = _FakeFastMCP

    monkeypatch.setitem(sys.modules, 'mcp', ModuleType('mcp'))
    monkeypatch.setitem(sys.modules, 'mcp.server', ModuleType('mcp.server'))
    monkeypatch.setitem(sys.modules, 'mcp.server.fastmcp', fastmcp_module)
    sys.modules.pop('drqp_robot_mcp.server', None)

    imported = importlib.import_module('drqp_robot_mcp.server')
    yield imported
    sys.modules.pop('drqp_robot_mcp.server', None)


def test_drqp_robot_send_motion_command_delegates_to_controller(
    monkeypatch,
    server,
) -> None:
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


def test_drqp_robot_stop_motion_delegates_to_controller(monkeypatch, server) -> None:
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


def test_drqp_robot_walk_for_duration_delegates_to_controller(
    monkeypatch,
    server,
) -> None:
    """The MCP tool exposes repeated walking commands through the controller."""
    captured: dict[str, object] = {}

    def fake_walk_for_duration(**kwargs) -> MotionSequenceResult:
        captured.update(kwargs)
        return MotionSequenceResult(
            duration_sec=1.0,
            publish_hz=5.0,
            publish_count=5,
            stop_command_sent=True,
            command=MotionCommandResult(
                stride_direction=Vector3(1.0, 0.0, 0.0),
                rotation_speed=0.0,
                body_translation=Vector3(0.0, 0.0, 0.0),
                body_rotation=Vector3(0.0, 0.0, 0.0),
                gait_type='tripod',
                message='Published motion command.',
            ),
            message='Published walking sequence.',
        )

    monkeypatch.setattr(server.controller, 'walk_for_duration', fake_walk_for_duration)

    result = server.drqp_robot_walk_for_duration(stride_x=1.0, duration_sec=1.0)

    assert captured['stride_x'] == 1.0
    assert captured['duration_sec'] == 1.0
    assert result.publish_count == 5
