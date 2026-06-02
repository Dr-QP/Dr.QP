"""Unit tests for the MCP surface exposed by the Dr.QP robot server."""

from __future__ import annotations

import importlib
import sys
from types import SimpleNamespace
from types import ModuleType

import pytest

from drqp_robot_mcp.models import MotionCommandResult, MotionSequenceResult, Vector3


EXPECTED_TOOL_NAMES = {
    'simulation.start',
    'simulation.stop',
    'simulation.state',
    'simulation.robot_state',
    'simulation.world_state',
    'robot.boot',
    'robot.shutdown',
    'robot.state',
    'robot.move',
    'robot.stop',
    'robot.walk_for_duration',
    'robot.recording.start',
    'robot.recording.stop',
    'robot.recording.status',
    'system.state',
}

EXPECTED_RESOURCE_URIS = {
    'drqp://simulation.state.stream',
    'drqp://simulation.robot_state.stream',
    'drqp://simulation.world_state.stream',
    'drqp://robot.state.stream',
    'drqp://system.state.stream',
}


class _FakeFastMCP:
    """Small FastMCP test double that records registered tools."""

    def __init__(self, name: str, instructions: str) -> None:
        self.name = name
        self.instructions = instructions
        self.tools: dict[str, object] = {}
        self.resources: dict[str, object] = {}

    def tool(self, name: str | None = None, **_: object):
        def decorator(function):
            self.tools[name or function.__name__] = function
            return function

        return decorator

    def resource(self, uri: str, **_: object):
        def decorator(function):
            self.resources[uri] = function
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


def test_server_registers_exact_spec_tool_names(server) -> None:
    """The server exposes the request-response tool surface from the spec."""
    assert set(server.mcp.tools) == EXPECTED_TOOL_NAMES
    assert not any(name.startswith('drqp_robot_') for name in server.mcp.tools)


def test_server_registers_exact_spec_stream_resources(server) -> None:
    """The server exposes stream surfaces as MCP resources."""
    assert set(server.mcp.resources) == EXPECTED_RESOURCE_URIS


def test_robot_move_tool_delegates_to_controller_send_motion_command(
    monkeypatch,
    server,
) -> None:
    """The robot.move tool delegates motion commands to the controller."""
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

    result = server.mcp.tools['robot.move'](stride_x=1.0, gait_type='tripod')

    assert captured['stride_x'] == 1.0
    assert captured['gait_type'] == 'tripod'
    assert result['gait_type'] == 'tripod'
    assert result['stride_direction']['x'] == 1.0


def test_robot_stop_tool_delegates_to_controller_stop_motion(
    monkeypatch,
    server,
) -> None:
    """The robot.stop tool delegates to the dedicated stop operation."""

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

    result = server.mcp.tools['robot.stop']()

    assert result['note'] == 'Published stop command.'


def test_robot_walk_for_duration_tool_delegates_to_controller(
    monkeypatch,
    server,
) -> None:
    """The robot.walk_for_duration tool delegates repeated motion commands."""
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

    result = server.mcp.tools['robot.walk_for_duration'](
        stride_x=1.0,
        duration_sec=1.0,
    )

    assert captured['stride_x'] == 1.0
    assert captured['duration_sec'] == 1.0
    assert result['publish_count'] == 5


def test_simulation_start_tool_delegates_to_controller(monkeypatch, server) -> None:
    """The simulation.start tool delegates simulation startup."""
    captured: dict[str, object] = {}
    expected = object()

    def fake_start_simulation(timeout_sec: float = 120.0) -> object:
        captured['timeout_sec'] = timeout_sec
        return expected

    monkeypatch.setattr(
        server,
        'controller',
        SimpleNamespace(start_simulation=fake_start_simulation),
    )

    result = server.mcp.tools['simulation.start'](timeout_sec=30.0)

    assert captured == {'timeout_sec': 30.0}
    assert result is expected


def test_robot_state_stream_resource_delegates_to_controller(monkeypatch, server) -> None:
    """The robot state stream resource delegates to the controller."""
    expected = {'stream': 'robot.state.stream'}

    def fake_stream_robot_state() -> object:
        return expected

    monkeypatch.setattr(
        server,
        'controller',
        SimpleNamespace(stream_robot_state=fake_stream_robot_state),
    )

    result = server.mcp.resources['drqp://robot.state.stream']()

    assert result is expected
