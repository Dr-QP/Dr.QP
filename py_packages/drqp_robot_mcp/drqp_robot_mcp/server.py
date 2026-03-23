"""FastMCP server exposing Dr.QP robot tools."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP

from .controller import RobotMcpController
from .models import (
    LifecycleActionResult,
    RecordedRobotStates,
    RecordingStatus,
    RobotStateSnapshot,
    WorldStateSnapshot,
)

mcp = FastMCP(
    'Dr.QP Robot MCP',
    instructions=(
        'Use these tools to boot the Dr.QP Gazebo robot, query its lifecycle '
        'and pose state, record robot state snapshots, and inspect Gazebo world poses.'
    ),
)
controller = RobotMcpController()


@mcp.tool()
def drqp_robot_boot_up(timeout_sec: float = 120.0) -> LifecycleActionResult:
    """Boot the Dr.QP Gazebo robot to the `torque_on` lifecycle state."""
    return controller.boot_up(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_shut_down(timeout_sec: float = 120.0) -> LifecycleActionResult:
    """Shut the Dr.QP robot down through its lifecycle state machine."""
    return controller.shut_down(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_get_state(timeout_sec: float = 10.0) -> RobotStateSnapshot:
    """Return the latest robot lifecycle state, joint states, and robot pose."""
    return controller.get_robot_state(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_start_state_recording(
    sample_interval_sec: float = 0.5,
) -> RecordingStatus:
    """Start recording robot state snapshots at a fixed interval."""
    return controller.start_recording(sample_interval_sec=sample_interval_sec)


@mcp.tool()
def drqp_robot_stop_state_recording() -> RecordedRobotStates:
    """Stop robot state recording and return the recorded snapshots."""
    return controller.stop_recording()


@mcp.tool()
def drqp_robot_get_recording_status() -> RecordingStatus:
    """Return whether robot state recording is currently active."""
    return controller.get_recording_status()


@mcp.tool()
def drqp_robot_get_world_state(timeout_sec: float = 10.0) -> WorldStateSnapshot:
    """Return the latest Gazebo world entity poses and simulation time."""
    return controller.get_world_state(timeout_sec=timeout_sec)


def main() -> None:
    """Run the MCP server over stdio."""
    mcp.run()


if __name__ == '__main__':
    main()
